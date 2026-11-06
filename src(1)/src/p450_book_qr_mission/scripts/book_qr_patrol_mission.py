#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import os
import re
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rospy

try:
    from pyzbar.pyzbar import decode as zbar_decode
    _PYZBAR_OK = True
except Exception:
    zbar_decode = None
    _PYZBAR_OK = False
from cv_bridge import CvBridge
from openpyxl import Workbook
from PIL import Image, ImageDraw, ImageFont
from sensor_msgs.msg import Image as RosImage
from std_srvs.srv import SetBool

from prometheus_msgs.msg import UAVCommand, UAVControlState, UAVState
from spirecv_msgs.msg import GimbalControl, GimbalState


def normalize_text(text: str) -> str:
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    lines = [line.strip() for line in text.split("\n")]
    lines = [line for line in lines if line]
    return "\n".join(lines).strip()


def parse_payload(raw_text: str) -> dict:
    result = {
        "编号": "",
        "分类号": "",
        "分类名称": "",
        "数量": "",
        "库位": "",
        "状态": "",
    }

    for line in raw_text.split("\n"):
        parts = re.split(r"[：:]", line, maxsplit=1)
        if len(parts) != 2:
            continue

        key = parts[0].strip()
        value = parts[1].strip()

        if key == "编号":
            result["编号"] = value
        elif key == "分类":
            if "-" in value:
                class_code, class_name = value.split("-", 1)
                result["分类号"] = class_code.strip()
                result["分类名称"] = class_name.strip()
            else:
                result["分类名称"] = value
        elif key == "数量":
            m = re.search(r"(\d+)", value)
            result["数量"] = m.group(1) if m else value
        elif key == "库位":
            result["库位"] = value
        elif key == "状态":
            result["状态"] = value

    return result


def find_chinese_font() -> str:
    candidates = [
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        "/usr/share/fonts/truetype/arphic/ukai.ttc",
        "/usr/share/fonts/truetype/arphic/uming.ttc",
    ]
    for path in candidates:
        if Path(path).exists():
            return path
    return ""


def load_font(size: int = 24):
    font_path = find_chinese_font()
    if font_path:
        return ImageFont.truetype(font_path, size=size), font_path
    return ImageFont.load_default(), ""


def draw_multiline_text(frame: np.ndarray, lines: List[str], font) -> np.ndarray:
    if frame is None:
        return frame
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image = Image.fromarray(rgb)
    draw = ImageDraw.Draw(image)

    text = "\n".join(lines)
    x, y = 12, 12
    padding = 10
    spacing = 6

    if hasattr(draw, "multiline_textbbox"):
        bbox = draw.multiline_textbbox((x + padding, y + padding), text, font=font, spacing=spacing)
        rect = (x, y, bbox[2] + padding, bbox[3] + padding)
    else:
        rect = (x, y, x + 700, y + 220)

    draw.rectangle(rect, fill=(0, 0, 0))
    draw.multiline_text(
        (x + padding, y + padding),
        text,
        font=font,
        fill=(255, 255, 255),
        spacing=spacing,
    )
    return cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)


@dataclass
class Waypoint:
    name: str
    stage: str
    scan: bool
    x: float
    y: float
    z: float
    yaw_deg: float
    hold_sec: float = 1.5
    scan_timeout_sec: float = 6.0
    note: str = ""


class BookQrPatrolMission:
    def __init__(self):
        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self.use_pyzbar = bool(rospy.get_param("~mission/use_pyzbar", True))
        self.use_opencv_fallback = bool(rospy.get_param("~mission/use_opencv_fallback", False))
        self.font, self.font_path = load_font(size=20)

        self.uav_id = int(rospy.get_param("~uav_id", 1))
        self.command_topic = f"/uav{self.uav_id}/prometheus/command"
        self.state_topic = f"/uav{self.uav_id}/prometheus/state"
        self.control_state_topic = f"/uav{self.uav_id}/prometheus/control_state"
        self.image_topic = rospy.get_param("~image_topic", f"/uav{self.uav_id}/gimbal/image_original")
        self.gimbal_control_topic = f"/uav{self.uav_id}/gimbal/control"
        self.gimbal_state_topic = f"/uav{self.uav_id}/gimbal/state"
        self.gimbal_home_service = f"/uav{self.uav_id}/gimbal_server"

        self.takeoff_height = float(rospy.get_param("~mission/takeoff_height", 1.6))
        onboard_takeoff_param = f"/uav_control_main_{self.uav_id}/control/Takeoff_height"
        if rospy.has_param(onboard_takeoff_param):
            self.takeoff_height = float(rospy.get_param(onboard_takeoff_param))
        self.takeoff_wait_sec = float(rospy.get_param("~mission/takeoff_wait_sec", 6.0))
        self.xy_tol = float(rospy.get_param("~mission/waypoint_reach_xy_tol", 0.18))
        self.z_tol = float(rospy.get_param("~mission/waypoint_reach_z_tol", 0.12))
        self.yaw_tol_deg = float(rospy.get_param("~mission/waypoint_reach_yaw_tol_deg", 8.0))
        self.command_rate_hz = float(rospy.get_param("~mission/command_rate_hz", 10.0))
        self.decode_interval_sec = float(rospy.get_param("~mission/decode_interval_sec", 0.2))
        self.success_repeat_threshold = int(rospy.get_param("~mission/success_repeat_threshold", 2))
        self.abort_on_scan_fail = bool(rospy.get_param("~mission/abort_on_scan_fail", False))
        self.land_after_finish = bool(rospy.get_param("~mission/land_after_finish", True))
        self.gimbal_settle_sec = float(rospy.get_param("~mission/gimbal_settle_sec", 1.0))
        self.show_debug_window = bool(rospy.get_param("~mission/show_debug_window", False))
        self.show_snapshot_preview = bool(rospy.get_param("~mission/show_snapshot_preview", True))
        self.snapshot_preview_wait_ms = int(rospy.get_param("~mission/snapshot_preview_wait_ms", 1200))
        self.snapshot_preview_window_name = str(rospy.get_param("~mission/snapshot_preview_window_name", "book_qr_patrol_snapshot_preview"))
        self.finish_hold_sec = float(rospy.get_param("~mission/finish_hold_sec", 2.0))
        self.land_timeout_sec = float(rospy.get_param("~mission/land_timeout_sec", 20.0))
        self.land_min_z = float(rospy.get_param("~mission/land_min_z", 0.15))
        self.center_distance_weight = float(rospy.get_param("~mission/center_distance_weight", 0.002))
        self.save_raw_debug_frames = bool(rospy.get_param("~mission/save_raw_debug_frames", True))
        self.save_debug_every_n_frames = int(rospy.get_param("~mission/save_debug_every_n_frames", 45))
        self.log_decode_candidates = bool(rospy.get_param("~mission/log_decode_candidates", False))
        rospy.loginfo("mission decoder: pyzbar_ok=%s use_pyzbar=%s use_opencv_fallback=%s", _PYZBAR_OK, self.use_pyzbar, self.use_opencv_fallback)
        self.heartbeat_sec = float(rospy.get_param("~mission/heartbeat_sec", 2.0))

        self.gimbal_fixed_roll_deg = float(rospy.get_param("~mission/gimbal_fixed_roll_deg", 0.0))
        self.gimbal_fixed_pitch_deg = float(rospy.get_param("~mission/gimbal_fixed_pitch_deg", 0.0))
        self.gimbal_fixed_yaw_deg = float(rospy.get_param("~mission/gimbal_fixed_yaw_deg", 0.0))

        save_root = os.path.expanduser(rospy.get_param("~mission/save_root", "~/p450_results/book_qr_patrol"))
        self.output_dir = Path(save_root) / datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.snap_dir = self.output_dir / "snapshots"
        self.snap_dir.mkdir(parents=True, exist_ok=True)
        self.debug_dir = self.output_dir / "debug_frames"
        self.debug_dir.mkdir(parents=True, exist_ok=True)

        self.expected_txt_dir = os.path.expanduser(rospy.get_param("~mission/expected_txt_dir", ""))
        self.expected_payloads = self._load_expected_payloads(self.expected_txt_dir)

        self.uav_state: Optional[UAVState] = None
        self.control_state: Optional[UAVControlState] = None
        self.gimbal_state: Optional[GimbalState] = None
        self.latest_image: Optional[np.ndarray] = None
        self.latest_image_stamp = None
        self.last_debug_frame = None
        self.frame_count = 0
        self.last_heartbeat_log = 0.0

        self.command_id = 0
        self.records: List[Dict] = []
        self.seen_texts: Dict[str, int] = {}

        self.waypoints = self._load_waypoints()
        if not self.waypoints:
            raise RuntimeError("参数 ~waypoints 为空，请先在 launch 中加载航点 yaml。")

        self.cmd_pub = rospy.Publisher(self.command_topic, UAVCommand, queue_size=10)
        self.gimbal_pub = rospy.Publisher(self.gimbal_control_topic, GimbalControl, queue_size=10)

        rospy.Subscriber(self.state_topic, UAVState, self._state_cb, queue_size=10)
        rospy.Subscriber(self.control_state_topic, UAVControlState, self._control_state_cb, queue_size=10)
        rospy.Subscriber(self.image_topic, RosImage, self._image_cb, queue_size=1, buff_size=2**24)
        rospy.Subscriber(self.gimbal_state_topic, GimbalState, self._gimbal_state_cb, queue_size=10)

        self.gimbal_home_client = None
        try:
            rospy.wait_for_service(self.gimbal_home_service, timeout=5.0)
            self.gimbal_home_client = rospy.ServiceProxy(self.gimbal_home_service, SetBool)
        except Exception:
            rospy.logwarn("未连接到 %s，继续运行，但不会调用吊舱回中服务。", self.gimbal_home_service)

        rospy.loginfo("book_qr_patrol_mission 启动完成，结果目录：%s", str(self.output_dir))
        rospy.loginfo("图像话题：%s", self.image_topic)
        if self.expected_payloads:
            rospy.loginfo("已加载预期二维码文本 %d 条。", len(self.expected_payloads))

    def _load_expected_payloads(self, folder: str) -> Dict[str, Dict]:
        results = {}
        if not folder:
            return results
        path = Path(folder)
        if not path.exists():
            rospy.logwarn("expected_txt_dir 不存在：%s", folder)
            return results
        for txt_file in sorted(path.glob("*.txt")):
            raw = txt_file.read_text(encoding="utf-8", errors="ignore")
            normalized = normalize_text(raw)
            if not normalized:
                continue
            parsed = parse_payload(normalized)
            parsed["原始内容"] = normalized
            parsed["来源文件"] = str(txt_file)
            results[normalized] = parsed
        return results

    def _load_waypoints(self) -> List[Waypoint]:
        items = rospy.get_param("~waypoints", [])
        result = []
        for item in items:
            result.append(
                Waypoint(
                    name=str(item.get("name")),
                    stage=str(item.get("stage", "")),
                    scan=bool(item.get("scan", True)),
                    x=float(item.get("x", 0.0)),
                    y=float(item.get("y", 0.0)),
                    z=float(item.get("z", self.takeoff_height)),
                    yaw_deg=float(item.get("yaw_deg", 0.0)),
                    hold_sec=float(item.get("hold_sec", 1.5)),
                    scan_timeout_sec=float(item.get("scan_timeout_sec", 6.0)),
                    note=str(item.get("note", "")),
                )
            )
        return result

    def _state_cb(self, msg: UAVState):
        self.uav_state = msg

    def _control_state_cb(self, msg: UAVControlState):
        self.control_state = msg

    def _gimbal_state_cb(self, msg: GimbalState):
        self.gimbal_state = msg

    def _image_cb(self, msg: RosImage):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_image_stamp = msg.header.stamp
            self.frame_count += 1
            now = time.time()
            if self.latest_image is not None and now - self.last_heartbeat_log >= self.heartbeat_sec:
                self.last_heartbeat_log = now
                h, w = self.latest_image.shape[:2]
                rospy.loginfo("图像心跳：frame_count=%d size=%dx%d topic=%s", self.frame_count, w, h, self.image_topic)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "图像转换失败：%s", str(exc))

    @staticmethod
    def _wrap_pi(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _yaw_error_deg(self, target_yaw_rad: float) -> float:
        if self.uav_state is None:
            return 180.0
        current_yaw = float(self.uav_state.attitude[2])
        return abs(math.degrees(self._wrap_pi(target_yaw_rad - current_yaw)))

    def _distance_xyz(self, wp: Waypoint) -> Tuple[float, float, float]:
        if self.uav_state is None:
            return 999.0, 999.0, 999.0
        dx = wp.x - float(self.uav_state.position[0])
        dy = wp.y - float(self.uav_state.position[1])
        dz = wp.z - float(self.uav_state.position[2])
        return dx, dy, dz

    def _at_waypoint(self, wp: Waypoint) -> bool:
        dx, dy, dz = self._distance_xyz(wp)
        xy = math.hypot(dx, dy)
        yaw_err = self._yaw_error_deg(math.radians(wp.yaw_deg))
        return xy <= self.xy_tol and abs(dz) <= self.z_tol and yaw_err <= self.yaw_tol_deg

    def _next_command_id(self) -> int:
        self.command_id += 1
        return self.command_id

    def _publish_move_command(self, wp: Waypoint, command_id: Optional[int] = None):
        cmd = UAVCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "ENU"
        cmd.Agent_CMD = UAVCommand.Move
        cmd.Move_mode = UAVCommand.XYZ_POS
        cmd.position_ref[0] = wp.x
        cmd.position_ref[1] = wp.y
        cmd.position_ref[2] = wp.z
        cmd.yaw_ref = math.radians(wp.yaw_deg)
        cmd.Command_ID = command_id if command_id is not None else self._next_command_id()
        self.cmd_pub.publish(cmd)
        return cmd.Command_ID

    def _publish_takeoff_command(self, command_id: Optional[int] = None):
        cmd = UAVCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "ENU"
        cmd.Agent_CMD = UAVCommand.Init_Pos_Hover
        cmd.Command_ID = command_id if command_id is not None else self._next_command_id()
        self.cmd_pub.publish(cmd)
        return cmd.Command_ID

    def _publish_land_command(self, command_id: Optional[int] = None):
        cmd = UAVCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "ENU"
        cmd.Agent_CMD = UAVCommand.Land
        cmd.Command_ID = command_id if command_id is not None else self._next_command_id()
        self.cmd_pub.publish(cmd)
        return cmd.Command_ID

    def _publish_gimbal_fixed(self):
        msg = GimbalControl()
        msg.header.stamp = rospy.Time.now()
        msg.mode = 2
        msg.angle[0] = self.gimbal_fixed_roll_deg
        msg.angle[1] = self.gimbal_fixed_pitch_deg
        msg.angle[2] = self.gimbal_fixed_yaw_deg
        self.gimbal_pub.publish(msg)

    def _try_gimbal_home(self):
        if self.gimbal_home_client is None:
            return
        try:
            self.gimbal_home_client(True)
            rospy.loginfo("已调用吊舱回中服务。")
        except Exception as exc:
            rospy.logwarn("吊舱回中服务调用失败：%s", str(exc))

    def _wait_for_topics(self, timeout_sec: float = 15.0) -> bool:
        start = time.time()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            ok = self.uav_state is not None and self.control_state is not None and self.latest_image is not None
            if ok:
                return True
            if time.time() - start > timeout_sec:
                return False
            rate.sleep()
        return False

    def _wait_for_command_control(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.control_state is not None and self.control_state.control_state == UAVControlState.COMMAND_CONTROL:
                rospy.loginfo("无人机已进入 COMMAND_CONTROL。")
                return
            rospy.logwarn_throttle(5.0, "等待进入 COMMAND_CONTROL。请先解锁并切到命令控制模式。")
            rate.sleep()

    def _takeoff(self):
        rospy.loginfo("发送 Init_Pos_Hover 起飞命令。")
        cmd_id = self._publish_takeoff_command()
        start = time.time()
        rate = rospy.Rate(self.command_rate_hz)
        while not rospy.is_shutdown():
            self._publish_takeoff_command(command_id=cmd_id)
            self._publish_gimbal_fixed()
            if self.uav_state is not None:
                current_z = float(self.uav_state.position[2])
                if abs(current_z - self.takeoff_height) <= self.z_tol:
                    if time.time() - start >= self.takeoff_wait_sec:
                        rospy.loginfo("起飞完成，高度 %.2f m", current_z)
                        return
            rate.sleep()

    def _goto_waypoint(self, wp: Waypoint):
        rospy.loginfo("飞往航点 %s [stage=%s] 目标=(%.2f, %.2f, %.2f, yaw=%.1f°)",
                      wp.name, wp.stage, wp.x, wp.y, wp.z, wp.yaw_deg)
        cmd_id = self._next_command_id()
        stable_start = None
        rate = rospy.Rate(self.command_rate_hz)
        while not rospy.is_shutdown():
            self._publish_move_command(wp, command_id=cmd_id)
            self._publish_gimbal_fixed()
            if self._at_waypoint(wp):
                if stable_start is None:
                    stable_start = time.time()
                elif time.time() - stable_start >= wp.hold_sec:
                    rospy.loginfo("航点 %s 已稳定到位。", wp.name)
                    return True
            else:
                stable_start = None
            rate.sleep()
        return False

    @staticmethod
    def _polygon_area(pts: np.ndarray) -> float:
        if pts is None or len(pts) < 4:
            return 0.0
        pts = pts.reshape(-1, 2)
        x = pts[:, 0]
        y = pts[:, 1]
        return 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))

    def _select_best_candidate(self, decoded_texts: List[str], points, image_shape) -> Tuple[Optional[str], Optional[np.ndarray]]:
        if not decoded_texts:
            return None, None
        if points is None:
            return normalize_text(decoded_texts[0]), None
        h, w = image_shape[:2]
        center = np.array([w / 2.0, h / 2.0], dtype=np.float32)
        best_score = None
        best_text = None
        best_pts = None
        for idx, text in enumerate(decoded_texts):
            if idx >= len(points):
                break
            pts = np.array(points[idx], dtype=np.float32).reshape(-1, 2)
            area = self._polygon_area(pts)
            centroid = np.mean(pts, axis=0)
            dist = float(np.linalg.norm(centroid - center))
            score = float(area - self.center_distance_weight * dist)
            if best_score is None or score > best_score:
                best_score = score
                best_text = normalize_text(text)
                best_pts = pts
        return best_text, best_pts

    def _variants(self, image: np.ndarray):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        eq = cv2.equalizeHist(gray)
        resize2 = cv2.resize(image, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
        gray2 = cv2.cvtColor(resize2, cv2.COLOR_BGR2GRAY)
        th = cv2.adaptiveThreshold(eq, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY, 31, 5)
        blur = cv2.GaussianBlur(gray, (3, 3), 0)
        sharp = cv2.addWeighted(gray, 1.6, blur, -0.6, 0)
        return [
            ("bgr", image),
            ("gray", gray),
            ("equalizeHist", eq),
            ("resize2_bgr", resize2),
            ("resize2_gray", gray2),
            ("adaptiveThreshold", th),
            ("sharpen", sharp),
        ]

    def _decode_variant(self, frame: np.ndarray) -> Tuple[List[str], Optional[np.ndarray]]:
        decoded_texts: List[str] = []
        points = None

        if self.use_pyzbar and _PYZBAR_OK:
            try:
                results = zbar_decode(frame)
                if results:
                    pts_all = []
                    for r in results:
                        try:
                            txt = normalize_text(r.data.decode("utf-8", errors="ignore"))
                        except Exception:
                            txt = normalize_text(str(r.data))
                        if txt:
                            decoded_texts.append(txt)
                        poly = getattr(r, 'polygon', None)
                        if poly and len(poly) >= 4:
                            pts = np.array([[p.x, p.y] for p in poly], dtype=np.float32)
                        else:
                            rect = getattr(r, 'rect', None)
                            if rect is not None:
                                x, y, w, h = rect.left, rect.top, rect.width, rect.height
                                pts = np.array([[x, y], [x+w, y], [x+w, y+h], [x, y+h]], dtype=np.float32)
                            else:
                                pts = None
                        if pts is not None:
                            pts_all.append(pts)
                    if pts_all:
                        points = np.array(pts_all)
            except Exception as e:
                rospy.logwarn_throttle(2.0, 'pyzbar decode failed: %s', str(e))

        if (not decoded_texts) and self.use_opencv_fallback:
            if hasattr(self.detector, "detectAndDecodeMulti"):
                try:
                    retval, decoded_info, points, _ = self.detector.detectAndDecodeMulti(frame)
                    if retval and decoded_info:
                        decoded_texts = [normalize_text(text) for text in decoded_info if text and text.strip()]
                except Exception:
                    pass
            if not decoded_texts:
                try:
                    text, single_points, _ = self.detector.detectAndDecode(frame)
                    text = normalize_text(text) if text else ""
                    if text:
                        decoded_texts = [text]
                        if single_points is not None:
                            points = np.array([single_points])
                except Exception:
                    pass
        return decoded_texts, points

    def _decode_frame(self, frame: np.ndarray) -> Tuple[Optional[str], Optional[np.ndarray], str, List[str], Optional[np.ndarray]]:
        best_text = None
        best_pts = None
        best_strategy = ""
        decoded_texts_all: List[str] = []
        raw_points = None
        for strategy, variant in self._variants(frame):
            decoded_texts, points = self._decode_variant(variant)
            if decoded_texts:
                if self.log_decode_candidates:
                    rospy.loginfo("decode candidate by %s: %s", strategy, decoded_texts[0].replace("\n", " | "))
                selected_text, selected_pts = self._select_best_candidate(decoded_texts, points, variant.shape)
                if selected_text:
                    best_text = selected_text
                    best_pts = selected_pts
                    best_strategy = strategy
                    decoded_texts_all = decoded_texts
                    raw_points = points
                    break
        return best_text, best_pts, best_strategy, decoded_texts_all, raw_points

    def _record_scan(self, wp: Waypoint, raw_text: Optional[str], snapshot_path: str, success: bool,
                     fail_reason: str = "", strategy: str = ""):
        parsed = parse_payload(raw_text) if raw_text else {
            "编号": "",
            "分类号": "",
            "分类名称": "",
            "数量": "",
            "库位": "",
            "状态": "",
        }
        duplicate_index = self.seen_texts.get(raw_text, 0) if raw_text else 0
        if raw_text:
            self.seen_texts[raw_text] = duplicate_index + 1
        expected = self.expected_payloads.get(raw_text, {}) if raw_text else {}
        row = {
            "顺序": len(self.records) + 1,
            "航点名称": wp.name,
            "阶段": wp.stage,
            "是否扫描点": wp.scan,
            "成功": success,
            "失败原因": fail_reason,
            "解码策略": strategy,
            "编号": parsed.get("编号", ""),
            "分类号": parsed.get("分类号", ""),
            "分类名称": parsed.get("分类名称", ""),
            "数量": parsed.get("数量", ""),
            "库位": parsed.get("库位", ""),
            "状态": parsed.get("状态", ""),
            "原始内容": raw_text or "",
            "重复次数": duplicate_index,
            "与预期匹配": bool(expected) if raw_text else False,
            "预期来源文件": expected.get("来源文件", ""),
            "时间": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "x": wp.x,
            "y": wp.y,
            "z": wp.z,
            "yaw_deg": wp.yaw_deg,
            "图片": snapshot_path,
            "备注": wp.note,
        }
        self.records.append(row)
        return row

    def _save_snapshot(self, frame: np.ndarray, wp: Waypoint, overlay_lines: List[str], best_pts: Optional[np.ndarray]) -> str:
        draw = frame.copy()
        if best_pts is not None:
            pts = np.array(best_pts, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(draw, [pts], True, (0, 255, 0), 2)
        draw = draw_multiline_text(draw, overlay_lines, self.font)
        filename = f"{len(self.records)+1:02d}_{wp.name}.jpg"
        path = self.snap_dir / filename

        if self.show_snapshot_preview:
            preview = draw.copy()
            h, w = preview.shape[:2]
            max_w = 960
            if w > max_w:
                scale = max_w / float(w)
                preview = cv2.resize(preview, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            cv2.imshow(self.snapshot_preview_window_name, preview)
            cv2.waitKey(max(1, self.snapshot_preview_wait_ms))

        cv2.imwrite(str(path), draw)
        self.last_debug_frame = draw
        return str(path)

    def _save_raw_debug_frame(self, frame: np.ndarray, wp: Waypoint, tick: int):
        if not self.save_raw_debug_frames:
            return
        if tick % max(1, self.save_debug_every_n_frames) != 0:
            return
        path = self.debug_dir / f"{wp.name}_{tick:04d}.jpg"
        cv2.imwrite(str(path), frame)
        rospy.loginfo("保存原始调试帧：%s", str(path))

    def _scan_at_waypoint(self, wp: Waypoint):
        if not wp.scan:
            return None, True
        rospy.loginfo("开始扫描航点 %s，超时时间 %.1f s", wp.name, wp.scan_timeout_sec)
        start = time.time()
        last_decode_t = 0.0
        success_counter = 0
        last_text = None
        best_pts = None
        best_frame = None
        best_strategy = ""
        tick = 0

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._publish_gimbal_fixed()
            now = time.time()
            if self.latest_image is not None and now - last_decode_t >= self.decode_interval_sec:
                last_decode_t = now
                tick += 1
                frame = self.latest_image.copy()
                self._save_raw_debug_frame(frame, wp, tick)
                text, selected_pts, strategy, all_texts, _ = self._decode_frame(frame)
                if text:
                    if last_text == text:
                        success_counter += 1
                    else:
                        last_text = text
                        success_counter = 1
                    best_pts = selected_pts
                    best_frame = frame
                    best_strategy = strategy
                    parsed = parse_payload(text)
                    rospy.loginfo(
                        "航点 %s 检测到二维码：strategy=%s 编号=%s 分类=%s [重复计数=%d]",
                        wp.name, strategy, parsed.get("编号", ""), parsed.get("分类名称", ""), success_counter
                    )
                    if success_counter >= self.success_repeat_threshold:
                        overlay = [
                            f"航点：{wp.name}",
                            f"阶段：{wp.stage}",
                            f"扫描成功 strategy={strategy}",
                            text,
                        ]
                        snap = self._save_snapshot(best_frame, wp, overlay, best_pts)
                        return self._record_scan(wp, text, snap, True, strategy=strategy), True
                if self.show_debug_window:
                    debug = frame.copy()
                    if selected_pts is not None:
                        pts = np.array(selected_pts, dtype=np.int32).reshape((-1, 1, 2))
                        cv2.polylines(debug, [pts], True, (0, 255, 0), 2)
                    lines = [
                        f"航点：{wp.name}",
                        f"阶段：{wp.stage}",
                        f"倒计时：{max(0.0, wp.scan_timeout_sec - (now - start)):.1f}s",
                        f"候选数：{len(all_texts)}",
                        f"当前重复次数：{success_counter}",
                        f"策略：{strategy or 'none'}",
                        f"文本：{(last_text or '未识别').replace(chr(10), ' | ')}",
                    ]
                    debug = draw_multiline_text(debug, lines, self.font)
                    cv2.imshow("book_qr_patrol_debug", debug)
                    cv2.waitKey(1)
            if now - start >= wp.scan_timeout_sec:
                frame = self.latest_image.copy() if self.latest_image is not None else np.zeros((720, 1280, 3), dtype=np.uint8)
                overlay = [
                    f"航点：{wp.name}",
                    f"阶段：{wp.stage}",
                    "扫描失败",
                    f"超时：{wp.scan_timeout_sec:.1f}s",
                    f"最后策略：{best_strategy or 'none'}",
                    f"最后文本：{(last_text or '').replace(chr(10), ' | ')}",
                ]
                snap = self._save_snapshot(frame, wp, overlay, best_pts)
                return self._record_scan(wp, last_text, snap, False, fail_reason="scan_timeout", strategy=best_strategy), False
            rate.sleep()
        return None, False

    def _export_csv(self, save_path: Path):
        fieldnames = [
            "顺序", "航点名称", "阶段", "是否扫描点", "成功", "失败原因", "解码策略", "编号", "分类号", "分类名称",
            "数量", "库位", "状态", "原始内容", "重复次数", "与预期匹配", "预期来源文件",
            "时间", "x", "y", "z", "yaw_deg", "图片", "备注",
        ]
        with open(save_path, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.records:
                writer.writerow(row)

    def _export_xlsx(self, save_path: Path):
        wb = Workbook()
        ws = wb.active
        ws.title = "巡检结果"
        headers = [
            "顺序", "航点名称", "阶段", "是否扫描点", "成功", "失败原因", "解码策略", "编号", "分类号", "分类名称",
            "数量", "库位", "状态", "原始内容", "重复次数", "与预期匹配", "预期来源文件",
            "时间", "x", "y", "z", "yaw_deg", "图片", "备注",
        ]
        ws.append(headers)
        for row in self.records:
            ws.append([row.get(h, "") for h in headers])
        widths = {
            "A": 8, "B": 18, "C": 14, "D": 12, "E": 8, "F": 16, "G": 16, "H": 20, "I": 8, "J": 20,
            "K": 8, "L": 12, "M": 10, "N": 60, "O": 10, "P": 10, "Q": 40, "R": 20,
            "S": 8, "T": 8, "U": 8, "V": 10, "W": 40, "X": 20,
        }
        for col, width in widths.items():
            ws.column_dimensions[col].width = width
        wb.save(save_path)

    def _land_and_wait(self):
        if self.finish_hold_sec > 0.0:
            rospy.loginfo("全部扫描完成，悬停 %.1f 秒后开始降落。", self.finish_hold_sec)
            time.sleep(self.finish_hold_sec)

        rospy.loginfo("发送降落命令。")
        cmd_id = self._publish_land_command()
        end_t = time.time() + self.land_timeout_sec
        rate = rospy.Rate(self.command_rate_hz)
        last_log_t = 0.0
        while not rospy.is_shutdown() and time.time() < end_t:
            self._publish_land_command(command_id=cmd_id)
            now = time.time()
            if self.uav_state is not None and now - last_log_t > 1.0:
                last_log_t = now
                rospy.loginfo("降落中：当前高度 z=%.2f m", float(self.uav_state.position[2]))
                if float(self.uav_state.position[2]) <= self.land_min_z:
                    rospy.loginfo("高度已低于 %.2f m，认为已完成降落。", self.land_min_z)
                    break
            rate.sleep()

    def _save_results(self):
        csv_path = self.output_dir / "book_qr_patrol_summary.csv"
        xlsx_path = self.output_dir / "book_qr_patrol_summary.xlsx"
        self._export_csv(csv_path)
        self._export_xlsx(xlsx_path)
        summary_lines = [
            "book_qr_patrol_mission 已结束",
            f"结果目录：{self.output_dir}",
            f"CSV：{csv_path}",
            f"XLSX：{xlsx_path}",
            f"总记录数：{len(self.records)}",
            f"成功数：{sum(1 for r in self.records if r.get('成功'))}",
        ]
        (self.output_dir / "summary.txt").write_text("\n".join(summary_lines), encoding="utf-8")
        for line in summary_lines:
            rospy.loginfo(line)

    def run(self):
        if not self._wait_for_topics():
            rospy.logerr("启动超时：未等到 UAVState / UAVControlState / G1 图像话题。")
            return
        self._wait_for_command_control()
        self._try_gimbal_home()
        time.sleep(self.gimbal_settle_sec)
        self._publish_gimbal_fixed()
        self._takeoff()

        for wp in self.waypoints:
            ok = self._goto_waypoint(wp)
            if not ok:
                rospy.logerr("航点 %s 飞行失败，任务中止。", wp.name)
                break
            row, scan_ok = self._scan_at_waypoint(wp)
            if row is not None:
                rospy.loginfo("航点 %s 结果：成功=%s 编号=%s 分类=%s 策略=%s",
                              wp.name, row.get("成功"), row.get("编号"), row.get("分类名称"), row.get("解码策略"))
            if wp.scan and (not scan_ok) and self.abort_on_scan_fail:
                rospy.logerr("航点 %s 扫描失败，按配置终止后续任务。", wp.name)
                break

        self._save_results()

        if self.land_after_finish:
            self._land_and_wait()

        if self.show_debug_window or self.show_snapshot_preview:
            cv2.destroyAllWindows()


def main():
    rospy.init_node("book_qr_patrol_mission", anonymous=False)
    try:
        node = BookQrPatrolMission()
        node.run()
    except Exception as exc:
        rospy.logfatal("book_qr_patrol_mission 异常退出：%s", str(exc))
        raise


if __name__ == "__main__":
    main()
