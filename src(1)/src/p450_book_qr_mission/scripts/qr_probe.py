#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import csv
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

try:
    from pyzbar.pyzbar import decode as zbar_decode
    _PYZBAR_OK = True
except Exception:
    zbar_decode = None
    _PYZBAR_OK = False


def normalize_text(text: str) -> str:
    text = text.replace('\r\n', '\n').replace('\r', '\n')
    lines = [line.strip() for line in text.split('\n')]
    lines = [line for line in lines if line]
    return '\n'.join(lines).strip()


def parse_payload(raw_text: str) -> dict:
    result = {
        '编号': '',
        '分类号': '',
        '分类名称': '',
        '数量': '',
        '库位': '',
        '状态': '',
    }
    for line in raw_text.split('\n'):
        parts = re.split(r'[：:]', line, maxsplit=1)
        if len(parts) != 2:
            continue
        key = parts[0].strip()
        value = parts[1].strip()
        if key == '编号':
            result['编号'] = value
        elif key == '分类':
            if '-' in value:
                class_code, class_name = value.split('-', 1)
                result['分类号'] = class_code.strip()
                result['分类名称'] = class_name.strip()
            else:
                result['分类名称'] = value
        elif key == '数量':
            m = re.search(r'(\d+)', value)
            result['数量'] = m.group(1) if m else value
        elif key == '库位':
            result['库位'] = value
        elif key == '状态':
            result['状态'] = value
    return result


class QRProbeDebug(object):
    def __init__(self):
        self.uav_id = rospy.get_param('~uav_id', 1)
        self.image_topic = rospy.get_param('~image_topic', f'/uav{self.uav_id}/gimbal/image_original')
        self.output_root = os.path.expanduser(rospy.get_param('~output_root', '~/p450_results/qr_probe_debug'))
        self.min_save_interval = float(rospy.get_param('~min_save_interval', 1.5))
        self.heartbeat_sec = float(rospy.get_param('~heartbeat_sec', 2.0))
        self.save_debug_frames = bool(rospy.get_param('~save_debug_frames', True))
        self.save_every_n_frames = int(rospy.get_param('~save_every_n_frames', 60))
        self.print_repeat = bool(rospy.get_param('~print_repeat', False))
        self.show_decode_attempts = bool(rospy.get_param('~show_decode_attempts', True))

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self.use_pyzbar = bool(rospy.get_param('~use_pyzbar', True))
        self.use_opencv_fallback = bool(rospy.get_param('~use_opencv_fallback', False))

        self.session = time.strftime('%Y%m%d_%H%M%S')
        self.output_dir = os.path.join(self.output_root, self.session)
        self.image_dir = os.path.join(self.output_dir, 'images')
        self.debug_dir = os.path.join(self.output_dir, 'debug_frames')
        os.makedirs(self.image_dir, exist_ok=True)
        os.makedirs(self.debug_dir, exist_ok=True)

        self.csv_path = os.path.join(self.output_dir, 'detections.csv')
        self.latest_txt_path = os.path.join(self.output_dir, 'latest_detection.txt')
        with open(self.csv_path, 'w', newline='', encoding='utf-8-sig') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'topic', 'strategy', 'qr_text', '编号', '分类号', '分类名称', '数量', '库位', '状态', 'image_path'])

        self.last_text = None
        self.last_save_time = 0.0
        self.last_hb = 0.0
        self.frame_count = 0
        self.last_msg_stamp = None
        self.last_msg_wall = None

        self.sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)

        rospy.loginfo('==============================================')
        rospy.loginfo('QR probe debug started')
        rospy.loginfo('image topic: %s', self.image_topic)
        rospy.loginfo('output dir : %s', self.output_dir)
        rospy.loginfo('pyzbar ok  : %s', _PYZBAR_OK)
        rospy.loginfo('opencv fallback: %s', self.use_opencv_fallback)
        rospy.loginfo('==============================================')

    def _variants(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        eq = cv2.equalizeHist(gray)
        resize2 = cv2.resize(image, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
        gray2 = cv2.cvtColor(resize2, cv2.COLOR_BGR2GRAY)
        th = cv2.adaptiveThreshold(eq, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY, 31, 5)
        blur = cv2.GaussianBlur(gray, (3, 3), 0)
        sharp = cv2.addWeighted(gray, 1.6, blur, -0.6, 0)
        return [
            ('bgr', image),
            ('gray', gray),
            ('equalizeHist', eq),
            ('resize2_bgr', resize2),
            ('resize2_gray', gray2),
            ('adaptiveThreshold', th),
            ('sharpen', sharp),
        ]

    def _decode_one(self, img):
        texts = []
        points_out = None

        if self.use_pyzbar and _PYZBAR_OK:
            try:
                results = zbar_decode(img)
                if results:
                    texts = []
                    point_list = []
                    for r in results:
                        try:
                            txt = normalize_text(r.data.decode("utf-8", errors="ignore"))
                        except Exception:
                            txt = normalize_text(str(r.data))
                        if txt:
                            texts.append(txt)
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
                            point_list.append(pts)
                    if point_list:
                        points_out = np.array(point_list)
            except Exception as e:
                rospy.logwarn_throttle(2.0, 'pyzbar decode failed: %s', str(e))

        if (not texts) and self.use_opencv_fallback:
            try:
                retval, decoded_info, points, _ = self.detector.detectAndDecodeMulti(img)
                if retval and decoded_info:
                    for t in decoded_info:
                        if t is not None:
                            t = normalize_text(t)
                            if t:
                                texts.append(t)
                    points_out = points
            except Exception:
                pass
            if not texts:
                try:
                    text, points, _ = self.detector.detectAndDecode(img)
                    text = normalize_text(text) if text else ''
                    if text:
                        texts = [text]
                        points_out = None if points is None else [points]
                except Exception:
                    pass
        return texts, points_out

    def _draw_boxes(self, image, points):
        if points is None:
            return image
        canvas = image.copy()
        try:
            for box in points:
                if box is None:
                    continue
                pts = np.array(box).reshape(-1, 2).astype(int)
                for i in range(len(pts)):
                    cv2.line(canvas, tuple(pts[i]), tuple(pts[(i + 1) % len(pts)]), (0, 255, 0), 2)
        except Exception:
            pass
        return canvas

    def _save_detection(self, text, image, strategy):
        now = time.time()
        if (now - self.last_save_time) < self.min_save_interval:
            return
        parsed = parse_payload(text)
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        image_path = os.path.join(self.image_dir, f'{timestamp}_{strategy}.jpg')
        cv2.imwrite(image_path, image)
        with open(self.csv_path, 'a', newline='', encoding='utf-8-sig') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp, self.image_topic, strategy, text,
                parsed.get('编号', ''), parsed.get('分类号', ''), parsed.get('分类名称', ''),
                parsed.get('数量', ''), parsed.get('库位', ''), parsed.get('状态', ''), image_path
            ])
        with open(self.latest_txt_path, 'w', encoding='utf-8') as f:
            f.write(text + '\n')
        rospy.loginfo('==============================================')
        rospy.loginfo('QR decode success')
        rospy.loginfo('strategy  : %s', strategy)
        rospy.loginfo('text      : %s', text.replace('\n', ' | '))
        rospy.loginfo('parsed    : 编号=%s 分类号=%s 分类名称=%s 数量=%s 库位=%s 状态=%s',
                      parsed.get('编号', ''), parsed.get('分类号', ''), parsed.get('分类名称', ''),
                      parsed.get('数量', ''), parsed.get('库位', ''), parsed.get('状态', ''))
        rospy.loginfo('saved img : %s', image_path)
        rospy.loginfo('saved txt : %s', self.latest_txt_path)
        rospy.loginfo('saved csv : %s', self.csv_path)
        rospy.loginfo('==============================================')
        self.last_save_time = now
        self.last_text = text

    def image_callback(self, msg):
        self.frame_count += 1
        self.last_msg_stamp = msg.header.stamp.to_sec() if msg.header.stamp else 0.0
        self.last_msg_wall = time.time()
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn_throttle(2.0, 'cv_bridge failed: %s', str(e))
            return

        # heartbeat proves frames are arriving even if decode fails
        if time.time() - self.last_hb >= self.heartbeat_sec:
            self.last_hb = time.time()
            h, w = image.shape[:2]
            rospy.loginfo('heartbeat: frame_count=%d size=%dx%d topic=%s', self.frame_count, w, h, self.image_topic)

        if self.save_debug_frames and self.frame_count % max(1, self.save_every_n_frames) == 0:
            dbg_path = os.path.join(self.debug_dir, f'frame_{self.frame_count:06d}.jpg')
            cv2.imwrite(dbg_path, image)
            rospy.loginfo('saved raw debug frame: %s', dbg_path)

        best = None
        best_points = None
        best_strategy = None
        for strategy, variant in self._variants(image):
            texts, points = self._decode_one(variant)
            if texts:
                best = texts[0]
                best_points = points
                best_strategy = strategy
                if self.show_decode_attempts:
                    rospy.loginfo('decode candidate by %s: %s', strategy, best.replace('\n', ' | '))
                break

        if not best:
            return

        if (not self.print_repeat) and (best == self.last_text):
            return

        annotated = self._draw_boxes(image, best_points)
        self._save_detection(best, annotated, best_strategy or 'unknown')


if __name__ == '__main__':
    rospy.init_node('qr_probe_debug', anonymous=False)
    QRProbeDebug()
    rospy.spin()
