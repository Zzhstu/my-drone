#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS node for P450/G1 no-flight YOLO detection + click/auto single-object tracking.

Main fixes in this version:
1) Explicit task='detect' for Ultralytics backend.
2) Add class_names param and fallback category mapping for VisDrone engines without full metadata.
3) Strict click selection: if the click does not land inside any bbox, do NOT lock any target.
4) reset() publishes an empty TargetsInFrame once to stop follow logic more reliably.
5) Optional safer tracking defaults through ROS params:
   - lost_threshold
   - tracker_type
   - conf
"""
import os
import math
import time
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from spirecv_msgs.msg import Target, TargetsInFrame, Control
from std_msgs.msg import Bool

try:
    from ultralytics import YOLO
except Exception as e:
    YOLO = None
    ULTRA_ERR = e
else:
    ULTRA_ERR = None

try:
    import torch
except Exception as e:
    torch = None
    TORCH_ERR = e
else:
    TORCH_ERR = None


DEFAULT_VISDRONE_NAMES = [
    'pedestrian', 'people', 'bicycle', 'car', 'van',
    'truck', 'tricycle', 'awning-tricycle', 'bus', 'motor'
]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def parse_classes(s):
    if s in [None, '', 'all', 'ALL']:
        return None
    if isinstance(s, list):
        return [int(x) for x in s]
    return [int(x.strip()) for x in str(s).split(',') if x.strip()]


def parse_class_names(v):
    if v is None:
        return list(DEFAULT_VISDRONE_NAMES)
    if isinstance(v, (list, tuple)):
        out = [str(x).strip() for x in v if str(x).strip()]
        return out if out else list(DEFAULT_VISDRONE_NAMES)
    s = str(v).strip()
    if not s:
        return list(DEFAULT_VISDRONE_NAMES)
    if ',' in s:
        out = [x.strip() for x in s.split(',') if x.strip()]
        return out if out else list(DEFAULT_VISDRONE_NAMES)
    return [s]


def looks_numeric_names(names):
    if isinstance(names, dict):
        vals = list(names.values())
    elif isinstance(names, (list, tuple)):
        vals = list(names)
    else:
        return True
    if not vals:
        return True
    numeric_like = 0
    for x in vals:
        sx = str(x).strip().lower()
        if sx.isdigit() or sx.startswith('class'):
            numeric_like += 1
    return numeric_like >= max(1, int(0.6 * len(vals)))


def make_name_map(class_names):
    return {i: n for i, n in enumerate(class_names)}


def best_effort_names(raw_names, class_names):
    if isinstance(raw_names, dict) and raw_names and not looks_numeric_names(raw_names):
        return {int(k): str(v) for k, v in raw_names.items()}
    if isinstance(raw_names, (list, tuple)) and raw_names and not looks_numeric_names(raw_names):
        return {i: str(v) for i, v in enumerate(raw_names)}
    return make_name_map(class_names)


def make_tracker(name):
    name = str(name).upper()
    creators = []
    if hasattr(cv2, 'legacy'):
        creators += [getattr(cv2.legacy, 'Tracker%s_create' % name, None)]
        for n in ['KCF', 'CSRT', 'MOSSE', 'MIL']:
            creators.append(getattr(cv2.legacy, 'Tracker%s_create' % n, None))
    creators += [getattr(cv2, 'Tracker%s_create' % name, None)]
    for n in ['KCF', 'CSRT', 'MOSSE', 'MIL']:
        creators.append(getattr(cv2, 'Tracker%s_create' % n, None))
    for c in creators:
        if c:
            try:
                return c()
            except Exception:
                pass
    return None


class YoloBackend:
    def __init__(self, backend, model_path, imgsz, conf, iou, classes, max_det, device, yolov5_repo, class_names):
        self.backend = backend
        self.model_path = os.path.expanduser(model_path)
        self.imgsz = int(imgsz)
        self.conf = float(conf)
        self.iou = float(iou)
        self.classes = classes
        self.max_det = int(max_det)
        self.device = str(device)
        self.yolov5_repo = os.path.expanduser(yolov5_repo)
        self.class_names = parse_class_names(class_names)
        self.names = make_name_map(self.class_names)
        self.model = None
        self._load()

    def _load(self):
        if self.backend == 'auto':
            lower = os.path.basename(self.model_path).lower()
            self.backend = 'yolov5_hub' if 'yolov5' in lower and os.path.isdir(self.yolov5_repo) else 'ultralytics'

        if self.backend == 'ultralytics':
            if YOLO is None:
                raise RuntimeError('Cannot import ultralytics. Install with: pip3 install ultralytics. Error: %r' % ULTRA_ERR)
            # Explicit detect task to avoid task auto-guess warning for engine files.
            self.model = YOLO(self.model_path, task='detect')
            raw_names = getattr(self.model, 'names', {}) or {}
            self.names = best_effort_names(raw_names, self.class_names)
            rospy.loginfo('Loaded Ultralytics detect model: %s', self.model_path)
        elif self.backend == 'yolov5_hub':
            if torch is None:
                raise RuntimeError('Cannot import torch. Install PyTorch first. Error: %r' % TORCH_ERR)
            if not os.path.isdir(self.yolov5_repo):
                raise RuntimeError('backend=yolov5_hub requires yolov5_repo, e.g. /home/amov/yolov5')
            self.model = torch.hub.load(self.yolov5_repo, 'custom', path=self.model_path, source='local')
            self.model.conf = self.conf
            self.model.iou = self.iou
            self.model.max_det = self.max_det
            if self.classes is not None:
                self.model.classes = self.classes
            if self.device not in ['cpu', 'CPU', '-1']:
                self.model.to('cuda:%s' % self.device if self.device.isdigit() else self.device)
            raw_names = getattr(self.model, 'names', {}) or {}
            self.names = best_effort_names(raw_names, self.class_names)
            rospy.loginfo('Loaded original YOLOv5 model: %s repo=%s', self.model_path, self.yolov5_repo)
        else:
            raise RuntimeError('Unknown backend=%s, use auto|ultralytics|yolov5_hub' % self.backend)

    def predict(self, img):
        dets = []
        t0 = time.time()
        if self.backend == 'ultralytics':
            r = self.model.predict(
                img,
                task='detect',
                imgsz=self.imgsz,
                conf=self.conf,
                iou=self.iou,
                classes=self.classes,
                max_det=self.max_det,
                device=self.device,
                verbose=False
            )[0]
            if getattr(r, 'names', None):
                self.names = best_effort_names(r.names, self.class_names)
            if r.boxes is not None:
                boxes = r.boxes.xyxy.cpu().numpy()
                scores = r.boxes.conf.cpu().numpy()
                clss = r.boxes.cls.cpu().numpy().astype(int)
                for box, score, cls_id in zip(boxes, scores, clss):
                    d = self._format_det(box, score, cls_id)
                    if d is not None:
                        dets.append(d)
        else:
            res = self.model(img, size=self.imgsz)
            arr = res.xyxy[0].detach().cpu().numpy() if len(res.xyxy) else []
            for row in arr:
                x1, y1, x2, y2, score, cls_id = row[:6]
                d = self._format_det((x1, y1, x2, y2), score, int(cls_id))
                if d is not None:
                    dets.append(d)
        return dets, (time.time() - t0) * 1000.0

    def _format_det(self, box, score, cls_id):
        x1, y1, x2, y2 = [int(round(float(v))) for v in box]
        if x2 <= x1 or y2 <= y1:
            return None
        name = self.names.get(int(cls_id), 'class%d' % int(cls_id))
        return {
            'bbox': (x1, y1, x2 - x1, y2 - y1),
            'score': float(score),
            'cls_id': int(cls_id),
            'name': str(name),
        }


class Node:
    def __init__(self):
        self.uav_id = int(rospy.get_param('~uav_id', 1))
        self.model_path = rospy.get_param('~model_path', '/home/amov/SpireCV/models/yolo11n.pt')
        self.model_name = rospy.get_param('~model_name', os.path.basename(self.model_path))
        self.backend = rospy.get_param('~backend', 'auto')
        self.yolov5_repo = rospy.get_param('~yolov5_repo', '/home/amov/yolov5')
        self.device = rospy.get_param('~device', '0')
        self.imgsz = int(rospy.get_param('~imgsz', 640))
        self.conf = float(rospy.get_param('~conf', 0.50))
        self.iou = float(rospy.get_param('~iou', 0.45))
        self.classes = parse_classes(rospy.get_param('~classes', 'all'))
        self.class_names = parse_class_names(rospy.get_param('~class_names', ','.join(DEFAULT_VISDRONE_NAMES)))
        self.max_det = int(rospy.get_param('~max_det', 50))
        self.input_topic = rospy.get_param('~input_image_topic', '/uav%d/gimbal/image_original' % self.uav_id)
        self.output_topic = rospy.get_param('~output_image_topic', '/uav%d/spirecv/image_det' % self.uav_id)
        self.target_topic = rospy.get_param('~target_topic', '/uav%d/spirecv/target' % self.uav_id)
        self.control_topic = rospy.get_param('~control_topic', '/uav%d/spirecv/control' % self.uav_id)
        self.fov_x = float(rospy.get_param('~fov_x', 60.0))
        self.fov_y = float(rospy.get_param('~fov_y', 39.0))
        self.target_w_m = float(rospy.get_param('~target_real_width_m', 0.0))
        self.target_h_m = float(rospy.get_param('~target_real_height_m', 0.0))
        self.auto_select = bool(rospy.get_param('~auto_select', False))
        self.auto_policy = rospy.get_param('~auto_select_policy', 'highest_score')
        self.enable_tracking = bool(rospy.get_param('~enable_tracking', True))
        self.tracker_type = rospy.get_param('~tracker_type', 'KCF')
        self.lost_threshold = int(rospy.get_param('~lost_threshold', 5))
        self.enable_ui = bool(rospy.get_param('~enable_ui', False))
        self.log_every_n = int(rospy.get_param('~log_every_n', 30))
        self.strict_click = bool(rospy.get_param('~strict_click', True))
        self.publish_empty_on_reset = bool(rospy.get_param('~publish_empty_on_reset', True))
        self.click_bbox_pad = float(rospy.get_param('~click_bbox_pad', 0.10))
        self.click_bbox_min_px = int(rospy.get_param('~click_bbox_min_px', 20))
        self.click_pick_policy = rospy.get_param('~click_pick_policy', 'nearest')
        self.manual_override_topic = rospy.get_param(
             '~manual_override_topic',
             '/uav%d/p450/manual_override' % self.uav_id
             )
        self.manual_override = False
        self.manual_override_empty_period = float(rospy.get_param('~manual_override_empty_period', 0.5))
        self.last_manual_empty_pub = rospy.Time(0)	
	
	
        self.detector = YoloBackend(
            self.backend, self.model_path, self.imgsz, self.conf, self.iou,
            self.classes, self.max_det, self.device, self.yolov5_repo, self.class_names
        )
        self.bridge = CvBridge()
        self.frame_id = 0
        self.last_time = rospy.Time.now()
        self.fps = 0.0
        self.infer_ms = 0.0
        self.click = None
        self.tracker = None
        self.tracking = False
        self.bbox = None
        self.no_track = 999
        self.cat = 'target'
        self.cat_id = -1
        self.score = 1.0
        self.last_w = 1
        self.last_h = 1
        self.last_header = None

        self.pub_img = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.pub_tgt = rospy.Publisher(self.target_topic, TargetsInFrame, queue_size=1)
        self.sub_img = rospy.Subscriber(self.input_topic, Image, self.cb_img, queue_size=1, buff_size=2 ** 24)
        self.sub_ctl = rospy.Subscriber(self.control_topic, Control, self.cb_ctl, queue_size=10)
        self.manual_override_sub = rospy.Subscriber(
              self.manual_override_topic,
              Bool,
              self.cb_manual_override,
              queue_size=1
              )


        if self.enable_ui:
            cv2.namedWindow('P450 YOLO Detect Track', cv2.WINDOW_NORMAL)
            cv2.setMouseCallback('P450 YOLO Detect Track', self.cb_mouse)

        rospy.loginfo(
            'YOLO node ready: backend=%s model=%s input=%s target=%s strict_click=%s tracker=%s conf=%.2f lost=%d',
            self.detector.backend, self.model_path, self.input_topic, self.target_topic,
            self.strict_click, self.tracker_type, self.conf, self.lost_threshold
        )
         

    def publish_output_image(self, vis):
        try:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(vis, 'bgr8'))
            return True
        except Exception as e:
            rospy.logwarn_throttle(1.0, 'publish output image failed: %r', e)
            return False 

    def cb_ctl(self, msg):
        """
        Receive click command from /uavX/spirecv/control.

        Left click:
            - release manual override
            - reset old tracker if needed
            - save normalized click position for the next image frame

        Right click:
            - stop visual tracking immediately
            - publish an empty target message
            - enter manual override mode, so gimbal_manual_teleop.py can take control
        """
        if msg.mouse == Control.MOUSE_LEFT:
            # 用户重新点击目标，说明希望恢复视觉跟踪控制
            self.manual_override = False

            if self.tracking:
                self.reset(publish_empty=True)

            self.click = (
                float(max(0.0, min(1.0, msg.x))),
                float(max(0.0, min(1.0, msg.y)))
            )

            rospy.loginfo(
                'left click received: x=%.3f y=%.3f, manual_override=False',
                self.click[0], self.click[1]
            )

        elif msg.mouse == Control.MOUSE_RIGHT:
            self.click = None
            self.manual_override = False

            self.reset(publish_empty=True)

            try:
                self.publish_empty_targets()
            except Exception as e:
                rospy.logwarn('right click empty target publish failed: %r', e)

            rospy.logwarn('right click received: tracking reset, empty target published')

    def cb_mouse(self, event, x, y, flags, param):
        """
        OpenCV local mouse callback.
        This is used only when enable_ui=True.
        Keep it consistent with cb_ctl().
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            self.manual_override = False

            if self.tracking:
                self.reset(publish_empty=True)

            self.click = (
                float(max(0.0, min(1.0, x / max(1, self.last_w)))),
                float(max(0.0, min(1.0, y / max(1, self.last_h))))
            )

            rospy.loginfo(
                'local left click: x=%.3f y=%.3f, manual_override=False',
                self.click[0], self.click[1]
            )

        elif event == cv2.EVENT_RBUTTONDOWN:
            self.click = None
            self.manual_override = False

            self.reset(publish_empty=True)

            try:
                self.publish_empty_targets()
            except Exception as e:
                rospy.logwarn('local right click empty target publish failed: %r', e)

            rospy.logwarn('local right click: tracking reset, empty target published')


    def publish_empty_targets(self):
        out = TargetsInFrame()
        if self.last_header is not None:
            out.header = self.last_header
        out.frame_id = self.frame_id
        out.height = self.last_h
        out.width = self.last_w
        out.fps = self.fps
        out.fov_x = self.fov_x
        out.fov_y = self.fov_y
        out.no_track_frame_count = self.no_track
        self.pub_tgt.publish(out)

    def reset(self, publish_empty=None):
        self.tracker = None
        self.tracking = False
        self.bbox = None
        self.no_track = 999
        self.click = None
        if publish_empty is None:
            publish_empty = self.publish_empty_on_reset
        if publish_empty:
            self.publish_empty_targets()

    def update_fps(self, stamp):
        now = stamp if stamp != rospy.Time() else rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        inst = 1.0 / dt if dt > 1e-4 else 0.0
        self.fps = inst if self.fps <= 0 else 0.9 * self.fps + 0.1 * inst

    def select_click(self, dets, click, w, h):
        px = clamp(click[0], 0.0, 1.0) * w
        py = clamp(click[1], 0.0, 1.0) * h

        hits = []
        for d in dets:
            x, y, bw, bh = d['bbox']

            pad_x = max(float(self.click_bbox_min_px), float(bw) * float(self.click_bbox_pad))
            pad_y = max(float(self.click_bbox_min_px), float(bh) * float(self.click_bbox_pad))

            x1 = max(0.0, x - pad_x)
            y1 = max(0.0, y - pad_y)
            x2 = min(float(w - 1), x + bw + pad_x)
            y2 = min(float(h - 1), y + bh + pad_y)

            if x1 <= px <= x2 and y1 <= py <= y2:
                cx = x + bw / 2.0
                cy = y + bh / 2.0
                dist2 = (cx - px) ** 2 + (cy - py) ** 2
                hits.append((d, dist2))

        if hits:
            if str(self.click_pick_policy).lower() == 'score':
                return max([d for d, _ in hits], key=lambda d: d['score'])
            return min(hits, key=lambda item: item[1])[0]

        if self.strict_click:
            return None

        return min(
            dets,
            key=lambda d: (d['bbox'][0] + d['bbox'][2] / 2 - px) ** 2 +
                          (d['bbox'][1] + d['bbox'][3] / 2 - py) ** 2
        )

    def select_auto(self, dets, w, h):
        if not dets:
            return None
        if self.auto_policy == 'center':
            return min(dets, key=lambda d: (d['bbox'][0] + d['bbox'][2] / 2 - w / 2) ** 2 + (d['bbox'][1] + d['bbox'][3] / 2 - h / 2) ** 2)
        if self.auto_policy == 'largest':
            return max(dets, key=lambda d: d['bbox'][2] * d['bbox'][3])
        return max(dets, key=lambda d: d['score'])

    def init_track(self, img, d):
        if d is None:
            return False
        self.bbox = tuple(int(v) for v in d['bbox'])
        self.cat = d['name']
        self.cat_id = d['cls_id']
        self.score = d['score']
        self.no_track = 0
        self.tracker = make_tracker(self.tracker_type) if self.enable_tracking else None
        if self.tracker is not None:
            self.tracker.init(img, self.bbox)
        self.tracking = True
        rospy.loginfo('track init category=%s score=%.3f bbox=%s tracker=%s', self.cat, self.score, self.bbox, self.tracker_type)
        return True

    def target_from_bbox(self, bbox, w, h, score, cat, cat_id, mode):
        x, y, bw, bh = bbox
        x = int(clamp(x, 0, w - 1))
        y = int(clamp(y, 0, h - 1))
        bw = int(clamp(bw, 1, w - x))
        bh = int(clamp(bh, 1, h - y))
        cx = (x + bw / 2.0) / float(w)
        cy = (y + bh / 2.0) / float(h)
        nw = bw / float(w)
        nh = bh / float(h)
        t = Target()
        t.cx = cx
        t.cy = cy
        t.w = nw
        t.h = nh
        t.score = float(score)
        t.category = str(cat)
        t.category_id = int(cat_id)
        t.tracked_id = 0
        t.mode = bool(mode)
        t.sim_score = 1.0 if mode else 0.0
        t.los_ax = (cx - 0.5) * self.fov_x
        t.los_ay = (cy - 0.5) * self.fov_y
        t.yaw_a = t.los_ax
        z = []
        if self.target_w_m > 1e-3 and nw > 1e-5:
            z.append(self.target_w_m / (2.0 * math.tan(math.radians(self.fov_x) / 2.0) * nw))
        if self.target_h_m > 1e-3 and nh > 1e-5:
            z.append(self.target_h_m / (2.0 * math.tan(math.radians(self.fov_y) / 2.0) * nh))
        if z:
            t.pz = sum(z) / len(z)
            t.px = math.tan(math.radians(t.los_ax)) * t.pz
            t.py = math.tan(math.radians(t.los_ay)) * t.pz
        return t

    def draw(self, img, t, locked):
        h, w = img.shape[:2]
        x1 = int((t.cx - t.w / 2) * w)
        y1 = int((t.cy - t.h / 2) * h)
        x2 = int((t.cx + t.w / 2) * w)
        y2 = int((t.cy + t.h / 2) * h)
        color = (0, 255, 0) if locked else (255, 170, 0)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        label = ('TRACK ' if locked else 'DET ') + '%s %.2f' % (t.category, t.score)
        cv2.putText(img, label, (x1, max(20, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    def cb_manual_override(self, msg):
        new_state = bool(msg.data)

        if new_state and not self.manual_override:
            rospy.logwarn('manual_override=True: reset YOLO tracker and suppress /uav%d/spirecv/target', self.uav_id)
            self.reset()
            try:
                self.publish_empty_targets()
            except Exception as e:
                rospy.logwarn('manual_override empty target publish failed: %r', e)

        if (not new_state) and self.manual_override:
            rospy.loginfo('manual_override=False: YOLO click tracking is enabled again')

        self.manual_override = new_state


    def cb_img(self, msg):
        if self.manual_override:
            self.tracking = False
            self.tracker = None
            self.track_bbox = None
            self.click = None

            now = rospy.Time.now()
            if (now - self.last_manual_empty_pub).to_sec() >= self.manual_override_empty_period:
                try:
                    self.publish_empty_targets()
                except Exception as e:
                    rospy.logwarn_throttle(
                        1.0,
                        'manual_override empty target publish failed: %r',
                        e
                    )
                self.last_manual_empty_pub = now

            vis = frame.copy()
            cv2.putText(
                vis,
                'MANUAL OVERRIDE - YOLO target suppressed',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 255),
                2,
                cv2.LINE_AA
            )

            self.publish_output_image(vis)
            return
          
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr('cv_bridge conversion failed: %s', e)
            return

        h, w = img.shape[:2]
        self.last_w = w
        self.last_h = h
        self.last_header = msg.header
        self.frame_id += 1
        self.update_fps(msg.header.stamp)

        dets = []
        if not self.tracking:
            dets, self.infer_ms = self.detector.predict(img)

        if self.click is not None:
            if not dets:
                rospy.logwarn_throttle(1.0, 'click ignored: current frame has no detections')
                self.click = None
            else:
                chosen = self.select_click(dets, self.click, w, h)
      
                if chosen is None:
                  rospy.logwarn_throttle(
                    1.0,
                    'click ignored: point=(%.3f, %.3f), no bbox hit; try larger ~click_bbox_pad',
                     float(self.click[0]), float(self.click[1])
                     )
                else:
                    self.init_track(img, chosen)
                self.click = None

        elif self.auto_select and (not self.tracking) and dets:
            self.init_track(img, self.select_auto(dets, w, h))

        locked_t = None
        if self.tracking:
            if self.tracker is not None:
                ok, bbox = self.tracker.update(img)
                if ok:
                    self.bbox = tuple(int(v) for v in bbox)
                    self.no_track = 0
                else:
                    self.no_track += 1
            if self.no_track <= self.lost_threshold and self.bbox is not None:
                locked_t = self.target_from_bbox(self.bbox, w, h, self.score, self.cat, self.cat_id, True)
            else:
                rospy.logwarn_throttle(1.0, 'tracking lost, reset after %d frames', self.no_track)
                self.reset()

        out = TargetsInFrame()
        out.header = msg.header
        out.frame_id = self.frame_id
        out.height = h
        out.width = w
        out.fps = self.fps
        out.fov_x = self.fov_x
        out.fov_y = self.fov_y
        out.no_track_frame_count = self.no_track

        vis = img.copy()
        if locked_t:
            out.targets.append(locked_t)
            self.draw(vis, locked_t, True)
        else:
            for d in dets:
                t = self.target_from_bbox(d['bbox'], w, h, d['score'], d['name'], d['cls_id'], False)
                out.targets.append(t)
                self.draw(vis, t, False)

        if self.log_every_n > 0 and self.frame_id % self.log_every_n == 0:
            rospy.loginfo(
                '%s backend=%s fps=%.1f infer=%.1fms targets=%d tracking=%s',
                self.model_name, self.detector.backend, self.fps, self.infer_ms, len(out.targets), self.tracking
            )

        cv2.putText(
            vis,
            '%s FPS:%.1f infer:%.1fms tracking:%s' % (self.model_name, self.fps, self.infer_ms, self.tracking),
            (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2
        )

        self.pub_tgt.publish(out)
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(vis, 'bgr8'))

        if self.enable_ui:
            cv2.imshow('P450 YOLO Detect Track', vis)
            cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('yolo_detect_track_node')
    Node()
    rospy.spin()
