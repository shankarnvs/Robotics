import cv2
import numpy as np
import time
from picamera2 import Picamera2
from libcamera import Transform

from ultrafastLaneDetector import UltrafastLaneDetector, ModelType


class Perception:
    def __init__(self):
        # ===== CAMERA SETTINGS =====
        self.CAM_W, self.CAM_H = 2304, 1296
        self.INFER_W, self.INFER_H = 1280, 960

        # Deadband for steering stability
        self.DEADBAND = 80

        # ===== LOAD MODEL =====
        MODEL_PATH = "models/lane.tflite"
        self.lane_detector = UltrafastLaneDetector(MODEL_PATH, ModelType.TUSIMPLE)

        # ===== CAMERA INIT =====
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (self.CAM_W, self.CAM_H), "format": "RGB888"},
            transform=Transform(hflip=1, vflip=1)  # 180° rotation
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1)

        print("ML Lane Perception Initialized")

    # ---------------- PREPROCESS ----------------
    def preprocess(self, frame_bgr):
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        gamma = 1.4
        invGamma = 1.0 / gamma
        table = np.array([(i / 255.0) ** invGamma * 255
                          for i in np.arange(256)]).astype("uint8")
        gray = cv2.LUT(gray, table)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    # ---------------- SCALE POINTS ----------------
    def scale_lane_points(self, lanes, sx, sy):
        scaled = []
        for lane in lanes:
            scaled.append([(int(x * sx), int(y * sy)) for x, y in lane])
        return scaled

    # ---------------- STEERING LOGIC ----------------
    def compute_steering(self, lanes_points, lanes_detected):
        left_idx, right_idx = 1, 2

        have_left = len(lanes_detected) > left_idx and lanes_detected[left_idx]
        have_right = len(lanes_detected) > right_idx and lanes_detected[right_idx]

        if not (have_left and have_right):
            return "no steer", None  # fail-safe

        left_lane = sorted(lanes_points[left_idx], key=lambda p: p[1])
        right_lane = sorted(lanes_points[right_idx], key=lambda p: p[1])

        # Use bottom-most points for lookahead
        lx, _ = left_lane[-1]
        rx, _ = right_lane[-1]

        lane_center = (lx + rx) // 2
        image_center = self.CAM_W // 2
        offset = lane_center - image_center

        if offset > self.DEADBAND:
            return "right", offset
        elif offset < -self.DEADBAND:
            return "left", offset
        else:
            return "no steer", offset

    # ---------------- MAIN FRAME PROCESS ----------------
    def process_frame(self):
        try:
            frame_rgb = self.picam2.capture_array()
        except Exception as e:
            print("Camera capture failed:", e)
            return None, None, None

        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        raw_full = frame_bgr.copy()

        processed = self.preprocess(frame_bgr)
        infer_bgr = cv2.resize(processed, (self.INFER_W, self.INFER_H))

        overlay = self.lane_detector.detect_lanes(infer_bgr)

        lanes_points = self.lane_detector.lanes_points
        lanes_detected = self.lane_detector.lanes_detected

        if hasattr(lanes_points, "tolist"):
            lanes_points = lanes_points.tolist()
        if hasattr(lanes_detected, "tolist"):
            lanes_detected = lanes_detected.tolist()

        sx, sy = self.CAM_W / self.INFER_W, self.CAM_H / self.INFER_H
        lanes_points_full = self.scale_lane_points(lanes_points, sx, sy)

        steer, offset = self.compute_steering(lanes_points_full, lanes_detected)

        # Debug overlay
        debug = raw_full.copy()
        if offset is not None:
            cv2.putText(debug, f"Offset: {offset}", (30, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        return steer, raw_full, overlay

    def cleanup(self):
        self.picam2.stop()
        cv2.destroyAllWindows()
