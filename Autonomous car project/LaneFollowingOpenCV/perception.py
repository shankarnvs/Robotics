import cv2
import numpy as np
import time
from picamera2 import Picamera2
from libcamera import Transform
from ultrafastLaneDetector import UltrafastLaneDetector, ModelType


class Perception:
    def __init__(self):
        # ================= CAMERA SETTINGS =================
        self.CAM_W, self.CAM_H = 2304, 1296   # Full resolution
        self.INFER_W, self.INFER_H = 640, 480  # Model input resolution

        # Steering parameters
        self.DEADBAND = 70
        self.prev_offset = 0

        # ================= LOAD LANE MODEL =================
        MODEL_PATH = "models/lane.tflite"
        self.lane_detector = UltrafastLaneDetector(MODEL_PATH, ModelType.TUSIMPLE)

        # ================= CAMERA INIT =================
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (self.CAM_W, self.CAM_H), "format": "RGB888"},
            transform=Transform(hflip=1, vflip=1)
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1)

        print("🚗 ML Lane Perception Initialized")

    # ---------------- PREPROCESS ----------------
    def preprocess(self, frame_bgr):
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        gamma = 1.4
        invGamma = 1.0 / gamma
        table = np.array([(i / 255.0) ** invGamma * 255
                          for i in np.arange(256)]).astype("uint8")
        gray = cv2.LUT(gray, table)

        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    # ---------------- STEERING LOGIC ----------------
     def compute_steering(self, lanes_points, lanes_detected):
        sx = self.CAM_W / self.INFER_W
        sy = self.CAM_H / self.INFER_H
        image_center = self.CAM_W // 2

        left_lane = None
        right_lane = None

        for i in range(len(lanes_points)):
            if i < len(lanes_detected) and lanes_detected[i] and len(lanes_points[i]) > 5:
                lane_scaled = [(int(x * sx), int(y * sy)) for x, y in lanes_points[i]]
                xs = [pt[0] for pt in lane_scaled]
                lane_mean_x = np.mean(xs)

                # Classify lane by its average horizontal position
                if lane_mean_x < image_center:
                    if left_lane is None or lane_mean_x > np.mean([p[0] for p in left_lane]):
                        left_lane = lane_scaled
                else:
                    if right_lane is None or lane_mean_x < np.mean([p[0] for p in right_lane]):
                        right_lane = lane_scaled

        if left_lane is None or right_lane is None:
            print("❌ Missing left or right lane → STOP")
            return "stop", None

        # Use bottom-most visible points
        def bottom_avg_x(lane, num_points=8):
            lane_sorted = sorted(lane, key=lambda p: p[1], reverse=True)
            bottom_pts = lane_sorted[:num_points]
            xs = [p[0] for p in bottom_pts]
            return int(np.mean(xs))

        lx = bottom_avg_x(left_lane)
        rx = bottom_avg_x(right_lane)

        lane_center = (lx + rx) // 2
        offset = lane_center - image_center

        # Smooth steering
        offset = int(0.6 * self.prev_offset + 0.4 * offset)
        self.prev_offset = offset

        print(f"Lane center: {lane_center}, Offset: {offset}")

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

        steer, offset = self.compute_steering(lanes_points, lanes_detected)

        # Debug overlay
        cv2.putText(raw_full, f"STEER: {steer}", (40, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)

        return steer, raw_full, overlay

    def cleanup(self):
        self.picam2.stop()
        cv2.destroyAllWindows()
