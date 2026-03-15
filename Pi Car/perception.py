import cv2
import numpy as np
from picamera2 import Picamera2
import time


class Perception:
    def __init__(self, lane_color="WHITE", boundary_color="BLACK"):
        self.lane_color = lane_color
        self.boundary_color = boundary_color

        # ✅ CREATE picam2 ATTRIBUTE (THIS WAS MISSING)
        self.picam2 = Picamera2()

        # Stable, wide-FOV preview configuration
        config = self.picam2.create_preview_configuration(
            main={"size": (2592, 1944), "format": "RGB888"},
            controls={"FrameRate": 15}
        )

        self.picam2.configure(config)
        self.picam2.start()

        # Allow camera to warm up
        time.sleep(1)

        print("Picamera2 initialized")

    def process_frame(self):
        try:
            frame = self.picam2.capture_array()
        except Exception as e:
            print("Picamera2 capture failed:", e)
            return None, None, None

        raw_full = frame.copy()
        h, w, _ = frame.shape

        # 1️⃣ Rotate 180 degrees
        rotated = cv2.rotate(frame, cv2.ROTATE_180)

        # 2️⃣ Bottom 60% ROI
        roi = rotated[int(0.4 * h):h, :]

        if roi.size == 0:
            return None, raw_full, None

        # 3️⃣ Resize ROI to 640x480
        roi_resized = cv2.resize(roi, (640, 480))

        # 4️⃣ Grayscale
        gray = cv2.cvtColor(roi_resized, cv2.COLOR_RGB2GRAY)

        # 5️⃣ Threshold
        thresh_type = (
            cv2.THRESH_BINARY
            if self.lane_color == "WHITE"
            else cv2.THRESH_BINARY_INV
        )

        _, binary = cv2.threshold(gray, 120, 255, thresh_type)

        # 6️⃣ Morphology
        kernel = np.ones((5, 5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        # ================= FIX 2 =================
        # Use LOOK-AHEAD slice instead of full ROI
        roi_h = binary.shape[0]
        lookahead = binary[int(0.6 * roi_h):int(0.8 * roi_h), :]

        contours, _ = cv2.findContours(
            lookahead,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        steer = "no steer"

        if contours:
            lane = max(contours, key=cv2.contourArea)
            M = cv2.moments(lane)

            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                center = lookahead.shape[1] // 2
                offset = cx - center

                # ================= FIX 1 =================
                # Deadband to prevent early steering
                DEADBAND = 60

                if offset > DEADBAND:
                    steer = "right"
                elif offset < -DEADBAND:
                    steer = "left"
                else:
                    steer = "no steer"

                # Visual debug
                cv2.circle(
                    lookahead,
                    (cx, lookahead.shape[0] // 2),
                    6,
                    128,
                    -1
                )
                cv2.line(
                    lookahead,
                    (center, 0),
                    (center, lookahead.shape[0]),
                    128,
                    2
                )

        processed = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

        return steer, raw_full, processed

    def cleanup(self):
        self.picam2.stop()
        cv2.destroyAllWindows()
