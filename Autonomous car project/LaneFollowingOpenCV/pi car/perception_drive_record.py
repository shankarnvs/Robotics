
import cv2
import numpy as np
from picamera2 import Picamera2
import time
import os

class Perception:
    def __init__(self):
        self.picam2 = Picamera2()

        config = self.picam2.create_preview_configuration(
            main={"size": (2592, 1944), "format": "RGB888"},
            controls={"FrameRate": 15}
        )

        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1)

        # Dataset folders
        self.base_dataset = "dataset"
        self.img_dir = os.path.join(self.base_dataset, "images")
        self.mask_dir = os.path.join(self.base_dataset, "masks")

        os.makedirs(self.img_dir, exist_ok=True)
        os.makedirs(self.mask_dir, exist_ok=True)

        self.deadband = 80

        print("Autodetect Drive + Record Mode Initialized")

    def glare_correction(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        h, s, v = cv2.split(hsv)

        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        v = clahe.apply(v)

        mask = cv2.inRange(v, 240, 255)
        v[mask > 0] = 200

        hsv = cv2.merge((h, s, v))
        corrected = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
        return corrected

    def generate_mask(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        # White candidate
        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 60, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        # Dark candidate
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 60])
        black_mask = cv2.inRange(hsv, lower_black, upper_black)

        combined = cv2.bitwise_or(white_mask, black_mask)

        kernel = np.ones((7,7), np.uint8)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)

        return combined

    def process_frame(self):
        frame = self.picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        corrected = self.glare_correction(frame)

        h, w, _ = corrected.shape

        full_mask = self.generate_mask(corrected)

        # ROI for steering (bottom 50%)
        roi = full_mask[int(h*0.5):h, :]

        contours, _ = cv2.findContours(
            roi,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        steer = "no steer"

        if contours:
            lane = max(contours, key=cv2.contourArea)
            M = cv2.moments(lane)

            if M["m00"] > 500:
                cx = int(M["m10"] / M["m00"])
                center = roi.shape[1] // 2
                offset = cx - center

                if offset > self.deadband:
                    steer = "right"
                elif offset < -self.deadband:
                    steer = "left"
                else:
                    steer = "no steer"

        # Save dataset
        timestamp = int(time.time() * 1000)

        img_path = os.path.join(self.img_dir, f"{timestamp}.jpg")
        mask_path = os.path.join(self.mask_dir, f"{timestamp}.png")

        cv2.imwrite(img_path, cv2.cvtColor(corrected, cv2.COLOR_RGB2BGR))
        cv2.imwrite(mask_path, full_mask)

        with open(os.path.join(self.base_dataset, "labels.csv"), "a") as f:
            f.write(f"{timestamp},{steer}\n")

        return steer, corrected

    def cleanup(self):
        self.picam2.stop()
