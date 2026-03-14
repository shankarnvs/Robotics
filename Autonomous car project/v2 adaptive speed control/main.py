# main.py
# Orchestrates perception and actuation

from perception import Perception
from actuation import Actuation

def main():
    perception = Perception()
    actuation = Actuation()

    while True:
        frame = get_frame_from_camera()

        result = perception.process(frame)
        steering = result["steering"]
        move_flag = result["move"]

        actuation.drive_forward(steering, move_flag)
        actuation.stall_guard()

def get_frame_from_camera():
    # TODO: integrate picamera2 frame capture
    return None

if __name__ == "__main__":
    main()
