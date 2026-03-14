
import cv2
from perception_drive_record import Perception
from actuation import Actuation

def main():
    perception = Perception()
    act = Actuation()

    print("Autonomous Driving + Dataset Recording Started")
    print("Press 'q' to quit")

    try:
        while True:
            steer, frame = perception.process_frame()

            if frame is not None:
                disp = cv2.resize(frame, (640, 480))
                cv2.putText(disp, f"STEER: {steer}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (0, 0, 255), 2)
                cv2.imshow("Drive + Record", disp)

            if steer in ("left", "right", "no steer"):
                if steer == "left":
                    steer_cmd = "right"
                elif steer == "right":
                    steer_cmd = "left"
                else:
                    steer_cmd = "no steer"

                act.navigate(steer_cmd, "fwd")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        perception.cleanup()
        act.cleanup()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
