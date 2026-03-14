import cv2
from perception import Perception
from actuation import Actuation


def main():
    # Initialize modules
    perception = Perception(lane_color="WHITE", boundary_color="BLACK")
    act = Actuation()

    print("Auto lane-following started")
    print("Top: Raw frame | Bottom: Processed ROI")
    print("Press 'q' to quit")

    try:
        while True:
            steer, raw, processed = perception.process_frame()

            if raw is None or processed is None:
                continue

            # Resize raw frame for display
            raw_disp = cv2.resize(raw, (640, 480))

            # Overlay steering info
            cv2.putText(
                raw_disp,
                f"STEER: {steer}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2
            )

            # Stack raw (top) and processed ROI (bottom)
            combined = cv2.vconcat([raw_disp, processed])

            cv2.imshow("Lane Following Debug", combined)

            # === ACTUATION HOOK ===
            if steer in ("left", "right", "no steer"):
                if steer == "left":
                    steer = "right"
                elif steer == "right":
                    steer = "left"
                act.navigate(steer, "fwd")

            # Exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        print("Shutting down...")
        perception.cleanup()
        act.cleanup()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
