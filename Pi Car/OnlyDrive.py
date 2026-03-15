import time
from actuation import Actuation


def main():
    act = Actuation()

    print("\n=== MANUAL ACTUATION DEBUG MODE ===")
    print("Commands:")
    print("  a  -> steer left + drive forward")
    print("  d  -> steer right + drive forward")
    print("  w  -> no steer + drive forward")
    print("  s  -> no steer + drive backward")
    print("  q  -> quit\n")

    try:
        while True:
            cmd = input("Enter command: ").strip().lower()

            if cmd == "a":
                print("LEFT + FORWARD")
                act.steer("left")
                act.drive("fwd", "left")

            elif cmd == "d":
                print("RIGHT + FORWARD")
                act.steer("right")
                act.drive("fwd", "right")

            elif cmd == "w":
                print("STRAIGHT + FORWARD")
                act.steer("no steer")
                act.drive("fwd", "no steer")

            elif cmd == "s":
                print("STRAIGHT + BACKWARD")
                act.steer("no steer")
                act.drive("bck", "no steer")

            elif cmd == "q":
                print("Exiting...")
                break

            else:
                print("Invalid command")

            time.sleep(3)

    except KeyboardInterrupt:
        print("\nInterrupted")

    finally:
        act.cleanup()
        print("GPIO cleaned up")


if __name__ == "__main__":
    main()