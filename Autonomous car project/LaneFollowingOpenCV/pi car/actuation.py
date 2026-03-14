import RPi.GPIO as GPIO
import time
import threading


class Actuation:
    def __init__(self):
        self.lock = threading.Lock()

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Steering motor (M1)
        self.ENA = 17
        self.IN1 = 27
        self.IN2 = 22

        # Drive motor (M2)
        self.IN3 = 13
        self.IN4 = 19
        self.ENB = 26

        for pin in [self.ENA, self.IN1, self.IN2,
                    self.IN3, self.IN4, self.ENB]:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        # Steering motor always enabled at full power
        GPIO.output(self.ENA, GPIO.HIGH)

        # PWM for drive motor
        self.drive_pwm = GPIO.PWM(self.ENB, 1000)  # 1 kHz
        self.drive_pwm.start(0)

    # ---------------- STEERING ----------------
    def steer(self, direction):
        """
        direction: 'left', 'right', 'no steer'
        """
        with self.lock:
            if direction == "left":
                GPIO.output(self.IN1, GPIO.HIGH)
                GPIO.output(self.IN2, GPIO.LOW)

            elif direction == "right":
                GPIO.output(self.IN1, GPIO.LOW)
                GPIO.output(self.IN2, GPIO.HIGH)

            elif direction == "no steer":
                GPIO.output(self.IN1, GPIO.LOW)
                GPIO.output(self.IN2, GPIO.LOW)

            else:
                raise ValueError("Invalid steering direction")

    # ---------------- DRIVE ----------------
    def drive(self, direction, steer_state="no steer"):
        """
        direction: 'fwd', 'bck'
        steer_state: 'left', 'right', 'no steer'

        - 100% duty when steering
        - 25% duty when straight
        """

        with self.lock:
            # Direction control
            if direction == "fwd":
                GPIO.output(self.IN3, GPIO.HIGH)
                GPIO.output(self.IN4, GPIO.LOW)

            elif direction == "bck":
                GPIO.output(self.IN3, GPIO.LOW)
                GPIO.output(self.IN4, GPIO.HIGH)

            else:
                raise ValueError("Invalid drive direction")

            # Speed control logic
            if steer_state in ["left", "right"]:
                duty = 75
            else:
                duty = 60

            self.drive_pwm.ChangeDutyCycle(duty)

        # Run motor for 1 second
        time.sleep(1)

        # Stop motor
        with self.lock:
            self.drive_pwm.ChangeDutyCycle(0)
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.LOW)

    # ---------------- NAVIGATION ----------------
    def navigate(self, steer_dir, drive_dir):
        """
        Always:
        1. Steer
        2. Drive (speed depends on steer state)
        """
        self.steer(steer_dir)
        self.drive(drive_dir, steer_dir)

    # ---------------- CLEANUP ----------------
    def cleanup(self):
        self.drive_pwm.stop()
        GPIO.cleanup()
