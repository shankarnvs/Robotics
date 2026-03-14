# actuation.py
# Adaptive duty-cycle motor control logic

import time

class Actuation:
    def __init__(self):
        self.current_duty = 0.0
        self.min_moving_duty = None
        self.max_duty = 0.80

        self.increment = 0.02
        self.turn_boost = 0.05
        self.alpha = 0.2

        self.last_motion_time = time.time()
        self.stall_timeout = 1.0

    def drive_forward(self, steering, move_flag):
        if not move_flag:
            self.brake()
            return

        if self.min_moving_duty is None:
            # Learning minimum moving duty
            self.current_duty += self.increment
            if self.motion_observed():
                self.min_moving_duty = self.current_duty
        else:
            target = self.min_moving_duty
            if abs(steering) > 0.2:
                target += self.turn_boost

            self.current_duty += self.alpha * (target - self.current_duty)

        self.current_duty = max(
            min(self.current_duty, self.max_duty),
            self.min_moving_duty if self.min_moving_duty else 0.0
        )

        self.apply_pwm(self.current_duty, steering)

    def stall_guard(self):
        if time.time() - self.last_motion_time > self.stall_timeout:
            self.current_duty += self.increment

    def brake(self):
        self.stop_motor()
        time.sleep(0.03)
        self.reverse_motor(0.4)
        time.sleep(0.1)
        self.stop_motor()

    # --- Hardware abstraction layer (fill these in) ---

    def apply_pwm(self, duty, steering):
        # TODO: map duty + steering to L298D outputs
        pass

    def stop_motor(self):
        # TODO: stop motors
        pass

    def reverse_motor(self, duty):
        # TODO: short reverse pulse
        pass

    def motion_observed(self):
        # TODO: replace with encoder / optical flow / heuristic
        self.last_motion_time = time.time()
        return True
