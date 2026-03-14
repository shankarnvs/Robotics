# perception.py
# Provides steering and move/not-move suggestion based on lane confidence

class Perception:
    def __init__(self, confidence_threshold=0.5):
        self.confidence_threshold = confidence_threshold

    def process(self, frame):
        # --- PLACEHOLDERS ---
        # Replace these with your existing lane detection pipeline
        steering = self.compute_steering(frame)
        confidence = self.compute_confidence(frame)

        move = confidence >= self.confidence_threshold

        return {
            "steering": steering,
            "move": move
        }

    def compute_steering(self, frame):
        # TODO: hook your existing lookahead / deadband logic here
        return 0.0

    def compute_confidence(self, frame):
        # TODO: return a value in [0,1] indicating lane confidence
        return 1.0
