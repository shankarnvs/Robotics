import time

class FPSMonitor:
    def __init__(self, interval=2):
        self.interval = interval
        self.start = time.time()
        self.count = 0

    def tick(self):
        self.count += 1
        now = time.time()
        if now - self.start >= self.interval:
            print("FPS:", self.count / (now - self.start))
            self.count = 0
            self.start = now