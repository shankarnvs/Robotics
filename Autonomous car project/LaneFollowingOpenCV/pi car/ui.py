import cv2
import threading
import time
import tkinter as tk
from PIL import Image, ImageTk

from perception import Perception
from actuation import Actuation


class RobotUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Autonomous RC Car UI")

        # State
        self.auto_mode = False
        self.running = True

        # Core modules
        self.perception = Perception("BLACK", "WHITE")
        self.actuation = Actuation()

        # Video writer
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.raw_writer = cv2.VideoWriter("raw.avi", fourcc, 20, (320, 240))
        self.proc_writer = cv2.VideoWriter("processed.avi", fourcc, 20, (320, 96))

        # UI Frames
        self._build_video_panel()
        self._build_control_panel()

        # Start video thread
        self.video_thread = threading.Thread(target=self.video_loop, daemon=True)
        self.video_thread.start()

        self.root.protocol("WM_DELETE_WINDOW", self.shutdown)

    # ---------------- UI BUILD ----------------
    def _build_video_panel(self):
        frame = tk.Frame(self.root)
        frame.pack()

        self.raw_label = tk.Label(frame)
        self.raw_label.pack(side=tk.LEFT)

        self.proc_label = tk.Label(frame)
        self.proc_label.pack(side=tk.RIGHT)

    def _build_control_panel(self):
        frame = tk.Frame(self.root)
        frame.pack(pady=10)

        # Buttons arranged as inclined square
        self.btn_up = tk.Button(frame, text="UP", width=8,
                                 command=lambda: self.manual_drive("no steer", "fwd"))
        self.btn_up.grid(row=0, column=1)

        self.btn_left = tk.Button(frame, text="LEFT", width=8,
                                   command=lambda: self.manual_drive("left", "fwd"))
        self.btn_left.grid(row=1, column=0)

        self.btn_auto = tk.Button(frame, text="MANUAL", width=10,
                                   bg="red", fg="white", command=self.toggle_mode)
        self.btn_auto.grid(row=1, column=1)

        self.btn_right = tk.Button(frame, text="RIGHT", width=8,
                                    command=lambda: self.manual_drive("right", "fwd"))
        self.btn_right.grid(row=1, column=2)

        self.btn_down = tk.Button(frame, text="DOWN", width=8,
                                   command=lambda: self.manual_drive("no steer", "bck"))
        self.btn_down.grid(row=2, column=1)

    # ---------------- MODE TOGGLE ----------------
    def toggle_mode(self):
        self.auto_mode = not self.auto_mode
        if self.auto_mode:
            self.btn_auto.config(text="AUTO", bg="green")
        else:
            self.btn_auto.config(text="MANUAL", bg="red")

    # ---------------- MANUAL DRIVE ----------------
    def manual_drive(self, steer, drive):
        if not self.auto_mode:
            threading.Thread(
                target=self.actuation.navigate,
                args=(steer, drive),
                daemon=True
            ).start()

    # ---------------- VIDEO LOOP ----------------
    def video_loop(self):
        while self.running:
            steer, raw, processed = self.perception.process_frame()
            if raw is None:
                continue

            # Save video
            self.raw_writer.write(raw)
            self.proc_writer.write(processed)

            # Auto mode actuation
            if self.auto_mode and steer:
                threading.Thread(
                    target=self.actuation.navigate,
                    args=(steer, "fwd"),
                    daemon=True
                ).start()

            # Update UI
            self._update_video(self.raw_label, raw)
            self._update_video(self.proc_label, processed)

            # Flush frame buffer → fresh frame every cycle
            cv2.waitKey(1)

    # ---------------- VIDEO UPDATE ----------------
    def _update_video(self, label, frame):
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(img)
        imgtk = ImageTk.PhotoImage(image=img)
        label.imgtk = imgtk
        label.config(image=imgtk)

    # ---------------- SHUTDOWN ----------------
    def shutdown(self):
        self.running = False
        time.sleep(0.5)

        self.perception.cleanup()
        self.actuation.cleanup()
        self.raw_writer.release()
        self.proc_writer.release()

        self.root.destroy()

    def run(self):
        self.root.mainloop()


# ---------------- ENTRY POINT ----------------
if __name__ == "__main__":
    ui = RobotUI()
    ui.run()
