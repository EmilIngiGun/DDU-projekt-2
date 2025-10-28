# robust_tello_frontend_fixed.py
from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time
import serial
import threading
import sys

# ===== CONFIG =====
PORT = 'COM6'           # your Arduino port
BAUD_RATE = 115200
SPEED_VALUE = 60        # Tello velocity (-100..100)
FPS = 60

STREAM_RETRY = 4
STREAM_START_WAIT = 2.0
USE_UDP_FALLBACK = True
# ===================

# add near top of file, with other config
AUTO_TAKEOFF = False    # safe default: require manual 'T' to take off
CALIBRATE_SECONDS = 3   # how long to collect "idle" samples on startup


class FrontEnd:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([960, 720])

        self.tello = Tello()

        # velocities
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10

        # state flags
        self.send_rc_control = False
        self.serial_thread = None
        self.serial_running = False
        self.udp_capture = None
        self.using_udp_fallback = False

        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)

    # ---------------- STREAM SETUP ---------------- #
    def try_start_stream(self):
        try:
            self.tello.streamoff()
        except Exception:
            pass
        try:
            self.tello.streamon()
        except Exception as e:
            print("streamon() failed:", e)
            return None

        time.sleep(STREAM_START_WAIT)
        try:
            frame_read = self.tello.get_frame_read()
        except Exception as e:
            print("get_frame_read() raised:", e)
            return None

        for attempt in range(STREAM_RETRY):
            if getattr(frame_read, "frame", None) is not None and getattr(frame_read, "frame", None).size > 0:
                print("Frames received from djitellopy.")
                return frame_read
            print(f"Waiting for frames... attempt {attempt + 1}/{STREAM_RETRY}")
            time.sleep(0.5)
        print("No frames from djitellopy after retries.")
        return None

    def open_udp_fallback(self):
        print("Attempting UDP fallback (cv2.VideoCapture)...")
        cap = cv2.VideoCapture("udp://0.0.0.0:11111", cv2.CAP_FFMPEG)
        if not cap.isOpened():
            print("UDP fallback: VideoCapture couldn't open udp://0.0.0.0:11111")
            return None
        for i in range(30):
            ret, frame = cap.read()
            if ret and frame is not None and getattr(frame, "size", 0) > 0:
                print("UDP fallback: frames received from VideoCapture.")
                self.using_udp_fallback = True
                self.udp_capture = cap
                return cap
            time.sleep(0.1)
        print("UDP fallback: no frames read.")
        cap.release()
        return None

    # ---------------- SERIAL THREAD ---------------- #
    def start_serial_thread(self):
        if self.serial_thread is not None and self.serial_thread.is_alive():
            return
        self.serial_running = True
        self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.serial_thread.start()

    def stop_serial_thread(self):
        self.serial_running = False
        if self.serial_thread is not None:
            self.serial_thread.join(timeout=0.5)

    # ---------------- MAIN LOOP ---------------- #
    def run(self):
        frame_read = None
        try:
            print("Connecting to Tello...")
            self.tello.connect()
            self.tello.set_speed(self.speed)

            frame_read = self.try_start_stream()
            if frame_read is None and USE_UDP_FALLBACK:
                for attempt in range(2):
                    print(f"Stream attempt fallback loop {attempt + 1}/2")
                    frame_read = self.try_start_stream()
                    if frame_read is not None:
                        break
                if frame_read is None:
                    udp_cap = self.open_udp_fallback()
                    if udp_cap is None:
                        print("Video stream failed entirely.")
                        raise RuntimeError("Failed to get video frames.")
            else:
                print("Using djitellopy frame_read.")

            self.start_serial_thread()
            should_stop = False

            while not should_stop:
                for event in pygame.event.get():
                    if event.type == pygame.USEREVENT + 1:
                        self.update()
                    elif event.type == pygame.QUIT:
                        should_stop = True
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            should_stop = True
                        else:
                            self.keydown(event.key)
                    elif event.type == pygame.KEYUP:
                        self.keyup(event.key)

                if not self.using_udp_fallback:
                    if frame_read.stopped:
                        print("Frame read stopped.")
                        break
                    frame = frame_read.frame
                else:
                    if self.udp_capture is None:
                        break
                    ret, frame = self.udp_capture.read()
                    if not ret or frame is None:
                        frame = np.zeros((720, 960, 3), dtype=np.uint8)

                # Display battery level
                try:
                    battery = self.tello.get_battery()
                except Exception:
                    battery = "N/A"
                text = f"Battery: {battery}%"
                try:
                    cv2.putText(frame, text, (5, 720 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                except Exception:
                    pass

                # Display frame
                if frame is not None and getattr(frame, "ndim", 0) >= 2:
                    try:
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        frame = np.rot90(frame)
                        frame = np.flipud(frame)
                        surf = pygame.surfarray.make_surface(frame)
                        self.screen.blit(surf, (0, 0))
                    except Exception:
                        self.screen.fill([0, 0, 0])
                pygame.display.update()
                time.sleep(1.0 / FPS)

        except Exception as e:
            print("Exception in run():", e)
        finally:
            print("Shutting down...")
            self.stop_serial_thread()
            try:
                self.tello.streamoff()
            except Exception:
                pass
            try:
                self.tello.end()
            except Exception:
                pass
            if self.udp_capture is not None:
                try:
                    self.udp_capture.release()
                except Exception:
                    pass
            pygame.quit()

    # ---------------- KEYBOARD HANDLING ---------------- #
    def keydown(self, key):
        if key == pygame.K_UP:
            self.for_back_velocity = SPEED_VALUE
        elif key == pygame.K_DOWN:
            self.for_back_velocity = -SPEED_VALUE
        elif key == pygame.K_LEFT:
            self.left_right_velocity = -SPEED_VALUE
        elif key == pygame.K_RIGHT:
            self.left_right_velocity = SPEED_VALUE
        elif key == pygame.K_w:
            self.up_down_velocity = SPEED_VALUE
        elif key == pygame.K_s:
            self.up_down_velocity = -SPEED_VALUE
        elif key == pygame.K_a:
            self.yaw_velocity = -SPEED_VALUE
        elif key == pygame.K_d:
            self.yaw_velocity = SPEED_VALUE
        elif key == pygame.K_t:
            try:
                self.tello.takeoff()
                self.send_rc_control = True
            except Exception as e:
                print("Takeoff failed:", e)
        elif key == pygame.K_l:
            try:
                self.tello.land()
            except Exception as e:
                print("Land failed:", e)
            self.send_rc_control = False

    def keyup(self, key):
        if key in (pygame.K_UP, pygame.K_DOWN):
            self.for_back_velocity = 0
        elif key in (pygame.K_LEFT, pygame.K_RIGHT):
            self.left_right_velocity = 0
        elif key in (pygame.K_w, pygame.K_s):
            self.up_down_velocity = 0
        elif key in (pygame.K_a, pygame.K_d):
            self.yaw_velocity = 0

    # ---------------- SERIAL INPUT ---------------- #
    def read_serial(self):
        """
        Robust serial reader with startup calibration and safe auto-takeoff toggle.

        Behavior:
         - Collects CALIBRATE_SECONDS of samples at start to establish baseline.
         - Waits until the rolling window is full before making decisions.
         - Uses adaptive baseline (slowly drifts) but only after calibration.
         - Will NOT auto-takeoff unless AUTO_TAKEOFF is True (safe default).
        """
        import statistics
        try:
            arduino = serial.Serial(PORT, BAUD_RATE, timeout=1)
            time.sleep(2)
            print(f"✅ Connected to {PORT} at {BAUD_RATE} baud")
        except serial.SerialException as e:
            print("Could not open serial port:", e)
            return

        try:
            # Parameters to tune
            window_size = 30  # samples for rolling average (increase to smooth more)
            offset = 20  # how far above baseline counts as a press
            hysteresis = 12  # avoid flicker on threshold edges
            adapt_rate = 0.995  # baseline = baseline*adapt_rate + avg*(1-adapt_rate)
            calibration_time = CALIBRATE_SECONDS  # seconds to sample baseline at start

            values = []
            baseline = None
            pressing = False
            last_value = None

            # ---------------- Startup calibration ----------------
            print(f"📏 Calibrating baseline for {calibration_time} s. Keep the plate untouched.")
            calib_deadline = time.time() + calibration_time
            while time.time() < calib_deadline:
                line = arduino.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    time.sleep(0.01)
                    continue
                if "Filtered_ADC" not in line:
                    continue
                try:
                    for p in line.split(','):
                        if "Filtered_ADC" in p:
                            v = float(p.split(':')[1])
                            values.append(v)
                            break
                except Exception:
                    continue
                # keep sampling quickly
            if len(values) == 0:
                print("⚠️ Calibration failed: no samples. Falling back to 200.")
                baseline = 200.0
                values = [baseline] * window_size
            else:
                # initialize rolling window with recent samples, and baseline to mean
                # pad with the mean so avg is stable immediately
                mean_cal = float(sum(values) / len(values))
                values = [mean_cal] * min(window_size, len(values))  # start with stable window
                baseline = mean_cal
            print(
                f"📏 Calibration done. baseline={baseline:.2f}. Starting live read (offset={offset}, hyst={hysteresis}).")

            # ---------------- Continuous loop ----------------
            while self.serial_running:
                try:
                    line = arduino.readline().decode('utf-8', errors='ignore').strip()
                except Exception:
                    line = ""
                if not line:
                    time.sleep(0.01)
                    continue
                if "Filtered_ADC" not in line:
                    continue

                try:
                    for p in line.split(','):
                        if "Filtered_ADC" in p:
                            last_value = float(p.split(':')[1])
                            break
                    else:
                        continue
                except Exception:
                    continue

                # update rolling window
                values.append(last_value)
                if len(values) > window_size:
                    values.pop(0)
                if len(values) < window_size:
                    # wait until window is full for reliable avg
                    avg_value = float(sum(values) / len(values))
                else:
                    avg_value = float(sum(values) / len(values))

                # slowly adapt baseline toward avg_value (very slow)
                baseline = baseline * adapt_rate + avg_value * (1.0 - adapt_rate)
                threshold = baseline + offset

                # debug line (keeps console readable)
                print(
                    f"[ADC] raw:{last_value:.1f} avg:{avg_value:.1f} base:{baseline:.1f} thr:{threshold:.1f} press={pressing}")

                # Decision: require window full to change pressing state (optional safety)
                if len(values) >= window_size:
                    if not pressing and avg_value > threshold:
                        pressing = True
                        print("🔵 PRESS detected — will move forward")
                        # Only auto-takeoff if enabled; otherwise rely on manual T key
                        if not self.send_rc_control and AUTO_TAKEOFF:
                            try:
                                print("Taking off (AUTO_TAKEOFF enabled)...")
                                self.tello.takeoff()
                                time.sleep(2)
                                self.send_rc_control = True
                            except Exception as e:
                                print("Auto takeoff failed:", e)
                        # set velocity variable (actual send happens in update())
                        self.for_back_velocity = SPEED_VALUE

                    elif pressing and avg_value < (threshold - hysteresis):
                        pressing = False
                        print("⚪ RELEASE detected — stop")
                        self.for_back_velocity = 0

            # loop end
        except Exception as e:
            print("Serial reader error:", e)
        finally:
            if arduino and arduino.is_open:
                arduino.close()
                print("Serial port closed.")

    # ---------------- UPDATE LOOP ---------------- #
    def update(self):
        if self.send_rc_control:
            try:
                self.tello.send_rc_control(
                    int(self.left_right_velocity),
                    int(self.for_back_velocity),
                    int(self.up_down_velocity),
                    int(self.yaw_velocity)
                )
            except Exception as e:
                print("Failed to send rc control:", e)


def main():
    frontend = FrontEnd()
    frontend.run()


if __name__ == '__main__':
    main()
