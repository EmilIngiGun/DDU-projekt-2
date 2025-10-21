# robust_tello_frontend.py
from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time
import serial
import threading
import sys

# Config
PORT = 'COM4'           # change to your Arduino port
BAUD_RATE = 115200
SPEED_VALUE = 60        # velocity magnitude for key presses (-100..100)
FPS = 60

# How many times to attempt to get frames from djitellopy before fallback
STREAM_RETRY = 4
# How many seconds to wait after calling streamon()
STREAM_START_WAIT = 2.0
# Fallback: try opening UDP stream directly with OpenCV
USE_UDP_FALLBACK = True

class FrontEnd:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([960, 720])

        self.tello = Tello()

        # velocities (Tello expects ints -100..100)
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10  # set speed for Tello set_speed()

        # whether we are actively sending rc control
        self.send_rc_control = False

        # timer event to call update at FPS
        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)

        # serial thread control
        self.serial_thread = None
        self.serial_running = False

        # if we use a direct cv2 VideoCapture fallback
        self.udp_capture = None
        self.using_udp_fallback = False

    def start_serial_thread(self):
        """Start serial thread to read Arduino button presses."""
        if self.serial_thread is not None and self.serial_thread.is_alive():
            return

        self.serial_running = True
        self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.serial_thread.start()

    def stop_serial_thread(self):
        self.serial_running = False
        if self.serial_thread is not None:
            self.serial_thread.join(timeout=0.5)

    def try_start_stream(self):
        """Try to start Tello stream and return a frame_read object or None."""
        # Attempt to ensure clean stream state
        try:
            self.tello.streamoff()
        except Exception:
            pass

        try:
            self.tello.streamon()
        except Exception as e:
            print("streamon() failed:", e)
            return None

        # wait a little for the drone to start streaming packets
        time.sleep(STREAM_START_WAIT)

        try:
            frame_read = self.tello.get_frame_read()
        except Exception as e:
            print("get_frame_read() raised:", e)
            return None

        # Try a few times to see if frames arrive
        for attempt in range(STREAM_RETRY):
            if getattr(frame_read, "frame", None) is not None and getattr(frame_read, "frame", None).size > 0:
                print("Frames received from djitellopy.")
                return frame_read
            print(f"Waiting for frames... attempt {attempt + 1}/{STREAM_RETRY}")
            time.sleep(0.5)
        print("No frames from djitellopy after retries.")
        return None

    def open_udp_fallback(self):
        """Try to open the UDP stream directly with OpenCV cv2.VideoCapture."""
        print("Attempting UDP fallback (cv2.VideoCapture)...")
        # If another program is using port 11111 this will fail.
        # On Windows the protocol string is: 'udp://0.0.0.0:11111'
        cap = cv2.VideoCapture("udp://0.0.0.0:11111", cv2.CAP_FFMPEG)
        if not cap.isOpened():
            print("UDP fallback: VideoCapture couldn't open udp://0.0.0.0:11111")
            return None
        # Try reading a few frames
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

    def run(self):
        """Main loop: connect to Tello, start video, start serial thread, handle events."""
        frame_read = None
        try:
            print("Connecting to Tello...")
            self.tello.connect()
            self.tello.set_speed(self.speed)

            # Try djitellopy stream first (robustly)
            frame_read = self.try_start_stream()

            # If djitellopy failed to provide frames, attempt UDP fallback
            if frame_read is None and USE_UDP_FALLBACK:
                # sometimes another streamoff/streamon helps — try a couple of times
                for attempt in range(2):
                    print(f"Stream attempt fallback loop {attempt + 1}/2")
                    frame_read = self.try_start_stream()
                    if frame_read is not None:
                        break
                if frame_read is None:
                    udp_cap = self.open_udp_fallback()
                    if udp_cap is None:
                        # final failure: provide actionable diagnostics then raise
                        print("\n=== VIDEO STREAM DIAGNOSTICS ===")
                        print("1) Ensure your PC is connected to the Tello Wi-Fi (SSID like 'TELLO-XXXXXX').")
                        print("2) Run: ping 192.168.10.1  (should respond).")
                        print("3) Disable firewall/antivirus briefly or allow Python/port 11111 UDP.")
                        print("4) Close apps that might use port 11111 (e.g. other VideoCapture/ffplay).")
                        print("5) Try `ffplay udp://0.0.0.0:11111` while connected to Tello Wi-Fi to check raw stream.")
                        print("6) Upgrade djitellopy: pip install -U djitellopy")
                        print("================================\n")
                        raise RuntimeError("Failed to grab video frames from video stream (both djitellopy and UDP fallback).")
            else:
                print("Using djitellopy frame_read.")

            # start serial reader thread (non-blocking)
            self.start_serial_thread()

            should_stop = False
            while not should_stop:
                for event in pygame.event.get():
                    if event.type == pygame.USEREVENT + 1:
                        # timer-based update
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

                # If using djitellopy frame_read:
                if not self.using_udp_fallback:
                    if frame_read.stopped:
                        print("Frame read stopped.")
                        break
                    frame = frame_read.frame
                else:
                    # using UDP fallback VideoCapture
                    if self.udp_capture is None:
                        print("UDP capture missing.")
                        break
                    ret, frame = self.udp_capture.read()
                    if not ret or frame is None or getattr(frame, "size", 0) == 0:
                        # show a black frame and continue; don't crash immediately
                        print("UDP capture read failed for one frame.")
                        frame = np.zeros((720, 960, 3), dtype=np.uint8)

                self.screen.fill([0, 0, 0])

                # show battery
                try:
                    battery = self.tello.get_battery()
                except Exception:
                    battery = "N/A"
                text = f"Battery: {battery}%"
                try:
                    cv2.putText(frame, text, (5, 720 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                except Exception:
                    # if frame shape doesn't match expected, skip text
                    pass

                # convert & display (only if frame has at least two dims)
                if frame is not None and getattr(frame, "ndim", 0) >= 2:
                    try:
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        frame = np.rot90(frame)
                        frame = np.flipud(frame)
                        surf = pygame.surfarray.make_surface(frame)
                        self.screen.blit(surf, (0, 0))
                    except Exception as e:
                        # if conversion fails just display a blank panel
                        print("Frame conversion/display error:", e)
                        self.screen.fill([0, 0, 0])
                pygame.display.update()

                time.sleep(1.0 / FPS)

        except Exception as e:
            print("Exception in run():", e)
            # do not sys.exit here so you can see the traceback in PyCharm
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
            # don't call sys.exit so PyCharm doesn't kill the console unexpectedly

    def keydown(self, key):
        """Handle key down (single arg)."""
        if key == pygame.K_UP:           # forward
            self.for_back_velocity = SPEED_VALUE
        elif key == pygame.K_DOWN:       # backward
            self.for_back_velocity = -SPEED_VALUE
        elif key == pygame.K_LEFT:       # left
            self.left_right_velocity = -SPEED_VALUE
        elif key == pygame.K_RIGHT:      # right
            self.left_right_velocity = SPEED_VALUE
        elif key == pygame.K_w:          # up
            self.up_down_velocity = SPEED_VALUE
        elif key == pygame.K_s:          # down
            self.up_down_velocity = -SPEED_VALUE
        elif key == pygame.K_a:          # yaw left
            self.yaw_velocity = -SPEED_VALUE
        elif key == pygame.K_d:          # yaw right
            self.yaw_velocity = SPEED_VALUE
        elif key == pygame.K_t:          # takeoff
            try:
                self.tello.takeoff()
                self.send_rc_control = True
            except Exception as e:
                print("Takeoff failed:", e)
        elif key == pygame.K_l:          # land
            try:
                self.tello.land()
            except Exception as e:
                print("Land failed:", e)
            self.send_rc_control = False

    def keyup(self, key):
        """Handle key release (single arg)."""
        if key in (pygame.K_UP, pygame.K_DOWN):
            self.for_back_velocity = 0
        elif key in (pygame.K_LEFT, pygame.K_RIGHT):
            self.left_right_velocity = 0
        elif key in (pygame.K_w, pygame.K_s):
            self.up_down_velocity = 0
        elif key in (pygame.K_a, pygame.K_d):
            self.yaw_velocity = 0

    def read_serial(self):
        """Read continuous sensor data from Arduino and detect press/release."""
        try:
            arduino = serial.Serial(PORT, BAUD_RATE, timeout=1)
            time.sleep(2)  # allow Arduino to reset
            print(f"✅ Connected to {PORT} at {BAUD_RATE} baud")
        except serial.SerialException as e:
            print("Could not open serial port:", e)
            return

        try:
            pressing = False
            threshold = 410  # adjust based on your Filtered_ADC range
            hysteresis = 10  # helps avoid jitter (so release happens below 400)
            last_value = 0

            while self.serial_running:
                try:
                    line = arduino.readline().decode('utf-8', errors='ignore').strip()
                except Exception:
                    line = ""

                if not line:
                    time.sleep(0.01)
                    continue

                # Example line:
                # Raw_ADC:423,Filtered_ADC_(1st-order-filter):413.16,movingThreshold_(2nd-order-filter):392.34,...
                if "Filtered_ADC" not in line:
                    continue

                try:
                    parts = line.split(',')
                    for p in parts:
                        if "Filtered_ADC" in p:
                            value = float(p.split(':')[1])
                            last_value = value
                            break
                    else:
                        continue
                except Exception:
                    continue

                # Print value for debugging
                print(f"[ADC] {last_value:.2f}")

                # === Button press logic ===
                if not pressing and last_value > threshold:
                    pressing = True
                    print("🔵 PRESS detected — move forward")
                    # Auto-takeoff if not flying
                    if not self.send_rc_control:
                        try:
                            self.tello.takeoff()
                            time.sleep(2)
                            self.send_rc_control = True
                        except Exception as e:
                            print("Auto takeoff failed:", e)
                    self.keydown(pygame.K_UP)

                elif pressing and last_value < (threshold - hysteresis):
                    pressing = False
                    print("⚪ RELEASE detected — stop")
                    self.keyup(pygame.K_UP)

        except Exception as e:
            print("Serial reader error:", e)
        finally:
            if arduino and arduino.is_open:
                arduino.close()
                print("Serial port closed.")

    def update(self):
        """Send velocities to Tello at regular intervals."""
        if self.send_rc_control:
            try:
                # djitellopy: send_rc_control(left_right, forward_back, up_down, yaw)
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
