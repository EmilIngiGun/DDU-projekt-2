from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time
import serial
import threading

# Serial setup (listen to arduino)
ser = serial.Serial('COM5', 9600, timeout=1)  # change COM4 if needed
time.sleep(2)  # allow Arduino reset
GOOO = False

def serial_listener():
    global bang_trigger, zoom_mode
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line == "yahoo":
                print("YUUUUUUUUUUUHHH")
        except:
            pass

threading.Thread(target=serial_listener, daemon=True).start()
