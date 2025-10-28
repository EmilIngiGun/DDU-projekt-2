import cv2
from djitellopy import Tello

def main():
    # Connect to drone
    tello = Tello()
    tello.connect()

    print(f"Battery: {tello.get_battery()}%")

    # Start video stream
    tello.streamon()
    frame_read = tello.get_frame_read()

    while True:
        frame = frame_read.frame
        if frame is not None:
            cv2.imshow("Tello Video", frame)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    tello.streamoff()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
