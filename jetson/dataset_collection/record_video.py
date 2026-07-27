"""Record raw CSI video for dataset collection (no model, just capture).
Move the object freely in front of the camera while this runs.
Run on the Jetson. Stops after DURATION_SEC or Ctrl+C.
"""
import time
import cv2

CSI_PIPELINE = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
    "nvvidconv ! "
    "video/x-raw, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! appsink drop=1"
)

OUTPUT_PATH = "duck_raw.mp4"
DURATION_SEC = 90

def main():
    cap = cv2.VideoCapture(CSI_PIPELINE, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("Could not open CSI camera.")

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = None

    print(f"Recording for {DURATION_SEC}s - move the object around now...")
    start = time.time()
    try:
        while time.time() - start < DURATION_SEC:
            ok, frame = cap.read()
            if not ok:
                print("Frame grab failed, stopping.")
                break
            if writer is None:
                h, w = frame.shape[:2]
                writer = cv2.VideoWriter(OUTPUT_PATH, fourcc, 20.0, (w, h))
            writer.write(frame)
    except KeyboardInterrupt:
        print("Stopped by user (Ctrl+C).")

    cap.release()
    if writer is not None:
        writer.release()
    print(f"Saved {OUTPUT_PATH}")

if __name__ == "__main__":
    main()
