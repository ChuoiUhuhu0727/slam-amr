"""Capture synchronized stereo image pairs of the checkerboard for calibration.
Run on the Jetson over SSH (headless, no live preview).
Move the checkerboard to a new position/angle, press Enter to save that pair, repeat.
Aim for TARGET_PAIRS shots: vary distance, angle/tilt, and position (corners too, not just center).
"""
import cv2

CSI_PIPELINE_LEFT = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
    "nvvidconv ! "
    "video/x-raw, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! appsink drop=1"
)
CSI_PIPELINE_RIGHT = (
    "nvarguscamerasrc sensor-id=1 ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
    "nvvidconv ! "
    "video/x-raw, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! appsink drop=1"
)

OUTPUT_DIR = "stereo_pairs"
TARGET_PAIRS = 20

def main():
    import os
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    cap_left = cv2.VideoCapture(CSI_PIPELINE_LEFT, cv2.CAP_GSTREAMER)
    cap_right = cv2.VideoCapture(CSI_PIPELINE_RIGHT, cv2.CAP_GSTREAMER)
    if not cap_left.isOpened() or not cap_right.isOpened():
        raise RuntimeError("Could not open one or both CSI cameras. Check sensor-id / pipeline.")

    count = 0
    print(f"Target: {TARGET_PAIRS} pairs. Move the checkerboard, then press Enter to capture (or 'q' + Enter to stop).")
    try:
        while count < TARGET_PAIRS:
            cmd = input(f"[{count}/{TARGET_PAIRS}] Enter to capture, q to quit: ")
            if cmd.strip().lower() == "q":
                break

            ok_l, frame_l = cap_left.read()
            ok_r, frame_r = cap_right.read()
            if not ok_l or not ok_r:
                print("Frame grab failed, try again.")
                continue

            cv2.imwrite(f"{OUTPUT_DIR}/left_{count:02d}.jpg", frame_l)
            cv2.imwrite(f"{OUTPUT_DIR}/right_{count:02d}.jpg", frame_r)
            print(f"Saved pair {count}")
            count += 1
    except KeyboardInterrupt:
        print("Stopped by user (Ctrl+C).")

    cap_left.release()
    cap_right.release()
    print(f"Done. {count} pairs saved in {OUTPUT_DIR}/")

if __name__ == "__main__":
    main()
