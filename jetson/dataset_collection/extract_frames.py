"""Split a recorded video into individual JPEG frames for labeling.
Run wherever the video file is (Jetson or after scp'ing to laptop).
"""
import os
import cv2

VIDEO_PATH = "duck_raw.mp4"
OUTPUT_DIR = "duck_frames"
SAVE_EVERY_N_FRAMES = 15  # ~1 image every 0.5-0.75s at 20-30fps

def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open {VIDEO_PATH}")

    frame_idx = 0
    saved = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if frame_idx % SAVE_EVERY_N_FRAMES == 0:
            out_path = os.path.join(OUTPUT_DIR, f"duck_{saved:04d}.jpg")
            cv2.imwrite(out_path, frame)
            saved += 1
        frame_idx += 1

    cap.release()
    print(f"Saved {saved} frames to {OUTPUT_DIR}/")

if __name__ == "__main__":
    main()
