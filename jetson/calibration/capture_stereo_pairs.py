"""Capture synchronized stereo image pairs of the checkerboard for calibration.
Run on the Jetson over SSH (headless, no live preview).
Move the checkerboard to a new position/angle, press Enter to save that pair, repeat.

IMPORTANT: the two cameras must stay physically fixed for the ENTIRE session (all
TARGET_PAIRS shots). If the rig gets bumped/repositioned partway through, the whole
batch is invalid and must be re-shot from scratch — only the checkerboard moves.

Each shot opens, grabs, and closes ONE camera pipeline at a time (never both open at
once) — running two live nvarguscamerasrc sessions concurrently in one process was
crashing with an Argus dmabuf/segfault error. The board is held still while pressing
Enter, so a ~1-2s open/close gap between the two grabs doesn't hurt sync.
"""
import os
import shutil
import time

import cv2

# Guided shot plan: (shot index cutoff, instruction). Printed as a reminder at each cutoff.
SHOT_PLAN = [
    (5, "shots 0-4: CLOSE, ~20-30cm from the board"),
    (10, "shots 5-9: NORMAL distance, ~50-80cm"),
    (15, "shots 10-14: FAR, ~1.5-2m"),
    (20, "shots 15-19: mixed distance, but push the board to the EDGES/CORNERS of frame"),
    (26, "shots 20-25: mixed distance + heavy tilt angles (not facing straight-on)"),
]

OUTPUT_DIR = "stereo_pairs"
TARGET_PAIRS = 26
WARMUP_FRAMES = 3  # first frame(s) after opening nvarguscamerasrc are often stale
SETTLE_SEC = 2.0   # Argus needs a moment to release a session's buffers before the next opens


def pipeline_for(sensor_id):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
        "nvvidconv ! "
        "video/x-raw, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! appsink drop=1"
    )


def grab_one(sensor_id):
    cap = cv2.VideoCapture(pipeline_for(sensor_id), cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera sensor-id={sensor_id}")
    frame = None
    for _ in range(WARMUP_FRAMES):
        ok, frame = cap.read()
        if not ok:
            cap.release()
            raise RuntimeError(f"Frame grab failed for sensor-id={sensor_id}")
    cap.release()
    time.sleep(SETTLE_SEC)
    return frame


def main():
    if os.path.isdir(OUTPUT_DIR):
        shutil.rmtree(OUTPUT_DIR)  # wipe any previous (now-invalid) batch
    os.makedirs(OUTPUT_DIR)

    count = 0
    print(f"Target: {TARGET_PAIRS} pairs. Cameras must NOT move for the whole session — only the board moves.")
    try:
        while count < TARGET_PAIRS:
            for cutoff, note in SHOT_PLAN:
                if count < cutoff:
                    print(f"  >> {note}")
                    break
            cmd = input(f"[{count}/{TARGET_PAIRS}] Enter to capture, q to quit: ")
            if cmd.strip().lower() == "q":
                break

            try:
                frame_l = grab_one(0)
                frame_r = grab_one(1)
            except RuntimeError as e:
                print(f"{e}, try again.")
                continue

            cv2.imwrite(f"{OUTPUT_DIR}/left_{count:02d}.jpg", frame_l)
            cv2.imwrite(f"{OUTPUT_DIR}/right_{count:02d}.jpg", frame_r)
            print(f"Saved pair {count}")
            count += 1
    except KeyboardInterrupt:
        print("Stopped by user (Ctrl+C).")

    print(f"Done. {count} pairs saved in {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
