"""Capture synchronized stereo image pairs of the checkerboard for calibration.
Run on the Jetson over SSH (headless, no live preview).
Move the checkerboard to a new position/angle, press Enter to save that pair, repeat.

IMPORTANT: the two cameras must stay physically fixed for the ENTIRE session (all
TARGET_PAIRS shots). If the rig gets bumped/repositioned partway through, the whole
batch is invalid and must be re-shot from scratch — only the checkerboard moves.

Each shot runs `gst-launch-1.0` as a fresh OS subprocess per camera (same recipe that
already worked cleanly for the earlier single-camera verification test). Holding a
live Argus session open inside this long-running Python process (via cv2.VideoCapture,
even opened/closed sequentially with delays) reliably broke the second camera with a
dmabuf error and eventually wedged the whole nvargus-daemon. A subprocess exits and
releases Argus completely each time, so there's nothing left to leak or race.
"""
import os
import shutil
import subprocess

SHOT_PLAN = [
    (5, "shots 0-4: CLOSE, ~20-30cm from the board"),
    (10, "shots 5-9: NORMAL distance, ~50-80cm"),
    (15, "shots 10-14: FAR, ~1.5-2m"),
    (20, "shots 15-19: mixed distance, but push the board to the EDGES/CORNERS of frame"),
    (26, "shots 20-25: mixed distance + heavy tilt angles (not facing straight-on)"),
]

OUTPUT_DIR = "stereo_pairs"
TARGET_PAIRS = 26


def capture_jpeg(sensor_id, path):
    cmd = (
        f"gst-launch-1.0 -e nvarguscamerasrc sensor-id={sensor_id} num-buffers=1 ! "
        "'video/x-raw(memory:NVMM),width=1280,height=720' ! "
        f"nvjpegenc ! filesink location={path}"
    )
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=15)
    if result.returncode != 0 or not os.path.exists(path):
        raise RuntimeError(f"Capture failed for sensor-id={sensor_id}: {result.stderr[-500:]}")


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

            left_path = f"{OUTPUT_DIR}/left_{count:02d}.jpg"
            right_path = f"{OUTPUT_DIR}/right_{count:02d}.jpg"
            try:
                capture_jpeg(0, left_path)
                capture_jpeg(1, right_path)
            except RuntimeError as e:
                print(f"{e}, try again.")
                continue

            print(f"Saved pair {count}")
            count += 1
    except KeyboardInterrupt:
        print("Stopped by user (Ctrl+C).")

    print(f"Done. {count} pairs saved in {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
