"""Capture synchronized stereo image pairs of the checkerboard for calibration.
Run on the Jetson over SSH (headless, no live preview).
Move the checkerboard to a new position/angle, press Enter to save that pair, repeat.

IMPORTANT: the two cameras must stay physically fixed for the ENTIRE session (all
TARGET_PAIRS shots). If the rig gets bumped/repositioned partway through, the whole
batch is invalid and must be re-shot from scratch — only the checkerboard moves.

Each shot launches both cameras' gst-launch-1.0 processes CONCURRENTLY (Popen for
both, then wait on both) rather than one after another. Sequential single-camera
sessions (whether via cv2.VideoCapture or via subprocess, with or without a settle
delay) reliably broke on the second session with an Argus "Correctable Error Status".
Simultaneous dual-camera capture was already independently verified working on this
Jetson earlier — this matches that proven-working pattern, and as a bonus gives
better time-synced pairs than sequential capture ever did.
"""
import os
import shutil
import subprocess
import time

SHOT_PLAN = [
    (5, "shots 0-4: CLOSE, ~20-30cm from the board"),
    (10, "shots 5-9: NORMAL distance, ~50-80cm"),
    (15, "shots 10-14: FAR, ~1.5-2m"),
    (20, "shots 15-19: mixed distance, but push the board to the EDGES/CORNERS of frame"),
    (26, "shots 20-25: mixed distance + heavy tilt angles (not facing straight-on)"),
]

OUTPUT_DIR = "stereo_pairs"
TARGET_PAIRS = 26


def gst_cmd(sensor_id, path):
    return (
        f"gst-launch-1.0 -e nvarguscamerasrc sensor-id={sensor_id} num-buffers=1 ! "
        "'video/x-raw(memory:NVMM),width=1280,height=720' ! "
        f"nvjpegenc ! filesink location={path}"
    )


STAGGER_SEC = 0.5  # small human-like gap between starting each camera process


def _valid(path):
    return os.path.exists(path) and os.path.getsize(path) > 0


def capture_pair(left_path, right_path):
    proc_l = subprocess.Popen(gst_cmd(0, left_path), shell=True,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(STAGGER_SEC)
    proc_r = subprocess.Popen(gst_cmd(1, right_path), shell=True,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    out_l, _ = proc_l.communicate(timeout=15)
    out_r, _ = proc_r.communicate(timeout=15)

    # gst-launch can exit non-zero on a benign teardown-time Argus error even after
    # the frame was captured fine (EOS already reached) -- trust the file, not the exit code.
    if not _valid(left_path):
        raise RuntimeError(f"Left (sensor-id=0) capture failed:\n{out_l[-500:]}")
    if not _valid(right_path):
        raise RuntimeError(f"Right (sensor-id=1) capture failed:\n{out_r[-500:]}")


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
                capture_pair(left_path, right_path)
            except RuntimeError as e:
                print(f"{e}\ntry again.")
                continue

            print(f"Saved pair {count}")
            count += 1
    except KeyboardInterrupt:
        print("Stopped by user (Ctrl+C).")

    print(f"Done. {count} pairs saved in {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
