"""One-off diagnostic: prove (with real numbers, not just theory) that
feeding the duck model a rectified frame instead of a raw frame hurts
detection -- see the 2026-08-25 search_and_rescue.py fix this is checking.

Grabs ONE frame from the left camera, builds both a raw version (what the
model was trained on) and a rectified version (what the bug fed it), runs
the model on both, and prints a side-by-side comparison. Also saves both
images to disk so you can eyeball the visual difference yourself.

Run on the Jetson (needs the ROS2 env NOT required -- this is standalone):
    python3 jetson/tools/compare_raw_vs_rectified.py
"""
import time
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

LEFT_SENSOR_ID = 1  # must match search_and_rescue.py's LEFT_SENSOR_ID
WEIGHTS_PATH = Path(__file__).resolve().parents[1] / "training/runs/detect/train-4/weights/best.pt"
CALIB_PATH = Path(__file__).resolve().parents[2] / "stereo_calibration.npz"
CONF_THRESHOLD = 0.1  # deliberately low -- we want to SEE a weak detection, not hide it


def csi_pipeline(sensor_id: int) -> str:
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
        "nvvidconv ! "
        "video/x-raw, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! appsink drop=1"
    )


def best_box(model, frame):
    results = model(frame, conf=CONF_THRESHOLD, verbose=False)
    boxes = results[0].boxes
    if boxes is None or len(boxes) == 0:
        return None
    box = boxes[boxes.conf.argmax()]
    return box.xyxy[0].tolist(), float(box.conf[0])


def main():
    calib = np.load(str(CALIB_PATH))
    K1, D1, K2, D2, R, T = calib["K1"], calib["D1"], calib["K2"], calib["D2"], calib["R"], calib["T"]
    size = (int(calib["image_size"][0]), int(calib["image_size"][1]))

    # Same stereo rectification search_and_rescue.py builds for the left
    # camera -- this IS the transform the bug applied before detection.
    R1, _, P1, _, _, _, _ = cv2.stereoRectify(
        K1, D1, K2, D2, size, R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
    )
    map_x, map_y = cv2.initUndistortRectifyMap(K1, D1, R1, P1, size, cv2.CV_32FC1)

    print(f"Loading model from {WEIGHTS_PATH}...")
    model = YOLO(str(WEIGHTS_PATH))

    print("Opening left camera...")
    cap = cv2.VideoCapture(csi_pipeline(LEFT_SENSOR_ID), cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("Could not open left CSI camera")

    for _ in range(10):  # first frames off a freshly opened Argus pipeline are often junk
        cap.read()
        time.sleep(0.05)

    input("Hold the duck in view of the LEFT camera, then press Enter to capture...")
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError("Frame grab failed")

    # Same one captured frame feeds BOTH versions below -- same duck, same
    # lighting, same instant. Only the processing differs, which is exactly
    # what makes this a fair, controlled comparison.
    frame_raw = cv2.rotate(frame, cv2.ROTATE_180)  # undo the physical 180deg mount roll
    frame_rectified = cv2.remap(frame_raw, map_x, map_y, cv2.INTER_LINEAR)

    print()
    for label, img, path in [
        ("RAW        (matches training)          ", frame_raw, "compare_raw.jpg"),
        ("RECTIFIED  (what the bug fed the model) ", frame_rectified, "compare_rectified.jpg"),
    ]:
        result = best_box(model, img)
        cv2.imwrite(path, img)
        if result is None:
            print(f"{label}: NO detection above conf={CONF_THRESHOLD}   (saved {path})")
        else:
            (x1, y1, x2, y2), conf = result
            print(f"{label}: conf={conf:.3f}  box=({x1:.0f},{y1:.0f})-({x2:.0f},{y2:.0f})   (saved {path})")

    print("\nOpen compare_raw.jpg and compare_rectified.jpg side by side for the visual check too.")


if __name__ == "__main__":
    main()
