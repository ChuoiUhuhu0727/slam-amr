"""Stereo calibration from captured checkerboard pairs (jetson/calibration/capture_stereo_pairs.py).
Run on the Jetson after stereo_pairs/ has ~20 left/right image pairs.

What this computes:
- Per-camera intrinsics (K, distortion) — how each lens maps 3D points to pixels.
- Extrinsics between the two cameras (R, T) — T's length is the real baseline in meters,
  should come out close to the measured ~0.083m mount baseline as a sanity check.
Output: stereo_calibration.npz, loadable via np.load() for the isaac_ros_visual_slam camera_info setup.
"""
import glob

import cv2
import numpy as np

INTERNAL_CORNERS = (9, 6)  # (cols, rows) of internal corners, matches the printed 10x7-square board
SQUARE_SIZE_M = 0.025       # 25mm, measured after printing

PAIRS_DIR = "stereo_pairs"
OUTPUT_PATH = "stereo_calibration.npz"

def find_corners(img_paths):
    objp = np.zeros((INTERNAL_CORNERS[0] * INTERNAL_CORNERS[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:INTERNAL_CORNERS[0], 0:INTERNAL_CORNERS[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_M

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objpoints, imgpoints, used_paths, image_size = [], [], [], None
    for path in img_paths:
        img = cv2.imread(path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]

        found, corners = cv2.findChessboardCorners(gray, INTERNAL_CORNERS, None)
        if not found:
            print(f"  corners NOT found: {path}")
            continue

        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners)
        used_paths.append(path)

    return objpoints, imgpoints, used_paths, image_size

def main():
    left_paths = sorted(glob.glob(f"{PAIRS_DIR}/left_*.jpg"))
    right_paths = sorted(glob.glob(f"{PAIRS_DIR}/right_*.jpg"))
    if len(left_paths) != len(right_paths) or not left_paths:
        raise RuntimeError(f"Mismatched or empty pairs: {len(left_paths)} left, {len(right_paths)} right")

    print(f"Found {len(left_paths)} pairs. Detecting checkerboard corners...")
    print("Left camera:")
    objpoints_l, imgpoints_l, used_l, size = find_corners(left_paths)
    print("Right camera:")
    objpoints_r, imgpoints_r, used_r, _ = find_corners(right_paths)

    # Only keep pairs where BOTH cameras found the board
    used_l_set, used_r_set = set(used_l), set(used_r)
    common = sorted(used_l_set & {p.replace("right_", "left_") for p in used_r_set})
    print(f"Corners found in both images for {len(common)}/{len(left_paths)} pairs.")
    if len(common) < 10:
        print("WARNING: fewer than 10 usable pairs — calibration may be inaccurate. Consider capturing more.")

    idx_l = [used_l.index(p) for p in common]
    idx_r = [used_r.index(p.replace("left_", "right_")) for p in common]
    objpoints = [objpoints_l[i] for i in idx_l]
    imgpoints_left = [imgpoints_l[i] for i in idx_l]
    imgpoints_right = [imgpoints_r[i] for i in idx_r]

    print("Calibrating left camera intrinsics...")
    ret_l, K1, D1, _, _ = cv2.calibrateCamera(objpoints, imgpoints_left, size, None, None)
    print(f"  left reprojection error: {ret_l:.4f} px")

    print("Calibrating right camera intrinsics...")
    ret_r, K2, D2, _, _ = cv2.calibrateCamera(objpoints, imgpoints_right, size, None, None)
    print(f"  right reprojection error: {ret_r:.4f} px")

    print("Running stereo calibration (extrinsics)...")
    flags = cv2.CALIB_FIX_INTRINSIC
    ret_s, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpoints_left, imgpoints_right, K1, D1, K2, D2, size,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5),
        flags=flags,
    )

    baseline_m = np.linalg.norm(T)
    print(f"  stereo reprojection error: {ret_s:.4f} px")
    print(f"  baseline: {baseline_m:.4f} m  (measured mount baseline was ~0.083m — should be close)")

    np.savez(
        OUTPUT_PATH,
        K1=K1, D1=D1, K2=K2, D2=D2, R=R, T=T, E=E, F=F,
        image_size=np.array(size), baseline_m=baseline_m,
    )
    print(f"Saved: {OUTPUT_PATH}")

if __name__ == "__main__":
    main()
