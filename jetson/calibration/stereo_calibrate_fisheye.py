"""Fisheye-model stereo calibration — diagnostic variant of stereo_calibrate.py.

stereo_calibrate.py (pinhole model) gave a baseline of 0.1016m against a measured
physical baseline of 0.085m, a 22% error too large to explain by input measurement
noise (checkerboard square and physical baseline were both re-measured and matched
the script's inputs closely). This camera is a 160deg FOV IMX219 module; the pinhole
model's polynomial distortion is known to fit poorly past ~120-130deg FOV, which can
bias the focal length / baseline estimate even while per-image reprojection error
looks low. cv2.fisheye implements the equidistant model built for this FOV range.

Same 20 stereo_pairs/ images as stereo_calibrate.py, only the calibration model changes.
If baseline converges near 0.085m here, that confirms the pinhole model was the bug.
"""
import glob

import cv2
import numpy as np

INTERNAL_CORNERS = (9, 6)  # (cols, rows) of internal corners
SQUARE_SIZE_M = 0.025

PAIRS_DIR = "stereo_pairs"
OUTPUT_PATH = "stereo_calibration_fisheye.npz"

SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
CALIB_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)


def find_corners(img_paths):
    objp = np.zeros((1, INTERNAL_CORNERS[0] * INTERNAL_CORNERS[1], 3), np.float64)
    objp[0, :, :2] = np.mgrid[0:INTERNAL_CORNERS[0], 0:INTERNAL_CORNERS[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_M

    objpoints, imgpoints, used_paths, image_size = [], [], [], None
    for path in img_paths:
        img = cv2.imread(path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]

        found, corners = cv2.findChessboardCorners(gray, INTERNAL_CORNERS, None)
        if not found:
            print(f"  corners NOT found: {path}")
            continue

        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUBPIX_CRITERIA)
        objpoints.append(objp)
        imgpoints.append(corners.reshape(1, -1, 2))
        used_paths.append(path)

    return objpoints, imgpoints, used_paths, image_size


def calibrate_single(objpoints, imgpoints, image_size):
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    n = len(objpoints)
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(n)]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(n)]
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
    rms, K, D, _, _ = cv2.fisheye.calibrate(
        objpoints, imgpoints, image_size, K, D, rvecs, tvecs, flags, CALIB_CRITERIA
    )
    return rms, K, D


def main():
    left_paths = sorted(glob.glob(f"{PAIRS_DIR}/left_*.jpg"))
    right_paths = sorted(glob.glob(f"{PAIRS_DIR}/right_*.jpg"))
    if len(left_paths) != len(right_paths) or not left_paths:
        raise RuntimeError(f"Mismatched or empty pairs: {len(left_paths)} left, {len(right_paths)} right")

    print(f"Found {len(left_paths)} pairs. Detecting checkerboard corners (fisheye model)...")
    print("Left camera:")
    objpoints_l, imgpoints_l, used_l, size = find_corners(left_paths)
    print("Right camera:")
    objpoints_r, imgpoints_r, used_r, _ = find_corners(right_paths)

    used_r_set = set(used_r)
    common = sorted(set(used_l) & {p.replace("right_", "left_") for p in used_r_set})
    print(f"Corners found in both images for {len(common)}/{len(left_paths)} pairs.")
    if len(common) < 10:
        print("WARNING: fewer than 10 usable pairs.")

    idx_l = [used_l.index(p) for p in common]
    idx_r = [used_r.index(p.replace("left_", "right_")) for p in common]
    objpoints = [objpoints_l[i] for i in idx_l]
    imgpoints_left = [imgpoints_l[i] for i in idx_l]
    imgpoints_right = [imgpoints_r[i] for i in idx_r]

    print("Calibrating left camera intrinsics (fisheye)...")
    rms_l, K1, D1 = calibrate_single(objpoints, imgpoints_left, size)
    print(f"  left reprojection error: {rms_l:.4f} px")

    print("Calibrating right camera intrinsics (fisheye)...")
    rms_r, K2, D2 = calibrate_single(objpoints, imgpoints_right, size)
    print(f"  right reprojection error: {rms_r:.4f} px")

    print("Running fisheye stereo calibration (extrinsics)...")
    R = np.zeros((1, 1, 3), dtype=np.float64)
    T = np.zeros((1, 1, 3), dtype=np.float64)
    flags = cv2.fisheye.CALIB_FIX_INTRINSIC
    ret_s, K1, D1, K2, D2, R, T = cv2.fisheye.stereoCalibrate(
        objpoints, imgpoints_left, imgpoints_right, K1, D1, K2, D2, size, R, T,
        flags=flags, criteria=CALIB_CRITERIA,
    )

    baseline_m = np.linalg.norm(T)
    print(f"  stereo reprojection error: {ret_s:.4f} px")
    print(f"  baseline: {baseline_m:.4f} m  (measured physical baseline is 0.085m)")

    np.savez(
        OUTPUT_PATH,
        K1=K1, D1=D1, K2=K2, D2=D2, R=R, T=T,
        image_size=np.array(size), baseline_m=baseline_m,
    )
    print(f"Saved: {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
