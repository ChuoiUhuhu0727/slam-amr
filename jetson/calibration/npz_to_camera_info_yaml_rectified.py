"""Convert stereo_calibration.npz into ROS camera_info YAML files for TRUE stereo rectification
(rectified_images:=True path), used to test whether the raw-mode (rectified_images:=False)
camera_info interpretation is the source of the vo_pose scale bug (see README "Lessons
Learned" 2026-08-04, Part 3).

Unlike npz_to_camera_info_yaml.py (which sends raw distorted images + K/D straight to cuVSLAM,
rectification_matrix=identity, no baseline in P -- cuVSLAM rectifies on GPU internally), this
script runs cv2.stereoRectify() to compute the real rectification rotations (R1, R2) and
projection matrices (P1, P2) for a proper rectified pair. P2's Tx term comes directly out of
stereoRectify's own math (Tx = -fx' * baseline), not hand-computed -- so it can't have the
sign/double-counting issue found when Tx was manually patched into the raw-mode P matrix
(see git history: PR #27 fixed, then reverted by PR #28 after it made vo_pose scale worse).

Run on the Jetson HOST, same as npz_to_camera_info_yaml.py. Paired with
jetson/slam/visual_slam_argus_rectified.launch.py, which adds an isaac_ros_image_proc
RectifyNode per camera between ArgusMonoNode and VisualSlamNode.
"""
import numpy as np
import cv2

CALIB_PATH = "/home/chuoichiuchiu/slam-amr/stereo_calibration.npz"
OUTPUT_DIR = "/home/chuoichiuchiu/workspaces/isaac_ros-dev/camera_info"

TEMPLATE = """image_width: {width}
image_height: {height}
camera_name: {name}
camera_matrix:
  rows: 3
  cols: 3
  data: [{fx}, 0.0, {cx}, 0.0, {fy}, {cy}, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [{k1}, {k2}, {p1}, {p2}, {k3}]
rectification_matrix:
  rows: 3
  cols: 3
  data: [{r0}, {r1}, {r2}, {r3}, {r4}, {r5}, {r6}, {r7}, {r8}]
projection_matrix:
  rows: 3
  cols: 4
  data: [{p00}, {p01}, {p02}, {p03}, {p10}, {p11}, {p12}, {p13}, {p20}, {p21}, {p22}, {p23}]
"""


def write_yaml(path, name, K, D, width, height, R, P):
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    d = D.flatten()
    k1, k2, p1, p2, k3 = d[0], d[1], d[2], d[3], d[4]
    r = R.flatten()
    p = P.flatten()
    content = TEMPLATE.format(
        width=width, height=height, name=name,
        fx=fx, fy=fy, cx=cx, cy=cy,
        k1=k1, k2=k2, p1=p1, p2=p2, k3=k3,
        r0=r[0], r1=r[1], r2=r[2], r3=r[3], r4=r[4], r5=r[5], r6=r[6], r7=r[7], r8=r[8],
        p00=p[0], p01=p[1], p02=p[2], p03=p[3],
        p10=p[4], p11=p[5], p12=p[6], p13=p[7],
        p20=p[8], p21=p[9], p22=p[10], p23=p[11],
    )
    with open(path, "w") as f:
        f.write(content)
    print(f"Wrote {path}")


def main():
    import os
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    calib = np.load(CALIB_PATH)
    width, height = int(calib["image_size"][0]), int(calib["image_size"][1])
    K1, D1, K2, D2 = calib["K1"], calib["D1"], calib["K2"], calib["D2"]
    R, T = calib["R"], calib["T"]

    # CALIB_ZERO_DISPARITY aligns principal points across both rectified images -- the
    # standard choice for stereo/VSLAM disparity math. alpha=0 crops to the valid pixel
    # region only (no black borders), matching what RectifyNode will actually output.
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K1, D1, K2, D2, (width, height), R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
    )

    write_yaml(f"{OUTPUT_DIR}/left_rect.yaml", "left", K1, D1, width, height, R1, P1)
    write_yaml(f"{OUTPUT_DIR}/right_rect.yaml", "right", K2, D2, width, height, R2, P2)


if __name__ == "__main__":
    main()
