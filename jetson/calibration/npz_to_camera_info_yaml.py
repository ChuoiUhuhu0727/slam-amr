"""Convert stereo_calibration.npz (from stereo_calibrate.py) into 2 ROS camera_info YAML
files (standard camera_calibration_parsers format), one per camera.

Run on the Jetson HOST (not inside the Isaac ROS container) with plain python3 + numpy.
Output goes directly into the Isaac ROS workspace so it's visible inside the container too
(run_dev.sh mounts the whole ISAAC_ROS_WS directory, not just src/).

Since visual_slam_node will run with rectified_images:=False (raw distorted images fed
straight to cuVSLAM, which rectifies on GPU), rectification_matrix is identity and
projection_matrix is just the raw K padded with a zero column/row -- except for Tx
(projection_matrix[3]), which per the standard ROS stereo camera_info convention still
carries the baseline (Tx = -fx * baseline) on the right camera even in this raw/unrectified
case, so anything reading camera_info directly (rather than TF) can recover metric scale.
Left camera keeps Tx = 0 (it's the reference/origin of the stereo pair).
"""
import numpy as np

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
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [{fx}, 0.0, {cx}, {tx}, 0.0, {fy}, {cy}, 0.0, 0.0, 0.0, 1.0, 0.0]
"""


def write_yaml(path, name, K, D, width, height, tx=0.0):
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    d = D.flatten()
    k1, k2, p1, p2, k3 = d[0], d[1], d[2], d[3], d[4]
    content = TEMPLATE.format(
        width=width, height=height, name=name,
        fx=fx, fy=fy, cx=cx, cy=cy, tx=tx,
        k1=k1, k2=k2, p1=p1, p2=p2, k3=k3,
    )
    with open(path, "w") as f:
        f.write(content)
    print(f"Wrote {path}")


def main():
    import os
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    calib = np.load(CALIB_PATH)
    width, height = int(calib["image_size"][0]), int(calib["image_size"][1])
    # T is the left->right camera translation from stereo_calibrate.py (same value the
    # launch file's static TF uses for x). T[0] is the horizontal baseline in meters.
    baseline = float(calib["T"].flatten()[0])
    fx_right = float(calib["K2"][0, 0])
    tx_right = -fx_right * baseline

    write_yaml(f"{OUTPUT_DIR}/left.yaml", "left", calib["K1"], calib["D1"], width, height)
    write_yaml(f"{OUTPUT_DIR}/right.yaml", "right", calib["K2"], calib["D2"], width, height, tx=tx_right)


if __name__ == "__main__":
    main()
