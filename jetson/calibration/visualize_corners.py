"""Draw detected checkerboard corners onto captured stereo pairs for a visual sanity check
(headless-safe: saves annotated images, no live display). Run after capture_stereo_pairs.py,
before trusting stereo_calibrate.py's numbers — confirms corners are found in the right
place across the whole board, not just that findChessboardCorners returned True.
"""
import glob
import os

import cv2

INTERNAL_CORNERS = (9, 6)
PAIRS_DIR = "stereo_pairs"
OUTPUT_DIR = "stereo_pairs_annotated"


def annotate(path):
    img = cv2.imread(path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, INTERNAL_CORNERS, None)
    cv2.drawChessboardCorners(img, INTERNAL_CORNERS, corners, found)
    label = "FOUND" if found else "NOT FOUND"
    cv2.putText(img, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                (0, 255, 0) if found else (0, 0, 255), 3)
    return img, found


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    paths = sorted(glob.glob(f"{PAIRS_DIR}/*.jpg"))
    if not paths:
        raise RuntimeError(f"No images found in {PAIRS_DIR}/")

    n_found = 0
    for path in paths:
        img, found = annotate(path)
        name = os.path.basename(path)
        cv2.imwrite(f"{OUTPUT_DIR}/{name}", img)
        n_found += found
        print(f"{name}: {'FOUND' if found else 'NOT FOUND'}")

    print(f"{n_found}/{len(paths)} images had corners detected. Annotated images saved to {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
