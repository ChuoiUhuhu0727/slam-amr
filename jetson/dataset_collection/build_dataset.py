"""Assemble images + makesense.ai YOLO labels into a proper YOLO training
dataset layout (images/train, images/val, labels/train, labels/val) plus
data.yaml. Run locally on the laptop (stdlib only, no special packages).
"""
import os
import random
import shutil

IMAGES_DIR = "duck_frames"
LABELS_DIR = "labeled_image"
OUTPUT_DIR = "duck_dataset"
VAL_FRACTION = 0.1
CLASS_NAMES = ["duck"]

random.seed(42)

def main():
    label_files = [f for f in os.listdir(LABELS_DIR) if f.endswith(".txt")]
    stems = sorted(os.path.splitext(f)[0] for f in label_files)

    pairs = []
    for stem in stems:
        img_path = os.path.join(IMAGES_DIR, stem + ".jpg")
        lbl_path = os.path.join(LABELS_DIR, stem + ".txt")
        if os.path.exists(img_path):
            pairs.append((img_path, lbl_path, stem))
        else:
            print(f"Skipping {stem}: no matching image")

    random.shuffle(pairs)
    n_val = max(1, int(len(pairs) * VAL_FRACTION))
    val_pairs = pairs[:n_val]
    train_pairs = pairs[n_val:]

    for split, split_pairs in [("train", train_pairs), ("val", val_pairs)]:
        img_out = os.path.join(OUTPUT_DIR, "images", split)
        lbl_out = os.path.join(OUTPUT_DIR, "labels", split)
        os.makedirs(img_out, exist_ok=True)
        os.makedirs(lbl_out, exist_ok=True)
        for img_path, lbl_path, stem in split_pairs:
            shutil.copy(img_path, os.path.join(img_out, stem + ".jpg"))
            shutil.copy(lbl_path, os.path.join(lbl_out, stem + ".txt"))

    yaml_path = os.path.join(OUTPUT_DIR, "data.yaml")
    with open(yaml_path, "w") as f:
        # No absolute "path:" - Ultralytics resolves train/val relative to
        # this file's own location, which keeps the dataset portable across
        # machines (built on Windows, trained on the Jetson).
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write(f"nc: {len(CLASS_NAMES)}\n")
        f.write(f"names: {CLASS_NAMES}\n")

    print(f"Train: {len(train_pairs)} images, Val: {len(val_pairs)} images")
    print(f"Dataset ready at {OUTPUT_DIR}/, config at {yaml_path}")

if __name__ == "__main__":
    main()
