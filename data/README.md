# data/

Datasets and raw captures. Paths in [`jetson/dataset_collection/`](../jetson/dataset_collection)
are written relative to the repo root, so run those scripts from the root.

| Folder | What it is | In git |
|---|---|---|
| `duck_dataset/` | YOLO-format training set assembled from `raw_frames/` + `raw_labels/` — `images/{train,val}`, `labels/{train,val}`, `data.yaml`. 164 labelled frames, one class. The deployed model was later retrained on a larger Roboflow export (1,240 images) that lives only on the Jetson. | yes |
| `raw_frames/` | Source JPEGs extracted from recorded video by `extract_frames.py`. Provenance for `duck_dataset/`. | yes |
| `raw_labels/` | YOLO `.txt` labels for `raw_frames/`, hand-drawn in makesense.ai. | yes |
| `imu/` | MPU6050 raw + processed logs from three static poses (flat / nose-up / on-side), captured by `jetson/tools/log_imu_raw.py` and reduced by `process_imu_raw.py`. Used to characterise gyro bias. | yes |
| `stereo_pairs/` | Checkerboard capture pairs with detected corners drawn on, from `capture_stereo_pairs.py`. Input to the 0.33 px calibration. Regenerable in ~10 minutes with the physical rig, so kept local and **gitignored** rather than adding 11 MB to the repo. | no |

The calibration output itself (`stereo_calibration.npz`) lives on the Jetson, not here — it is
specific to one physical rig and is invalidated by any change to the camera mount.

## Rebuilding the dataset

```bash
python3 jetson/dataset_collection/extract_frames.py   # video -> data/raw_frames/
# label data/raw_frames/ in makesense.ai, export YOLO txt -> data/raw_labels/
python3 jetson/dataset_collection/build_dataset.py    # -> data/duck_dataset/ (90/10 split, seed 42)
```
