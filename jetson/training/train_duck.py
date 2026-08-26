"""Fine-tune YOLOv8n on the custom duck dataset. Run on the Jetson (GPU).
Expects duck_dataset_roboflow/ (exported from Roboflow, 1240 images,
CC BY 4.0) in the same directory -- replaces the original hand-labeled
164-image duck_dataset/ as of 2026-08-26.
"""
from ultralytics import YOLO

def main():
    model = YOLO("yolov8n.pt")  # start from COCO-pretrained weights
    # batch/workers lowered from ultralytics' defaults (16/8) -- the default
    # 8 dataloader worker processes + batch 16 OOM-killed a run on this
    # Orin Nano's 8GB shared CPU+GPU memory at epoch 10/25 (confirmed via
    # dmesg: "Out of memory: Killed process ... python3"). 4/2 is
    # conservative on purpose after that failure -- not tuned for max
    # throughput.
    model.train(data="duck_dataset_roboflow/data.yaml", epochs=25, imgsz=640, batch=4, workers=2)

if __name__ == "__main__":
    main()
