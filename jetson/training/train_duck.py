"""Fine-tune YOLOv8n on the custom duck dataset. Run on the Jetson (GPU).
Expects duck_dataset/ (from build_dataset.py) in the same directory.
"""
from ultralytics import YOLO

def main():
    model = YOLO("yolov8n.pt")  # start from COCO-pretrained weights
    model.train(data="duck_dataset/data.yaml", epochs=25, imgsz=640)

if __name__ == "__main__":
    main()
