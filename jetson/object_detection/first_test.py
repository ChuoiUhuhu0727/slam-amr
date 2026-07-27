"""First smoke test: CSI camera -> YOLOv8n pretrained -> draw boxes -> record.
Run on the Jetson (needs the CSI camera and `pip install ultralytics`).
Press 'q' to stop.
"""
import cv2
from ultralytics import YOLO

CSI_PIPELINE = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
    "nvvidconv ! "
    "video/x-raw, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! appsink drop=1"
)

OUTPUT_PATH = "first_test_output.mp4"
CONF_THRESHOLD = 0.5

def main():
    cap = cv2.VideoCapture(CSI_PIPELINE, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("Could not open CSI camera. Check the GStreamer pipeline / sensor-id.")

    model = YOLO("yolov8n.pt")  # pretrained COCO weights, auto-downloads on first run

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = None

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame grab failed, stopping.")
            break

        results = model(frame, conf=CONF_THRESHOLD, verbose=False)
        annotated = results[0].plot()  # draws boxes + labels + confidence

        if writer is None:
            h, w = annotated.shape[:2]
            writer = cv2.VideoWriter(OUTPUT_PATH, fourcc, 20.0, (w, h))
        writer.write(annotated)

        cv2.imshow("YOLOv8n - CSI camera", annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()
    print(f"Saved recording to {OUTPUT_PATH}")

if __name__ == "__main__":
    main()
