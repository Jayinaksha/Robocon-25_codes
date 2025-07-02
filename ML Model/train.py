from ultralytics import YOLO

model = YOLO('yolov8s.pt')  # Or yolov8s.pt, etc.

model.train(
    data='data.yaml',
    epochs=50,
    imgsz=640,
    batch=16
)