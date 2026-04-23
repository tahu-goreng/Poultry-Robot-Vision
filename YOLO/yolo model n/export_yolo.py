# Using Ultralytics (YOLOv8/v11)
from ultralytics import YOLO

model = YOLO("my_model/my_model.pt")
model.export(format="engine", device=0, half=True)  # Exports to .engine (TensorRT)

# Then load and run the TensorRT model
trt_model = YOLO("your_model.engine")
results = trt_model(frame)
