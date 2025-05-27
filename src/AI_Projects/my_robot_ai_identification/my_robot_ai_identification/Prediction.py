# This script demonstrates how to train a YOLOv8n model using the Ultralytics YOLO library.
from ultralytics import YOLO

# Load a pretrained YOLO8n model
model = YOLO("yolov8n_custom.pt")  # Load the YOLOv8n model

# Perform object detection on an image
results = model("test/images/Foto_64_jpg.rf.e1bc01a8004eef2fe69f12b70f2a7b54.jpg")  # Predict on an image from test set
results[0].show()  # Display results