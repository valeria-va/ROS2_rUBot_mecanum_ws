# This script demonstrates how to train a YOLOv8n model using the Ultralytics YOLO library.
from ultralytics import YOLO

# Load a pretrained YOLO8n model
model = YOLO("yolov8n.pt")  # Load the YOLOv8n model

# Train the model on the our dataset for 100 epochs
train_results = model.train(
    data="data.yaml",  # Path to dataset configuration file (Roboflow dataset)
    epochs=100,  # Number of training epochs
    imgsz=640,  # Image size for training
    device="cpu",  # Device to run on (e.g., 'cpu', 0, [0,1,2,3])
)

# Evaluate the model's performance on the validation set
metrics = model.val()

# Perform object detection on an image
results = model("test/images/Foto_77_jpg.rf.34d8e25dfabc6ff512c83619a62eed88.jpg")  # Predict on an image from test set
results[0].show()  # Display results

# Save the model's weights
model.save("yolov8n_custom.pt")  # Save the model with custom weights
# Export the model to ONNX format for deployment
path = model.export(format="onnx")  # Returns the path to the exported model