# Project for Traffic Signal Detection with YOLO

We will describe the Computer Vision based method to identify the Traffic Sign.

Webgraphy:
- https://github.com/xvrobotics/yolov9_ros/tree/main
- TheConstruct Class n.197: 3D Object Detection & Navigation with YOLO https://www.robotigniteacademy.com/rosjects/890667/
- For Yolo-v7: https://github.com/ros2/openrobotics_darknet_ros
- For Yolo-v9: https://github.com/xvrobotics/yolov9_ros

## **1. ROS packages installation**

We will analyse for YOLO-v7 and YOLO-v9

### **1.1. YOLO-v7**

Follow the instructions on: https://github.com/ros2/openrobotics_darknet_ros

- Install needed packages:
````shell
sudo apt update
sudo apt install python3-pip
sudo apt install python3-opencv
pip3 install numpy matplotlib
sudo apt install ros-humble-cv-bridge
````
- Clone the YOLO ROS Package: There are several repositories that integrate YOLO with ROS2:
    - Clone the repository:
    ````shell
    cd src/AI_Projects
    git clone https://github.com/ros2/openrobotics_darknet_ros.git
    cd ../..
    ````
- Compiling this package with
    ````shell
    colcon build --cmake-args -DDOWNLOAD_YOLO_CONFIG=ON
    ````
    **You have an error because there is not installed Darkned**
- Download and install Darknet
    ````shell
    cd ~/ROS2_rUBot_mecanum_ws/src/AI_Projects/openrobotics_darknet_ros
    git clone https://github.com/AlexeyAB/darknet.git
    cd darknet
    make
    ````
- Download yolo7.weights
    ````shell
    cd ~/ROS2_rUBot_mecanum_ws/src/AI_Projects/openrobotics_darknet_ros/darknet
    wget https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7.weights
    ````

    >will automatically download the pretrained YOLO v3, v4 and v7 configuration files.

- You can then launch the detector node with:
    ````shell
    ros2 launch openrobotics_darknet_ros detector_launch.py rgb_image:=/topic
    ````
    >optionally supplying a desired parameter file detector_parameters:=path/to/detector_node_params.yaml.

- Train your model:
    ````shell
    ./darknet detector train data/obj.data yolov7-custom.cfg yolov7.weights
    ````
    
- You can also train YOLO to detect custom objects like described here: 
    - https://github.com/AlexeyAB/darknet#how-to-train-tiny-yolo-to-detect-your-custom-objects
 and create the following as detector_node_params.yaml:
    - and create the following as detector_node_params.yaml:
    ````shell
    /**:
  ros__parameters:
    network:
      config: "./your-yolo-config.cfg"
      weights: "./your-yolo-weights.weights"
      class_names: "./your-cocos.names"
    detection:
      threshold: 0.25
      nms_threshold: 0.45
    ````
- Finally you can run the detector node with
    ````shell
    $ ros2 run openrobotics_darknet_ros detector_node --ros-args --params-file path/to/detector_node_params.yaml
    ````
- and publish images on ~/images to get the node to detect objects. You can also manually remap an external topic to the ~/images topic with:
    ````shell
    $ ros2 run openrobotics_darknet_ros detector_node --ros-args --params-file path/to/detector_node_params.yaml -r '~/images:=/your/camera/topic'
    ````

### **1.1. YOLO-v9**

- The needed to install:
````shell
sudo apt update
sudo apt install python3-pip
sudo apt install python3-opencv
pip3 install numpy matplotlib
sudo apt install ros-humble-cv-bridge
pip3 install torch torchvision opencv-python
````
- Clone the YOLO ROS Package: There are several repositories that integrate YOLO with ROS2:
    - Clone the repository:
    ````shell
    cd src
    git clone https://github.com/xvrobotics/yolov9_ros.git
    cd yolov9_ros
    ````
    - Install Dependencies: Ensure all required dependencies are installed.
    ````shell
    cd src/yolov9/yolov9
    pip3 install -r requirements.txt
    ````
    - Build the Package:
    ````shell
    cd ~/yolov9_ros
    colcon build
    source install/setup.bash
    ````
- Launch the YOLO Node: Launch the Node: Use the provided launch file to start the YOLOv9 detection node
````shell
ros2 launch yolov9_ros yolov9.launch.py
````
- Configure Topics and Parameters
    - Published Topics:
        - /yolov9/detections: Publishes detection results including bounding box coordinates, confidence scores, and object labels.
        - /yolov9/annotated_image: Publishes an image with bounding boxes drawn around detected objects.
    - Subscribed Topics:
        - /camera/raw/image: Subscribes to an image topic for object detection.
    - Parameters: Configure parameters like model weights, confidence threshold, image size, etc., in the launch file or parameter file.

Example Integration with ROS2 Humble:

Here's a simplified example of how you might set up the YOLO node in a ROS2 Humble project: This example sets up a ROS2 node that subscribes to an image topic, processes the image using YOLO, and publishes the annotated image.
````shell
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import torch
from PIL import Image as PILImage
import cv2
import numpy as np

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/raw/image',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, '/yolov9/annotated_image', 10)
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

    def listener_callback(self, msg):
        frame = self.convert_image(msg)
        results = self.model(frame)
        annotated_frame = results.render()[0]
        self.publisher.publish(self.convert_to_ros_image(annotated_frame))

    def convert_image(self, msg):
        np_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        return PILImage.fromarray(np_img)

    def convert_to_ros_image(self, img):
        ros_img = Image()
        ros_img.height, ros_img.width, _ = img.shape
        ros_img.data = img.tobytes()
        return ros_img

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
````

