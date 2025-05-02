# Project for Traffic Signal Detection with YOLO

We will describe the Computer Vision based method to identify the Traffic Sign

Webgraphy:
- https://github.com/xvrobotics/yolov9_ros/tree/main
- TheConstruct Class n.197: 3D Object Detection & Navigation with YOLO https://www.robotigniteacademy.com/rosjects/890667/

## ROS packages installation

- The needed to install:
````shell
sudo apt update
sudo apt install python3-pip
sudo apt install python3-opencv
pip3 install numpy matplotlib
pip3 install torch torchvision opencv-python
sudo apt install ros-humble-cv-bridge
````
- Clone the YOLO ROS Package: There are several repositories that integrate YOLO with ROS2:
    - Clone the repository:
    ````shell
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

We will create a new "my_robot_AI_identification" package with:
````shell
ros2 pkg create --build-type ament_python my_robot_AI_identification --dependencies rclpy std_msgs sensor_msgs geometry_msgs nav_msgs nav2_simple_commander tf_transformations cv_bridge
cd ..
colcon build
````

## Getting started

1. Bringup the rUBot: This is done when robot turn-on
   
2. Place the rUBot in front of the Traffic Signal

3. Take photos from USB camera
    ````bash
    ros2 run my_robot_AI_identification take_photo_exec
    ````
    You can use a more performand program to take photos continuously, add data in picture name for model generation and also to detect traffic signs when model is created.
    ````bash
    ros2 run my_robot_AI_identification takePhoto_detectSign_keras_exec
    ````
4. Open RVIZ to see the picture frame
    ````bash
    rviz
    ````
- Open the "teachablemachine" app to create a model for "Traffic Signs". Go to https://teachablemachine.withgoogle.com/ and create an image project.  
- Collect images with rUBot USB_CAM for each sign and upload them to the project.  
- Train the model.  
- Export the model as a keras .h5 model. The model can be created with some pictures, but this would be improved with some more pictures.  
- The models will be uploaded in "models" folder  

8. Verify topics (theConstruct):
   ```bash
   ros2 topic list
   ````
   Verify the topic name: /usb_cam/image_raw to place it in "takePhoto_detectSign_keras.py" file
   
9. Launch classification node and take photos (theConstruct):
   ```bash
   ros2 launch my_robot_AI_identification takePhoto_detectSign_keras.launch.py
   ````
10.1. Stop doing photos (theConstruct) (in a new Terminal):
   ```bash
   ros2 topic pub /capture_toggle std_msgs/Bool "data: false"
   ````
   If you want to do photos again set false value to true 
   ```bash
   ros2 topic pub /capture_toggle std_msgs/Bool "data: true"
   ````

### Final Exercise:

#### Navigate to a target point taking care of traffic signals 

* Load the room map 
* Start the navigation stack. 
* Get the coordinates of all the traffic sign using Rviz.
* Write a Python node with the folowing behaviour:
	* Go to the target point. Nav2 finds an optimat trajectory and starts to move
    * robot takes photos and identifies if there is a traffic sign
    * if robot finds a traffic sign, it obey the traffic signal
        * Left: turn left
        * Right: turn right
        * STOP: stops
        * Forbidden: turn 180º  
    * the nav2 package finds a new optimal trajectory to reach the target point

