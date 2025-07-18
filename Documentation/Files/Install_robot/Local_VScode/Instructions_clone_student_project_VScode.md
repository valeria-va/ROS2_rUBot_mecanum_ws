# Install the ROS2 environment and clone the student project

You have to copy the following files to the `home/ubuntu` folder of your robot:

- `install_ros2_humble_robot.sh`
- `clone_student_project.sh`

# ✅ Install the ROS2 environment

Students have to:


# ✅ Clone Student Project

Students have to:
- Execute the clone process of `Director`student project (4minutes aprox)
```bash
cd /home/ubuntu
./clone_student_project.sh your_github_username_director
````
- Add environment setup to .bashrc:
````shell
  export ROS_DOMAIN_ID=0
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export GAZEBO_MODEL_PATH=/home/user/ROS2_rUBot_mecanum_ws/src/my_robot_bringup/models:$GAZEBO_MODEL_PATH
  source /opt/ros/humble/setup.bash
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
  source /home/user/ROS2_rUBot_tutorial_ws/install/setup.bash
  source /home/user/ROS2_rUBot_mecanum_ws/install/setup.bash
  #cd /home/user/ROS2_rUBot_tutorial_ws
  cd /home/user/ROS2_rUBot_mecanum_ws
  ````
# ✅ Verify the installation
You can work with:
- Nomachine
- VScode:
  - Open: MobaXterm in your computer
  - In a new terminal, type:
  ```bash
  export DISPLAY=192.168.1.3:0.0 #IP address of your computer 
  ros2 launch my_robot_bringup my_robot_nano_bringup_hw.launch.py
  ros2 launch my_robot_description display.launch.xml use_sim_time:=False robot_model:=rubot/rubot_mecanum.urdf
  ```
Your robot will be displayed in rviz2
