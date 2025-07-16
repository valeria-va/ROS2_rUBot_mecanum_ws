# ✅ Clone Student Project

Students have to:

```bash
cd /home/ubuntu
chmod +x clone_student_project.sh
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
After it finishes (approx. 5 minutes):

## Graphical display

Open: MobaXterm

Then run:

```bash
export DISPLAY=192.168.1.3:0.0
ros2 launch my_robot_bringup my_robot_nano_bringup_hw.launch.py
ros2 launch my_robot_description display.launch.xml use_sim_time:=False robot_model:=rubot/rubot_mecanum.urdf
```
Your robot will be displayed in rviz2
