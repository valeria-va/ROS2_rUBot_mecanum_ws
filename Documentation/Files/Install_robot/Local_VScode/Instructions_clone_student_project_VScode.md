# Install the ROS2 environment and clone the student project

You have to:

- Connect VScode remotelly with SSH to the robot: If you are using a raspberrypi with another SD card you have to generate new KEYs
  ````shell
  ssh-keygen -R 192.168.1.xx
  ````
- Copy the following files to the `home/ubuntu` folder of your robot:
  - `install_ros2_humble_code_robot.sh`
  - `clone_student_project.sh`

# ✅ Install the ROS2 environment

Students have to execute the script (40min approx):
  ```bash
  cd /home/ubuntu
  ./install_ros2_humble_code_robot.sh
  ````

# ✅ Clone Student Project

Students have to:
- Execute the clone process of `Director`student project (4minutes aprox)
```bash
cd /home/ubuntu
./clone_student_project.sh director_github_username display_number
````

# ✅ Verify the installation
You proceed with:
  - Execute MobaXterm in your computer
  - Open a new terminal on VScode and type:
    ```bash
    ros2 launch my_robot_description display.launch.xml use_sim_time:=False robot_model:=rubot/rubot_mecanum.urdf
    ```
  - To verify only the running nodes, you can type:
    ```bash
    ros2 node list
    ```
Your robot will be displayed in rviz2
