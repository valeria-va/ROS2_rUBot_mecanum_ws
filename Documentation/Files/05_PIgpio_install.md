
### **2.5. PIGPIO verification**

To install pigpio package in ubuntu for gpio from raspberrypi, we had to follow instructions in: https://abyz.me.uk/rpi/pigpio/download.html

The installation is done with theses instructions already defined in Dockerfile:
````shell
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
````
To verify that the installation was succesfully in Container, we could open a new container terminal:
````shell
docker exec -it rubot-humble-container bash
````
- test the pigpio installation
````shell
python3 -c "import pigpio; print('pigpio is installed')"
````
- Test the deamon is running
````shell
pidof pigpiod
````
>If the daemon is running, this command will return the PID (process identifier) ​​of pigpiod.
- Verify the file permissions:
````shell
stat -c /root/ROS2_rUBot_mecanum_ws/src/my_robot_driver/my_robot_driver/rubot_mecanum_driver.py
````
>The result will show the permissions (the first number should be 4777), the owner (root), the group (root), and the file name.
