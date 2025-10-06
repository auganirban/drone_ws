# PX4 ROS 2

In this tutorial, my plan is to first show to install PX4-ROS2 interface related packages and utilities. Then secondly, how to sequencially start various utility functions to communicate with a ros2 node. And finally, running a ros2 node to read sensor information of the vehicle.

Please note that all the informations are taken from the `https://docs.px4.io/main/en/`.

## Installation
The following fours items need to be built and installed.
- Install PX4
- Install ROS2
- Setup Micro XRCE-DDS Agent & Client
- Build and Run ROS2 Workspace
- (Optional) Install QGroundControl
- (Optional) PX4 ROS2 Message Translation Nodes

For detailed descriptions about the installation, please check out the website `https://docs.px4.io/main/en/ros2/user_guide.html`.

### Install PX4
```
cd
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl
```

### Install ROS2
```
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
```

Also install some python dependencies

```
pip install --user -U empy==3.3.4 pyros-genmsg setuptools
```

### Setup Micro XRCE-DDS Agent & Client
```
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### Build and Run ROS2 Workspace
```
mkdir -p ~/ws_sensor_combined/src/
cd ~/ws_sensor_combined/src/
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/auganirban/drone_ws.git
cd ..
source /opt/ros/humble/setup.bash
colcon build
```

At this point you are now ready to test the ROS2 nodes.
```
# terminal 1
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# terminal 2
cd ~/Micro-XRCE-DDS-Agent
MicroXRCEAgent udp4 -p 8888

# terminal 3
cd ~/AS_Dev_workspaces/ws_sensor_combined
source install/local_setup.bash
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```

To visualize the robot model
```
ros2 launch urdf_launch display.launch.py urdf_package:=px4_description urdf_package_path:=urdf/px4_description.xacro

ros2 run px4_description dynamic_joint_state.py

ros2 run px4_description move_drone.py
```

### (Optional) Install QGroundControl
PX4 alone is just the autopilot firmware running on a flight controller.
QGroundControl acts as the `human interface` to:

- Configure your drone
- Monitor its status,
- Plan and execute missions

Follow the instructions below,
```
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
chmod +x QGroundControl-<arch>.AppImage
./QGroundControl-<arch>.AppImage
```

### (Optional) PX4 ROS2 Message Translation Nodes
The message translation node allows ROS 2 applications that were compiled against different versions of the PX4 messages to interwork with newer versions of PX4, and vice versa, without having to change either the application or the PX4 side.
```
mkdir -p /path/to/ros_ws/src
cd /path/to/ros_ws
/path/to/PX4-Autopilot/Tools/copy_to_ros_ws.sh .
colcon build
source /path/to/ros_ws/install/setup.bash
ros2 run translation_node translation_node_bin
```

Some useful links:
- https://github.com/ARK-Electronics/ros2_px4_teleop_example/blob/dev/pat/.gitmodules
- https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example
