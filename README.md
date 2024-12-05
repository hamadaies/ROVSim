# ROVSim
Simulate ROV using Gazebo 7, ROS 2, ArduSub, MAVProxy

There are two methods to setup the simulation environment:  
1- Automatic install by building a Docker Container (More likely to fail)  
2- Manually installing the Ubuntu VM (More time consuming)  

## Method 1 (Docker)
~~~
cd ~/
git clone https://github.com/clydemcqueen/orca4.git
cd ~/orca4/docker
./build.sh
~~~

To launch Gazebo, RViz, all nodes:
~~~
./run.sh
ros2 launch orca_bringup sim_launch.py
~~~

To execute a mission:
~~~
docker exec -it orca4 /bin/bash
ros2 run orca_bringup mission_runner.py
~~~

## Method 2 
Install [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)

Install ROS2 Humble HawkShell
~~~
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop
~~~

Configure ROS2 Environment and Check ENV Variables
~~~
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
printenv | grep -i ROS
~~~

Install Gazebo 7
~~~
sudo apt-get update
sudo apt-get install lsb-release curl gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden
~~~

Try Installation
~~~
gz sim shapes.sdf
~~~

If it Gazebo opens but crashes just before it loads, run the following. Then if that fixes your problem save it to your .bashrc by the second command
~~~
export LIBGL_ALWAYS_SOFTWARE=1

echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
~~~

Install ArduPilot/Sub
~~~
cd
git clone https://github.com/ArduPilot/ardupilot.git --recurse-submodules
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
~~~

Reload Path then log out and log back in by
~~~
. ~/.profile
~~~

Build ArduSub for SITL
~~~
cd ~/ardupilot
./waf configure --board sitl
./waf sub
~~~

ArduPilot_Gazebo Dependancies 
~~~
sudo apt update
sudo apt install libgz-sim7-dev rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
~~~


Install ArduPilot_Gazebo Plugin
~~~
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
echo "export GZ_VERSION=garden" >> ~/.bashrc

cd ardupilot_gazebo
git checkout 4945088
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4

echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
~~~


Install VCS, ROSDEP, and Colcon
~~~
sudo apt install python3-vcstool
sudo apt-get install python3-rosdep
sudo apt install python3-colcon-common-extensions
~~~

Install Orca4
~~~
mkdir -p ~/colcon_ws/src

cd ~/colcon_ws/src
git clone https://github.com/clydemcqueen/orca4
vcs import < orca4/workspace.repos

cd ~/colcon_ws/
rosdep update
rosdep install -y --from-paths . --ignore-src --skip-keys="gz-transport12 gz-sim7 gz-math7 gz-msgs9"
~~~

MavROS Dependancies
~~~
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
~~~

Build Everything (Will take alot of time)
~~~
cd ~/colcon_ws
colcon build
~~~


IF EVERYTHING WAS INSTALLED CORRECTLY:
just try this first
~~~
source src/orca4/setup.bash
ros2 launch orca_bringup sim_launch.py
~~~
and maybe if you are lukcy u get to try this sample mission by excuting this a second terminal
~~~
source src/orca4/setup.bash
ros2 run orca_bringup mission_runner.py
~~~

if that worked out fine we can know try to use mavproxy version (each command in a seperate temrinal)

~~~
ros2 launch orca_bringup sim_launch.py base:=false mavros:=false nav:=false rviz:=false slam:=false

mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --out udp:0.0.0.0:14550 --console
~~~


You can use MAVProxy to send commands directly to ArduSub:
~~~
arm throttle
rc 3 1450
rc 3 1500
mode alt_hold
disarm
~~~

RC channels:
* RC 3 -- vertical
* RC 4 -- yaw
* RC 5 -- forward
* RC 6 -- strafe

or 

You should be able to use The ROV GUI and the default ip addresses should work
