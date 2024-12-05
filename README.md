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


