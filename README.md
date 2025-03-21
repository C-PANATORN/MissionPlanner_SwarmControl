# MissionPlanner_SwarmControl

- [Introduction](#introduction)
  - [Getting Started](#getting-started)
  - [Installation](#installation)
- [Usage](#usage)
- [Advanced](#advanced)
  - [Ardupilot Installation](#ardupilot-stil-installation)
  - [Gazebo Installation](#gazebo-installation)
  - [Ardupilot Gazebo Plugin](#ardupilot-gazebo-plugin)
  - [ROS Installation](#ros-installation)
  - [Setting up Catkin Workspace](#setup-catkin-workspace)
  - [MAVROS Installation](#mavros-installation)
  - [ROS Packages](#ros-packages)
  - [MAVProxy Installation](#mavproxy-installation)
- [License](#license)

## Introduction

### Getting Started 
### Installation 

## Usage

## Advanced (Linux Only)
Please refer to the following repositories for further details on multiple quadrotor simulations.
- [Intelligent-Quads/iq_sim](https://github.com/Intelligent-Quads/iq_sim)  
- [monemati/multiuav-gazebo-simulation](https://github.com/monemati/multiuav-gazebo-simulation) 

#### Ardupilot STIL Installation
This repository uses the latest verion of Ardupilot as of time of writting (Arducopter V4.5.7 and Copter-4.5.7)
```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install git

git clone --recurse-submodules https://github.com/your-github-userid/ardupilot
git clone https://github.com/ArduPilot/ardupilot.git

ls
cd ardupilot/
Tools/environment_install/install-prereqs-ubuntu.sh -y
mavproxy.py --version
pip show MAVProxy
. ~/.profile


git checkout Copter-4.5.7
git submodule update --init --recursive
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```
#### Gazebo Installation
This repository has been tested using Gazebo Classic. For Gazebo Ignition users, please refer to **Open Source Robotics Foundation** offical documention for ROS/Gazebo stable releases. 
```bash
cd
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
cat /etc/apt/sources.list.d/gazebo-stable.list
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo11
sudo apt-get install libgazebo11-dev
gazebo
```
#### Ardupilot Gazebo Plugin
```bash
git clone https://github.com/SwiftGust/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install

sudo gedit ~/.bashrc
source /usr/share/gazebo/setup.sh

export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models_gazebo:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=~/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH}
source ~/.bashrc
```
#### ROS Installation 
This repository has been tested using ROS Noetic. 
```bash
sudo apt-get update
sudo apt-get upgrade

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-get install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```
 *Notice: As of writting, ROS 2 has offically been supported by Gazebo and Ardupilot. Please refer to offfical sources for documentation.*
#### Setup Catkin Workspace
```bash
sudo apt-get install python3-wstool python3-rosinstall-generator python3-catkin-lint python3-pip python3-catkin-tools
pip3 install osrf-pycommon

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```
#### MAVROS Installation 
Install MAVROS and MAVLink from source
```bash
cd ~/catkin_ws
wstool init ~/catkin_ws/src

rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```
#### ROS Packages
Install example packages for launching mutli-drone simulation
```bash
cd ~/catkin_ws/src
git clone https://github.com/Intelligent-Quads/iq_sim.git

echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/iq_sim/models" >> ~/.bashrc
source ~/.bashrc

cd ~/catkin_ws
catkin build
source ~/.bashrc
roslaunch iq_sim runway.launch
```
#### MAVProxy Installation
```bash
https://ardupilot.org/mavproxy/docs/getting_started/download_and_installation.html

sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
python3 -m pip install PyYAML mavproxy --user
echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc
source ~/.bashrc
```

## License

This software is distributed under the GPL-2.0 License:

```
stitchmd
Copyright (C) 2023 Abhinav Gupta

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
```