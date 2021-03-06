# Code Suite of RQT GUI for multi-drone flight (px4/mavros based)

<img src = "https://img.shields.io/github/license/Naereen/StrapDown.js.svg">
<p align = "center">
<img src= "https://github.com/icsl-Jeon/px4_code2/blob/master/img/intro.gif">
</p> 

## Youtube videos 

* **2 min manual for gui** [Youtube](https://www.youtube.com/watch?v=-kpmBBiJndk&t=51s)
* **2 min manual for trajectory creation** [Youtube](https://www.youtube.com/watch?v=-kpmBBiJndk&t=51s)


## Installation 

Ubuntu 16.04 (ROS kinetic), 18.04 (ROS melodic) and 20.04 (ROS noetic) were tested. 
  

* **qpOASES**

  ```
  $ git clone https://github.com/coin-or/qpOASES.git
  $ cd path/to/qpOASES
  $ mkdir build && cd build
  $ cmake .. -DCMAKE_CXX_FLAGS=-fPIC
  $ sudo make install
  ```
  *qpOASES* is quadratic programming (QP) solver. *px4_code* generates the flight trajectory based on QP to produce input-efficient flight for drones by optimizing the high order-derivatives of the entire path. The key principle can be found in [traj_gen](https://github.com/icsl-Jeon/traj_gen) (you don't have to install the pacakge).   
  

* **mavros-***
  ```
  cd ~/catkin_ws/src
  git clone https://github.com/icsl-Jeon/px4_code2.git
  cd px4_code2
  ```
  ```
  sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-msgs ros-${ROS_DISTRO}-mavros-extras 
  sudo chmod +x ./resource/install_geographic.sh 
  sudo ./resource/install_geographic.sh
  sudo usermod -a -G dialout <user>
  sudo usermod -a -G tty <user>
  sudo reboot
  ```
  This package was desinged to flight of the drones which has pixhawk FCU and an onboard computer (e.g. [upboard](https://up-board.org/) or [NUC](https://www.amazon.com/intel-nuc8i7/s?k=intel+nuc8i7&page=2)). We assume that the drones can be provided with their [odometry information](https://github.com/icsl-Jeon/px4_code2/blob/master/README.md#Setup) and run mavros for the communicate with GCS (e.g. your laptop).           
  
* **px4_code2**
  ```
  cd ~/catkin_ws/src
  git clone https://github.com/icsl-Jeon/px4_code2.git
  git clone https://github.com/icsl-Jeon/px4_code2_msgs.git
  catkin build px4_code2_msgs
  catkin build px4_code2
  ```
  * Note: if you want to use `catkin build` in Noetic, run the following command:
  ```
  sudo apt-get install python3-catkin-tools python3-osrf-pycommon
  ```
  
  

## Quick start for client rqt node in GCS (e.g. your commander laptop)
If you try to load the dashboard of px4_code2, follow

### Option 1. Run fresh px4_code2 via rqt_gui 
Like any other [rqt node](http://wiki.ros.org/rqt), *px4_code2* can be loaded in `rqt_gui` node.
```
rosrun rqt_gui rqt_gui --force-discover
```
Load by `Plugins > Robot Tools > FelipeSuite`.

###  Option 2. Roslaunch by importing preset *.perspective file and *.ini qt settings 
#### kinectic ~ melodic 

```
roslaunch px4_code2 client.launch is_noetic:=false
```

#### noetic 
```
roslaunch px4_code2 client.launch 
```


## Setup 

* Mission : trajectory to be followed by a drone. Calling `trigger()` will set triggerTime.

  Usage = 1. load trajectory. 2. Call trigger 

*  

## Features



* **Takeoff** : we use the yaw angle state when called takeoff service. Auto - triggered once mission uploaded.  StartPose = curMavrosPose  / Final pose = height in ui 
* **Lock** : Set cur pose as cur desired pose (NOTE = /mavros pose).  If mission exists, deactivates it.  
* **Land** : starting pose = cur mavros pose / desired height = 0 





## Lessons 

### rqt plugin ([Ref](https://fjp.at/ros/rqt-turtle/))

First, create Qt application project with `Qt creator` inside of a project. All the UI (form file) and SLOT functions are composed only in the Qt project.  

Include the files in CMakeList.txt under `qtx_wrap_cpp` and build first. That will produce header and source in your catkin_ws. That's all we need to auto-complete in CLion.  Don't forget the below in CLion cmake build setting: 

```
-DCATKIN_DEVEL_PREFIX:PATH=/home/jbs/catkin_ws/devel
```


### rqt gui list 
```
# Build your package 
catkin build px4_code2

# Check the plugins are loaded correctly
rosrun rqt_gui rqt_gui --force-discover

```

### PX4 

* PWM_ARM_* : for initial thrust when arming begins
* Gain tuning : for small drone such as f330 size, the gain tuning was required. See resource folder  
* HAS_BARO = 0 : Disabling barometer was only available in 1.10.1. For the firmware, see resource folder. 
  In the recent version, disabling barometer blocks EKF2 operation and QGC does not receive attitude info.
* Do not use Pixhwak 4 mini. It caused a lot of abrupt increase in thrust.   
* [Land detection](https://docs.px4.io/master/en/advanced_config/land_detector.html#land-detector-states) 
