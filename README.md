# Code suite to manipulate multiple drone (server and client nodes)

## Installation 

* qpOASES

  ```
  $ git clone https://github.com/coin-or/qpOASES.git
  $ cd path/to/qpOASES
  $ mkdir build && cd build
  $ cmake .. -DCMAKE_CXX_FLAGS=-fPIC
  $ sudo make install
  ```

  

* mavros-*

  ```
  sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-msgs ros-${ROS_DISTRO}-mavros-extras 
  sudo chmod +x ./resource/install_geographic.sh 
  sudo ./resource/install_geographic.sh
  sudo usermod -a -G dialout <user>
  sudo usermod -a -G tty <user>
  sudo reboot
  ```

  

## Terms 

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