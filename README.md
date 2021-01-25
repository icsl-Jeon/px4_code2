# Code suite to manipulate multiple drone (server and client nodes)

## Installation 

* qpOASES

## Terms 

* Mission : trajectory to be followed by a drone. Calling `trigger()` will set triggerTime.

  Usage = 1. load trajectory. 2. Call trigger 

*  

## Features

* **Takeoff** : we use the yaw angle state when called takeoff service. Auto - triggered once mission uploaded.   
* **Lock**
* **Land**





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