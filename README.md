# Code suite to manipulate multiple drone (server and client nodes)



## Features

* **Takeoff** 
* 





## Lessons 

### rqt plugin ([Ref](https://fjp.at/ros/rqt-turtle/))

First, make .ui file with qt designer. Include the file in CMakeList.txt under `qtx_wrap_cpp` and build first. That will produce header and source in your catkin_ws. That's all we need to auto-complete in CLion.  Don't forget the below: 

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