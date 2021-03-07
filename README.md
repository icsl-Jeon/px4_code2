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
  git clone https://github.com/coin-or/qpOASES.git
  cd path/to/qpOASES
  mkdir build && cd build
  cmake .. -DCMAKE_CXX_FLAGS=-fPIC
  sudo make install
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
To use `px4_code2` in your multi-drone flight, you have a few things to be set in your GCS and drones' computer. 

### Step 1. Client side (your GCS)
 
An example of [client.launch](https://github.com/icsl-Jeon/px4_code2/blob/master/launch/client.launch) is shown below.
 
```
    <node name = "rqt_client" pkg = "rqt_gui" type = "rqt_gui"
          args="--perspective-file $(arg client_perspective)" output="screen">
        <rosparam param="/drone_name_set">[target1,target2]</rosparam>
        <rosparam param = "/world_frame_id">"map" </rosparam>
        <rosparam param="/qt_setting_dir" subst_value="True">$(find px4_code2)/qt_settings/setting.ini</rosparam>
    </node>
```

#### Parameters for client node 

* `/drone_name_set` : a list of name of drones used for the flight (up to three). The names should be matched with the [name](https://github.com/icsl-Jeon/px4_code2/blob/master/README.md#Parameters-for-server) in each launch of drone.  
* `/world_frame_id` : the name of world frame. The tf should be fully connected so that the reference frame of the odometry (fraome_id of header field of [odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)).      
* `/qt_setting_dir` : this is the setting file (.ini) containing the values in rqt gui. This file is loaded at every open of rqt gui, and saved at every normal termination.  



### Step 2. Server side (drones)

In setting up the server side, you have to provide the odometry topic and static transform from `world_frame_id` to the reference frame of the odometry. Also, [mavros] should be running in the onboard computer of the drone in the group namespace [`drone_name`]((https://github.com/icsl-Jeon/px4_code2/blob/master/README.md#ROS-paramteres-for-server-node).     

<p align = "center">
<img src= "https://github.com/icsl-Jeon/px4_code2/blob/master/img/frame.png" width="700">
</p> 

An example of launch can be found here: [server_t265.launch](https://github.com/icsl-Jeon/px4_code2/blob/master/launch/server_t265.launch) where Intel realsense t265 was used to collect odometry topic. In the example, t265 is mounted in the [opposite direction](https://github.com/icsl-Jeon/px4_code2/blob/master/README.md#ROS-paramteres-for-server-node) from the pixhawk. 

#### 1. Pixhawk setup of your drone 
*px4_code2* was designed for the [pixhawk setup](https://docs.px4.io/master/en/ros/external_position_estimation.html) where the [`mavros/vision_pose/pose`](https://docs.px4.io/master/en/ros/external_position_estimation.html#relaying-pose-data-to-px4) is used for EKF2 fusion yielding `mavros/local_position/pose`.

Thus, pixhawk should have the following paramters: 
* EKF2_AID_MASK = position + yaw
* EKF2_HGT_MODE = vision 

#### 2. Odometry setup 
`px4_code2` subscribes [odometry](https://github.com/icsl-Jeon/px4_code2/blob/master/README.md#ROS-paramteres-for-server-node) from other packages such as [realsense](http://wiki.ros.org/realsense2_camera) to cater the above pixhawk setting.
Basically, `px4_code2` receives the odometry and re-publish it as `geometry_msgs/PoseStamped` topic with respect to `world_frame_id`. 
Anyway, have your own odoemtry. 


#### ROS paramteres for server node
* `~drone_name` : the name of the considered drone. This is also the group namespace when launching the server.  
* `yaw_from_px4_to_sensor` : the relative yaw angle from pixhawk to the heading direction of the incoming odometry. 
   

#### Subscribed ros topics for server node

* `/<drone_name>/t265/odom/sample` : the incoming odometry topic. The frame_id in the header of the topic should be connected to `world_frame_id` in [tf](https://www.google.com/search?q=tf+ros&oq=tf+ros&aqs=chrome.0.0l7j69i65.2487j0j4&sourceid=chrome&ie=UTF-8)
* `/<drone_name>/mavros/state` : the state information of the pixhawk via mavros. This is used to identify whether the drone is in [offboard](https://docs.px4.io/master/en/flight_modes/offboard.html) mode or manual. 
* `/<drone_name>/mavros/local_position/pose` : this is the fused mavros pose (w.r.t `world_frame_id`) as the output of EKF2 fusion algorithm of pixhawk. This pose is used to [odometry](https://github.com/icsl-Jeon/px4_code2/blob/master/README.md#Features) as the current pose.        





## Features

* **Takeoff** : we use the yaw angle state when called takeoff service. Auto - triggered once mission uploaded.  StartPose = curMavrosPose  / Final pose = height in ui 
* **Lock** : Set cur pose as cur desired pose (NOTE = /mavros pose).  If mission exists, deactivates it.  
* **Land** : starting pose = cur mavros pose / desired height = 0 



