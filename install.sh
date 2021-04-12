#qpOASES
cd ~/catkin_ws/src
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES
mkdir build && cd build
cmake .. -DCMAKE_CXX_FLAGS=-fPIC
sudo make install

# px4 code and mavros
cd ~/catkin_ws/src
git clone https://github.com/icsl-Jeon/px4_code2.git
cd px4_code2
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-msgs ros-${ROS_DISTRO}-mavros-extras -y
sudo chmod +x ./resource/install_geographic.sh
sudo usermod -a -G dialout ${USER}
sudo usermod -a -G tty ${USER}

echo "You might have to reboot to use mavros"

# px4_code2_msg
cd ~/catkin_ws/src
git clone https://github.com/icsl-Jeon/px4_code2_msgs.git
catkin build px4_code2_msgs
catkin build px4_code2










