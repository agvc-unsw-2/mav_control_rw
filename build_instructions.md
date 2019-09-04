For mbzirc2019 unsw members:

also clone this repo
git clone https://github.com/agvc-unsw-2/mav_control_rw
also clone mav_comm and eigen_catkin
git clone https://github.com/ethz-asl/mav_comm.git
git clone https://github.com/ethz-asl/eigen_catkin.git

Replace kalibr/catkin_simple with catkin/catkin_simple by deleting kalibr/catkin_simple and cloning catkin/catkin_simple into kalibr/

(git clone https://github.com/catkin/catkin_simple.git)

Also install dependencies
sudo apt-get install liblapacke-dev
sudo apt install libopenblas-dev
(if you still have build problems: sudo apt install libatlas-base-dev)

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox

