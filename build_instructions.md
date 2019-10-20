For mbzirc2019 unsw members:

1. 
Replace kalibr/catkin_simple with catkin/catkin_simple by deleting kalibr/catkin_simple and cloning catkin/catkin_simple into kalibr/

i.e. from ~/catkin_ws/src, run the following commands

rm -rf kalibr/catkin_simple

git clone https://github.com/catkin/catkin_simple.git

2. 
also clone mav_comm and eigen_catkin

  $ git clone https://github.com/ethz-asl/mav_comm.git
  
  $ git clone https://github.com/ethz-asl/eigen_catkin.git

3.
also clone this repo

  $ git clone https://github.com/agvc-unsw-2/mav_control_rw/

4.
Install the following dependencies

sudo apt-get install liblapacke-dev

sudo apt install libopenblas-dev

(if you still have build problems: sudo apt install libatlas-base-dev)

