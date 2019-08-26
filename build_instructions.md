For mbzirc2019 unsw members:

Replace kalibr/catkin_simple with catkin/catkin_simple by deleting kalibr/catkin_simple and cloning catkin/catkin_simple into kalibr/

(git clone https://github.com/catkin/catkin_simple.git)


also clone mav_comm and eigen_catkin
  $ git clone https://github.com/ethz-asl/mav_comm.git
  $ git clone https://github.com/ethz-asl/eigen_catkin.git

also clone this repo

sudo apt-get install liblapacke-dev
sudo apt install libopenblas-dev
(if you still have build problems: sudo apt install libatlas-base-dev)

