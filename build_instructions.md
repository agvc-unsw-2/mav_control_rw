For mbzirc2019 unsw members:

- Follow the necessary steps in the mbzirc2019 repo wiki

- Go to the mbzirc2019 repo and switch to the following branch using the following instruction

$ git checkout feature/mav_control_rw_vrep_intergration_v2

- Go to the following directory using the below command

$ cd ~/catkin_ws/src/mbzirc2019/mbz_utils

- Run the following scripts using the below commands

$ ./pull_required_repos.sh

$ ./install_deps.sh

- Go to the following directory using the below command

$ cd ~/catkin_ws/src/mav_control_rw

- FOR SIMULATION: Switch branches in this repo to feature/simulation using the following command

$ git checkout feature/simulation

BEFORE BUILDING:

- Delete the kalibr repo, or replace kalibr/catkin_simple with catkin/catkin_simple by deleting kalibr/catkin_simple and running the following command(s)

$ catkin clean kalibr

$ catkin clean catkin_simple

