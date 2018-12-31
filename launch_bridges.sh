#!/bin/bash

xterm -hold -e roslaunch rotors_gazebo joy_to_roll_pitch_yawrate_thrust.launch mav_name:=ardrone &

xterm -hold -e python sim_bridges/mav_actuators_to_float64.py &
