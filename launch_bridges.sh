#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 MAV_NAME" >&2
    exit 1
fi

mav_name=$1

#xterm -hold -e roslaunch rotors_gazebo joy_to_roll_pitch_yawrate_thrust.launch mav_name:=$mav_name &

xterm -hold -e roslaunch mav_linear_mpc joy_publisher.launch mav_name:=$mav_name & 
xterm -hold -e python sim_bridges/mav_actuators_to_float64.py $mav_name
