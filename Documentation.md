This document should be read in conjunction wtih README.md in this folder.
So far this only describes things mostly off the top of my head and some brief skimming through files.
If there is any more detailed information regarding files in this repo please let me know.
Regards,
Hideyoshi

Packages:

mav_control_interface:
This package contains the code for the state machine which controls what commands are sent to the UAV.
Refer to state_machine.cpp for the code showing the state transitions (I have modified some of them).
There is a slightly convoluted diagram of the state machine in the resource folder of this package.

mav_disturbance_observer:
This package contains a standard Kalman filter which estimates external disturbances based on a second order
response model of the roll and pitch and measurements. The use of the disturbance observer is enabled in the resource
files of mav_linear_mpc and mav_nonlinear mpc. From simulations so far, it is found that this disturbance observer is
good at helping go from point to point, but not necessarily helping in dynamic trajectory tracking.

mav_linear_mpc:
linear MPC package. I have added lots of launch files to this package with lots of repeated code (mainly done so I could
tab-complete instead of having to remember to pass in arguments all the time).
When launching this package, there must be an appropriately named file in the "resources" folder which manages parameters.

mav_nonlinear_mpc:
Nonlinear MPC package. Follow the same rules for the nonlinear mpc as the linear mpc. linear mpc has additional parameters
for punishing the changes in roll and pitch while nonlinear does not. Nonlinear according to the paper performs better.

mav_lowlevel_attitude_controller:
pid controller for simulation purposes

Changes:
- All parameters are now reconfigurable using dynamic_reconfigure during flight.
- Lots of launch files added to mav_linear_mpc and mav_nonlinear_mpc
- Resource files added to mav_linear_mpc, mav_nonlinear_mpc, and mav_disturbance_observer
- State machine events and actions changed
- Added lots of scripts to mav_linear_mpc and mav_nonlinear_mpc

State machine switching (custom):
- I have an excel sheet which helps illustrate switching between states and control actions. I will probably add it to this repo

Checklist before using this MPC:
- Understand the launch and resource files for each package
of particular note:
  mav_nonlinear_mpc_aero_aero_auto.launch
  nonlinear_mpc_aero.yaml 
- Be aware of and how to use the attitude_cmd_rescale_rc.py and attitude_cmd_rescale_auto.py which
allow you to rescale the thrust differently for flying manually for system identification purposes or
when flying autonomously for emergency purposes. In hindsight I probably only need one of these.
- Be comfortable with flying in manual mode. I didn't test the teleop position control mode for sysid purposes.
I got Stan to fly it in manual mode (controlling desired roll pitch yawrate and thrust from stick)

SysID process:
- Configure joystick mapping and thrust scaling appropriately
- Toggle axis 5 switch to make sure you are in MPC manual flight mode (but qgroundcontrol offboard mode)
- Arm and fly manually as aggressively as possible and with some step response flights if possible.
- Example flight behaviour to test is in https://github.com/ethz-asl/mav_tools_public/wiki/Controller-Tuning
- Follow instructions in above
- There is a modified sysid matlab script in https://github.com/agvc-unsw-2/mav_system_identification/tree/feature/vrep_sysid

Checklist when running MPC
- Verify you are running the custom px4 firmware and mavros
- Make sure mavros is publishing odom messages (nav_msgs/Odometry) at 100Hz (can be modified in QGroundControl
from mavlink console. Details in google drive UAV notes->configuring parameters folder)
https://drive.google.com/open?id=1VBEb3evLOOwgozG4jIZqiT51j68pjgZDHQHtluxm078
IMU should also be published at 100Hz.
- You can check whether the topics are publishing correctly and fast enough in rviz

Steps for using MPC:
- If using a new vehicle, follow the checklist outlined below.
- Complete sysID process
- Test autonomous flight with reduced thrust scaling factor and slowly increase again.
- Run the MPC using roslaunch

Checklist for using a new UAV:
- Add appropriate launch and resource files to mav_linear_mpc/mav_nonlinear_mpc and mav_disturbance_observer
Refer to mav_nonlinear_mpc_aero_auto.launch or mav_nonlinear_mpc_aero_teleop_real.launch. The files are
essentially identical except for teleop being enabled or not. However, the state machine should have been
modified such that teleop (position control) is the same as manual flying. 
- Modify the launch files as necessary (e.g. default thrust_scaling_factor, topic names)
- Remap the joystick in the launch file as necessary. You can check whether the joystick is mapped
appropriately by starting the mav_nonlinear_mpc, going to manual mode on the joystick (axis 5), and echoing
the roll_pitch_yawrate_thrust command.


Words of warning:
- I recommend leaving on the disturbance observer and integrator on when first flying.
- Read the following: https://github.com/ethz-asl/mav_tools_public/wiki/Controller-Tuning
- In the above website, it mentions having a thrust_scaling_factor which can be changed in the
mbzirc2019/uav_controller package under px4_config.yaml. Instead, I have added a python script
which normalises the output thrust from Newtons to a scale from 0-1. I have chosen to do this
as using the thrust_scaling_factor required restarting the uav_controller every time I wanted
to test a new scaling factor. This scaling should be adjusted appropriately before flying
otherwise you may crash into the ceiling. It is better to slowly increase this scaling from a
low value until adequate. A good way to test if you have a good scaling factor is to slowly
increase until you are at the desired hover position. Then turn off the integrator through
dynamic_reconfigure or otherwise, adjust the scaling factor. Then turn off the disturbance
observer and adjust the scaling factor so that you achieve the desired hover height.
- The thrust output by the controller is dependent on the mass specified in the resource files.
I have set this to mass to one and introduced the thrust scaling script to account for this so
that you only have to change thrust scaling in one location.
