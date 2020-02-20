Before doing sysID:
- Tune the low level attitude controller (in PX4 parameters other otherwise) until you get stable, easy to fly parameters in position flight mode.
- I highly recommend an overdamped attitude control response (minimal or no overshoot). Check PX4 logs to see if this is achieved.

SysID in pseudo autonomous mode:
- Use some set of default parameters such as those for the f500 and launch MPC (check Usage.md)
- If stable, give it position commands of varying changes in distance. Try change both X and Y at the same time and capture as wide a variety of movements as possible. Yawing is not necessary.
- Record this data in a bag file

SysID in pseudo manual attitude control mode:

- Configure joystick mapping and thrust_scaling_factor appropriately
  - You can change the thrust_scaling_factor in rqt_reconfigure
- Toggle axis 5 switch to make sure you are in MPC manual flight mode (but qgroundcontrol offboard mode)
- Arm and fly manually as aggressively as possible and with some step response flights if possible.
- Record this data in a bag file.
- Example flight behaviour to test is in https://github.com/ethz-asl/mav_tools_public/wiki/Controller-Tuning
- Follow instructions in above

Running sysID script to get new parameters:
- There is a modified sysid matlab script in https://github.com/agvc-unsw-2/mav_system_identification/tree/feature/vrep_sysid
