TL;DR / Quickstart
1. Launch MPC:
roslaunch mav_nonlinear_mpc mav_nonlinear_mpc_f550_auto.launch
2. Open dynamic reconfigure gui
rosrun rqt_reconfigure rqt_reconfigure
3. Check status of
- thrust_scaling_factor
  - Adjust value accordingly based on mass and motors of drone. To play it safe you can set it arbitrary low and increase slowly.
- enable_disturbance_observer (enable after testing without disturbance_observer)
- enable_integrator (generally disabled)
4. Start sending commands to /nuc2/position_yaw (namespace changeable from launch file). This can be done using various methods including
- rqt
- uav_high_level_fsm package

For maximum stability with no load, you can increase r_roll and r_pitch to 200 or 500.
For effective load carrying and z-axis tracking, it is recommended to enable the disturbance observer. However, this may introduce oscillations in roll and pitch. 

Possible solutions:
- Set a dynamic thrust_scaling_factor when carrying loads
- Modify MPC to only take estimated force in z axis, and use integrator for x and y error
