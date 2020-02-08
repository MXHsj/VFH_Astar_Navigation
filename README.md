# VFH_Astar_Navigation
## This navigation algorithm for non-holonomic mobile robots uses binary occupancy grid based A* path planner as the global planner and VFH as the local planner.
nav_pack_comp is for testing and comparing with ROS NAV STACK by sending goal pose inside MATLAB and collecting robot data
trajectory_tracking_dynamic_astar is the dynamic version of VFH Astar. It uses quintic function to convert A* path to desired trajectory and implement non-linear controller for trajectory tracking. Real-time replanning is added into A* function and will be triggered when tracking error gets too large.
vfh_astar_jackal_gui contains a MATLAB GUI interface for the user to enter goal pose and inspect the occupancy grid as well as the robot trajectory.
