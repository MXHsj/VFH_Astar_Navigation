## Algorithm Explaination
This algorithm combines VFH and A* to perform wheeled robot navigation:

The environment in the form of probabilistic occupancy grid is known to the robot.

A* takes the goal position and the robot's current position under global frame and calculates the optimal path. This path is given in the form of a list of waypoints.

NOTICE A* only runs once when navigation begins.

VFH locally navigates the robot to each of the waypoint until the last waypoint, which is the final goal is reached.

Advantage:
1. This algorithm can handle pop up obstacle as long as the obstacle does not sit on one of the waypoints. A* does not need to be run multiple times when new obstacles appear.

Limitation:
1. If one waypoint is blocked or gets too close to an obstacle, VFH cannot navigates the robot to this waypoint, causing failure of the entire algorithm.

## Running the Code
### Environment:
1. ubuntu  14.04 / 16.04

2. ROS indigo / kinetic

3. Jackal simulator

4. MATLAB r2017b or later version 

5. MATLAB robotics toolbox

### Steps:

1. Open a new terminal, run:
$ roslaunch jackal_gazebo jackal_world.launch config:=front_laser

2. Open MATLAB, run jackal_nav_gui.m

3. In the MATLAB GUI, click "Initialize" botton

4. Enter total time for map generation in the "set time" box and click "Start Mapping" botton

5. The robot will start moving and mapping the environment till the set time has been reached. A map will be generated in real time and displayed in the left figure.

6. In the generated map, areas marked white are explored available areas, areas marked black are obstacles, areas marked grey are unexplored region. 

7. Click on a point where you want the robot to go. Enter X, Y coordinates in the box above, and click "Gnerate Path"

8. A path will be gnerated and shown in red curve in the right figure. Click the "Navigate" botton below, the robot will start moving towards the goal position. A real time trajectory will be displayed in blue curve in the same figure.

9. After the robot arrives at the goal, the total time spent will be shown in the "Total Time" box, then you can re-enter the goal position and repeat step 6 to 8.
