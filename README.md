### Time Optimal Trajectory Planning in Unknown Environment
This project was undertaken as a part of Motion Planning Project for Spring 2022

Requirements: 
Before running the code make sure that you have the following:
  1. ROS Noetic (Complete Package)
  2. [Gurobi Optimizer](https://www.gurobi.com/products/gurobi-optimizer/)
  3. Turtlebot 3 

After installing all the required files and libraries. 
Make Sure you build your workspace using ```catkin build ``` and source it.

Run the following commands to start Gazebo and Rviz:
```
roslaunch trajectory_planner ground_robot.launch
roslaunch global_mapper_ros global_mapper_node.launch
roslaunch trajectroy_planner interface.launch
roslaunch trajectroy_planner planner.launch
```
To give the goal position to the robot, open the rviz window, press G and then click anywhere on the grid. This point will be taken as the goal configuration and thereafter planner starts planning and turtlebot makes its way to this point.

![Screenshot from 2023-11-07 22-21-24](https://github.com/cskate1997/TrajectoryPlannerUnknownEnvironment/assets/94412831/f1a18a99-16b0-4906-9e0e-c662b58bf4f1)
