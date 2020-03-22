# 2D-RRT-star

This repository includes a simulation code for 2D RRT*. 

To make this code run, you need to:
1. Run "roscore"
2. Run "rosrun rviz rviz"
3. Run "rosrun path planning env_node" (which contains the information about the boundary)
4. Run "rosrun path planning rrt_node" (which contains the 3D Informed RRT* path planner)

RVIZ parameters:  <br  />
1. Frame_id = "/path_planner"  <br  />
2. marker_topic = "path_planner_rrt"  <br  />

Once you have successfully run the path planner, the simulation should be like:

<p align="center">
  <img width="300" height="280" src="https://github.com/jiaweimeng/2D-RRT-star/blob/master/simulation_2d.png">
</p>

<p align="center"> 
  Figure 1. Simulation of 2D RRT star path planner: final path is indicated by blue, start point is indicated by red, goal point is indicated by green, rrt tree is indicated by orange and obstacle is indicated by black.
</p>

