# DecompROS2

ROS2 wrapper for [DecompUtil](https://github.com/sikang/DecompUtil)

The DecompUtil code has been included here directly (instead of as a submodule) since I have made some tiny changes to the DecompUtil library. 

## Installation
simply clone and build:
```
cd colcon_ws/src
git clone git@github.com:dev10110/DecompROS2.git
cd colcon_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
(building in Release mode is important for performance)

## Usage
The library provides mainly a single composable node that listens to pointclouds and returns the Safe Flight Corridor associated with it. At the moment, only the seed decomposition has been ported, but it should be fairly easy to port other decomposition techniques. 

Using a launch file:
```
ros2 launch decomp_ros seedDecomp.launch.py
```
but really you should write a similar launch file that also contains the node that generates the pointcloud. If you do so, it will significantly reduce the memory and bandwidth requirements since the data will be directly shared rather than sent over the network. 

You can also launch it as a single node
```
ros2 run decomp_ros seedDecomp_node
```
The output will be a `decomp_ros::msg::PolyhedronStamped` msg. The `decomp_ros::msg::Polyhedron` msg contains a list of points `ps` and normals `ns` which defines the polyhedron by
```
S = \{ x : ns[i] . x <= ns[i].ps[i] for all i \}
```

To visualize the polyhedron in `rviz2`, run the visualization node:
```
ros2 run decomp_ros vizPoly_node
```
and this can also be run as a composable node. 

This will publish a `visualization_msgs::msg::Marker` which contains a list of lines, and marks the edges of the polyhedron. Unfortunately, this node needs to find all the vertices of the polyhedron so it can be an expensive operation. 

## Disclaimer
This is very much research code. It works, but do not trust it. 
