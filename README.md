# Astar-Project-Udacity-Cpp

## Files:

BUILD:
Exe Files.

CMAKE:
Targets for Cmake.

SRC:
Main.cpp: Contains main code.
model.cpp & model.h: Contain the class node and the related members. 
route_model.cpp & route_model.h: Contains the other classes as well related to paths and related members.
route_planner.cpp & route_planner.h: Contains the AStar algorithm and related functions.

## Theory:

This project was used to construct a path planning algorithm using AStar method.
The steps followed were:

1) Taking pre-existing map data and storing the paths and ways accordingly.
2) Creating a class called node with attributes such as g value, h value, visited flag, etc along with related methods.
3) The node class were then used to translate the data from the map to the code.
4) The starting node was taken as the node closest to the starting coordinates given.
5) Then all neighbours were considered for the starting node based on the roadway that it was a path of.
6) This list of nodes was then sorted by the f value. (f = g+h)
7) Then node with smallest f was considered and it's neighbors were added to the open list.
8) This was done till the end node was reached and then the final path was constructed using the IO2d library.

## Keywords:

C++, Path Planning, AStar, Pointers, References, OOP, Vectors, Map integration, IO2D.
