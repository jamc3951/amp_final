# amp_final
Final Project for ASEN 5519

Usage: ROS Kinetic, Python 2.7

Requires: generated .pkl file!

To Run: 'roslaunch policy.launch'

Main Components:

- PolicyServer.py: ROS node that accesses data from .pkl, runs MCTS. Communication via msg/srv.
- Controller.py: ROS node, runs RRT based algorithms
- rrt.py: Contains all rrt algorithms for solving and plotting

All other .py files are accessory
