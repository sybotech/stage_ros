# Stage bindings to ROS #

Forked from https://github.com/ros-simulation/stage_ros

Contains additional features:
 - persistent scene. Scene is saved to separate location and used on next load. That allows to save simulation state between server reboots
 - you can ask sim to publish ground map in form of nav_msgs::OccupancyGrid
 - you can ask sim to publish robot coordinates relative to common TF root (like map)
 - improved multi-robot support

TODO: describe export modes
TODO: describe TF tree
TODO: describe output topics
