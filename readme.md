# Stage bindings to ROS #

Forked from https://github.com/ros-simulation/stage_ros

Contains additional features:
 - persistent scene. Scene is saved to separate location and used on next load. That allows to save simulation state between server reboots
 - you can ask sim to publish ground map in form of nav_msgs::OccupancyGrid
 - you can ask sim to publish robot coordinates relative to common TF root (like map)
 - improved multi-robot support
 - does not need FLTK. But right now you need [patched Stage](https://github.com/sybotech/stage/tree/headless_without_fltk)

# Nodes #

 - stageros - default executable to be built. Uses only headless part of the stage.
 - stagerosg - contains GUI code. This node will be built only if stage has GUI support.

# Parameters #

 - ~common\_root (bool, default: false) - flag to start publishing TF transforms from common 'map' root to each exported robot, like map -> /era1/odom, map -> /era2/odom
 - ~control\_timeout (double, default: 0.2) - timeout for robot controls. Robot will stop automatically, if there were no commands during that time.
 - ~export (varies) - current export mode. Set to '1' to enable multi robot export. Delete this parameter if you need simple export mode
 - ~is\_depth\_canonical (bool, default: true) -
 - ~root\_frame\_id (string, default: "map") - name of TF frame to be used to publish robot transforms and the map
 - ~rate (double, default: 20.0) - update rate for simulation and data publishing
 - ~persistent\_file (string, default: "") - path to file to be used for persistent storage of scene state
 - ~control\_acceleration (bool, default: true) - flag to enable acceleration control mode
 - ~map\_publish\_period (bool, default: 2.0) - period for occupancy grid to be published
 - ~map\_resolution (double, default: 0.05) - resolution of generated occupancy grid
 - ~map\_model\_name (string, default: "ground") - name of the model to be used for generation of occupancy grid



# Exporting data to ROS #

Export modes:

 - ExportManual = 0 - manual export. Should call a service to export another robot. To be implemented
 - ExportSimple = 1 - export the first position model found in a world. All topics are created relative to 'global' namespace. Raises an error if no models found
 - ExportSelected = 2 - Export only the robots from specified tag list. Should set rosparam `export` parameter to array of strings 
 - ExportAll = 3 - Export all position models from the world. It will create ROS namespace for each robot. Model token is used for a name. To be implemented

TODO: describe output topics

'ranger' model is exported to 'scan' topic, inside the robot's namespace. If robot has multiple rangers, they are mapped to 'scan0', ... 'scanN' topic. Output topic will be attached to 'laser\_link' TF frame, or 'laser\_link0' ... 'laser_linkN' in case of multiple sensors

Emulator will publish the folliwing TF chain:
`odom->base_footprint->base_link->sensor_link`

In case of multi-robot export, each odom, base\_footprint and base\_link frames will have robot's name as prefix, like:
/era1/odom -> /era1/base\_footprint -> /era1/base\_link


# Examples #

## Exporting multiple robots ##

```
# Example rangefinder model
define topurg ranger
(
  sensor(
    range [ 0.0  30.0 ]
    fov 270.25
    samples 1081
  )
  ...
)

# Example robot with rangefinder
define erratic position
(
  ...
  gui_nose 1
  drive "diff"
  topurg ( pose [ 0.100 0.000 0.400 0.000 ] )
  ...
)

# This one will have default stage name position.0 that will be mapped to poisition_0
erratic( pose [ -12.331 -0.547 0.000 -171.406 ] color "gray")
# This robot will be expoted to era1/ namespace, for all chind sensors and TF frames
erratic( pose [ -1.696 9.119 0.000 180.000 ] name "era1" color "gray")
# This robot will be expoted to era2/ namespace, for all chind sensors and TF frames
erratic( pose [ -1.655 7.280 0.000 180.000 ] name "era2" color "white")
```

When we run `rosrun stage_ros stageros example.world _export:=1`, we will get the following topics exported:

```
/position0/scan
/position0/odom
/position0/cmd_vel
/era1/scan
/era1/odom
/era1/cmd_vel
/era2/scan
/era2/odom
/era2/cmd_vel
```

## Exporting the map ##

Stage can export map and localization data

Example for a world file:

```
define floorplan model
(
...
)

# Example of floorplan to be exported to ROS
floorplan
( 
  name "ground"
  bitmap "hospital_section.png"
  size [ 54.300 21.650 3.000 ]
  pose [ 0.000 0.000 0.000 0.000 ]
)
```

Example for running this world: 
`rosrun stage_ros stageros example.world _map_model_name:=ground`
