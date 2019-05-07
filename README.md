# GoodBot Navigation Manager

The Navigation Manager takes scene and intent information as parsed by the Intent Recognition and Scene Understanding software, and combines it with odometry information from the Movo platform to compute goal locations in the environment. These goal locations are published to `/movo/move_base_simple/goal` as ROS `Odometry` messages.

## Running the Navigation Manager

To run the navigation manager software, make sure that you have build the package in your catkin workspace, and sourced the workspace, then simply run

```
$ roslaunch navigation_manager nav_manager.launch
```

The node will receive odometry information, and wait to receive information about a POI that should be navigated to.

To run another experiement or navigate to another goal, it is currently required to restart the navigation manager.

## Running Movo Navigation

The ROS launch file necessary to run the required navigation software on the Movo platform is also included in this repository, with the name `map_nav.launch`. This launch file starts the Movo move_base interface, the navigation stack, and a map server with an empty map. This map server and empty map are necessary to send arbitrary goal points to the navigation stack.

_Note: launching these packages has only been tested running the launch file on the Movo platform, but the result should be same runninng on any machine on the ROS network, if that machine has the movo software stack built._

To launch the navigation software run

```
$ roslaunch navigation_manager map_nav.launch
```