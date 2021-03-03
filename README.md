# topoNBS

This branch contains a ROS-implementation of [Next-Best-Sense](https://ieeexplore.ieee.org/document/9113679) adapted for working on topological maps.
It relies on the ROS package [bayesian_topological_localisation](https://github.com/francescodelduchetto/topological_navigation/tree/master/bayesian_topological_localisation) for updating the belief over the objects tracked.

A video of the working principle of topoNBS combined with a topological particle filter for localizing human pickers in a polytunnel is hosted on YouTube.


[![topoNBS and TPF](https://i.imgur.com/B5cf7Eq.png)](https://www.youtube.com/watch?v=gLYIWcQfXHA&feature=youtu.be)

## Quick setup

This is a short description on how to install and run the components used for the topological localization presented above.

### Topological Navigation

 1. Install the LCAS ROS software packages: https://github.com/LCAS/rosdistro/wiki
 2. Setup `topological_navigation`: https://github.com/LCAS/topological_navigation/wiki/Quick-Topological-Navigation-in-simulation-tutorial
 3. How to use `topological_navigation`: https://github.com/LCAS/topological_navigation/blob/master/topological_navigation/README.md

### Topological Localization

The package is already installed if you have followed the previous step. Just follow the instructions to use: https://github.com/francescodelduchetto/topological_navigation/blob/master/bayesian_topological_localisation/README.md

## NBS

In order to launch the Next-Best-Sense framework for navigating the robot, you can simply use the following command

```bash
roslaunch next_best_sense pure.launch
```

If you have a look at the launch file, there are some interesting parameteres which can be tuned. In particular, 

```xml
<arg name="w_info_gain"         default="0.00"/>
<arg name="w_travel_distance"   default="0.2"/>
<arg name="w_sensing_time"      default="0.1"/>
<arg name="w_battery_status"    default="0.2"/>
<arg name="w_rfid_gain"         default="0.5"/>
```

are the five criteria currently implemented for choosing the next robot pose. A value in `[0,1]` can be assigned to each of them in order to express a relative preference among them. Details of how the criteria are combined is offered in the relative [paper](https://ieeexplore.ieee.org/document/9113679).

The following piece of code launch the topoNBS experiments for one or two RFID antennas (defined by the argument `two_readers`). It requires the package [RFID](https://github.com/LCAS/RFID/) in order to work with likelihood readings coming from the antennas.

```xml
<node pkg="next_best_sense" type="pure_navigation" name="nbs" output="screen"  args="$(arg pure_args)" unless="$(arg two_readers)">
    <param name="move_base_costmap_topic_name"          value="/map"/>
    <param name="move_base_costmap_updates_topic_name"  value="/move_base/global_costmap/costmap_updates"/>
    <param name="belief_map_srv_name"                   value="/thorvald/rfid_grid_map_node/get_rfid_belief"/>
    <param name="fake_belief_map_srv_name"              value="/thorvald/rfid_grid_map_node/get_rfid_fake_belief"/>
</node>
<node pkg="next_best_sense" type="pure_navigation_two_readers" name="nbs" output="screen"  args="$(arg pure_args)" if="$(arg two_readers)">
    <param name="move_base_costmap_topic_name"          value="/map"/>
    <param name="move_base_costmap_updates_topic_name"  value="/move_base/global_costmap/costmap_updates"/>
    <param name="belief_map_srv_name_left"              value="/thorvald_left/rfid_grid_map_node/get_rfid_belief_left"/>
    <param name="belief_map_srv_name_right"             value="/thorvald_right/rfid_grid_map_node/get_rfid_belief_right"/>
    <param name="fake_belief_map_srv_name_left"         value="/thorvald_left/rfid_grid_map_node/get_rfid_fake_belief_left"/>
    <param name="fake_belief_map_srv_name_right"        value="/thorvald_right/rfid_grid_map_node/get_rfid_fake_belief_right"/>
</node>
```



