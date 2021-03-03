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

...



