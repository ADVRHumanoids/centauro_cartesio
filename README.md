# centauro_cartesio
Package containg CartesI/O addons relared with the Centauro robot

## Wheeled motion
Based on the following article
```
A. Laurenzi, E. M. Hoffman, M. P. Polverini and N. G. Tsagarakis, 
"An Augmented Kinematic Model for the Cartesian Control of the Hybrid Wheeled-Legged Quadrupedal Robot CENTAURO," 
in IEEE Robotics and Automation Letters, vol. 5, no. 2, pp. 508-515, April 2020.
```
0) optionally run XBotCore/Gazebo (otherwise, just visualization)
1) `mon launch centauro_cartesio centauro_car_model.launch` (if XbotCore is running, this send commands to it via ROS)
2) the Cartesio task named `car_frame` is used to move the robot around

Note: the `mon` command is available inside the `rosmon` ROS package (`sudo apt-get install ros-$ROSDISTRO-rosmon`)
