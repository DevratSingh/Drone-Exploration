## Autonomous Exploration of Crazyflie Drone in a Known Environment
<p align="justify">
The main goal of this project was to allow for a "Crazyflie" drone to autonomously navigate and explore a known environment. The environment is given in terms of a map which contains the positions of walls and landmarks. Accordingly, three main components, namely  Perception, Localization, and Planning are integrated to allow for the system to navigate. The perception part uses a Deep Neural Network (SqueezeNet) to detect and classify the landmarks and then publishes the 6D pose of the perceived landmarks in the map. This is then fed into the localization component, which uses Kalman filter to integrate these measurements in order to provide the current pose of the drone. Finally, the planning part uses the map and the drone location to plan 3D obstacle free paths. An integral part of the planning system is the RRT-based exploration which ensures that all the areas within the map has been explored.
</p> 

<p align="justify">
This project was implemented using ROS, with the implemented components being gradually tested first on a Gazebo simulation and then finally integrated to test on the real drone.
</p> 

#### Exploration through KF localization, Traffic sign detection using CNN and RRT-based exploration 
[![DD2419ph1](https://github.com/DevratSingh/Myprojects/blob/main/Images/thumbph1.png)](https://youtu.be/8_smu3H0q7g "Project Video")

[System Design Figure](SysDiagph1.png)

[Brief System Design Report](Brief%20System%20Design%20Report%20_Crazyflie%20Project.pdf)
