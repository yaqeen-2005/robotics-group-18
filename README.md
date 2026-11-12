# Autonomous Pallet Transport Robot for a Simulated Warehouse 
Group 18: Peifeng Xu, Sirui Meng, Thomas Morris, Yaqeen Bharoochi, Yi Gu. 
Objective: Develop and evaluate an autonomous pallet transport robot that fulfils pickup and 
delivery requests in a simulated warehouse while avoiding obstacles and reported hazards. 
Scenario and Approach 
Our project will involve adapting an existing robot model, such as Clearpath’s Ridgeback, to 
autonomously transport pallets between pickup and delivery points in a simulated warehouse.  
The robot will be on wheels with a flat platform acting as the pallet carrying area.  It will first 
explore the unknown warehouse using SLAM to build a map for subsequent delivery tasks.  
Reported spills may be manually reported by workers and act as a forbidden area.  Manual 
control of the robot will be possible, temporarily disabling autonomous navigation and allowing it 
to be navigated out of situations it cannot handle.  Loading and unloading will be represented by 
the simulated attachment and removal of pallets while the robot is stationary.  The workflow will 
be as follows: receive a transport request, navigate to the pick-up location, wait for loading 
confirmation, navigate to the delivery destination, wait for unloading confirmation, then navigate 
back to the original waiting point.  We will use Gazebo for our simulation, ROS 2 for 
communication, Python for task control, and Nav2 for navigation. 

## System Requirements

This project targets the following software versions:

- Ubuntu 24.04.5 LTS (Noble Numbat)
- ROS 2 Jazzy Jalisco
- Gazebo Harmonic, using the `gz-harmonic` metapackage and Gazebo Sim (`gz-sim`) 8.15.0

ROS 2 Jazzy and Gazebo Harmonic are the recommended stable ROS/Gazebo combination for Ubuntu 24.04 LTS.

## Minimum Working Version 
Our minimum requirements are autonomous SLAM mapping of an initially unknown warehouse, 
followed by localisation and navigation.  The robot will plan routes on its SLAM-generated map 
and distinguish temporary from persistent obstacles to wait, avoid or replan.  For the main 
functionality of the robot, it must be able to lift the crate using a simplified attachment 
mechanism, transport and release the crate. For our component interfaces, we will use LiDAR 
and odometry for the localisation, estimated pose and map for the pathfinding algorithm and a 
transport controller will manage crate pickup and delivery. 
## Evaluation 
To evaluate the success of our project, we must conduct repeated trials with varied crate 
locations, destinations and obstacles, Measure successful crate collection and delivery rate. 
We can evaluate localisation accuracy against Gazebo ground-truth position and the efficiency 
of paths by measuring route length, delivery time and collision count through either A* or fixed 
path algorithms. Evaluating the robot's ability to replan when unexpected obstacles block its 
route will be key due to the busy nature of a warehouse. The main success criteria and bare 
minimum is whether or not it can successfully collect and deliver at least one crate, navigate 
without collisions and complete a delivery using an automatically calculated route. 
## Plan 
Thomas: Develop robot, lifting mechanism and warehouse simulation.  
- Xu: Implement SLAM mapping and localisation using LiDAR and odometry. 
- Yi: Develop A* pathfinding and autonomous navigation. 
- Yaqeen: Implement crate pickup, transport and delivery mechanisms. 
- Sirui: Implement obstacle avoidance, integration, testing and evaluation.  
 ● Main Risk: Integrating localisation, pathfinding and crate pickup within five weeks.  
 ● Fallback: Use predefined pickup and delivery stations with a simplified crate attachment 
mechanism, while retaining autonomous pathfinding and obstacle avoidance. 
 ● Reference: Hart, Nilsson and Raphael (1968), A Formal Basis for the Heuristic 
Determination of Minimum Cost Paths. IEEE Transactions on Systems Science and 
Cybernetics, 4(2), 100–107.
