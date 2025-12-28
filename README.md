# UR10 Trajectory Optimisation

## Overview

This project focuses on optimizing the waypoints for the UR10 robot manipulator. The goal is to sort the waypoints so that the robot can reach all of them along the shortest possible trajectory. The algorithm implemented here is based on the Travelling Salesman Problem (TSP) with a heuristic approach, using the Nearest Neighbour method to select the next waypoint while marking visited points.

The project aims to reduce the total movement of the robot and improve efficiency in performing tasks involving multiple target positions.

## Features

- Generates waypoints in 3D space with random positions and orientations.
- Computes inverse kinematics (IK) solutions for each waypoint using MoveIt. IKFast is used and multiple solutions are obtained for the single pose. IKFast is used as a plugin and generated using openRave. 
- Implements a Travelling Salesman Problem (TSP) using the Nearest Neighbour heuristic.
- Produces optimized waypoint sequences for shortest robot trajectory.
- Saves waypoint positions and IK solutions in JSON files for further processing or visualization.

## Installation

1. Clone the repository:

```bash
git clone https://github.com/vinayakaraju46/ur10_trajectory_optimisation.git
cd ur10_trajectory_optimisation
```

2. Ensure your ROS workspace is sourced and MoveIt! is installed.

3. Build the package:

```bash
catkin_make
source devel/setup.bash
```

## Usage

1. Launch the UR10 robot and MoveIt! configuration.
2. Run the node to generate random waypoints and compute IK solutions:

```bash
rosrun random_markers random_markers_node
```

3. The node publishes visualization markers in RViz and saves the following JSON files:

- \`points.json\` – containing generated waypoint positions and orientations.
- \`ik_solutions.json\` – containing IK solutions for each waypoint along with pose numbers.

4. The TSP optimization computes the order to visit waypoints for minimal joint-space distance.

## Future Work

- Implement dynamic programming techniques to reduce repeated joint distance calculations.
- Explore alternative TSP heuristics and exact solutions for better optimization.
- Add analytics and comparisons between different TSP approaches.

## Repository Structure

- \`src/\` – ROS C++ nodes for waypoint generation, IK computation, and TSP optimization.
- \`CMakeLists.txt\` and \`package.xml\` – ROS package configuration files.
- \`README.md\` – Project documentation.
- JSON files – Output of generated waypoints and IK solutions.

## Video Demonstration

A demonstration of the UR10 trajectory optimisation is included in the repository. 

https://github.com/user-attachments/assets/5cc9d64f-9522-4384-a917-cf998a9d7482


## Time Results
### Unoptimized waypoints
<img width="521" height="301" alt="Screenshot 2025-12-28 at 15 56 51" src="https://github.com/user-attachments/assets/563ca6e8-50c1-4b22-8c08-f92e78983795" />

### Optimized waypoints
<img width="532" height="293" alt="Screenshot 2025-12-28 at 15 50 25" src="https://github.com/user-attachments/assets/1b430091-2efd-4ff3-a69d-de0474a0831d" />

## License

This project is open-source and available under the MIT License.
