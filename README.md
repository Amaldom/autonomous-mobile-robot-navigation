Autonomous Mobile Robot Navigation & Path Planning

A mobile robotics project exploring autonomous navigation using a Pioneer 3-DX robot, combining odometry-based waypoint navigation, colour-based target tracking, reactive sonar obstacle avoidance, and A* path planning.

The project was developed using Python, MobileSim, Mapper3, and Webots, with testing performed in both simulated and physical-robot environments.

⸻

🚀 Project Overview

Autonomous mobile robots must be able to navigate towards a goal while maintaining reliable movement and responding to obstacles in their environment.

This project implemented a behaviour-based navigation system for a Pioneer 3-DX mobile robot. The navigation task was divided into three main behaviours:

1. Waypoint Navigation using Odometry
2. Colour-Based Target Tracking
3. Sonar-Based Obstacle Avoidance

In addition, an A path-planning algorithm* was implemented and evaluated separately in Webots using a custom 3D environment.

The overall project demonstrates how planned navigation, visual perception, and reactive sensing can be combined to support autonomous robot behaviour.

⸻

🤖 Robot Platform

The experiments used an Adept/MobileRobots Pioneer 3-DX differential-drive mobile robot.

Hardware

* Pioneer 3-DX mobile robot
* Differential-drive wheel system
* Wheel encoder feedback for odometry
* 8 active ultrasonic sonar sensors
* Laptop webcam for visual target tracking
* Onboard robot controller
* USB serial communication with external computer

Software

* Python
* MobileSim
* Mapper3
* Webots
* Visual Studio Code

⸻

🧭 Navigation Architecture

The main navigation task followed a sequential behaviour structure:

                    Start
                      │
                      ▼
            ┌───────────────────┐
            │ Odometry-Based    │
            │ Waypoint Navigation│
            └─────────┬─────────┘
                      │
                   Node N5
                      │
                      ▼
            ┌───────────────────┐
            │ Colour-Based      │
            │ Target Tracking   │
            └─────────┬─────────┘
                      │
                      ▼
            ┌───────────────────┐
            │ Sonar Obstacle    │
            │ Avoidance         │
            └─────────┬─────────┘
                      │
                      ▼
                    Goal

The robot first navigated through predefined waypoints using wheel-encoder odometry. After reaching the final waypoint, the system switched to vision-based tracking of an orange target while sonar sensing provided reactive obstacle avoidance.

⸻

1. Odometry Waypoint Navigation

The first navigation phase used wheel encoder feedback to estimate the robot’s displacement and orientation.

The robot followed a predefined sequence of waypoints:

Node	X (mm)	Y (mm)
N0	0	0
N1	2060	0
N2	2060	-2530
N3	4060	-2530
N4	4060	-3790
N5	-1880	-3790

The robot aligned itself at each waypoint before continuing to the next segment. These intermediate points also provided opportunities to reduce accumulated odometry drift.

Key concepts

* Wheel encoder odometry
* Position estimation
* Heading control
* Waypoint following
* Re-alignment at intermediate nodes
* Odometry drift management

⸻

2. Colour-Based Target Tracking

After reaching waypoint N5, the navigation behaviour switched from waypoint navigation to visual target tracking.

A laptop webcam was used to identify an orange target.

The tracking pipeline consisted of:

Camera Frame
     │
     ▼
Colour Thresholding
     │
     ▼
Orange Region Detection
     │
     ▼
Largest Orange Area
     │
     ▼
Centroid Calculation
     │
     ▼
Steering Decision
     │
     ▼
Robot Movement

The centroid of the detected orange region was compared with the camera centreline.

The robot adjusted its steering depending on the target’s position in the image and continued moving towards the target.

⸻

3. Sonar-Based Obstacle Avoidance

Reactive obstacle avoidance was implemented using the Pioneer 3-DX’s ultrasonic sonar sensors.

When an obstacle was detected within the defined proximity range, the robot performed an avoidance manoeuvre before attempting to continue along its intended navigation behaviour.

The system used the sonar measurements to select an appropriate avoidance direction and subsequently restore forward navigation.

Key concepts

* Ultrasonic sensing
* Reactive obstacle detection
* Left/right avoidance behaviour
* Real-time sensor response
* Recovery to the navigation task

⸻

4. MobileSim & Mapper3

The physical test environment was reproduced in MobileSim to allow preliminary testing before physical-robot deployment.

Mapper3 was used to generate the map of the experimental environment.

The simulated environment contained indoor features including:

* Walls
* Corners
* Doorway
* Central obstacles

The same general environment was used to compare simulated and physical robot behaviour.

⸻

5. A* Path Planning

As an additional contribution, an A path-planning algorithm* was implemented in Python and evaluated in Webots.

The objective was to generate a safe and efficient route between a start position and a goal position in an environment containing obstacles.

Planning pipeline

Environment
     │
     ▼
Occupancy Grid
     │
     ▼
Obstacle Inflation
     │
     ▼
A* Search
     │
     ▼
Wall-Penalty Cost
     │
     ▼
Path Generation
     │
     ▼
Waypoint Reduction
     │
     ▼
Webots Robot Controller

⸻

⭐ A* Implementation Features

Euclidean Heuristic

The implementation used Euclidean distance as the heuristic for estimating the remaining distance to the goal.

The A* cost function was:

f(n) = g(n) + h(n)

where:

* g(n) = cost accumulated from the start
* h(n) = estimated cost to the goal

8-Directional Movement

The planner allowed movement in eight directions rather than restricting the robot to four-connected grid movement.

Additional checks were implemented to prevent invalid diagonal movement through obstacles.

Obstacle Inflation

Obstacles were inflated to introduce a safety margin around blocked regions.

This helps prevent the planned robot path from passing too close to obstacles.

Wall Penalty

A wall-penalty function introduced additional cost for nodes located close to obstacles.

This encouraged the planner to favour paths with greater clearance and produced smoother, more practical routes.

Waypoint Reduction

The generated path was simplified by reducing the number of waypoints while retaining the overall shape of the planned route.

This makes the resulting path more suitable for point-to-point robot movement.

⸻

6. Webots Simulation

The A* planner was evaluated in a custom-built 3D Webots environment.

The architecture separated path planning from robot control:

Python A* Planner
       │
       ▼
Generated Waypoints
       │
       ▼
Waypoint File
       │
       ▼
Webots Controller
       │
       ▼
Pioneer Robot
       │
       ▼
Goal

The Webots controller reads the generated waypoints and commands the simulated Pioneer robot to move between them.

⸻

📊 Evaluation

### Target Tracking Results

The colour-tracking experiment was evaluated over 10 trials.

| Metric | Mean | Standard Deviation |
|---|---:|---:|
| X Position | -3482.7 mm | 15.569 mm |
| Y Position | -477.9 mm | 19.628 mm |
| Distance Error | 23.44 mm | 9.478 mm |
| Completion Time | 123.34 s | 8.473 s |

The reported correlation coefficient between distance error and completion time was **0.97**.

The A* implementation was also evaluated in Webots. The generated path successfully navigated around obstacles and reached the goal without collision in the demonstrated simulation.

⸻

🎥 Demonstrations

The repository contains demonstrations of:

* Odometry waypoint navigation
* Colour-based target tracking
* A* path planning and Webots navigation

Videos and screenshots are provided as project evidence and demonstrations of the implemented system.

⸻

🛠️ Technologies

Programming

* Python

Robotics

* Pioneer 3-DX
* Differential-drive navigation
* Odometry
* Ultrasonic sonar sensing
* Behaviour-based control

Computer Vision

* Colour thresholding
* Target detection
* Centroid-based steering

Path Planning

* A*
* Occupancy grids
* Euclidean heuristic
* 8-directional search
* Obstacle inflation
* Wall-penalty cost
* Waypoint reduction

Simulation & Mapping

* MobileSim
* Mapper3
* Webots

⸻

📁 Repository Structure

```text
autonomous-mobile-robot-navigation/
├── README.md
├── requirements.txt
├── .gitignore
├── LICENSE
│
├── pioneer-navigation/
│   ├── pioneer2.py
│   ├── pioneerMR.py
│   └── README.md
│
├── astar-path-planning/
│   ├── MRcode.py
│   └── README.md
│
├── webots/
│   ├── MR_controller.py
│   ├── mycontri.wbt
│   └── README.md
│
├── maps/
│   └── projectM.map
│
├── media/
│   ├── odometry-navigation.mp4
│   ├── colour-tracking.mp4
│   └── webots-simulation.mp4
│
└── results/
    └── figures/
```
⸻

🔬 Key Learning Outcomes

This project provided practical experience in:

* Mobile robot kinematics and odometry
* Autonomous waypoint navigation
* Sensor-based reactive control
* Basic computer vision for robot perception
* Path planning using A*
* Occupancy-grid processing
* Simulation-based robotics development
* Integrating planning and robot control
* Evaluating robot navigation experimentally

⸻

🚧 Limitations

The current implementation primarily targets known, static environments.

The colour-tracking approach also relies on detecting a predefined colour target, while the A* planner was evaluated separately within the Webots simulation.

⸻

🔮 Future Improvements

Potential extensions include:

* SLAM-based navigation
* Sensor fusion
* More robust visual target detection
* Dynamic obstacle handling
* Real-time path replanning
* Multi-sensor perception
* Improved localisation
* More sophisticated motion planning
* Physical-robot deployment of the A* navigation pipeline

⸻

👤 Author

Amal Dominic

MSc Robotics & Automation
University of Salford, UK

⸻

📌 Project Context

This project was developed as part of postgraduate study in Robotics & Automation and has been reorganised into a technical portfolio project to demonstrate practical experience in autonomous mobile robotics, navigation, perception, sensing, path planning, simulation, and Python-based robot control.
