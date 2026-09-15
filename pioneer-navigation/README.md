Pioneer Navigation

This folder contains the Python control software developed for autonomous navigation of a Pioneer P3-DX mobile robot.

The implementation supports both MobileSim simulation and physical Pioneer robot communication, using the P2OS protocol over TCP or serial communication.

Navigation Features

The navigation system includes:

* Odometry-based waypoint navigation
* ALIGN → MOVE control behaviour
* Position and heading estimation from robot feedback
* Configurable waypoint coordinates
* MobileSim TCP communication
* Physical robot serial communication
* Sonar-based obstacle detection
* Colour-based target tracking
* Manual robot control through a graphical interface
* Navigation data logging and export

Odometry-Based Waypoint Navigation

The robot follows a predefined sequence of waypoints expressed in millimetres relative to the starting position.

The default navigation sequence is:

| Point | X (mm) | Y (mm) |
|------|-------:|-------:|
| P1 | 2060 | 0 |
| P2 | 2060 | -2530 |
| P3 | 4060 | -2530 |
| P4 | 4060 | -3790 |
| P5 | -1880 | -3790 |

The controller uses an ALIGN → MOVE strategy:

1. Calculate the bearing from the current robot position to the target waypoint.
2. Calculate the heading error.
3. Rotate in place while the heading error is above the configured threshold.
4. Move forward while applying steering correction.
5. Mark the waypoint as reached when the robot enters the configured distance tolerance.
6. Continue to the next waypoint.

The controller uses proportional gains for rotational and steering corrections, with configurable angular and distance thresholds.

### Robot Communication

The navigation software implements low-level communication with the Pioneer P3-DX using the P2OS packet protocol.

Two communication modes are supported:

### MobileSim

The robot can connect to MobileSim through TCP.

Default configuration:

- Host: 127.0.0.1
- Port: 8101

### Physical Pioneer Robot

The same control framework can communicate with the physical robot through a serial connection.

The serial port and baud rate can be configured through the control software.

### Odometry and Robot State

Robot feedback is parsed to obtain:

* X position
* Y position
* Heading
* Left wheel velocity
* Right wheel velocity
* Linear velocity
* Angular velocity

The navigation controller uses the robot’s reported pose to determine its position relative to the initial starting point.

Colour-Based Target Tracking

The integrated controller also contains a camera-based target tracking system implemented using OpenCV.

The tracking pipeline:

1. Capture frames from the camera.
2. Select a region of interest (ROI) containing the target.
3. Learn the target colour in HSV space.
4. Generate a colour mask.
5. Apply filtering and morphological operations.
6. Detect the largest valid contour.
7. Calculate the target centroid.
8. Determine whether the target is to the left, centre, or right of the image.
9. Command the robot to turn or move towards the target.

The controller uses a smoothed target position and a configurable deadband to reduce unstable left/right switching.

Sonar-Based Obstacle Avoidance

The Pioneer sonar sensors are used to detect obstacles during navigation.

The integrated controller includes:

* Side obstacle detection
* Front emergency stopping
* Short-duration avoidance actions
* Safety distance thresholds
* Direction selection based on the detected obstacle position

When an obstacle is detected close to the robot, the avoidance behaviour can override the normal tracking command.

Graphical Control Interface

pioneerMR.py provides a Tkinter-based graphical control panel.

The interface supports:

* Simulation or physical robot connection
* Manual driving
* Odometry mode
* Tracking mode
* Automatic operation
* Camera control
* Sonar monitoring
* Waypoint reset
* Robot stop controls
* Navigation parameter tuning
* Navigation data logging
* CSV/Excel export

Keyboard controls are also supported for manual operation.

Pose Smoothing Experiments

The controller includes two experimental pose-smoothing approaches:

* Extended Kalman Filter (EKF)
* Particle Filter (PF)

These were implemented as smoothing/experimental comparison layers rather than the primary odometry controller.

The odometry navigation behaviour continues to use the raw robot odometry for control, while the EKF and particle filter can be used for display and logged comparisons.

Files

File	Description
pioneer2.py	Standalone odometry waypoint navigation controller
pioneerMR.py	Integrated Pioneer control application with odometry, tracking, sonar handling, GUI and experimental pose smoothing

Main Technologies

* Python
* P2OS robot communication
* TCP/IP
* Serial communication
* Tkinter
* OpenCV
* NumPy
* MobileSim
* Pioneer P3-DX

Related Project Components

The complete project also includes:

* A* path planning → astar-path-planning/
* Webots simulation → webots/
* Mapper3 map → maps/
* Demonstration videos → media/
* Experimental results → results/

Project Context

This navigation software was developed as part of a mobile robotics project investigating autonomous navigation using a Pioneer P3-DX platform.

The implementation was tested using both simulation and physical-robot environments, with waypoint navigation forming the first stage of the overall navigation architecture.
