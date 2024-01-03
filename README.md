# Bath-RosBot Project

## Project Overview
This project develops control software for the ROSBot robot to autonomously navigate a challenging obstacle course. The objective is to achieve intelligent navigation without prior knowledge of the course, using the robot's sensors for real-time data processing.

## Key Features
- **Autonomous Navigation:** Utilizes a finite state machine (FSM) for decision-making and course navigation.
- **Sensor Integration:** Employs LIDAR, ultrasonic sensors and encoders for obstacle detection and distance measurement.
- **Course correction:** Provides course correction using orientation data from IMU sensor.

## Implementation
The main source code is in `src/test_move.py`, featuring FSM logic for controlling the robot's movements. This includes state management for various navigation scenarios and dynamic obstacle avoidance.

## Testing and Results



