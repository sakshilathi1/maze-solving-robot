# 🤖 Maze Solving Robot

<div align="center">

![MATLAB](https://img.shields.io/badge/MATLAB-R2023a+-orange.svg)
![Simulink](https://img.shields.io/badge/Simulink-Robotics_Toolbox-blue.svg)
![Status](https://img.shields.io/badge/Status-Complete-brightgreen.svg)

**Autonomous maze-solving system using Mycobot Pro 600 with computer vision and path planning**

[Overview](#-overview) • [Demo](#-demo) • [Architecture](#-architecture) • [Installation](#-installation) • [Usage](#-usage)

</div>

---

## 📖 Overview

This project implements an **autonomous maze-solving system** using the Mycobot Pro 600 collaborative robot. The system captures a maze image via camera, processes it using computer vision, finds the shortest path using **Dijkstra's algorithm**, and guides the robot's end-effector through the maze using **inverse kinematics**.

### Key Features

- 🎯 **Computer Vision** - Detects maze, start (red), and goal (green) points using HSV filtering
- 🛤️ **Path Planning** - Dijkstra's algorithm finds the shortest path avoiding obstacles
- 🦾 **Inverse Kinematics** - Generalized IK solver computes joint angles for robot motion
- 🖥️ **Digital Twin** - MATLAB/Simulink simulation validates movements before physical deployment
- ⚡ **Real-time Control** - TCP commands control the physical robot via Ethernet

### Performance

| Metric | Result |
|--------|--------|
| Maze Completion Time | < 1 minute |
| Path Accuracy | No wall collisions |
| Path Optimality | Shortest path guaranteed |

## 🎬 Demo

**Video Demonstrations:**
- [Maze 1 Solution](https://drive.google.com/file/d/1SGUauAAnzm52wTT2bye0lF3kd9r02C1d/view)
- [Maze 2 Solution](https://drive.google.com/file/d/12n77chQ8feZQnO4TgxdxiJE_mDu46aCH/view)
- [Maze 3 Solution](https://drive.google.com/file/d/1CSNxA7ySV_TKTQYoXODhl2ewi53KrgOG/view)

## 🏗️ Architecture

### System Pipeline

```
Camera Capture → Image Processing → Path Planning → Inverse Kinematics → Robot Control
      │                │                  │                 │                │
   Maze Image    HSV Filtering      Dijkstra's         GIK Solver      TCP Commands
                 Thresholding       Algorithm          Joint Angles     to Mycobot
```

### Workflow

1. **Image Acquisition** - Camera captures maze workpiece (15cm x 15cm)
2. **Preprocessing** - Convert to grayscale, adaptive thresholding, binary mask
3. **Point Detection** - HSV filtering identifies red (start) and green (goal) circles
4. **Pathfinding** - Dijkstra's algorithm computes shortest path avoiding walls
5. **Coordinate Transform** - Pixel coordinates → Physical coordinates (projective transformation)
6. **Inverse Kinematics** - GIK solver computes 6 joint angles for each path point
7. **Simulation** - Digital twin validates path in MATLAB/Simulink
8. **Execution** - TCP commands sent to robot via Ethernet

## 📁 Project Structure

```
maze-solving-robot/
├── gikmazefinal.m          # Main MATLAB script (image processing + IK + path planning)
├── interparc.m             # Path interpolation utility
├── test_tcp.m              # TCP communication test script
├── myCobot28.urdf          # Robot model definition
├── myCobot28DT.slx         # Digital Twin Simulink model
├── myCobot28FK.slx         # Forward Kinematics Simulink model
├── myCobot28IK.slx         # Inverse Kinematics Simulink model
├── meshes/                 # Robot 3D mesh files (STL)
│   ├── base_link.STL
│   ├── link1.STL
│   ├── Link2.STL
│   ├── Link3.STL
│   ├── Link4.STL
│   ├── Link5.STL
│   └── Link6.STL
├── maze*.jpg               # Sample maze images for testing
├── xaxiscobot.mat          # Calibration data (X-axis)
├── yaxiscobot.mat          # Calibration data (Y-axis)
└── zaxiscobot.mat          # Calibration data (Z-axis)
```

## 🚀 Installation

### Prerequisites

- MATLAB R2023a or later
- Robotics System Toolbox
- Image Processing Toolbox
- Simulink (for digital twin)

### Hardware Requirements

- Mycobot Pro 600 robot
- USB camera
- Ethernet cable (RJ45)
- 15cm x 15cm maze workpiece

### Setup

```bash
# Clone the repository
git clone https://github.com/sakshilathi1/maze-solving-robot.git
cd maze-solving-robot

# Open MATLAB and navigate to project folder
# Run the main script
```

## 💻 Usage

### 1. Connect to Robot

```matlab
% Set robot IP (check robot's configuration panel)
robot_ip = '192.168.0.100';
robot_port = 5001;

% Test connection
run('test_tcp.m')
```

### 2. Capture Maze Image

Place maze workpiece in camera view with:
- **Red circle** = Start point (away from camera)
- **Green circle** = End point (near camera)

### 3. Run Maze Solver

```matlab
% Run main script
run('gikmazefinal.m')
```

The script will:
1. Load and process the maze image
2. Detect start/goal points
3. Compute shortest path
4. Calculate joint angles via IK
5. Send commands to robot

### 4. Simulate with Digital Twin

```matlab
% Open Simulink model
open('myCobot28DT.slx')

% Run simulation to verify path before physical execution
```

## 🔬 Technical Details

### Image Processing

- **Color Space**: HSV for robust color detection
- **Thresholding**: Adaptive thresholding for maze wall detection
- **Morphology**: Erosion/dilation to clean binary mask

### Path Planning

- **Algorithm**: Dijkstra's shortest path
- **Graph**: Maze converted to weighted graph
- **Optimization**: Only turning points are sent to robot (reduces commands)

### Inverse Kinematics

- **Solver**: Generalized Inverse Kinematics (GIK)
- **Constraints**: Position + orientation (end-effector pointing down)
- **DOF**: 6-axis robot arm

### Robot Communication

- **Protocol**: TCP/IP over Ethernet
- **Commands**: `set_angles(j1, j2, j3, j4, j5, j6, speed)`
- **Synchronization**: `wait_command_done()` between movements

## 📊 Results

The system successfully:
- ✅ Detects maze structure from camera image
- ✅ Identifies start (red) and goal (green) points
- ✅ Computes shortest path using Dijkstra's algorithm
- ✅ Transforms pixel coordinates to robot workspace
- ✅ Calculates valid joint angles via inverse kinematics
- ✅ Navigates maze without wall collisions
- ✅ Completes maze in under 1 minute

## 🔮 Future Improvements

- **Machine Learning**: Adaptive navigation using reinforcement learning
- **Speed Optimization**: Smoother trajectory planning with velocity profiles
- **Dynamic Obstacles**: Real-time obstacle detection and replanning
- **Multiple Robots**: Coordinated multi-robot maze solving

---

<div align="center">

*Developed as part of RAS545 Final Project at Arizona State University*

</div>
