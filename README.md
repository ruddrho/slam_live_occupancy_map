<p align="center">

<img src="results/navigation_dashboard.png" width="850">

</p>


<h1 align="center">
Hybrid LiDAR Navigation and Live Occupancy Mapping
</h1>


<p align="center">

A MATLAB-based Autonomous Mobile Robot Navigation Framework

</p>


<p align="center">

![MATLAB](https://img.shields.io/badge/MATLAB-R2020a+-orange)
![Robotics](https://img.shields.io/badge/Field-Autonomous%20Robotics-blue)
![SLAM](https://img.shields.io/badge/SLAM-Occupancy%20Mapping-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

</p>



# 🤖 Project Overview


This project implements an autonomous mobile robot navigation framework combining:

- Global path planning
- Local motion control
- LiDAR perception
- Odometry estimation
- Scan matching
- SLAM occupancy mapping
- Navigation performance evaluation


The objective is to develop a complete autonomous navigation pipeline similar to modern mobile robot systems.


---

# 🚗 System Architecture


```
                 Goal Position

                      |

                      ↓

              A* Global Planner

                      |

                      ↓

             Path Processing

                      |

                      ↓

        Dynamic Window Approach (DWA)

                      |

                      ↓

          Differential Drive Robot

          ------------------------

          |                      |

        LiDAR                Odometry

          |                      |

          ↓                      ↓

       Scan Matching -------- SLAM

                      |

                      ↓

            Occupancy Grid Map

```


---

# ✨ Key Features


## 🗺️ Path Planning

- A* global path planning
- Efficient obstacle-aware route generation
- Goal-directed navigation


## 🎯 Local Control

Dynamic Window Approach (DWA):

- Velocity sampling
- Trajectory prediction
- Collision checking
- Clearance evaluation
- Dynamic obstacle avoidance


## 👁️ LiDAR Perception

Simulated 360° LiDAR system:

- Range measurement
- Obstacle detection
- Environment scanning
- Sensor-based navigation


## 🧭 SLAM Mapping

Live occupancy mapping using:

- Odometry prediction
- Scan matching
- Log-odds inverse sensor model
- Occupancy probability update


## 📊 Evaluation

Navigation performance measurement:

- Path length
- Goal error
- Tracking RMSE
- Minimum obstacle clearance
- Mission completion time
- Replanning events


---

# 🎬 Simulation Demo


Add your GitHub video link:


```
https://github.com/user-attachments/assets/YOUR_VIDEO_LINK
```


The simulation demonstrates:


- Autonomous navigation
- LiDAR scanning
- Dynamic obstacle avoidance
- SLAM map generation
- Goal reaching behavior


---

# 📸 Results


## Navigation Dashboard


<img src="results/navigation_dashboard.png" width="900">


---

## Live Occupancy Map


<img src="results/slam_map.png" width="900">


---

## Performance Analysis


<img src="results/performance_graph.png" width="900">


---

# 🧠 Algorithms Implemented


## 1. A* Global Planner


Used for global route generation.

Advantages:

- Complete grid search
- Optimal path generation
- Obstacle avoidance


---

## 2. Dynamic Window Approach (DWA)


DWA selects safe velocity commands by evaluating possible robot trajectories.


Evaluation criteria:


```
Trajectory Score =

Goal Progress

+

Path Alignment

+

Obstacle Clearance

-

Motion Cost

```


---

## 3. Differential Drive Control


Robot motion model:


```
Linear Velocity  → Forward Motion

Angular Velocity → Rotation Control

```


The controller respects:

- Velocity limits
- Acceleration constraints
- Turning radius


---

## 4. LiDAR Based SLAM


Mapping pipeline:


```
LiDAR Scan

     ↓

Scan Matching

     ↓

Pose Update

     ↓

Log-Odds Map Update

     ↓

Occupancy Grid

```


---

# 📂 Project Structure


```
slam_live_occupancy_map


│
├── main.m
│
├── planning
│   └── planPathAStar.m
│
├── control
│   ├── dynamicWindowApproach.m
│   └── evaluateTrajectory.m
│
├── mapping
│   ├── initializeSlamMap.m
│   └── updateSlamMap.m
│
├── sensing
│   └── simulateLidar.m
│
├── robot
│   └── robotKinematics.m
│
├── results
│   ├── navigation_dashboard.png
│   ├── slam_map.png
│   └── performance_graph.png
│
├── PROJECT_REPORT.md
└── LICENSE

```


---

# ⚙️ Requirements


Software:

- MATLAB R2020a or newer


Recommended:

- MATLAB Robotics Toolbox


---

# 🚀 Running the Simulation


Open MATLAB:


```matlab
main
```


The simulation generates:

- Robot trajectory
- SLAM occupancy map
- Performance plots
- Navigation results


---

# 🔬 Research Contribution


This project demonstrates practical implementation of:


- Autonomous mobile robotics
- SLAM
- Motion planning
- Feedback control
- LiDAR perception
- Navigation optimization


---

# 🚀 Future Development Roadmap


## ROS2 Jazzy Migration


The next development phase will convert this MATLAB framework into a complete ROS2 robotic system.


Target architecture:


```
ROS2 Jazzy

      |

      ↓

Gazebo Harmonic Simulation

      |

      ↓

Robot URDF + ros2_control

      |

      ↓

LiDAR + IMU + Encoder

      |

      ↓

SLAM Toolbox

      |

      ↓

Nav2 Navigation Stack

      |

      ↓

Real Robot Deployment

```


Future upgrades:


- ROS2 C++ nodes
- Gazebo Harmonic simulation
- Nav2 integration
- SLAM Toolbox
- EKF sensor fusion
- Real LiDAR testing
- Hardware deployment


---

# 📚 Academic Relevance


Suitable for:


- Robotics Master's Portfolio
- Control Systems Research
- Autonomous Systems Development


Relevant fields:


- Mobile Robot Navigation
- SLAM
- Robot Control
- Motion Planning
- Sensor Fusion


---

# 📖 Citation


```
Ruddrho Mollik,

Hybrid LiDAR Navigation and Live Occupancy Mapping
for Autonomous Mobile Robots.

MATLAB Robotics Simulation Project.
```


---

# 📜 License


This project is licensed under the MIT License.


---

# 👨‍💻 Author


**Ruddrho Mollik**


Research Interests:

- Autonomous Robots
- SLAM
- Motion Planning
- Control Systems
- Robotics Software
