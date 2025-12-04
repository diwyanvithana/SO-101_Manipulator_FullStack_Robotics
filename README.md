# 🚀 SO-101 Robotic Manipulator — Full-Stack Control, Simulation & Learning Pipeline

This repository contains a complete robotics control and simulation framework for the **SO-101 manipulator**, integrating:

- **Low-level serial servo actuation** (Feetech STS3215)  
- **Real-time Xbox controller teleoperation**  
- **MuJoCo-based physics simulation & digital twin**  
- **Joint calibration & servo feedback utilities**  
- **Unified hardware–simulation bridge**  
- **Foundation for machine-learning-based autonomous control**

The goal of this project is to build a fully operational **learning robotic system**, capable of teleoperation, trajectory recording, simulation-based training, and future deployment of ML-driven motion policies.

---

## 🧩 System Architecture

```text
Xbox Controller  
     ↓  
Teleop Layer (Python)  
     ↓  
Control Pipeline (Joint / IK)  
     ↓  
MuJoCo Simulation (Digital Twin)  
     ↓  
Hardware Bridge (UART)  
     ↓  
STS3215 Servos (Real Arm)  
     ↓  
Encoder Feedback  
     ↓  
Back to Control Layer
