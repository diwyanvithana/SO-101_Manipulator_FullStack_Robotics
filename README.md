🚀 SO-101 Robotic Manipulator — Full-Stack Control, Simulation & Learning Pipeline

This repository contains a complete robotics control and simulation framework for the SO-101 manipulator, integrating:

Low-level serial servo actuation (Feetech STS3215)

Real-time Xbox controller teleoperation

MuJoCo-based physics simulation & digital twin

Joint calibration & servo feedback utilities

Unified hardware–simulation bridge

Foundation for machine-learning-based autonomous control

The goal of this project is to build a fully operational learning robotic system, capable of teleoperation, trajectory recording, simulation-based training, and future deployment of ML-driven motion policies.

🧩 System Architecture
Xbox Controller  →  Teleop Layer (Python) 
                  →  Control Pipeline (Joint/IK)
                  →  MuJoCo Simulation (Digital Twin)
                  →  Hardware Bridge (UART)
                  →  STS3215 Servos (Real Arm)
                  →  Encoder Feedback → Back to Control Layer


This unified architecture allows seamless switching between simulation and hardware, enabling safe testing, data collection, and eventual sim-to-real ML transfer.

🔧 1. Servo Control Layer (Feetech STS3215)

Located in servo_control/

Features:

Custom UART driver (Python) for SC-protocol

Servo ping, write, read, torque enable

Raw encoder sampling

Automatic joint range calibration

Raw ↔ angle conversion with safety clamping

PWM, goal position, speed, and time control

Real-time servo feedback monitoring

Example Scripts:

feetech_sts.py — main driver

Motor_control_cli.py — command-line joint teleop

calibrate_sts_joint.py — auto-calibration tool

Motor_test*.py — hardware validation

🎮 2. Teleoperation Layer (Xbox Controller → Robot)

Located in teleoperation/

Features:

Analog stick → joint velocity or IK motion

Trigger → end-effector / gripper functions

Mode switching (Joint / Cartesian / Safety mode)

Debounced fine-step controls

Fully configurable button mapping

Real-time feedback printing

Scripts:

xbox_servo_teleop_basic.py

xbox_mujoco_hardware_teleop.py

xbox_mujoco_hardware_teleop_full.py

phyXbox.py — gamepad interface

This system allows smooth, low-latency teleoperation comparable to professional research platforms.

🧠 3. MuJoCo Simulation & Digital Twin

Located in mujoco_simulation/

Includes:

Full MuJoCo XML model of SO-101 (so101_arm.xml)

Joint limits matched to physical servo calibration

IK demonstration scripts

Hardware–simulation synchronization tools

Motion profiling tests (linear vs S-curve)

Torque & trajectory logging utilities

Example Scripts:

test_mujoco.py — base simulation

so101_ik_demo.py — inverse kinematics demo

mujoco_hardware_bridge.py — sim ↔ hardware syncing

step*_*.py — trajectory, logging, experiments

MuJoCo enables safe algorithm testing, trajectory generation, and ML dataset creation.

🎯 4. Machine Learning Roadmap (Upcoming)

This repository is structured for future integration of:

✔ Imitation Learning

Using teleoperation data to train movement policies.

✔ Reinforcement Learning

Training autonomous reaching, grasping, and manipulation behaviour in simulation.

✔ Domain Randomization

Improving sim-to-real performance with variability in physics, noise, and delays.

✔ Hybrid Controllers

Combining classical joint control with learned residual policies.

📁 Repository Structure
servo_control/            # STS3215 driver, calibration, CLI tests  
teleoperation/            # Xbox control interfaces  
mujoco_simulation/        # MuJoCo XML model + sim scripts  
feedback_tools/           # Real-time servo feedback and monitoring  
docs/                     # Diagrams, architecture notes (optional)  
images/                   # Screenshots, demo images (optional)  
requirements.txt  
README.md  
.gitignore

▶️ Getting Started
1. Install dependencies
pip install -r requirements.txt

2. Test servo communication
python servo_control/Motor_control_cli.py

3. Run MuJoCo simulation
python mujoco_simulation/test_mujoco.py

4. Run hardware teleoperation
python teleoperation/xbox_mujoco_hardware_teleop_full.py

🔧 Hardware Requirements

Feetech STS3215 serial servos

FE-URT-1 USB–UART servo controller

Xbox One/Series controller

Arduino / ESP32 (optional future expansions)

Jetson / PC with Python 3.10+

📌 Software Requirements

Python 3.10+

MuJoCo 3.x

pyserial

pygame

numpy

📜 License

MIT License — open and free for research and educational use.

🤝 Contributions

Pull requests are welcome for:

IK improvements

Additional teleoperation modes

ML integration modules

Better MuJoCo modeling

🌐 Author

Diwyan Vithana
Mechatronics Engineering — Robotics & AI
Sri Lanka Institute of Information Technology (SLIIT)

🔖 Technical Tags

Robotics Mechatronics Control Systems MuJoCo
UART STS3215 Digital Twin Teleoperation
Imitation Learning Reinforcement Learning Sim2Real
