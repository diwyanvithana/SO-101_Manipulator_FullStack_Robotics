# SO-101 Robotic Manipulator
## Full-Stack Control, Simulation, Teleoperation, and Learning Pipeline

This repository provides a complete robotics software stack for the **SO-101 robotic manipulator**, combining:

- Low-level servo control for **Feetech STS3215** actuators
- Real-time teleoperation using an **Xbox controller**
- **MuJoCo** physics-based simulation and digital-twin modeling
- Hardware–simulation bridging for synchronized testing
- Joint calibration, feedback, logging, and safety tools
- A modular structure designed for future machine-learning control

The system enables teleoperation, trajectory recording, simulation-based testing, and groundwork for sim-to-real learning.

---

## 1. System Architecture

```text
Xbox Controller
      ↓
Teleoperation Layer (Python)
      ↓
Control Pipeline (Joint / IK)
      ↓
MuJoCo Digital-Twin Simulation
      ↓
Hardware Bridge (UART Serial)
      ↓
Feetech STS3215 Servos (Real Arm)
      ↓
Encoder Feedback → State Estimation
```

This unified architecture allows:

- Hardware–simulation consistency
- Safe, reproducible motion testing
- Real-time control on physical hardware
- Simulation rollout for ML training
- Sim-to-real policy transfer

---

## 2. Servo Control Layer (STS3215)

### Features

- Custom Python SC-protocol implementation
- UART communication via FE-URT-1
- Position, speed, and time-based motion control
- Encoder-based feedback
- Auto-calibration for raw-to-angle mapping
- Safety clamping to prevent over-rotation
- Diagnostic utilities for temperature, voltage, torque

### Key Files

```
servo_control/
  feetech_sts.py                  # Core driver
  Motor_control_cli.py            # CLI for angle commands + feedback
  calibrate_sts_joint.py          # Auto-calibration tool
  Motor_test1.py
  Motor_test2.py
  python_sts_simple_test.py
```

---

## 3. Teleoperation Layer (Xbox Controller)

### Features

- Real-time joint-based control
- IK-based target control (for simulation)
- Adjustable speed modes
- Fine-step precision control
- Button mapping for various joints
- Supports both simulation and real hardware

### Key Files

```
teleoperation/
  phyXbox.py
  xbox_servo_teleop_basic.py
  xbox_mujoco_hardware_teleop.py
  xbox_mujoco_hardware_teleop_full.py
```

---

## 4. MuJoCo Simulation & Digital Twin

This project contains a MuJoCo digital twin of the SO-101 manipulator, enabling:

- Visualization and animation
- Physically realistic dynamics
- Inverse kinematics testbeds
- Trajectory and motion-profile evaluation
- Control algorithm prototyping
- Offline dataset generation for ML

### Key Files

```
mujoco_simulation/
  so101_arm.xml                   # Main MuJoCo robot model
  test_mujoco.py
  so101_ik_demo.py
  so101_ik_keyboard_control.py
  mujoco_hardware_bridge.py
  step2_move_one_joint.py
  step3_move_all_joints.py
  step7_scurve_motion.py
  step8_scurve_log.py
  step9_compare_linear_vs_scurve.py
  step10_log_torque_scurve.py
```

---

## 5. Feedback & Diagnostics Tools

### Features

- Live servo monitoring
- Temperature, voltage, torque, encoder polling
- Stress-testing mode
- Multi-servo visual feedback

### Key Files

```
feedback_tools/
  servo_feedback_monitor.py
  sts_feedback_test.py
  watch_all_servos.py
```

---

## 6. Machine Learning Roadmap (Planned)

This repository is structured to support future ML modules:

### Imitation Learning

- Teleoperate the robot
- Record trajectories
- Train neural policies to imitate human motion

### Reinforcement Learning

- Train reaching/manipulation tasks in MuJoCo
- Deploy learned policies to hardware

### Sim-to-Real Transfer

- Domain randomization
- Disturbance injection
- Residual policy learning (PID + Neural Network)

---

## 7. Repository Structure

```
SO-101_Manipulator_FullStack_Robotics
│
├── servo_control/                 # Low-level STS3215 driver & tests
├── teleoperation/                 # Xbox teleoperation code
├── mujoco_simulation/             # Digital twin, IK, simulation tools
├── feedback_tools/                # Monitoring & diagnostics
│
├── requirements.txt               # Python dependencies
├── README.md
├── .gitignore
└── LICENSE (MIT)
```

---

## 8. Installation

### Install Python requirements

```bash
pip install -r requirements.txt
```

### Install MuJoCo (if not installed)

Download MuJoCo from:  
https://mujoco.org/

---

## 9. Usage

### Test servo communication

```bash
python servo_control/Motor_control_cli.py
```

### Run the MuJoCo simulation

```bash
python mujoco_simulation/test_mujoco.py
```

### Run Xbox teleoperation for hardware + simulation

```bash
python teleoperation/xbox_mujoco_hardware_teleop_full.py
```

---

## 10. Requirements

### Hardware

- SO-101 6-DOF robotic manipulator
- Feetech STS3215 serial servos
- FE-URT-1 USB UART adapter
- Xbox One/Series controller
- PC or Jetson (Python 3.8+ recommended)

### Software

- Python 3.8–3.12
- MuJoCo 3.x
- PySerial
- NumPy
- Pygame

---

## 11. License

This project is released under the **MIT License**.  
You are free to use it for research, education, and open-source development.

---

## 12. Contributions

Contributions, pull requests, and issue reports are welcome.

Suggested areas:

- Improved IK solvers
- Additional teleoperation modes
- Real-time plotting tools
- ML-based policy modules
- Enhanced MuJoCo modeling

---

## 13. Author

**Diwyan Vithana**  
Mechatronics Engineering — Robotics & AI  
Sri Lanka Institute of Information Technology (SLIIT)

---

## 14. Technical Tags

```
Robotics
Mechatronics
Control Systems
MuJoCo
Digital Twin
Servo Control
Feetech STS
UART
Teleoperation
Imitation Learning
Reinforcement Learning
Sim2Real
Python Robotics
```
