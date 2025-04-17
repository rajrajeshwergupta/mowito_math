# Maths Program

This repository contains two C++ programs for robotic computations:

1. **Euler to Quaternion Converter**
2. **Forward Kinematics (4-DOF Arm using Modified DH Parameters)**
---

## 🧠 Features

### ✅ Euler to Quaternion Converter
- Converts Euler angles (Roll, Pitch, Yaw in degrees) to Quaternion.
- Converts back from Quaternion to Euler angles.
- Uses standard math and angle normalization.

### ✅ Forward Kinematics
- Simulates a 4-joint robotic arm using Modified Denavit-Hartenberg convention.
- Computes the end-effector's 3D position based on 4 joint angles.
- Link length is assumed to be 1.0 meter for all links.

---

## 📁 File Structure

.
├── euler_quaternion.cpp   # Euler <-> Quaternion conversion
├── forward_kinematics.cpp # Forward kinematics with DH parameters
├── README.md              # This file

---

## 🛠️ Compilation

### Euler to Quaternion


    g++ -o euler_quaternion euler_quaternion.cpp -lm


### Forward Kinematics

Using g++ on Linux/macOS/WSL:

    g++ -o forward_kinematics forward_kinematics.cpp -lm


---

## 🚀 Usage

### Run Euler to Quaternion:

    ./euler_quaternion

Example input:

    Enter Euler angles (degrees):
    Roll (X): 45
    Pitch (Y): 30
    Yaw (Z): 60

Output:

    Quaternion: [w=0.822364, x=0.200562, y=0.391903, z=0.360424]
    Euler Angles (deg): [Roll=45.00, Pitch=30.00, Yaw=60.00]

---

### Run Forward Kinematics:

    ./forward_kinematics

Example input:

    Enter joint angles in degrees:
    Joint 1: 45
    Joint 2: 30
    Joint 3: 15
    Joint 4: -10

Output:

    End-Effector Position:
    X = 2.19045 m
    Y = 2.19045 m
    Z = -0.0522642 m
