# Parol6 Main

~~Very experimental, currently randomly trying out various trajectories and motion profiles for the esp32 for the Parol6 project.~~

Less Experemental now. 
This repo contains the entire codebase for my custom software stack for the Parol6 robot by Source Robotics.

## Project Structure

- **`mcu-code/`** - ESP32 firmware and embedded source code:
  - **`gripper-slave/`** - ESP32 firmware for the gripper end-effector, communicating with the main ESP32 over ESP-NOW / Wi-Fi.
  - **`quintic-motion/`** - Main base ESP32 firmware responsible for DDS step generation, quintic motion planning, homing routines, and motor control.
- **`pinocchio/`** - High-level C++ kinematics and dynamics engine powered by the Pinocchio library. Also contains the web control server (`web/`), telemetry scripts, and MCAP logging tools.
- **`STLs/`** - 3D CAD models, STL meshes, and enclosure designs for the robot and gripper add-ons.
- **`calculations/`** - MATLAB (`.mlx`) scripts and analytical calculations for kinematics verification and simulation.
- **`misc_scripts/`** - Collection of helper scripts, VCD logic analyzer diagnostic tools, serial sliders, URDF models, and pulse analysis utilities.
 

