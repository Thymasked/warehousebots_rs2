# Multi-TurtleBot3 Coordination (Distributed Python Control - ROS Noetic)

This project implements a **distributed control system** for multiple TurtleBot3 robots using **ROS 1 Noetic**. Each robot is controlled via a separate remote PC running its own dedicated Python script to navigate to goal positions on a pre-mapped environment.

> Unlike centralized multi-robot ROS launches, this project separates control logic by device. Each robot runs independently on its own networked PC, improving modularity and simplifying resource constraints.

---

## Features

- TurtleBot3 navigation with `move_base`
- ROS 1 Noetic-based distributed architecture
- Map-based localization (AMCL)
- Individual Python scripts per robot PC
- Map folder with pre-built `.yaml` and `.pgm` files
