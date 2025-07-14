# climb_radio_interface

A ROS 2 package designed to make the **interface of various remote controllers more practical and standardized** for controlling the **Climb Robot Alpine**.

---

## 🎯 Purpose

This package provides a unified ROS 2 interface for different radio controllers, converting their inputs into **standard ROS 2 topics**.  
This simplifies the integration of remote controls into the robot system, regardless of the specific hardware.

---

## 🎮 Supported Radio Controllers

Currently supported devices:

- **Radiomaster Pocket** (configurable RC radio controller)
- **PS4 Controller (DualShock 4)** via Bluetooth or USB

---

## 🚀 Main Features

- Read input from **Radiomaster Pocket** (USB/serial or configurable interface)
- Full support for **PS4 Controller**, including axes, triggers, and buttons
- YAML-based configuration for mapping axes and buttons to specific ROS topics
- Hardware-independent ROS 2 interface

---
