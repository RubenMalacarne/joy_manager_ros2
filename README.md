# Dynamic_radio_interface

A ROS 2 package designed to make the interface of various remote controllers more practical and standardized for controlling the **Climb Robot Alpine**.

---

## Purpose

This package provides a unified ROS 2 interface for different radio controllers, converting their inputs into **standard ROS 2 topics**.  
This simplifies the integration of remote controls into the robot system, regardless of the specific hardware used.

---

## Supported Controllers

Currently supported devices:

- Radiomaster Pocket (configurable RC radio controller),  via Bluetooth or USB
- PS4 Controller (DualShock 4), via Bluetooth or USB

---

## Joystick: [radiomaster pocket](https://radiomasterrc.com/products/pocket-radio-controller-m2) and [dualShock 4](https://www.playstation.com/it-it/accessories/dualshock-4-wireless-controller/)  

This launch file starts two ROS 2 nodes:

1. `joy_node` – reads raw joystick data using the `joy` package.
2. `dualshock_ps4` – interprets data from the DualShock 4 controller and publishes commands and services to custom topics.

### Goal

Enable advanced control using a PS4 controller, with the ability to dynamically configure the topics and service names associated with buttons, triggers, and joysticks.

---

## How to Modify Topics and Services

The topics and services used by node can be configured directly in the Python LAUNCH file, inside the `parameters` section.

### Example
before:

```python
    "roll_topic", "joystick/roll"
    "pitch_topic", "joystick/pitch"
    "yaw_topic", "joystick/yaw"
    "thrust_topic", "joystick/thrust"
```

after:

```python
    'roll_topic': '/arganello/sx/target_velocity',
    'pitch_topic': '/arganello/sx/target_torque',
    'yaw_topic': '/arganello/sx/position_command',
    'thrust_topic': '/arganello/dx/target_torque',
```