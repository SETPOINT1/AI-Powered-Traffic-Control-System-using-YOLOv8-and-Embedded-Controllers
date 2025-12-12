# AI-Powered-Traffic-Control-System-using-YOLOv8-and-Embedded-Controllers

![Python](https://img.shields.io/badge/Python-3.x-blue?logo=python&logoColor=white)
![Arduino](https://img.shields.io/badge/Hardware-Arduino_Mega-teal?logo=arduino&logoColor=white)
![YOLOv8](https://img.shields.io/badge/AI-YOLOv8-purple)
![OpenCV](https://img.shields.io/badge/Vision-OpenCV-green?logo=opencv&logoColor=white)

An intelligent, adaptive traffic signal control system that uses **Computer Vision** and **Deep Learning** to optimize traffic flow in real-time.

## Overview

Traditional traffic lights use fixed timers, leading to unnecessary waiting times when lanes are empty. This project solves that problem by using **YOLOv8** to detect vehicle density and dynamically adjust green light duration.

The system operates on a **Master-Slave Architecture**:
*   ** Master (Raspberry Pi):** Captures images from 4 cameras, detects vehicles (Cars, Trucks, Buses, Bikes), calculates a "Density Score", and determines the optimal green light duration.
*   ** Slave (Arduino Mega):** Controls the high-voltage traffic lights via Relays, ensures safety timings (Yellow light, All-Red intervals), and executes commands received from the Master.

---

## Repository Structure

This repository contains the essential source code for the system:

| File | Description |
| :--- | :--- |
| `traffic_master.py` | **Python Script for Raspberry Pi.** Handles image processing, YOLOv8 detection, density calculation, and Serial communication. |
| `Traffic_Control_Slave.ino` | **Arduino Firmware.** Handles Relay control, state machine logic, safety timers, and command parsing. |

---

## Hardware Requirements

1.  **Processing Unit:** Raspberry Pi 4 or 5 (8GB RAM recommended).
2.  **Controller Unit:** Arduino Mega 2560.
3.  **Vision Sensors:** 4x USB Webcams (North, East, South, West).
4.  **Actuators:** 4-Channel or 8-Channel Relay Module (Active Low).
5.  **Traffic Lights:** LED Traffic Light modules (or LEDs for prototyping).
6.  **Communication:** USB Cable (Type-A to Type-B).

---

## Installation & Setup

### 1. Raspberry Pi (Master) Setup
Ensure your Raspberry Pi is running Raspberry Pi OS (64-bit).

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install dependencies
pip install opencv-python ultralytics pyserial

# Clone this repository
git clone https://github.com/YOUR_USERNAME/AI-Powered-Traffic-Control-System-using-YOLOv8-and-Embedded-Controllers.git
cd AI-Powered-Traffic-Control-System-using-YOLOv8-and-Embedded-Controllers
