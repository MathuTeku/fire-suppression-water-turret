# fire-suppression-water-turret
Python control and monitoring software for a ceiling-mounted fire-suppressing water turret using fire detection, thermal verification, motor control, relay activation, and SMS alerting.

# Ceiling-Mounted Fire-Suppressing Water Turret

This repository contains the Python software for our capstone project. The system is designed to detect possible fire using a camera, verify heat using a thermal sensor, aim the water turret, activate the relay for water suppression, and send SMS alerts.

## Project Overview

The system has two main modes:

- Automatic Mode: Detects fire, verifies heat, aims the turret, and activates suppression.
- Manual Mode: Allows the user to control turret movement and relay activation manually.

## Main Features

- Fire and smoke detection using camera input
- Thermal verification using AMG8833 thermal sensor
- Pan and tilt motor control
- Manual relay ON/OFF control
- Automatic relay activation during confirmed fire
- SMS alerting through GSM module
- Local graphical user interface for monitoring and control

## Hardware Used

- Jetson Nano
- USB camera
- AMG8833 thermal camera
- Arduino Uno
- SIM900A GSM module
- TB6600 stepper motor drivers
- NEMA 23 stepper motors
- Relay module
- Water pump or solenoid-controlled water system

## Software Requirements

- Python 3
- OpenCV
- Tkinter
- NumPy
- PySerial
- Pillow

## How to Run

Install the required Python packages:

```bash
pip install -r requirements.txt
