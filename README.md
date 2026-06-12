# Ceiling-Mounted Fire-Suppressing Water Turret

This repository contains the Python file used for our capstone project at National University Fairview.

The project is a ceiling-mounted fire-suppressing water turret. It uses camera-based fire detection, thermal verification, motor movement, relay activation, and SMS alerting to help detect and suppress fire.

This repository is intended for future NU Fairview students who want to continue, modify, or improve the project.

## Important Note for Future Students

The current working Python file on the Jetson Nano is located at:

```bash
Downloads/Cirilo/working/aq_mode.py
```

Because of this, the system should be run from that folder on the Jetson Nano.

If the file is moved to another folder, some paths, imports, model files, or hardware-related settings may need to be changed.

## How to Run the System on the Jetson Nano

1. Turn on the Jetson Nano.

2. Make sure the required hardware is connected:

   * USB camera
   * Arduino Uno
   * AMG8833 thermal sensor
   * SIM900A GSM module
   * Stepper motor drivers
   * Relay module
   * Water pump

3. Open the terminal.

4. Go to the working folder:

```bash
cd ~/Downloads/Cirilo/working
```

5. Run the Python file:

```bash
python3 aq_mode.py
```

If this does not work, try:

```bash
python aq_mode.py
```

## Main System Features

The Python file controls the main system interface and logic.

The system includes:

* Automatic fire detection
* Thermal verification using the AMG8833 thermal sensor
* Manual and automatic control modes
* Stepper motor movement for aiming
* Relay control for water activation
* SMS alert function through GSM
* Local graphical user interface
* Manual relay ON and OFF buttons
* Confirmation window for manual relay activation
* Philippine phone number validation for saved SMS numbers

## System Modes

### Automatic Mode

In automatic mode, the system monitors the camera feed and checks for possible fire. If fire is detected and confirmed by thermal readings, the system aims the turret and activates the relay for water suppression.

### Manual Mode

In manual mode, the user can control the turret movement manually. The user can also manually turn the relay ON or OFF.

The manual relay ON button has a confirmation window to prevent accidental activation.

## Hardware Used

The system uses the following main components:

* Jetson Nano
* USB webcam
* AMG8833 thermal camera
* Arduino Uno
* SIM900A GSM module
* TB6600 stepper motor drivers
* NEMA 23 stepper motors
* Relay module
* Water pump or water control mechanism
* Power supplies for Jetson Nano, motors, and other modules

## Software Requirements

The system uses Python 3.

The main Python libraries used may include:

* OpenCV
* NumPy
* Tkinter
* Pillow
* PySerial
* Threading
* Time
* OS

Install missing libraries using:

```bash
pip3 install opencv-python numpy pillow pyserial
```

## Important Note About the Fire Detection Model

The fire detection model used in this project is connected through an online Roboflow model/API. This means the model may stop working in the future if the API key expires, the account access changes, the Roboflow project is deleted, or the model version becomes unavailable.

Future students are advised to create their own Roboflow project and train a new fire detection model if the current model no longer works.

After creating a new model in Roboflow, update the API key inside the Python code.

Check the part of the code where Roboflow is initialized.

Replace the values with the new Roboflow project details.

## Files in This Repository

Important files:

```text
aq_mode.py
```

Main Python file for the project.

```text
README.md
```

Documentation for future students.

## Suggested Folder Location

For the current Jetson Nano setup, keep the file in:

```bash
~/Downloads/Cirilo/working/
```

The system was tested using this location.

If future students want to reorganize the files, they should check the Python code for file paths, model paths, image paths, and serial port settings.

## Serial Port Notes

The project communicates with external hardware. If the system does not respond, check the serial port used by the Arduino or other connected modules.

Common things to check:

* Arduino is connected through USB
* Correct serial port is selected
* Correct baud rate is used
* User has permission to access serial devices
* Arduino code is uploaded properly
* Relay and GSM module are powered properly

## Before Running the System

Check the following before starting:

* The turret is mechanically safe to move
* The wires are not twisted
* The water system is not leaking
* The relay is OFF before testing
* The camera is connected
* The Arduino is connected
* The GSM module has signal and load if SMS testing is needed
* A working API_KEY from Roboflow is changed in the code

## Troubleshooting

### The camera does not open

Check if the USB camera is connected properly.

Try restarting the Jetson Nano.

### The motors do not move

Check the motor power supply, TB6600 driver wiring.

### The relay does not activate

Check the relay wiring, Arduino code, and serial communication.

### SMS is not sent

Check the SIM900A module, SIM card, signal, power supply, and phone number format.

### The program does not start

Make sure you are inside the correct folder:

```bash
cd ~/Downloads/Cirilo/working
```

Then run:

```bash
python3 aq_mode.py
```

## Notes for Future Development

Future students may improve the project by:

* Improving fire detection model
* Adding better object tracking
* Adding safer motor rotation limits
* Improving wire protection
* Improving the user interface
* Adding better logging
* Adding a setup script
* Separating the code into multiple Python files
* Adding better documentation for wiring and calibration
* Adding automatic startup when the Jetson Nano turns on

## Warning

This project involves electricity, moving motors, and water. Always test carefully. Do not activate the water system near exposed electronics.

Use manual testing first before using automatic mode.

## Contributors

Add the names of the group members here.

* John Hervey Abella
* Cirilo Ignacio Geronio
* Mike Jasper Lingasin
* Yuan Gabriel Vinegas

## School

National University Fairview
Computer Engineering Department
Capstone Project
