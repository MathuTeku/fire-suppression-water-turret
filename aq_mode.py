#!/usr/bin/env python3
"""
Combined Jetson Nano 2GB control program - calibrated auto scan / aim version

What this combines:
1. Roboflow object detection camera feed from jetson_infer10.py
2. AMG8833 thermal camera monitoring from half_module.txt
3. Manual 2-axis motor control, relay, SMS, and call controls from half_module.txt

Run:
    python3 combined_blastoys_heat_first_relay_auto_call_xgear_py36.py

Quit:
    Press ESC or click Exit.
"""

import base64
import os
import json
import queue
import builtins
import time
import threading
import re
from collections import deque
from datetime import datetime
from typing import Any, Dict, Iterable, List, Tuple

import cv2
import requests
import serial
import tkinter as tk

# =====================================================
# GUI SERIAL MONITOR / LOG CAPTURE
# =====================================================
# This file runs on the Jetson from a terminal, but the UI also has a built-in
# serial monitor. This print wrapper keeps normal terminal output while copying
# messages into a queue that the Tkinter Text box can display.
_ORIGINAL_PRINT = builtins.print
serial_log_queue = queue.Queue()

def print(*args, **kwargs):
    try:
        sep = kwargs.get("sep", " ")
        end = kwargs.get("end", "\n")
        text = sep.join(str(item) for item in args) + end
        line = text.rstrip("\n")
        if line:

            allow_gui_log = True

            try:
                if get_operation_mode() == "AUTO" and not SERIAL_MONITOR_ENABLED_IN_AUTO:
                    allow_gui_log = False
            except Exception:
                pass

            if allow_gui_log:
                timestamp = time.strftime("%H:%M:%S", time.localtime())
                try:
                    serial_log_queue.put_nowait("[{}] {}".format(timestamp, line))
                except Exception:
                    pass
    except Exception:
        pass
    _ORIGINAL_PRINT(*args, **kwargs)

import busio
import board
import adafruit_amg88xx

import Jetson.GPIO as GPIO


# =====================================================
# USER SETTINGS
# =====================================================

# Arduino / SIM900A / Relay
ARDUINO_PORT = "/dev/ttyACM0"
BAUD_RATE = 9600
PHONE_NUMBER = "+639182719703"

# Firebase
FIREBASE_BASE_URL = "https://blastoys-9a063-default-rtdb.asia-southeast1.firebasedatabase.app"
FIREBASE_READINGS_PATH = "readings"
FIREBASE_AUTH_TOKEN = ""

# Roboflow hosted inference settings
ROBOFLOW_API_KEY = "kQYHBcA9MIPGtUC7OuAl"
ROBOFLOW_MODEL_PROJECT = "blastoys-0zggr-nnsrv"
ROBOFLOW_MODEL_VERSION = "1"
ROBOFLOW_HOSTED_URL = "https://detect.roboflow.com"

# Camera settings
USE_CSI_CAMERA = False          # False = USB webcam, True = Jetson CSI camera
CAMERA_INDEX = 0
CAPTURE_WIDTH = 640
CAPTURE_HEIGHT = 480
DISPLAY_WIDTH = 560
DISPLAY_HEIGHT = 420
INFERENCE_WIDTH = 640
INFERENCE_HEIGHT = 480
CAMERA_FPS = 15
JPEG_QUALITY = 80
CONFIDENCE_THRESHOLD = 0.50
IOU_THRESHOLD = 0.40
INFERENCE_INTERVAL_SECONDS = 0.75   # Larger value = less lag/network usage

# Thermal settings
HIGH_TEMP = 50.0
CRITICAL_TEMP = 100.0
MINTEMP = 20.0
MAXTEMP = 40.0
THERMAL_DISPLAY_WIDTH = 360
THERMAL_DISPLAY_HEIGHT = 270
THERMAL_UPDATE_INTERVAL_SECONDS = 0.25  # Thermal runs slower so object inference stays responsive

# Automatic SMS cooldown
SMS_COOLDOWN_SECONDS = 10
# Backup/escalation SMS sends if confirmed fire is still detected after this many seconds.
DEFAULT_FAIL_TIME_LIMIT_SECONDS = 15.0
ESCALATION_SMS_COOLDOWN_SECONDS = 10
SIM900_COMMAND_DELAY_SECONDS = 0.20
SIM900_SMS_TIMEOUT_SECONDS = 45
SIM900_CALL_TIMEOUT_SECONDS = 18

# Automatic mode confirmation delay.
# New heat-first logic:
# 1. AUTO mode scans X-axis while AMG8833 is normal.
# 2. When AMG8833 sees heat >= HIGH_TEMP, scanning stops in place.
# 3. The webcam gets this many seconds to confirm visible fire.
# 4. If no webcam fire is seen, that heat source is ignored briefly and scan continues.
FIRE_CONFIRMATION_SECONDS = 3.0
HEAT_SOURCE_IGNORE_SECONDS = 3.0

# Confirmed fire logic
# The relay and automatic SMS will trigger ONLY when BOTH are true:
# 1. Object detection sees class "fire"
# 2. AMG8833 thermal camera sees HIGH/CRITICAL temperature
# These hold windows allow the two background loops to confirm each other even
# if they update at slightly different times.
OBJECT_FIRE_HOLD_SECONDS = 4.0
THERMAL_FIRE_HOLD_SECONDS = 4.0
RELAY_OFF_DELAY_SECONDS = 5.0
FIXED_SPRAY_SECONDS = 5.0


# =====================================================
# MOTOR CALIBRATION
# =====================================================

# NEMA 23 base motor: 1.8 degrees per full step = 200 full steps/revolution.
NEMA_FULL_STEPS_PER_REV = 200
NEMA_FULL_STEP_DEGREES = 1.8

# X axis / horizontal pan.
# TB6600 is set to 6400 pulse steps per motor revolution.
# New X gearbox ratio is 73:21, approximately 3.476:1.
# Output 360 degrees therefore needs 6400 * 73 / 21 = 22247.6 pulse steps.
X_MICROSTEPS_PER_REV = 6400
X_GEAR_NUMERATOR = 73.0
X_GEAR_DENOMINATOR = 21.0
X_GEAR_RATIO = X_GEAR_NUMERATOR / X_GEAR_DENOMINATOR
X_OUTPUT_STEPS_PER_360 = int(round(X_MICROSTEPS_PER_REV * X_GEAR_RATIO))  # about 22248 steps/output 360
X_STEPS_PER_DEGREE = X_OUTPUT_STEPS_PER_360 / 360.0   # about 61.8 steps/degree

# =====================================================
# X AXIS SAFETY LIMIT + POSITION MEMORY
# =====================================================

# Safe X movement range.
# 0 degrees = center/front reference position.
# Negative = left side, Positive = right side.
#
# Adjust these based on your real wire slack.
# Example: -160 to +160 gives 320 degrees total movement, not full endless rotation.
X_LEFT_LIMIT_DEGREES = -175.0
X_RIGHT_LIMIT_DEGREES = 175.0

X_MIN_STEPS = int(X_STEPS_PER_DEGREE * X_LEFT_LIMIT_DEGREES)
X_MAX_STEPS = int(X_STEPS_PER_DEGREE * X_RIGHT_LIMIT_DEGREES)
X_START_REFERENCE_STEPS = 0

# Saves last known X/Y position so restart does not treat the current angle as zero.
POSITION_STATE_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "blastoys_position_state.json"
)

POSITION_SAVE_MIN_INTERVAL_SECONDS = 0.25
last_position_save_time = 0.0


# X/Y-axis wiggle while spraying
WIGGLE_DURING_SPRAY_ENABLED = True

# How far left/right X wiggles.
# Start small for testing. 1 degree is about 62 steps based on your X calibration.
WIGGLE_X_DEGREES = 5.0
WIGGLE_X_STEPS = int(X_STEPS_PER_DEGREE * WIGGLE_X_DEGREES)

# How far up/down Y wiggles.
# WIGGLE_Y_STEPS is calculated later after Y_STEPS_PER_DEGREE exists.
WIGGLE_Y_DEGREES = 2.0

# Delay after each wiggle movement.
# Smaller = faster wiggle, larger = slower wiggle.
WIGGLE_SETTLE_SECONDS = 0.15

wiggle_is_running = False






# Y axis / vertical tilt.
# TB6600 is set to 3200 pulse steps per motor revolution.
# Gearbox is 10:1, so the motor needs 10 revolutions for 360 degrees output.
# Mechanical Y movement is limited to 90 degrees.



Y_MICROSTEPS_PER_REV = 3200
Y_GEAR_RATIO = 10.0
Y_OUTPUT_STEPS_PER_360 = int(Y_MICROSTEPS_PER_REV * Y_GEAR_RATIO)  # 32000 steps/output 360
Y_STEPS_PER_DEGREE = Y_OUTPUT_STEPS_PER_360 / 360.0                # 88.888... steps/degree
Y_MAX_DEGREES = 90.0
Y_MIN_STEPS = 0
Y_MAX_STEPS = int(Y_STEPS_PER_DEGREE * Y_MAX_DEGREES)              # 8000 steps/output 90

# Y-axis spray wiggle steps.
# This is calculated here because Y_STEPS_PER_DEGREE now exists.
WIGGLE_Y_STEPS = int(Y_STEPS_PER_DEGREE * WIGGLE_Y_DEGREES)





# IMPORTANT: without limit switches/homing, the code cannot know the true physical Y angle at startup.
# For safe testing, place the Y axis around the middle of its physical 90-degree travel before running.
# This gives the software room to move both up and down.
Y_START_REFERENCE_STEPS = Y_MAX_STEPS // 2

# Direction tuning.
# Controls are intentionally inverted per system testing: left/right and up/down are swapped.
# The manual buttons already use the inverted physical directions below.
# These AUTO_* flags invert the automatic scan and aiming directions too.
AUTO_INVERT_X_DIRECTION = True
AUTO_INVERT_Y_DIRECTION = True
# Base direction mapping before automatic inversion. If auto aiming still moves away from the target, flip these.
X_AIM_RIGHT_IS_HIGH_BASE = False
Y_AIM_DOWN_IS_HIGH_BASE = True #This one should always be True, do not change.
Y_INCREASES_WHEN_DIR_HIGH = True

SERIAL_MONITOR_ENABLED_IN_AUTO = False

def auto_x_dir(direction_high):
    return (not direction_high) if AUTO_INVERT_X_DIRECTION else direction_high

def auto_y_dir(direction_high):
    return (not direction_high) if AUTO_INVERT_Y_DIRECTION else direction_high

# Final localization correction.
# Your physical testing showed that ALL motor directions are inverted, including
# the automatic localization/aiming step. These two flags flip only the aiming
# movement after heat + camera fire confirmation, without changing the relay/SMS
# safety logic. If the nozzle still moves away from the fire, toggle these values.
LOCALIZATION_INVERT_X = False
LOCALIZATION_INVERT_Y = False

def localization_x_dir(direction_high):
    # Automatic localization correction.
    # These flags are intentionally set to False in this build because the previous
    # localization version moved away from the target. Set LOCALIZATION_INVERT_X=True
    # only if X aiming is still opposite after testing.
    return (not direction_high) if LOCALIZATION_INVERT_X else direction_high

def localization_y_dir(direction_high):
    # Automatic localization correction.
    # These flags are intentionally set to False in this build because the previous
    # localization version moved away from the target. Set LOCALIZATION_INVERT_Y=True
    # only if Y aiming is still opposite after testing.
    return (not direction_high) if LOCALIZATION_INVERT_Y else direction_high

# Automatic aiming logic.
# The camera is attached to the nozzle, so the safest practical method is
# visual servo aiming: move until the fire box center is near the camera center,
# then lock the aim and spray instead of chasing the changing flame shape.
AUTO_AIM_ENABLED = True
AUTO_AIM_BEFORE_RELAY = True
FIRE_CENTER_DEADZONE_X = 0.10     # 10% of frame width / thermal width
FIRE_CENTER_DEADZONE_Y = 0.10     # 10% of frame height / thermal height

# Localization source.
# Previous versions aimed using the webcam fire bounding box. This version aims using
# the AMG8833 thermal hotspot instead, so the hottest heat source is centered in the
# thermal camera before spraying. The webcam is still used only to confirm that the
# heat source is visually fire.
AIM_USING_THERMAL_HOTSPOT = True
THERMAL_HOTSPOT_MIN_TEMP_FOR_AIM = HIGH_TEMP
# If the displayed thermal hotspot moves opposite because the AMG8833 board is
# mounted upside down or mirrored, flip these without changing motor wiring.
THERMAL_HOTSPOT_FLIP_X = False
THERMAL_HOTSPOT_FLIP_Y = False

# Step sizes are now based on your calibrated motors.
# X: max one aiming nudge ~= 2 degrees. Y: max one aiming nudge ~= 1 degree.
AUTO_AIM_MAX_STEPS_PER_UPDATE_X = int(X_STEPS_PER_DEGREE * 2.0)    # about 35 steps
AUTO_AIM_MAX_STEPS_PER_UPDATE_Y = int(Y_STEPS_PER_DEGREE * 1.0)    # about 88 steps
AUTO_AIM_MIN_STEPS = 3
AUTO_AIM_STEP_GAIN_X = int(X_STEPS_PER_DEGREE * 6.0)               # proportional nudge gain
AUTO_AIM_STEP_GAIN_Y = int(Y_STEPS_PER_DEGREE * 4.0)               # proportional nudge gain
AUTO_AIM_SETTLE_SECONDS = 0.15
AUTO_AIM_SAVE_SNAPSHOT = True
AUTO_AIM_SNAPSHOT_DIR = "fire_snapshots"

# Auto/manual mode and automatic X-axis scanning.
# MANUAL mode keeps the buttons working like before.
# AUTO mode continuously scans Motor 1 / X-axis only.
# Motor 2 / Y-axis does not move during scan; it only moves after confirmed fire.
START_IN_AUTO_MODE = False
AUTO_SCAN_ENABLED = True
AUTO_SCAN_X_DIRECTION_HIGH_FIRST = auto_x_dir(False)
AUTO_SCAN_BIDIRECTIONAL = True
AUTO_SCAN_STEPS_PER_SWEEP = X_MAX_STEPS - X_MIN_STEPS  # about 22248 steps = one full X-axis 360-degree sweep with 73:21 gearbox
AUTO_SCAN_CHUNK_STEPS = 25                          # small chunks so detection can stop scanning quickly
AUTO_SCAN_SETTLE_SECONDS = 0.005
AUTO_SCAN_STOP_ON_OBJECT_FIRE_ONLY = False  # heat-first logic now stops scan on AMG8833 heat, not webcam-only fire

# Smoke-assisted Y-axis scan.
# AUTO mode still scans X normally. If smoke is continuously detected for this
# many seconds, the Y axis will step once at every X sweep end/reversal point.
SMOKE_Y_SCAN_ENABLED = True
SMOKE_Y_SCAN_CONFIRM_SECONDS = 5.0

# Change this if your Roboflow smoke class name is different, for example "Smoke".
SMOKE_Y_SCAN_LABELS = ("smoke",)

# 4 Y changes from top to bottom, then 4 changes from bottom to top.
SMOKE_Y_SCAN_CHANGES_PER_HALF_CYCLE = 4
SMOKE_Y_SCAN_STEP_STEPS = int(Y_MAX_STEPS / SMOKE_Y_SCAN_CHANGES_PER_HALF_CYCLE)

SMOKE_Y_SCAN_SETTLE_SECONDS = 0.05

# After the first acceptable aim, lock the aim and do not keep chasing the changing fire shape.
# The relay/pump stays ON until BOTH object fire and high heat are gone.
LOCK_AIM_AFTER_CENTERED = True

# Web-camera bounding-box targeting / anti-overshoot mode.
# Coordinates are normalized: X 0.0=left, 1.0=right; Y 0.0=top, 1.0=bottom.
# New AUTO mechanics:
# - Thermal is used first because it is real-time.
# - After thermal pre-centering, the webcam/cloud model confirms fire.
# - Once confirmed, the FIRE BOUNDING BOX CENTER is frozen once.
# - X is moved to screen center.
# - Y is moved to 1/5 of the way from the bottom toward the top = 0.80.
#   This means the locked fire box is moved near the lower part of the camera view
#   before the relay/pump turns ON.
BBOX_AIM_TARGET_X_NORM = 0.50
BBOX_AIM_TARGET_Y_NORM = 0.80

# Kept for display/backward compatibility. The new code uses BBOX_AIM_TARGET_*.
WATER_HIT_TARGET_X_NORM = BBOX_AIM_TARGET_X_NORM
WATER_HIT_TARGET_Y_NORM = BBOX_AIM_TARGET_Y_NORM

# IMPORTANT: This prevents laggy camera/inference updates from causing chase/overshoot.
# After heat is detected and the webcam confirms fire, the current webcam fire bounding box is frozen.
# The system calculates one aiming correction from that frozen point, locks it, then ignores later camera changes
# until the relay spray cycle finishes and the relay turns off.
FREEZE_AIM_TARGET_AFTER_CONFIRMATION = True
ONE_SHOT_AIM_AFTER_CONFIRMATION = True

# Spray retry logic.
# Cycles 1-3: normal aim -> spray 5s -> relay OFF -> re-check fire.
# If fire still exists on the 4th response cycle, the system marks it uncontrollable,
# sends the backup/emergency SMS, and keeps extinguishing instead of returning to scan.
MAX_CONTROLLED_SPRAY_CYCLES = 3
UNCONTROLLABLE_CONTINUOUS_RELAY = True

# Larger one-shot limits allow one deliberate correction instead of many small feedback corrections.
# If the system overshoots mechanically, lower these degree values. If it undershoots, raise them slightly.
ONE_SHOT_AIM_MAX_DEGREES_X = 8.0
ONE_SHOT_AIM_MAX_DEGREES_Y = 5.0
ONE_SHOT_AIM_MAX_STEPS_X = int(X_STEPS_PER_DEGREE * ONE_SHOT_AIM_MAX_DEGREES_X)
ONE_SHOT_AIM_MAX_STEPS_Y = int(Y_STEPS_PER_DEGREE * ONE_SHOT_AIM_MAX_DEGREES_Y)
ONE_SHOT_AIM_GAIN_X = int(X_STEPS_PER_DEGREE * 14.0)
ONE_SHOT_AIM_GAIN_Y = int(Y_STEPS_PER_DEGREE * 10.0)

# Thermal pre-centering stage.
# AUTO mode now centers the thermal hotspot first, before asking the delayed webcam/cloud model to confirm fire.
# This gives the system a stable thermal reference before freezing the aim target and doing the final water-zone correction.
THERMAL_PRECENTER_BEFORE_CAMERA_CONFIRM = True
THERMAL_PRECENTER_TARGET_X_NORM = 0.50
THERMAL_PRECENTER_TARGET_Y_NORM = 0.50
THERMAL_PRECENTER_DEADZONE_X = 0.10
THERMAL_PRECENTER_DEADZONE_Y = 0.10
THERMAL_PRECENTER_MAX_STEPS_PER_UPDATE_X = int(X_STEPS_PER_DEGREE * 1.5)
THERMAL_PRECENTER_MAX_STEPS_PER_UPDATE_Y = int(Y_STEPS_PER_DEGREE * 0.8)
THERMAL_PRECENTER_GAIN_X = int(X_STEPS_PER_DEGREE * 5.0)
THERMAL_PRECENTER_GAIN_Y = int(Y_STEPS_PER_DEGREE * 3.5)
THERMAL_PRECENTER_SETTLE_SECONDS = 0.10

# Firebase logging cooldowns
FIREBASE_LOG_INTERVAL_SECONDS = 1.0
OBJECT_FIREBASE_COOLDOWN_SECONDS = 5.0

# Motor 1 LEFT / RIGHT, BOARD pin numbering
DIR1_PIN = 12
PUL1_PIN = 16

# Motor 2 UP / DOWN, BOARD pin numbering
DIR2_PIN = 18
PUL2_PIN = 22

# Stepper pulse speed
STEPPER_PULSE_DELAY_SECONDS = 0.0002

# Window
WINDOW_TITLE = "Blastoys Fire Suppression Control"
START_FULLSCREEN = True

# UI and alert settings
# The alert phone/message can be changed from the application using Save Alert Settings.
# It also saves to this JSON file so the next run remembers your last settings.
ALERT_SETTINGS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "blastoys_alert_settings.json")
DEFAULT_ALERT_MESSAGE = (
    "CONFIRMED FIRE ALERT! The system detected high heat and visible fire. "
    "Relay/pump has been activated."
)
DEFAULT_ESCALATION_PHONE_NUMBER = "+639XXXXXXXXX"
DEFAULT_ESCALATION_MESSAGE = (
    "The system failed to put out the fire within the set time limit. "
    "Fire department intervention is needed."
)
# Optional automatic call recipients. Put one or more numbers separated by commas/new lines in the UI.
DEFAULT_CALL_RECIPIENTS = PHONE_NUMBER

# Automatic calls are disabled so the single SIM900A is free for primary and
# escalation SMS alerts during automatic fire response.
# Manual call buttons in the GUI remain available for testing.
AUTO_CALL_BEFORE_RELAY = False
AUTO_CALL_COOLDOWN_SECONDS = 120
AUTO_CALL_RING_SECONDS_PER_NUMBER = 8


# =====================================================
# GLOBAL STATE
# =====================================================

wait_response=True

# Master Firebase switch from the GUI.
FIREBASE_UPLOAD_ENABLED = False

# Automatic SMS alerts.
# Relay activation still happens first; after the relay is commanded ON,
# the system starts the primary SMS instead of an automatic phone call.
AUTO_PRIMARY_SMS_ENABLED = True
AUTO_ESCALATION_SMS_ENABLED = True

# The escalation number is contacted automatically only when the fire is formally
# marked UNCONTROLLABLE after the configured spray-cycle limit is exhausted.
# This prevents a time-based backup SMS from competing with the emergency SMS.
ESCALATION_SMS_ONLY_WHEN_UNCONTROLLABLE = True
#0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
CONFIRMED_FIRE_FIREBASE_ENABLED = False 

relay_command_busy = False


arduino = None
serial_lock = threading.Lock()

# Keeps complete SIM900 SMS/CALL transactions from interrupting each other.
# This prevents a CALL command from overwriting the secondary SMS phone number.
sim900_transaction_lock = threading.RLock()

program_running = True


running1 = False
running2 = False

relay_is_on = False
# Cache the last relay command sent to Arduino. This prevents the GUI/auto loop
# from flooding USB serial with repeated RELAY_OFF or RELAY_ON commands.
last_relay_command_sent = None
last_relay_command_time = 0.0
RELAY_COMMAND_MIN_INTERVAL_SECONDS = 0.35
sms_is_sending = False
call_is_sending = False

# Relay control mode:
# MANUAL = manual Relay ON/OFF buttons directly control the pump.
# AUTO = automatic logic controls relay, but only after heat + fire confirmation and aiming/localization.
relay_control_mode_lock = threading.Lock()
relay_control_mode = "AUTO"

last_sms_time = 0.0
last_escalation_sms_time = 0.0
last_auto_call_time = 0.0
last_thermal_log_time = 0.0
last_object_firebase_time = 0.0

thermal_alert_active = False
object_alert_active = False
confirmed_fire_active = False
last_thermal_fire_time = 0.0
last_object_fire_time = 0.0
last_confirmed_fire_time = 0.0
last_confirmed_firebase_time = 0.0
spray_started_time = 0.0

alert_settings_lock = threading.Lock()
saved_alert_phone_number = PHONE_NUMBER
saved_alert_message = DEFAULT_ALERT_MESSAGE
saved_call_recipients = DEFAULT_CALL_RECIPIENTS
saved_escalation_phone_number = DEFAULT_ESCALATION_PHONE_NUMBER
saved_escalation_message = DEFAULT_ESCALATION_MESSAGE
saved_fail_time_limit_seconds = DEFAULT_FAIL_TIME_LIMIT_SECONDS

fire_sms_event_active = False
fire_sms_event_start_time = 0.0
primary_sms_sent_for_current_fire = False
escalation_sms_sent_for_current_fire = True

fire_confirmation_start_time = 0.0
fire_confirmation_status = "WAITING FOR HEAT"
heat_confirmation_object_seen = False
heat_ignore_until_time = 0.0

latest_thermal_frame = None
latest_thermal_status = "THERMAL STARTING"
latest_thermal_max_temp = 0.0
latest_thermal_hotspot_x = None  # normalized 0.0 left to 1.0 right
latest_thermal_hotspot_y = None  # normalized 0.0 top to 1.0 bottom
latest_thermal_hotspot_temp = 0.0
thermal_lock = threading.Lock()

latest_predictions: List[Dict[str, Any]] = []
latest_inference_error = ""
latest_inference_latency_ms = 0.0
latest_inference_mode = "starting"

latest_object_fire_detected = False
latest_object_smoke_detected = False
latest_object_labels: List[str] = []
latest_object_confidence = 0.0
last_object_smoke_time = 0.0

latest_fire_center_x = None
latest_fire_center_y = None
latest_fire_bbox = None
latest_fire_frame = None
last_aim_status = "AUTO AIM: IDLE"
last_saved_fire_snapshot = ""
aimed_at_fire = False
fire_event_snapshot_saved = False
prediction_lock = threading.Lock()

motor_motion_lock = threading.Lock()
x_position_lock = threading.Lock()
y_position_lock = threading.Lock()
current_x_position_steps = X_START_REFERENCE_STEPS
current_y_position_steps = Y_START_REFERENCE_STEPS

mode_lock = threading.Lock()
operation_mode = "AUTO" if START_IN_AUTO_MODE else "MANUAL"

auto_scan_direction_high = AUTO_SCAN_X_DIRECTION_HIGH_FIRST
auto_scan_steps_in_sweep = 0
auto_scan_status = "AUTO SCAN: READY" if START_IN_AUTO_MODE else "AUTO SCAN: OFF"
aim_locked_on_fire = False

smoke_scan_start_time = 0.0
smoke_y_scan_direction_down = True
last_smoke_y_scan_status = "SMOKE Y SCAN: IDLE"

# Frozen aiming target for anti-overshoot mode.
# These values are captured once after heat + webcam fire confirmation and reused until relay OFF.
locked_aim_target_x = None
locked_aim_target_y = None
locked_aim_target_source = ""
locked_aim_target_temp = 0.0
aim_once_motion_done = False
auto_response_phase = "IDLE"
thermal_precenter_done_for_current_heat = False
thermal_precenter_motion_count = 0

# Fire response retry/uncontrollable state.
controlled_spray_cycle_count = 0
fire_uncontrollable = False
uncontrollable_sms_sent = False
uncontrollable_sms_sending = False
last_uncontrollable_sms_attempt_time = 0.0
UNCONTROLLABLE_SMS_RETRY_SECONDS = 10.0

# Last webcam fire bounding box record.
# This is updated only when the cloud model sees class "fire".
# It is intentionally not cleared immediately when a later frame has no detection,
# because cloud inference can be delayed/jittery.
last_seen_fire_bbox_x = None
last_seen_fire_bbox_y = None
last_seen_fire_bbox_confidence = 0.0
last_seen_fire_bbox_time = 0.0

inference_thread_busy = False
inference_lock = threading.Lock()

camera_capture = None
http_session = requests.Session()

frame_times = deque(maxlen=30)

serial_monitor_visible = True
serial_log_text = None
serial_monitor_card = None
serial_toggle_button = None

invalid_number_warning_window = None
manual_relay_confirm_window = None

def get_thermal_colormap():
    # Some older Jetson OpenCV builds do not include COLORMAP_INFERNO.
    # JET is used as a safe fallback so the thermal feed still displays.
    return getattr(cv2, "COLORMAP_INFERNO", cv2.COLORMAP_JET)


def set_thermal_status(status, max_temp=None, frame=None):
    global latest_thermal_frame, latest_thermal_status, latest_thermal_max_temp
    with thermal_lock:
        latest_thermal_status = str(status)
        if max_temp is not None:
            latest_thermal_max_temp = float(max_temp)
        if frame is not None:
            latest_thermal_frame = frame


def toggle_firebase_upload():

    global FIREBASE_UPLOAD_ENABLED
    global firebase_button_var

    FIREBASE_UPLOAD_ENABLED = not FIREBASE_UPLOAD_ENABLED

    firebase_button_var.set(
        "Firebase Upload: ON"
        if FIREBASE_UPLOAD_ENABLED
        else
        "Firebase Upload: OFF"
    )

    print(
        "Firebase Upload:",
        "ENABLED" if FIREBASE_UPLOAD_ENABLED else "DISABLED",
        flush=True
    )

# =====================================================
# ALERT SETTINGS HELPERS
# =====================================================

def normalize_phone_number_for_check(number):
    """
    Keep only the characters needed for checking a Philippine mobile number.
    Spaces, dashes, and parentheses are ignored.
    """
    number = str(number or "").strip()
    number = number.replace(" ", "").replace("-", "").replace("(", "").replace(")", "")
    return number


def is_valid_ph_mobile_number(number):
    """
    Accepted Philippine mobile number formats:
    - 09XXXXXXXXX
    - +639XXXXXXXXX
    - 639XXXXXXXXX

    These all represent a Philippine mobile number starting with 9.
    """
    number = normalize_phone_number_for_check(number)
    return re.match(r"^(09\d{9}|\+639\d{9}|639\d{9})$", number) is not None


def parse_recipient_numbers(text):
    raw = str(text or "").replace(";", ",").replace("\n", ",")
    numbers = []
    for part in raw.split(","):
        number = part.strip()
        if number:
            numbers.append(number)
    # Remove duplicates while keeping order.
    unique = []
    for number in numbers:
        if number not in unique:
            unique.append(number)
    return unique


def get_invalid_phone_numbers(primary_number, escalation_number, call_recipients):
    invalid = []

    if not is_valid_ph_mobile_number(primary_number):
        invalid.append(("Primary SMS Phone Number", primary_number))

    if not is_valid_ph_mobile_number(escalation_number):
        invalid.append(("Backup/Escalation Phone Number", escalation_number))

    call_list = parse_recipient_numbers(call_recipients)
    for call_number in call_list:
        if not is_valid_ph_mobile_number(call_number):
            invalid.append(("Automatic Call Recipient", call_number))

    return invalid


def load_alert_settings():
    """Load saved primary SMS, backup SMS, fail timeout, and call recipients from JSON."""
    global saved_alert_phone_number, saved_alert_message, saved_call_recipients
    global saved_escalation_phone_number, saved_escalation_message, saved_fail_time_limit_seconds

    try:
        if os.path.exists(ALERT_SETTINGS_FILE):
            with open(ALERT_SETTINGS_FILE, "r") as handle:
                data = json.load(handle)

            number = str(data.get("phone_number", "")).strip()
            message = str(data.get("message", "")).strip()
            call_recipients = data.get("call_recipients", "")
            escalation_number = str(data.get("escalation_phone_number", "")).strip()
            escalation_message = str(data.get("escalation_message", "")).strip()
            fail_time = data.get("fail_time_limit_seconds", DEFAULT_FAIL_TIME_LIMIT_SECONDS)

            if isinstance(call_recipients, list):
                call_recipients = ", ".join(str(x).strip() for x in call_recipients if str(x).strip())
            call_recipients = str(call_recipients).strip()

            try:
                fail_time = float(fail_time)
                if fail_time <= 0:
                    fail_time = DEFAULT_FAIL_TIME_LIMIT_SECONDS
            except Exception:
                fail_time = DEFAULT_FAIL_TIME_LIMIT_SECONDS

            with alert_settings_lock:
                if number:
                    saved_alert_phone_number = number
                if message:
                    saved_alert_message = message
                if call_recipients:
                    saved_call_recipients = call_recipients
                if escalation_number:
                    saved_escalation_phone_number = escalation_number
                if escalation_message:
                    saved_escalation_message = escalation_message
                saved_fail_time_limit_seconds = fail_time

            print("Loaded alert settings from", ALERT_SETTINGS_FILE, flush=True)
    except Exception as error:
        print("Could not load alert settings:", error, flush=True)


def save_alert_settings(number, message, call_recipients=None, escalation_number=None, escalation_message=None, fail_time_limit_seconds=None):
    """Save primary SMS, backup SMS, fail timeout, and automatic call recipients from the UI."""
    global saved_alert_phone_number, saved_alert_message, saved_call_recipients
    global saved_escalation_phone_number, saved_escalation_message, saved_fail_time_limit_seconds

    number = str(number).strip()
    message = str(message).strip()
    call_recipients = str(call_recipients if call_recipients is not None else saved_call_recipients).strip()
    escalation_number = str(escalation_number if escalation_number is not None else saved_escalation_phone_number).strip()
    escalation_message = str(escalation_message if escalation_message is not None else saved_escalation_message).strip()

    try:
        fail_time = float(fail_time_limit_seconds if fail_time_limit_seconds is not None else saved_fail_time_limit_seconds)
        if fail_time <= 0:
            fail_time = DEFAULT_FAIL_TIME_LIMIT_SECONDS
    except Exception:
        fail_time = DEFAULT_FAIL_TIME_LIMIT_SECONDS

    if not number:
        return False, "Primary phone number is empty."
    if not message:
        return False, "Primary message is empty."
    if not escalation_number:
        return False, "Backup/escalation phone number is empty."
    if not escalation_message:
        return False, "Backup/escalation message is empty."

    call_list = parse_recipient_numbers(call_recipients)
    if not call_list:
        call_recipients = number
        call_list = [number]

    with alert_settings_lock:
        saved_alert_phone_number = number
        saved_alert_message = message
        saved_call_recipients = ", ".join(call_list)
        saved_escalation_phone_number = escalation_number
        saved_escalation_message = escalation_message
        saved_fail_time_limit_seconds = fail_time

    try:
        with open(ALERT_SETTINGS_FILE, "w") as handle:
            json.dump(
                {
                    "phone_number": number,
                    "message": message,
                    "call_recipients": call_list,
                    "escalation_phone_number": escalation_number,
                    "escalation_message": escalation_message,
                    "fail_time_limit_seconds": fail_time,
                },
                handle,
                indent=2,
            )
        print("Saved alert settings to", ALERT_SETTINGS_FILE, flush=True)
        return True, "Saved primary SMS, backup SMS, fail timeout, and call settings."
    except Exception as error:
        print("Could not save alert settings:", error, flush=True)
        return True, "Settings active, but file save failed: {}".format(error)


def get_saved_alert_settings():
    with alert_settings_lock:
        return saved_alert_phone_number, saved_alert_message


def get_saved_call_recipients():
    with alert_settings_lock:
        return parse_recipient_numbers(saved_call_recipients)


def get_saved_escalation_settings():
    with alert_settings_lock:
        return saved_escalation_phone_number, saved_escalation_message, saved_fail_time_limit_seconds


def build_confirmed_fire_message(base_message, max_temp, confidence):
    """Append live fire data to the saved primary UI message."""
    base_message = str(base_message).strip() or DEFAULT_ALERT_MESSAGE
    details = " Max Temp: {:.2f}C | Object Confidence: {:.0f}%".format(
        max_temp,
        confidence * 100.0,
    )
    combined = base_message + details
    return combined[:300]


def build_escalation_fire_message(base_message, max_temp, confidence, elapsed_seconds):
    """Append live fire data to the saved backup/escalation UI message."""
    base_message = str(base_message).strip() or DEFAULT_ESCALATION_MESSAGE
    details = " Max Temp: {:.2f}C | Object Confidence: {:.0f}% | Fire Duration: {:.1f}s".format(
        max_temp,
        confidence * 100.0,
        elapsed_seconds,
    )
    combined = base_message + details
    return combined[:300]

# =====================================================
# MODE / SCAN HELPERS
# =====================================================

def ensure_mode_globals():
    """
    Defensive helper for Jetson/Python 3.6 testing.
    If the file was edited and mode_lock/operation_mode was accidentally removed,
    recreate them instead of crashing the auto-scan thread or GUI.
    """
    global mode_lock, operation_mode, auto_scan_status

    if "mode_lock" not in globals():
        mode_lock = threading.Lock()

    if "operation_mode" not in globals():
        operation_mode = "AUTO" if START_IN_AUTO_MODE else "MANUAL"

    if "auto_scan_status" not in globals():
        auto_scan_status = "AUTO SCAN: READY" if operation_mode == "AUTO" else "AUTO SCAN: OFF"


def set_manual_mode():
    global operation_mode, running1, running2, auto_scan_status
    ensure_mode_globals()
    with mode_lock:
        operation_mode = "MANUAL"
    running1 = False
    running2 = False
    auto_scan_status = "AUTO SCAN: OFF"
    try:
        GPIO.output(PUL1_PIN, GPIO.LOW)
        GPIO.output(PUL2_PIN, GPIO.LOW)
    except Exception:
        pass
    try:
        reset_aim_lock_state("MANUAL MODE")
    except Exception:
        pass
    try:
        reset_smoke_y_scan_state("MANUAL MODE")
    except Exception:
        pass
    
    print("Mode changed to MANUAL", flush=True)


def set_auto_mode():
    global operation_mode, running1, running2, auto_scan_status
    ensure_mode_globals()

    with mode_lock:
        operation_mode = "AUTO"

    # Important fix:
    # AUTO operation mode should also restore AUTO relay control.
    # Otherwise, pressing Manual Relay ON/OFF leaves relay_control_mode = MANUAL,
    # and automatic fire response will refuse to activate the relay.
    set_relay_control_mode("AUTO")

    running1 = False
    running2 = False
    auto_scan_status = "AUTO SCAN: READY"

    try:
        GPIO.output(PUL1_PIN, GPIO.LOW)
        GPIO.output(PUL2_PIN, GPIO.LOW)
    except Exception:
        pass

    try:
        reset_aim_lock_state("AUTO MODE READY")
    except Exception:
        pass

    try:
        reset_smoke_y_scan_state("AUTO MODE READY")
    except Exception:
        pass

    # Safety: make sure the pump is OFF first.
    # Automatic logic will turn it ON only after valid heat + fire confirmation.
    try:
        relay_off(force=True)
    except Exception:
        pass

    print("Mode changed to AUTO", flush=True)


def get_operation_mode():
    ensure_mode_globals()
    with mode_lock:
        return operation_mode


def recent_object_fire():
    return (time.time() - last_object_fire_time) <= OBJECT_FIRE_HOLD_SECONDS


def recent_thermal_fire():
    return (time.time() - last_thermal_fire_time) <= THERMAL_FIRE_HOLD_SECONDS


def confirmed_fire_now():
    return recent_object_fire() and recent_thermal_fire()

def prediction_has_smoke(predictions):
    """
    Returns True if the object detector sees a smoke class.

    This does not affect fire confirmation, relay, SMS, or thermal logic.
    It is only used to decide whether AUTO search should also step the Y axis.
    """
    smoke_labels = [str(label).strip().lower() for label in SMOKE_Y_SCAN_LABELS]

    for prediction in iter_predictions(predictions):
        label = str(prediction.get("class") or "").strip().lower()
        if label in smoke_labels:
            return True

    return False


def reset_smoke_y_scan_state(reason=""):
    global smoke_scan_start_time, smoke_y_scan_direction_down, last_smoke_y_scan_status

    smoke_scan_start_time = 0.0
    smoke_y_scan_direction_down = True
    last_smoke_y_scan_status = "SMOKE Y SCAN: RESET"
    if reason:
        last_smoke_y_scan_status += " - " + str(reason)[:35]


def smoke_y_scan_ready():
    """
    Smoke must be continuously/recently detected for SMOKE_Y_SCAN_CONFIRM_SECONDS
    before Y scan steps are allowed.

    The Y axis still only moves at X sweep end points, not every frame.
    """
    global smoke_scan_start_time, last_smoke_y_scan_status

    if not SMOKE_Y_SCAN_ENABLED:
        last_smoke_y_scan_status = "SMOKE Y SCAN: DISABLED"
        return False

    if get_operation_mode() != "AUTO":
        reset_smoke_y_scan_state("NOT AUTO")
        return False

    now = time.time()

    with prediction_lock:
        smoke_detected = bool(latest_object_smoke_detected)

    if not smoke_detected:
        smoke_scan_start_time = 0.0
        last_smoke_y_scan_status = "SMOKE Y SCAN: WAITING FOR SMOKE"
        return False

    if smoke_scan_start_time <= 0.0:
        smoke_scan_start_time = now

    elapsed = now - smoke_scan_start_time

    if elapsed < SMOKE_Y_SCAN_CONFIRM_SECONDS:
        last_smoke_y_scan_status = "SMOKE Y SCAN: CONFIRMING {:.1f}/{:.1f}s".format(
            elapsed,
            SMOKE_Y_SCAN_CONFIRM_SECONDS,
        )
        return False

    last_smoke_y_scan_status = "SMOKE Y SCAN: ARMED"
    return True


def smoke_y_step_at_x_reversal():
    """
    Move Y one step only when X reaches the sweep end and reverses.

    Downward movement continues until the Y software limit blocks it.
    Then it reverses upward. At the top, it reverses downward again.
    """
    global smoke_y_scan_direction_down, last_smoke_y_scan_status

    if not smoke_y_scan_ready():
        return 0

    requested_steps = max(1, int(SMOKE_Y_SCAN_STEP_STEPS))

    # Down uses the same base direction used by the auto/localization Y aiming logic.
    if smoke_y_scan_direction_down:
        direction_high = localization_y_dir(Y_AIM_DOWN_IS_HIGH_BASE)
        direction_text = "DOWN"
    else:
        direction_high = localization_y_dir(not Y_AIM_DOWN_IS_HIGH_BASE)
        direction_text = "UP"

    moved = pulse_y_limited(direction_high, requested_steps)

    # If the limit blocked movement, reverse direction and try once in the opposite direction.
    if moved <= 0:
        smoke_y_scan_direction_down = not smoke_y_scan_direction_down

        if smoke_y_scan_direction_down:
            direction_high = localization_y_dir(Y_AIM_DOWN_IS_HIGH_BASE)
            direction_text = "DOWN"
        else:
            direction_high = localization_y_dir(not Y_AIM_DOWN_IS_HIGH_BASE)
            direction_text = "UP"

        moved = pulse_y_limited(direction_high, requested_steps)

    # If the move was clipped by the software limit, reverse for the next X reversal.
    if moved < requested_steps:
        smoke_y_scan_direction_down = not smoke_y_scan_direction_down

    with y_position_lock:
        y_steps_text = current_y_position_steps
        y_degrees_text = current_y_position_steps / max(1.0, Y_STEPS_PER_DEGREE)

    last_smoke_y_scan_status = "SMOKE Y SCAN: {} {} steps | Y={} / {:.1f}deg".format(
        direction_text,
        moved,
        y_steps_text,
        y_degrees_text,
    )
    print(last_smoke_y_scan_status, flush=True)

    if moved > 0:
        time.sleep(SMOKE_Y_SCAN_SETTLE_SECONDS)

    return moved



def center_thermal_hotspot_before_confirmation():
    """
    AUTO-only thermal pre-centering stage.

    When AMG8833 first detects HIGH/CRITICAL heat, the X scan is already stopped.
    This function uses the fast thermal hotspot, not the delayed webcam result,
    to move the hotspot toward the thermal center. Only after this is centered
    does the system begin the webcam/cloud fire confirmation timer.
    """
    global thermal_precenter_done_for_current_heat, thermal_precenter_motion_count
    global fire_confirmation_status, last_aim_status

    if not THERMAL_PRECENTER_BEFORE_CAMERA_CONFIRM:
        thermal_precenter_done_for_current_heat = True
        return True

    if get_operation_mode() != "AUTO":
        return False

    if thermal_precenter_done_for_current_heat:
        return True

    with thermal_lock:
        cx = latest_thermal_hotspot_x
        cy = latest_thermal_hotspot_y
        hotspot_temp = latest_thermal_hotspot_temp

    if cx is None or cy is None or hotspot_temp < THERMAL_HOTSPOT_MIN_TEMP_FOR_AIM:
        fire_confirmation_status = "HEAT FOUND - WAITING FOR THERMAL HOTSPOT"
        last_aim_status = "AUTO AIM: PRE-CENTER WAITING FOR HOTSPOT"
        return False

    cx = clamp01(cx)
    cy = clamp01(cy)

    error_x = cx - clamp01(THERMAL_PRECENTER_TARGET_X_NORM)
    error_y = cy - clamp01(THERMAL_PRECENTER_TARGET_Y_NORM)

    x_aligned = abs(error_x) <= THERMAL_PRECENTER_DEADZONE_X
    y_aligned = abs(error_y) <= THERMAL_PRECENTER_DEADZONE_Y

    if x_aligned and y_aligned:
        thermal_precenter_done_for_current_heat = True
        fire_confirmation_status = "THERMAL CENTERED - READY FOR CAMERA CHECK"
        last_aim_status = "AUTO AIM: THERMAL PRE-CENTERED"
        print(
            "Thermal hotspot centered before camera confirmation at x={:.2f}, y={:.2f}".format(cx, cy),
            flush=True,
        )
        return True

    x_steps = 0
    y_steps = 0

    if not x_aligned:
        x_steps = int(min(
            THERMAL_PRECENTER_MAX_STEPS_PER_UPDATE_X,
            max(AUTO_AIM_MIN_STEPS, abs(error_x) * THERMAL_PRECENTER_GAIN_X),
        ))
        x_direction_high = localization_x_dir(X_AIM_RIGHT_IS_HIGH_BASE if error_x > 0 else not X_AIM_RIGHT_IS_HIGH_BASE)
        pulse_motor_steps(DIR1_PIN, PUL1_PIN, x_direction_high, x_steps)

    if not y_aligned:
        y_steps = int(min(
            THERMAL_PRECENTER_MAX_STEPS_PER_UPDATE_Y,
            max(AUTO_AIM_MIN_STEPS, abs(error_y) * THERMAL_PRECENTER_GAIN_Y),
        ))
        y_direction_high = localization_y_dir(Y_AIM_DOWN_IS_HIGH_BASE if error_y > 0 else not Y_AIM_DOWN_IS_HIGH_BASE)
        y_steps = pulse_y_limited(y_direction_high, y_steps)

    thermal_precenter_motion_count += 1
    fire_confirmation_status = "CENTERING THERMAL HOTSPOT X={} Y={} ERR=({:+.2f},{:+.2f})".format(
        x_steps,
        y_steps,
        error_x,
        error_y,
    )
    last_aim_status = "AUTO AIM: PRE-CENTER X={} Y={}".format(x_steps, y_steps)
    time.sleep(THERMAL_PRECENTER_SETTLE_SECONDS)
    return False

def stable_confirmed_fire_now():
    """
    Heat-first confirmation stage for AUTO mode.

    The system now stops scanning because AMG8833 found heat first. It then
    looks at the stopped camera view for FIRE_CONFIRMATION_SECONDS. If the
    webcam sees class "fire" at least once during that window and heat is
    still present, fire is confirmed. If not, the heat source is ignored for
    HEAT_SOURCE_IGNORE_SECONDS so the X scan can move past non-fire heat
    sources like boiling water.
    """
    global fire_confirmation_start_time, fire_confirmation_status
    global heat_confirmation_object_seen, heat_ignore_until_time

    now = time.time()

    # Do not let the same non-fire heat source stop scanning repeatedly.
    if now < heat_ignore_until_time:
        remaining = heat_ignore_until_time - now
        fire_confirmation_status = "IGNORING HEAT SOURCE {:.1f}s".format(remaining)
        fire_confirmation_start_time = 0.0
        heat_confirmation_object_seen = False
        return False

    # Heat is the first trigger. If there is no heat, keep scanning.
    if not recent_thermal_fire():
        fire_confirmation_status = "WAITING FOR HEAT"
        fire_confirmation_start_time = 0.0
        heat_confirmation_object_seen = False
        try:
            global thermal_precenter_done_for_current_heat, thermal_precenter_motion_count
            thermal_precenter_done_for_current_heat = False
            thermal_precenter_motion_count = 0
        except Exception:
            pass
        return False

    # New process step 2: center the thermal hotspot first using the real-time AMG8833 position.
    # Do not start the delayed webcam/cloud confirmation timer until thermal pre-centering is done.
    if not center_thermal_hotspot_before_confirmation():
        fire_confirmation_start_time = 0.0
        heat_confirmation_object_seen = False
        return False

    # Heat found and thermal hotspot centered: stop and begin/continue a visual webcam check.
    if fire_confirmation_start_time <= 0.0:
        fire_confirmation_start_time = now
        heat_confirmation_object_seen = False
        fire_confirmation_status = "THERMAL CENTERED - CHECKING CAMERA 0.0/{:.1f}s".format(FIRE_CONFIRMATION_SECONDS)

    if recent_object_fire():
        heat_confirmation_object_seen = True

    elapsed = now - fire_confirmation_start_time

    if elapsed < FIRE_CONFIRMATION_SECONDS:
        fire_confirmation_status = "HEAT FOUND - CAMERA {} {:.1f}/{:.1f}s".format(
            "FIRE SEEN" if heat_confirmation_object_seen else "NO FIRE YET",
            elapsed,
            FIRE_CONFIRMATION_SECONDS,
        )
        return False

    # At the end of the 3 seconds, require heat still present and webcam fire seen.
    if recent_thermal_fire() and heat_confirmation_object_seen:
        fire_confirmation_status = "CONFIRMED FIRE AFTER HEAT CHECK"
        return True

    # Heat existed, but webcam did not confirm fire. Ignore this source briefly.
    fire_confirmation_status = "NO FIRE IN CAMERA - IGNORE HEAT {:.1f}s".format(HEAT_SOURCE_IGNORE_SECONDS)
    fire_confirmation_start_time = 0.0
    heat_confirmation_object_seen = False
    try:
        thermal_precenter_done_for_current_heat = False
        thermal_precenter_motion_count = 0
    except Exception:
        pass
    heat_ignore_until_time = now + HEAT_SOURCE_IGNORE_SECONDS
    return False


# =====================================================
# FIREBASE HELPERS
# =====================================================

def normalize_firebase_url(firebase_url: str) -> str:
    normalized = (firebase_url or "").strip().rstrip("/")
    if normalized.endswith(".json"):
        normalized = normalized[:-5]
    return normalized


def build_firebase_rest_url(firebase_url: str, readings_path: str) -> str:
    base_url = normalize_firebase_url(firebase_url)
    clean_path = "/".join(
        segment.strip().strip("/")
        for segment in (readings_path or "").split("/")
        if segment.strip()
    )
    if not base_url:
        return ""
    if clean_path:
        return f"{base_url}/{clean_path}.json"
    return f"{base_url}.json"


FIREBASE_ENDPOINT = build_firebase_rest_url(FIREBASE_BASE_URL, FIREBASE_READINGS_PATH)


def iso_timestamp() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


def publish_to_firebase(payload: Dict[str, Any], timeout: float = 0.8) -> None:
    if not FIREBASE_ENDPOINT:
        return

    params = {"auth": FIREBASE_AUTH_TOKEN} if FIREBASE_AUTH_TOKEN else None
    response = http_session.post(
        FIREBASE_ENDPOINT,
        params=params,
        json=payload,
        timeout=timeout,
    )
    response.raise_for_status()


# =====================================================
# GPIO SETUP
# =====================================================

def setup_gpio() -> None:
    try:
        GPIO.setmode(GPIO.BOARD)
    except ValueError:
        GPIO.cleanup()
        GPIO.setmode(GPIO.BOARD)

    GPIO.setup(DIR1_PIN, GPIO.OUT)
    GPIO.setup(PUL1_PIN, GPIO.OUT)
    GPIO.setup(DIR2_PIN, GPIO.OUT)
    GPIO.setup(PUL2_PIN, GPIO.OUT)

    GPIO.output(DIR1_PIN, GPIO.LOW)
    GPIO.output(PUL1_PIN, GPIO.LOW)
    GPIO.output(DIR2_PIN, GPIO.LOW)
    GPIO.output(PUL2_PIN, GPIO.LOW)


# =====================================================
# ARDUINO SERIAL / SIM900A / RELAY FUNCTIONS
# =====================================================

def open_arduino():
    ser = serial.Serial(
        port=ARDUINO_PORT,
        baudrate=BAUD_RATE,
        timeout=0.2,
        write_timeout=2,
    )

    # Opening USB serial usually resets the Arduino Uno. Give the Arduino
    # sketch time to boot and print ARDUINO_SIM900_RELAY_READY.
    time.sleep(3.0)
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass
    return ser


def ensure_arduino_connected():
    global arduino

    if arduino is not None:
        try:
            if getattr(arduino, "is_open", True):
                return True
        except Exception:
            pass

    try:
        print("Reconnecting Arduino serial...", flush=True)
        arduino = open_arduino()
        return True
    except Exception as error:
        print("Arduino reconnect failed:", error, flush=True)
        arduino = None
        return False


def read_response(timeout=2):
    global arduino

    start = time.time()
    response = ""

    while time.time() - start < timeout and program_running:
        try:
            if arduino and arduino.in_waiting:
                data = arduino.readline().decode(errors="ignore").strip()
                if data:
                    print(data, flush=True)
                    response += data + "\n"
            else:
                time.sleep(0.01)
        except Exception as error:
            print("Serial read failed:", error, flush=True)
            break

    return response


def send_command(command,timeout=2,clear_input=False,wait_response=True):    
    global arduino

    if not ensure_arduino_connected():
        print("Arduino is not connected.", flush=True)
        return ""

    with serial_lock:
        print("Sending:", command, flush=True)
        try:
            if clear_input:
                try:
                    arduino.reset_input_buffer()
                except Exception:
                    pass

            arduino.write((command.strip() + "\n").encode())
            arduino.flush()
            time.sleep(SIM900_COMMAND_DELAY_SECONDS)

            if wait_response:
                return read_response(timeout)

            return ""


        except Exception as error:
            print("Serial command failed:", error, flush=True)
            try:
                arduino.close()
            except Exception:
                pass
            arduino = None
            return ""


def send_sms(number, message):
    """
    Queue one SMS request on the Arduino without waiting for the SIM900A
    transaction to finish.

    The Arduino now handles SMS sending in a non-blocking state machine.
    This allows RELAY_OFF to be sent during an active SMS transaction.
    """
    global sms_is_sending

    if sms_is_sending:
        print("SMS request is already being queued. Skipping duplicate request.", flush=True)
        return False

    sms_is_sending = True

    try:
        number = str(number).strip()
        message = str(message).strip()

        with sim900_transaction_lock:
            print("Queueing SMS request. Destination:", number, flush=True)

            send_command(
                "NUMBER:" + number,
                timeout=0,
                clear_input=False,
                wait_response=False,
            )

            time.sleep(0.05)

            send_command(
                "MESSAGE:" + message,
                timeout=0,
                clear_input=False,
                wait_response=False,
            )

            time.sleep(0.05)

            send_command(
                "SMS",
                timeout=0,
                clear_input=False,
                wait_response=False,
            )

        print(
            "SMS request sent to Arduino. Arduino will report SMS_SUCCESS or SMS_FAILED asynchronously.",
            flush=True,
        )
        return True

    except Exception as error:
        print("Failed to queue SMS request:", error, flush=True)
        return False

    finally:
        sms_is_sending = False


def make_call(number):
    """
    Start one call without allowing it to interrupt an SMS transaction.
    """
    with sim900_transaction_lock:
        number_response = send_command(
            "NUMBER:" + str(number).strip(),
            2,
            clear_input=True,
        )

        response = send_command(
            "CALL",
            SIM900_CALL_TIMEOUT_SECONDS,
        )

        full_response = number_response + response

    if "CALL_STARTED" in full_response:
        print("Jetson: Call started.", flush=True)
        return True

    if "NETWORK_FAIL" in full_response:
        print("Jetson: Call failed because SIM900A is not registered to the network.", flush=True)
    elif "SIGNAL_FAIL" in full_response:
        print("Jetson: Call failed because SIM900A signal is too weak.", flush=True)
    else:
        print("Jetson: Call may have failed.", flush=True)

    return False


def automatic_call_recipients(numbers):
    """Call saved recipients without interrupting SMS operations."""
    global call_is_sending

    if call_is_sending:
        print("Automatic call already running. Skipping duplicate call batch.", flush=True)
        return

    call_is_sending = True

    try:
        for number in numbers:
            if not program_running:
                break

            print("Automatic fire call target:", number, flush=True)

            # Keep the whole active call period protected so an SMS will wait
            # until the current call has been ended.
            with sim900_transaction_lock:
                make_call(number)
                time.sleep(AUTO_CALL_RING_SECONDS_PER_NUMBER)
                hang_up()

            time.sleep(1.0)

    finally:
        call_is_sending = False


def trigger_confirmed_fire_calls():
    global last_auto_call_time

    if not AUTO_CALL_BEFORE_RELAY:
        return

    now = time.time()

    if now - last_auto_call_time < AUTO_CALL_COOLDOWN_SECONDS:
        return

    numbers = get_saved_call_recipients()

    if not numbers:
        return

    last_auto_call_time = now

    threading.Thread(
        target=automatic_call_recipients,
        args=(numbers,),
        daemon=True,
    ).start()


def hang_up():
    with sim900_transaction_lock:
        send_command("HANG", 5, clear_input=True)


def get_relay_control_mode():
    with relay_control_mode_lock:
        return relay_control_mode


def set_relay_control_mode(mode):
    global relay_control_mode
    mode = str(mode).strip().upper()
    if mode not in ("MANUAL", "AUTO"):
        mode = "AUTO"
    with relay_control_mode_lock:
        relay_control_mode = mode
    print("Relay control mode:", mode, flush=True)


def send_relay_command_once(command, desired_state, force=False):
    """
    Send RELAY_ON / RELAY_OFF only when needed.

    The GUI loop and automatic state machine run many times per second. Without
    this guard, AUTO mode can spam RELAY_OFF repeatedly while no fire is active,
    which slows the app and floods the built-in serial monitor.

    force=True is used by the manual relay buttons and when switching back to
    automatic relay mode, so the Arduino is commanded even if the Jetson cache
    already thinks the relay is in that state.
    """
    global relay_is_on, last_relay_command_sent, last_relay_command_time

    now = time.time()
    if not force:
        if relay_is_on == desired_state and last_relay_command_sent == command:
            return ""
        if last_relay_command_sent == command and (now - last_relay_command_time) < RELAY_COMMAND_MIN_INTERVAL_SECONDS:
            return ""

    response = send_command(command,timeout=0,clear_input=True,wait_response=False)    
    relay_is_on = bool(desired_state)
    last_relay_command_sent = command
    last_relay_command_time = now
    return response


def relay_on(force=False):

    global relay_command_busy

    if relay_command_busy:
        return

    relay_command_busy = True

    try:
        return send_relay_command_once("RELAY_ON",True, force=force)
    finally:
        relay_command_busy = False


def relay_off(force=False):

    global relay_command_busy

    if relay_command_busy:
        return

    relay_command_busy = True

    try:
        return send_relay_command_once("RELAY_OFF",False, force=force)
    finally:
        relay_command_busy = False


def _send_confirmed_fire_sms_background(number, message):
    try:
        ok = send_sms(number, message)
        if ok:
            print("Automatic SMS completed successfully.", flush=True)
        else:
            print("Automatic SMS failed, but relay/pump logic will continue running.", flush=True)
    except Exception as error:
        print("Automatic SMS thread error: {}. Relay/pump logic will continue running.".format(error), flush=True)


def trigger_confirmed_fire_sms(max_temp, confidence):
    """Send the primary automatic fire SMS using the saved primary GUI fields."""
    global last_sms_time

    if not AUTO_PRIMARY_SMS_ENABLED:
        print("Primary automatic SMS disabled.", flush=True)
        return False

    now = time.time()
    if now - last_sms_time < SMS_COOLDOWN_SECONDS:
        return False

    last_sms_time = now

    number, base_message = get_saved_alert_settings()
    message = build_confirmed_fire_message(base_message, max_temp, confidence)

    print("Starting PRIMARY automatic SMS in background. Target:", number, flush=True)
    threading.Thread(
        target=_send_confirmed_fire_sms_background,
        args=(number, message),
        daemon=True,
    ).start()
    return True


def trigger_escalation_fire_sms(max_temp, confidence, elapsed_seconds):
    """Send backup/escalation SMS if fire remains after the fail timeout."""
    global last_escalation_sms_time

    if not AUTO_ESCALATION_SMS_ENABLED:
        print("Backup/escalation automatic SMS disabled.", flush=True)
        return False

    now = time.time()
    if now - last_escalation_sms_time < ESCALATION_SMS_COOLDOWN_SECONDS:
        return False

    last_escalation_sms_time = now

    number, base_message, fail_time = get_saved_escalation_settings()
    message = build_escalation_fire_message(base_message, max_temp, confidence, elapsed_seconds)

    print("Starting BACKUP/ESCALATION SMS in background. Target:", number, flush=True)
    threading.Thread(
        target=_send_confirmed_fire_sms_background,
        args=(number, message),
        daemon=True,
    ).start()
    return True


def update_fire_sms_escalation_state(active, max_temp, confidence):
    """
    Tracks one confirmed-fire response event for SMS purposes.

    active=True should be called only after the system has confirmed fire and
    reached the point where the relay/pump response is active or about to activate.
    The first call sends the primary SMS. If active stays true beyond the GUI
    fail-time limit, the backup/escalation SMS is sent once.
    """
    global fire_sms_event_active, fire_sms_event_start_time
    global primary_sms_sent_for_current_fire, escalation_sms_sent_for_current_fire

    now = time.time()

    if active:
        if not fire_sms_event_active:
            fire_sms_event_active = True
            fire_sms_event_start_time = now
            primary_sms_sent_for_current_fire = False
            escalation_sms_sent_for_current_fire = False
            print("SMS fire event started.", flush=True)

        if not primary_sms_sent_for_current_fire:
            if trigger_confirmed_fire_sms(max_temp, confidence):
                primary_sms_sent_for_current_fire = True

        elapsed = now - fire_sms_event_start_time
        _, _, fail_time = get_saved_escalation_settings()

        # Keep the existing optional fail-time feature available in the code,
        # but disable it in this build. Automatic escalation should occur only
        # when the system formally declares the fire UNCONTROLLABLE.
        if not ESCALATION_SMS_ONLY_WHEN_UNCONTROLLABLE:
            if elapsed >= fail_time and not escalation_sms_sent_for_current_fire:
                if trigger_escalation_fire_sms(max_temp, confidence, elapsed):
                    escalation_sms_sent_for_current_fire = True
                    print("Time-based escalation SMS condition reached after {:.1f}s.".format(elapsed), flush=True)
    else:
        if fire_sms_event_active:
            print("SMS fire event reset because confirmed fire is no longer active.", flush=True)
        fire_sms_event_active = False
        fire_sms_event_start_time = 0.0
        primary_sms_sent_for_current_fire = False
        escalation_sms_sent_for_current_fire = False

def publish_confirmed_fire_to_firebase(max_temp, confidence):
    global last_confirmed_firebase_time

    if not FIREBASE_UPLOAD_ENABLED:
        return

    if not CONFIRMED_FIRE_FIREBASE_ENABLED:
        return

    now = time.time()
    if now - last_confirmed_firebase_time < OBJECT_FIREBASE_COOLDOWN_SECONDS:
        return

    payload = {
        "timestamp": iso_timestamp(),
        "max_temperature": round(max_temp, 2),
        "alert_status": "CONFIRMED_FIRE",
        "source": "thermal_and_object_detection",
        "model_id": MODEL_ID,
        "fire_confidence": round(confidence, 4),
        "thermal_confirmed": True,
        "object_fire_detected": True,
        "relay_command": "RELAY_ON",
        "sms_triggered": AUTO_PRIMARY_SMS_ENABLED,
    }

    try:
        publish_to_firebase(payload, timeout=3.0)
        last_confirmed_firebase_time = now
    except Exception as error:
        print("Confirmed fire Firebase upload failed:", error, flush=True)



def save_fire_snapshot_if_needed():
    global fire_event_snapshot_saved, last_saved_fire_snapshot

    if not AUTO_AIM_SAVE_SNAPSHOT or fire_event_snapshot_saved:
        return

    with prediction_lock:
        frame = None if latest_fire_frame is None else latest_fire_frame.copy()

    if frame is None:
        return

    try:
        if not os.path.exists(AUTO_AIM_SNAPSHOT_DIR):
            os.makedirs(AUTO_AIM_SNAPSHOT_DIR)

        filename = "confirmed_fire_{}.jpg".format(
            datetime.now().strftime("%Y%m%d_%H%M%S")
        )
        path = os.path.join(AUTO_AIM_SNAPSHOT_DIR, filename)
        cv2.imwrite(path, frame)
        last_saved_fire_snapshot = path
        fire_event_snapshot_saved = True
        print("Saved confirmed fire snapshot:", path, flush=True)
    except Exception as error:
        print("Failed to save fire snapshot:", error, flush=True)

def wiggle_x_during_spray(duration_seconds):
    """
    Wiggle the X and Y axes while the relay is spraying.

    X pattern:
    - left
    - right past center
    - center

    Y pattern:
    - down
    - center
    - down
    - center
    - up
    - center
    - up
    - center

    The function tracks relative movement and returns both axes to the starting
    center position before it exits.
    """
    global wiggle_is_running, last_aim_status

    if not WIGGLE_DURING_SPRAY_ENABLED:
        return

    if wiggle_is_running:
        return

    wiggle_is_running = True

    # X direction choices based on your existing manual direction behavior.
    # RIGHT uses GPIO.LOW / False
    # LEFT uses GPIO.HIGH / True
    x_left_direction = True
    x_right_direction = False

    # Y direction choices based on the automatic localization direction mapping.
    y_down_direction = localization_y_dir(Y_AIM_DOWN_IS_HIGH_BASE)
    y_up_direction = localization_y_dir(not Y_AIM_DOWN_IS_HIGH_BASE)

    x_offset_from_original = 0
    y_offset_from_original = 0
    start_time = time.time()

    def should_continue():
        return (
            program_running
            and relay_is_on
            and ((time.time() - start_time) < duration_seconds)
        )

    def move_x(direction, steps, offset_delta_sign, status_text):
        nonlocal x_offset_from_original

        if not should_continue():
            return 0

        moved = pulse_motor_steps(DIR1_PIN, PUL1_PIN, direction, steps)
        x_offset_from_original += offset_delta_sign * moved
        last_aim_status = status_text
        time.sleep(WIGGLE_SETTLE_SECONDS)
        return moved

    def move_y(direction, steps, offset_delta_sign, status_text):
        nonlocal y_offset_from_original

        if not should_continue():
            return 0

        moved = pulse_y_limited(direction, steps)
        y_offset_from_original += offset_delta_sign * moved
        last_aim_status = status_text
        time.sleep(WIGGLE_SETTLE_SECONDS)
        return moved

    def return_x_to_center():
        nonlocal x_offset_from_original

        if x_offset_from_original > 0:
            moved = pulse_motor_steps(DIR1_PIN, PUL1_PIN, x_left_direction, abs(x_offset_from_original))
            x_offset_from_original -= moved
        elif x_offset_from_original < 0:
            moved = pulse_motor_steps(DIR1_PIN, PUL1_PIN, x_right_direction, abs(x_offset_from_original))
            x_offset_from_original += moved

    def return_y_to_center():
        nonlocal y_offset_from_original

        if y_offset_from_original > 0:
            moved = pulse_y_limited(y_up_direction, abs(y_offset_from_original))
            y_offset_from_original -= moved
        elif y_offset_from_original < 0:
            moved = pulse_y_limited(y_down_direction, abs(y_offset_from_original))
            y_offset_from_original += moved

    try:
        print("X/Y wiggle started during spray.", flush=True)

        while should_continue():
            # -----------------------------
            # LOWER COVERAGE: down + X sweep
            # -----------------------------

            # Y: down
            move_y(
                y_down_direction,
                WIGGLE_Y_STEPS,
                -1,
                "AUTO AIM: SPRAYING + Y WIGGLE DOWN",
            )

            # X: left -> right -> center while Y is down
            move_x(
                x_left_direction,
                WIGGLE_X_STEPS,
                -1,
                "AUTO AIM: SPRAYING + X LEFT / Y DOWN",
            )
            move_x(
                x_right_direction,
                WIGGLE_X_STEPS * 2,
                +1,
                "AUTO AIM: SPRAYING + X RIGHT / Y DOWN",
            )
            return_x_to_center()
            last_aim_status = "AUTO AIM: SPRAYING + X CENTER / Y DOWN"
            time.sleep(WIGGLE_SETTLE_SECONDS)

            # Y: center
            return_y_to_center()
            last_aim_status = "AUTO AIM: SPRAYING + Y CENTER"
            time.sleep(WIGGLE_SETTLE_SECONDS)

            if not should_continue():
                break

            # Y: down again
            move_y(
                y_down_direction,
                WIGGLE_Y_STEPS,
                -1,
                "AUTO AIM: SPRAYING + Y WIGGLE DOWN 2",
            )

            # X: left -> right -> center while Y is down again
            move_x(
                x_left_direction,
                WIGGLE_X_STEPS,
                -1,
                "AUTO AIM: SPRAYING + X LEFT / Y DOWN 2",
            )
            move_x(
                x_right_direction,
                WIGGLE_X_STEPS * 2,
                +1,
                "AUTO AIM: SPRAYING + X RIGHT / Y DOWN 2",
            )
            return_x_to_center()
            last_aim_status = "AUTO AIM: SPRAYING + X CENTER / Y DOWN 2"
            time.sleep(WIGGLE_SETTLE_SECONDS)

            # Y: center
            return_y_to_center()
            last_aim_status = "AUTO AIM: SPRAYING + Y CENTER"
            time.sleep(WIGGLE_SETTLE_SECONDS)

            if not should_continue():
                break

            # -----------------------------
            # UPPER COVERAGE: up + X sweep
            # -----------------------------

            # Y: up
            move_y(
                y_up_direction,
                WIGGLE_Y_STEPS,
                +1,
                "AUTO AIM: SPRAYING + Y WIGGLE UP",
            )

            # X: left -> right -> center while Y is up
            move_x(
                x_left_direction,
                WIGGLE_X_STEPS,
                -1,
                "AUTO AIM: SPRAYING + X LEFT / Y UP",
            )
            move_x(
                x_right_direction,
                WIGGLE_X_STEPS * 2,
                +1,
                "AUTO AIM: SPRAYING + X RIGHT / Y UP",
            )
            return_x_to_center()
            last_aim_status = "AUTO AIM: SPRAYING + X CENTER / Y UP"
            time.sleep(WIGGLE_SETTLE_SECONDS)

            # Y: center
            return_y_to_center()
            last_aim_status = "AUTO AIM: SPRAYING + Y CENTER"
            time.sleep(WIGGLE_SETTLE_SECONDS)

            if not should_continue():
                break

            # Y: up again
            move_y(
                y_up_direction,
                WIGGLE_Y_STEPS,
                +1,
                "AUTO AIM: SPRAYING + Y WIGGLE UP 2",
            )

            # X: left -> right -> center while Y is up again
            move_x(
                x_left_direction,
                WIGGLE_X_STEPS,
                -1,
                "AUTO AIM: SPRAYING + X LEFT / Y UP 2",
            )
            move_x(
                x_right_direction,
                WIGGLE_X_STEPS * 2,
                +1,
                "AUTO AIM: SPRAYING + X RIGHT / Y UP 2",
            )
            return_x_to_center()
            last_aim_status = "AUTO AIM: SPRAYING + X CENTER / Y UP 2"
            time.sleep(WIGGLE_SETTLE_SECONDS)

            # Y: center
            return_y_to_center()
            last_aim_status = "AUTO AIM: SPRAYING + Y CENTER"
            time.sleep(WIGGLE_SETTLE_SECONDS)

        # Always return both axes to the original center before exiting.
        return_x_to_center()
        return_y_to_center()

        print("X/Y wiggle finished and returned to original position.", flush=True)

    except Exception as error:
        print("X/Y wiggle error:", error, flush=True)

        # Best effort return to center after an error.
        try:
            return_x_to_center()
            return_y_to_center()
        except Exception:
            pass

    finally:
        wiggle_is_running = False




def start_spray_wiggle_if_needed(duration_seconds):
    """
    Start X/Y spray wiggle if enabled, relay is ON, and no wiggle is currently running.
    This is used for both normal spray and uncontrollable spray.
    """
    if not WIGGLE_DURING_SPRAY_ENABLED:
        return

    if wiggle_is_running:
        return

    if not relay_is_on:
        return

    threading.Thread(
        target=wiggle_x_during_spray,
        args=(duration_seconds,),
        daemon=True,
    ).start()


def save_position_state(force=False):
    """
    Save the last known software position of X and Y.

    Important:
    This only remembers software-estimated position.
    It is not the same as a real encoder or limit switch.
    """
    global last_position_save_time

    now = time.time()
    if not force and (now - last_position_save_time) < POSITION_SAVE_MIN_INTERVAL_SECONDS:
        return

    try:
        with x_position_lock:
            x_steps = int(current_x_position_steps)
        with y_position_lock:
            y_steps = int(current_y_position_steps)

        data = {
            "x_steps": x_steps,
            "y_steps": y_steps,
            "x_degrees": x_steps / max(1.0, X_STEPS_PER_DEGREE),
            "y_degrees": y_steps / max(1.0, Y_STEPS_PER_DEGREE),
            "saved_at": iso_timestamp(),
        }

        with open(POSITION_STATE_FILE, "w") as handle:
            json.dump(data, handle, indent=2)

        last_position_save_time = now

    except Exception as error:
        print("Position save failed:", error, flush=True)


def load_position_state():
    """
    Load the last known X/Y software position from the previous run.

    If the file does not exist, the program assumes X is at center and Y is at middle.
    """
    global current_x_position_steps, current_y_position_steps

    try:
        if not os.path.exists(POSITION_STATE_FILE):
            print("No saved position file. Using X center and Y middle reference.", flush=True)
            return

        with open(POSITION_STATE_FILE, "r") as handle:
            data = json.load(handle)

        loaded_x = int(data.get("x_steps", X_START_REFERENCE_STEPS))
        loaded_y = int(data.get("y_steps", Y_START_REFERENCE_STEPS))

        loaded_x = max(X_MIN_STEPS, min(X_MAX_STEPS, loaded_x))
        loaded_y = max(Y_MIN_STEPS, min(Y_MAX_STEPS, loaded_y))

        with x_position_lock:
            current_x_position_steps = loaded_x

        with y_position_lock:
            current_y_position_steps = loaded_y

        print(
            "Loaded saved position: X={:.1f} deg, Y={:.1f} deg".format(
                current_x_position_steps / max(1.0, X_STEPS_PER_DEGREE),
                current_y_position_steps / max(1.0, Y_STEPS_PER_DEGREE),
            ),
            flush=True,
        )

    except Exception as error:
        print("Position load failed. Using default software reference:", error, flush=True)


def pulse_x_limited(direction_high, requested_steps):
    """
    Move the X axis while respecting left/right software limits.

    This prevents endless manual rotation that can twist and tear wires.
    """
    global current_x_position_steps

    requested_steps = int(max(0, requested_steps))
    if requested_steps <= 0:
        return 0

    with x_position_lock:
        # For your current manual mapping:
        # RIGHT button uses GPIO.LOW / False.
        # LEFT button uses GPIO.HIGH / True.
        #
        # In this software reference:
        # Right movement increases X position.
        # Left movement decreases X position.
        sign = 1 if direction_high == False else -1

        requested_delta = sign * requested_steps
        target = current_x_position_steps + requested_delta
        clipped_target = max(X_MIN_STEPS, min(X_MAX_STEPS, target))
        allowed_delta = clipped_target - current_x_position_steps

        if allowed_delta == 0:
            return 0

        actual_direction_high = False if allowed_delta > 0 else True
        allowed_steps = abs(int(allowed_delta))

    moved = pulse_motor_steps(DIR1_PIN, PUL1_PIN, actual_direction_high, allowed_steps)

    with x_position_lock:
        if actual_direction_high == False:
            current_x_position_steps = min(X_MAX_STEPS, current_x_position_steps + moved)
        else:
            current_x_position_steps = max(X_MIN_STEPS, current_x_position_steps - moved)

    save_position_state(force=False)
    return moved


def reset_x_reference_to_center():
    """
    Use this only when the physical X axis is manually placed at the true center/front position.
    """
    global current_x_position_steps

    with x_position_lock:
        current_x_position_steps = X_START_REFERENCE_STEPS

    save_position_state(force=True)

    print(
        "X software reference reset to center: {} steps / {:.1f} deg".format(
            current_x_position_steps,
            current_x_position_steps / max(1.0, X_STEPS_PER_DEGREE),
        ),
        flush=True,
    )


def pulse_motor_steps(dir_pin, pul_pin, direction_high, steps):
    if steps <= 0:
        return 0

    actual_steps = int(steps)

    with motor_motion_lock:
        GPIO.output(dir_pin, GPIO.HIGH if direction_high else GPIO.LOW)

        moved = 0
        for _ in range(actual_steps):
            if not program_running:
                break
            GPIO.output(pul_pin, GPIO.HIGH)
            time.sleep(STEPPER_PULSE_DELAY_SECONDS)
            GPIO.output(pul_pin, GPIO.LOW)
            time.sleep(STEPPER_PULSE_DELAY_SECONDS)
            moved += 1

    return moved


def pulse_y_limited(direction_high, requested_steps):
    """
    Move the Y axis while respecting the 90-degree software limit.

    Y range is 0..8000 steps based on:
    3200 microsteps/rev * 10:1 gearbox * 90/360 = 8000 steps.

    Because there are no limit switches/homing sensors in this code, this is a
    relative software limit from Y_START_REFERENCE_STEPS. Before running tests,
    physically place the Y axis near the middle of its safe 90-degree travel.
    """
    global current_y_position_steps

    requested_steps = int(max(0, requested_steps))
    if requested_steps <= 0:
        return 0

    with y_position_lock:
        sign = 1 if direction_high == Y_INCREASES_WHEN_DIR_HIGH else -1
        requested_delta = sign * requested_steps
        target = current_y_position_steps + requested_delta
        clipped_target = max(Y_MIN_STEPS, min(Y_MAX_STEPS, target))
        allowed_delta = clipped_target - current_y_position_steps

        if allowed_delta == 0:
            return 0

        actual_direction_high = Y_INCREASES_WHEN_DIR_HIGH if allowed_delta > 0 else (not Y_INCREASES_WHEN_DIR_HIGH)
        allowed_steps = abs(int(allowed_delta))

    moved = pulse_motor_steps(DIR2_PIN, PUL2_PIN, actual_direction_high, allowed_steps)

    with y_position_lock:
        if actual_direction_high == Y_INCREASES_WHEN_DIR_HIGH:
            current_y_position_steps = min(Y_MAX_STEPS, current_y_position_steps + moved)
        else:
            current_y_position_steps = max(Y_MIN_STEPS, current_y_position_steps - moved)

    save_position_state(force=False)
    return moved


def reset_y_reference_to_middle():
    global current_y_position_steps

    with y_position_lock:
        current_y_position_steps = Y_START_REFERENCE_STEPS

    save_position_state(force=True)

    print(
        "Y software reference reset to middle: {} / {} steps".format(
            current_y_position_steps,
            Y_MAX_STEPS,
        ),
        flush=True,
    )


def clamp01(value):
    try:
        return max(0.0, min(1.0, float(value)))
    except Exception:
        return 0.5


def reset_aim_lock_state(reason=""):
    """Clear the frozen aim target after each spray cycle or when leaving AUTO mode."""
    global locked_aim_target_x, locked_aim_target_y, locked_aim_target_source, locked_aim_target_temp
    global aim_once_motion_done, aim_locked_on_fire, aimed_at_fire, last_aim_status, auto_response_phase

    locked_aim_target_x = None
    locked_aim_target_y = None
    locked_aim_target_source = ""
    locked_aim_target_temp = 0.0
    aim_once_motion_done = False
    aim_locked_on_fire = False
    aimed_at_fire = False
    auto_response_phase = "IDLE"
    if reason:
        last_aim_status = "AUTO AIM: RESET - " + str(reason)[:40]

def get_current_aim_point():
    """
    Return the current target point used for final aiming.

    New mechanics:
    - Thermal is used to stop scanning and pre-center the fire area first.
    - After the webcam/cloud model confirms fire, final aiming uses the recorded
      webcam fire bounding box center, not the thermal hotspot.
    - last_seen_fire_bbox_* is used as a fallback because cloud inference can be delayed.
    """
    with prediction_lock:
        cx = latest_fire_center_x
        cy = latest_fire_center_y
        confidence = latest_object_confidence

    if cx is not None and cy is not None:
        return True, clamp01(cx), clamp01(cy), confidence, "WEBCAM_BBOX"

    # Fallback to the most recently seen fire bbox if it is still fresh.
    # This avoids losing the target because a delayed/noisy cloud frame arrived.
    if last_seen_fire_bbox_x is not None and last_seen_fire_bbox_y is not None:
        age = time.time() - last_seen_fire_bbox_time
        if age <= OBJECT_FIRE_HOLD_SECONDS:
            return (
                True,
                clamp01(last_seen_fire_bbox_x),
                clamp01(last_seen_fire_bbox_y),
                last_seen_fire_bbox_confidence,
                "WEBCAM_BBOX_LAST",
            )

    return False, 0.5, 0.5, 0.0, "WEBCAM_BBOX"

def capture_locked_aim_target_if_needed():
    """
    Capture the webcam fire bounding box once after confirmation.

    This is the anti-lag fix: after the bbox is captured, the code stops following
    later cloud/camera changes that can arrive late and cause overshoot.
    """
    global locked_aim_target_x, locked_aim_target_y, locked_aim_target_source, locked_aim_target_temp
    global last_aim_status, auto_response_phase

    if locked_aim_target_x is not None and locked_aim_target_y is not None:
        return True

    ok, cx, cy, confidence, source = get_current_aim_point()
    if not ok:
        last_aim_status = "AUTO AIM: WAITING FOR FIRE BBOX"
        return False

    locked_aim_target_x = cx
    locked_aim_target_y = cy
    locked_aim_target_source = source
    locked_aim_target_temp = confidence
    auto_response_phase = "BBOX_TARGET_LOCKED"
    print(
        "Locked fire bounding box from {} at x={:.2f}, y={:.2f}; target x={:.2f}, y={:.2f}".format(
            source,
            locked_aim_target_x,
            locked_aim_target_y,
            BBOX_AIM_TARGET_X_NORM,
            BBOX_AIM_TARGET_Y_NORM,
        ),
        flush=True,
    )
    return True

def auto_aim_step_to_fire():
    """
    Anti-overshoot automatic aim using the locked webcam fire bounding box.

    AUTO mode sequence:
    1. Thermal detects HIGH/CRITICAL heat.
    2. Thermal hotspot is pre-centered using the fast AMG8833 reading.
    3. Webcam/cloud confirms fire.
    4. The current webcam fire bounding box center is frozen one time.
    5. The system moves the locked bbox:
       - X to center: 0.50
       - Y to 1/5 from bottom to top: 0.80
    6. The aim locks and ignores later camera/inference changes until relay OFF.
    """
    global last_aim_status, aimed_at_fire, aim_locked_on_fire, aim_once_motion_done, auto_response_phase

    if get_operation_mode() != "AUTO":
        last_aim_status = "AUTO AIM: DISABLED - MANUAL MODE"
        return False

    if not AUTO_AIM_ENABLED:
        aimed_at_fire = True
        aim_locked_on_fire = True
        last_aim_status = "AUTO AIM: DISABLED"
        return True

    if LOCK_AIM_AFTER_CENTERED and aim_locked_on_fire:
        aimed_at_fire = True
        last_aim_status = "AUTO AIM: LOCKED / SPRAYING"
        return True

    if not capture_locked_aim_target_if_needed():
        aimed_at_fire = False
        return False

    # Use the frozen webcam bbox target, not the latest camera/inference value.
    cx = locked_aim_target_x
    cy = locked_aim_target_y
    aim_source = locked_aim_target_source or "LOCKED_BBOX"

    target_x = clamp01(BBOX_AIM_TARGET_X_NORM)
    target_y = clamp01(BBOX_AIM_TARGET_Y_NORM)

    error_x = float(cx) - target_x
    error_y = float(cy) - target_y

    x_aligned = abs(error_x) <= FIRE_CENTER_DEADZONE_X
    y_aligned = abs(error_y) <= FIRE_CENTER_DEADZONE_Y

    if x_aligned and y_aligned:
        aimed_at_fire = True
        aim_locked_on_fire = True
        aim_once_motion_done = True
        auto_response_phase = "AIM_LOCKED"
        last_aim_status = "AUTO AIM: {} LOCKED X=0.50 Y=0.80".format(aim_source)
        return True

    # One-shot mode: do one calculated correction, then lock immediately.
    # This prevents delayed cloud inference from causing repeated chase/overshoot.
    if ONE_SHOT_AIM_AFTER_CONFIRMATION:
        if aim_once_motion_done:
            aimed_at_fire = True
            aim_locked_on_fire = True
            auto_response_phase = "AIM_LOCKED"
            last_aim_status = "AUTO AIM: ONE-SHOT LOCKED / WAITING SPRAY"
            return True

        x_steps = 0
        y_steps = 0

        if not x_aligned:
            x_steps = int(min(
                ONE_SHOT_AIM_MAX_STEPS_X,
                max(AUTO_AIM_MIN_STEPS, abs(error_x) * ONE_SHOT_AIM_GAIN_X),
            ))
            x_direction_high = localization_x_dir(X_AIM_RIGHT_IS_HIGH_BASE if error_x > 0 else not X_AIM_RIGHT_IS_HIGH_BASE)
            pulse_motor_steps(DIR1_PIN, PUL1_PIN, x_direction_high, x_steps)

        if not y_aligned:
            y_steps = int(min(
                ONE_SHOT_AIM_MAX_STEPS_Y,
                max(AUTO_AIM_MIN_STEPS, abs(error_y) * ONE_SHOT_AIM_GAIN_Y),
            ))
            y_direction_high = localization_y_dir(Y_AIM_DOWN_IS_HIGH_BASE if error_y > 0 else not Y_AIM_DOWN_IS_HIGH_BASE)
            y_steps = pulse_y_limited(y_direction_high, y_steps)

        aim_once_motion_done = True
        aimed_at_fire = True
        aim_locked_on_fire = True
        auto_response_phase = "AIM_LOCKED"
        last_aim_status = "AUTO AIM: ONE-SHOT {} X={} Y={} ERR=({:+.2f},{:+.2f})".format(
            aim_source,
            x_steps,
            y_steps,
            error_x,
            error_y,
        )
        print(last_aim_status, flush=True)
        return True

    # Fallback feedback mode, still using the frozen bbox point.
    aimed_at_fire = False
    x_steps = 0
    y_steps = 0

    if not x_aligned:
        x_steps = int(min(AUTO_AIM_MAX_STEPS_PER_UPDATE_X, max(AUTO_AIM_MIN_STEPS, abs(error_x) * AUTO_AIM_STEP_GAIN_X)))
        x_direction_high = localization_x_dir(X_AIM_RIGHT_IS_HIGH_BASE if error_x > 0 else not X_AIM_RIGHT_IS_HIGH_BASE)
        pulse_motor_steps(DIR1_PIN, PUL1_PIN, x_direction_high, x_steps)

    if not y_aligned:
        y_steps = int(min(AUTO_AIM_MAX_STEPS_PER_UPDATE_Y, max(AUTO_AIM_MIN_STEPS, abs(error_y) * AUTO_AIM_STEP_GAIN_Y)))
        y_direction_high = localization_y_dir(Y_AIM_DOWN_IS_HIGH_BASE if error_y > 0 else not Y_AIM_DOWN_IS_HIGH_BASE)
        y_steps = pulse_y_limited(y_direction_high, y_steps)

    last_aim_status = "AUTO AIM: {} MOVING X={} Y={} ERR=({:+.2f},{:+.2f})".format(
        aim_source,
        x_steps,
        y_steps,
        error_x,
        error_y,
    )
    time.sleep(AUTO_AIM_SETTLE_SECONDS)
    return False


def reset_confirmation_cycle_for_next_attempt(reason=""):
    """
    Reset only the heat/camera confirmation and aim lock so AUTO can repeat:
    thermal center -> camera confirm -> bbox aim -> spray.
    This does not reset the overall spray-cycle counter.
    """
    global fire_confirmation_start_time, heat_confirmation_object_seen, fire_confirmation_status
    global thermal_precenter_done_for_current_heat, thermal_precenter_motion_count

    fire_confirmation_start_time = 0.0
    heat_confirmation_object_seen = False
    thermal_precenter_done_for_current_heat = False
    thermal_precenter_motion_count = 0
    reset_aim_lock_state(reason or "NEXT ATTEMPT")
    fire_confirmation_status = "WAITING FOR HEAT"


def reset_fire_response_cycle(reason=""):
    """Reset all retry/uncontrollable state after fire is gone or mode changes."""
    global controlled_spray_cycle_count, fire_uncontrollable
    global uncontrollable_sms_sent, uncontrollable_sms_sending
    global last_uncontrollable_sms_attempt_time
    global confirmed_fire_active, spray_started_time, fire_event_snapshot_saved

    controlled_spray_cycle_count = 0
    fire_uncontrollable = False
    uncontrollable_sms_sent = False
    uncontrollable_sms_sending = False
    last_uncontrollable_sms_attempt_time = 0.0
    confirmed_fire_active = False
    spray_started_time = 0.0
    fire_event_snapshot_saved = False
    reset_confirmation_cycle_for_next_attempt(reason or "FIRE OUT")


def _send_uncontrollable_fire_sms_background(number, message):
    """
    Attempt the uncontrollable-fire SMS.
    Only mark it as sent after the Arduino/SIM900A reports success.
    """
    global uncontrollable_sms_sent, uncontrollable_sms_sending

    try:
        ok = send_sms(number, message)

        if ok:
            uncontrollable_sms_sent = True
            print("UNCONTROLLABLE FIRE emergency SMS sent successfully to:", number, flush=True)
        else:
            print(
                "UNCONTROLLABLE FIRE emergency SMS failed. "
                "The system will retry while the fire remains uncontrollable.",
                flush=True,
            )
    except Exception as error:
        print(
            "UNCONTROLLABLE FIRE emergency SMS error: {}. "
            "The system will retry.".format(error),
            flush=True,
        )
    finally:
        uncontrollable_sms_sending = False


def trigger_uncontrollable_fire_alert(max_temp, confidence, elapsed_seconds=0.0):
    """
    Send the emergency/secondary SMS when repeated spray cycles fail.
    Retry periodically until the SIM900A confirms that the SMS was sent.
    Uses the backup/escalation number and message saved in the GUI.
    """
    global uncontrollable_sms_sent, uncontrollable_sms_sending
    global last_uncontrollable_sms_attempt_time

    if uncontrollable_sms_sent:
        return False

    if uncontrollable_sms_sending:
        return False

    now = time.time()

    if (now - last_uncontrollable_sms_attempt_time) < UNCONTROLLABLE_SMS_RETRY_SECONDS:
        return False

    number, base_message, fail_time = get_saved_escalation_settings()

    message = build_escalation_fire_message(
        base_message,
        max_temp,
        confidence,
        elapsed_seconds,
    )
    message = "UNCONTROLLABLE FIRE ALERT. " + message

    last_uncontrollable_sms_attempt_time = now
    uncontrollable_sms_sending = True

    print(
        "Fire considered UNCONTROLLABLE after {} completed spray cycles. Attempting emergency SMS to: {}".format(
            controlled_spray_cycle_count,
            number,
        ),
        flush=True,
    )

    threading.Thread(
        target=_send_uncontrollable_fire_sms_background,
        args=(number, message[:300]),
        daemon=True,
    ).start()

    return True

def update_auto_relay_state():
    """
    Main automatic fire response.

    AUTO-only sequence:
    1. Thermal detects HIGH/CRITICAL heat.
    2. Thermal hotspot is centered first using AMG8833.
    3. Auto scan remains stopped.
    4. Webcam/cloud detection confirms if it is fire.
    5. The webcam fire bounding box location is recorded/frozen.
    6. The locked bbox X axis is moved to center.
    7. The locked bbox Y axis is moved to 1/5 from bottom to top (Y=0.80).
    8. Relay/pump turns ON.
    9. After 5 seconds, relay turns OFF and the locked bbox is removed.
    10. If fire still exists, repeat from thermal centering. After the 4th needed response,
        mark fire as uncontrollable.
    11. Primary SMS/call warns of confirmed fire. Uncontrollable fire sends backup/emergency SMS.
    12. If fire is put out, return to normal AUTO scan. If uncontrollable, keep extinguishing.
    """
    global confirmed_fire_active, last_confirmed_fire_time, spray_started_time
    global fire_event_snapshot_saved, aimed_at_fire, last_aim_status, aim_locked_on_fire
    global controlled_spray_cycle_count, fire_uncontrollable, uncontrollable_sms_sent

    now = time.time()

    # Hard safety/control guard: automatic scan, aim, relay, and SMS response must not run in MANUAL mode.
    if get_operation_mode() != "AUTO":
        if confirmed_fire_active or aim_locked_on_fire or aimed_at_fire or fire_uncontrollable:
            reset_fire_response_cycle("MANUAL MODE")
        update_fire_sms_escalation_state(False, max_temp=latest_thermal_max_temp, confidence=latest_object_confidence)
        last_aim_status = "AUTO AIM: OFF - MANUAL MODE"
        return

    with thermal_lock:
        max_temp = latest_thermal_max_temp

    with prediction_lock:
        confidence = latest_object_confidence

    # If the system is already marked uncontrollable, keep extinguishing while fire evidence exists.
    # Once fire evidence disappears, turn off and return to normal AUTO scan.
    if fire_uncontrollable:
        fire_still_seen = recent_thermal_fire() or recent_object_fire()
        if fire_still_seen:

            if get_relay_control_mode() == "AUTO":
                relay_on()
                start_spray_wiggle_if_needed(FIXED_SPRAY_SECONDS)

            update_fire_sms_escalation_state(True, max_temp=max_temp, confidence=confidence)
            elapsed = now - fire_sms_event_start_time if fire_sms_event_start_time else 0.0
            trigger_uncontrollable_fire_alert(max_temp=max_temp, confidence=confidence, elapsed_seconds=elapsed)
            last_aim_status = "AUTO AIM: UNCONTROLLABLE - CONTINUING SPRAY"
            return

        print("Uncontrollable state cleared because fire evidence is gone.", flush=True)
        threading.Thread(target=relay_off,daemon=True).start()
        update_fire_sms_escalation_state(False, max_temp=max_temp, confidence=confidence)
        reset_fire_response_cycle("FIRE OUT")
        return

    confirmed_now = stable_confirmed_fire_now()

    if confirmed_now:
        last_confirmed_fire_time = now
        save_fire_snapshot_if_needed()

        if not confirmed_fire_active:
            # If we already used the allowed controlled cycles and fire is still confirmed,
            # the next needed response is the 4th attempt, so mark it uncontrollable.
            if controlled_spray_cycle_count >= MAX_CONTROLLED_SPRAY_CYCLES:
                fire_uncontrollable = True
                confirmed_fire_active = True
                print(
                    "Fire still exists after {} controlled spray cycles. Marking UNCONTROLLABLE.".format(
                        MAX_CONTROLLED_SPRAY_CYCLES
                    ),
                    flush=True,
                )

                if get_relay_control_mode() == "AUTO":
                    relay_on()
                    start_spray_wiggle_if_needed(FIXED_SPRAY_SECONDS)

                update_fire_sms_escalation_state(True, max_temp=max_temp, confidence=confidence)

                elapsed = now - fire_sms_event_start_time if fire_sms_event_start_time else 0.0

                # Give priority to the emergency SMS when the fire becomes uncontrollable.
                # Do not start another call here because the SIM900A is needed for the SMS.
                trigger_uncontrollable_fire_alert(
                    max_temp=max_temp,
                    confidence=confidence,
                    elapsed_seconds=elapsed,
                )

                last_aim_status = "AUTO AIM: UNCONTROLLABLE - CONTINUING SPRAY"
                return

            reset_aim_lock_state("NEW CONFIRMED FIRE")
            print(
                "CONFIRMED FIRE: thermal centered, webcam confirmed fire. Lock bbox, aim to X=0.50/Y=0.80, then relay ON.",
                flush=True,
            )

        confirmed_fire_active = True

        if LOCK_AIM_AFTER_CENTERED and aim_locked_on_fire:
            aimed = True
            aimed_at_fire = True
            last_aim_status = "AUTO AIM: LOCKED / SPRAYING"
        else:
            aimed = auto_aim_step_to_fire()
            if aimed and LOCK_AIM_AFTER_CENTERED:
                aim_locked_on_fire = True
                last_aim_status = "AUTO AIM: LOCKED / SPRAYING"

        publish_confirmed_fire_to_firebase(max_temp=max_temp, confidence=confidence)

        if get_relay_control_mode() == "AUTO":
            if (not AUTO_AIM_BEFORE_RELAY) or aimed:
                was_relay_on = relay_is_on

                relay_on()

                update_fire_sms_escalation_state(True, max_temp=max_temp, confidence=confidence)

                if not was_relay_on:
                    controlled_spray_cycle_count += 1
                    spray_started_time = time.time()
                    print(
                        "Relay/pump ON for fixed {:.1f}s. Controlled spray cycle {}/{}.".format(
                            FIXED_SPRAY_SECONDS,
                            controlled_spray_cycle_count,
                            MAX_CONTROLLED_SPRAY_CYCLES,
                        ),
                        flush=True,
                    )

                    # Start X-axis wiggle only after relay_is_on has already been set to True.
                    start_spray_wiggle_if_needed(FIXED_SPRAY_SECONDS)
                    


                    update_fire_sms_escalation_state(True, max_temp=max_temp, confidence=confidence)
                    trigger_confirmed_fire_calls()

                if spray_started_time > 0.0 and (now - spray_started_time) >= FIXED_SPRAY_SECONDS:
                    if wiggle_is_running:
                        last_aim_status = "AUTO AIM: WAITING FOR X WIGGLE RETURN"
                        return

                    print("Fixed spray time finished. Relay OFF. Re-checking fire.", flush=True)

                    threading.Thread(target=relay_off, daemon=True).start()

                    confirmed_fire_active = False
                    fire_event_snapshot_saved = False
                    spray_started_time = 0.0

                    # Remove locked bbox and force the next loop to start again at thermal centering.
                    reset_confirmation_cycle_for_next_attempt("SPRAY COMPLETE")
                    last_aim_status = "AUTO AIM: SPRAY COMPLETE - RECHECK FIRE"
            else:
                threading.Thread(target=relay_off,daemon=True).start()
        else:
            last_aim_status = "AUTO AIM: READY - RELAY MODE MANUAL"

    else:
        # Finish the 5-second spray even if a detection update briefly disappears.
        if confirmed_fire_active and relay_is_on and spray_started_time > 0.0:

            if (now - spray_started_time) >= FIXED_SPRAY_SECONDS:
                if wiggle_is_running:
                    last_aim_status = "AUTO AIM: WAITING FOR X WIGGLE RETURN"
                    return

                print("Fixed spray time finished. Relay OFF. Re-checking fire.", flush=True)

                threading.Thread(target=relay_off, daemon=True).start()
                confirmed_fire_active = False
                fire_event_snapshot_saved = False
                spray_started_time = 0.0

                reset_confirmation_cycle_for_next_attempt("SPRAY COMPLETE")
                last_aim_status = "AUTO AIM: SPRAY COMPLETE - RECHECK FIRE"
            else:
                remaining = FIXED_SPRAY_SECONDS - (now - spray_started_time)
                last_aim_status = "AUTO AIM: SPRAYING {:.1f}s LEFT".format(remaining)
            return

        # No confirmed fire and no active spray. If there is no recent fire evidence,
        # reset the whole response and return to normal AUTO scanning.
        if not recent_thermal_fire() and not recent_object_fire():
            if controlled_spray_cycle_count > 0:
                print("Fire appears out. Returning to normal AUTO operation.", flush=True)
            threading.Thread(target=relay_off,daemon=True).start()
            update_fire_sms_escalation_state(False, max_temp=max_temp, confidence=confidence)
            reset_fire_response_cycle("FIRE OUT")
        else:
            # Heat/object evidence exists but confirmation is not complete yet.
            # Keep relay OFF until the confirmation + aiming stages succeed.
            if get_relay_control_mode() == "AUTO" and not confirmed_fire_active:
                threading.Thread(target=relay_off,daemon=True).start()

def stepper1_loop():
    global running1, program_running

    while program_running:
        if running1 and get_operation_mode() == "MANUAL":
            direction_high = GPIO.input(DIR1_PIN) == GPIO.HIGH
            moved = pulse_x_limited(direction_high, 1)

            if moved <= 0:
                running1 = False
                time.sleep(0.02)
        else:
            time.sleep(0.001)


def stepper2_loop():
    global running2, program_running

    while program_running:
        if running2 and get_operation_mode() == "MANUAL":
            # Manual Y movement also respects the 0..8000 software range.
            direction_high = GPIO.input(DIR2_PIN) == GPIO.HIGH
            moved = pulse_y_limited(direction_high, 1)
            if moved <= 0:
                running2 = False
                time.sleep(0.02)
        else:
            time.sleep(0.001)


def start_right(event=None):
    global running1
    if get_operation_mode() != "MANUAL":
        return
    # Inverted manual control: RIGHT button now uses the previous LEFT direction.
    GPIO.output(DIR1_PIN, GPIO.LOW)
    running1 = True


def start_left(event=None):
    global running1
    if get_operation_mode() != "MANUAL":
        return
    # Inverted manual control: LEFT button now uses the previous RIGHT direction.
    GPIO.output(DIR1_PIN, GPIO.HIGH)
    running1 = True


def stop_motor1(event=None):
    global running1
    running1 = False
    GPIO.output(PUL1_PIN, GPIO.LOW)


def start_up(event=None):
    global running2
    if get_operation_mode() != "MANUAL":
        return
    # Inverted manual control: UP button now uses the previous DOWN direction.
    GPIO.output(DIR2_PIN, GPIO.LOW)
    running2 = True


def start_down(event=None):
    global running2
    if get_operation_mode() != "MANUAL":
        return
    # Inverted manual control: DOWN button now uses the previous UP direction.
    GPIO.output(DIR2_PIN, GPIO.HIGH)
    running2 = True


def stop_motor2(event=None):
    global running2
    running2 = False
    GPIO.output(PUL2_PIN, GPIO.LOW)


# =====================================================
# AUTOMATIC X-AXIS SCAN FUNCTIONS
# =====================================================

def auto_scan_loop():
    """
    AUTO mode heat-first scanning behavior.

    - Moves Motor 1 / X-axis only while searching.
    - Does not move Motor 2 / Y-axis during search.
    - Stops when AMG8833 detects heat.
    - Holds still for 3 seconds while the webcam checks for visible fire.
    - If no visible fire appears, ignores that heat source for 3 seconds and scans on.
    - Relay/SMS still require confirmed heat + camera fire.
    """
    global auto_scan_direction_high, auto_scan_steps_in_sweep, auto_scan_status

    while program_running:
        if get_operation_mode() != "AUTO" or not AUTO_SCAN_ENABLED:
            auto_scan_status = "AUTO SCAN: OFF"
            time.sleep(0.05)
            continue

        now = time.time()

        if confirmed_fire_active or relay_is_on:
            auto_scan_status = "AUTO SCAN: STOPPED - RELAY/PUMP ON"
            try:
                GPIO.output(PUL1_PIN, GPIO.LOW)
                GPIO.output(PUL2_PIN, GPIO.LOW)
            except Exception:
                pass
            time.sleep(0.05)
            continue

        # Heat-first trigger: stop the X scan while the 3-second visual check runs.
        # During the ignore cooldown, continue scanning so the camera moves away
        # from the non-fire heat source.
        if now >= heat_ignore_until_time and recent_thermal_fire():
            stable_confirmed_fire_now()
            auto_scan_status = "AUTO SCAN: STOPPED - " + fire_confirmation_status
            time.sleep(0.05)
            continue

        direction_text = "CW/HIGH" if auto_scan_direction_high else "CCW/LOW"
        if now < heat_ignore_until_time:
            auto_scan_status = "AUTO SCAN: IGNORE HEAT / X " + direction_text + " | " + last_smoke_y_scan_status
        else:
            auto_scan_status = "AUTO SCAN: SEARCHING HEAT / X " + direction_text + " | " + last_smoke_y_scan_status

        moved = pulse_x_limited(
            auto_scan_direction_high,
            AUTO_SCAN_CHUNK_STEPS,
        )

        auto_scan_steps_in_sweep += int(moved)

        # If X hits the software limit, reverse direction immediately.
        # This prevents wire twisting during AUTO scan.
        if moved <= 0:
            auto_scan_steps_in_sweep = 0

            smoke_y_step_at_x_reversal()

            auto_scan_direction_high = not auto_scan_direction_high

        elif AUTO_SCAN_BIDIRECTIONAL and auto_scan_steps_in_sweep >= AUTO_SCAN_STEPS_PER_SWEEP:
            auto_scan_steps_in_sweep = 0

            smoke_y_step_at_x_reversal()

            auto_scan_direction_high = not auto_scan_direction_high

        time.sleep(AUTO_SCAN_SETTLE_SECONDS)


# =====================================================
# ROBOFLOW OBJECT DETECTION FUNCTIONS
# =====================================================

def build_model_id() -> str:
    project = (ROBOFLOW_MODEL_PROJECT or "").strip().strip("/")
    version = str(ROBOFLOW_MODEL_VERSION or "").strip().strip("/")
    return f"{project}/{version}"


MODEL_ID = build_model_id()


def build_csi_pipeline(width: int, height: int, fps: int) -> str:
    return (
        "nvarguscamerasrc ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, "
        f"format=NV12, framerate={fps}/1 ! "
        "nvvidconv flip-method=0 ! "
        f"video/x-raw, width={width}, height={height}, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


def open_camera():
    if USE_CSI_CAMERA:
        capture = cv2.VideoCapture(
            build_csi_pipeline(CAPTURE_WIDTH, CAPTURE_HEIGHT, CAMERA_FPS),
            cv2.CAP_GSTREAMER,
        )
    else:
        capture = cv2.VideoCapture(CAMERA_INDEX)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
        capture.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

    if not capture.isOpened():
        raise RuntimeError(
            "Unable to open the camera. Check CAMERA_INDEX or set USE_CSI_CAMERA=True for a CSI camera."
        )

    return capture


def encode_frame(frame: Any, jpeg_quality: int) -> str:
    success, buffer = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)],
    )
    if not success:
        raise RuntimeError("Failed to encode frame for inference.")

    return base64.b64encode(buffer).decode("ascii")


def infer_frame_hosted(
    image_base64: str,
    confidence: float,
    iou_threshold: float,
) -> Dict[str, Any]:
    if not ROBOFLOW_API_KEY:
        raise RuntimeError("Roboflow API key is empty.")

    base_url = (ROBOFLOW_HOSTED_URL or "https://detect.roboflow.com").strip().rstrip("/")
    hosted_url = f"{base_url}/{MODEL_ID}"

    response = http_session.post(
        hosted_url,
        params={
            "api_key": ROBOFLOW_API_KEY,
            "confidence": max(0, min(100, round(confidence * 100))),
            "overlap": max(0, min(100, round(iou_threshold * 100))),
        },
        data=image_base64,
        headers={
            "Content-Type": "application/x-www-form-urlencoded",
            "Accept": "application/json",
        },
        timeout=(4, 20),
    )

    if response.status_code == 401:
        raise RuntimeError("401 Unauthorized from Roboflow. Check API key.")
    if response.status_code == 403:
        raise RuntimeError("403 Forbidden from Roboflow. Check model slug/version and API key.")

    response.raise_for_status()
    data = response.json()

    if not isinstance(data, dict):
        raise RuntimeError("Roboflow returned an invalid response.")

    return data


def get_prediction_bbox(prediction: Dict[str, Any]) -> Dict[str, float]:
    bbox = prediction.get("bbox")
    if isinstance(bbox, dict):
        return bbox

    if all(key in prediction for key in ("x", "y", "width", "height")):
        return {
            "x": float(prediction.get("x", 0) or 0),
            "y": float(prediction.get("y", 0) or 0),
            "width": float(prediction.get("width", 0) or 0),
            "height": float(prediction.get("height", 0) or 0),
        }

    return {}


def iter_predictions(predictions: Iterable[Dict[str, Any]]) -> Iterable[Dict[str, Any]]:
    for prediction in predictions:
        if isinstance(prediction, dict) and get_prediction_bbox(prediction):
            yield prediction


def summarize_predictions(predictions: Iterable[Dict[str, Any]]) -> Tuple[bool, List[str], float]:
    labels: List[str] = []
    max_confidence = 0.0
    fire_detected = False

    for prediction in iter_predictions(predictions):
        label = str(prediction.get("class") or "").strip()
        if label:
            labels.append(label)

        confidence = float(prediction.get("confidence", 0) or 0)
        max_confidence = max(max_confidence, confidence)

        if label.lower() == "fire":
            fire_detected = True

    return fire_detected, sorted(set(labels)), max_confidence



def get_best_fire_target(predictions: Iterable[Dict[str, Any]]) -> Tuple[bool, float, float, float, Dict[str, float]]:
    """
    Returns:
        fire_found, center_x_norm, center_y_norm, confidence, bbox

    center_x_norm and center_y_norm are 0.0 to 1.0 relative to the inference frame.
    """
    best_prediction = None
    best_confidence = -1.0

    for prediction in iter_predictions(predictions):
        label = str(prediction.get("class") or "").strip().lower()
        if label != "fire":
            continue

        confidence = float(prediction.get("confidence", 0) or 0)
        if confidence > best_confidence:
            best_confidence = confidence
            best_prediction = prediction

    if best_prediction is None:
        return False, 0.5, 0.5, 0.0, {}

    bbox = get_prediction_bbox(best_prediction)
    center_x = float(bbox.get("x", INFERENCE_WIDTH / 2.0))
    center_y = float(bbox.get("y", INFERENCE_HEIGHT / 2.0))

    center_x_norm = max(0.0, min(1.0, center_x / max(1.0, float(INFERENCE_WIDTH))))
    center_y_norm = max(0.0, min(1.0, center_y / max(1.0, float(INFERENCE_HEIGHT))))

    return True, center_x_norm, center_y_norm, best_confidence, bbox


def draw_predictions(frame: Any, predictions: List[Dict[str, Any]], inference_size: Tuple[int, int]) -> None:
    frame_height, frame_width = frame.shape[:2]
    inference_width, inference_height = inference_size

    scale_x = frame_width / max(1, inference_width)
    scale_y = frame_height / max(1, inference_height)

    font = cv2.FONT_HERSHEY_SIMPLEX

    for prediction in iter_predictions(predictions):
        bbox = get_prediction_bbox(prediction)

        x = float(bbox.get("x", 0))
        y = float(bbox.get("y", 0))
        width = float(bbox.get("width", 0))
        height = float(bbox.get("height", 0))

        label = str(prediction.get("class") or "Object")
        confidence = float(prediction.get("confidence", 0) or 0)

        left = int((x - width / 2) * scale_x)
        top = int((y - height / 2) * scale_y)
        right = int((x + width / 2) * scale_x)
        bottom = int((y + height / 2) * scale_y)

        is_fire = label.lower() == "fire"
        color = (0, 200, 0) if is_fire else (48, 48, 209)
        thickness = 3 if is_fire else 2

        cv2.rectangle(frame, (left, top), (right, bottom), color, thickness)

        label_text = f"{label} {confidence * 100:.0f}%"
        font_scale = 0.8 if is_fire else 0.55
        font_thickness = 2 if is_fire else 1

        (text_width, text_height), _ = cv2.getTextSize(
            label_text,
            font,
            font_scale,
            font_thickness,
        )
        text_top = max(0, top - text_height - 10)

        cv2.rectangle(
            frame,
            (left, text_top),
            (left + text_width + 10, text_top + text_height + 10),
            color,
            -1,
        )
        cv2.putText(
            frame,
            label_text,
            (left + 5, text_top + text_height + 2),
            font,
            font_scale,
            (255, 255, 255),
            font_thickness,
            cv2.LINE_AA,
        )


def draw_object_overlay(
    frame: Any,
    fps: float,
    detections_count: int,
    latency_ms: float,
    last_error: str,
    fire_detected: bool,
) -> None:
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (345, 142), (17, 24, 39), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

    cv2.putText(frame, f"Object Detection | FPS: {fps:.0f}", (20, 36),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, f"Detections: {detections_count}", (20, 64),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 226, 232), 1, cv2.LINE_AA)
    cv2.putText(frame, f"Latency: {latency_ms:.0f} ms", (20, 88),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 226, 232), 1, cv2.LINE_AA)
    cv2.putText(frame, MODEL_ID, (20, 112),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 169, 181), 1, cv2.LINE_AA)
    cv2.putText(frame, f"Firebase: {'ON' if FIREBASE_ENDPOINT else 'OFF'}", (20, 132),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 169, 181), 1, cv2.LINE_AA)

    if fire_detected and int(time.time() * 2) % 2 == 0:
        banner_overlay = frame.copy()
        cv2.rectangle(banner_overlay, (0, 0), (frame.shape[1], 58), (0, 0, 220), -1)
        cv2.addWeighted(banner_overlay, 0.35, frame, 0.65, 0, frame)
        cv2.putText(
            frame,
            "FIRE DETECTED",
            (max(20, frame.shape[1] // 2 - 145), 38),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 255, 255),
            3,
            cv2.LINE_AA,
        )

    cv2.putText(
        frame,
        last_aim_status[:70],
        (20, frame.shape[0] - 44),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (240, 240, 240),
        1,
        cv2.LINE_AA,
    )

    if last_error:
        cv2.putText(
            frame,
            last_error[:90],
            (20, frame.shape[0] - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (72, 72, 255),
            1,
            cv2.LINE_AA,
        )


def inference_worker(inference_frame: Any) -> None:
    global inference_thread_busy
    global latest_predictions, latest_inference_error, latest_inference_latency_ms
    
    global latest_inference_mode, latest_object_fire_detected, latest_object_smoke_detected, latest_object_labels, latest_object_confidence

    global latest_fire_center_x, latest_fire_center_y, latest_fire_bbox, latest_fire_frame

    global object_alert_active, last_object_firebase_time, last_object_fire_time, last_object_smoke_time

    global last_seen_fire_bbox_x, last_seen_fire_bbox_y, last_seen_fire_bbox_confidence, last_seen_fire_bbox_time

    started_at = time.perf_counter()
    predictions: List[Dict[str, Any]] = []
    last_error = ""
    mode = "hosted"

    try:
        encoded = encode_frame(inference_frame, JPEG_QUALITY)
        result = infer_frame_hosted(
            image_base64=encoded,
            confidence=CONFIDENCE_THRESHOLD,
            iou_threshold=IOU_THRESHOLD,
        )
        predictions = list(result.get("predictions") or [])

    except Exception as error:
        mode = "error"
        last_error = f"Object inference error: {error}"
        print(last_error, flush=True)

    fire_detected, labels, max_confidence = summarize_predictions(predictions)
    # Smoke detection must be evaluated after the Roboflow response has populated predictions.
    # In the broken x_mode version this ran before predictions existed, crashing the worker.
    smoke_detected = prediction_has_smoke(predictions)
    fire_found, fire_cx, fire_cy, fire_target_confidence, fire_bbox = get_best_fire_target(predictions)
    latency_ms = (time.perf_counter() - started_at) * 1000.0

    with prediction_lock:
        latest_predictions = predictions
        latest_inference_error = last_error
        latest_inference_latency_ms = latency_ms
        latest_inference_mode = mode
        latest_object_fire_detected = fire_detected
        latest_object_smoke_detected = smoke_detected
        latest_object_labels = labels
        latest_object_confidence = max_confidence

        if fire_found:
            latest_fire_center_x = fire_cx
            latest_fire_center_y = fire_cy
            latest_fire_bbox = fire_bbox
            latest_fire_frame = inference_frame.copy()

            # Record the latest valid fire bounding box for the locked-bbox aiming stage.
            last_seen_fire_bbox_x = fire_cx
            last_seen_fire_bbox_y = fire_cy
            last_seen_fire_bbox_confidence = fire_target_confidence
            last_seen_fire_bbox_time = time.time()
        else:
            latest_fire_center_x = None
            latest_fire_center_y = None
            latest_fire_bbox = None

    object_alert_active = fire_detected
    if fire_detected:
        last_object_fire_time = time.time()

    if smoke_detected:
        last_object_smoke_time = time.time()

    # Relay/SMS are controlled only by combined confirmation logic.
    # Object detection alone will NOT activate relay or send automatic SMS.
    update_auto_relay_state()

    if fire_detected:
        now = time.time()
        if now - last_object_firebase_time >= OBJECT_FIREBASE_COOLDOWN_SECONDS:
            payload = {
                "timestamp": iso_timestamp(),
                "max_temperature": 70.0,
                "alert_status": "CRITICAL",
                "source": "object_detection",
                "model_id": MODEL_ID,
                "detections": len(predictions),
                "labels": labels,
                "fire_confidence": round(max_confidence, 4),
                "fire_detected": True,
            }
            try:
                publish_to_firebase(payload, timeout=3.0)
                last_object_firebase_time = now
            except Exception as error:
                print("Object Firebase upload failed:", error, flush=True)

    with inference_lock:
        inference_thread_busy = False


# =====================================================
# THERMAL CAMERA FUNCTIONS
# =====================================================

def make_status_frame(width, height, title, message):
    import numpy as np
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.rectangle(frame, (0, 0), (width, height), (17, 24, 39), -1)
    cv2.putText(frame, title[:38], (18, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2, cv2.LINE_AA)
    y = 78
    words = str(message).split()
    line = ""
    for word in words:
        test = (line + " " + word).strip()
        if len(test) > 38:
            cv2.putText(frame, line, (18, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220,226,232), 1, cv2.LINE_AA)
            y += 25
            line = word
        else:
            line = test
    if line:
        cv2.putText(frame, line, (18, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220,226,232), 1, cv2.LINE_AA)
    return frame


def thermal_values_to_frame(flat_pixels: List[float], max_temp: float, status: str, hotspot_x_norm=None, hotspot_y_norm=None) -> Any:
    import numpy as np

    matrix = np.array(flat_pixels, dtype=np.float32).reshape((8, 8))

    # Upscale the 8x8 AMG8833 data into a smoother feed.
    upscaled = cv2.resize(
        matrix,
        (THERMAL_DISPLAY_WIDTH, THERMAL_DISPLAY_HEIGHT),
        interpolation=cv2.INTER_LINEAR,
    )

    clipped = np.clip(upscaled, MINTEMP, MAXTEMP)
    normalized = ((clipped - MINTEMP) / max(0.01, (MAXTEMP - MINTEMP)) * 255.0).astype("uint8")

    colored = cv2.applyColorMap(normalized, get_thermal_colormap())

    # Draw hotspot marker used by the automatic localization logic.
    if hotspot_x_norm is not None and hotspot_y_norm is not None:
        hx = int(max(0.0, min(1.0, float(hotspot_x_norm))) * (THERMAL_DISPLAY_WIDTH - 1))
        hy = int(max(0.0, min(1.0, float(hotspot_y_norm))) * (THERMAL_DISPLAY_HEIGHT - 1))
        cv2.drawMarker(colored, (hx, hy), (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=24, thickness=2)
        cv2.circle(colored, (hx, hy), 14, (255, 255, 255), 2)

    # Overlay status information.
    overlay = colored.copy()
    cv2.rectangle(overlay, (10, 10), (min(350, THERMAL_DISPLAY_WIDTH - 10), 118), (17, 24, 39), -1)
    cv2.addWeighted(overlay, 0.55, colored, 0.45, 0, colored)

    status_color = (255, 255, 255)
    if status == "HIGH":
        status_color = (0, 220, 255)
    elif status == "CRITICAL":
        status_color = (0, 0, 255)

    cv2.putText(colored, "AMG8833 Thermal Camera", (20, 38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.70, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(colored, f"Max Temp: {max_temp:.2f} C", (20, 68),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (240, 240, 240), 2, cv2.LINE_AA)
    cv2.putText(colored, f"Status: {status}", (20, 98),
                cv2.FONT_HERSHEY_SIMPLEX, 0.72, status_color, 2, cv2.LINE_AA)

    # Draw a simple temperature bar.
    bar_x1 = THERMAL_DISPLAY_WIDTH - 70
    bar_y1 = 30
    bar_x2 = THERMAL_DISPLAY_WIDTH - 38
    bar_y2 = THERMAL_DISPLAY_HEIGHT - 30

    for y in range(bar_y1, bar_y2):
        ratio = 1.0 - ((y - bar_y1) / max(1, bar_y2 - bar_y1))
        value = int(ratio * 255)
        color = cv2.applyColorMap(
            np.array([[value]], dtype=np.uint8),
            get_thermal_colormap(),
        )[0][0]
        cv2.line(colored, (bar_x1, y), (bar_x2, y), tuple(int(c) for c in color), 1)

    cv2.rectangle(colored, (bar_x1, bar_y1), (bar_x2, bar_y2), (255, 255, 255), 1)
    cv2.putText(colored, f"{MAXTEMP:.0f}C", (bar_x1 - 10, bar_y1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(colored, f"{MINTEMP:.0f}C", (bar_x1 - 10, bar_y2 + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

    return colored


def thermal_loop():
    global latest_thermal_frame, latest_thermal_status, latest_thermal_max_temp
    global latest_thermal_hotspot_x, latest_thermal_hotspot_y, latest_thermal_hotspot_temp
    global last_thermal_log_time, thermal_alert_active, last_thermal_fire_time

    set_thermal_status("THERMAL INITIALIZING", frame=make_status_frame(THERMAL_DISPLAY_WIDTH, THERMAL_DISPLAY_HEIGHT, "Thermal Camera", "Initializing AMG8833 on I2C..."))

    try:
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        time.sleep(0.2)
        sensor = None
        last_sensor_error = None
        for sensor_addr in (None, 0x69, 0x68):
            try:
                if sensor_addr is None:
                    sensor = adafruit_amg88xx.AMG88XX(i2c_bus)
                    print("AMG8833 connected using default address", flush=True)
                else:
                    sensor = adafruit_amg88xx.AMG88XX(i2c_bus, addr=sensor_addr)
                    print("AMG8833 connected at address 0x{:02X}".format(sensor_addr), flush=True)
                break
            except Exception as sensor_error:
                last_sensor_error = sensor_error
                sensor = None
        if sensor is None:
            raise last_sensor_error
    except Exception as error:
        message = "AMG8833 not detected: {}".format(error)
        print(message, flush=True)
        set_thermal_status(message, frame=make_status_frame(THERMAL_DISPLAY_WIDTH, THERMAL_DISPLAY_HEIGHT, "Thermal Camera Error", message))
        return

    time.sleep(0.2)
    print("Starting embedded thermal camera stream...", flush=True)
    set_thermal_status("THERMAL CONNECTED", frame=make_status_frame(THERMAL_DISPLAY_WIDTH, THERMAL_DISPLAY_HEIGHT, "Thermal Camera", "Connected. Waiting for first sensor reading..."))

    while program_running:
        current_time = time.time()

        try:
            pixels = sensor.pixels
            flat_pixels = [float(p) for row in pixels for p in row]
            max_temp = max(flat_pixels)

            # AMG8833 hotspot location, normalized 0..1 for automatic aiming.
            # The sensor is 8x8. col=0 is left, row=0 is top in the displayed thermal image.
            try:
                max_index = flat_pixels.index(max_temp)
                hotspot_row = int(max_index // 8)
                hotspot_col = int(max_index % 8)
                hotspot_x_norm = hotspot_col / 7.0
                hotspot_y_norm = hotspot_row / 7.0
                if THERMAL_HOTSPOT_FLIP_X:
                    hotspot_x_norm = 1.0 - hotspot_x_norm
                if THERMAL_HOTSPOT_FLIP_Y:
                    hotspot_y_norm = 1.0 - hotspot_y_norm
            except Exception:
                hotspot_x_norm = None
                hotspot_y_norm = None

            if max_temp >= CRITICAL_TEMP:
                alert_status = "CRITICAL"
            elif max_temp >= HIGH_TEMP:
                alert_status = "HIGH"
            else:
                alert_status = "NORMAL"

            thermal_alert_active = alert_status in ("HIGH", "CRITICAL")
            if thermal_alert_active:
                last_thermal_fire_time = current_time

            # Relay/SMS are controlled only by combined confirmation logic.
            # Thermal heat alone will NOT activate relay or send automatic SMS.
            update_auto_relay_state()

            if current_time - last_thermal_log_time >= FIREBASE_LOG_INTERVAL_SECONDS:
                timestamp_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                print(
                    f"[{timestamp_str}] Thermal Max Temp: {max_temp:.2f}C | Status: {alert_status}",
                    flush=True,
                )

                if FIREBASE_UPLOAD_ENABLED and max_temp >= HIGH_TEMP:

                    payload = {
                        "timestamp": timestamp_str,
                        "max_temperature": round(max_temp, 2),
                        "alert_status": alert_status,
                        "source": "thermal_camera",
                        "thermal_hotspot_x": hotspot_x_norm,
                        "thermal_hotspot_y": hotspot_y_norm,
                    }

                    try:
                        publish_to_firebase(payload, timeout=0.5)
                    except requests.exceptions.RequestException as error:
                        print(f"Firebase upload failed: {error}", flush=True)
                    except Exception as error:
                        print(f"Firebase upload failed: {error}", flush=True)

                last_thermal_log_time = current_time

            frame = thermal_values_to_frame(flat_pixels, max_temp, alert_status, hotspot_x_norm, hotspot_y_norm)

            with thermal_lock:
                latest_thermal_frame = frame
                latest_thermal_status = alert_status
                latest_thermal_max_temp = max_temp
                latest_thermal_hotspot_x = hotspot_x_norm
                latest_thermal_hotspot_y = hotspot_y_norm
                latest_thermal_hotspot_temp = max_temp

            time.sleep(THERMAL_UPDATE_INTERVAL_SECONDS)

        except Exception as error:
            message = "Thermal loop error: {}".format(error)
            print(message, flush=True)
            set_thermal_status(message, frame=make_status_frame(THERMAL_DISPLAY_WIDTH, THERMAL_DISPLAY_HEIGHT, "Thermal Camera Error", message))
            time.sleep(0.5)


# =====================================================
# TKINTER GUI FUNCTIONS
# =====================================================

def cv_frame_to_tk_photo(frame):
    success, buffer = cv2.imencode(".png", frame)
    if not success:
        return None
    return tk.PhotoImage(data=buffer.tobytes())


def make_blank_frame(width, height, text):
    import numpy as np
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    y = height // 2 - 20
    for line in text.splitlines():
        cv2.putText(frame, line, (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        y += 32
    return frame


def gui_send_sms():
    number = number_entry.get().strip()
    message = message_entry.get().strip()
    if number and message:
        # Manual test SMS uses what is currently typed in the boxes.
        threading.Thread(target=send_sms, args=(number, message), daemon=True).start()

def gui_send_escalation_sms():
    number = escalation_number_entry.get().strip()
    message = escalation_message_entry.get().strip()
    if number and message:
        threading.Thread(target=send_sms, args=(number, message), daemon=True).start()


def gui_test_primary_auto_sms():
    number = number_entry.get().strip()
    message = build_confirmed_fire_message(message_entry.get().strip(), 55.55, 0.88)
    if number and message:
        print("Debug: sending PRIMARY automatic-style SMS.", flush=True)
        threading.Thread(target=send_sms, args=(number, message), daemon=True).start()


def gui_test_escalation_auto_sms():
    number = escalation_number_entry.get().strip()
    message = build_escalation_fire_message(escalation_message_entry.get().strip(), 70.00, 0.92, get_gui_fail_time_limit())
    if number and message:
        print("Debug: sending BACKUP/ESCALATION automatic-style SMS.", flush=True)
        threading.Thread(target=send_sms, args=(number, message), daemon=True).start()


def get_gui_fail_time_limit():
    try:
        value = float(fail_time_entry.get().strip())
        if value <= 0:
            return DEFAULT_FAIL_TIME_LIMIT_SECONDS
        return value
    except Exception:
        return DEFAULT_FAIL_TIME_LIMIT_SECONDS


def show_invalid_number_warning(invalid_numbers):
    global invalid_number_warning_window

    try:
        if invalid_number_warning_window is not None and invalid_number_warning_window.winfo_exists():
            invalid_number_warning_window.lift()
            invalid_number_warning_window.focus_force()
            return
    except Exception:
        invalid_number_warning_window = None

    invalid_number_warning_window = tk.Toplevel(root)
    invalid_number_warning_window.title("Invalid Phone Number")
    invalid_number_warning_window.configure(bg="#0f172a")
    invalid_number_warning_window.resizable(False, False)
    invalid_number_warning_window.transient(root)
    invalid_number_warning_window.grab_set()

    window_w = 430
    window_h = 260

    try:
        root.update_idletasks()
        x = root.winfo_x() + (root.winfo_width() // 2) - (window_w // 2)
        y = root.winfo_y() + (root.winfo_height() // 2) - (window_h // 2)
        invalid_number_warning_window.geometry("{}x{}+{}+{}".format(window_w, window_h, x, y))
    except Exception:
        invalid_number_warning_window.geometry("{}x{}".format(window_w, window_h))

    card = tk.Frame(
        invalid_number_warning_window,
        bg="#1f2937",
        highlightbackground="#dc2626",
        highlightthickness=1,
    )
    card.pack(fill="both", expand=True, padx=14, pady=14)

    tk.Label(
        card,
        text="Invalid Phone Number",
        font=("Arial", 14, "bold"),
        fg="white",
        bg="#1f2937",
    ).pack(anchor="w", padx=14, pady=(14, 8))

    detail_lines = []
    for field_name, bad_number in invalid_numbers:
        shown_number = bad_number if str(bad_number).strip() else "(empty)"
        detail_lines.append("{}: {}".format(field_name, shown_number))

    message_text = (
        "The settings were not saved because one or more phone numbers are invalid.\n\n"
        + "\n".join(detail_lines)
        + "\n\nAccepted Philippine mobile formats:\n"
        + "09XXXXXXXXX, +639XXXXXXXXX, or 639XXXXXXXXX"
    )

    tk.Label(
        card,
        text=message_text,
        wraplength=370,
        justify="left",
        font=("Arial", 9),
        fg="#cbd5e1",
        bg="#1f2937",
    ).pack(anchor="w", padx=14, pady=(0, 12), fill="x")

    def close_warning():
        global invalid_number_warning_window
        try:
            invalid_number_warning_window.grab_release()
        except Exception:
            pass
        try:
            invalid_number_warning_window.destroy()
        except Exception:
            pass
        invalid_number_warning_window = None

    button_row = tk.Frame(card, bg="#1f2937")
    button_row.pack(side="bottom", fill="x", padx=14, pady=(0, 14))

    style_button(
        tk.Button(button_row, text="Confirm", command=close_warning),
        "danger"
    ).pack(fill="x")

    invalid_number_warning_window.protocol("WM_DELETE_WINDOW", close_warning)


def show_manual_relay_on_confirmation():
    global manual_relay_confirm_window

    try:
        if manual_relay_confirm_window is not None and manual_relay_confirm_window.winfo_exists():
            manual_relay_confirm_window.lift()
            manual_relay_confirm_window.focus_force()
            return
    except Exception:
        manual_relay_confirm_window = None

    manual_relay_confirm_window = tk.Toplevel(root)
    manual_relay_confirm_window.title("Confirm Manual Relay ON")
    manual_relay_confirm_window.configure(bg="#0f172a")
    manual_relay_confirm_window.resizable(False, False)
    manual_relay_confirm_window.transient(root)
    manual_relay_confirm_window.grab_set()

    window_w = 440
    window_h = 230

    try:
        root.update_idletasks()
        x = root.winfo_x() + (root.winfo_width() // 2) - (window_w // 2)
        y = root.winfo_y() + (root.winfo_height() // 2) - (window_h // 2)
        manual_relay_confirm_window.geometry("{}x{}+{}+{}".format(window_w, window_h, x, y))
    except Exception:
        manual_relay_confirm_window.geometry("{}x{}".format(window_w, window_h))

    card = tk.Frame(
        manual_relay_confirm_window,
        bg="#1f2937",
        highlightbackground="#d97706",
        highlightthickness=1,
    )
    card.pack(fill="both", expand=True, padx=14, pady=14)

    tk.Label(
        card,
        text="Confirm Manual Relay ON",
        font=("Arial", 14, "bold"),
        fg="white",
        bg="#1f2937",
    ).pack(anchor="w", padx=14, pady=(14, 8))

    message_text = (
        "You are about to manually turn ON the relay/pump.\n\n"
        "Fire and heat may not have been confirmed by the automatic detection process.\n\n"
        "Are you sure you want to continue?"
    )

    tk.Label(
        card,
        text=message_text,
        wraplength=380,
        justify="left",
        font=("Arial", 10),
        fg="#cbd5e1",
        bg="#1f2937",
    ).pack(anchor="w", padx=14, pady=(0, 12), fill="x")

    def close_confirmation():
        global manual_relay_confirm_window
        try:
            manual_relay_confirm_window.grab_release()
        except Exception:
            pass
        try:
            manual_relay_confirm_window.destroy()
        except Exception:
            pass
        manual_relay_confirm_window = None

    def confirm_manual_relay_on():
        close_confirmation()

        # Proceed with manual relay ON after confirmation.
        set_relay_control_mode("MANUAL")
        threading.Thread(
            target=relay_on,
            kwargs={"force": True},
            daemon=True
        ).start()

    button_row = tk.Frame(card, bg="#1f2937")
    button_row.pack(side="bottom", fill="x", padx=14, pady=(0, 14))
    button_row.grid_columnconfigure(0, weight=1)
    button_row.grid_columnconfigure(1, weight=1)

    style_button(
        tk.Button(button_row, text="Cancel", command=close_confirmation),
        "normal"
    ).grid(row=0, column=0, sticky="ew", padx=(0, 5))

    style_button(
        tk.Button(button_row, text="Confirm", command=confirm_manual_relay_on),
        "warning"
    ).grid(row=0, column=1, sticky="ew", padx=(5, 0))

    manual_relay_confirm_window.protocol("WM_DELETE_WINDOW", close_confirmation)

def gui_save_alert_settings():
    number = number_entry.get().strip()
    message = message_entry.get().strip()
    call_recipients = call_recipients_entry.get().strip()
    escalation_number = escalation_number_entry.get().strip()
    escalation_message = escalation_message_entry.get().strip()

    invalid_numbers = get_invalid_phone_numbers(number, escalation_number, call_recipients)
    if invalid_numbers:
        show_invalid_number_warning(invalid_numbers)
        try:
            alert_save_status_var.set("Settings were not saved. Invalid phone number detected.")
        except Exception:
            pass
        return

    fail_time = get_gui_fail_time_limit()
    ok, message_text = save_alert_settings(
        number,
        message,
        call_recipients,
        escalation_number,
        escalation_message,
        fail_time,
    )
    try:
        alert_save_status_var.set(message_text)
    except Exception:
        pass


def gui_make_call():
    number = number_entry.get().strip()
    if number:
        threading.Thread(target=make_call, args=(number,), daemon=True).start()


def gui_make_escalation_call():
    """Manual test call using the number currently typed in the escalation field."""
    number = escalation_number_entry.get().strip()
    if number:
        threading.Thread(target=make_call, args=(number,), daemon=True).start()


def gui_hang_up():
    threading.Thread(target=hang_up, daemon=True).start()


def gui_relay_on():
    global manual_relay_confirm_window

    # If the confirmation window is already open, do not open another one
    # and do not allow another relay ON action.
    try:
        if manual_relay_confirm_window is not None and manual_relay_confirm_window.winfo_exists():
            manual_relay_confirm_window.lift()
            manual_relay_confirm_window.focus_force()
            return
    except Exception:
        manual_relay_confirm_window = None

    # Always ask for confirmation before manually turning ON the relay,
    # regardless of whether the system is in AUTO or MANUAL mode.
    show_manual_relay_on_confirmation()


def gui_relay_off():
    set_relay_control_mode("MANUAL")
    threading.Thread(target=relay_off, kwargs={"force": True}, daemon=True).start()


def gui_set_auto_relay_mode():
    set_relay_control_mode("AUTO")
    # Safety: keep pump off until the automatic confirmation + aiming condition becomes true.
    threading.Thread(target=relay_off, kwargs={"force": True}, daemon=True).start()


def gui_set_manual_mode():
    set_manual_mode()


def gui_set_auto_mode():
    set_auto_mode()

    # When switching the system to AUTO mode, also return relay control to AUTO.
    # This prevents Manual Relay ON/OFF testing from leaving the relay locked
    # in MANUAL relay mode.
    set_relay_control_mode("AUTO")

    # Safety: keep the pump OFF until automatic heat + fire confirmation
    # and aiming/localization are completed.
    threading.Thread(
        target=relay_off,
        kwargs={"force": True},
        daemon=True
    ).start()


def update_status_labels():
    # Keep evaluating the confirmation state even when the feed loops update at different speeds.
    update_auto_relay_state()

    relay_text = "ON" if relay_is_on else "OFF"
    sms_text = "SENDING" if sms_is_sending else "READY"
    call_text = "CALLING" if call_is_sending else "READY"
    confirmed_text = "YES" if confirmed_fire_active else "NO"

    with thermal_lock:
        thermal_status_text = latest_thermal_status
        thermal_temp_text = latest_thermal_max_temp
        thermal_hotspot_x_text = latest_thermal_hotspot_x
        thermal_hotspot_y_text = latest_thermal_hotspot_y

    with prediction_lock:
        object_fire = latest_object_fire_detected
        object_conf = latest_object_confidence
        object_labels = ", ".join(latest_object_labels) if latest_object_labels else "none"
        object_error = latest_inference_error

    relay_status_var.set("Relay: " + relay_text + " | Relay Mode: " + get_relay_control_mode())
    sms_status_var.set("SMS: " + sms_text + " | Call: " + call_text)



    with x_position_lock:
        x_steps_text = current_x_position_steps
        x_degrees_text = current_x_position_steps / max(1.0, X_STEPS_PER_DEGREE)

    with y_position_lock:
        y_steps_text = current_y_position_steps
        y_degrees_text = current_y_position_steps / max(1.0, Y_STEPS_PER_DEGREE)

    mode_status_var.set("Mode: " + get_operation_mode() + " | " + auto_scan_status)
    confirmed_status_var.set(
        "Confirmed Fire: {} | {} | X: {} steps/{:.1f} deg | Y: {} steps/{:.1f} deg".format(
            confirmed_text,
            last_aim_status,
            x_steps_text,
            x_degrees_text,
            y_steps_text,
            y_degrees_text,
        )
    )



    if thermal_hotspot_x_text is not None and thermal_hotspot_y_text is not None:
        thermal_status_var.set("Thermal: {} | {:.2f} C | Hotspot: {:.0f}%, {:.0f}%".format(
            thermal_status_text,
            thermal_temp_text,
            thermal_hotspot_x_text * 100.0,
            thermal_hotspot_y_text * 100.0,
        ))
    else:
        thermal_status_var.set("Thermal: {} | {:.2f} C".format(thermal_status_text, thermal_temp_text))
    object_status_var.set("Object: {} | Conf: {:.0f}% | Labels: {}".format("FIRE" if object_fire else "Normal", object_conf * 100, object_labels))
    error_status_var.set(object_error[:90] if object_error else "No object inference error")


def update_object_feed_once():
    global inference_thread_busy
    ok = False
    raw_frame = None

    try:
        if camera_capture is not None:
            ok, raw_frame = camera_capture.read()
    except Exception as error:
        error_status_var.set("Camera read error: {}".format(error))

    if not ok or raw_frame is None:
        blank = make_blank_frame(DISPLAY_WIDTH, DISPLAY_HEIGHT, "Object camera not available")
        photo = cv_frame_to_tk_photo(blank)
        if photo is not None:
            object_feed_label.configure(image=photo, text="")
            object_feed_label.image = photo
        return

    started_at = time.perf_counter()
    inference_frame = cv2.resize(raw_frame, (INFERENCE_WIDTH, INFERENCE_HEIGHT), interpolation=cv2.INTER_AREA)
    display_frame = cv2.resize(raw_frame, (DISPLAY_WIDTH, DISPLAY_HEIGHT), interpolation=cv2.INTER_LINEAR)

    # Hosted inference runs in a background worker. The GUI camera display never waits for Roboflow.
    with inference_lock:
        if not inference_thread_busy:
            inference_thread_busy = True
            threading.Thread(target=inference_worker, args=(inference_frame.copy(),), daemon=True).start()

    with prediction_lock:
        predictions = list(latest_predictions)
        last_error = latest_inference_error
        latency_ms = latest_inference_latency_ms
        fire_detected = latest_object_fire_detected

    draw_predictions(display_frame, predictions, (INFERENCE_WIDTH, INFERENCE_HEIGHT))

    elapsed = max(time.perf_counter() - started_at, 1e-6)
    frame_times.append(elapsed)
    fps = len(frame_times) / max(sum(frame_times), 1e-6)

    draw_object_overlay(display_frame, fps=fps, detections_count=len(predictions), latency_ms=latency_ms, last_error=last_error, fire_detected=fire_detected)

    photo = cv_frame_to_tk_photo(display_frame)
    if photo is not None:
        object_feed_label.configure(image=photo, text="")
        object_feed_label.image = photo


def update_thermal_feed_once():
    # thermal_loop reads AMG8833 in the background. This only displays the newest frame, so it never blocks inference.
    with thermal_lock:
        frame = None if latest_thermal_frame is None else latest_thermal_frame.copy()
        status_text = latest_thermal_status

    if frame is None:
        frame = make_blank_frame(THERMAL_DISPLAY_WIDTH, THERMAL_DISPLAY_HEIGHT, "Waiting for thermal camera...\n{}".format(status_text))

    photo = cv_frame_to_tk_photo(frame)
    if photo is not None:
        thermal_feed_label.configure(image=photo, text="")
        thermal_feed_label.image = photo


def update_feed_loop():
    if not program_running:
        return
    update_object_feed_once()
    update_thermal_feed_once()
    update_status_labels()
    update_serial_monitor_once()
    root.after(100, update_feed_loop)


def on_close(event=None):
    global program_running, running1, running2, arduino, camera_capture
    program_running = False
    running1 = False
    running2 = False

    try:
        save_position_state(force=True)
    except Exception:
        pass

    try:
        relay_off(force=True)
    except Exception:
        pass
    try:
        GPIO.output(PUL1_PIN, GPIO.LOW)
        GPIO.output(PUL2_PIN, GPIO.LOW)
        GPIO.cleanup()
    except Exception:
        pass
    try:
        if camera_capture is not None:
            camera_capture.release()
    except Exception:
        pass
    try:
        if arduino:
            arduino.close()
    except Exception:
        pass
    try:
        http_session.close()
    except Exception:
        pass
    try:
        root.destroy()
    except Exception:
        pass


def toggle_fullscreen(event=None):
    current = bool(root.attributes("-fullscreen"))
    root.attributes("-fullscreen", not current)


def toggle_serial_monitor():
    global serial_monitor_visible

    serial_monitor_visible = not serial_monitor_visible
    try:
        if serial_monitor_visible:
            serial_monitor_card.grid()
            serial_toggle_button.configure(text="Hide Serial Monitor")
        else:
            serial_monitor_card.grid_remove()
            serial_toggle_button.configure(text="Show Serial Monitor")
    except Exception:
        pass


def clear_serial_monitor():
    try:
        if serial_log_text is not None:
            serial_log_text.configure(state="normal")
            serial_log_text.delete("1.0", "end")
            serial_log_text.configure(state="disabled")
    except Exception:
        pass


def update_serial_monitor_once():
    try:
        if serial_log_text is None:
            return

        updated = False
        count = 0
        serial_log_text.configure(state="normal")
        while count < 80:
            try:
                line = serial_log_queue.get_nowait()
            except queue.Empty:
                break
            serial_log_text.insert("end", line + "\n")
            updated = True
            count += 1

        if updated:
            # Keep the text box from growing forever on long tests.
            try:
                line_count = int(serial_log_text.index("end-1c").split(".")[0])
                if line_count > 700:
                    serial_log_text.delete("1.0", "120.0")
            except Exception:
                pass
            serial_log_text.see("end")
        serial_log_text.configure(state="disabled")
    except Exception:
        pass


def style_button(button, kind="normal"):
    colors = {
        "normal": ("#4b5563", "white"),
        "primary": ("#2563eb", "white"),
        "success": ("#16a34a", "white"),
        "warning": ("#d97706", "white"),
        "danger": ("#dc2626", "white"),
        "dark": ("#111827", "white"),
    }
    bg, fg = colors.get(kind, colors["normal"])
    try:
        button.configure(
            bg=bg,
            fg=fg,
            activebackground=bg,
            activeforeground=fg,
            relief="flat",
            bd=0,
            padx=8,
            pady=6,
            font=("Arial", 10, "bold"),
            cursor="hand2",
        )
    except Exception:
        pass
    return button


def make_card(parent, title):
    card = tk.Frame(parent, bg="#1f2937", highlightbackground="#374151", highlightthickness=1)
    header = tk.Label(card, text=title, font=("Arial", 12, "bold"), fg="white", bg="#1f2937")
    header.pack(anchor="w", padx=12, pady=(10, 6))
    return card


def make_labeled_entry(parent, label_text, initial_value=""):
    tk.Label(parent, text=label_text, fg="#d1d5db", bg="#1f2937", font=("Arial", 9, "bold")).pack(anchor="w", padx=12)
    entry = tk.Entry(parent, width=32, font=("Arial", 10), relief="flat", bg="#f9fafb", fg="#111827")
    entry.pack(padx=12, pady=(3, 8), fill="x", ipady=5)
    entry.insert(0, initial_value)
    return entry


def create_scrollable_panel(parent, bg="#111827"):
    canvas = tk.Canvas(parent, bg=bg, highlightthickness=0)
    scrollbar = tk.Scrollbar(parent, orient="vertical", command=canvas.yview)
    content = tk.Frame(canvas, bg=bg)
    content.bind("<Configure>", lambda event: canvas.configure(scrollregion=canvas.bbox("all")))
    canvas_window = canvas.create_window((0, 0), window=content, anchor="nw")
    canvas.configure(yscrollcommand=scrollbar.set)
    canvas.pack(side="left", fill="both", expand=True)
    scrollbar.pack(side="right", fill="y")

    def resize_content(event):
        canvas.itemconfig(canvas_window, width=event.width)
    canvas.bind("<Configure>", resize_content)
    return content


def update_adaptive_dimensions():
    global DISPLAY_WIDTH, DISPLAY_HEIGHT, THERMAL_DISPLAY_WIDTH, THERMAL_DISPLAY_HEIGHT

    screen_w = root.winfo_screenwidth()
    screen_h = root.winfo_screenheight()

    # New layout: top area contains object camera, thermal camera, and optional serial monitor.
    # Bottom area contains the controls.  Feed sizes are calculated so the whole top row stays visible.
    top_available_h = max(260, int(screen_h * 0.50) - 70)

    serial_w = int(screen_w * 0.28) if screen_w >= 1000 else int(screen_w * 0.34)
    left_feed_w = max(520, screen_w - serial_w - 90)

    if screen_w < 1050:
        DISPLAY_WIDTH = max(300, min(430, int(left_feed_w * 0.55)))
        THERMAL_DISPLAY_WIDTH = max(230, min(330, int(left_feed_w * 0.38)))
    else:
        DISPLAY_WIDTH = max(420, min(620, int(left_feed_w * 0.58)))
        THERMAL_DISPLAY_WIDTH = max(280, min(440, int(left_feed_w * 0.38)))

    DISPLAY_HEIGHT = min(int(DISPLAY_WIDTH * 0.75), top_available_h)
    THERMAL_DISPLAY_HEIGHT = min(int(THERMAL_DISPLAY_WIDTH * 0.75), top_available_h)


def build_gui():
    global root, object_feed_label, thermal_feed_label
    global number_entry, message_entry, call_recipients_entry
    global escalation_number_entry, escalation_message_entry, fail_time_entry
    global relay_status_var, sms_status_var, mode_status_var, confirmed_status_var, thermal_status_var, object_status_var, error_status_var
    global alert_save_status_var
    global serial_log_text, serial_monitor_card, serial_toggle_button

    root = tk.Tk()
    root.title(WINDOW_TITLE)
    root.configure(bg="#0f172a")

    update_adaptive_dimensions()

    if START_FULLSCREEN:
        root.attributes("-fullscreen", True)

    root.bind("<Escape>", on_close)
    root.bind("<F11>", toggle_fullscreen)

    screen_w = root.winfo_screenwidth()
    screen_h = root.winfo_screenheight()
    outer_pad = max(6, int(min(screen_w, screen_h) * 0.012))
    serial_width = max(260, min(420, int(screen_w * 0.28)))

    main = tk.Frame(root, bg="#0f172a")
    main.pack(fill="both", expand=True, padx=outer_pad, pady=outer_pad)
    main.grid_columnconfigure(0, weight=1)
    main.grid_rowconfigure(0, weight=3)
    main.grid_rowconfigure(1, weight=2)

    # =====================================================
    # TOP SECTION: Camera + thermal feed on the left, serial monitor on the right
    # =====================================================
    top_area = tk.Frame(main, bg="#0f172a")
    top_area.grid(row=0, column=0, sticky="nsew", pady=(0, outer_pad // 2))
    top_area.grid_columnconfigure(0, weight=1)
    top_area.grid_columnconfigure(1, weight=0)
    top_area.grid_rowconfigure(0, weight=1)

    feeds_panel = tk.Frame(top_area, bg="#0f172a")
    feeds_panel.grid(row=0, column=0, sticky="nsew", padx=(0, outer_pad // 2))
    feeds_panel.grid_columnconfigure(0, weight=1)
    feeds_panel.grid_rowconfigure(1, weight=1)

    title_row = tk.Frame(feeds_panel, bg="#0f172a")
    title_row.grid(row=0, column=0, sticky="ew", pady=(0, 6))
    title_row.grid_columnconfigure(0, weight=1)
    tk.Label(
        title_row,
        text="Blastoys Live Fire Monitoring",
        font=("Arial", max(16, min(24, int(screen_w / 72))), "bold"),
        fg="white",
        bg="#0f172a",
    ).grid(row=0, column=0, sticky="w")
    serial_toggle_button = style_button(tk.Button(title_row, text="Hide Serial Monitor", command=toggle_serial_monitor), "normal")
    serial_toggle_button.grid(row=0, column=1, sticky="e", padx=(8, 0))

    feed_row = tk.Frame(feeds_panel, bg="#0f172a")
    feed_row.grid(row=1, column=0, sticky="nsew")
    feed_row.grid_columnconfigure(0, weight=3)
    feed_row.grid_columnconfigure(1, weight=2)
    feed_row.grid_rowconfigure(0, weight=1)

    object_card = tk.Frame(feed_row, bg="#111827", highlightbackground="#334155", highlightthickness=1)
    object_card.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
    tk.Label(object_card, text="Object Detection Camera", font=("Arial", 13, "bold"), fg="white", bg="#111827").pack(anchor="w", padx=10, pady=(10, 5))
    object_feed_label = tk.Label(object_card, text="Starting object camera...", font=("Arial", 14), fg="white", bg="black")
    object_feed_label.pack(expand=True, padx=10, pady=(0, 10))

    thermal_card = tk.Frame(feed_row, bg="#111827", highlightbackground="#334155", highlightthickness=1)
    thermal_card.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
    tk.Label(thermal_card, text="AMG8833 Thermal Camera", font=("Arial", 13, "bold"), fg="white", bg="#111827").pack(anchor="w", padx=10, pady=(10, 5))
    thermal_feed_label = tk.Label(thermal_card, text="Starting thermal camera...", font=("Arial", 12), fg="white", bg="black")
    thermal_feed_label.pack(expand=True, padx=10, pady=(0, 10))

    serial_monitor_card = tk.Frame(top_area, bg="#111827", width=serial_width, highlightbackground="#334155", highlightthickness=1)
    serial_monitor_card.grid(row=0, column=1, sticky="nsew", padx=(outer_pad // 2, 0))
    serial_monitor_card.grid_propagate(False)
    serial_monitor_card.grid_rowconfigure(2, weight=1)
    serial_monitor_card.grid_columnconfigure(0, weight=1)

    tk.Label(serial_monitor_card, text="Built-in Serial Monitor", font=("Arial", 13, "bold"), fg="white", bg="#111827").grid(row=0, column=0, sticky="w", padx=10, pady=(10, 4))
    serial_tools = tk.Frame(serial_monitor_card, bg="#111827")
    serial_tools.grid(row=1, column=0, sticky="ew", padx=10, pady=(0, 6))
    serial_tools.grid_columnconfigure(0, weight=1)
    style_button(tk.Button(serial_tools, text="Clear", command=clear_serial_monitor), "dark").grid(row=0, column=0, sticky="ew")

    serial_text_frame = tk.Frame(serial_monitor_card, bg="#020617")
    serial_text_frame.grid(row=2, column=0, sticky="nsew", padx=10, pady=(0, 10))
    serial_text_frame.grid_rowconfigure(0, weight=1)
    serial_text_frame.grid_columnconfigure(0, weight=1)
    serial_log_text = tk.Text(serial_text_frame, bg="#020617", fg="#d1fae5", insertbackground="white", relief="flat", bd=0, wrap="word", font=("Courier New", 8), state="disabled")
    serial_scroll = tk.Scrollbar(serial_text_frame, orient="vertical", command=serial_log_text.yview)
    serial_log_text.configure(yscrollcommand=serial_scroll.set)
    serial_log_text.grid(row=0, column=0, sticky="nsew")
    serial_scroll.grid(row=0, column=1, sticky="ns")

    # =====================================================
    # BOTTOM SECTION: Controls on the left, SMS/call + status on the right
    # =====================================================
    bottom_area = tk.Frame(main, bg="#0f172a")
    bottom_area.grid(row=1, column=0, sticky="nsew", pady=(outer_pad // 2, 0))
    bottom_area.grid_columnconfigure(0, weight=1)
    bottom_area.grid_columnconfigure(1, weight=1)
    bottom_area.grid_rowconfigure(0, weight=1)

    controls_shell = tk.Frame(bottom_area, bg="#111827", highlightbackground="#334155", highlightthickness=1)
    controls_shell.grid(row=0, column=0, sticky="nsew", padx=(0, outer_pad // 2))
    sms_shell = tk.Frame(bottom_area, bg="#111827", highlightbackground="#334155", highlightthickness=1)
    sms_shell.grid(row=0, column=1, sticky="nsew", padx=(outer_pad // 2, 0))

    tk.Label(controls_shell, text="Controls and Buttons", font=("Arial", 15, "bold"), fg="white", bg="#111827").pack(anchor="w", padx=12, pady=(10, 4))
    tk.Label(sms_shell, text="SMS / Call Function and Status", font=("Arial", 15, "bold"), fg="white", bg="#111827").pack(anchor="w", padx=12, pady=(10, 4))

    controls_panel = create_scrollable_panel(controls_shell, bg="#111827")
    sms_panel = create_scrollable_panel(sms_shell, bg="#111827")

    global firebase_button_var

    firebase_button_var = tk.StringVar()

    firebase_button_var.set("Firebase Upload: ON")


    style_button( tk.Button(controls_panel,textvariable=firebase_button_var,command=toggle_firebase_upload),"warning").pack(fill="x", padx=10, pady=6)

    # Operation mode card
    mode_card = make_card(controls_panel, "Operation Mode")
    mode_card.pack(fill="x", padx=10, pady=6)
    mode_btn_frame = tk.Frame(mode_card, bg="#1f2937")
    mode_btn_frame.pack(fill="x", padx=10, pady=(0, 8))
    style_button(tk.Button(mode_btn_frame, text="Manual Mode", command=gui_set_manual_mode), "warning").pack(side="left", fill="x", expand=True, padx=(0, 4))
    style_button(tk.Button(mode_btn_frame, text="Auto Scan", command=gui_set_auto_mode), "success").pack(side="left", fill="x", expand=True, padx=(4, 0))


    style_button(
        tk.Button(mode_card, text="Reset X Reference to Center", command=reset_x_reference_to_center),
        "normal"
    ).pack(fill="x", padx=10, pady=(0, 6))

    style_button(
        tk.Button(mode_card, text="Reset Y Reference to Middle", command=reset_y_reference_to_middle),
        "normal"
    ).pack(fill="x", padx=10, pady=(0, 8))

    
    tk.Label(
        mode_card,
        text="Auto scans X-axis only. It stops on heat, checks webcam for fire, then aims and activates relay/SMS only after confirmation.",
        wraplength=max(280, int(screen_w * 0.38)),
        justify="left",
        font=("Arial", 9),
        fg="#cbd5e1",
        bg="#1f2937",
    ).pack(anchor="w", padx=10, pady=(0, 10))

    # Motor card
    motor_card = make_card(controls_panel, "Manual 2-Axis Motor Control")
    motor_card.pack(fill="x", padx=10, pady=6)
    motor_frame = tk.Frame(motor_card, bg="#1f2937")
    motor_frame.pack(pady=(0, 12))
    btn_up = style_button(tk.Button(motor_frame, text="UP", width=10, height=2), "primary")
    btn_down = style_button(tk.Button(motor_frame, text="DOWN", width=10, height=2), "primary")
    btn_left = style_button(tk.Button(motor_frame, text="LEFT", width=10, height=2), "primary")
    btn_right = style_button(tk.Button(motor_frame, text="RIGHT", width=10, height=2), "primary")
    btn_up.grid(row=0, column=1, pady=5)
    btn_left.grid(row=1, column=0, padx=5)
    btn_right.grid(row=1, column=2, padx=5)
    btn_down.grid(row=2, column=1, pady=5)
    btn_left.bind("<ButtonPress>", start_left)
    btn_left.bind("<ButtonRelease>", stop_motor1)
    btn_right.bind("<ButtonPress>", start_right)
    btn_right.bind("<ButtonRelease>", stop_motor1)
    btn_up.bind("<ButtonPress>", start_up)
    btn_up.bind("<ButtonRelease>", stop_motor2)
    btn_down.bind("<ButtonPress>", start_down)
    btn_down.bind("<ButtonRelease>", stop_motor2)

    # Relay card
    relay_card = make_card(controls_panel, "Manual Relay Control")
    relay_card.pack(fill="x", padx=10, pady=6)
    relay_frame = tk.Frame(relay_card, bg="#1f2937")
    relay_frame.pack(fill="x", padx=10, pady=(0, 10))
    style_button(tk.Button(relay_frame, text="Manual Relay ON", command=gui_relay_on), "success").pack(side="left", fill="x", expand=True, padx=(0, 4))
    style_button(tk.Button(relay_frame, text="Manual Relay OFF", command=gui_relay_off), "danger").pack(side="left", fill="x", expand=True, padx=(4, 0))
    style_button(tk.Button(relay_card, text="Automatic Relay Mode", command=gui_set_auto_relay_mode), "primary").pack(fill="x", padx=10, pady=(0, 6))
    tk.Label(relay_card, text="Manual ON/OFF directly controls the pump. Automatic Relay Mode only activates after heat + fire confirmation and aiming/localization.", wraplength=max(280, int(screen_w * 0.38)), justify="left", font=("Arial", 8), fg="#cbd5e1", bg="#1f2937").pack(anchor="w", padx=12, pady=(0, 8))

    # Alert settings card
    sim_card = make_card(sms_panel, "Automatic Fire Alert SMS + Backup Escalation")
    sim_card.pack(fill="x", padx=10, pady=6)
    saved_number, saved_message = get_saved_alert_settings()
    saved_escalation_number, saved_escalation_message, saved_fail_time = get_saved_escalation_settings()

    number_entry = make_labeled_entry(sim_card, "Primary SMS Phone Number:", saved_number)
    message_entry = make_labeled_entry(sim_card, "Primary Message Used When Relay Activates:", saved_message)

    escalation_number_entry = make_labeled_entry(sim_card, "Backup/Escalation Phone Number:", saved_escalation_number)
    escalation_message_entry = make_labeled_entry(sim_card, "Backup Message If Fire Is Not Put Out:", saved_escalation_message)
    fail_time_entry = make_labeled_entry(sim_card, "Fail Time Before Backup SMS Sends (seconds):", str(saved_fail_time))

    call_recipients_entry = make_labeled_entry(sim_card, "Automatic Call Recipients (comma/newline separated):", ", ".join(get_saved_call_recipients()))

    save_row = tk.Frame(sim_card, bg="#1f2937")
    save_row.pack(fill="x", padx=10, pady=(0, 6))
    style_button(tk.Button(save_row, text="Save Alert Settings", command=gui_save_alert_settings), "success").pack(fill="x")
    alert_save_status_var = tk.StringVar(value="Automatic SMS uses the saved primary/backup number and messages above.")
    tk.Label(sim_card, textvariable=alert_save_status_var, wraplength=max(280, int(screen_w * 0.38)), justify="left", font=("Arial", 8), fg="#cbd5e1", bg="#1f2937").pack(anchor="w", padx=12, pady=(0, 8))

    test_row = tk.Frame(sim_card, bg="#1f2937")
    test_row.pack(fill="x", padx=10, pady=(0, 6))
    style_button(tk.Button(test_row, text="Manual Primary SMS", command=gui_send_sms), "normal").pack(side="left", fill="x", expand=True, padx=(0, 4))
    style_button(tk.Button(test_row, text="Manual Backup SMS", command=gui_send_escalation_sms), "normal").pack(side="left", fill="x", expand=True, padx=4)

    debug_sms_row = tk.Frame(sim_card, bg="#1f2937")
    debug_sms_row.pack(fill="x", padx=10, pady=(0, 6))
    style_button(tk.Button(debug_sms_row, text="Test Auto Primary SMS", command=gui_test_primary_auto_sms), "primary").pack(side="left", fill="x", expand=True, padx=(0, 4))
    style_button(tk.Button(debug_sms_row, text="Test Auto Backup SMS", command=gui_test_escalation_auto_sms), "primary").pack(side="left", fill="x", expand=True, padx=4)

    call_row = tk.Frame(sim_card, bg="#1f2937")
    call_row.pack(fill="x", padx=10, pady=(0, 10))
    style_button(tk.Button(call_row, text="Call Primary", command=gui_make_call), "normal").pack(side="left", fill="x", expand=True, padx=(0, 4))
    style_button(tk.Button(call_row, text="Call Escalation", command=gui_make_escalation_call), "normal").pack(side="left", fill="x", expand=True, padx=4)
    style_button(tk.Button(call_row, text="Hang Up", command=gui_hang_up), "danger").pack(side="left", fill="x", expand=True, padx=(4, 0))

    tk.Label(
        sim_card,
        text="Automatic behavior: primary SMS sends when the relay/pump response activates. Backup SMS sends once if confirmed fire is still active after the fail time.",
        wraplength=max(280, int(screen_w * 0.38)),
        justify="left",
        font=("Arial", 8),
        fg="#cbd5e1",
        bg="#1f2937",
    ).pack(anchor="w", padx=12, pady=(0, 8))

    # Status card
    status_card = make_card(sms_panel, "System Status")
    status_card.pack(fill="x", padx=10, pady=6)

    firebase_button_var = tk.StringVar()
    firebase_button_var.set("Firebase Upload: ON")
    relay_status_var = tk.StringVar(value="Relay: OFF")
    sms_status_var = tk.StringVar(value="SMS: READY")
    mode_status_var = tk.StringVar(value="Mode: " + get_operation_mode())
    confirmed_status_var = tk.StringVar(value="Confirmed Fire: NO")
    thermal_status_var = tk.StringVar(value="Thermal: STARTING")
    object_status_var = tk.StringVar(value="Object: STARTING")
    error_status_var = tk.StringVar(value="No object inference error")

    for var in (relay_status_var, sms_status_var, mode_status_var, confirmed_status_var, thermal_status_var, object_status_var, error_status_var):
        tk.Label(status_card, textvariable=var, wraplength=max(280, int(screen_w * 0.38)), justify="left", font=("Arial", 9), fg="#e5e7eb", bg="#1f2937").pack(anchor="w", padx=12, pady=3)

    footer = tk.Frame(sms_panel, bg="#111827")
    footer.pack(fill="x", padx=10, pady=6)
    style_button(tk.Button(footer, text="Exit", height=2, command=on_close), "danger").pack(fill="x")
    tk.Label(footer, text="ESC = Exit | F11 = Toggle Fullscreen", font=("Arial", 8), fg="#94a3b8", bg="#111827").pack(pady=(6, 0))

    root.protocol("WM_DELETE_WINDOW", on_close)
    return root


# =====================================================
# STARTUP
# =====================================================

def startup():
    global arduino, camera_capture

    load_alert_settings()
    load_position_state()
    setup_gpio()

    print("Opening Arduino serial...", flush=True)
    try:
        arduino = open_arduino()
        print("Checking Arduino...", flush=True)
        send_command("STATUS", 7, clear_input=True)
    except Exception as error:
        print("Arduino failed to open:", error, flush=True)
        arduino = None

    print("Opening object detection camera...", flush=True)
    try:
        camera_capture = open_camera()
    except Exception as error:
        print("Camera failed to open:", error, flush=True)
        camera_capture = None

    threading.Thread(target=stepper1_loop, daemon=True).start()
    threading.Thread(target=stepper2_loop, daemon=True).start()
    threading.Thread(target=auto_scan_loop, daemon=True).start()
    threading.Thread(target=thermal_loop, daemon=True).start()


def main():
    startup()
    build_gui()
    root.after(100, update_feed_loop)
    root.mainloop()


if __name__ == "__main__":
    main()
