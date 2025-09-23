# Raspberry Pi Mosquito Trap Control Script
# Uses pigpio for servos and RPi.GPIO for fans/lights/pressor

import time
import RPi.GPIO as GPIO
from pigpio import pi as pigpio_pi
from picamera2 import Picamera2
from datetime import datetime
import smbus
from enum import Enum
import os

# --- Configuration ---

# I2C Pins for RTC (do not change)
RTC_SDA = 2
RTC_SCL = 3

# Servo Pins (BCM numbering)
SERVO_DOOR_PIN = 12
SERVO_EVAC_PIN = 18

# Fan/Light/Pressor Pins (BCM)
FUNNEL_FAN_PIN = 17
EVAC_FAN_PIN = 27
TO_MARK_FAN_PIN = 22
PRESSOR_PIN = 23
LIGHT_PIN = 24

# --- Operation Variables ---
START_TIME_1 = 11  # 11:00
START_TIME_2 = 19  # 19:00
TRAPPING_MINUTES = 1 # TODO: Change to longer duration for deployment

# --- Servo Positions ---
DOOR_CLOSED_POS = 1500
DOOR_OPEN_POS = 500
EVAC_CLOSED_POS = 1500
EVAC_OPEN_POS = 2500

# --- Image Settings ---
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
SAVE_DIRECTORY = "/home/pi/trap_images/"

# --- State Machine ---
class TrapState(Enum):
    IDLE = 0
    TRAPPING = 1
    EVACUATION = 2
    MARKING = 3
    FINAL_EVAC = 4

current_state = TrapState.IDLE
minute_counter = 0
session_started = False

# --- Hardware Initialization ---

# Initialize RPi.GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(FUNNEL_FAN_PIN, GPIO.OUT)
GPIO.setup(EVAC_FAN_PIN, GPIO.OUT)
GPIO.setup(TO_MARK_FAN_PIN, GPIO.OUT)
GPIO.setup(PRESSOR_PIN, GPIO.OUT)
GPIO.setup(LIGHT_PIN, GPIO.OUT)

# Initialize pigpio for servos
pi = pigpio_pi()
if not pi.connected:
    raise RuntimeError("Cannot connect to pigpio daemon. Run 'sudo pigpiod' first.")

# Initialize I2C for RTC
bus = smbus.SMBus(1)
RTC_ADDRESS = 0x68  # PCF8523

# Initialize camera
picam2 = Picamera2()
capture_config = picam2.create_still_configuration(main={"size": (IMAGE_WIDTH, IMAGE_HEIGHT)})
picam2.configure(capture_config)
picam2.start()
print("Camera started and warming up...")
time.sleep(2)

# --- Helper Functions ---
def bcd_to_dec(bcd):
    return (bcd // 16 * 10) + (bcd % 16)

def get_rtc_time():
    try:
        time_data = bus.read_i2c_block_data(RTC_ADDRESS, 0x03, 3)
        sec = bcd_to_dec(time_data[0] & 0x7F)
        minute = bcd_to_dec(time_data[1] & 0x7F)
        hour = bcd_to_dec(time_data[2] & 0x3F)
        return hour, minute, sec
    except IOError:
        print("Error reading RTC.")
        return 0, 0, 0

def set_all_outputs_off():
    GPIO.output(FUNNEL_FAN_PIN, 1)
    GPIO.output(EVAC_FAN_PIN, 1)
    GPIO.output(TO_MARK_FAN_PIN, 1)
    GPIO.output(PRESSOR_PIN, 1)
    GPIO.output(LIGHT_PIN, 1)

def set_all_outputs_on():
    print("turned all outputs on")
    GPIO.output(FUNNEL_FAN_PIN, 0)
    GPIO.output(EVAC_FAN_PIN, 0)
    GPIO.output(TO_MARK_FAN_PIN, 0)
    GPIO.output(PRESSOR_PIN, 0)
    GPIO.output(LIGHT_PIN, 0)
    
def set_servo_position(pin, position_us):
    pi.set_servo_pulsewidth(pin, position_us)

def take_picture():
    save_dir = "test_images"
    os.makedirs(save_dir, exist_ok=True)

    print("10 images being taken")
    for i in range(1, 2):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(save_dir, f"image_{i}_{timestamp}.jpg")
        picam2.capture_file(filename)
        print(f"Saved {filename}")
        time.sleep(1)  

    picam2.stop()
    print("images saved in 'test_images' folder.")

# --- State Machine Functions ---
def execute_idle_state():
    global minute_counter, session_started
    minute_counter = 0
    session_started = False
    set_all_outputs_off()
    set_servo_position(SERVO_DOOR_PIN, DOOR_CLOSED_POS)
    set_servo_position(SERVO_EVAC_PIN, EVAC_CLOSED_POS)

def execute_trapping_state():
    global minute_counter, current_state
    set_servo_position(SERVO_DOOR_PIN, DOOR_CLOSED_POS)
    set_servo_position(SERVO_EVAC_PIN, EVAC_CLOSED_POS)
    set_all_outputs_off()
    GPIO.output(FUNNEL_FAN_PIN, 0)
    if minute_counter >= TRAPPING_MINUTES:
        print("Trapping complete, moving to evacuation.")
        current_state = TrapState.EVACUATION
        minute_counter = 0

def execute_evacuation_state():
    global current_state
    print("State: Evacuation to Marking")
    set_servo_position(SERVO_DOOR_PIN, DOOR_OPEN_POS)
    time.sleep(1)
    GPIO.output(TO_MARK_FAN_PIN, 0)
    time.sleep(4)
    GPIO.output(FUNNEL_FAN_PIN, 1)
    time.sleep(10)
    current_state = TrapState.MARKING

def execute_marking_state():
    global current_state
    print("State: Marking")
    set_servo_position(SERVO_DOOR_PIN, DOOR_CLOSED_POS)
    set_all_outputs_off()
    time.sleep(2)
    GPIO.output(PRESSOR_PIN, 0)
    time.sleep(1)
    GPIO.output(PRESSOR_PIN, 1)
    time.sleep(3)
    current_state = TrapState.FINAL_EVAC

def execute_final_evac_state():
    global current_state
    print("State: Final Evacuation")
    set_servo_position(SERVO_DOOR_PIN, DOOR_CLOSED_POS)
    set_servo_position(SERVO_EVAC_PIN, EVAC_OPEN_POS)
    GPIO.output(EVAC_FAN_PIN, 0)
    time.sleep(10)
    current_state = TrapState.IDLE

# --- Main Loop ---
if __name__ == "__main__":
    try:
        last_minute = -1
        while True:
            hour, minute, sec = get_rtc_time()
            print(f"State: {current_state.name} | Time: {hour:02}:{minute:02}:{sec:02} | Trap Timer: {minute_counter} mins")

            # Scheduled operation
            should_run = (hour >= START_TIME_1 and hour < START_TIME_1 + 3) or \
                         (hour >= START_TIME_2 and hour < START_TIME_2 + 3)
            
            if should_run and not session_started:
                print("Starting scheduled operation.")
                session_started = True
                current_state = TrapState.TRAPPING
                minute_counter = 0
                last_minute = -1
            elif not should_run and session_started:
                print("Scheduled operation complete. Returning to idle.")
                current_state = TrapState.IDLE

            # State machine
            if current_state == TrapState.IDLE:
                execute_idle_state()
            elif current_state == TrapState.TRAPPING:
                execute_trapping_state()
            elif current_state == TrapState.EVACUATION:
                execute_evacuation_state()
            elif current_state == TrapState.MARKING:
                execute_marking_state()
            elif current_state == TrapState.FINAL_EVAC:
                execute_final_evac_state()

            # Increment timer
            if current_state == TrapState.TRAPPING and minute != last_minute:
                minute_counter += 1
                last_minute = minute

            time.sleep(10)

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        set_all_outputs_off()
        pi.set_servo_pulsewidth(SERVO_DOOR_PIN, 0)
        pi.set_servo_pulsewidth(SERVO_EVAC_PIN, 0)
        pi.stop()
        GPIO.cleanup()
        picam2.stop()
        print("GPIO cleaned up and camera stopped.")
