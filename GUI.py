# PART 1/7 — Core Imports, Globals, Setup, UART/GPIO/SPI Initialization
# GPIO Pinout:
# KEY1 (Select)      -> GPIO 21
# KEY2 (Back)        -> GPIO 20
# KEY3 (Mode Toggle) -> GPIO 16
# Joystick (Optional) Up/Down/Left/Right -> To be defined in Part 2
# External Trigger   -> GPIO 17 (manual_button_pin)

import tkinter as tk
from tkinter import ttk
from sense_hat_c import SenseHatC  # Updated for Sense HAT (C)
try:
    from ballistics import simulate_trajectory, BULLETS
except ImportError:
    simulate_trajectory = None
    BULLETS = {'.308_M80': {}}
    logging.error("Failed to import ballistics module. GUI will load in safe mode.")
import time
import serial
import os
import math
import json
import RPi.GPIO as GPIO
import spidev
from gpiozero import Button
import threading
import traceback
import logging

# --- Logging Setup ---
logging.basicConfig(
    filename='ballistica_log.txt',
    filemode='a',
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# --- Sense HAT C Initialization ---
sense = SenseHatC()

# --- Orientation Helpers for Sense HAT C ---
def get_pitch_roll():
    accel = sense.get_accelerometer()
    x = accel.get('x', 0.0)
    y = accel.get('y', 0.0)
    z = accel.get('z', 0.0)
    denom_pitch = math.sqrt(y**2 + z**2) or 1e-6
    denom_roll = z if z != 0 else 1e-6
    pitch = math.degrees(math.atan2(x, denom_pitch))
    roll = math.degrees(math.atan2(y, denom_roll))
    return pitch, roll

def get_heading():
    mag = sense.get_magnetometer()
    heading = math.degrees(math.atan2(mag['y'], mag['x']))
    if heading < 0:
        heading += 360
    return heading

# --- Globals ---
latest_temp = latest_pressure = latest_humidity = 0
initial_pitch = initial_roll = 0
heading_offset = 0
highlighted_field_index = 0

mode = "manual"
selected_bullet = BULLETS['.308_M80']
wind_speed = 0.0
wind_angle = 0.0
fields = ['bullet', 'wind_speed', 'wind_direction']
current_ballistics_info = {
    'time_of_flight': 'N/A', 'drop': 'N/A', 'mil_adjustment': 'N/A',
    'wind_drift': 'N/A', 'range': 'N/A', 'atmospheric': 'Temp: N/A | Pressure: N/A | Humidity: N/A'
}

# --- Filepaths ---
calibration_file = 'calibration.txt'
settings_file = 'settings.json'
wind_input_file = 'wind_input.json'

import atexit

# --- GPIO Setup ---
# --- GPIO Setup ---
GPIO.setmode(GPIO.BCM)

# --- External Button ---
man_button = 17  # External manual trigger button
GPIO.setup(man_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# --- KEY Buttons on Waveshare 1.44" TFT HAT ---
button1 = Button(21)  # KEY1: Select
button2 = Button(20)  # KEY2: Back
button3 = Button(16)  # KEY3: Toggle Mode

# --- Joystick GPIO Pins for 1.44" TFT Screen HAT ---
JOYSTICK_UP = 6
JOYSTICK_DOWN = 19
JOYSTICK_LEFT = 5
JOYSTICK_RIGHT = 26
JOYSTICK_PRESS = 13  # Not used yet, but reserved

GPIO.setup(JOYSTICK_UP, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(JOYSTICK_DOWN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(JOYSTICK_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(JOYSTICK_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(JOYSTICK_PRESS, GPIO.IN, pull_up_down=GPIO.PUD_UP)


# --- SPI Setup (Waveshare screen) ---
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 50000
spi.mode = 0b00

# --- UART Setup with Safety ---
try:
    ser = serial.Serial("/dev/serial0", 9600, timeout=1)
except Exception as e:
    logging.warning("UART failed: %s", e)
    ser = None

# --- GPIO Pin Safety ---
try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(man_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
except RuntimeError:
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(man_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# --- Ensure GPIO cleanup on exit ---
atexit.register(GPIO.cleanup)

# --- Thread Safety Lock for Updates ---
update_lock = threading.Lock()
continuous_thread = None


# PART 2/7a — Display Labels, Sensor Polling, Wind Persistence, UART Rangefinder

# --- Update GUI Labels with Ballistics Info ---
def update_display_labels(labels):
    labels['time_of_flight'].config(text=current_ballistics_info['time_of_flight'])
    labels['drop'].config(text=current_ballistics_info['drop'])
    labels['mil_adjustment'].config(text=current_ballistics_info['mil_adjustment'])
    labels['wind_drift'].config(text=current_ballistics_info['wind_drift'])
    labels['range'].config(text=current_ballistics_info['range'])
    labels['atmospheric'].config(text=current_ballistics_info['atmospheric'])

# --- Read and Average Environmental Data from Sense HAT C ---
def get_average_atmospheric_conditions():
    global latest_temp, latest_pressure, latest_humidity

    temp_sum = 0
    pressure_sum = 0
    humidity_sum = 0

    for _ in range(3):
        try:
            temp_sum += sense.get_temperature()
            pressure_sum += sense.get_pressure()
            humidity_sum += sense.get_humidity()
        except Exception as e:
            logging.warning("Sensor read failed: %s", e)
            continue
        time.sleep(0.5)  # Short delay between reads for stability

    latest_temp = temp_sum / 3
    latest_pressure = pressure_sum / 3
    latest_humidity = humidity_sum / 3

    return latest_temp, latest_pressure, latest_humidity

# --- Wind Input Persistence (load and save from file) ---
def load_wind_inputs():
    global wind_speed, wind_angle
    if os.path.exists(wind_input_file):
        try:
            with open(wind_input_file, 'r') as f:
                data = json.load(f)
                wind_speed = data.get('wind_speed', 0.0)
                wind_angle = data.get('wind_angle', 0.0)
        except Exception as e:
            logging.warning("Failed to load wind input file: %s", e)


def save_wind_inputs():
    global wind_speed, wind_angle
    data = {'wind_speed': wind_speed, 'wind_angle': wind_angle}
    try:
        with open(wind_input_file, 'w') as f:
            json.dump(data, f)
    except Exception as e:
        logging.warning("Failed to save wind input file: %s", e)

# --- UART-Based Rangefinder Data Retrieval ---
def get_range():
    try:
        if ser and ser.in_waiting > 0:
            range_data = ser.readline().decode('utf-8').strip()
            range_value = int(range_data)
            return range_value
    except Exception as e:
        logging.warning("Rangefinder read error: %s", e)
    return 0

# --- Placeholder for Progress Feedback (used later during calculations) ---
def show_progress_indicator():
    print("Processing...")  # To be expanded with graphical feedback in GUI later

# PART 2/7b — Joystick Navigation, Button Logic, Field Highlighting, Input Control

# --- Field Input State and Joystick Input Modes ---
field_values = {
    'bullet': list(BULLETS.keys()),
    'wind_speed': wind_speed,
    'wind_direction': wind_angle
}

highlighted_field_index = 0  # Keeps track of which field is selected
fields = ['bullet', 'wind_speed', 'wind_direction']

# --- Joystick Direction Mapping (for GPIO or hat-based joystick logic) ---
def move_highlight(direction):
    global highlighted_field_index
    if direction == 'up':
        highlighted_field_index = (highlighted_field_index - 1) % len(fields)
    elif direction == 'down':
        highlighted_field_index = (highlighted_field_index + 1) % len(fields)

# --- Field Value Modification ---
def adjust_field_value(direction):
    global wind_speed, wind_angle, selected_bullet
    field = fields[highlighted_field_index]

    if field == 'wind_speed':
        if direction == 'left':
            wind_speed = max(0.0, wind_speed - 0.1)
        elif direction == 'right':
            wind_speed += 0.1
        elif direction == 'up':
            wind_speed += 0.5
        elif direction == 'down':
            wind_speed = max(0.0, wind_speed - 0.5)

    elif field == 'wind_direction':
        if direction == 'left':
            wind_angle = (wind_angle - 5) % 360
        elif direction == 'right':
            wind_angle = (wind_angle + 5) % 360
        elif direction == 'up':
            wind_angle = (wind_angle + 15) % 360
        elif direction == 'down':
            wind_angle = (wind_angle - 15) % 360

    elif field == 'bullet':
        current_idx = field_values['bullet'].index(selected_bullet)
        if direction == 'left':
            selected_bullet = field_values['bullet'][(current_idx - 1) % len(field_values['bullet'])]
        elif direction == 'right':
            selected_bullet = field_values['bullet'][(current_idx + 1) % len(field_values['bullet'])]

# --- Menu Navigation Buttons ---
def handle_button1_press():
    # Selects current highlighted field or confirms selection
    print(f"Button 1 (Select) pressed for: {fields[highlighted_field_index]}")

def handle_button2_press():
    # Back or exit submenu
    print("Button 2 (Back) pressed")

def handle_button3_press():
    global mode
    # Toggle between modes
    if mode == "manual":
        mode = "continuous"
    else:
        mode = "manual"
    print(f"Button 3 (Mode toggle) pressed, mode is now: {mode}")

# --- Attach Buttons to Handlers ---
button1.when_pressed = handle_button1_press
button2.when_pressed = handle_button2_press
button3.when_pressed = handle_button3_press

# --- Joystick Handler Placeholder ---
def on_joystick_input(direction):
    if direction in ['up', 'down']:
        move_highlight(direction)
    elif direction in ['left', 'right']:
        adjust_field_value(direction)

# --- Field Highlighting (visual feedback) ---
def get_highlighted_field():
    return fields[highlighted_field_index]



# PART 3/7 — GUI Field Layout, Ballistics Info, Bullet Selector, Wind Inputs

# --- GUI Field + Widget Setup ---
bullet_choice = ttk.Combobox(main, values=list(BULLETS.keys()))
bullet_choice.set(".308_M80")
bullet_choice.grid(row=0, column=1, sticky='w', padx=5, pady=2)

field_labels = {}
info_labels = {}
field_order = ['bullet', 'wind_speed', 'wind_direction']

# --- Bullet Field ---
ttk.Label(main, text="Bullet Type:").grid(row=0, column=0, sticky='e', padx=5, pady=2)
field_labels['bullet'] = ttk.Label(main, text=bullet_choice.get())
field_labels['bullet'].grid(row=0, column=2, sticky='w', padx=5, pady=2)

# --- Wind Speed Field ---
ttk.Label(main, text="Wind Speed (m/s):").grid(row=1, column=0, sticky='e', padx=5, pady=2)
field_labels['wind_speed'] = ttk.Label(main, text=f"{wind_speed:.2f} m/s")
field_labels['wind_speed'].grid(row=1, column=1, sticky='w', padx=5, pady=2)

# --- Wind Direction Field ---
ttk.Label(main, text="Wind Angle (deg):").grid(row=2, column=0, sticky='e', padx=5, pady=2)
field_labels['wind_direction'] = ttk.Label(main, text=f"{wind_angle:.2f}°")
field_labels['wind_direction'].grid(row=2, column=1, sticky='w', padx=5, pady=2)

# --- Ballistics Output Labels ---
output_keys = ['time_of_flight', 'drop', 'mil_adjustment', 'wind_drift', 'range', 'atmospheric']
for i, key in enumerate(output_keys, start=3):
    ttk.Label(main, text=key.replace('_', ' ').title() + ":").grid(row=i, column=0, sticky='e', padx=5, pady=2)
    info_labels[key] = ttk.Label(main, text="...")
    info_labels[key].grid(row=i, column=1, columnspan=2, sticky='w', padx=5, pady=2)

# --- Progress Label for status messages ---
progress_label = ttk.Label(main, text="")
progress_label.grid(row=10, column=0, columnspan=3, pady=(10, 0))

# --- Function to Refresh Output Info Display ---
def update_display():
    field_labels['wind_speed'].config(text=f"{wind_speed:.2f} m/s")
    field_labels['wind_direction'].config(text=f"{wind_angle:.2f}°")
    field_labels['bullet'].config(text=bullet_choice.get())
    for key in info_labels:
        info_labels[key].config(text=current_ballistics_info.get(key, '...'))

# --- Callback for Bullet Change ---
def on_bullet_select(event=None):
    global selected_bullet
    label = bullet_choice.get()
    if label in BULLETS:
        selected_bullet = BULLETS[label]
        update_display()
        save_settings()

bullet_choice.bind("<<ComboboxSelected>>", on_bullet_select)

# --- Proceed to Part 4: Joystick Navigation, Highlighting, and Field Modification ---

# PART 4a/7 — Joystick Navigation, Field Highlighting, and Field Value Editing

import time

last_toggle = 0  # For debounce timing on mode toggle
highlighted_field_index = 0  # Track selected field
highlight_field()  # Ensure the first field is highlighted at startup

# --- Highlight Field with Blue Color ---
def highlight_field():
    for i, field in enumerate(field_order):
        if i == highlighted_field_index:
            field_labels[field].config(background="lightblue")
        else:
            field_labels[field].config(background="SystemButtonFace")

# --- Field Navigation ---
def navigate_fields(direction):
    global highlighted_field_index
    if direction == 'up':
        highlighted_field_index = (highlighted_field_index - 1) % len(field_order)
    elif direction == 'down':
        highlighted_field_index = (highlighted_field_index + 1) % len(field_order)
    print(f"Navigated to field: {field_order[highlighted_field_index]}")
    highlight_field()

# --- Field Adjustment (Small = Left/Right, Large = Up/Down) ---
def modify_field_value(direction):
    global wind_speed, wind_angle, selected_bullet
    field = field_order[highlighted_field_index]

    if field == 'wind_speed':
        if direction == 'left':
            wind_speed = max(0.0, wind_speed - 0.1)
        elif direction == 'right':
            wind_speed = min(30.0, wind_speed + 0.1)
        elif direction == 'up':
            wind_speed = min(30.0, wind_speed + 0.5)
        elif direction == 'down':
            wind_speed = max(0.0, wind_speed - 0.5)
        print(f"Wind speed adjusted: {wind_speed:.2f} m/s")

    elif field == 'wind_direction':
        if direction == 'left':
            wind_angle = (wind_angle - 5) % 360
        elif direction == 'right':
            wind_angle = (wind_angle + 5) % 360
        elif direction == 'up':
            wind_angle = (wind_angle + 15) % 360
        elif direction == 'down':
            wind_angle = (wind_angle - 15) % 360
        print(f"Wind direction adjusted: {wind_angle:.2f}°")

    elif field == 'bullet':
        keys = list(BULLETS.keys())
        current_idx = keys.index(bullet_choice.get())
        if direction == 'left':
            bullet_choice.set(keys[(current_idx - 1) % len(keys)])
        elif direction == 'right':
            bullet_choice.set(keys[(current_idx + 1) % len(keys)])
        on_bullet_select()
        print(f"Bullet changed to: {bullet_choice.get()}")

    update_display()
    highlight_field()


4B

# PART 4b/7 — Button Logic, Joystick Input Routing, and GPIO Event Detection

# --- Button 1 (KEY1): Select Current Field ---
def on_button1_press():
    print("KEY1 (Select) pressed")
    update_display()
    highlight_field()

# --- Button 2 (KEY2): Back or Cancel ---
def on_button2_press():
    print("KEY2 (Back) pressed")
    # Reserved for submenu/back navigation in future

# --- Button 3 (KEY3): Toggle Between Manual and Continuous Mode ---
def on_button3_press():
    global mode, last_toggle
    now = time.time()
    if now - last_toggle > 0.5:  # debounce period
        mode = "continuous" if mode == "manual" else "manual"
        print(f"KEY3 (Mode toggle): Mode switched to {mode}")
        last_toggle = now

# --- GPIO Bindings for Buttons ---
button1.when_pressed = on_button1_press
button2.when_pressed = on_button2_press
button3.when_pressed = on_button3_press

# --- Joystick Input Dispatcher ---
def joystick_input(direction):
    if direction in ['up', 'down']:
        navigate_fields(direction)
    elif direction in ['left', 'right']:
        modify_field_value(direction)

# --- Register Joystick GPIO Events ---
GPIO.add_event_detect(JOYSTICK_UP, GPIO.FALLING, callback=lambda ch: joystick_input('up'), bouncetime=150)
GPIO.add_event_detect(JOYSTICK_DOWN, GPIO.FALLING, callback=lambda ch: joystick_input('down'), bouncetime=150)
GPIO.add_event_detect(JOYSTICK_LEFT, GPIO.FALLING, callback=lambda ch: joystick_input('left'), bouncetime=150)
GPIO.add_event_detect(JOYSTICK_RIGHT, GPIO.FALLING, callback=lambda ch: joystick_input('right'), bouncetime=150)

# --- Developer Note ---
# Joystick GPIO callbacks are now active. Movement will automatically trigger navigation or editing.
# Joystick press (GPIO 13) is reserved for future functionality.


# PART 5/7 — Ballistics Calculation, Sensor Sync, Rangefinding, Environmental Averaging

# --- Atmospheric Sensor Sampling (3 reads every 30s) ---
def get_average_atmospheric_conditions():
    global latest_temp, latest_pressure, latest_humidity
    temp_sum = pressure_sum = humidity_sum = 0
    for _ in range(3):
        temp_sum += sense.get_temperature()
        pressure_sum += sense.get_pressure()
        humidity_sum += sense.get_humidity()
        time.sleep(10)
    latest_temp = temp_sum / 3
    latest_pressure = pressure_sum / 3
    latest_humidity = humidity_sum / 3

# --- Rangefinder Reading with UART ---
def get_range():
    try:
        if ser and ser.in_waiting > 0:
            ser.flushInput()
            raw = ser.readline().decode('utf-8').strip()
            if raw.isdigit():
                val = int(raw)
                if 0 < val < 3000:
                    return val
    except Exception as e:
        logging.error("Range read failed: %s", e)
    return 0

# --- Calculate Elevation Using Pitch and Range ---
def calculate_elevations(range_distance):
    pitch = sense.get_orientation_degrees()['pitch'] - initial_pitch
    return range_distance * math.tan(math.radians(pitch)), 0

# --- Calculate Relative Wind Angle ---
def get_relative_wind_angle():
   heading = (sense.get_compass() - heading_offset + 360) % 360
   return (wind_angle - heading + 360) % 360


# --- Ballistics Calculation Trigger ---
def update_ballistics():
    get_average_atmospheric_conditions()
    range_to_target = get_range()
    target_elevation, shooter_elevation = calculate_elevations(range_to_target)
    rel_wind_angle = get_relative_wind_angle()
    try:
        result = simulate_trajectory(
            bullet=selected_bullet,
            range_m=range_to_target,
            wind_speed_mps=wind_speed,
            wind_angle_deg=rel_wind_angle,
            shooter_elev_m=shooter_elevation,
            target_elev_m=target_elevation,
            temp_c=latest_temp,
            pressure_hpa=latest_pressure,
            humidity=latest_humidity,
            pitch_deg=sense.get_orientation_degrees()['pitch'] - initial_pitch,
            roll_deg=sense.get_orientation_degrees()['roll'] - initial_roll
        )
        current_ballistics_info['time_of_flight'] = f"{result['time_of_flight']:.2f} s"
        current_ballistics_info['drop'] = f"{result['drop_m']:.2f} m / {result['drop_in']:.2f} in"
        current_ballistics_info['mil_adjustment'] = f"{result['mil_adjustment']:.2f} mil"
        current_ballistics_info['wind_drift'] = f"{result['wind_drift_mil']:.2f} mil"
        current_ballistics_info['range'] = f"{range_to_target} m"
        current_ballistics_info['atmospheric'] = f"Temp: {latest_temp:.1f}°C | Pressure: {latest_pressure:.1f} hPa | Hum: {latest_humidity:.1f}%"
        update_display()
    except Exception as e:
        logging.error("Ballistics calculation failed: %s", traceback.format_exc())
        current_ballistics_info['time_of_flight'] = "ERROR"
        current_ballistics_info['drop'] = "ERROR"
        current_ballistics_info['mil_adjustment'] = "ERROR"
        current_ballistics_info['wind_drift'] = "ERROR"
        current_ballistics_info['range'] = "0"
        current_ballistics_info['atmospheric'] = "Sensor Error"
        update_display()

# --- Proceed to Part 6: Continuous Mode Threading, Manual Trigger Handling, and Progress Indicators ---
# PART 6/7 — Manual Trigger, Continuous Mode, Progress Feedback, Sync

# --- Manual Trigger Button ---
def handle_manual_trigger(channel=None):
    if mode == "manual":
        logging.info("Manual trigger pressed. Starting calculation...")
        progress_label.config(text="Calculating...")
        root.update_idletasks()
        update_ballistics()
        progress_label.config(text="Update Complete")
        root.after(1500, lambda: progress_label.config(text=""))

GPIO.add_event_detect(man_button, GPIO.FALLING, callback=handle_manual_trigger, bouncetime=300)

# --- Continuous Update Loop ---
def schedule_continuous_updates():
    global continuous_thread

    def loop():
        logging.info("Continuous mode thread started.")
        while mode == "continuous":
            if update_lock.acquire(blocking=False):
                try:
                    progress_label.config(text="Updating...")
                    root.update_idletasks()
                    logging.info("Continuous update triggered.")
                    root.update()
                    update_ballistics()
                    progress_label.config(text="Update Complete")
                    root.after(1500, lambda: progress_label.config(text=""))
                finally:
                    update_lock.release()
            time.sleep(3)  # Continuous update interval

    if continuous_thread is None or not continuous_thread.is_alive():
        continuous_thread = threading.Thread(target=loop, daemon=True)
        continuous_thread.start()

# --- GUI Exit Cleanup ---
def graceful_exit():
    GPIO.cleanup()
    root.destroy()

# --- Final GUI Bindings ---
root.protocol("WM_DELETE_WINDOW", graceful_exit)

# --- Highlight Active Field and Sync Display at Launch ---
highlight_field()
update_display()

# --- Confirm Sensor Access and Initial Sync ---
try:
    heading = get_heading()
    pitch, roll = get_pitch_roll()
    logging.info(f"Initial orientation: Pitch={pitch:.2f}, Roll={roll:.2f}, Heading={heading:.2f}")
except Exception as e:
    logging.warning(f"Sensor read error at launch: {e}")
    progress_label.config(text="⚠️ Sensor error. Check calibration or connection.")

# --- Proceed to Part 7: Final Touches, Calibration Mode, Wind Menu, Splash, Logging, and Export ---

# PART 7/7 — Calibration UI, Wind Menu, Final Logging, Wrap-up

# --- Wind Settings Window ---
def open_wind_settings():
    win = tk.Toplevel(root)
    win.title("Wind Settings")

    tk.Label(win, text="Wind Speed (m/s):").grid(row=0, column=0, padx=10, pady=5, sticky='e')
    speed_var = tk.DoubleVar(value=wind_speed)
    speed_entry = tk.Entry(win, textvariable=speed_var)
    speed_entry.grid(row=0, column=1, padx=10, pady=5)

    tk.Label(win, text="Wind Direction (° or cardinal):").grid(row=1, column=0, padx=10, pady=5, sticky='e')
    angle_var = tk.StringVar(value=str(wind_angle))
    angle_entry = tk.Entry(win, textvariable=angle_var)
    angle_entry.grid(row=1, column=1, padx=10, pady=5)

    def save_wind():
        global wind_speed, wind_angle
        try:
            wind_speed = float(speed_var.get())
            try:
                wind_angle = float(angle_var.get())
            except ValueError:
                cardinal = angle_var.get().strip().upper()
                wind_angle = {
                    'N': 0, 'NE': 45, 'E': 90, 'SE': 135, 'S': 180, 'SW': 225, 'W': 270, 'NW': 315
                }.get(cardinal, 0)
            save_settings()
            update_display()
            win.destroy()
        except Exception as e:
            logging.error("Failed to save wind input: %s", e)
            tk.messagebox.showerror("Input Error", str(e))

    tk.Button(win, text="Save", command=save_wind).grid(row=2, column=0, columnspan=2, pady=10)
    win.transient(root)
    win.grab_set()
    win.wait_window()


# --- Calibration GUI ---
def calibrate():
    cal_win = tk.Toplevel(root)
    cal_win.title("Calibrate Sensors")

    tk.Label(cal_win, text="Place device on flat surface and press 'Calibrate'.").pack(pady=10)

    def perform_pitch_roll_cal():
        global initial_pitch, initial_roll
        orientation = sense.get_orientation_degrees()
        initial_pitch = orientation['pitch']
        initial_roll = orientation['roll']
        with open(calibration_file, 'w') as f:
            f.write(f"{initial_pitch},{initial_roll}")
        logging.info("Pitch/Roll calibration saved: pitch=%.2f, roll=%.2f", initial_pitch, initial_roll)
        tk.Label(cal_win, text="Now point the rifle NORTH and press 'Zero Compass'.").pack(pady=10)
        tk.Button(cal_win, text="Zero Compass", command=perform_compass_cal).pack(pady=5)

    def perform_compass_cal():
        heading = sense.get_compass()
        with open("compass_cal.txt", 'w') as f:
            f.write(str(heading))
        logging.info("Compass heading zeroed at: %.2f°", heading)
        tk.Label(cal_win, text="Calibration complete!", fg="green").pack(pady=5)
        tk.Button(cal_win, text="Done", command=cal_win.destroy).pack(pady=10)

    tk.Button(cal_win, text="Calibrate", command=perform_pitch_roll_cal).pack(pady=10)

# --- Final Main Menu Additions (Optional Buttons) ---
ttk.Button(main, text="Wind Settings", command=open_wind_settings).grid(row=11, column=0, columnspan=2, pady=5)
ttk.Button(main, text="Calibrate Device", command=calibrate).grid(row=12, column=0, columnspan=2, pady=5)

# --- Final Launch Setup ---
logging.info("RAT 9 Ballistics GUI fully initialized and ready.")
highlight_field()
update_display()
root.mainloop()
GPIO.cleanup()
