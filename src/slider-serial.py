import tkinter as tk
import serial

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/cu.usbserial-0001'
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    print(f"Connected to {SERIAL_PORT}")
except serial.SerialException:
    ser = None
    print(f"Warning: Could not open {SERIAL_PORT}. Running in offline testing mode.")

# --- STATE VARIABLES ---
is_looping = False
current_waypoint = 1

# --- PRESETS DICTIONARY ---
# You can easily change names, add more, or edit values right here
PRESETS = {
    "Preset 1 (From Image)": {
        "set1": [-90, -45, 64, -121, 100, 0],
        "set2": [5, 65, -61, 140, -100, 0],
        "delay": 3.5
    },
    "Preset 2 (Custom Name)": {
        "set1": [10, 20, 30, 40, 50, 60],
        "set2": [-10, -20, -30, -40, -50, -60],
        "delay": 2.0
    }
}

TRAVEL_POSE = [0, -47, 72, 0, 0, 0] # Edit these to match your safe shutdown pose

def send_angles(j1, j2, j3, j4, j5, j6):
    data = f"{j1},{j2},{j3},{j4},{j5},{j6}\n"
    if ser:
        ser.write(data.encode())
    else:
        print(f"Mock Serial Out -> {data.strip()}")

def on_slider_moved(event=None):
    if not is_looping:
        send_angles(
            slider1_set1.get(), slider2_set1.get(), slider3_set1.get(),
            slider4_set1.get(), slider5_set1.get(), slider6_set1.get()
        )

# --- NEW: POSE & PRESET FUNCTIONS ---

def apply_pose_to_set1(angles):
    """Updates Set 1 sliders and immediately sends the command."""
    sliders = [slider1_set1, slider2_set1, slider3_set1, slider4_set1, slider5_set1, slider6_set1]
    for slider, angle in zip(sliders, angles):
        slider.set(angle)
    # The set() method triggers on_slider_moved automatically, so it will send the data.

def set_home_pose():
    print("Moving to Home Pose...")
    apply_pose_to_set1([0, 0, 0, 0, 0, 0])

def set_travel_pose():
    print("Moving to Travel/Shutdown Pose...")
    apply_pose_to_set1(TRAVEL_POSE)

def load_preset(preset_name):
    """Loads a preset from the dictionary into the sliders."""
    data = PRESETS.get(preset_name)
    if not data: return
    
    # Update Set 1
    sliders_set1 = [slider1_set1, slider2_set1, slider3_set1, slider4_set1, slider5_set1, slider6_set1]
    for slider, angle in zip(sliders_set1, data["set1"]):
        slider.set(angle)
        
    # Update Set 2
    sliders_set2 = [slider1_set2, slider2_set2, slider3_set2, slider4_set2, slider5_set2, slider6_set2]
    for slider, angle in zip(sliders_set2, data["set2"]):
        slider.set(angle)
        
    # Update Delay
    delay_slider.set(data["delay"])
    print(f"Loaded {preset_name}")

# --- LOOPING LOGIC ---
def toggle_loop():
    global is_looping, current_waypoint
    if not is_looping:
        is_looping = True
        btn_loop.config(text="Stop Looping", bg="#ffcccc")
        current_waypoint = 1
        execute_loop_step()
    else:
        is_looping = False
        btn_loop.config(text="Start Looping", bg="SystemButtonFace")

def execute_loop_step():
    global current_waypoint
    if not is_looping: return

    if current_waypoint == 1:
        send_angles(
            slider1_set1.get(), slider2_set1.get(), slider3_set1.get(),
            slider4_set1.get(), slider5_set1.get(), slider6_set1.get()
        )
        current_waypoint = 2
    else:
        send_angles(
            slider1_set2.get(), slider2_set2.get(), slider3_set2.get(),
            slider4_set2.get(), slider5_set2.get(), slider6_set2.get()
        )
        current_waypoint = 1

    delay_ms = int(delay_slider.get() * 1000)
    root.after(delay_ms, execute_loop_step)

# ==========================================
#                GUI SETUP
# ==========================================
root = tk.Tk()
root.title("Robot 6-Axis Joint Control - Waypoint Looper")
root.geometry("650x1050")  # Made taller to fit the new buttons

# --- QUICK ACTIONS (NEW) ---
frame_actions = tk.LabelFrame(root, text="Quick Actions & Presets", padx=10, pady=5)
frame_actions.pack(fill="x", padx=10, pady=5)

# Special Poses
btn_home = tk.Button(frame_actions, text="🏠 Home Pose (All 0s)", command=set_home_pose, bg="#e6f2ff")
btn_home.grid(row=0, column=0, padx=5, pady=5, sticky="ew")

btn_travel = tk.Button(frame_actions, text="🧳 Travel/Shutdown Pose", command=set_travel_pose, bg="#ffe6e6")
btn_travel.grid(row=0, column=1, padx=5, pady=5, sticky="ew")

# Dynamic Preset Buttons based on the dictionary
col = 0
row = 1
for preset_name in PRESETS.keys():
    # Capture the current preset_name in the lambda using default arg (p=preset_name)
    btn = tk.Button(frame_actions, text=f"Load: {preset_name}", command=lambda p=preset_name: load_preset(p))
    btn.grid(row=row, column=col, padx=5, pady=5, sticky="ew")
    col += 1
    if col > 1:  # 2 buttons per row
        col = 0
        row += 1

# Configure grid to expand buttons evenly
frame_actions.columnconfigure(0, weight=1)
frame_actions.columnconfigure(1, weight=1)

# --- SET 1 ---
frame_set1 = tk.LabelFrame(root, text="Set 1 (Manual Control / Waypoint A)", padx=10, pady=5)
frame_set1.pack(fill="x", padx=10, pady=5)

slider1_set1 = tk.Scale(frame_set1, from_=-180, to=180, orient='horizontal', length=450, label='Joint 1 Angle', command=on_slider_moved)
slider1_set1.pack()
slider2_set1 = tk.Scale(frame_set1, from_=-180, to=180, orient='horizontal', length=450, label='Joint 2 Angle', command=on_slider_moved)
slider2_set1.pack()
slider3_set1 = tk.Scale(frame_set1, from_=-180, to=180, orient='horizontal', length=450, label='Joint 3 Angle', command=on_slider_moved)
slider3_set1.pack()
slider4_set1 = tk.Scale(frame_set1, from_=-180, to=180, orient='horizontal', length=450, label='Joint 4 Angle', command=on_slider_moved)
slider4_set1.pack()
slider5_set1 = tk.Scale(frame_set1, from_=-180, to=180, orient='horizontal', length=450, label='Joint 5 Angle', command=on_slider_moved)
slider5_set1.pack()
slider6_set1 = tk.Scale(frame_set1, from_=-180, to=180, orient='horizontal', length=450, label='Joint 6 Angle', command=on_slider_moved)
slider6_set1.pack()

# --- SET 2 ---
frame_set2 = tk.LabelFrame(root, text="Set 2 (Waypoint B for Looping)", padx=10, pady=5)
frame_set2.pack(fill="x", padx=10, pady=5)

slider1_set2 = tk.Scale(frame_set2, from_=-180, to=180, orient='horizontal', length=450, label='Joint 1 Angle')
slider1_set2.pack()
slider2_set2 = tk.Scale(frame_set2, from_=-180, to=180, orient='horizontal', length=450, label='Joint 2 Angle')
slider2_set2.pack()
slider3_set2 = tk.Scale(frame_set2, from_=-180, to=180, orient='horizontal', length=450, label='Joint 3 Angle')
slider3_set2.pack()
slider4_set2 = tk.Scale(frame_set2, from_=-180, to=180, orient='horizontal', length=450, label='Joint 4 Angle')
slider4_set2.pack()
slider5_set2 = tk.Scale(frame_set2, from_=-180, to=180, orient='horizontal', length=450, label='Joint 5 Angle')
slider5_set2.pack()
slider6_set2 = tk.Scale(frame_set2, from_=-180, to=180, orient='horizontal', length=450, label='Joint 6 Angle')
slider6_set2.pack()

# --- LOOP CONTROLS ---
frame_controls = tk.LabelFrame(root, text="Looping Controls", padx=10, pady=10)
frame_controls.pack(fill="x", padx=10, pady=5)

delay_slider = tk.Scale(frame_controls, from_=0.5, to=10.0, resolution=0.5, orient='horizontal', length=380, label='Delay Between Points (Seconds)')
delay_slider.set(2.0)
delay_slider.pack()

btn_loop = tk.Button(frame_controls, text="Start Looping", font=("Helvetica", 12, "bold"), command=toggle_loop)
btn_loop.pack(pady=10)

root.mainloop()