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
    print(f"Warning: Could not open {SERIAL_PORT}. Running in offline mode.")

# --- STATE VARIABLES ---
is_looping = False
current_waypoint = 1

# --- PRESETS DICTIONARY ---
PRESETS = {
    "Preset 1 (From Image)": {
        "set1_j": [-90, -45, 64, -121, 100, 0],
        "set1_p": [150, 0, 200, 0, 0, 0], # X, Y, Z, Rx, Ry, Rz
        "set2_j": [5, 65, -61, 140, -100, 0],
        "set2_p": [200, 50, 150, 0, 0, 0],
        "delay": 3.5
    },
    "Preset 2 (Custom Name)": {
        "set1_j": [10, 20, 30, 40, 50, 60],
        "set1_p": [100, 100, 100, 45, 0, 0],
        "set2_j": [-10, -20, -30, -40, -50, -60],
        "set2_p": [-100, -100, 100, -45, 0, 0],
        "delay": 2.0
    }
}

TRAVEL_JOINTS = [0, -47, 72, 0, 0, 0]
TRAVEL_POSE = [100, 0, 150, 0, 0, 0]

def send_data(j1, j2, j3, j4, j5, j6, x, y, z, rx, ry, rz):
    # Sends 12 comma-separated variables
    data = f"{j1},{j2},{j3},{j4},{j5},{j6},{x},{y},{z},{rx},{ry},{rz}\n"
    if ser:
        ser.write(data.encode())
    else:
        print(f"Mock Serial Out -> {data.strip()}")

def on_slider_moved(event=None):
    if not is_looping:
        send_data(
            j1_s1.get(), j2_s1.get(), j3_s1.get(), j4_s1.get(), j5_s1.get(), j6_s1.get(),
            x_s1.get(), y_s1.get(), z_s1.get(), rx_s1.get(), ry_s1.get(), rz_s1.get()
        )

# --- POSE & PRESET FUNCTIONS ---
def apply_to_set1(joints, pose):
    sliders_j = [j1_s1, j2_s1, j3_s1, j4_s1, j5_s1, j6_s1]
    sliders_p = [x_s1, y_s1, z_s1, rx_s1, ry_s1, rz_s1]
    
    for slider, val in zip(sliders_j, joints):
        slider.set(val)
    for slider, val in zip(sliders_p, pose):
        slider.set(val)

def set_home_pose():
    print("Moving to Home Pose...")
    apply_to_set1([0]*6, [0]*6)

def set_travel_pose():
    print("Moving to Travel/Shutdown Pose...")
    apply_to_set1(TRAVEL_JOINTS, TRAVEL_POSE)

def load_preset(preset_name):
    data = PRESETS.get(preset_name)
    if not data: return
    
    # Set 1 (Triggers sending data automatically)
    apply_to_set1(data["set1_j"], data["set1_p"])
        
    # Set 2
    sliders_s2_j = [j1_s2, j2_s2, j3_s2, j4_s2, j5_s2, j6_s2]
    sliders_s2_p = [x_s2, y_s2, z_s2, rx_s2, ry_s2, rz_s2]
    
    for slider, val in zip(sliders_s2_j, data["set2_j"]):
        slider.set(val)
    for slider, val in zip(sliders_s2_p, data["set2_p"]):
        slider.set(val)
        
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
        send_data(
            j1_s1.get(), j2_s1.get(), j3_s1.get(), j4_s1.get(), j5_s1.get(), j6_s1.get(),
            x_s1.get(), y_s1.get(), z_s1.get(), rx_s1.get(), ry_s1.get(), rz_s1.get()
        )
        current_waypoint = 2
    else:
        send_data(
            j1_s2.get(), j2_s2.get(), j3_s2.get(), j4_s2.get(), j5_s2.get(), j6_s2.get(),
            x_s2.get(), y_s2.get(), z_s2.get(), rx_s2.get(), ry_s2.get(), rz_s2.get()
        )
        current_waypoint = 1

    delay_ms = int(delay_slider.get() * 1000)
    root.after(delay_ms, execute_loop_step)

# ==========================================
#                GUI SETUP
# ==========================================
root = tk.Tk()
root.title("Robot 6-Axis & Cartesian Control")
root.geometry("800x1050") # Widened to support side-by-side columns

# --- QUICK ACTIONS ---
frame_actions = tk.LabelFrame(root, text="Quick Actions & Presets", padx=10, pady=5)
frame_actions.pack(fill="x", padx=10, pady=5)

btn_home = tk.Button(frame_actions, text="🏠 Home Pose (All 0s)", command=set_home_pose, bg="#e6f2ff")
btn_home.grid(row=0, column=0, padx=5, pady=5, sticky="ew")

btn_travel = tk.Button(frame_actions, text="🧳 Travel/Shutdown Pose", command=set_travel_pose, bg="#ffe6e6")
btn_travel.grid(row=0, column=1, padx=5, pady=5, sticky="ew")

col, row = 0, 1
for preset_name in PRESETS.keys():
    btn = tk.Button(frame_actions, text=f"Load: {preset_name}", command=lambda p=preset_name: load_preset(p))
    btn.grid(row=row, column=col, padx=5, pady=5, sticky="ew")
    col += 1
    if col > 1:
        col, row = 0, row + 1

frame_actions.columnconfigure(0, weight=1)
frame_actions.columnconfigure(1, weight=1)

# --- HELPER FACTORY FOR SLIDERS ---
def create_sliders(parent, command=None):
    parent.columnconfigure(0, weight=1)
    parent.columnconfigure(1, weight=1)
    
    # Left Column: Joints
    j1 = tk.Scale(parent, from_=-180, to=180, orient='horizontal', label='Joint 1', command=command)
    j2 = tk.Scale(parent, from_=-180, to=180, orient='horizontal', label='Joint 2', command=command)
    j3 = tk.Scale(parent, from_=-180, to=180, orient='horizontal', label='Joint 3', command=command)
    j4 = tk.Scale(parent, from_=-180, to=180, orient='horizontal', label='Joint 4', command=command)
    j5 = tk.Scale(parent, from_=-180, to=180, orient='horizontal', label='Joint 5', command=command)
    j6 = tk.Scale(parent, from_=-180, to=180, orient='horizontal', label='Joint 6', command=command)
    
    for i, slider in enumerate([j1, j2, j3, j4, j5, j6]):
        slider.grid(row=i, column=0, sticky="ew", padx=15, pady=2)

    # Right Column: Cartesian Pose
    x  = tk.Scale(parent, from_=-500, to=500, orient='horizontal', label='Pose X (mm)', command=command)
    y  = tk.Scale(parent, from_=-500, to=500, orient='horizontal', label='Pose Y (mm)', command=command)
    z  = tk.Scale(parent, from_=-500, to=500, orient='horizontal', label='Pose Z (mm)', command=command)
    rx = tk.Scale(parent, from_=-180, to=180, orient='horizontal', label='Pose Rx (deg)', command=command)
    ry = tk.Scale(parent, from_=-180, to=180, orient='horizontal', label='Pose Ry (deg)', command=command)
    rz = tk.Scale(parent, from_=-180, to=180, orient='horizontal', label='Pose Rz (deg)', command=command)
    
    for i, slider in enumerate([x, y, z, rx, ry, rz]):
        slider.grid(row=i, column=1, sticky="ew", padx=15, pady=2)
        
    return j1, j2, j3, j4, j5, j6, x, y, z, rx, ry, rz

# --- SET 1 ---
frame_set1 = tk.LabelFrame(root, text="Set 1 (Manual Control / Waypoint A)", padx=10, pady=5)
frame_set1.pack(fill="x", padx=10, pady=5)
j1_s1, j2_s1, j3_s1, j4_s1, j5_s1, j6_s1, x_s1, y_s1, z_s1, rx_s1, ry_s1, rz_s1 = create_sliders(frame_set1, on_slider_moved)
y_s1.set(300)
z_s1.set(300)
# --- SET 2 ---
frame_set2 = tk.LabelFrame(root, text="Set 2 (Waypoint B for Looping)", padx=10, pady=5)
frame_set2.pack(fill="x", padx=10, pady=5)
j1_s2, j2_s2, j3_s2, j4_s2, j5_s2, j6_s2, x_s2, y_s2, z_s2, rx_s2, ry_s2, rz_s2 = create_sliders(frame_set2)
y_s2.set(300)
z_s2.set(300)
# --- LOOP CONTROLS ---
frame_controls = tk.LabelFrame(root, text="Looping Controls", padx=10, pady=10)
frame_controls.pack(fill="x", padx=10, pady=5)

delay_slider = tk.Scale(frame_controls, from_=0.5, to=10.0, resolution=0.5, orient='horizontal', length=380, label='Delay Between Points (Seconds)')
delay_slider.set(2.0)
delay_slider.pack()

btn_loop = tk.Button(frame_controls, text="Start Looping", font=("Helvetica", 12, "bold"), command=toggle_loop)
btn_loop.pack(pady=10)

root.mainloop()