import tkinter as tk
from tkinter import filedialog, messagebox
import serial
import json

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
is_animating = False
current_wp_idx = 0       
sequence_data = []       

TRAVEL_JOINTS = [0, -47, 72, 0, 0, 0]
TRAVEL_POSE = [100, 0, 150, 0, 0, 0]

# ==========================================
#              CORE FUNCTIONS
# ==========================================

def update_status(message):
    status_var.set(message)
    root.after(3000, lambda: status_var.set("Ready"))

def send_data(*args):
    # Sends 12 comma-separated variables
    formatted_args = [f"{round(val, 2)}" for val in args]
    data = ",".join(formatted_args) + "\n"
    if ser:
        ser.write(data.encode())
    else:
        pass # Silenced mock output during fast animation

def on_slider_moved(event=None):
    if not is_animating and live_update_var.get():
        send_current_sliders()

def send_current_sliders():
    send_data(
        j1_val.get(), j2_val.get(), j3_val.get(), j4_val.get(), j5_val.get(), j6_val.get(),
        x_val.get(), y_val.get(), z_val.get(), rx_val.get(), ry_val.get(), rz_val.get()
    )
    update_status("📡 Sent current pose to ESP.")

def apply_to_sliders(joints, pose):
    sliders_j = [j1_val, j2_val, j3_val, j4_val, j5_val, j6_val]
    sliders_p = [x_val, y_val, z_val, rx_val, ry_val, rz_val]
    
    for slider, val in zip(sliders_j, joints):
        slider.set(val)
    for slider, val in zip(sliders_p, pose):
        slider.set(val)

def set_home_pose():
    apply_to_sliders([0]*6, [0]*6)
    update_status("Moved to Home Pose.")

def set_travel_pose():
    apply_to_sliders(TRAVEL_JOINTS, TRAVEL_POSE)
    update_status("Moved to Travel Pose.")

def get_current_pose_dict():
    return {
        "j": [j1_val.get(), j2_val.get(), j3_val.get(), j4_val.get(), j5_val.get(), j6_val.get()],
        "p": [x_val.get(), y_val.get(), z_val.get(), rx_val.get(), ry_val.get(), rz_val.get()]
    }

def jog_axis(slider, direction):
    """Adjusts the given slider by the step size and immediately sends the command."""
    step = jog_step_var.get()
    current_val = slider.get()
    new_val = current_val + (step * direction)
    
    # tk.Scale automatically clamps to its from_/to limits when using .set()
    slider.set(new_val)
    
    # Force send immediately for tactile jogging feedback
    send_current_sliders()

# ==========================================
#          SEQUENCE / WAYPOINT LOGIC
# ==========================================

def add_waypoint():
    wp = get_current_pose_dict()
    sequence_data.append(wp)
    update_sequence_listbox()
    listbox_seq.selection_clear(0, tk.END)
    listbox_seq.selection_set(tk.END)
    listbox_seq.see(tk.END)
    update_status("➕ Waypoint added.")

def update_selected_waypoint():
    selected = listbox_seq.curselection()
    if not selected:
        messagebox.showwarning("No Selection", "Please select a waypoint from the list to update.")
        return
    idx = selected[0]
    sequence_data[idx] = get_current_pose_dict()
    update_sequence_listbox()
    listbox_seq.selection_set(idx) 
    update_status(f"🔄 Waypoint {idx+1} updated with current sliders.")

def delete_waypoint():
    selected = listbox_seq.curselection()
    if not selected: return
    idx = selected[0]
    sequence_data.pop(idx)
    update_sequence_listbox()
    
    if len(sequence_data) > 0:
        new_idx = min(idx, len(sequence_data) - 1)
        listbox_seq.selection_set(new_idx)
    update_status("➖ Waypoint deleted.")

def move_wp_up():
    selected = listbox_seq.curselection()
    if not selected: return
    idx = selected[0]
    if idx == 0: return 
    sequence_data[idx], sequence_data[idx-1] = sequence_data[idx-1], sequence_data[idx]
    update_sequence_listbox()
    listbox_seq.selection_set(idx-1)
    listbox_seq.see(idx-1)

def move_wp_down():
    selected = listbox_seq.curselection()
    if not selected: return
    idx = selected[0]
    if idx == len(sequence_data) - 1: return 
    sequence_data[idx], sequence_data[idx+1] = sequence_data[idx+1], sequence_data[idx]
    update_sequence_listbox()
    listbox_seq.selection_set(idx+1)
    listbox_seq.see(idx+1)

def clear_sequence():
    if messagebox.askyesno("Clear", "Are you sure you want to clear all waypoints?"):
        sequence_data.clear()
        update_sequence_listbox()
        update_status("🗑️ Sequence cleared.")

def update_sequence_listbox():
    listbox_seq.delete(0, tk.END)
    for i, wp in enumerate(sequence_data):
        listbox_seq.insert(tk.END, f"WP {i+1} | J1: {wp['j'][0]}°, X: {wp['p'][0]}mm")

def load_selected_waypoint(event=None):
    if is_animating: return
    selected = listbox_seq.curselection()
    if not selected: return
    idx = selected[0]
    wp = sequence_data[idx]
    apply_to_sliders(wp['j'], wp['p'])
    update_status(f"Loaded Waypoint {idx+1} into sliders.")

def save_to_file():
    if not sequence_data:
        messagebox.showwarning("Empty", "No waypoints to save!")
        return
    file_path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON Files", "*.json")])
    if file_path:
        with open(file_path, 'w') as f:
            json.dump(sequence_data, f, indent=4)
        update_status(f"💾 Saved sequence to file.")

def load_from_file():
    file_path = filedialog.askopenfilename(filetypes=[("JSON Files", "*.json")])
    if file_path:
        with open(file_path, 'r') as f:
            global sequence_data
            sequence_data = json.load(f)
        update_sequence_listbox()
        update_status(f"📂 Loaded sequence from file.")

# ==========================================
#          POINT-TO-POINT ANIMATION
# ==========================================

def toggle_animation():
    global is_animating, current_wp_idx
    
    if len(sequence_data) < 2:
        messagebox.showwarning("Need More Waypoints", "Add at least 2 waypoints to the sequence to animate.")
        return

    if not is_animating:
        is_animating = True
        btn_loop.config(text="Stop Execution", bg="#ffcccc")
        selected = listbox_seq.curselection()
        current_wp_idx = selected[0] if selected else 0
        execute_anim_step()
    else:
        is_animating = False
        btn_loop.config(text="▶️ Start Sequence Cycling", bg="SystemButtonFace")
        update_status("Execution stopped.")

def execute_anim_step():
    global is_animating, current_wp_idx
    if not is_animating: return

    wp = sequence_data[current_wp_idx]
    apply_to_sliders(wp['j'], wp['p'])
    listbox_seq.selection_clear(0, tk.END)
    listbox_seq.selection_set(current_wp_idx)
    listbox_seq.see(current_wp_idx)
    update_status(f"Moving to Waypoint {current_wp_idx + 1}...")
    
    send_current_sliders()
    
    current_wp_idx = (current_wp_idx + 1) % len(sequence_data)
    delay_ms = int(duration_slider.get() * 1000)
    root.after(delay_ms, execute_anim_step)

# ==========================================
#                GUI SETUP
# ==========================================
root = tk.Tk()
root.title("ESP Robot Waypoint Admission Controller")
root.geometry("900x1050") 

# --- QUICK ACTIONS ---
frame_actions = tk.LabelFrame(root, text="Quick Actions & Presets", padx=10, pady=5)
frame_actions.pack(fill="x", padx=10, pady=5)

btn_home = tk.Button(frame_actions, text="🏠 Home Pose", command=set_home_pose, bg="#e6f2ff")
btn_home.grid(row=0, column=0, padx=5, pady=5, sticky="ew")

btn_travel = tk.Button(frame_actions, text="🧳 Travel Pose", command=set_travel_pose, bg="#ffe6e6")
btn_travel.grid(row=0, column=1, padx=5, pady=5, sticky="ew")

frame_actions.columnconfigure(0, weight=1)
frame_actions.columnconfigure(1, weight=1)

# --- MAIN LAYOUT ---
frame_main = tk.Frame(root)
frame_main.pack(fill="both", expand=True, padx=10, pady=5)
frame_main.columnconfigure(0, weight=2)
frame_main.columnconfigure(1, weight=1)

# --- ACTIVE SLIDERS (LEFT, TOP) ---
frame_sliders = tk.LabelFrame(frame_main, text="Active Pose Control", padx=10, pady=5)
frame_sliders.grid(row=0, column=0, sticky="nsew", padx=(0, 5), pady=(0, 5))

frame_sliders.columnconfigure(0, weight=1)
frame_sliders.columnconfigure(1, weight=1)

live_update_var = tk.BooleanVar(value=False)

j1_val = tk.Scale(frame_sliders, from_=-180, to=180, orient='horizontal', label='Joint 1', command=on_slider_moved)
j2_val = tk.Scale(frame_sliders, from_=-180, to=180, orient='horizontal', label='Joint 2', command=on_slider_moved)
j3_val = tk.Scale(frame_sliders, from_=-180, to=180, orient='horizontal', label='Joint 3', command=on_slider_moved)
j4_val = tk.Scale(frame_sliders, from_=-180, to=180, orient='horizontal', label='Joint 4', command=on_slider_moved)
j5_val = tk.Scale(frame_sliders, from_=-180, to=180, orient='horizontal', label='Joint 5', command=on_slider_moved)
j6_val = tk.Scale(frame_sliders, from_=-180, to=180, orient='horizontal', label='Joint 6', command=on_slider_moved)

x_val  = tk.Scale(frame_sliders, from_=-500, to=500, orient='horizontal', label='Pose X (mm)', command=on_slider_moved)
y_val  = tk.Scale(frame_sliders, from_=-500, to=500, orient='horizontal', label='Pose Y (mm)', command=on_slider_moved)
z_val  = tk.Scale(frame_sliders, from_=-500, to=500, orient='horizontal', label='Pose Z (mm)', command=on_slider_moved)
rx_val = tk.Scale(frame_sliders, from_=-180, to=180, orient='horizontal', label='Pose Rx (deg)', command=on_slider_moved)
ry_val = tk.Scale(frame_sliders, from_=-180, to=180, orient='horizontal', label='Pose Ry (deg)', command=on_slider_moved)
rz_val = tk.Scale(frame_sliders, from_=-180, to=180, orient='horizontal', label='Pose Rz (deg)', command=on_slider_moved)

for i, slider in enumerate([j1_val, j2_val, j3_val, j4_val, j5_val, j6_val]):
    slider.grid(row=i, column=0, sticky="ew", padx=10, pady=2)
for i, slider in enumerate([x_val, y_val, z_val, rx_val, ry_val, rz_val]):
    slider.grid(row=i, column=1, sticky="ew", padx=10, pady=2)

y_val.set(300)
z_val.set(300)

frame_publish = tk.Frame(frame_sliders)
frame_publish.grid(row=6, column=0, columnspan=2, pady=15, sticky="ew")

chk_live = tk.Checkbutton(frame_publish, text="Live Update ESP on Drag", variable=live_update_var, font=("Helvetica", 10, "bold"), fg="#d9534f")
chk_live.pack(side="left", padx=10)

btn_send_now = tk.Button(frame_publish, text="📡 Send Pose Now", command=send_current_sliders, bg="#e6ffe6", font=("Helvetica", 10, "bold"))
btn_send_now.pack(side="right", padx=10)

# --- JOGGING PANEL (LEFT, BOTTOM) ---
frame_jog = tk.LabelFrame(frame_main, text="Cartesian Jogging", padx=10, pady=5)
frame_jog.grid(row=1, column=0, sticky="nsew", padx=(0, 5), pady=(5, 0))

jog_step_var = tk.DoubleVar(value=5.0)

frame_step = tk.Frame(frame_jog)
frame_step.pack(fill="x", pady=5)
tk.Label(frame_step, text="Delta Step Size (mm/deg):", font=("Helvetica", 10, "bold")).pack(side="left", padx=(0, 10))

for val in [1.0, 5.0, 10.0, 25.0, 50.0]:
    tk.Radiobutton(frame_step, text=str(val), variable=jog_step_var, value=val).pack(side="left", padx=5)

frame_jog_btns = tk.Frame(frame_jog)
frame_jog_btns.pack(fill="x", expand=True, pady=10)

# Grouping sliders for the UI loop
cartesian_axes = [
    ("X", x_val), ("Y", y_val), ("Z", z_val),
    ("Rx", rx_val), ("Ry", ry_val), ("Rz", rz_val)
]

for i, (name, slider) in enumerate(cartesian_axes):
    col_offset = (i % 3) * 3
    row_offset = i // 3
    
    btn_minus = tk.Button(frame_jog_btns, text="◀", command=lambda s=slider: jog_axis(s, -1), bg="#f0f0f0")
    btn_minus.grid(row=row_offset, column=col_offset, padx=(10, 2), pady=5)
    
    lbl = tk.Label(frame_jog_btns, text=name, width=4, font=("Helvetica", 10, "bold"))
    lbl.grid(row=row_offset, column=col_offset + 1, padx=2)
    
    btn_plus = tk.Button(frame_jog_btns, text="▶", command=lambda s=slider: jog_axis(s, 1), bg="#f0f0f0")
    btn_plus.grid(row=row_offset, column=col_offset + 2, padx=(2, 20), pady=5)

# --- SEQUENCE BUILDER (RIGHT) ---
frame_seq = tk.LabelFrame(frame_main, text="Waypoint Sequence Builder", padx=10, pady=5)
frame_seq.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=(5, 0)) # Spans both rows

listbox_seq = tk.Listbox(frame_seq, height=25, selectbackground="#0078D7", exportselection=False)
listbox_seq.pack(fill="both", expand=True, pady=5)
listbox_seq.bind('<Double-1>', load_selected_waypoint) 

frame_list_tools = tk.Frame(frame_seq)
frame_list_tools.pack(fill="x", pady=2)

btn_add_wp = tk.Button(frame_list_tools, text="➕ Add", command=add_waypoint, bg="#e6ffe6", width=8)
btn_add_wp.grid(row=0, column=0, padx=2, pady=2, sticky="ew")

btn_update_wp = tk.Button(frame_list_tools, text="🔄 Update", command=update_selected_waypoint, bg="#fffde6", width=8)
btn_update_wp.grid(row=0, column=1, padx=2, pady=2, sticky="ew")

btn_del_wp = tk.Button(frame_list_tools, text="➖ Del", command=delete_waypoint, width=8)
btn_del_wp.grid(row=0, column=2, padx=2, pady=2, sticky="ew")

btn_up = tk.Button(frame_list_tools, text="⇧ Up", command=move_wp_up, width=8)
btn_up.grid(row=1, column=0, padx=2, pady=2, sticky="ew")

btn_down = tk.Button(frame_list_tools, text="⇩ Down", command=move_wp_down, width=8)
btn_down.grid(row=1, column=1, padx=2, pady=2, sticky="ew")

btn_clear = tk.Button(frame_list_tools, text="🗑️ Clear", command=clear_sequence, width=8)
btn_clear.grid(row=1, column=2, padx=2, pady=2, sticky="ew")

frame_list_tools.columnconfigure([0,1,2], weight=1)

frame_file = tk.Frame(frame_seq)
frame_file.pack(fill="x", pady=10)
btn_save = tk.Button(frame_file, text="💾 Save File", command=save_to_file)
btn_save.pack(side="left", fill="x", expand=True, padx=(0, 2))
btn_load = tk.Button(frame_file, text="📂 Load File", command=load_from_file)
btn_load.pack(side="right", fill="x", expand=True, padx=(2, 0))

# --- ANIMATION CONTROLS ---
frame_controls = tk.LabelFrame(root, text="ESP Command Dispatcher", padx=10, pady=10)
frame_controls.pack(fill="x", padx=10, pady=5)

duration_slider = tk.Scale(frame_controls, from_=0.5, to=10.0, resolution=0.5, orient='horizontal', label='PC Wait Delay Between Sends (Seconds)')
duration_slider.set(2.0)
duration_slider.pack(side="left", fill="x", expand=True, padx=10)

btn_loop = tk.Button(frame_controls, text="▶️ Start Sequence Cycling", font=("Helvetica", 12, "bold"), command=toggle_animation, height=2)
btn_loop.pack(side="right", fill="x", expand=True, padx=10, pady=10)

# --- STATUS BAR ---
status_var = tk.StringVar()
status_var.set("Ready")
status_bar = tk.Label(root, textvariable=status_var, bd=1, relief=tk.SUNKEN, anchor=tk.W, font=("Helvetica", 9, "italic"), fg="gray")
status_bar.pack(side=tk.BOTTOM, fill=tk.X)

root.mainloop()