import tkinter as tk
import serial

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/cu.usbserial-0001'
BAUD_RATE = 115200

# Try to connect, but don't crash if the Arduino isn't plugged in
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    print(f"Connected to {SERIAL_PORT}")
except serial.SerialException:
    ser = None
    print(f"Warning: Could not open {SERIAL_PORT}. Running in offline testing mode.")

# --- STATE VARIABLES ---
is_looping = False
current_waypoint = 1  # Tracks whether we are moving to Set 1 or Set 2

def send_angles(j1, j2, j3, j4, j5, j6):
    """Formats and sends all 6 joint angle data points over serial."""
    # Formats as "j1,j2,j3,j4,j5,j6\n" to match the 6-axis Arduino parser
    data = f"{j1},{j2},{j3},{j4},{j5},{j6}\n"
    if ser:
        ser.write(data.encode())
    else:
        print(f"Mock Serial Out -> {data.strip()}")

def on_slider_moved(event=None):
    """Called whenever any Set 1 sliders are manually moved."""
    # Only send manual commands if we aren't actively running a loop
    if not is_looping:
        send_angles(
            slider1_set1.get(), slider2_set1.get(), slider3_set1.get(),
            slider4_set1.get(), slider5_set1.get(), slider6_set1.get()
        )

def toggle_loop():
    """Starts or stops the looping sequence between Waypoint A and Waypoint B."""
    global is_looping, current_waypoint

    if not is_looping:
        # Start Looping
        is_looping = True
        btn_loop.config(text="Stop Looping", bg="#ffcccc")
        current_waypoint = 1  # Reset to start at Waypoint 1
        execute_loop_step()
    else:
        # Stop Looping
        is_looping = False
        btn_loop.config(text="Start Looping", bg="SystemButtonFace")

def execute_loop_step():
    """Sends the current waypoint data and schedules the next one using a timer."""
    global current_waypoint

    if not is_looping:
        return  # Exit safely if the user stopped the loop

    # Send data based on the current waypoint target
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

    # Read the delay slider (in seconds), convert to ms, and schedule the next loop
    delay_ms = int(delay_slider.get() * 1000)
    root.after(delay_ms, execute_loop_step)

# ==========================================
#                GUI SETUP
# ==========================================
root = tk.Tk()
root.title("Robot 6-Axis Joint Control - Waypoint Looper")
root.geometry("650x900")  # Expanded height to accommodate 12 total sliders smoothly

# --- SET 1 (Manual Control & Waypoint A) ---
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

# --- SET 2 (Waypoint B) ---
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
delay_slider.set(2.0)  # Default to 2 seconds
delay_slider.pack()

btn_loop = tk.Button(frame_controls, text="Start Looping", font=("Helvetica", 12, "bold"), command=toggle_loop)
btn_loop.pack(pady=10)

root.mainloop()