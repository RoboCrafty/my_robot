import re
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
VCD_FILE = 'wokwi.vcd'
TARGET_CHANNEL = 'D4' # D0 is mapped to Joint 1 STEP in your diagram.json

def parse_vcd(filepath, target_chan):
    timestamps = []
    current_time = 0
    target_symbol = None
    
    print(f"Reading {filepath}...")
    
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            
            # 1. Find the symbol assigned to our target channel (e.g., 'D0')
            if line.startswith('$var') and target_chan in line:
                parts = line.split()
                target_symbol = parts[3] # Usually a character like '!' or '"'
                print(f"Found {target_chan} mapped to symbol '{target_symbol}'")
                
            # 2. Track time progression
            elif line.startswith('#'):
                current_time = int(line[1:]) # Time in microseconds (Wokwi default)
                
            # 3. Detect a Rising Edge (0 to 1 transition) on our target pin
            elif target_symbol and line == f"1{target_symbol}":
                timestamps.append(current_time)
                
    return timestamps

def plot_motion(timestamps):
    if len(timestamps) < 2:
        print("Not enough pulses found to plot motion.")
        return

    # Convert timestamps from microseconds to seconds
    times_sec = [t / 1_000_000.0 for t in timestamps]
    
    positions = list(range(len(times_sec))) # 1 pulse = 1 step
    velocities = [0] # Starting velocity is 0
    vel_times = [times_sec[0]]

    # Calculate instantaneous velocity (steps per second)
    for i in range(1, len(times_sec)):
        dt = times_sec[i] - times_sec[i-1]
        if dt > 0:
            velocity = 1.0 / dt # Steps per second (Hz)
            velocities.append(velocity)
            vel_times.append(times_sec[i])

    # Plotting
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    
    # Graph 1: Position over time
    ax1.plot(times_sec, positions, 'b-', linewidth=2)
    ax1.set_ylabel('Position (Steps)')
    ax1.set_title(f'Stepper Motion Profile Analysis ({TARGET_CHANNEL})')
    ax1.grid(True)
    
    # Graph 2: Velocity over time (This shows your Trapezoid / S-Curve!)
    ax2.plot(vel_times, velocities, 'r-', linewidth=1.5)
    ax2.set_ylabel('Velocity (Steps/sec)')
    ax2.set_xlabel('Time (Seconds)')
    ax2.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    try:
        pulse_times = parse_vcd(VCD_FILE, TARGET_CHANNEL)
        print(f"Captured {len(pulse_times)} step pulses. Generating graphs...")
        plot_motion(pulse_times)
    except FileNotFoundError:
        print(f"Error: Could not find '{VCD_FILE}'. Make sure you ran the Wokwi simulation and saved the logic analyzer output.")