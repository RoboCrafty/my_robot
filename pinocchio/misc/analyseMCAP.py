import sys
import numpy as np
import matplotlib.pyplot as plt
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory

def analyze_mcap(file_path):
    times = []
    z_positions = []

    print(f"Reading {file_path}...")
    
    with open(file_path, "rb") as f:
        # The DecoderFactory automatically handles the Foxglove Protobuf schemas
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema, channel, message, proto_msg in reader.iter_decoded_messages():
            if channel.topic == "/fk_pose":
                # log_time is in nanoseconds, convert to seconds
                times.append(message.log_time / 1e9)
                # Extract the Z-axis translation
                z_positions.append(proto_msg.pose.position.z)

    if not times:
        print("No /fk_pose data found in the MCAP file.")
        return

    # Convert to numpy arrays and normalize time to start at 0s
    t = np.array(times)
    t = t - t[0]
    z = np.array(z_positions)

    # Calculate Velocity (dz/dt) and Acceleration (dv/dt)
    dt = np.diff(t)
    # Avoid division by zero if timestamps are duplicated
    dt[dt == 0] = 1e-9 
    
    velocity = np.diff(z) / dt
    acceleration = np.diff(velocity) / dt[1:]

    # --- Plotting ---
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle('Z-Axis Kinematics Analysis')

    # 1. Position Plot
    axs[0].plot(t, z, label='Z Position (m)', color='blue')
    axs[0].set_ylabel('Position (m)')
    axs[0].grid(True)
    axs[0].legend()

    # 2. Velocity Plot
    axs[1].plot(t[:-1], velocity, label='Z Velocity (m/s)', color='orange')
    axs[1].set_ylabel('Velocity (m/s)')
    axs[1].grid(True)
    axs[1].legend()

    # 3. Acceleration Plot
    axs[2].plot(t[:-2], acceleration, label='Z Accel (m/s²)', color='red')
    axs[2].set_ylabel('Accel (m/s²)')
    axs[2].set_xlabel('Time (seconds)')
    axs[2].grid(True)
    axs[2].legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    analyze_mcap("test5.mcap")