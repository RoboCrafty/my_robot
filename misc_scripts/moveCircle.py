import socket
import time
import math

# Connection to your C++ controller
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

# Create the UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_cmd(cmd):
    """Helper to encode and send a string command to the C++ loop."""
    sock.sendto(cmd.encode('utf-8'), (UDP_IP, UDP_PORT))

# --- Circle Parameters ---
radius = 0.05        # 5 cm radius (0.1 meter diameter)
duration = 5.0       # 5 seconds to complete one full circle
update_rate = 30.0   # Send commands at 30 Hz
dt = 1.0 / update_rate

# Angular frequency (radians per second)
omega = (2 * math.pi) / duration

print("Setting to Tool Frame...")
send_cmd("cartframe tool")
time.sleep(0.1)

print(f"Drawing a {radius*100}cm circle over {duration} seconds...")
start_time = time.time()

try:
    while True:
        t = time.time() - start_time
        
        # Stop after one full circle
        if t > duration:
            break
            
        # Calculate instantaneous velocities
        vx = -radius * omega * math.sin(omega * t)
        vy = radius * omega * math.cos(omega * t)
        
        # Blast the commands over UDP
        # We send them separately because the C++ parser evaluates one command per packet
        send_cmd(f"cartjogvel x {vx:.5f}")
        send_cmd(f"cartjogvel y {vy:.5f}")
        
        # Sleep to maintain our 30Hz loop rate
        time.sleep(dt)

except KeyboardInterrupt:
    print("\nInterrupted by user.")

# Always ensure the robot stops when the script finishes!
print("Stopping motion...")
send_cmd("cartjogvel x 0.0")
send_cmd("cartjogvel y 0.0")
print("Done.")