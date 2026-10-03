import mujoco
import mujoco.viewer
import socket
import json
import threading
import time
import math

# Shared state between the network thread and MuJoCo thread
running = True
latest_targets = [0.0] * 6
latest_gripper = 0.0
# ROBOT_IP = "192.168.50.10"
ROBOT_IP = "rpi.local"
def udp_telemetry_listener():
    """Listens to the C++ app's 31Hz JSON telemetry broadcast."""
    global latest_targets, latest_gripper
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", 0))  # local address; the Pi replies to whatever port we send from
    sock.settimeout(0.1)
    
    # Resolve once: "rpi.local" goes through mDNS, which is slow to repeat per packet
    server_addr = (socket.gethostbyname(ROBOT_IP), 5005)
    sock.sendto(b"ping", server_addr)
    last_ping_time = 0
    
    while running:
        if time.time() - last_ping_time > 2.0:
            sock.sendto(b"ping", server_addr)
            last_ping_time = time.time()
        try:
            data, _ = sock.recvfrom(2048)
            msg = json.loads(data.decode('utf-8'))
            # print(msg)
            
            # The C++ app sends angles in degrees, MuJoCo uses radians.
            # Using msg["tgt"] (the Ruckig output) to drive the actuators
            latest_targets = [math.radians(deg) for deg in msg["pos"]]
            
            # Gripper is 0-140 in C++. MuJoCo left_jaw_joint is 0.0 to 0.027.
            latest_gripper = 0.027 - ((msg["grip"] / 140.0) * 0.027)
            
        except socket.timeout:
            # Re-register if the C++ app restarts or drops us
            sock.sendto(b"ping", server_addr)
        except json.JSONDecodeError:
            pass
            
    sock.close()

def main():
    global running
    
    # 1. Load your specific scene XML
    model = mujoco.MjModel.from_xml_path("assets/parol6-scene1.xml")
    data = mujoco.MjData(model)
    
    # 2. Start the UDP background thread
    listener_thread = threading.Thread(target=udp_telemetry_listener)
    listener_thread.daemon = True
    listener_thread.start()
    
    # 3. Launch the passive MuJoCo viewer
    print("Launching MuJoCo Viewer. Press 'sim on' in your C++ terminal.")
    try:
        with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
            
            while viewer.is_running():
                step_start = time.time()
                
                # Feed the latest UDP targets to the MuJoCo position actuators
                # Actuators 0-5 are the arm, Actuator 6 is the left_jaw_joint
                data.ctrl[0:6] = latest_targets
                data.ctrl[6] = latest_gripper
                
                # Step physics
                mujoco.mj_step(model, data)
                
                # Sync the viewer (cap at ~60fps visually)
                viewer.sync()
                
                # Physics loop timing (matches MuJoCo default 0.001 timestep in your XML)
                time_until_next_step = model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
    except KeyboardInterrupt:
        # FIX 2: Catch Ctrl+C gracefully
        print("\nReceived exit signal. Shutting down MuJoCo...")
        
    finally:
        # Guarantee cleanup runs no matter how the loop exits
        running = False
        listener_thread.join(timeout=1.0)
        print("Shutdown complete.")
                
    # Shutdown
    running = False
    listener_thread.join()

if __name__ == "__main__":
    main()



    