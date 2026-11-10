


import mujoco
import cv2
import threading
import time
from flask import Flask, Response

app = Flask(__name__)

# Global variable to hold the most recent JPEG frame
latest_frame = None
lock = threading.Lock()

def physics_loop():
    """Runs the MuJoCo simulation in a background thread"""
    global latest_frame
    
    model = mujoco.MjModel.from_xml_path('parol6-scene1.xml')
    data = mujoco.MjData(model)
    
    # 640x480 is recommended for smooth network streaming. 
    # (1080p MJPEG can sometimes lag the browser)
    renderer = mujoco.Renderer(model, height=1080, width=1080)
    
    fps = 120
    frametime = 1.0 / fps
    mujoco.mj_step(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            
            if data.time >= frametime:
                renderer.update_scene(data, camera="wrist_cam")
                pixels = renderer.render()
                
                # Convert RGB to BGR for OpenCV
                bgr_pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
                
                # Encode the raw numpy array into a JPEG image
                ret, buffer = cv2.imencode('.jpg', bgr_pixels)
                if ret:
                    # Thread-safe update of the global frame
                    with lock:
                        latest_frame = buffer.tobytes()
                
                frametime += 1.0 / fps

def generate_stream():
    """Generator that yields the JPEG frames in the multipart format browsers expect"""
    while True:
        with lock:
            frame = latest_frame
            
        if frame is None:
            time.sleep(0.001)
            continue
            
        # Yield the MJPEG byte payload
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        
        # Limit the browser polling rate roughly to our FPS
        time.sleep(1.0 / 30)

@app.route('/')
def video_feed():
    """Route that serves the video stream"""
    return Response(generate_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # 1. Start the physics loop in a background thread
    sim_thread = threading.Thread(target=physics_loop, daemon=True)
    sim_thread.start()
    
    # 2. Start the web server on the main thread
    print("Simulation running! Open http://localhost:5001 in your browser to view the feed.")
    app.run(host='0.0.0.0', port=5001, threaded=True)