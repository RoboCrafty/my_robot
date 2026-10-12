import time

from lerobot_robot_my_parol6 import myParol6, myParol6Config

# Uses the default camera from the config; the controller must already be running with --sim
robot = myParol6(myParol6Config(host="rpi.local", port=5005))
robot.connect()

obs = robot.get_observation()
print({k: (v.shape if hasattr(v, "shape") else v) for k, v in obs.items()})

target = {k: v for k, v in obs.items() if k.endswith(".pos")}
target["joint_1.pos"] -=5.0  # nudge J1 by 10 deg
robot.send_action(target)

for _ in range(20):
    time.sleep(0.25)
    print("j1 =", round(robot.get_observation()["joint_1.pos"], 2))

robot.disconnect()
