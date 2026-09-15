from roboticstoolbox.robot.Robot import Robot  
import roboticstoolbox as rtb
import numpy as np
import swift
import spatialmath as sm
import spatialgeometry as sg

env = swift.Swift()
env.launch(realtime=True)

class MyRobot(Robot):  
    def __init__(self):  
        # Load your URDF file  
        links, name, urdf_string, urdf_filepath = self.URDF_read("parol6.urdf",tld="/Users/fudayl/git/my_robot/src/")  
          
        # Initialize the robot  
        super().__init__(  
            links,  
            name=name,  
            manufacturer="parol",  
            urdf_string=urdf_string,  
            urdf_filepath=urdf_filepath,  
        )  
          
        # Optionally set default joint configurations  
        self.qz = np.zeros(self.n)  # zero configuration  
        self.qr = np.array([...])   # ready configuration


parol = MyRobot()
env.add(parol)

Teq = parol.fkine(parol.q) * sm.SE3.Tx(0.2) * sm.SE3.Ty(0.0)
axes = sg.Axes(length=0.1, pose=Teq)
env.add(axes)
arrived = False

dt = 0.005


while not arrived:
    v, arrived = rtb.p_servo(parol.fkine(parol.q), Teq, gain=1, threshold=0.01)
    J = parol.jacobe(parol.q)
    parol.qd = np.linalg.pinv(J) @ v
    env.step(dt)

arrived = False
Teq = parol.fkine(parol.q) * sm.SE3.Tx(-0.2) * sm.SE3.Ty(0.1)
axes = sg.Axes(length=0.1, pose=Teq)
env.add(axes)




while not arrived:
    v, arrived = rtb.p_servo(parol.fkine(parol.q), Teq, gain=1, threshold=0.01)
    J = parol.jacobe(parol.q)
    parol.qd = np.linalg.pinv(J) @ v
    env.step(dt)
arrived = False
Teq = parol.fkine(parol.q) * sm.SE3.Tx(0.2) * sm.SE3.Ty(-0.1)
axes = sg.Axes(length=0.1, pose=Teq)
env.add(axes)
while not arrived:
    v, arrived = rtb.p_servo(parol.fkine(parol.q), Teq, gain=1, threshold=0.01)
    J = parol.jacobe(parol.q)
    parol.qd = np.linalg.pinv(J) @ v
    env.step(dt)
arrived = False
Teq = parol.fkine(parol.q) * sm.SE3.Tx(0.2) * sm.SE3.Ty(0.1)
axes = sg.Axes(length=0.1, pose=Teq)
env.add(axes)
while not arrived:
    v, arrived = rtb.p_servo(parol.fkine(parol.q), Teq, gain=1, threshold=0.01)
    J = parol.jacobe(parol.q)
    parol.qd = np.linalg.pinv(J) @ v
    env.step(dt)
arrived = False
Teq = parol.fkine(parol.q) * sm.SE3.Tx(0.2) * sm.SE3.Ty(0.1)
axes = sg.Axes(length=0.1, pose=Teq)
env.add(axes)
while not arrived:
    v, arrived = rtb.p_servo(parol.fkine(parol.q), Teq, gain=1, threshold=0.01)
    J = parol.jacobe(parol.q)
    parol.qd = np.linalg.pinv(J) @ v
    env.step(dt)

env.hold()
