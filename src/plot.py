import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("joint_log.csv")

t = df.columns[0]
q1 = df.columns[1]
q2 = df.columns[2]
q1dot = df.columns[3]
q2dot  = df.columns[4]
q1ddot = df.columns[5]
q2ddot = df.columns[6]


df.plot(x=t, y=[q1,q2,q1dot,q2dot,q1ddot,q2ddot],style=['-', '-'], kind='line')
plt.legend()
plt.grid()
plt.isinteractive = False
plt.show()
