import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("joint_log.csv")

t = df.columns[0]
q = df.columns[1]
qdot = df.columns[2]
qddot = df.columns[3]

df.plot(x=t, y=[q,qdot,qddot],style=['-', '-'], kind='line')
plt.legend()
plt.grid()
plt.isinteractive = False
plt.show()
