import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
df = pd.read_csv('logs/test.csv')
df['datetime'] = pd.to_datetime(df['Time'])

position = df.columns.get_loc('datetime')
elapsed = df.iloc[1:, position] - df.iat[0, position]
df["seconds"]=elapsed.dt.total_seconds()
df.at[0, "seconds"] = 0.0
print(np.diff(df["seconds"]))
print("Mean update rate:", 1/np.diff(df["seconds"]).mean())

title = "Test run"

plt.subplot(2,1,1)
plt.plot(df["seconds"], df["Velocity"], label="Velocity")
plt.plot(df["seconds"], df["ControlVelocity"], label="Velocity setpoint")
plt.ylabel("hz")
plt.legend()
plt.title(title)

plt.subplot(2,1,2)
plt.plot(df["seconds"], df["Torque"], label="Motor Driver Torque Nm")
plt.ylabel("Torque [Nm]")
plt.xlabel("Seconds")
plt.legend()

plt.show()