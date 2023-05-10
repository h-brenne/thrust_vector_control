import pandas as pd
import matplotlib.pyplot as plt

command_file = "logs/test.csv"
df = pd.read_csv(command_file)
df['datetime'] = pd.to_datetime(df['Time'])
position = df.columns.get_loc('datetime')
elapsed = df.iloc[1:, position] - df.iat[0, position]
df["seconds"] = elapsed.dt.total_seconds()
df.at[0, "seconds"] = 0.0
df.reset_index(drop=True, inplace=True)
command_df = df

if "Velocity" in command_df.columns:
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(command_df["seconds"], command_df["Velocity"], label="Velocity")
    plt.plot(command_df["seconds"], command_df["ControlVelocity"], label="Velocity setpoint")
    plt.ylabel("hz")
    plt.legend()
    plt.title("Trajectory Tracking Performance")
    plt.grid()

    plt.subplot(2, 1, 2)
    plt.plot(command_df["seconds"], command_df["Torque"], label="Motor Driver Torque Nm")
    plt.ylabel("Torque [Nm]")
    plt.xlabel("Seconds")
    plt.legend()

    plt.grid()

plt.show()
