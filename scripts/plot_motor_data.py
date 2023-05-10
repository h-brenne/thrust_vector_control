import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

command_file = "logs/flight_test.csv"
df = pd.read_csv(command_file)
df['datetime'] = pd.to_datetime(df['Time'])
position = df.columns.get_loc('datetime')
elapsed = df.iloc[1:, position] - df.iat[0, position]
df["seconds"] = elapsed.dt.total_seconds()
df.at[0, "seconds"] = 0.0
df.reset_index(drop=True, inplace=True)
command_df = df

# Split the data into unique motors using the 'ID' and 'Bus' fields
motor_data = {}
for _, row in command_df.iterrows():
    motor_key = (row['Bus'], row['ID'])
    if motor_key not in motor_data:
        motor_data[motor_key] = []
    motor_data[motor_key].append(row)

# Convert the lists of rows back to DataFrames
for key in motor_data:
    motor_data[key] = pd.DataFrame(motor_data[key])

# Calculate the time differences between consecutive data points for each motor
for key, motor_df in motor_data.items():
    motor_df['time_diff'] = motor_df['datetime'].diff().dt.total_seconds()

# Create a separate plot for each motor to visualize the distribution of time differences
for i, (key, motor_df) in enumerate(motor_data.items(), start=1):
    bus, motor_id = key
    plt.figure(i)
    plt.hist(motor_df["time_diff"][1:], bins=100)
    plt.xlabel("Elapsed Time [s]")
    plt.ylabel("Time Difference [s]")
    plt.title(f'''Motor {motor_id} on Bus {bus}: Time Difference Distribution
              Mean: {motor_df['time_diff'].mean():.5f} s, Std: {motor_df['time_diff'].std():.5f} s
              Min: {motor_df['time_diff'].min():.5f} s, Max: {motor_df['time_diff'].max():.5f} s''')
    plt.grid()
plt.show()
# Create a separate plot for 'ControlVelocity' and 'Velocity' for each motor
for i, (key, motor_df) in enumerate(motor_data.items(), start=1):
    bus, motor_id = key
    plt.figure(i)
    plt.subplot(2, 1, 1)
    plt.plot(motor_df["seconds"], motor_df["Velocity"], label="Velocity")
    plt.plot(motor_df["seconds"], motor_df["ControlVelocity"], label="Velocity setpoint")
    plt.ylabel("hz")
    plt.xlabel("Seconds")
    plt.title(f"Motor {motor_id} on Bus {bus}: Trajectory Tracking Performance")
    plt.legend()
    plt.grid()

    plt.subplot(2, 1, 2)
    plt.plot(motor_df["seconds"], motor_df["Torque"], label="Motor Driver Torque Nm")
    plt.ylabel("Torque [Nm]")
    plt.xlabel("Seconds")
    plt.title(f"Motor {motor_id} on Bus {bus}: Torque")
    plt.legend()
    plt.grid()


# Plot VelocityCommand for each motor in the same plot
plt.figure(len(motor_data) + 1)
for i, (key, motor_df) in enumerate(motor_data.items(), start=1):
    bus, motor_id = key
    plt.plot(motor_df["seconds"], motor_df["VelocityCommand"], label=f"Motor {motor_id} on Bus {bus}")
    plt.ylabel("Velocity [Hz]")
    plt.xlabel("Seconds")
    plt.title("Velocity Command")
    plt.legend()
    plt.grid()

# Plot AmplitudeCommand for each motor in the same plot
plt.figure(len(motor_data) + 2)
for i, (key, motor_df) in enumerate(motor_data.items(), start=1):
    bus, motor_id = key
    plt.plot(motor_df["seconds"], motor_df["AmplitudeCommand"], label=f"Motor {motor_id} on Bus {bus}")
    plt.ylabel("Amplitude")
    plt.xlabel("Seconds")
    plt.title("Amplitude Command")
    plt.legend()
    plt.grid()

# Plot PhaseCommand for each motor in the same plot
plt.figure(len(motor_data) + 3)
for i, (key, motor_df) in enumerate(motor_data.items(), start=1):
    bus, motor_id = key
    plt.plot(motor_df["seconds"], motor_df["PhaseCommand"]*180/np.pi, label=f"Motor {motor_id} on Bus {bus}")
    plt.ylabel("Phase [deg]")
    plt.xlabel("Seconds")
    plt.title("Phase Command")
    plt.legend()
    plt.grid()
plt.show()
