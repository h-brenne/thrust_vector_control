import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from analyze_thrust_vectoring import (
    process_dataset)
# Enable LaTeX style
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Palatino"],
    'font.size' : 18,
})

# Settings plot
save_plot = True
folder = "logs/large_rotor/sim/"

# Settings sim data
sim_file_80hz_leishman = "logs/large_rotor/sim/multi_amp60hz_leishman.csv"
#sim_file_60hz_leisman_small_a = "logs/large_rotor/sim/multi_amp60hz_leishman_small_a.csv"
sim_file_80hz_no_tip_losses = "logs/large_ccw/sim/multi_amp80hz_no_tip_losses.csv"
sim_file_80hz_stahlhut = "logs/large_ccw/sim/multi_amp80hz_stahlhut.csv"
sim_file_0amp = "logs/large_rotor/sim/multi_amp60hz_leishman_amp_0.csv"
sim_file_02amp = "logs/large_rotor/sim/multi_amp60hz_leishman_amp_0.2.csv"

# Settings real data
startup_time = 1.0
transient_duration = 0.2

#command_file = "logs/large_ccw/inverted/test_buffering_3.csv"
#force_file = "logs/large_ccw/inverted/force_logs/test_buffering_3.csv"
#inverted = True

command_file = "logs/large_rotor/normal/test_1.csv"
force_file = "logs/large_rotor/normal/force_logs/test_1.csv"

#inverted dataset
command_file_inverted = "logs/large_rotor/inverted/test_1.csv"
force_file_inverted = "logs/large_rotor/inverted/force_logs/test_1.csv"


# Load sim datasets

df = pd.read_csv(sim_file_80hz_leishman)
# Header: Input_Amplitude,Amp_Speed,Phase_Speed,Amp_Teeter,Phase_Teeter,Amp_Pos,Phase_Pos,Amp_Neg,Phase_Neg
sim_amplitude_commands = df["Input_Amplitude"].to_numpy()
sim_teetering_amplitude = df["Amp_Teeter"].to_numpy()
sim_teetering_phase = df["Phase_Teeter"].to_numpy()

df = pd.read_csv(sim_file_80hz_no_tip_losses)
# Header: Input_Amplitude,Amp_Speed,Phase_Speed,Amp_Teeter,Phase_Teeter,Amp_Pos,Phase_Pos,Amp_Neg,Phase_Neg
sim_amplitude_commands_no_tip_losses = df["Input_Amplitude"].to_numpy()
sim_teetering_amplitude_no_tip_losses = df["Amp_Teeter"].to_numpy()
sim_teetering_phase_no_tip_losses = df["Phase_Teeter"].to_numpy()

df = pd.read_csv(sim_file_80hz_stahlhut)
# Header: Input_Amplitude,Amp_Speed,Phase_Speed,Amp_Teeter,Phase_Teeter,Amp_Pos,Phase_Pos,Amp_Neg,Phase_Neg
sim_amplitude_commands_stahlhut = df["Input_Amplitude"].to_numpy()
sim_teetering_amplitude_stahlhut = df["Amp_Teeter"].to_numpy()
sim_teetering_phase_stahlhut = df["Phase_Teeter"].to_numpy()

df = pd.read_csv(sim_file_0amp)
# Header: Motor_Angle,Motor_Speed,Motor_Acceleration,Teetering_Angle,Side_Hub_Positive_Angle,Side_Hub_Negative_Angle
# remove half of the data 
df = df.iloc[::2, :]
sim_motor_angle_0a = df["Motor_Angle"].to_numpy()
sim_motor_speed_0a = df["Motor_Speed"].to_numpy()
sim_motor_acceleration_0a = np.gradient(sim_motor_speed_0a)
sim_teetering_angle_0a = df["Teetering_Angle"].to_numpy()
sim_side_hub_pos_0a = df["Side_Hub_Positive_Angle"].to_numpy()
sim_side_hub_neg_0a = df["Side_Hub_Negative_Angle"].to_numpy()

df = pd.read_csv(sim_file_02amp)
# Header: Motor_Angle,Motor_Speed,Motor_Acceleration,Teetering_Angle,Side_Hub_Positive_Angle,Side_Hub_Negative_Angle
# remove half of the data 
df = df.iloc[::2, :]

sim_motor_angle_02a = df["Motor_Angle"].to_numpy()
sim_motor_speed_02a = df["Motor_Speed"].to_numpy()
sim_motor_acceleration_02a = df["Motor_Acceleration"].to_numpy()
sim_teetering_angle_02a = df["Teetering_Angle"].to_numpy()
sim_side_hub_pos_02a = df["Side_Hub_Positive_Angle"].to_numpy()
sim_side_hub_neg_02a = df["Side_Hub_Negative_Angle"].to_numpy()

#Flip motor angle to correspond to azimuth angle
#sim_motor_angle_0a = 360 - sim_motor_angle_0a
#sim_motor_angle_02a = 360 - sim_motor_angle_02a


# Sim manual data
sim_force_force = np.array([2.5355, 4.2698, 6.5576, 9.3645, 12.6181, 16.1918])
sim_force_torque = np.array([0.0456, 0.0713, 0.1020, 0.1372, 0.1758, 0.2166])
sim_force_rpm = np.array([40, 50, 60, 70, 80, 90])*60 # Convert to rpm

sim_stahlhut_force_force = np.array([2.1998, 3.6301, 5.4696, 7.527, 10.5325, 13.8461])
sim_stahlhut_force_torque = np.array([0.0437, 0.0684, 0.0986, 0.1357, 0.1803, 0.2350])
sim_stahlhut_force_rpm = np.array([40, 50, 60, 70, 80, 90])*60 # Convert to rpm



# Load experimenal datasets
(
    amplitude_commands,
    phase_commands,
    velocity_commands,
    elevation_angles,
    azimuth_angles,
    force_vectors,
    torque_vectors,
    command_df,
    force_df,
    unique_command_timestamps,
    force_timesteps_downsampled,
) = process_dataset(command_file, force_file, startup_time, transient_duration, False)
force_magnitudes = np.linalg.norm(force_vectors, axis=1)
torque_magnitudes = np.linalg.norm(torque_vectors, axis=1)

# load inverted dataset
(
    amplitude_commands_inv,
    phase_commands_inv,
    velocity_commands_inv,
    elevation_angles_inv,
    azimuth_angles_inv,
    force_vectors_inv,
    torque_vectors_inv,
    command_df_inv,
    force_df_inv,
    unique_command_timestamps_inv,
    force_timesteps_downsampled_inv,
) = process_dataset(command_file_inverted, force_file_inverted, startup_time, transient_duration, True)
force_magnitudes_inv = np.linalg.norm(force_vectors_inv, axis=1)
torque_magnitudes_inv = np.linalg.norm(torque_vectors_inv, axis=1)

# Find indexes of non-zero amplitude commands at a certain velocity_command
index_80_hz =  np.where((velocity_commands == 60) & (amplitude_commands > 0))


zero_amp_indexes = []
for i in range(40, 100, 10):
    zero_amp_indexes.append(np.where((velocity_commands == i) & (amplitude_commands == 0))[0])
zero_amp_indexes = np.concatenate(zero_amp_indexes)

zero_amp_indexes_inv = []
for i in range(40, 100, 10):
    zero_amp_indexes_inv.append(np.where((velocity_commands_inv == i) & (amplitude_commands_inv == 0))[0])
zero_amp_indexes_inv = np.concatenate(zero_amp_indexes_inv)

plt.figure(figsize=(10, 7))
#plt.title("Thrust Vectoring Elevation at $\Omega$ = 4800 rpm")
plt.scatter(
    amplitude_commands[index_80_hz],
    elevation_angles[index_80_hz],
    label="Measured: Thrust vector elevation angle",
    marker="x",
    s=200,
    linewidths=4,
)
# plt.scatter(
#     amplitude_commands_inv[index_80_hz],
#     elevation_angles_inv[index_80_hz],
#     label="Measured: Thrust vector elevation angle inverted rotor",
#     marker="x",
#     s=200,
#     linewidths=4,
# )

plt.scatter(
    sim_amplitude_commands_stahlhut,
    sim_teetering_amplitude_stahlhut,
    label = "BEMT: Maximum Teetering angle",
    marker="o",
    facecolors="none",
    edgecolors="C2",
    s=200,
    linewidths=4,
)

plt.scatter(
    sim_amplitude_commands,
    sim_teetering_amplitude,
    label = "Small angles approximation BEMT: Maximum Teetering angle",
    marker="v",
    facecolors="none",
    edgecolors="C3",
    s=200,
    linewidths=4,
)
plt.legend(bbox_to_anchor=(-0.1,1.02,1,0.2), loc="lower left")
plt.xlabel("Amplitude command")
plt.ylabel("Angle [deg]")
plt.grid()

plt.figure(figsize=(10, 7))
#plt.title("Thrust Vectoring Azimuth at $\Omega$ = 4800 rpm")
plt.scatter(
    amplitude_commands[index_80_hz],
    azimuth_angles[index_80_hz],
    label="Measured: Thrust vector azimuth angle",
    marker="x",
    s=200,
    linewidths=4,
)

# plt.scatter(
#     amplitude_commands_inv[index_80_hz],
#     azimuth_angles_inv[index_80_hz],  # wrong frame of reference
#     label="Measured: Thrust vector azimuth angle inverted rotor",
#     marker="x",
#     s=200,
#     linewidths=4,
# )
plt.scatter(
    sim_amplitude_commands_stahlhut,
    -sim_teetering_phase_stahlhut,
    label = "BEMT: Azimuth of Maximum Teetering angle",
    marker="o",
    facecolors="none",
    edgecolors="C2",
    s=200,
    linewidths=4,
)
plt.scatter(
    sim_amplitude_commands,
    -sim_teetering_phase,
    label = "Small angles approximation BEMT: Azimuth of Maximum Teetering angle",
    marker="v",
    facecolors="none",
    edgecolors="C3",
    s=200,
    linewidths=4,
)

plt.legend(bbox_to_anchor=(-0.1,1.02,1,0.2), loc="lower left")
plt.xlabel("Amplitude command")
plt.ylabel("Angle [deg]")
plt.yticks(np.arange(-120, 1, 30))
plt.ylim(-120, 1)
plt.grid()

'''plt.figure(figsize=(9, 6))
#plt.title("Simulated Motor Speed")
plt.scatter(
    sim_motor_angle_0a,
    sim_motor_speed_0a*60/(2*np.pi),
    label="A = 0.0",
    marker="o",
    s=10
)
plt.scatter(
    sim_motor_angle_02a,
    sim_motor_speed_02a*60/(2*np.pi),
    label="A = 0.2",
    marker="x",
    linewidths=1,
    s=50
)

plt.scatter(
    sim_motor_angle_02a,
    sim_motor_acceleration_02a,
    label="Acceleration: A = 0.2",
    marker="x",
    linewidths=1,
    s=50
)
plt.legend()
plt.xlabel("Motor angle [deg]")
plt.ylabel("Motor speed [rpm]")
plt.xlim(0, 360)
plt.xticks(np.arange(0, 361, 45))
plt.grid()'''

fig, ax1 = plt.subplots(figsize=(9, 6))

# First y-axis
#color = 'tab:blue'
ax1.set_xlabel('Motor angle [deg]')
ax1.set_ylabel('Motor speed [rpm]')
ax1.scatter(sim_motor_angle_0a, sim_motor_speed_0a*60/(2*np.pi), label="Speed: A = 0.0", marker="o", s=10, color="C0")
ax1.scatter(sim_motor_angle_02a, sim_motor_speed_02a*60/(2*np.pi), label="Speed: A = 0.2", marker="x", linewidths=1, s=50, color="C1")
ax1.set_xlim(0, 360)
ax1.set_xticks(np.arange(0, 361, 45))
ax1.grid()

# instantiate a second axes that shares the same x-axis
ax2 = ax1.twinx()  

# Second y-axis
#color = 'tab:red'
ax2.set_ylabel('Motor acceleration [rad/s$^2$]')  # we already handled the x-label with ax1
ax2.scatter(sim_motor_angle_02a, sim_motor_acceleration_02a, label="Acceleration: A = 0.2", marker="x", linewidths=1, s=50, color="C2")

# Display legend for both axes
fig.legend(loc="upper right", bbox_to_anchor=(1,1), bbox_transform=ax1.transAxes)

plt.figure(figsize=(9, 6))
#plt.title("Simulated Teetering Angle")
plt.scatter(
    sim_motor_angle_0a,
    sim_teetering_angle_0a,
    label="A = 0.0",
    marker="o",
    s=10
)
plt.scatter(
    sim_motor_angle_02a,
    sim_teetering_angle_02a,
    label="A = 0.2",
    marker="x",
    linewidths=1,
    s=50
)
plt.legend()
plt.xlabel("Motor angle [deg]")
plt.ylabel("Teetering angle [deg]")
plt.xlim(0, 360)
plt.xticks(np.arange(0, 361, 45))
plt.grid()

plt.figure(figsize=(9, 6))
#plt.title("Simulated skewed lag-pitch angle (positive)")
plt.scatter(
    sim_motor_angle_0a,
    sim_side_hub_pos_0a,
    label="A = 0.0",
    marker="o",
    s=10
)
plt.scatter(
    sim_motor_angle_02a,
    sim_side_hub_pos_02a,
    label="A = 0.2",
    marker="x",
    linewidths=1,
    s=50
)
plt.legend()
plt.xlabel("Motor angle [deg]")
plt.ylabel("Skewed lag-pitch angle [deg]")
plt.xlim(0, 360)
plt.xticks(np.arange(0, 361, 45))
plt.grid()

plt.figure(figsize=(9, 6))
#plt.title("Simulated skewed lag-pitch angle (negative)")
plt.scatter(
    sim_motor_angle_0a,
    sim_side_hub_neg_0a,
    label="A = 0.0",
    marker="o",
    s=10
)
plt.scatter(
    sim_motor_angle_02a,
    sim_side_hub_neg_02a,
    label="A = 0.2",
    marker="x",
    linewidths=1,
    s=50
)
plt.legend()
plt.xlabel("Motor angle [deg]")
plt.ylabel("skewed lag-pitch angle [deg]")
plt.xlim(0, 360)
plt.xticks(np.arange(0, 361, 45))
plt.grid()


fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(13, 5))
ax[0].scatter(
    sim_force_rpm,
    sim_force_force,
    label="Small angles approximation BEMT",
    marker="v",
    facecolors="none",
    edgecolors="C3",
    s=100,
    linewidths=2,
)

ax[0].scatter(
    sim_stahlhut_force_rpm,
    sim_stahlhut_force_force,
    label="BEMT",
    marker="o",
    facecolors="none",
    edgecolors="C2",
    s=100,
    linewidths=2,
)


ax[0].scatter(
    velocity_commands[zero_amp_indexes]*60,
    force_magnitudes[zero_amp_indexes],
    label="Measured rotor",
    marker="x",
    linewidths=2,
    s=100,
)

ax[0].scatter(
    velocity_commands_inv[zero_amp_indexes_inv]*60,
    force_magnitudes_inv[zero_amp_indexes_inv],
    label="Measured inverted rotor",
    marker="x",
    linewidths=2,
    s=100,
)

ax[1].scatter(
    sim_force_rpm,
    sim_force_torque,
    label="Small angles approximation BEMT",
    marker="v",
    facecolors="none",
    edgecolors="C3",
    s=100,
    linewidths=2,
)

ax[1].scatter(
    sim_stahlhut_force_rpm,
    sim_stahlhut_force_torque,
    label="BEMT",
    marker="o",
    facecolors="none",
    edgecolors="C2",
    s=100,
    linewidths=2,
)

ax[1].scatter(
    velocity_commands[zero_amp_indexes]*60,
    torque_magnitudes[zero_amp_indexes],
    label="Measured rotor",
    marker="x",
    linewidths=2,
    s=100,
)

ax[1].scatter(
    velocity_commands_inv[zero_amp_indexes_inv]*60,
    torque_magnitudes_inv[zero_amp_indexes_inv],
    label="Measured inverted rotor",
    marker="x",
    linewidths=2,
    s=100,
)
# set x and y labels for the first subplot
ax[0].set_xlabel("Speed [rpm]")
ax[0].set_ylabel("Force [N]")
ax[0].set_xticks(sim_force_rpm)
ax[0].grid()
# set x and y labels for the second subplot
ax[1].set_xlabel("Speed [rpm]")
ax[1].set_ylabel("Torque [Nm]")
ax[1].set_xticks(sim_force_rpm)
ax[1].grid()
# get handles and labels for legend
handles, labels = ax[0].get_legend_handles_labels()

# create a legend for the whole figure
fig.legend(handles, labels, bbox_to_anchor=(0.35,0.9,0,0.2), loc="lower center", mode="expand", borderaxespad=0, ncol=1)




# plt.figure(figsize=(7, 6))
# #plt.title("Force vs RPM: Simulation and Measured for Large Rotor")
# plt.scatter(
#     sim_force_rpm,
#     sim_force_force,
#     label="Linear BEMT",
#     marker="v",
#     facecolors="none",
#     edgecolors="C3",
#     s=100,
#     linewidths=2,
# )

# plt.scatter(
#     sim_stahlhut_force_rpm,
#     sim_stahlhut_force_force,
#     label="Non-linear BEMT",
#     marker="o",
#     facecolors="none",
#     edgecolors="C2",
#     s=100,
#     linewidths=2,
# )


# plt.scatter(
#     velocity_commands[zero_amp_indexes]*60,
#     force_magnitudes[zero_amp_indexes],
#     label="Measured rotor",
#     marker="x",
#     linewidths=2,
#     s=100,
# )

# plt.scatter(
#     velocity_commands_inv[zero_amp_indexes_inv]*60,
#     force_magnitudes_inv[zero_amp_indexes_inv],
#     label="Measured inverted rotor",
#     marker="x",
#     linewidths=2,
#     s=100,
# )
# plt.legend()
# plt.xlabel("Speed [rpm]")
# plt.ylabel("Force [N]")
# plt.xticks(sim_force_rpm)
# plt.grid()

# plt.figure(figsize=(7, 6))
# #plt.title("Torque vs RPM: Simulation and Measured for Large Rotor")
# plt.scatter(
#     sim_force_rpm,
#     sim_force_torque,
#     label="Linear BEMT",
#     marker="v",
#     facecolors="none",
#     edgecolors="C3",
#     s=100,
#     linewidths=2,
# )

# plt.scatter(
#     sim_stahlhut_force_rpm,
#     sim_stahlhut_force_torque,
#     label="Non-linear BEMT",
#     marker="o",
#     facecolors="none",
#     edgecolors="C2",
#     s=100,
#     linewidths=2,
# )

# plt.scatter(
#     velocity_commands[zero_amp_indexes]*60,
#     torque_magnitudes[zero_amp_indexes],
#     label="Measured rotor",
#     marker="x",
#     linewidths=2,
#     s=100,
# )

# plt.scatter(
#     velocity_commands_inv[zero_amp_indexes_inv]*60,
#     torque_magnitudes_inv[zero_amp_indexes_inv],
#     label="Measured inverted rotor",
#     marker="x",
#     linewidths=2,
#     s=100,
# )

# plt.legend()
# plt.xlabel("Speed [rpm]")
# plt.ylabel("Torque [Nm]")
# plt.xticks(sim_force_rpm)
# plt.grid()


# Small rotor data set

command_file = "logs/small_rotor/normal/test_1.csv"
force_file = "logs/small_rotor/normal/force_logs/test_1.csv"

#inverted dataset
command_file_inverted = "logs/small_rotor/inverted/test_1.csv"
force_file_inverted = "logs/small_rotor/inverted/force_logs/test_1.csv"


sim_small_rotor_force_force = np.array([2.1966, 3.6797, 5.6600, 8.1252, 11.0345])#, 14.3122])
sim_small_rotor_force_torque = np.array([0.0384, 0.0601, 0.0861, 0.1162, 0.1496])#, 0.1853])
sim_small_rotor_force_rpm = np.array([40, 50, 60, 70, 80])*60#, 90])*60 # Convert to rpm

sim_small_rotor_stahlhut_force_force = np.array([1.8904, 3.1161, 4.6947, 6.6597, 9.0618])#, 11.9512])
sim_small_rotor_stahlhut_force_torque = np.array([0.0366, 0.0574, 0.0827, 0.1134, 0.1506])#, 0.1961])
sim_small_rotor_stahlhut_force_rpm = np.array([40, 50, 60, 70, 80])*60#, 90])*60 # Convert to rpm

# Load experimenal datasets
(
    amplitude_commands,
    phase_commands,
    velocity_commands,
    elevation_angles,
    azimuth_angles,
    force_vectors,
    torque_vectors,
    command_df,
    force_df,
    unique_command_timestamps,
    force_timesteps_downsampled,
) = process_dataset(command_file, force_file, startup_time, transient_duration, False)
force_magnitudes = np.linalg.norm(force_vectors, axis=1)
torque_magnitudes = np.linalg.norm(torque_vectors, axis=1)

# load inverted dataset
(
    amplitude_commands_inv,
    phase_commands_inv,
    velocity_commands_inv,
    elevation_angles_inv,
    azimuth_angles_inv,
    force_vectors_inv,
    torque_vectors_inv,
    command_df_inv,
    force_df_inv,
    unique_command_timestamps_inv,
    force_timesteps_downsampled_inv,
) = process_dataset(command_file_inverted, force_file_inverted, startup_time, transient_duration, True)
force_magnitudes_inv = np.linalg.norm(force_vectors_inv, axis=1)
torque_magnitudes_inv = np.linalg.norm(torque_vectors_inv, axis=1)

zero_amp_indexes = []
for i in range(40, 100, 10):
    zero_amp_indexes.append(np.where((velocity_commands == i) & (amplitude_commands == 0))[0])
zero_amp_indexes = np.concatenate(zero_amp_indexes)

zero_amp_indexes_inv = []
for i in range(40, 100, 10):
    zero_amp_indexes_inv.append(np.where((velocity_commands_inv == i) & (amplitude_commands_inv == 0))[0])
zero_amp_indexes_inv = np.concatenate(zero_amp_indexes_inv)

plt.figure(figsize=(12, 6))
#plt.title("Force vs RPM: Simulation and Measured for Small Rotor")
plt.scatter(
    sim_small_rotor_force_rpm,
    sim_small_rotor_force_force,
    label="Small angles approximation BEMT",
    marker="v",
    facecolors="none",
    edgecolors="C3",
    s=100,
    linewidths=2,
)

plt.scatter(
    sim_small_rotor_stahlhut_force_rpm,
    sim_small_rotor_stahlhut_force_force,
    label="BEMT",
    marker="o",
    facecolors="none",
    edgecolors="C2",
    s=100,
    linewidths=2,
)


plt.scatter(
    velocity_commands[zero_amp_indexes]*60,
    force_magnitudes[zero_amp_indexes],
    label="Measured rotor",
    marker="x",
    linewidths=2,
    s=100,
)

plt.scatter(
    velocity_commands_inv[zero_amp_indexes_inv]*60,
    force_magnitudes_inv[zero_amp_indexes_inv],
    label="Measured inverted rotor",
    marker="x",
    linewidths=2,
    s=100,
)
plt.legend()
plt.xlabel("Speed [rpm]")
plt.ylabel("Force [N]")
plt.xticks(sim_small_rotor_force_rpm)
plt.grid()


fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(13, 5))
ax[0].scatter(
    sim_small_rotor_force_rpm,
    sim_small_rotor_force_force,
    label="Small angles approximation BEMT",
    marker="v",
    facecolors="none",
    edgecolors="C3",
    s=100,
    linewidths=2,
)

ax[0].scatter(
    sim_small_rotor_stahlhut_force_rpm,
    sim_small_rotor_stahlhut_force_force,
    label="BEMT",
    marker="o",
    facecolors="none",
    edgecolors="C2",
    s=100,
    linewidths=2,
)


ax[0].scatter(
    velocity_commands[zero_amp_indexes]*60,
    force_magnitudes[zero_amp_indexes],
    label="Measured rotor",
    marker="x",
    linewidths=2,
    s=100,
)

ax[0].scatter(
    velocity_commands_inv[zero_amp_indexes_inv]*60,
    force_magnitudes_inv[zero_amp_indexes_inv],
    label="Measured inverted rotor",
    marker="x",
    linewidths=2,
    s=100,
)

ax[1].scatter(
    sim_small_rotor_force_rpm,
    sim_small_rotor_force_torque,
    label="Small angles approximation BEMT",
    marker="v",
    facecolors="none",
    edgecolors="C3",
    s=100,
    linewidths=2,
)

ax[1].scatter(
    sim_small_rotor_stahlhut_force_rpm,
    sim_small_rotor_stahlhut_force_torque,
    label="BEMT",
    marker="o",
    facecolors="none",
    edgecolors="C2",
    s=100,
    linewidths=2,
)

ax[1].scatter(
    velocity_commands[zero_amp_indexes]*60,
    torque_magnitudes[zero_amp_indexes],
    label="Measured rotor",
    marker="x",
    linewidths=2,
    s=100,
)

ax[1].scatter(
    velocity_commands_inv[zero_amp_indexes_inv]*60,
    torque_magnitudes_inv[zero_amp_indexes_inv],
    label="Measured inverted rotor",
    marker="x",
    linewidths=2,
    s=100,
)
# set x and y labels for the first subplot
ax[0].set_xlabel("Speed [rpm]")
ax[0].set_ylabel("Force [N]")
ax[0].set_xticks(sim_force_rpm)
ax[0].grid()
# set x and y labels for the second subplot
ax[1].set_xlabel("Speed [rpm]")
ax[1].set_ylabel("Torque [Nm]")
ax[1].set_xticks(sim_force_rpm)
ax[1].grid()
# get handles and labels for legend
handles, labels = ax[0].get_legend_handles_labels()

# create a legend for the whole figure
fig.legend(handles, labels, bbox_to_anchor=(0.35,1.0,0,0.2), loc="lower center", mode="expand", borderaxespad=0, ncol=1)


plt.tight_layout()
if save_plot:
    for i in plt.get_fignums():
        plt.figure(i).savefig(folder + str(i) + '.pdf', bbox_inches="tight")
        plt.figure(i).savefig(folder + str(i) + '.png', bbox_inches="tight")

else:
    plt.show()
