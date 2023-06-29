import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from analyze_thrust_vectoring import (
    process_dataset,
    multiple_linear_regression,
    custom_linear_model,
    exponential_model,
)

# Settings
save_plot = False
folder = "logs/large_rotor_torqueff/"
startup_time = 1.0
transient_duration = 0.2
# Load datasets
command_file = "logs/large_rotor_torqueff/test5.csv"
force_file = "logs/large_rotor_torqueff/force_logs/test5.csv"
inverted = False

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
) = process_dataset(command_file, force_file, startup_time, transient_duration, inverted)
force_magnitudes = np.linalg.norm(force_vectors, axis=1)
torque_magnitudes = np.linalg.norm(torque_vectors, axis=1)

# Print commands
print("Amplitude commands: ", amplitude_commands)
print("Phase commands: ", phase_commands)
print("Velocity commands: ", velocity_commands)
print("Elevation angles: ", elevation_angles)
print("Azimuth angles: ", azimuth_angles)
print("Force magnitudes: ", force_magnitudes)

print(
    "force time duration: ", force_df["seconds"].iloc[-1] - force_df["seconds"].iloc[0]
)
print(
    "command time duration: ",
    command_df["seconds"].iloc[-1] - command_df["seconds"].iloc[0],
)
# Thrust vector model
# Calculate the regression coefficients from the combined data

# Mask out the data points where the velocity is 50 (Bad data)
# Multiply by 3 due to the torque ff scale ussed
amplitude_commands = 3*amplitude_commands[np.where(velocity_commands != 50)]
elevation_angles = elevation_angles[np.where(velocity_commands != 50)]
force_magnitudes = force_magnitudes[np.where(velocity_commands != 50)]
velocity_commands = velocity_commands[np.where(velocity_commands != 50)]
azimuth_angles = azimuth_angles[np.where(velocity_commands != 50)]
phase_commands = phase_commands[np.where(velocity_commands != 50)]
force_vectors = force_vectors[np.where(velocity_commands != 50)]
torque_vectors = torque_vectors[np.where(velocity_commands != 50)]


non_zero_amplitude_indexes = np.where(amplitude_commands > 0)
high_amplitude_indexes = np.where(amplitude_commands > 0.15)

mag_slope, mag_slope2, mag_intercept = multiple_linear_regression(
    velocity_commands, amplitude_commands, force_magnitudes
)

# Generate a seperate model for each velocity command
# Find number of unique velocity commands
unique_velocity_commands = np.unique(velocity_commands)
amp_slopes = np.zeros(len(unique_velocity_commands))
for i in range(len(amp_slopes)):
    amp_slopes[i], _ = curve_fit(
        custom_linear_model,
        amplitude_commands[np.where(non_zero_amplitude_indexes and velocity_commands == unique_velocity_commands[i])],
        elevation_angles[np.where(non_zero_amplitude_indexes and velocity_commands == unique_velocity_commands[i])],
    )
amp_slope, _ = curve_fit(
    custom_linear_model,
    amplitude_commands[non_zero_amplitude_indexes],
    elevation_angles[non_zero_amplitude_indexes],
)

exponential_coeffs, _ = curve_fit(
    exponential_model, velocity_commands, force_magnitudes
)

torque_coefficent, _ = curve_fit(
    custom_linear_model, force_vectors[:, 2], torque_vectors[:, 2]
)


# Plot relationship between force amplitude and elevation angle
plt.figure(figsize=(5, 2))
plt.scatter(
    amplitude_commands[non_zero_amplitude_indexes],
    elevation_angles[non_zero_amplitude_indexes],
    c=velocity_commands[non_zero_amplitude_indexes],
    cmap="viridis",
)
for i in range(len(amp_slopes)):
    x_amp = np.linspace(0, max(amplitude_commands), 100)
    y_elev = custom_linear_model(x_amp, amp_slopes[i])
    label = "Linear fit: " + str(round(amp_slopes[i], 3)) + "x"
    plt.plot(x_amp, custom_linear_model(x_amp, amp_slopes[i]), color="red", linestyle="--", linewidth=1, label=label)
plt.xlabel("Torque Feedforward Amplitude Command [Nm]")
plt.ylabel("Elevation Angle [deg]")
plt.colorbar(label="Velocity Command [Hz]")
#plt.legend()
#plt.title("Elevation Angle vs Amplitude Command")
plt.grid()

# Plot relationship between amp_slope and velocity
plt.figure()
plt.scatter(unique_velocity_commands, amp_slopes)
plt.xlabel("Velocity Command")
plt.ylabel("Amplitude Slope")
plt.title("Amplitude Slope vs Velocity Command")
plt.grid()

# Plot relationship between velocity command and azimuth angle
plt.figure()
plt.scatter(
    velocity_commands[high_amplitude_indexes],
    azimuth_angles[high_amplitude_indexes],
    c=amplitude_commands[high_amplitude_indexes],
    cmap="viridis",
)
plt.xlabel("Velocity Command")
plt.ylabel("Azimuth Angle [deg]")
plt.colorbar(label="Amplitude Command")
plt.legend()
plt.title("Azimuth Angle vs Velocity Command")
plt.grid()

# Plot relationship between force magnitude and velocity command and amplitude command
x_vel = np.linspace(
    min(velocity_commands), max(velocity_commands), 100
)
y_force_mag = exponential_model(x_vel, *exponential_coeffs)
plt.figure()
plt.scatter(velocity_commands, force_magnitudes, c=amplitude_commands, cmap="viridis")


# Show the exponential coefficients in the label
label = "Exponential fit: " + str(round(exponential_coeffs[0], 4)) + "^(x)"
plt.plot(x_vel, y_force_mag, color="red", linestyle="--", label=label)
plt.xlabel("Velocity Command")
plt.ylabel("Force Magnitude")
plt.colorbar(label="Amplitude Command")
plt.title("Force Magnitude vs Velocity Command and Amplitude Command")
plt.legend()
plt.grid()

# Plot relationship between force_magnitude and torque_vector magnitude
x_force = np.linspace(0, max(force_magnitudes), 100)
y_torque = custom_linear_model(x_force, torque_coefficent)
plt.figure()
plt.scatter(abs(force_vectors[:, 2]), abs(torque_vectors[:, 2]), c=amplitude_commands, cmap="viridis")
# Show coefficient in the label
label = "Linear fit: " + str(round(torque_coefficent[0], 4)) + "x"
plt.plot(x_force, y_torque, color="red", linestyle="--", label=label)
plt.xlabel("Force Magnitude")
plt.ylabel("Torque Magnitude")
plt.colorbar(label="Amplitude Command")
plt.title("Torque Magnitude vs Force Magnitude")
plt.legend()
plt.grid()

# Plot force data
plt.figure()
plt.subplot(2, 1, 1)
force_magnitude_full = np.linalg.norm(
    force_df[["Force X (N)", "Force Y (N)", "Force Z (N)"]], axis=1
)
plt.plot(force_df["seconds"], force_magnitude_full, label="Thrust magnitude")
plt.plot(force_df["seconds"], force_df["Force X (N)"], label="Force X (N)")
plt.plot(force_df["seconds"], force_df["Force Y (N)"], label="Force Y (N)")
plt.plot(force_df["seconds"], force_df["Force Z (N)"], label="Force Z (N)")
plt.ylabel("Force [N]")

# Draw vertical lines indicating the start of each time section that is averaged
for t in unique_command_timestamps:
    plt.axvline(x=t, color="gray", linestyle="--", alpha=0.5)
for t in force_timesteps_downsampled:
    plt.axvline(x=t, color="red", linestyle="--", alpha=0.2, linewidth=1)

plt.grid()
plt.legend()
plt.subplot(2, 1, 2)
plt.plot(force_df["seconds"], force_df["Torque X (N-m)"], label="Torque X (N-m)")
plt.plot(force_df["seconds"], force_df["Torque Y (N-m)"], label="Torque Y (N-m)")
plt.plot(force_df["seconds"], force_df["Torque Z (N-m)"], label="Torque Z (N-m)")
plt.ylabel("Torque [Nm]")
plt.xlabel("Seconds")
plt.grid()
plt.legend()

if "Velocity" in command_df.columns:
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(command_df["seconds"], command_df["Velocity"], label="Velocity")
    plt.plot(
        command_df["seconds"], command_df["ControlVelocity"], label="Velocity setpoint"
    )
    plt.ylabel("hz")
    plt.legend()
    plt.title("Trajectory Tracking Performance")
    plt.grid()

    plt.subplot(2, 1, 2)
    plt.plot(
        command_df["seconds"], command_df["Torque"], label="Motor Driver Torque Nm"
    )
    plt.ylabel("Torque [Nm]")
    plt.xlabel("Seconds")
    plt.legend()

    plt.grid()

plt.tight_layout()
if save_plot:
    for i in plt.get_fignums():
        plt.figure(i).savefig(folder + str(i) + '.pdf', bbox_inches="tight")
else:
    plt.show()
