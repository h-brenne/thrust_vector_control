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
save_plot = True
folder = "logs/large_ccw/3/"
startup_time = 1.0
transient_duration = 0.2
# Load datasets
command_file = "logs/large_ccw/inverted/test_buffering_3.csv"
force_file = "logs/large_ccw/inverted/force_logs/test_buffering_3.csv"
inverted = True

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
non_zero_amplitude_indexes = np.where(amplitude_commands > 0)
high_amplitude_indexes = np.where(amplitude_commands > 0.15)

mag_slope, mag_slope2, mag_intercept = multiple_linear_regression(
    velocity_commands, amplitude_commands, force_magnitudes
)
amp_slope, _ = curve_fit(
    custom_linear_model,
    amplitude_commands[non_zero_amplitude_indexes],
    elevation_angles[non_zero_amplitude_indexes],
)

exponential_coeffs, _ = curve_fit(
    exponential_model, velocity_commands, force_magnitudes
)

torque_coefficent_z, _ = curve_fit(
    custom_linear_model, force_vectors[:, 2], torque_vectors[:, 2]
)
torque_coefficent_x, _ = curve_fit(
    custom_linear_model, force_vectors[:, 1], torque_vectors[:, 0]
)
# Plot relationship between force amplitude and elevation angle
x_amp = np.linspace(0, max(amplitude_commands), 100)
y_elev = custom_linear_model(x_amp, amp_slope)
plt.figure()
plt.scatter(
    amplitude_commands[non_zero_amplitude_indexes],
    elevation_angles[non_zero_amplitude_indexes],
    c=velocity_commands[non_zero_amplitude_indexes],
    cmap="viridis",
)
# Show coefficient in the label
label = "Linear fit: " + str(round(amp_slope[0], 3)) + "x"
plt.plot(x_amp, y_elev, color="red", linestyle="--", label=label)
plt.xlabel("Amplitude Command")
plt.ylabel("Elevation Angle [deg]")
plt.colorbar(label="Velocity Command [Hz]")
plt.legend()
plt.title("Elevation Angle vs Amplitude Command")
plt.grid()

# Plot relationship between velocity command and azimuth angle
plt.figure()
plt.scatter(
    velocity_commands[high_amplitude_indexes],
    azimuth_angles[high_amplitude_indexes],
    c=amplitude_commands[high_amplitude_indexes],
    cmap="viridis",
)
plt.xlabel("Velocity Command [Hz]")
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
label = "Exponential fit: " + str(round(exponential_coeffs[0], 4)) + "(x)^2"
plt.plot(x_vel, y_force_mag, color="red", linestyle="--", label=label)
plt.xlabel("Velocity Command [Hz]")
plt.ylabel("Force Magnitude [N]")
plt.colorbar(label="Amplitude Command")
plt.title("Force Magnitude vs Velocity Command and Amplitude Command")
plt.legend()
plt.grid()

# Plot relationship between force and torque
x_force_z = np.linspace(0, force_vectors[np.argmax(abs(force_vectors[:, 2])), 2], 100)
y_torque_z = custom_linear_model(x_force_z, torque_coefficent_z)

x_force_x = np.linspace(0, force_vectors[np.argmax(abs(force_vectors[:, 1])), 1], 100)
y_torque_x = custom_linear_model(x_force_x, torque_coefficent_x)
plt.figure()
plt.gca().invert_xaxis()
plt.gca().invert_yaxis()
plt.scatter(force_vectors[:, 2], torque_vectors[:, 2], label="Force Z, Torque Z", color="red")
plt.scatter(force_vectors[:, 1], torque_vectors[:, 0], label="Force Y, Torque X", color="green")
# Show coefficient in the label
label_z = "Linear fit: " + str(round(torque_coefficent_z[0], 4)) + "x"
plt.plot(x_force_z, y_torque_z, color="red", linestyle="--", label=label_z)
label_x = "Linear fit: " + str(round(torque_coefficent_x[0], 4)) + "x"
plt.plot(x_force_x, y_torque_x, color="green", linestyle="--", label=label_x)
plt.xlabel("Force [N]")
plt.ylabel("Torque [Nm]")
plt.title("Torque vs Force")
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
        plt.figure(i).savefig(folder + str(i) + '.png', bbox_inches="tight")

else:
    plt.show()
