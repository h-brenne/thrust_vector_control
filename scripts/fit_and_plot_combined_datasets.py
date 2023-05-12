import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

from analyze_thrust_vectoring import (
    process_dataset,
    multiple_linear_regression,
    custom_linear_model,
    quadratic_model,
    exponential_model,
)
save_plot = True
folder = "logs/large_ccw/inverted/"
# Settings
startup_time = 1.0
transient_duration = 0.2
inverted = True
# Load datasets
datasets = [
    # Non-inverted datasets
    # ("logs/large_ccw/3/calib3.csv", "logs/large_ccw/3/force_logs/calib3.csv"),
    # ("logs/large_ccw/3/calib4.csv", "logs/large_ccw/3/force_logs/calib4.csv"),
    # ("logs/large_ccw/2/test1.csv", "logs/large_ccw/2/force_logs/test1.csv"),
    # Inverted datasets
    ("logs/large_ccw/inverted/calib2.csv", "logs/large_ccw/inverted/force_logs/calib2.csv"),
    ("logs/large_ccw/inverted/calib3.csv", "logs/large_ccw/inverted/force_logs/calib3.csv"),
    ("logs/large_ccw/inverted/test_cw_1.csv", "logs/large_ccw/inverted/force_logs/test_cw_1.csv"),
]


# Process datasets and combine the data
combined_amplitude_commands = []
combined_phase_commands = []
combined_velocity_commands = []
combined_elevation_angles = []
combined_azimuth_angles = []
combined_force_magnitudes = []
combined_unique_command_timestamps = []
combined_force_timesteps_downsampled = []
combined_force_vectors = []
combined_torque_vectors = []

for idx, (command_file, force_file) in enumerate(datasets):
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
    combined_amplitude_commands.extend(amplitude_commands)
    combined_phase_commands.extend(phase_commands)
    combined_velocity_commands.extend(velocity_commands)
    combined_elevation_angles.extend(elevation_angles)
    combined_azimuth_angles.extend(azimuth_angles)
    combined_force_magnitudes.extend(force_magnitudes)
    combined_force_vectors.extend(force_vectors)
    combined_unique_command_timestamps.extend(unique_command_timestamps)
    combined_force_timesteps_downsampled.extend(force_timesteps_downsampled)
    combined_torque_vectors.extend(torque_vectors)

# Convert combined data to NumPy arrays
combined_amplitude_commands = np.array(combined_amplitude_commands)
combined_phase_commands = np.array(combined_phase_commands)
combined_velocity_commands = np.array(combined_velocity_commands)
combined_elevation_angles = np.array(combined_elevation_angles)
combined_azimuth_angles = np.array(combined_azimuth_angles)
combined_force_magnitudes = np.array(combined_force_magnitudes)
combined_unique_command_timestamps = np.array(combined_unique_command_timestamps)
combined_force_timesteps_downsampled = np.array(combined_force_timesteps_downsampled)
combined_force_vectors = np.array(combined_force_vectors)
combined_torque_vectors = np.array(combined_torque_vectors)

# Thrust vector model
# # Calculate the regression coefficients from the combined data
non_zero_amplitude_indexes = np.where(combined_amplitude_commands > 0)
high_amplitude_indexes = np.where(combined_amplitude_commands > 0.15)

mag_slope, mag_slope2, mag_intercept = multiple_linear_regression(
    combined_velocity_commands, combined_amplitude_commands, combined_force_magnitudes
)
amp_slope, _ = curve_fit(
    custom_linear_model,
    combined_amplitude_commands[non_zero_amplitude_indexes],
    combined_elevation_angles[non_zero_amplitude_indexes],
)
quad_coeffs, _ = curve_fit(
    quadratic_model,
    combined_velocity_commands[high_amplitude_indexes],
    combined_azimuth_angles[high_amplitude_indexes],
)
exponential_coeffs, _ = curve_fit(
    exponential_model, combined_velocity_commands, combined_force_magnitudes
)
torque_coefficent, _ = curve_fit(
    custom_linear_model, combined_force_vectors[:, 2], combined_torque_vectors[:, 2]
)
# Plot relationship between force amplitude and elevation angle
x_amp = np.linspace(0, max(combined_amplitude_commands), 100)
y_elev = custom_linear_model(x_amp, amp_slope)
plt.figure()
plt.scatter(
    combined_amplitude_commands[non_zero_amplitude_indexes],
    combined_elevation_angles[non_zero_amplitude_indexes],
    c=combined_velocity_commands[non_zero_amplitude_indexes],
    cmap="viridis",
)
# Show coefficient in the label
label = "Linear fit: " + str(round(amp_slope[0], 3)) + "x"
plt.plot(x_amp, y_elev, color="red", linestyle="--", label=label)
plt.xlabel("Amplitude Command")
plt.ylabel("Elevation Angle [deg]")
plt.colorbar(label="Velocity Command")
plt.legend()
plt.title("Elevation Angle vs Amplitude Command")
plt.grid()

# Plot relationship between velocity command and azimuth angle
plt.figure()
plt.scatter(
    combined_velocity_commands[high_amplitude_indexes],
    combined_azimuth_angles[high_amplitude_indexes],
    c=combined_amplitude_commands[high_amplitude_indexes],
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
    min(combined_velocity_commands), max(combined_velocity_commands), 100
)
y_force_mag = exponential_model(x_vel, *exponential_coeffs)
plt.figure()
plt.scatter(combined_velocity_commands, combined_force_magnitudes, c=combined_amplitude_commands, cmap="viridis")


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
x_force = np.linspace(0, max(combined_force_magnitudes), 100)
y_torque = custom_linear_model(x_force, torque_coefficent)
plt.figure()
plt.scatter(abs(combined_force_vectors[:, 2]), abs(combined_torque_vectors[:, 2]),
            c=combined_amplitude_commands, cmap="viridis")
# Show coefficient in the label
label = "Linear fit: " + str(round(torque_coefficent[0], 4)) + "x"
plt.plot(x_force, y_torque, color="red", linestyle="--", label=label)
plt.xlabel("Force Magnitude")
plt.ylabel("Torque Magnitude")
plt.colorbar(label="Amplitude Command")
plt.title("Torque Magnitude vs Force Magnitude")
plt.legend()
plt.grid()

plt.tight_layout()
if save_plot:
    for i in plt.get_fignums():
        plt.figure(i).savefig(folder + str(i) + '.pdf', bbox_inches="tight")
else:
    plt.show()
