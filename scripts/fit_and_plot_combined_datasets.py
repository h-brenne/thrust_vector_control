import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from matplotlib import cm

from analyze_thrust_vectoring import (
    process_dataset,
    multiple_linear_regression,
    custom_linear_model,
    quadratic_model,
    exponential_model,
)

# Settings
startup_time = 1.0
transient_duration = 0.2
# Load datasets
datasets = [
    ("logs/large_ccw/test4.csv", "logs/large_ccw/force_logs/test4.csv"),
    ("logs/large_ccw/test5.csv", "logs/large_ccw/force_logs/test5.csv"),
    ("logs/large_ccw/test6.csv", "logs/large_ccw/force_logs/test6.csv"),
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

for idx, (command_file, force_file) in enumerate(datasets):
    (
        amplitude_commands,
        phase_commands,
        velocity_commands,
        elevation_angles,
        azimuth_angles,
        force_magnitudes,
        _,
        force_df_smooth,
        unique_command_timestamps,
        force_timesteps_downsampled,
    ) = process_dataset(command_file, force_file, startup_time, transient_duration)

    combined_amplitude_commands.extend(amplitude_commands)
    combined_phase_commands.extend(phase_commands)
    combined_velocity_commands.extend(velocity_commands)
    combined_elevation_angles.extend(elevation_angles)
    combined_azimuth_angles.extend(azimuth_angles)
    combined_force_magnitudes.extend(force_magnitudes)
    combined_unique_command_timestamps.extend(unique_command_timestamps)
    combined_force_timesteps_downsampled.extend(force_timesteps_downsampled)

# Convert combined data to NumPy arrays
combined_amplitude_commands = np.array(combined_amplitude_commands)
combined_phase_commands = np.array(combined_phase_commands)
combined_velocity_commands = np.array(combined_velocity_commands)
combined_elevation_angles = np.array(combined_elevation_angles)
combined_azimuth_angles = np.array(combined_azimuth_angles)
combined_force_magnitudes = np.array(combined_force_magnitudes)
combined_unique_command_timestamps = np.array(combined_unique_command_timestamps)
combined_force_timesteps_downsampled = np.array(combined_force_timesteps_downsampled)
print(combined_force_magnitudes)

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
plt.plot(x_amp, y_elev, color="red", linestyle="--", label="Linear fit")
plt.xlabel("Amplitude Command")
plt.ylabel("Elevation Angle [deg]")
plt.colorbar(label="Velocity Command")
plt.legend()
plt.title("Elevation Angle vs Amplitude Command")
plt.grid()

# Plot relationship between velocity command and azimuth angle
x_vel = np.linspace(
    min(combined_velocity_commands), max(combined_velocity_commands), 100
)
y_phase = quadratic_model(x_vel, *quad_coeffs)
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

# Create a 3D scatter plot
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Scatter plot of actual data
ax.scatter(
    combined_velocity_commands,
    combined_amplitude_commands,
    combined_force_magnitudes,
    c="blue",
    label="Actual Data",
)

# Create a grid of VelocityCommand and AmplitudeCommand values
x_range = np.linspace(
    min(combined_velocity_commands), max(combined_velocity_commands), 100
)
y_range = np.linspace(
    min(combined_amplitude_commands), max(combined_amplitude_commands), 100
)
X, Y = np.meshgrid(x_range, y_range)

# Calculate predicted force magnitudes using the multiple linear regression model
Z = mag_slope * X + mag_slope2 * Y + mag_intercept

# Surface plot of predicted force magnitudes
surf = ax.plot_surface(X, Y, Z, cmap=cm.viridis, alpha=0.6)
ax.set_xlabel("Velocity Command")
ax.set_ylabel("Amplitude Command")
ax.set_zlabel("Force Magnitude")
ax.legend()
plt.title(
    "Force Magnitude vs Velocity Command and Amplitude Command (Multiple Linear Regression)"
)

# Plot relationship between force magnitude and velocity command and amplitude command
x_vel = np.linspace(
    min(combined_velocity_commands), max(combined_velocity_commands), 100
)
y_force_mag = exponential_model(x_vel, *exponential_coeffs)
plt.figure()
plt.scatter(
    combined_velocity_commands,
    combined_force_magnitudes,
    c=combined_amplitude_commands,
    cmap="viridis",
)
plt.plot(x_vel, y_force_mag, color="red", linestyle="--", label="Exponential fit")
plt.xlabel("Velocity Command")
plt.ylabel("Force Magnitude")
plt.colorbar(label="Amplitude Command")
plt.title("Force Magnitude vs Velocity Command and Amplitude Command")
plt.legend()
plt.grid()

plt.show()
