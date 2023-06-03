import numpy as np
import matplotlib.pyplot as plt
from analyze_thrust_vectoring import (
    process_dataset,
)

# Settings
save_plot = False
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

# Settings
save_plot = False
folder = "logs/large_ccw/3/"
startup_time = 1.0
transient_duration = 0.2
# Load datasets
command_file = "logs/large_ccw/3/test_buffering.csv"
force_file = "logs/large_ccw/3/force_logs/test_buffering.csv"
inverted = False

(
    amplitude_commands2,
    phase_commands2,
    velocity_commands2,
    elevation_angles2,
    azimuth_angles2,
    force_vectors2,
    torque_vectors2,
    command_df2,
    force_df2,
    unique_command_timestamps2,
    force_timesteps_downsampled2,
) = process_dataset(command_file, force_file, startup_time, transient_duration, inverted)


# Thrust vector model
# Calculate the regression coefficients from the combined data
high_amplitude_indexes = np.where(amplitude_commands > 0.15)
high_amplitude_indexes2 = np.where(amplitude_commands2 > 0.15)
# Plot relationship between velocity command and azimuth angle
plt.figure()
plt.scatter(
    velocity_commands[high_amplitude_indexes],
    azimuth_angles[high_amplitude_indexes],
    c=amplitude_commands[high_amplitude_indexes],
    cmap="viridis",
)
plt.scatter(
    velocity_commands2[high_amplitude_indexes2],
    azimuth_angles2[high_amplitude_indexes2],
    c=amplitude_commands2[high_amplitude_indexes2],
    cmap="viridis",
)
plt.xlabel("Velocity Command [Hz]")
plt.ylabel("Azimuth Angle [deg]")
plt.colorbar(label="Amplitude Command")
plt.legend()
plt.title("Azimuth Angle vs Velocity Command")
plt.yticks(np.arange(-120, 120, 30))
plt.grid()

plt.tight_layout()
if save_plot:
    for i in plt.get_fignums():
        plt.figure(i).savefig(folder + str(i) + '.pdf', bbox_inches="tight")
        plt.figure(i).savefig(folder + str(i) + '.png', bbox_inches="tight")

else:
    plt.show()
