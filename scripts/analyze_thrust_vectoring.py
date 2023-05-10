import numpy as np
import pandas as pd
from sklearn.linear_model import LinearRegression
from numba import njit
import re


def multiple_linear_regression(x1, x2, y):
    X = np.column_stack((x1, x2))
    model = LinearRegression(fit_intercept=True).fit(X, y)
    return model.coef_[0], model.coef_[1], model.intercept_


def custom_linear_model(x, slope):
    return slope * x


def quadratic_model(x, a, b, c):
    return a * x**2 + b * x + c


def exponential_model(x, a, b):
    return a**x + b


def calc_elevation_azimuth_angles(force_vectors):
    x = force_vectors[:, 0]
    y = force_vectors[:, 1]
    z = force_vectors[:, 2]

    # Calculate the elevation angle (pitch) between the XY projection and the X-axis (in degrees)
    elevation = np.rad2deg(np.arctan2(np.sqrt(x**2 + y**2), -z))

    # Calculate the azimuth angle (yaw) about the Z-axis (in degrees)
    azimuth = np.rad2deg(np.arctan2(y, x))

    return elevation, azimuth


def downsample(
    force_vectors, torque_vectors, timestamps, sequence_length, experiment_duration, transient_duration
):
    """Downsample the force data to a fixed number of points.

    Parameters
    ----------
    force_vectors : array, shape (N, 3)
        The force vectors in x, y, z components.
    torque_vectors : array, shape (N, 3)
        The torque vectors in x, y, z components.
    timestamps : array, shape (N,)
        The timestamps corresponding to the force vectors.
    sequence_length : int
        The length of the downsampled sequence.
    experiment_duration : float
        The duration of the experiment.
    transient_duration : float
        The duration of the transient.

    Returns
    -------
    downsampled_force_vectors : array, shape (sequence_length, 3)
        The downsampled force vectors.
    downsampled_torque_vectors : array, shape (sequence_length, 3)
        The downsampled torque vectors.
    downsampled_timestamps : array, shape (sequence_length,)
        The timestamps corresponding to the downsampled force vectors."""

    measurement_interval = experiment_duration / sequence_length
    downsampled_force_vectors = []
    downsampled_torque_vectors = []
    downsampled_timestamps = []

    for i in range(sequence_length):
        start_time = i * measurement_interval + transient_duration
        end_time = (i + 1) * measurement_interval - transient_duration * 0.5

        mask = (timestamps >= start_time) & (timestamps < end_time)
        average_force = np.mean(force_vectors[mask], axis=0)
        average_torque = np.mean(torque_vectors[mask], axis=0)
        average_timestamp = np.mean(timestamps[mask])
        downsampled_force_vectors.append(average_force)
        downsampled_torque_vectors.append(average_torque)
        downsampled_timestamps.append(average_timestamp)

    return np.array(downsampled_force_vectors), np.array(downsampled_torque_vectors), np.array(downsampled_timestamps)


@njit
def find_change_indices(amplitude_commands, velocity_commands, phase_commands):
    change_indices = [0]

    for i in range(1, len(amplitude_commands)):
        current_amplitude = amplitude_commands[i]
        previous_amplitude = amplitude_commands[i - 1]
        current_velocity = velocity_commands[i]
        previous_velocity = velocity_commands[i - 1]
        current_phase = phase_commands[i]
        previous_phase = phase_commands[i - 1]

        if (
            current_amplitude != previous_amplitude
            or current_velocity != previous_velocity
            or current_phase != previous_phase
        ):
            change_indices.append(i)

    return change_indices


def find_unique_command_data(command_data):
    """Find the unique command timestamps and extract the commands."""

    amplitude_commands = command_data["AmplitudeCommand"].values
    phase_commands = command_data["PhaseCommand"].values
    velocity_commands = command_data["VelocityCommand"].values

    change_indices = find_change_indices(
        amplitude_commands, velocity_commands, phase_commands
    )

    unique_command_timestamps = command_data.loc[change_indices, "seconds"].values
    # Compute sequence_duration and sequence_length from unique_command_timestamps
    sequence_duration = (
        len(unique_command_timestamps) * np.diff(unique_command_timestamps)[1]
    )
    sequence_length = len(unique_command_timestamps)
    # Extract the commands
    amplitude_commands = command_data.loc[change_indices, "AmplitudeCommand"].values
    phase_commands = command_data.loc[change_indices, "PhaseCommand"].values
    velocity_commands = command_data.loc[change_indices, "VelocityCommand"].values
    return (
        unique_command_timestamps,
        sequence_duration,
        sequence_length,
        amplitude_commands,
        phase_commands,
        velocity_commands,
    )


def read_time_offset_from_header(force_file):
    with open(force_file, "r") as f:
        header = f.readline()

    if "Force Time Offset" in header:
        offset_str = header.split("Force Time Offset = ")[1].strip()
        force_time_offset = float(offset_str)
    else:
        force_time_offset = 0.0

    return force_time_offset


def write_time_offset_to_header(force_file, force_time_offset):
    with open(force_file, "r") as f:
        lines = f.readlines()

    header = lines[0].strip()
    if "Force Time Offset" not in header:
        header += f",Force Time Offset = {force_time_offset:.2f}\n"
    else:
        header = re.sub(
            r"Force Time Offset = [\d.]+",
            f"Force Time Offset = {force_time_offset:.2f}",
            header,
        )

    with open(force_file, "w") as f:
        f.write(header)
        f.writelines(lines[1:])


def process_dataset(command_file, force_file, startup_time, transient_duration, inverted):
    # Load command data
    df = pd.read_csv(command_file)
    df["datetime"] = pd.to_datetime(df["Time"])
    position = df.columns.get_loc("datetime")
    elapsed = df.iloc[1:, position] - df.iat[0, position]
    df["seconds"] = elapsed.dt.total_seconds()
    df.at[0, "seconds"] = 0.0
    df = df[df["seconds"] >= startup_time]
    df["seconds"] -= startup_time
    df.reset_index(drop=True, inplace=True)
    command_df = df

    (
        unique_command_timestamps,
        sequence_duration,
        sequence_length,
        amplitude_commands,
        phase_commands,
        velocity_commands,
    ) = find_unique_command_data(df)
    # Load force data
    force_df = pd.read_csv(force_file)
    force_time_offset = read_time_offset_from_header(force_file)
    if force_time_offset == 0.0:
        # Write default force_time_offset to the header
        write_time_offset_to_header(force_file, force_time_offset)
    force_offset = force_time_offset
    num_samples = len(force_df["Torque Z (N-m)"])
    # Change from 180 deg offset NWU to NED
    # Would be better to do this in the ATI tool transform
    force_df["Force Z (N)"] = -force_df["Force Z (N)"]
    force_df["Force X (N)"] = -force_df["Force X (N)"]

    # For the inverted rotor
    if inverted:
        force_df["Force Z (N)"] = -force_df["Force Z (N)"]
        force_df["Force Y (N)"] = -force_df["Force Y (N)"]
    force_df["seconds"] = np.linspace(
        -force_offset, num_samples / 2000 - force_offset, num_samples
    )
    # Filter force data
    force_df_masked = force_df[
        (force_df["seconds"] >= 0) & (force_df["seconds"] <= sequence_duration)
    ]
    # Extract force vectors, non filtered
    force_vectors = np.stack(
        (
            force_df_masked["Force X (N)"],
            force_df_masked["Force Y (N)"],
            force_df_masked["Force Z (N)"],
        ),
        axis=1,
    )
    torque_vectors = np.stack(
        (
            force_df_masked["Torque X (N-m)"],
            force_df_masked["Torque Y (N-m)"],
            force_df_masked["Torque Z (N-m)"],
        ),
        axis=1,
    )
    # Fiter force/torque data
    force_df = force_df_masked.ewm(span=500).mean()

    # Downsample force_vectors using the computed sequence_duration and sequence_length
    force_vectors_downsampled, torque_vectors_downsampled, force_timesteps_downsampled = downsample(
        force_vectors,
        torque_vectors,
        force_df_masked["seconds"],
        sequence_length,
        sequence_duration,
        transient_duration,
    )

    # Calculate elevation and azimuth angles
    elevation_angles, azimuth_angles = calc_elevation_azimuth_angles(
        force_vectors_downsampled
    )

    return (
        amplitude_commands,
        phase_commands,
        velocity_commands,
        elevation_angles,
        azimuth_angles,
        force_vectors_downsampled,
        torque_vectors_downsampled,
        command_df,
        force_df,
        unique_command_timestamps,
        force_timesteps_downsampled,
    )
