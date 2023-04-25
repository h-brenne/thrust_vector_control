import numpy as np
import pandas as pd
from datetime import datetime, timedelta


def generate_thrust_vector_sequence(
    min_velocity,
    max_velocity,
    step_velocity,
    min_amplitude,
    max_amplitude,
    step_amplitude,
    min_phase,
    max_phase,
    step_phase,
    enable_dual_amplitude_steps,
    start_time,
    experiment_duration,
):
    data = []

    num_velocity_steps = int((max_velocity - min_velocity) / step_velocity) + 1
    num_amplitude_steps = int((max_amplitude - min_amplitude) / step_amplitude) + 1
    num_phase_steps = int((max_phase - min_phase) / step_phase) + 1

    num_commands_per_velocity = num_amplitude_steps * num_phase_steps
    if enable_dual_amplitude_steps:
        num_commands_per_velocity *= 2

    num_commands = num_velocity_steps * num_commands_per_velocity

    time_step = experiment_duration / num_commands
    current_time = start_time + timedelta(seconds=1)

    for velocity in np.arange(
        min_velocity, max_velocity + step_velocity, step_velocity
    ):
        for amplitude in np.arange(
            min_amplitude, max_amplitude + step_amplitude, step_amplitude
        ):
            for phase in np.arange(min_phase, max_phase + step_phase, step_phase):
                data.append(
                    {
                        "Time": current_time,
                        "VelocityCommand": velocity,
                        "AmplitudeCommand": amplitude,
                        "PhaseCommand": phase,
                    }
                )
                current_time += timedelta(seconds=time_step)

        if enable_dual_amplitude_steps:
            for amplitude in np.arange(
                max_amplitude, min_amplitude - step_amplitude, -step_amplitude
            ):
                for phase in np.arange(min_phase, max_phase + step_phase, step_phase):
                    data.append(
                        {
                            "Time": current_time,
                            "VelocityCommand": velocity,
                            "AmplitudeCommand": amplitude,
                            "PhaseCommand": phase,
                        }
                    )
                    current_time += timedelta(seconds=time_step)

    return data


min_velocity = 40.0
max_velocity = 90.0
step_velocity = 5.0
min_amplitude = 0.0
max_amplitude = 0.26
step_amplitude = 0.04
min_phase = 0.0
max_phase = 0.0
step_phase = 1.0
enable_dual_amplitude_steps = True
start_time = datetime.strptime("2023-04-04 12:29:57.902859", "%Y-%m-%d %H:%M:%S.%f")
experiment_duration = 180

data = generate_thrust_vector_sequence(
    min_velocity,
    max_velocity,
    step_velocity,
    min_amplitude,
    max_amplitude,
    step_amplitude,
    min_phase,
    max_phase,
    step_phase,
    enable_dual_amplitude_steps,
    start_time,
    experiment_duration,
)

# Convert the list of dictionaries to a DataFrame
df = pd.DataFrame(data)

# Save the DataFrame to a CSV file
df.to_csv("generated_command_data.csv", index=False)
