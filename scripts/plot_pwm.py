import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from collections import Counter

df = pd.read_csv("logs/pwm_data/pwm_controller_test.csv")
# Data looks like this: Timestamp_us,Pin0,Pin1,Pin2,Pin3,Pin4,Pin5

timesteps_us = df['Timestamp_us']

# Analyze latency
latency_us = timesteps_us.diff()

print(latency_us.describe())

jitter_us = []
for i in range(6):
    jitter_us.append(df['Pin{}'.format(i)].diff())
    print(jitter_us[i].describe())

pin_diff = np.zeros((6*len(timesteps_us)))
plt.figure()
for i in range(6):
    # Find differenece for each pin from 900
    pin_diff = np.concatenate((pin_diff, np.abs(df['Pin{}'.format(i)] - 900).to_numpy()))
    plt.plot(timesteps_us, df['Pin{}'.format(i)], label='Pin{}'.format(i))
plt.legend()

counts = Counter(pin_diff)
plt.figure()
plt.bar(counts.keys(), counts.values(), log=True)
plt.xlabel("Jitter [µs]")
plt.ylabel("Count (log scale)")
plt.title(f'''RPi PWM pulse width jitter.
              Mean: {pin_diff.mean():.2f} µs, Std: {pin_diff.std():.2f} µs
              Max: {pin_diff.max():.2f} µs''')
plt.show()
