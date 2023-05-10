import pandas as pd
import matplotlib.pyplot as plt

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

for i in range(6):
    plt.plot(timesteps_us, df['Pin{}'.format(i)], label='Pin{}'.format(i))
plt.legend()
plt.show()
