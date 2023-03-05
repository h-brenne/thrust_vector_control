import pandas as pd
import numpy as np
df = pd.read_csv('example.csv')
df['datetime'] = pd.to_datetime(df['Time'])

position = df.columns.get_loc('datetime')
elapsed = df.iloc[1:, position] - df.iat[0, position]
df["seconds"]=elapsed.dt.total_seconds()
df.at[0, "seconds"] = 0.0
print(np.diff(df["seconds"]))
print("Mean update rate:", 1/np.diff(df["seconds"]).mean())