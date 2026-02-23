import pandas as pd

# Load your raw CSV
df = pd.read_csv("mpu_raw_data.csv")

# Convert Accelerometer (LSB -> m/s^2)
df['ax_ms2'] = (df['ax'] / 16384.0) * 9.81
df['ay_ms2'] = (df['ay'] / 16384.0) * 9.81
df['az_ms2'] = (df['az'] / 16384.0) * 9.81

# Convert Gyroscope (LSB -> degrees/sec)
df['gx_dps'] = df['gx'] / 131.0
df['gy_dps'] = df['gy'] / 131.0
df['gz_dps'] = df['gz'] / 131.0

# Reorder and keep the columns you want
df_fixed = df[['time_s', 'ax_ms2', 'ay_ms2', 'az_ms2', 'gx_dps', 'gy_dps', 'gz_dps', 'raw_pitch', 'kalman_pitch']]

# Save it!
df_fixed.to_csv("mpu_fixed_data.csv", index=False)
print("Conversion successful!")
