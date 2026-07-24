#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt

raw_x, raw_y, raw_z, raw_yaw = [], [], [], []
filt_x, filt_y, filt_z, filt_yaw = [], [], [], []

with open("filtered_results.csv") as f:
    reader = csv.DictReader(f)
    for row in reader:
        raw_x.append(float(row["raw_x"]))
        raw_y.append(float(row["raw_y"]))
        raw_z.append(float(row["raw_z"]))
        raw_yaw.append(float(row["raw_yaw"]))
        filt_x.append(float(row["filt_x"]))
        filt_y.append(float(row["filt_y"]))
        filt_z.append(float(row["filt_z"]))
        filt_yaw.append(float(row["filt_yaw"]))

plt.figure(figsize=(12, 10))

plt.subplot(2, 2, 1)
plt.plot(raw_x, color='red', alpha=0.5, label='Raw x', linewidth=1)
plt.plot(filt_x, color='green', alpha=0.5, label='Filtered x', linewidth=1.5)
plt.legend()
plt.title('Position X')
plt.xlabel('Frame')

plt.subplot(2, 2, 2)
plt.plot(raw_y, color='red', alpha=0.5, label='Raw y', linewidth=1)
plt.plot(filt_y, color='green', alpha=0.5, label='Filtered y', linewidth=1.5)
plt.legend()
plt.title('Position Y')
plt.xlabel('Frame')

plt.subplot(2, 2, 3)
plt.plot(raw_z, color='red', alpha=0.5, label='Raw z', linewidth=1)
plt.plot(filt_z, color='green', alpha=0.5, label='Filtered z', linewidth=1.5)
plt.legend()
plt.title('Position Z')
plt.xlabel('Frame')

plt.subplot(2, 2, 4)
plt.plot(raw_yaw, color='red', alpha=0.5, label='Raw yaw', linewidth=1)
plt.plot(filt_yaw, color='green', alpha=0.5, label='Filtered yaw', linewidth=1.5)
plt.legend()
plt.title('Yaw')
plt.xlabel('Frame')

plt.tight_layout()
plt.savefig("filter_result.png", dpi=150)
print("Saved filter_result.png")
