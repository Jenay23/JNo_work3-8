import csv
import matplotlib.pyplot as plt

raw_yaw = []
filt_yaw = []

with open("yaw_filtered_1d.csv") as f:
    reader = csv.DictReader(f)
    for row in reader:
        raw_yaw.append(float(row["raw_yaw"]))
        filt_yaw.append(float(row["filt_yaw"]))

plt.figure(figsize=(10, 6))
plt.plot(raw_yaw, color='red', alpha=0.5, label='Raw yaw', linewidth=1)
plt.plot(filt_yaw, color='green', alpha=0.5, label='Filtered yaw (1D EKF)', linewidth=1.5)
plt.legend()
plt.title('Yaw - 1D Extended Kalman Filter')
plt.xlabel('Frame')
plt.tight_layout()
plt.savefig("filter_result.png", dpi=150)
print("Saved filter_result.png")