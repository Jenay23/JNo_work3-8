import csv
import matplotlib.pyplot as plt

with open("yaw_1d_ekf.csv") as f:
    reader = list(csv.DictReader(f))
    raw_yaw = [float(r["raw_yaw"]) for r in reader]
    filt_yaw = [float(r["filt_yaw"]) for r in reader]

plt.figure(figsize=(10, 6))
plt.plot(raw_yaw, color='red', alpha=0.5, label='Raw yaw', linewidth=1)
plt.plot(filt_yaw, color='green', alpha=0.5, label='Filtered yaw (1D EKF)', linewidth=1.5)
plt.legend()
plt.title('Yaw - 1D Extended Kalman Filter')
plt.xlabel('Frame')
plt.tight_layout()
plt.savefig("yaw_1d_ekf.png", dpi=150)

with open("work6.csv") as f:
    reader = list(csv.DictReader(f))
    raw_x = [float(r["raw_x"]) for r in reader]
    raw_y = [float(r["raw_y"]) for r in reader]
    raw_z = [float(r["raw_z"]) for r in reader]
    raw_yaw2 = [float(r["raw_yaw"]) for r in reader]
    filt_x = [float(r["filt_x"]) for r in reader]
    filt_y = [float(r["filt_y"]) for r in reader]
    filt_z = [float(r["filt_z"]) for r in reader]
    filt_yaw2 = [float(r["filt_yaw"]) for r in reader]

fig, axes = plt.subplots(2, 2, figsize=(12, 10))
titles = ['X position', 'Y position', 'Z position', 'Yaw']
raws = [raw_x, raw_y, raw_z, raw_yaw2]
filts = [filt_x, filt_y, filt_z, filt_yaw2]
for ax, title, raw, filt in zip(axes.flat, titles, raws, filts):
    ax.plot(raw, color='red', alpha=0.5, linewidth=1)
    ax.plot(filt, color='blue', alpha=0.5, linewidth=1.5)
    ax.set_title(title)
    ax.set_xlabel('Frame')
    ax.legend(['Raw', 'Filtered'])
plt.tight_layout()
plt.savefig("work6.png", dpi=150)
