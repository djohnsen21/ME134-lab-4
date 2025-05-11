import matplotlib.pyplot as plt
import csv
import numpy as np

# final manual measurements in m
ground_truth = [
    (-0.08, 0.24),  # Trial 1
    (-0.17, -0.27),   # Trial 2
    (-0.94, 0.65)    # Trial 3
]

# Load CSV data
time_ms = []
x_fk = []
y_fk = []
theta_fk = []
x_kf = []
y_kf = []
theta_kf = []
gyro_z = []

with open("kalman_log.csv", "r") as file:
    reader = csv.DictReader(file)
    for row in reader:
        time_ms.append(int(row["time_ms"]))
        x_fk.append(float(row["x_fk"]))
        y_fk.append(float(row["y_fk"]))
        theta_fk.append(float(row["theta_fk"]))
        x_kf.append(float(row["x_kf"]))
        y_kf.append(float(row["y_kf"]))
        theta_kf.append(float(row["theta_kf"]))
        gyro_z.append(float(row["gyro_z"]))

time_s = [t / 1000.0 for t in time_ms]

# Trajectory Plot
plt.figure()
plt.plot(x_fk, y_fk, label="Forward Kinematics", linestyle="--")
plt.plot(x_kf, y_kf, label="Kalman Filter", linewidth=2)
plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")
plt.title("Trajectory Comparison")
plt.legend()
plt.grid(True)
plt.axis("equal")

# Theta vs Time
plt.figure()
plt.plot(time_s, theta_fk, label="FK θ", linestyle="--")
plt.plot(time_s, theta_kf, label="Kalman θ", linewidth=2)
plt.xlabel("Time (s)")
plt.ylabel("Theta (rad)")
plt.title("Orientation Over Time")
plt.legend()
plt.grid(True)

# Gyro Z vs Time
plt.figure()
plt.plot(time_s, gyro_z, label="Gyro Z", color="gray")
plt.xlabel("Time (s)")
plt.ylabel("Angular Rate (rad/s)")
plt.title("Gyroscope Z-axis Reading")
plt.grid(True)

# Comparison
n_trials = 3
trial_len = len(time_ms) // n_trials
trial_bounds = [(i * trial_len, (i + 1) * trial_len - 1) for i in range(n_trials)]
fk_final = [(x_fk[end], y_fk[end]) for (_, end) in trial_bounds]
kf_final = [(x_kf[end], y_kf[end]) for (_, end) in trial_bounds]

plt.figure()
for i in range(n_trials):
    gt = ground_truth[i]
    fk = fk_final[i]
    kf = kf_final[i]

    plt.plot(gt[0]*100, gt[1]*100, "bo", label="Ground Truth" if i==0 else "")
    plt.plot(fk[0]*100, fk[1]*100, "gx", label="FK Estimate" if i==0 else "")
    plt.plot(kf[0]*100, kf[1]*100, "rx", label="Kalman Estimate" if i==0 else "")
    plt.text(gt[0]*100 + 2, gt[1]*100, f"{i+1}", color='blue')
    plt.text(fk[0]*100 + 2, fk[1]*100, f"{i+1}", color='green')
    plt.text(kf[0]*100 + 2, kf[1]*100, f"{i+1}", color='red')

plt.xlabel("X position (cm)")
plt.ylabel("Y position (cm)")
plt.title("FK vs Kalman Estimates vs Ground Truth (Final Points)")
plt.legend()
plt.grid(True)
plt.axis("equal")

plt.show()
