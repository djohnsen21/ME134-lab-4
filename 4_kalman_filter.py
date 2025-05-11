# ME 134 Lab 4 (SELF-WRITTEN FOR EXTRA CREDIT)
# Darrien Johnsen
# Professor Nemitz
# SP25

import time
import math
import random
from XRPLib.differential_drive import DifferentialDrive
from XRPLib.board import Board

class KalmanFilter:
    def __init__(self):
        self.x = [0.0, 0.0, 0.0]  # [x, y, theta]
        self.P = [[1,0,0],[0,1,0],[0,0,0.1]]
        self.Q = [[0.01,0,0],[0,0.01,0],[0,0,0.01]]
        self.R = 0.05

    def predict(self, v_l, v_r, L, dt):
        v = (v_r + v_l) / 2.0
        omega = (v_r - v_l) / L
        theta = self.x[2]

        if abs(omega) < 1e-5:
            dx = v * math.cos(theta) * dt
            dy = v * math.sin(theta) * dt
            dtheta = 0
        else:
            R = v / omega
            dx = R * (math.sin(theta + omega * dt) - math.sin(theta))
            dy = -R * (math.cos(theta + omega * dt) - math.cos(theta))
            dtheta = omega * dt

        self.x[0] += dx
        self.x[1] += dy
        self.x[2] += dtheta

        for i in range(3):
            for j in range(3):
                self.P[i][j] += self.Q[i][j]

    def update(self, gyro_z, dt):
        theta_meas = self.x[2] + gyro_z * dt
        P_theta = self.P[2][2]
        K = P_theta / (P_theta + self.R)
        innovation = theta_meas - self.x[2]
        self.x[2] += K * innovation
        self.P[2][2] = (1 - K) * P_theta

def generate_random_sequence(n_moves):
    sequence = []
    for _ in range(n_moves):
        mode = random.choice(["fwd", "rev", "left", "right"])
        speed = random.randint(15, 30)  # cm/s
        duration = random.uniform(1.0, 2.0)  # seconds

        if mode == "fwd":
            sequence.append((speed, speed, duration))
        elif mode == "rev":
            sequence.append((-speed, -speed, duration))
        elif mode == "left":
            sequence.append((-speed, speed, duration))
        elif mode == "right":
            sequence.append((speed, -speed, duration))
    return sequence

def run_experiment():
    board = Board.get_default_board()
    drive = DifferentialDrive.get_default_differential_drive()
    kf = KalmanFilter()
    L = 0.08  # Wheelbase (meters)

    board.wait_for_button()
    print("Starting experiment")

    start_time = time.ticks_ms()
    logfile = open("kalman_log.csv", "w")
    logfile.write("time_ms,x_fk,y_fk,theta_fk,x_kf,y_kf,theta_kf,gyro_z\n")

    for trial_num in range(3):
        print(f"Running Trial {trial_num + 1}")
        sequence = generate_random_sequence(5)

        for left_speed, right_speed, duration in sequence:
            drive.set_speed(left_speed, right_speed)
            t0 = time.ticks_ms()

            while time.ticks_diff(time.ticks_ms(), t0) < duration * 1000:
                dt = 0.1
                time.sleep(dt)

                gyro_z = drive.imu.get_gyro_z_rate()
                v_l = drive.left_motor.get_speed() / 1000.0
                v_r = drive.right_motor.get_speed() / 1000.0

                x_fk = kf.x[0]
                y_fk = kf.x[1]
                theta_fk = kf.x[2]

                kf.predict(v_l, v_r, L, dt)
                kf.update(gyro_z, dt)

                now = time.ticks_diff(time.ticks_ms(), start_time)
                logfile.write(f"{now},{x_fk},{y_fk},{theta_fk},{kf.x[0]},{kf.x[1]},{kf.x[2]},{gyro_z}\n")

    drive.stop()
    logfile.close()
    print("Experiment complete. Data saved to kalman_log.csv")

if __name__ == "__main__":
    run_experiment()
