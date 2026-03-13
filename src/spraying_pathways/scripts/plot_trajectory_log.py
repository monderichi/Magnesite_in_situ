#!/usr/bin/env python3

import argparse
import os
import csv
import numpy as np
import matplotlib.pyplot as plt

def plot_csv_data(csv_path):
    with open(csv_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        headers = next(reader)
        data = [list(map(float, row)) for row in reader]

    data = np.array(data)
    time = data[:, 0]
    time = time - time[0]  # Shift time to start at 0

    # Column indices
    joint_pos_idxs = [i for i, name in enumerate(headers) if name.endswith('_pos') and not name.startswith('ee_')]
    joint_vel_idxs = [i for i, name in enumerate(headers) if name.endswith('_vel') and not name.startswith('ee_')]
    ee_pos_idxs = [headers.index(name) for name in ['ee_x', 'ee_y', 'ee_z'] if name in headers]
    ee_ori_idxs = [headers.index(name) for name in ['ee_roll', 'ee_pitch', 'ee_yaw'] if name in headers]
    ee_speed_idx = headers.index('ee_speed') if 'ee_speed' in headers else None

    # Plot Joint Positions
    plt.figure()
    for idx in joint_pos_idxs:
        plt.plot(time, data[:, idx], label=headers[idx])
    plt.title('Joint Positions')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [rad]')
    plt.grid()
    plt.legend()

    # Plot Joint Velocities
    plt.figure()
    for idx in joint_vel_idxs:
        plt.plot(time, data[:, idx], label=headers[idx])
    plt.title('Joint Velocities')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [rad/s]')
    plt.grid()
    plt.legend()

    # Plot End-Effector Position
    if ee_pos_idxs:
        plt.figure()
        for idx in ee_pos_idxs:
            plt.plot(time, data[:, idx], label=headers[idx])
        plt.title('End-Effector Position (x, y, z)')
        plt.xlabel('Time [s]')
        plt.ylabel('Position [m]')
        plt.grid()
        plt.legend()

    # Plot End-Effector Orientation
    if ee_ori_idxs:
        plt.figure()
        for idx in ee_ori_idxs:
            plt.plot(time, data[:, idx], label=headers[idx])
        plt.title('End-Effector Orientation (roll, pitch, yaw)')
        plt.xlabel('Time [s]')
        plt.ylabel('Angle [rad]')
        plt.grid()
        plt.legend()

    # Plot End-Effector Speed
    if ee_speed_idx is not None:
        plt.figure()
        plt.plot(time, data[:, ee_speed_idx], label='ee_speed', color='black')
        plt.title('End-Effector Linear Speed')
        plt.xlabel('Time [s]')
        plt.ylabel('Speed [m/s]')
        plt.grid()
        plt.legend()

    plt.show()

def main():
    parser = argparse.ArgumentParser(description='Plot robot joint and EE data from CSV log.')
    parser.add_argument('csv_file', help='Name of the CSV file to plot (e.g. trajectory_log_YYYYMMDD_HHMMSS.csv)')
    args = parser.parse_args()

    log_folder = os.path.expanduser('/ros2_ws/src/spraying_pathways/robot_logs')
    csv_path = os.path.join(log_folder, args.csv_file)

    if not os.path.exists(csv_path):
        print(f"File not found: {csv_path}")
        return

    plot_csv_data(csv_path)

if __name__ == '__main__':
    main()
