#!/usr/bin/env python3

import csv
import sys
import matplotlib.pyplot as plt

if len(sys.argv) != 2:
    print("Usage: python3 plot_trajectory.py <path_to_csv>")
    sys.exit(1)

csv_path = sys.argv[1]

time = []
x = []
y = []
v = []
a = []

# Read CSV file manually (skip header)
with open(csv_path, 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Skip header

    for row in reader:
        if len(row) != 5:
            continue  # skip malformed lines
        t_val = float(row[0])
        x_val = float(row[1])
        y_val = float(row[2])
        v_val = float(row[3])
        a_val = float(row[4])

        time.append(t_val)
        x.append(x_val)
        y.append(y_val)
        v.append(v_val)
        a.append(a_val)

# Plotting
fig, axs = plt.subplots(2, 2, figsize=(12, 8))

# X vs Time
axs[0, 0].plot(time, x, 'b-')
axs[0, 0].set_title('X vs Time')
axs[0, 0].set_xlabel('Time [s]')
axs[0, 0].set_ylabel('X [m]')
axs[0, 0].grid(True)

# Y vs Time
axs[0, 1].plot(time, y, 'm-')
axs[0, 1].set_title('Y vs Time')
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('Y [m]')
axs[0, 1].grid(True)

# Velocity vs Time
axs[1, 0].plot(time, v, 'g-')
axs[1, 0].set_title('Velocity vs Time')
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('Speed [m/s]')
axs[1, 0].grid(True)

# Acceleration vs Time
axs[1, 1].plot(time, a, 'r-')
axs[1, 1].set_title('Acceleration vs Time')
axs[1, 1].set_xlabel('Time [s]')
axs[1, 1].set_ylabel('Acceleration [m/s²]')
axs[1, 1].grid(True)

plt.tight_layout()
plt.show()
