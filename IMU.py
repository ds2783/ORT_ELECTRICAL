import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ahrs.filters import EKF
from DFRobot_BMX160 import BMX160
import time

# --- Initialize Sensor ---
bmx = BMX160(1)
while not bmx.begin():
    time.sleep(2)

# --- Calibrate Sensor ---
print("Calibrating... keep sensor still.")
samples = 1000
gyro_samples, accel_samples, mag_samples = [], [], []

for _ in range(samples):
    data = bmx.get_all_data()
    gyro_samples.append(data[3:6])
    accel_samples.append(data[6:9])
    mag_samples.append(data[0:3])
    time.sleep(0.01)

gyro_bias = np.mean(gyro_samples, axis=0)
accel_bias = np.mean(accel_samples, axis=0)
mag_bias = np.mean(mag_samples, axis=0)

print("Calibration complete.")

# --- EKF Initialization ---
ekf = EKF()
q = np.array([1.0, 0.0, 0.0, 0.0])  # Initial quaternion
dt = 0.01  # 100 Hz

# --- Low-pass filter parameters ---
alpha = 0.5  # smoothing factor
gyro_filtered = np.zeros(3)
accel_filtered = np.zeros(3)
mag_filtered = np.zeros(3)

# --- Cube Vertices (Local Coordinates) ---
cube_vertices = np.array([
    [-0.5, -0.5, -0.5],
    [ 0.5, -0.5, -0.5],
    [ 0.5,  0.5, -0.5],
    [-0.5,  0.5, -0.5],
    [-0.5, -0.5,  0.5],
    [ 0.5, -0.5,  0.5],
    [ 0.5,  0.5,  0.5],
    [-0.5,  0.5,  0.5]
])

# --- Cube Edges ---
cube_edges = [
    (0,1), (1,2), (2,3), (3,0),  # Bottom
    (4,5), (5,6), (6,7), (7,4),  # Top
    (0,4), (1,5), (2,6), (3,7)   # Sides
]

# --- Quaternion to Rotation Matrix ---
def quaternion_to_rotation_matrix(q):
    q0, q1, q2, q3 = q
    return np.array([
        [1 - 2*(q2**2 + q3**2),     2*(q1*q2 - q0*q3),     2*(q1*q3 + q0*q2)],
        [    2*(q1*q2 + q0*q3), 1 - 2*(q1**2 + q3**2),     2*(q2*q3 - q0*q1)],
        [    2*(q1*q3 - q0*q2),     2*(q2*q3 + q0*q1), 1 - 2*(q1**2 + q2**2)]
    ])

# --- 3D Plot Setup ---
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Cube Orientation via Quaternion')
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

# --- Update Function ---
def update(frame):
    global q, gyro_filtered, accel_filtered, mag_filtered

    # --- Read and bias-correct sensor data ---
    data = bmx.get_all_data()
    raw_gyro = np.array(data[3:6]) - gyro_bias
    raw_accel = np.array(data[6:9]) - accel_bias
    raw_mag = np.array(data[0:3]) - mag_bias

    # --- Apply axis remapping: Sensor → World Frame ---
    # Sensor X (right), Y (forward), Z (up) → World X (forward), Y (left), Z (up)
    raw_gyro = np.array([raw_gyro[1], -raw_gyro[0], raw_gyro[2]])
    raw_accel = np.array([raw_accel[1], -raw_accel[0], raw_accel[2]])
    raw_mag = np.array([raw_mag[1], -raw_mag[0], raw_mag[2]])

    # --- Apply Low-pass Filter ---
    gyro_filtered = alpha * raw_gyro + (1 - alpha) * gyro_filtered
    accel_filtered = alpha * raw_accel + (1 - alpha) * accel_filtered
    mag_filtered = alpha * raw_mag + (1 - alpha) * mag_filtered

    # Normalize accel and mag
    if np.linalg.norm(accel_filtered) > 0:
        accel_filtered /= np.linalg.norm(accel_filtered)
    if np.linalg.norm(mag_filtered) > 0:
        mag_filtered /= np.linalg.norm(mag_filtered)

    # --- Update EKF orientation ---
    q = ekf.update(q, gyr=gyro_filtered * 3.0, acc=accel_filtered, mag=mag_filtered)
    R = quaternion_to_rotation_matrix(q)

    # --- Rotate Cube Vertices ---
    rotated = cube_vertices @ R.T

    # --- Clear and Redraw ---
    ax.cla()
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Cube Orientation via Quaternion')

    # --- Draw Cube Edges ---
    for edge in cube_edges:
        points = rotated[list(edge)]
        ax.plot(points[:, 0], points[:, 1], points[:, 2], 'k')

    # --- Highlight Front Face in Red ---
    front_face = rotated[[0, 1, 2, 3, 0]]  # Close the loop
    ax.plot(front_face[:, 0], front_face[:, 1], front_face[:, 2], color='red', linewidth=3)

# --- Animate ---
ani = FuncAnimation(fig, update, interval=dt * 1000)
plt.show()
