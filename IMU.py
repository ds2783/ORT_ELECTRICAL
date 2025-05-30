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
samples = 200
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

# --- Cube Edges (pairs of vertex indices) ---
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
    global q

    # Get and calibrate sensor data
    data = bmx.get_all_data()
    gyro = (np.array(data[3:6]) - gyro_bias) * 3.0
    accel = np.array(data[6:9]) - accel_bias
    mag = np.array(data[0:3]) - mag_bias

    # Normalize accel and mag
    if np.linalg.norm(accel) > 0:
        accel /= np.linalg.norm(accel)
    if np.linalg.norm(mag) > 0:
        mag /= np.linalg.norm(mag)

    # Update EKF and get quaternion
    q = ekf.update(q, gyr=gyro, acc=accel, mag=mag)
    R = quaternion_to_rotation_matrix(q)
    

    # Flip only pitch (Y) and yaw (Z), keep roll (X) as is
    R[:, 1] *= -1  # Invert Y axis for pitch correction
    R[:, 2] *= -1  # Invert Z axis for yaw correction


    # Rotate cube vertices
    rotated = cube_vertices @ R.T

    # Clear and redraw
    ax.cla()
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Cube Orientation via Quaternion')

    # Draw cube edges
    for edge in cube_edges:
        points = rotated[list(edge)]
        ax.plot(points[:, 0], points[:, 1], points[:, 2], 'k')

    # Optional: Draw rotated axes
    origin = np.array([0, 0, 0])
    ax.quiver(*origin, *R[:, 0], color='r', length=0.6, normalize=True)  # X axis
    ax.quiver(*origin, *R[:, 1], color='g', length=0.6, normalize=True)  # Y axis
    ax.quiver(*origin, *R[:, 2], color='b', length=0.6, normalize=True)  # Z axis

# --- Animate ---
ani = FuncAnimation(fig, update, interval=dt * 1000)
plt.show()
