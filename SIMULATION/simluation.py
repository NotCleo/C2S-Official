import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ================= CONFIGURATION =================
CSV_FILE = 'mpu_flight.csv'
PLAYBACK_SPEED = 1.0  # 1.0 = real time

# ================= LOAD DATA =====================
data = pd.read_csv(CSV_FILE)

times = data['time_s'].values
pitch = np.radians(data['pitch'].values)
roll  = np.radians(data['roll'].values)
yaw   = np.radians(data['yaw'].values)

# ================= ROTATION MATH =================
def get_rotation_matrix(p, r, y):
    # Yaw-Pitch-Roll (Z-Y-X)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(r), -np.sin(r)],
        [0, np.sin(r),  np.cos(r)]
    ])

    Ry = np.array([
        [ np.cos(p), 0, np.sin(p)],
        [ 0,         1, 0        ],
        [-np.sin(p), 0, np.cos(p)]
    ])

    Rz = np.array([
        [np.cos(y), -np.sin(y), 0],
        [np.sin(y),  np.cos(y), 0],
        [0,          0,         1]
    ])

    return Rz @ Ry @ Rx

# ================= BOX GEOMETRY ==================
def get_box_vertices(scale=1.0):
    return np.array([
        [-1, -1, -1],
        [ 1, -1, -1],
        [ 1,  1, -1],
        [-1,  1, -1],
        [-1, -1,  1],
        [ 1, -1,  1],
        [ 1,  1,  1],
        [-1,  1,  1]
    ]) * scale

# Face index mapping
FACE_IDX = [
    [0, 1, 2, 3],  # bottom (-Z)
    [4, 5, 6, 7],  # top (+Z)
    [0, 1, 5, 4],  # front (+Y)
    [2, 3, 7, 6],  # back (-Y)
    [1, 2, 6, 5],  # right (+X)
    [4, 7, 3, 0]   # left (-X)
]

# Face colors (fixed orientation reference)
FACE_COLORS = [
    '#FF5733',  # bottom
    '#33C3FF',  # top
    '#33FF57',  # front
    '#FF33A8',  # back
    '#FFD433',  # right
    '#8E33FF'   # left
]

# ================= FIGURE SETUP ==================
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-2, 2)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

ax.set_title('MPU6050 Motion Replay')

# Initial box
base_verts = get_box_vertices()
faces = [[base_verts[j] for j in face] for face in FACE_IDX]

poly3d = Poly3DCollection(
    faces,
    facecolors=FACE_COLORS,
    edgecolors='k',
    linewidths=1,
    alpha=0.7
)

ax.add_collection3d(poly3d)

# ================= ANIMATION UPDATE ==============
def update(frame):
    R = get_rotation_matrix(pitch[frame], roll[frame], yaw[frame])
    rotated_verts = (R @ base_verts.T).T

    new_faces = [[rotated_verts[j] for j in face] for face in FACE_IDX]
    poly3d.set_verts(new_faces)

    ax.set_title(
        f"Time: {times[frame]:.2f}s | "
        f"P: {np.degrees(pitch[frame]):.1f}° "
        f"R: {np.degrees(roll[frame]):.1f}° "
        f"Y: {np.degrees(yaw[frame]):.1f}°"
    )

    return poly3d,

# ================= RUN ANIMATION =================
interval = (times[1] - times[0]) * 1000 / PLAYBACK_SPEED

ani = FuncAnimation(
    fig,
    update,
    frames=len(times),
    interval=interval,
    blit=False
)

plt.show()
