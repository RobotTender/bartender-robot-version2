import math

def mm(A, B):
    C = [[0,0,0],[0,0,0],[0,0,0]]
    for i in range(3):
        for j in range(3):
            for k in range(3):
                C[i][j] += A[i][k] * B[k][j]
    return C

def rpy_to_mat(roll, pitch, yaw):
    Rx = [[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]]
    Ry = [[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]]
    Rz = [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
    return mm(Rz, mm(Ry, Rx))

# Base TCP: 135, -90, 90
R_world_tcp = rpy_to_mat(math.radians(135), math.radians(-90), math.radians(90))

# Bottle relative
R_grasp_bottle = rpy_to_mat(1.5708, -2.3562, 0)

# Try rotating TCP around its local Z by -90 degrees (which is J6-90)
R_local_tilt = rpy_to_mat(0, 0, math.radians(-90))
R_world_tcp_tilted = mm(R_world_tcp, R_local_tilt)
R_world_bottle_tilted = mm(R_world_tcp_tilted, R_grasp_bottle)

v_local = [0, 0, 1]
v_world_tilted = [
    R_world_bottle_tilted[0][2] * v_local[2],
    R_world_bottle_tilted[1][2] * v_local[2],
    R_world_bottle_tilted[2][2] * v_local[2]
]
print(f"TCP Rz-90 (J6-90) mouth direction: {v_world_tilted}")
