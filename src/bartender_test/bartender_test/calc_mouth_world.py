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

# 1. World to Grasp rotation
R_world_grasp = rpy_to_mat(math.radians(135), math.radians(-90), math.radians(90))

# 2. Grasp to Bottle rotation
# URDF rpy: 1.5708 -2.3562 0
R_grasp_bottle = rpy_to_mat(1.5708, -2.3562, 0)

# 3. World to Bottle rotation
R_world_bottle = mm(R_world_grasp, R_grasp_bottle)

# Local mouth center offset (0, 0, 155mm)
v_local = [0, 0, 155]
v_world_offset = [
    R_world_bottle[0][2] * v_local[2],
    R_world_bottle[1][2] * v_local[2],
    R_world_bottle[2][2] * v_local[2]
]

# Grasp Center (from check_tcp output)
grasp_center = [308.48, 64.52, 313.75]

mouth_world = [
    grasp_center[0] + v_world_offset[0],
    grasp_center[1] + v_world_offset[1],
    grasp_center[2] + v_world_offset[2]
]

print(f"Grasp Center World: {grasp_center}")
print(f"World Offset: {v_world_offset}")
print(f"Mouth World: {mouth_world}")
