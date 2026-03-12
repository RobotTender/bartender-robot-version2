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

R_grasp_bottle = rpy_to_mat(1.5708, -2.3562, 0)
v_local = [0, 0, 155]
v_flange = [
    R_grasp_bottle[0][2] * v_local[2],
    R_grasp_bottle[1][2] * v_local[2],
    R_grasp_bottle[2][2] * v_local[2]
]
print(f"Mouth vector in Flange frame: {v_flange}")
