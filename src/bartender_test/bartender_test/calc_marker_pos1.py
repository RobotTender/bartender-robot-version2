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

# Grasp Center (World): [308.48, 64.52, 313.75]
grasp_center = [308.48, 64.52, 313.75]

# Rotation of TCP in CHEERS_POSE: [135, -90, 90]
R_world_grasp = rpy_to_mat(math.radians(135), math.radians(-90), math.radians(90))

# Bottle relative rotation from URDF: rpy="1.5708 -2.3562 0"
R_grasp_bottle = rpy_to_mat(1.5708, -2.3562, 0)
R_world_bottle = mm(R_world_grasp, R_grasp_bottle)

# Bottle center relative to grasp point: (0, 0, 20mm)
# Total height 240, Grasp height 100. Center is at 120.
v_local_center = [0, 0, 20]
v_world_offset = [
    R_world_bottle[0][2] * v_local_center[2],
    R_world_bottle[1][2] * v_local_center[2],
    R_world_bottle[2][2] * v_local_center[2]
]

marker_pos = [
    grasp_center[0] + v_world_offset[0],
    grasp_center[1] + v_world_offset[1],
    313.75
]

print(f"Calculated Marker Position: {marker_pos}")
print(f"World Z-axis of Bottle: {[R_world_bottle[0][2], R_world_bottle[1][2], R_world_bottle[2][2]]}")
