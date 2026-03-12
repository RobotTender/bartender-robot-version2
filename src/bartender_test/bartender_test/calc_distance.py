import math

def rpy_to_mat(roll, pitch, yaw):
    # DSR extrinsic RPY (XYZ) means R_final = Rz(yaw) * Ry(pitch) * Rx(roll)
    # in terms of order of multiplication of matrices.
    
    Rx = [
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ]
    
    Ry = [
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ]
    
    Rz = [
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ]
    
    def mm(A, B):
        C = [[0,0,0],[0,0,0],[0,0,0]]
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    C[i][j] += A[i][k] * B[k][j]
        return C
    
    # R_total = Rz * Ry * Rx
    return mm(Rz, mm(Ry, Rx))

def mat_mul(A, B):
    C = [[0,0,0],[0,0,0],[0,0,0]]
    for i in range(3):
        for j in range(3):
            for k in range(3):
                C[i][j] += A[i][k] * B[k][j]
    return C

# 1. World to TCP
R_world_tcp = rpy_to_mat(math.radians(135), math.radians(-90), math.radians(90))

# 2. TCP to Bottle
# URDF rpy: 1.5708 -2.3562 0
R_tcp_bottle = rpy_to_mat(1.5708, -2.3562, 0)

# 3. World to Bottle
R_world_bottle = mat_mul(R_world_tcp, R_tcp_bottle)

# Local vector from grasp (0, 0, 0) to top (0, 0, 140)
v_local = [0, 0, 140]
v_world = [
    R_world_bottle[0][2] * v_local[2],
    R_world_bottle[1][2] * v_local[2],
    R_world_bottle[2][2] * v_local[2]
]

print(f"World Z-axis of Bottle: {[R_world_bottle[0][2], R_world_bottle[1][2], R_world_bottle[2][2]]}")
print(f"World Vector: {v_world}")
print(f"World Z-distance: {abs(v_world[2]):.2f} mm")
