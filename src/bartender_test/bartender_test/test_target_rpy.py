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

def mat_to_rpy(R):
    sy = math.sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2][1], R[2][2])
        y = math.atan2(-R[2][0], sy)
        z = math.atan2(R[1][0], R[0][0])
    else:
        x = math.atan2(-R[1][2], R[1][1])
        y = math.atan2(-R[2][0], sy)
        z = 0

    return [math.degrees(x), math.degrees(y), math.degrees(z)]

R_world_tcp = rpy_to_mat(math.radians(135), math.radians(-90), math.radians(90))

# We want to rotate the TCP such that it mimics J6-90, 
# which we found points the mouth downwards.
# Rotation of J6 corresponds to rotating the TCP around its local Z axis.
# Let's rotate TCP around its local Z axis by -90 degrees.
R_local_tilt = rpy_to_mat(0, 0, math.radians(-90))
R_world_tcp_tilted = mm(R_world_tcp, R_local_tilt)

new_rpy = mat_to_rpy(R_world_tcp_tilted)
print(f"Original RPY: {[135, -90, 90]}")
print(f"Target RPY to pour: {new_rpy}")
