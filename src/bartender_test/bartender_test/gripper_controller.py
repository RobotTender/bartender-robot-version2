import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import DrlStart
import textwrap

# Base DRL code template for Modbus communication with the Robotis RH-12-RN gripper
DRL_BASE_CODE_TEMPLATE = """
g_slaveid = 0
flag = 0
def modbus_set_slaveid(slaveid):
    global g_slaveid
    g_slaveid = slaveid
def modbus_fc06(address, value):
    global g_slaveid
    data = (g_slaveid).to_bytes(1, byteorder='big')
    data += (6).to_bytes(1, byteorder='big')
    data += (address).to_bytes(2, byteorder='big')
    data += (value).to_bytes(2, byteorder='big')
    return modbus_send_make(data)
def modbus_fc16(startaddress, cnt, valuelist):
    global g_slaveid
    data = (g_slaveid).to_bytes(1, byteorder='big')
    data += (16).to_bytes(1, byteorder='big')
    data += (startaddress).to_bytes(2, byteorder='big')
    data += (cnt).to_bytes(2, byteorder='big')
    data += (2 * cnt).to_bytes(1, byteorder='big')
    for i in range(0, cnt):
        data += (valuelist[i]).to_bytes(2, byteorder='big')
    return modbus_send_make(data)
def recv_check():
    size, val = flange_serial_read(0.1)
    if size > 0:
        return True, val
    else:
        tp_log("CRC Check Fail")
        return False, val
def gripper_move(stroke):
    flange_serial_write(modbus_fc16(282, 2, [stroke, 0]))
    wait(1.0) 

# Initialization logic
while True:
    flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
    modbus_set_slaveid(1)
    flange_serial_write(modbus_fc06(256, 1))   # torque enable
    flag, val = recv_check()
    flange_serial_write(modbus_fc06(275, {current})) # goal current (Force)
    flag, val = recv_check()
    if flag is True:
        break
    flange_serial_close()
"""

class GripperController:
    def __init__(self, node: Node, namespace: str = "dsr01", robot_system: int = 0):
        self.node = node
        self.robot_system = robot_system
        self.cli = self.node.create_client(DrlStart, f"/{namespace}/drl/drl_start")
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info("Waiting for DRL service...")

    def move(self, stroke: int, current: int = 400) -> bool:
        """Sends a single stroke command to the gripper."""
        self.node.get_logger().info(f"Gripper Move: stroke={stroke}, force={current}")
        drl_code = DRL_BASE_CODE_TEMPLATE.format(current=current)
        task_code = f"gripper_move({stroke})"
        
        req = DrlStart.Request()
        req.robot_system = self.robot_system
        req.code = f"{drl_code}\n{task_code}"
        
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        return bool(future.result().success) if future.result() else False

    def move_sequence(self, sequence, current: int = 400) -> bool:
        """
        Sends a sequence of stroke commands in a single DRL script.
        sequence: list of strokes [s1, s2, ...]
        """
        self.node.get_logger().info(f"Gripper Sequence: {sequence}, force={current}")
        drl_code = DRL_BASE_CODE_TEMPLATE.format(current=current)
        
        task_code = ""
        for stroke in sequence:
            task_code += f"gripper_move({stroke})\n"
            task_code += "wait(1.0)\n"
            
        req = DrlStart.Request()
        req.robot_system = self.robot_system
        req.code = f"{drl_code}\n{task_code}"
        
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=15.0)
        return bool(future.result().success) if future.result() else False

    def terminate(self) -> bool:
        req = DrlStart.Request()
        req.code = "flange_serial_close()"
        self.cli.call_async(req)
        return True
