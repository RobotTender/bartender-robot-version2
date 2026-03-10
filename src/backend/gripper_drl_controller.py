import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import DrlStart
import textwrap
import time
import os
import DR_init

# Gripper mapping for log display
# distance 109mm -> pulse 0, distance 0mm -> pulse 740
_GRIPPER_DISTANCE_MIN_MM = 0.0
_GRIPPER_DISTANCE_MAX_MM = 109.0
_GRIPPER_PULSE_MIN = 0
_GRIPPER_PULSE_MAX = 740


def _pulse_to_distance_mm(pulse: int) -> float:
    try:
        p = int(pulse)
    except Exception:
        p = _GRIPPER_PULSE_MIN
    p = max(_GRIPPER_PULSE_MIN, min(_GRIPPER_PULSE_MAX, p))
    ratio = (p - _GRIPPER_PULSE_MIN) / float(_GRIPPER_PULSE_MAX - _GRIPPER_PULSE_MIN)
    return _GRIPPER_DISTANCE_MAX_MM - (ratio * (_GRIPPER_DISTANCE_MAX_MM - _GRIPPER_DISTANCE_MIN_MM))

DRL_BASE_CODE = """
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
    wait(1.0) # 물리적 동작 시간을 충분히 기다려줍니다.

# ---- init serial & torque/current ----
while True:
    flange_serial_open(
        baudrate=57600,
        bytesize=DR_EIGHTBITS,
        parity=DR_PARITY_NONE,
        stopbits=DR_STOPBITS_ONE,
    )

    modbus_set_slaveid(1)

    # 256(40257) Torque enable
    # 275(40276) Goal Current
    # 282(40283) Goal Position

    flange_serial_write(modbus_fc06(256, 1))   # torque enable
    flag, val = recv_check()

    flange_serial_write(modbus_fc06(275, 500)) # goal current
    #flange_serial_write(modbus_fc06(275, 100)) # goal current
    flag, val = recv_check()

    if flag is True:
        break

    flange_serial_close()
"""

class GripperController:
    def __init__(self, node: Node, namespace: str = "dsr01", robot_system: int = 0):
        if node is None:
            node = getattr(DR_init, "__dsr__node", None)
        if node is None:
            raise RuntimeError("GripperController node is None. DR_init.__dsr__node is also None.")

        self.node = node
        self.robot_system = robot_system
        ns = str(namespace or "dsr01").strip().strip("/")
        self._service_name = f"/{ns}/dsr_controller2/drl/drl_start"
        self.cli = self.node.create_client(DrlStart, self._service_name)
        wait_timeout_sec = float(os.environ.get("GRIPPER_DRL_WAIT_TIMEOUT_SEC", "25.0"))
        retry_interval_sec = float(os.environ.get("GRIPPER_DRL_WAIT_RETRY_SEC", "2.0"))
        deadline = time.monotonic() + max(1.0, wait_timeout_sec)

        self.node.get_logger().info(f"{self._service_name} 서비스 대기 중...")
        while not self.cli.wait_for_service(timeout_sec=max(0.2, retry_interval_sec)):
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"그리퍼 DRL 서비스 준비 시간 초과: {self._service_name} "
                    f"(timeout={wait_timeout_sec:.1f}s)"
                )
            self.node.get_logger().info("서비스가 아직 준비되지 않아 재시도합니다...")
        self.node.get_logger().info("그리퍼 컨트롤러 준비 완료")

    def _send_drl_script(self, code: str) -> bool:
        req = DrlStart.Request()
        req.robot_system = self.robot_system
        req.code = code
        future = self.cli.call_async(req)

        # NOTE:
        # node는 이미 외부 MultiThreadedExecutor에서 spin 중이므로,
        # 여기서 spin_until_future_complete()를 호출하면 executor 충돌/교착을 유발할 수 있다.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if future.done():
                break
            time.sleep(0.02)

        if not future.done():
            self.node.get_logger().error("서비스 호출 시간 초과(drl_start)")
            return False

        if future.exception() is not None:
            self.node.get_logger().error(f"서비스 호출 실패: {future.exception()}")
            return False

        res = future.result()
        ok = bool(getattr(res, "success", False))
        if not ok:
            head = str(code).strip().splitlines()
            snippet = head[0] if head else ""
            self.node.get_logger().error(
                f"drl_start 응답 실패(success=False, robot_system={self.robot_system}, code_head={snippet!r})"
            )
        return ok

    def initialize(self) -> bool:
        self.node.get_logger().info("그리퍼 연결 초기화를 시작합니다...")
        task_code = textwrap.dedent("""
            flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
            modbus_set_slaveid(1)
            flange_serial_write(modbus_fc06(256, 1))
            recv_check()
            flange_serial_write(modbus_fc06(275, 400))
            recv_check()
        """)
        init_script = f"{DRL_BASE_CODE}\n{task_code}"
        success = self._send_drl_script(init_script)
        if success:
            self.node.get_logger().info("그리퍼 연결 초기화 성공")
        else:
            self.node.get_logger().error("그리퍼 연결 초기화 실패(drl_start success=False)")
        return success

    def move(self, stroke: int) -> bool:
        dist_mm = _pulse_to_distance_mm(stroke)
        self.node.get_logger().info(f"그리퍼 이동 명령 전송(pulse={stroke}, distance={dist_mm:.2f}mm)")
        task_code = textwrap.dedent(f"""
            gripper_move({stroke})
        """)
        move_script = f"{DRL_BASE_CODE}\n{task_code}"
        success = self._send_drl_script(move_script)
        if success:
            self.node.get_logger().info(f"그리퍼 이동 명령 전송 성공(pulse={stroke}, distance={dist_mm:.2f}mm)")
        else:
            self.node.get_logger().error(f"그리퍼 이동 명령 전송 실패(pulse={stroke}, distance={dist_mm:.2f}mm)")
        return success

    def terminate(self) -> bool:
        self.node.get_logger().info("그리퍼 연결 종료를 시작합니다...")
        terminate_script = "flange_serial_close()"
        success = self._send_drl_script(terminate_script)
        if success:
            self.node.get_logger().info("그리퍼 연결 종료 성공")
        else:
            self.node.get_logger().error("그리퍼 연결 종료 실패")
        return success
