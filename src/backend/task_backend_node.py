import rclpy
from rclpy.node import Node
import time
import math
import os
import csv

import DR_init
try:
    from gripper_drl_controller import GripperController
except Exception:
    try:
        from dsr_example.simple.gripper_drl_controller import GripperController
    except Exception:
        GripperController = None

import threading
import queue
from rclpy.executors import MultiThreadedExecutor
from dsr_msgs2.msg import RobotState
try:
    from dsr_msgs2.msg import RobotStateRt
except Exception:
    RobotStateRt = None
from sensor_msgs.msg import JointState
try:
    from dsr_msgs2.srv import SetRobotControl
except Exception:
    SetRobotControl = None
try:
    from dsr_msgs2.srv import SetRobotMode
except Exception:
    SetRobotMode = None
try:
    from dsr_msgs2.srv import GetRobotMode
except Exception:
    GetRobotMode = None
try:
    from dsr_msgs2.srv import GetRobotSystem
except Exception:
    GetRobotSystem = None
try:
    from dsr_msgs2.srv import GetCurrentTcp
except Exception:
    GetCurrentTcp = None
try:
    from dsr_msgs2.srv import SetCurrentTcp
except Exception:
    SetCurrentTcp = None

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
VELOCITY, ACC = 100, 100
ROBOT_STATE_WATCHDOG_STALE_SEC = float(os.environ.get("ROBOT_STATE_WATCHDOG_STALE_SEC", "2.5"))
ROBOT_STATE_WATCHDOG_RESTART_INTERVAL_SEC = float(os.environ.get("ROBOT_STATE_WATCHDOG_RESTART_INTERVAL_SEC", "5.0"))

DR_MV_RA_NONE = 0
DR_MV_RA_DUPLICATE = 0
DR_MV_RA_OVERRIDE = 1

# Gripper mapping (distance mm <-> pulse)
# - opened distance: 109 mm -> pulse 0
# - opened distance:   0 mm -> pulse 740
GRIPPER_DISTANCE_MIN_MM = 0.0
GRIPPER_DISTANCE_MAX_MM = 109.0
GRIPPER_PULSE_MIN = 0
GRIPPER_PULSE_MAX = 740


def gripper_distance_mm_to_pulse(distance_mm: float) -> int:
    try:
        d = float(distance_mm)
    except Exception:
        d = GRIPPER_DISTANCE_MIN_MM
    d = max(GRIPPER_DISTANCE_MIN_MM, min(GRIPPER_DISTANCE_MAX_MM, d))
    ratio = (GRIPPER_DISTANCE_MAX_MM - d) / (GRIPPER_DISTANCE_MAX_MM - GRIPPER_DISTANCE_MIN_MM)
    pulse = GRIPPER_PULSE_MIN + ratio * (GRIPPER_PULSE_MAX - GRIPPER_PULSE_MIN)
    return int(round(pulse))


def gripper_pulse_to_distance_mm(pulse: int) -> float:
    try:
        p = int(pulse)
    except Exception:
        p = GRIPPER_PULSE_MIN
    p = max(GRIPPER_PULSE_MIN, min(GRIPPER_PULSE_MAX, p))
    ratio = (p - GRIPPER_PULSE_MIN) / float(GRIPPER_PULSE_MAX - GRIPPER_PULSE_MIN)
    distance_mm = GRIPPER_DISTANCE_MAX_MM - (ratio * (GRIPPER_DISTANCE_MAX_MM - GRIPPER_DISTANCE_MIN_MM))
    return float(distance_mm)


# Preserve previous behavior that used pulse-based constants (grab=300, release=0)
GRIPPER_GRAB_DISTANCE_MM = gripper_pulse_to_distance_mm(300)
GRIPPER_RELEASE_DISTANCE_MM = gripper_pulse_to_distance_mm(0)
HOME_POSJ = (0.00, -33.24, 104.14, -178.48, -22.49, 90.49)
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "..", ".."))
PARAM_DIR = os.path.join(PROJECT_ROOT, "config")
PARAM_FILE = os.path.join(PARAM_DIR, "parameter.csv")

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ROBOT_STATE_NAME_MAP = {
    0: "STATE_INITIALIZING",
    1: "STATE_STANDBY",
    2: "STATE_MOVING",
    3: "STATE_SAFE_OFF",
    4: "STATE_TEACHING",
    5: "STATE_SAFE_STOP",
    6: "STATE_EMERGENCY_STOP",
    7: "STATE_EMERGENCY_STOP",
    8: "STATE_HOMMING",
    9: "STATE_RECOVERY",
    10: "STATE_SAFE_STOP2",
    11: "STATE_SAFE_OFF2",
    12: "STATE_RESERVED1",
    13: "STATE_RESERVED2",
    14: "STATE_RESERVED4",
    15: "STATE_NOT_READY",
    16: "STATE_LAST",
}
# 시작 차단은 "실제 문제 상태"에서만 수행한다.
# MOVING(2)도 허용해서 pick -> place 연속 단계가 중간에 끊기지 않게 한다.
MOTION_BLOCKED_STATE_CODES = {3, 5, 6, 7, 10, 11, 15}  # SAFE_OFF/STOP/EMG/NOT_READY
PICK_PLACE_FIXED_ABC_ENV = os.environ.get("PICK_PLACE_FIXED_ABC", "").strip()
ROBOT_TOOL_NAME = os.environ.get("ROBOT_TOOL_NAME", "").strip()
ROBOT_TCP_NAME = os.environ.get("ROBOT_TCP_NAME", "").strip()
ROBOT_MODE_HINT = os.environ.get("BARTENDER_ROBOT_MODE_HINT", "").strip().upper()


def _parse_fixed_abc_env(raw: str):
    if not raw:
        return None
    try:
        parts = [float(v.strip()) for v in raw.split(",")]
        if len(parts) != 3:
            return None
        return parts
    except Exception:
        return None


DEFAULT_PICK_PLACE_ABC = _parse_fixed_abc_env(PICK_PLACE_FIXED_ABC_ENV)


def get_robot_state_name(state_code: int) -> str:
    return ROBOT_STATE_NAME_MAP.get(int(state_code), f"STATE_UNKNOWN_{int(state_code)}")


class RobotControllerNode(Node):
    def __init__(self, use_real_gripper=False):
        super().__init__("bartender_backend")
        self.use_real_gripper = use_real_gripper
        self._gripper_lock = threading.Lock()

        # NOTE: 카메라 관련 처리는 현재 비활성화 상태
        self._log_info("ROS 2 구독자 설정을 시작합니다.")

        self.gripper = None
        self.gripper_is_open = True
        if self.use_real_gripper:
            # 연결 타입/상태 확인 후 RobotBackend에서 지연 연결한다.
            self._log_info("실제 연결 확인 후 그리퍼 연결을 진행합니다.")
        else:
            self._log_info("VIRTUAL 모드: 그리퍼 통신을 비활성화합니다.")

        self._log_info("ROS 2 구독자와 로봇 컨트롤러가 초기화되었습니다.")

    def stop_camera(self):
        pass

    def _log_info(self, msg):
        self.get_logger().info(msg)
        print(f"[로봇] {msg}")

    def _log_error(self, msg):
        self.get_logger().error(msg)
        print(f"[오류] {msg}")

    def terminate_gripper(self):
        if not self.use_real_gripper:
            return
        with self._gripper_lock:
            gripper = self.gripper
        if gripper is not None:
            gripper.terminate()

    def ensure_gripper_connected(self):
        if not self.use_real_gripper:
            return True, "그리퍼 비활성화 설정"
        with self._gripper_lock:
            if self.gripper is not None:
                return True, "그리퍼 이미 연결됨"
            try:
                from DSR_ROBOT2 import wait
                if GripperController is None:
                    return False, "GripperController import 실패"
                self.gripper = GripperController(node=self, namespace=ROBOT_ID)
                wait(2)
                if not self.gripper.initialize():
                    self.gripper = None
                    return False, "그리퍼 초기화 실패"
                self._log_info("그리퍼를 활성화합니다.")
                self.gripper_is_open = True
                return True, "그리퍼 연결 성공"
            except Exception as e:
                self.gripper = None
                return False, f"그리퍼 세팅중에 실패하였습니다.: {e}"

    def move_gripper_manual(self, opening_distance_mm: float):
        try:
            target_mm = float(opening_distance_mm)
        except Exception:
            self._log_error(f"유효하지 않은 그리퍼 거리(mm) 값: {opening_distance_mm}")
            return False

        pulse = gripper_distance_mm_to_pulse(target_mm)

        if not self.use_real_gripper:
            self._log_info(
                f"VIRTUAL 모드: 그리퍼 수동 이동 생략(distance={target_mm:.2f}mm, pulse={pulse})"
            )
            return True

        with self._gripper_lock:
            gripper = self.gripper
        if gripper is None:
            self._log_error("그리퍼가 연결되지 않아 수동 이동을 수행할 수 없습니다.")
            return False

        try:
            self._log_info(f"그리퍼 수동 이동: distance={target_mm:.2f}mm, pulse={pulse}")
            ok = gripper.move(pulse)
            if not ok:
                self._log_error(f"그리퍼 move 명령 실패(distance={target_mm:.2f}mm, pulse={pulse})")
                return False
            self.gripper_is_open = target_mm >= (GRIPPER_RELEASE_DISTANCE_MM - 1e-6)
            return True
        except Exception as e:
            self._log_error(f"그리퍼 수동 이동 중 오류: {e}")
            return False

    def _motion_ok(self, ret, command_name):
        # DSR API는 환경에 따라 bool/int/None을 반환한다.
        if ret is None:
            return True
        if isinstance(ret, bool):
            ok = ret
        elif isinstance(ret, (int, float)):
            ok = ret >= 0
        else:
            ok = True
        if not ok:
            self._log_error(f"{command_name} 명령 실패(반환값={ret})")
        return ok

    def _fmt_list3(self, values):
        return "[" + ", ".join(f"{float(v):.3f}" for v in values) + "]"

    def _cart_motion_profile(self, vel=None, acc=None):
        v = VELOCITY if vel is None else float(vel)
        a = ACC if acc is None else float(acc)
        return [v, v], [a, a]

    def move_to_home_pose(self, home_posj=None, vel=None, acc=None):
        from DSR_ROBOT2 import movej, wait
        from DR_common2 import posj
        try:
            target = HOME_POSJ if home_posj is None else tuple(float(v) for v in list(home_posj)[:6])
            p_start = posj(*target)
            self.get_logger().info("초기 자세로 복귀합니다.")
            v = VELOCITY if vel is None else float(vel)
            a = ACC if acc is None else float(acc)
            ret = movej(p_start, v, a)
            if not self._motion_ok(ret, "movej(초기자세 복귀)"):
                return False
            wait(0.5)
            return True
        except Exception as e:
            self.get_logger().error(f"초기 자세 복귀 중 오류 발생: {e}")
            return False

    def move_to_vision_pose(self, x, y, z, abc=None):
        from DSR_ROBOT2 import movel, wait
        from DR_common2 import posx
        try:
            if abc is None or len(abc) < 3:
                abc = [0.0, 0.0, 0.0]
            target = [float(x), float(y), float(z), float(abc[0]), float(abc[1]), float(abc[2])]
            velx, accx = self._cart_motion_profile()
            self.get_logger().info(
                f"비전 좌표 이동: XYZ=[{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}], ABC={self._fmt_list3(abc)}"
            )
            ret = movel(posx(target), vel=velx, acc=accx, radius=0.0, ra=DR_MV_RA_DUPLICATE)
            if not self._motion_ok(ret, "movel(비전 좌표 이동)"):
                return False
            wait(0.5)
            return True
        except Exception as e:
            self.get_logger().error(f"비전 좌표 이동 중 오류 발생: {e}")
            return False

    def move_to_cartesian_pose(self, x, y, z, a, b, c, vel=None, acc=None):
        from DSR_ROBOT2 import movel, wait
        from DR_common2 import posx
        try:
            target = [float(x), float(y), float(z), float(a), float(b), float(c)]
            velx, accx = self._cart_motion_profile(vel=vel, acc=acc)
            self.get_logger().info(
                f"카테시안 이동: XYZABC=[{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}, "
                f"{target[3]:.3f}, {target[4]:.3f}, {target[5]:.3f}], "
                f"vel=[{velx[0]:.3f}, {velx[1]:.3f}], acc=[{accx[0]:.3f}, {accx[1]:.3f}]"
            )
            ret = movel(posx(target), vel=velx, acc=accx, radius=0.0, ra=DR_MV_RA_DUPLICATE)
            if not self._motion_ok(ret, "movel(카테시안 이동)"):
                return False
            wait(0.5)
            return True
        except Exception as e:
            self.get_logger().error(f"카테시안 이동 중 오류 발생: {e}")
            return False

    def move_to_joint_pose(self, j1, j2, j3, j4, j5, j6, vel=None, acc=None):
        from DSR_ROBOT2 import movej, wait
        from DR_common2 import posj
        try:
            target = [float(j1), float(j2), float(j3), float(j4), float(j5), float(j6)]
            self.get_logger().info(
                f"조인트 이동: J=[{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}, "
                f"{target[3]:.3f}, {target[4]:.3f}, {target[5]:.3f}]"
            )
            v = VELOCITY if vel is None else float(vel)
            a = ACC if acc is None else float(acc)
            ret = movej(posj(*target), v, a)
            if not self._motion_ok(ret, "movej(조인트 이동)"):
                return False
            wait(0.5)
            return True
        except Exception as e:
            self.get_logger().error(f"조인트 이동 중 오류 발생: {e}")
            return False

    def pick_move_robot_and_control_gripper(self, x, y, z, width, depth, abc=None):
        from DSR_ROBOT2 import movel, wait
        from DR_common2 import posx
        try:
            if abc is None or len(abc) < 3:
                abc = [0.0, 0.0, 0.0]
            target_pos_list_up = [x, y, z + 100.0, float(abc[0]), float(abc[1]), float(abc[2])]
            target_pos_list = [x, y, z, float(abc[0]), float(abc[1]), float(abc[2])]
            velx, accx = self._cart_motion_profile()
            self.get_logger().info(f"적용 ABC={self._fmt_list3(abc)}")

            self.get_logger().info(f"{self._fmt_list3(target_pos_list_up)} / 픽-업 위치 이동합니다.")
            ret = movel(posx(target_pos_list_up), vel=velx, acc=accx, radius=20.0, ra=DR_MV_RA_DUPLICATE)
            if not self._motion_ok(ret, "movel(픽-업)"):
                return False

            self.get_logger().info(f"{self._fmt_list3(target_pos_list)} / 픽 위치로 이동합니다.")
            ret = movel(posx(target_pos_list), vel=velx, acc=accx, radius=0, ra=DR_MV_RA_DUPLICATE)
            if not self._motion_ok(ret, "movel(픽-다운)"):
                return False
            wait(0.5)

            if self.use_real_gripper:
                if self.gripper is None:
                    self._log_error("REAL 모드인데 그리퍼가 준비되지 않아 픽을 중단합니다.")
                    return False
                self.get_logger().info("그리퍼를 그랩합니다.")
                self.gripper.move(gripper_distance_mm_to_pulse(GRIPPER_GRAB_DISTANCE_MM))
                wait(2)
            else:
                self.get_logger().info("VIRTUAL 모드: 그리퍼 그랩 동작 생략")

            self.get_logger().info("오브젝트를 잡았습니다.")

            self.get_logger().info(f"{self._fmt_list3(target_pos_list_up)} / 픽-업 위치로 이동합니다.")
            ret = movel(posx(target_pos_list_up), vel=velx, acc=accx, radius=5.0, ra=DR_MV_RA_DUPLICATE)
            if not self._motion_ok(ret, "movel(픽 리프트)"):
                return False
            return True
        except Exception as e:
            self.get_logger().error(f"로봇 이동 및 그리퍼 제어 중 오류 발생: {e}")
            return False

    def place_move_robot_and_control_gripper(self, x, y, z, width, depth, abc=None, home_posj=None):
        from DSR_ROBOT2 import movel, wait
        from DR_common2 import posx
        try:
            if abc is None or len(abc) < 3:
                abc = [0.0, 0.0, 0.0]
            target_pos_list_up = [x, y, z + 100.0, float(abc[0]), float(abc[1]), float(abc[2])]
            target_pos_list = [x, y, z, float(abc[0]), float(abc[1]), float(abc[2])]
            velx, accx = self._cart_motion_profile()
            self.get_logger().info(f"적용 ABC={self._fmt_list3(abc)}")

            self.get_logger().info(f"{self._fmt_list3(target_pos_list_up)} / 플레이스-업 위치로 이동합니다. ")
            ret = movel(posx(target_pos_list_up), vel=velx, acc=accx, radius=20.0, ra=DR_MV_RA_DUPLICATE)
            if not self._motion_ok(ret, "movel(플레이스-업)"):
                return False

            self.get_logger().info(f"{self._fmt_list3(target_pos_list)} / 플레이스-다운 위치로 이동합니다. ")
            ret = movel(posx(target_pos_list), vel=velx, acc=accx, radius=0, ra=DR_MV_RA_DUPLICATE)
            if not self._motion_ok(ret, "movel(플레이스-다운)"):
                return False
            wait(0.5)

            if self.use_real_gripper:
                if self.gripper is None:
                    self._log_error("REAL 모드인데 그리퍼가 준비되지 않아 플레이스를 중단합니다.")
                    return False
                self.get_logger().info("그리퍼를 릴리즈합니다.")
                self.gripper.move(gripper_distance_mm_to_pulse(GRIPPER_RELEASE_DISTANCE_MM))
                wait(2)
            else:
                self.get_logger().info("VIRTUAL 모드: 그리퍼 릴리즈 동작 생략")

            self.get_logger().info(f"{self._fmt_list3(target_pos_list_up)} / 플레이스-업 위치로 이동합니다: ")
            ret = movel(posx(target_pos_list_up), vel=velx, acc=accx, radius=0.0, ra=DR_MV_RA_DUPLICATE)
            if not self._motion_ok(ret, "movel(플레이스 리프트)"):
                return False

            if not self.move_to_home_pose(home_posj=home_posj):
                return False

            return True
        except Exception as e:
            self.get_logger().error(f"로봇 이동 및 그리퍼 제어 중 오류 발생: {e}")
            return False


class RobotBackend:
    def __init__(self, use_real_gripper=False):
        self.use_real_gripper = use_real_gripper
        self._cmd_q = queue.Queue(maxsize=1)
        self._busy_event = threading.Event()
        self._stop_event = threading.Event()
        self._ready_event = threading.Event()
        self._started = False
        self._last_error = None

        self._spin_thread = None
        self._worker_thread = None
        self._gripper_ready_event = threading.Event()
        self._gripper_init_started = False
        self._gripper_init_error = None
        self._gripper_init_lock = threading.Lock()

        self._executor = None
        self._dsr_node = None
        self._mode_node = None
        self.robot_controller = None
        self._position_lock = threading.Lock()
        self._last_positions = None
        self._position_seen_at = None
        self._robot_state_lock = threading.Lock()
        self._robot_state_code = None
        self._robot_state_name = ""
        self._robot_state_raw_name = ""
        self._robot_state_seen_at = None
        self._state_sub = None
        self._joint_state_sub = None
        self._robot_mode_sub = None
        self._robot_stream_enabled = True
        self._state_topic = f"/{ROBOT_ID}/state"
        self._joint_state_topics = [f"/{ROBOT_ID}/joint_states", f"/{ROBOT_ID}/gz/joint_states"]
        self._source_lock = threading.Lock()
        self._source_scan_interval_sec = 1.0
        self._last_source_scan_at = 0.0
        self._state_watchdog_lock = threading.Lock()
        self._last_state_watchdog_restart_at = 0.0
        self._last_mode_source_scan_at = 0.0
        self._mode_source_scan_interval_sec = 1.0
        self._set_robot_control_clients = {}
        self._set_robot_mode_clients = {}
        self._get_robot_mode_clients = {}
        self._robot_mode_service_lock = threading.Lock()
        self._set_current_tcp_clients = {}
        self._get_current_tcp_clients = {}
        self._tcp_service_lock = threading.Lock()
        self._get_robot_system_clients = {}
        self._startup_tool_tcp_config_done = False
        self._startup_gripper_init_enqueued = False
        self._connection_mode = "UNKNOWN"
        self._connection_mode_seen_at = None
        self._robot_mode_lock = threading.Lock()
        self._robot_mode_value = None
        self._robot_mode_seen_at = None
        self._robot_mode_initialized_once = False
        self._control_mode_value = None
        self._control_mode_seen_at = None
        self._tcp_state_lock = threading.Lock()
        self._current_tcp_name = None
        self._current_tcp_seen_at = None
        self._last_tcp_sync_at = 0.0
        self._fixed_abc_lock = threading.Lock()
        self._fixed_pick_place_abc = list(DEFAULT_PICK_PLACE_ABC) if DEFAULT_PICK_PLACE_ABC is not None else None
        self._home_posj_lock = threading.Lock()
        self._home_posj = tuple(float(v) for v in list(HOME_POSJ)[:6])
        self._load_home_posj_from_file()

    def _parse_home_row(self, row):
        vals = None
        if not row:
            return None
        if len(row) >= 7 and str(row[0]).strip().lower() in ("home_posj", "home", "home_j"):
            vals = row[1:7]
        elif len(row) >= 6:
            vals = row[:6]
        if vals is None:
            return None
        try:
            parsed = tuple(float(v) for v in vals[:6])
        except Exception:
            return None
        if len(parsed) != 6:
            return None
        return parsed

    def _load_param_rows(self):
        rows = {}
        path = PARAM_FILE
        if not os.path.exists(path):
            return rows
        try:
            with open(path, "r", encoding="utf-8", newline="") as f:
                for row in csv.reader(f):
                    if not row:
                        continue
                    key = str(row[0]).strip()
                    if not key or key.lower() == "name":
                        continue
                    rows[key] = [str(v).strip() for v in row[1:]]
        except Exception:
            return {}
        return rows

    def _write_param_rows(self, rows):
        os.makedirs(PARAM_DIR, exist_ok=True)
        with open(PARAM_FILE, "w", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["name", "v1", "v2", "v3", "v4", "v5", "v6"])
            for key in sorted(rows.keys()):
                writer.writerow([key, *rows[key]])

    def _load_home_posj_from_file(self):
        path = PARAM_FILE
        if not os.path.exists(path):
            return False, "홈위치 파일이 없어 기본값 사용"
        try:
            rows = self._load_param_rows()
            for key, values in rows.items():
                row = [key, *values]
                parsed = self._parse_home_row(row)
                if parsed is None:
                    continue
                with self._home_posj_lock:
                    self._home_posj = parsed
                return True, f"홈위치 로드 성공: {path}"
            return False, "홈위치 파일 파싱 실패(유효 데이터 없음)"
        except Exception as e:
            return False, f"홈위치 파일 로드 실패: {e}"

    def _save_home_posj_to_file(self):
        rows = self._load_param_rows()
        with self._home_posj_lock:
            vals = tuple(float(v) for v in self._home_posj)
        rows["home_posj"] = [f"{v:.6f}" for v in vals]
        self._write_param_rows(rows)
        return PARAM_FILE

    def get_home_posj(self):
        with self._home_posj_lock:
            return tuple(self._home_posj)

    def set_home_posj(self, values):
        try:
            vals = tuple(float(v) for v in list(values)[:6])
        except Exception:
            return False, "홈위치 저장 실패: 숫자 6개 입력이 필요합니다."
        if len(vals) != 6:
            return False, "홈위치 저장 실패: J1~J6 6개 값이 필요합니다."
        with self._home_posj_lock:
            self._home_posj = vals
        try:
            path = self._save_home_posj_to_file()
            return True, f"홈위치 저장 완료: {path}"
        except Exception as e:
            return False, f"홈위치 파일 저장 실패: {e}"

    def _report_progress(self, callback, percent, message):
        if callback is not None:
            callback(int(percent), str(message))

    def _apply_tool_tcp_config(self):
        # Optional startup hook:
        # - ROBOT_TOOL_NAME: preconfigured tool name in controller
        # - ROBOT_TCP_NAME: preconfigured TCP name in controller
        tool_name = str(ROBOT_TOOL_NAME).strip()
        tcp_name = str(ROBOT_TCP_NAME).strip()
        if not tool_name and not tcp_name:
            return True, "툴/TCP 자동 적용 비활성화(환경변수 미설정)"
        try:
            from DSR_ROBOT2 import set_tool, set_tcp
        except Exception as e:
            return False, f"set_tool/set_tcp import 실패: {e}"

        if tool_name:
            try:
                ret_tool = set_tool(tool_name)
                if self.robot_controller is not None:
                    self.robot_controller._log_info(f"Tool 적용: {tool_name} (ret={ret_tool})")
            except Exception as e:
                return False, f"Tool 적용 실패({tool_name}): {e}"
        if tcp_name:
            try:
                ret_tcp = set_tcp(tcp_name)
                if self.robot_controller is not None:
                    self.robot_controller._log_info(f"TCP 적용: {tcp_name} (ret={ret_tcp})")
            except Exception as e:
                return False, f"TCP 적용 실패({tcp_name}): {e}"
        return True, "툴/TCP 자동 적용 완료"

    def start(self, progress_callback=None):
        if self._started:
            return

        self._report_progress(progress_callback, 5, "ROS 초기화 중...")
        rclpy.init(args=None)

        self._report_progress(progress_callback, 15, "DSR 노드 생성 중...")
        self._dsr_node = rclpy.create_node("bartender_robot_app", namespace=ROBOT_ID)
        # 모드 set/get 서비스 전용 노드(무브 API와 분리)
        self._mode_node = rclpy.create_node("bartender_robot_mode", namespace=ROBOT_ID)
        # Avoid class-scope name-mangling on double-underscore attributes.
        setattr(DR_init, "__dsr__node", self._dsr_node)

        try:
            self._report_progress(progress_callback, 30, "DSR 라이브러리 로딩 중...")
            from DSR_ROBOT2 import get_current_posx, movel, wait, movej
            from DR_common2 import posx, posj
        except ImportError as e:
            self._last_error = f"DSR_ROBOT2 라이브러리를 임포트할 수 없습니다: {e}"
            rclpy.shutdown()
            raise

        self._report_progress(progress_callback, 45, "로봇 컨트롤러 생성 중...")
        self.robot_controller = RobotControllerNode(use_real_gripper=self.use_real_gripper)

        self._report_progress(progress_callback, 60, "ROS executor 시작 중...")
        self._executor = MultiThreadedExecutor(num_threads=3)
        self._executor.add_node(self.robot_controller)
        self._executor.add_node(self._dsr_node)
        self._executor.add_node(self._mode_node)

        self._spin_thread = threading.Thread(target=self._spin_bg, daemon=True)
        self._spin_thread.start()

        self._detect_connection_mode_once(progress_callback=progress_callback)
        self._setup_position_source(progress_callback, wait_timeout_sec=0.15)
        self._setup_robot_mode_source(progress_callback)
        self._report_gripper_startup_plan(progress_callback=progress_callback)
        self._report_progress(progress_callback, 99, "명령 워커 시작 중...")

        self._worker_thread = threading.Thread(target=self._command_loop, daemon=True)
        self._worker_thread.start()

        self._ready_event.set()
        self._started = True
        self._report_progress(progress_callback, 100, "초기화 완료")

    def _spin_bg(self):
        last_warn_at = 0.0
        while not self._stop_event.is_set():
            try:
                if self._executor is None:
                    time.sleep(0.05)
                    continue
                if not rclpy.ok():
                    break
                self._executor.spin_once(timeout_sec=0.1)
            except Exception as e:
                msg = str(e)
                # 런타임에 subscription teardown이 겹치면 일시적으로 발생할 수 있다.
                # 이 예외로 spin thread가 종료되면 전체 상태 수신이 끊기므로 계속 진행한다.
                if ("Destroyable" in msg) or ("destruction was requested" in msg):
                    if self._stop_event.is_set():
                        break
                    time.sleep(0.02)
                    continue
                now = time.monotonic()
                if (now - last_warn_at) > 1.0:
                    print(f"executor.spin_once 예외: {e}")
                    last_warn_at = now
                time.sleep(0.05)

    def _initialize_gripper_sequence(self, progress_callback=None):
        if not self.use_real_gripper:
            self.robot_controller._log_info("그리퍼 비활성화: 초기화 시퀀스 생략")
            self._report_progress(progress_callback, 97, "그리퍼 비활성화")
            return
        if str(self._connection_mode).upper() != "REAL":
            self.robot_controller._log_info(
                f"연결타입={self._connection_mode}: 그리퍼 연결/초기화 시퀀스 생략"
            )
            self._report_progress(progress_callback, 97, "REAL 연결 아님: 그리퍼 생략")
            return
        try:
            ok_mode, now_mode, msg_mode = self._call_get_robot_mode(timeout_sec=1.0)
            if not ok_mode or now_mode is None:
                raise RuntimeError(f"그리퍼 초기화 전 모드 조회 실패: {msg_mode}")
            if int(now_mode) != 1:
                raise RuntimeError(f"그리퍼 초기화 불가: 현재 로봇모드={int(now_mode)} (오토모드(1) 필요)")
            ok, msg = self.robot_controller.ensure_gripper_connected()
            if not ok:
                raise RuntimeError(msg)
            # from DSR_ROBOT2 import wait
            # init_dist_open = gripper_pulse_to_distance_mm(0)
            # init_dist_mid = gripper_pulse_to_distance_mm(500)
            # wait(5)
            # self.robot_controller._log_info(
            #     f"그리퍼 초기화 이동: {init_dist_open:.2f}mm, pulse={gripper_distance_mm_to_pulse(init_dist_open)}"
            # )
            # self.robot_controller.gripper.move(gripper_distance_mm_to_pulse(init_dist_open))
            # self._report_progress(progress_callback, 97, "그리퍼 초기화 1/5...")
            # wait(4)
            # self.robot_controller.gripper.move(gripper_distance_mm_to_pulse(init_dist_mid))
            # self._report_progress(progress_callback, 98, "그리퍼 초기화 2/5...")
            # wait(4)
            # self.robot_controller.gripper.move(gripper_distance_mm_to_pulse(init_dist_open))
            # self._report_progress(progress_callback, 98, "그리퍼 초기화 3/5...")
            # wait(4)
            # self.robot_controller.gripper.move(gripper_distance_mm_to_pulse(init_dist_mid))
            # self._report_progress(progress_callback, 99, "그리퍼 초기화 4/5...")
            # wait(4)
            # self.robot_controller.gripper.move(gripper_distance_mm_to_pulse(init_dist_open))
            # self._report_progress(progress_callback, 99, "그리퍼 초기화 5/5...")
            # wait(4)
            # self.robot_controller._log_info("그리퍼를 초기화 이동완료")
            self.robot_controller._log_info("그리퍼 초기화 이동 시퀀스는 주석 처리되어 건너뜁니다.")
        except Exception as e:
            self.robot_controller._log_error(f"그리퍼 초기화 시퀀스 실패: {e}")
            raise RuntimeError(f"실제 모드 그리퍼 초기화 시퀀스 실패: {e}") from e

    def _report_gripper_startup_plan(self, progress_callback=None):
        if not self.use_real_gripper:
            self._gripper_ready_event.set()
            self._report_progress(progress_callback, 97, "그리퍼 비활성화")
            return
        if str(self._connection_mode).upper() != "REAL":
            self._gripper_ready_event.set()
            self._report_progress(progress_callback, 97, f"{self._connection_mode}: 그리퍼 생략")
            return
        self._gripper_ready_event.clear()
        self._report_progress(progress_callback, 97, "그리퍼 초기화: 로봇 연결 후 실행")

    def _enqueue_gripper_init_if_needed(self):
        if not self._is_real_gripper_required():
            return True, "그리퍼 초기화 대상 아님"
        with self._gripper_init_lock:
            if self._gripper_init_started:
                return True, "그리퍼 초기화가 이미 진행/등록됨"
            self._gripper_init_started = True
            self._gripper_init_error = None
            self._gripper_ready_event.clear()
        try:
            self._cmd_q.put_nowait(("init_gripper", None))
            if self.robot_controller is not None:
                self.robot_controller._log_info("그리퍼 초기화를 로봇 명령 워커에 등록했습니다.")
            return True, "그리퍼 초기화 워커 등록 완료"
        except queue.Full:
            self._gripper_init_error = "그리퍼 초기화 대기열 등록 실패(큐 가득 참)"
            self._last_error = self._gripper_init_error
            if self.robot_controller is not None:
                self.robot_controller._log_error(self._gripper_init_error)
            return False, self._gripper_init_error

    def _retry_gripper_init_after_reset(self):
        if not self._is_real_gripper_required():
            return True, "그리퍼 초기화 대상 아님"
        with self._gripper_init_lock:
            self._gripper_init_started = False
            self._gripper_init_error = None
            self._gripper_ready_event.clear()
        return self._enqueue_gripper_init_if_needed()

    def _is_real_gripper_required(self):
        return bool(self.use_real_gripper) and str(self._connection_mode).upper() == "REAL"

    def _check_gripper_precondition(self):
        if not self._is_real_gripper_required():
            return True, ""
        if self._gripper_ready_event.is_set():
            return True, ""
        if self._gripper_init_error:
            return False, f"그리퍼 초기화 실패: {self._gripper_init_error}"
        return False, "그리퍼 초기화 진행 중입니다. 잠시 후 다시 시도하세요."

    def _is_robot_connection_ready_for_startup(self):
        if str(self._connection_mode).upper() != "REAL":
            return True
        with self._robot_state_lock:
            return self._robot_state_seen_at is not None

    def _try_run_startup_robot_init(self):
        if self.robot_controller is None:
            return
        if not self._is_robot_connection_ready_for_startup():
            return

        if not self._startup_tool_tcp_config_done:
            ok_cfg, msg_cfg = self._apply_tool_tcp_config()
            if ok_cfg:
                self._startup_tool_tcp_config_done = True
                self.robot_controller._log_info(msg_cfg)
            else:
                self.robot_controller._log_error(msg_cfg)
                return

        if self._startup_gripper_init_enqueued:
            return
        if not self._is_real_gripper_required():
            self._startup_gripper_init_enqueued = True
            return

        if not self._robot_mode_initialized_once:
            try:
                self.sync_robot_mode_once(force=False, timeout_sec=0.6)
            except Exception:
                return
            if not self._robot_mode_initialized_once:
                return

        ok_gripper, msg_gripper = self._enqueue_gripper_init_if_needed()
        if ok_gripper:
            self._startup_gripper_init_enqueued = True
        elif self.robot_controller is not None:
            self.robot_controller._log_error(msg_gripper)

    def _command_loop(self):
        next_mode_sync_try_at = 0.0
        while not self._stop_event.is_set():
            try:
                cmd, val = self._cmd_q.get(timeout=0.1)
            except queue.Empty:
                now = time.monotonic()
                self._try_upgrade_to_state_source()
                self._try_upgrade_robot_mode_source()
                self._try_run_startup_robot_init()
                if (not self._robot_mode_initialized_once) and now >= next_mode_sync_try_at:
                    try:
                        if rclpy.ok() and (not self._stop_event.is_set()):
                            self.sync_robot_mode_once(force=False, timeout_sec=0.6)
                    except Exception:
                        if self._stop_event.is_set() or (not rclpy.ok()):
                            break
                    next_mode_sync_try_at = now + 1.0
                continue

            if cmd == "quit":
                break

            if cmd == "pick_place":
                if self._busy_event.is_set():
                    continue

                self._busy_event.set()
                try:
                    state_ok, state_msg = self._check_motion_precondition(max_state_age_sec=1.5)
                    if not state_ok:
                        self.robot_controller._log_error(f"작업 중단: {state_msg}")
                        continue

                    value = val

                    final_x = 500
                    final_y = 0 + value * 20
                    final_z = 200
                    width = 0
                    depth = 0
                    abc = self._get_pick_place_abc()
                    ok_pick = self.robot_controller.pick_move_robot_and_control_gripper(
                        final_x, final_y, final_z, width, depth, abc=abc
                    )
                    if not ok_pick:
                        self.robot_controller._log_error("픽 동작 실패로 시퀀스를 중단합니다.")
                        continue

                    state_ok, state_msg = self._check_motion_precondition(max_state_age_sec=1.5)
                    if not state_ok:
                        self.robot_controller._log_error(f"플레이스 전 중단: {state_msg}")
                        continue

                    final_x = 300
                    final_y = 0 + value * 20
                    final_z = 200
                    width = 0
                    depth = 0
                    abc = self._get_pick_place_abc()
                    ok_place = self.robot_controller.place_move_robot_and_control_gripper(
                        final_x, final_y, final_z, width, depth, abc=abc, home_posj=self.get_home_posj()
                    )
                    if not ok_place:
                        self.robot_controller._log_error("플레이스 동작 실패로 시퀀스를 중단합니다.")
                finally:
                    self._busy_event.clear()
            elif cmd == "move_home":
                if self._busy_event.is_set():
                    continue

                self._busy_event.set()
                try:
                    vel = None
                    acc = None
                    if isinstance(val, (list, tuple)) and len(val) >= 2:
                        vel, acc = val[:2]
                    ok_home = self.robot_controller.move_to_home_pose(home_posj=self.get_home_posj(), vel=vel, acc=acc)
                    if not ok_home:
                        self.robot_controller._log_error("초기 자세 복귀 실패")
                finally:
                    self._busy_event.clear()
            elif cmd == "move_vision_point":
                if self._busy_event.is_set():
                    continue

                self._busy_event.set()
                try:
                    x, y, z, dwell_sec = val
                    state_ok, state_msg = self._check_motion_precondition(max_state_age_sec=2.0)
                    if not state_ok:
                        self.robot_controller._log_error(f"비전 이동 중단: {state_msg}")
                        continue
                    abc = self._get_pick_place_abc()
                    ok_move = self.robot_controller.move_to_vision_pose(x, y, z, abc=abc)
                    if not ok_move:
                        self.robot_controller._log_error("비전 좌표 이동 실패")
                        continue
                    time.sleep(max(0.0, float(dwell_sec)))
                    ok_home = self.robot_controller.move_to_home_pose(home_posj=self.get_home_posj())
                    if not ok_home:
                        self.robot_controller._log_error("비전 이동 후 홈 복귀 실패")
                finally:
                    self._busy_event.clear()
            elif cmd == "move_cartesian":
                if self._busy_event.is_set():
                    continue

                self._busy_event.set()
                try:
                    vel = None
                    acc = None
                    if isinstance(val, (list, tuple)) and len(val) >= 8:
                        x, y, z, a, b, c, vel, acc = val[:8]
                    else:
                        x, y, z, a, b, c = val
                    state_ok, state_msg = self._check_motion_precondition(max_state_age_sec=2.0)
                    if not state_ok:
                        self.robot_controller._log_error(f"카테시안 이동 중단: {state_msg}")
                        continue
                    ok_move = self.robot_controller.move_to_cartesian_pose(x, y, z, a, b, c, vel=vel, acc=acc)
                    if not ok_move:
                        self.robot_controller._log_error("카테시안 이동 실패")
                finally:
                    self._busy_event.clear()
            elif cmd == "move_joint":
                if self._busy_event.is_set():
                    continue

                self._busy_event.set()
                try:
                    vel = None
                    acc = None
                    if isinstance(val, (list, tuple)) and len(val) >= 8:
                        j1, j2, j3, j4, j5, j6, vel, acc = val[:8]
                    else:
                        j1, j2, j3, j4, j5, j6 = val
                    state_ok, state_msg = self._check_motion_precondition(max_state_age_sec=2.0)
                    if not state_ok:
                        self.robot_controller._log_error(f"조인트 이동 중단: {state_msg}")
                        continue
                    ok_move = self.robot_controller.move_to_joint_pose(j1, j2, j3, j4, j5, j6, vel=vel, acc=acc)
                    if not ok_move:
                        self.robot_controller._log_error("조인트 이동 실패")
                finally:
                    self._busy_event.clear()
            elif cmd == "gripper_move":
                if self._busy_event.is_set():
                    continue

                self._busy_event.set()
                try:
                    distance_mm = float(val)
                    gripper_ok, gripper_msg = self._check_gripper_precondition()
                    if not gripper_ok:
                        self.robot_controller._log_error(gripper_msg)
                        continue
                    ok_gripper = self.robot_controller.move_gripper_manual(distance_mm)
                    if not ok_gripper:
                        self.robot_controller._log_error(f"그리퍼 수동 이동 실패(distance={distance_mm:.2f}mm)")
                finally:
                    self._busy_event.clear()
            elif cmd == "init_gripper":
                if self._busy_event.is_set():
                    continue
                self._busy_event.set()
                try:
                    if self.robot_controller is not None:
                        self.robot_controller._log_info("로봇 워커에서 그리퍼 초기화를 시작합니다.")
                    try:
                        self._initialize_gripper_sequence(progress_callback=None)
                        self._gripper_init_error = None
                        self._gripper_ready_event.set()
                    except Exception as e:
                        self._gripper_init_error = str(e)
                        self._last_error = str(e)
                        if self.robot_controller is not None:
                            self.robot_controller._log_error(f"그리퍼 초기화 실패: {e}")
                finally:
                    self._busy_event.clear()

    def _get_pick_place_abc(self):
        with self._fixed_abc_lock:
            if self._fixed_pick_place_abc is not None:
                return list(self._fixed_pick_place_abc)

        abc = None
        source_text = "cache"
        posx_live, sol_live, err_live = self.get_current_posx_live()
        if err_live is None and posx_live is not None and len(posx_live) >= 6:
            abc = [float(posx_live[3]), float(posx_live[4]), float(posx_live[5])]
            source_text = "get_current_posx" if sol_live is None else f"get_current_posx(sol={int(sol_live)})"
        else:
            abc = self._get_cached_abc()
            if self.robot_controller is not None and err_live is not None:
                self.robot_controller._log_info(f"픽앤플레이스 기준 자세 서비스 조회 실패 -> 캐시 사용: {err_live}")

        with self._fixed_abc_lock:
            if self._fixed_pick_place_abc is None:
                self._fixed_pick_place_abc = list(abc)
                if self.robot_controller is not None:
                    self.robot_controller._log_info(
                        f"픽앤플레이스 기준 자세(ABC) 고정({source_text}): "
                        f"{self.robot_controller._fmt_list3(self._fixed_pick_place_abc)}"
                    )
            return list(self._fixed_pick_place_abc)

    def _get_cached_abc(self):
        with self._position_lock:
            if self._last_positions is None:
                return [0.0, 0.0, 0.0]
            posx = self._last_positions[1]
            if len(posx) < 6:
                return [0.0, 0.0, 0.0]
            return [posx[3], posx[4], posx[5]]

    def _on_robot_state(self, msg):
        if not self._robot_stream_enabled:
            return
        self._update_robot_state(msg.robot_state, msg.robot_state_str)
        if self._joint_state_sub is not None:
            with self._source_lock:
                if self._joint_state_sub is not None:
                    self._destroy_sub_safe(self._joint_state_sub)
                    self._joint_state_sub = None
        try:
            with self._robot_mode_lock:
                # 제어모드는 RT값이 아닌 RobotState.actual_mode를 기준으로만 사용한다.
                self._control_mode_value = int(getattr(msg, "actual_mode", 0))
                self._control_mode_seen_at = time.monotonic()
        except Exception:
            pass
        # Use only current_posx as the single source of TCP-applied position.
        posx = list(msg.current_posx)
        if len(posx) < 6:
            posx = (posx + [0.0] * 6)[:6]
        with self._position_lock:
            self._last_positions = (list(msg.current_posj), posx)
            self._position_seen_at = time.monotonic()

    def _update_robot_state(self, state_code, state_name_hint=""):
        state_code = int(state_code)
        raw_state_name = str(state_name_hint).strip() if state_name_hint else ""
        state_name = raw_state_name if raw_state_name else get_robot_state_name(state_code)
        with self._robot_state_lock:
            self._robot_state_code = state_code
            self._robot_state_name = state_name
            self._robot_state_raw_name = raw_state_name
            self._robot_state_seen_at = time.monotonic()

    def _on_joint_state(self, msg):
        if not self._robot_stream_enabled:
            return
        if len(msg.position) < 6:
            return
        raw_posj = None
        joint_names = [str(name).strip() for name in list(getattr(msg, "name", []) or [])]
        if len(joint_names) >= 6:
            name_to_pos = {}
            for idx, name in enumerate(joint_names):
                if idx >= len(msg.position):
                    break
                name_to_pos[name] = msg.position[idx]
            ordered = []
            for joint_idx in range(1, 7):
                key = f"joint_{joint_idx}"
                if key not in name_to_pos:
                    ordered = []
                    break
                ordered.append(name_to_pos[key])
            if len(ordered) == 6:
                raw_posj = ordered
        if raw_posj is None:
            raw_posj = list(msg.position[:6])
        # sensor_msgs/JointState follows ROS convention (rad for revolute joints).
        # Convert to deg to match Doosan RobotState/current_posj display convention.
        if max(abs(v) for v in raw_posj) <= (2.0 * math.pi + 0.5):
            posj = [math.degrees(v) for v in raw_posj]
        else:
            posj = raw_posj
        with self._position_lock:
            if self._last_positions is None:
                posx = [0.0] * 6
            else:
                posx = self._last_positions[1]
            self._last_positions = (posj, posx)
            self._position_seen_at = time.monotonic()

    def _on_robot_mode_rt(self, msg):
        if not self._robot_stream_enabled:
            return
        _ = msg
        with self._robot_mode_lock:
            # 로봇모드(robot_mode)는 서비스 1회 조회/모드 변경 시 갱신으로 유지한다.
            # 제어모드는 RobotState.actual_mode를 사용하므로 RT control_mode는 무시한다.
            pass

    def _destroy_sub_safe(self, sub):
        if sub is None or self.robot_controller is None:
            return
        try:
            self.robot_controller.destroy_subscription(sub)
        except Exception:
            pass

    def _clear_robot_runtime_cache(self):
        with self._position_lock:
            self._last_positions = None
            self._position_seen_at = None
        with self._robot_state_lock:
            self._robot_state_code = None
            self._robot_state_name = ""
            self._robot_state_raw_name = ""
            self._robot_state_seen_at = None
        with self._robot_mode_lock:
            self._robot_mode_value = None
            self._robot_mode_seen_at = None
            self._control_mode_value = None
            self._control_mode_seen_at = None
            self._robot_mode_initialized_once = False

    def _ensure_robot_state_watchdog(self):
        if not self._ready_event.is_set():
            return
        if not self._robot_stream_enabled:
            return
        if self.robot_controller is None:
            return
        with self._robot_state_lock:
            seen_at = self._robot_state_seen_at
        # 시작 초반 미수신은 기존 초기 대기 로직에 맡기고,
        # 한 번이라도 받은 뒤 stale 되는 경우만 watchdog이 복구를 시도한다.
        if seen_at is None:
            return
        now = time.monotonic()
        if (now - float(seen_at)) <= float(ROBOT_STATE_WATCHDOG_STALE_SEC):
            return
        with self._state_watchdog_lock:
            if (now - self._last_state_watchdog_restart_at) < float(ROBOT_STATE_WATCHDOG_RESTART_INTERVAL_SEC):
                return
            self._last_state_watchdog_restart_at = now
        try:
            if self.robot_controller is not None:
                self.robot_controller._log_info(
                    f"RobotState stale 감지({now - float(seen_at):.2f}s): 상태 구독 재시작 시도"
                )
            self.stop_robot_state_subscriptions()
            ok_restart, msg_restart = self.start_robot_state_subscriptions()
            if self.robot_controller is not None:
                if ok_restart:
                    self.robot_controller._log_info(f"RobotState watchdog 재구독 성공: {msg_restart}")
                else:
                    self.robot_controller._log_error(f"RobotState watchdog 재구독 실패: {msg_restart}")
        except Exception as e:
            if self.robot_controller is not None:
                self.robot_controller._log_error(f"RobotState watchdog 예외: {e}")

    def stop_robot_state_subscriptions(self):
        with self._source_lock:
            self._robot_stream_enabled = False
            self._destroy_sub_safe(self._state_sub)
            self._destroy_sub_safe(self._joint_state_sub)
            self._destroy_sub_safe(self._robot_mode_sub)
            self._state_sub = None
            self._joint_state_sub = None
            self._robot_mode_sub = None
            self._last_source_scan_at = 0.0
            self._last_mode_source_scan_at = 0.0
        self._clear_robot_runtime_cache()
        return True, "로봇 구독 종료 완료"

    def start_robot_state_subscriptions(self):
        if self.robot_controller is None:
            return False, "로봇 컨트롤러가 준비되지 않았습니다."
        with self._source_lock:
            self._robot_stream_enabled = True
            self._last_source_scan_at = 0.0
            self._last_mode_source_scan_at = 0.0
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self._setup_position_source(wait_timeout_sec=0.3)
            self._setup_robot_mode_source()
            if (self._state_sub is not None) or (self._joint_state_sub is not None):
                try:
                    self.sync_robot_mode_once(force=True, timeout_sec=0.8)
                except Exception:
                    pass
                return True, "로봇 구독 재시작 완료"
            time.sleep(0.15)
        return False, "로봇 구독 재시작 실패: RobotState/JointState 토픽 미확인"

    def _detect_connection_mode(self, topic_map):
        if self._connection_mode in ("REAL", "VIRTUAL"):
            return self._connection_mode
        gz_prefix = f"/{ROBOT_ID}/gz/"
        has_gz = any(name.startswith(gz_prefix) for name in topic_map.keys())
        has_robot_ns = any(name.startswith(f"/{ROBOT_ID}/") for name in topic_map.keys())
        if has_gz and has_robot_ns:
            # 혼합 토픽이 동시에 보이면 토픽만으로 단정하지 않는다.
            return "UNKNOWN"
        if has_gz:
            return "VIRTUAL"
        if has_robot_ns:
            return "REAL"
        return "UNKNOWN"

    def _set_connection_mode(self, mode_text):
        mode = str(mode_text).upper().strip()
        if mode not in ("REAL", "VIRTUAL", "UNKNOWN"):
            mode = "UNKNOWN"
        self._connection_mode = mode
        self._connection_mode_seen_at = time.monotonic()
        return mode

    def _detect_connection_mode_once(self, progress_callback=None):
        # 연결 시점에 1회만 system/get_robot_system 호출
        if self._connection_mode_seen_at is not None:
            return self._connection_mode

        if ROBOT_MODE_HINT in ("REAL", "VIRTUAL"):
            mode = self._set_connection_mode(ROBOT_MODE_HINT)
            self._report_progress(progress_callback, 88, f"연결타입 힌트 적용: {mode}")
            return mode

        if GetRobotSystem is not None and self._dsr_node is not None:
            try:
                candidates = (
                    "dsr_controller2/system/get_robot_system",
                    f"/{ROBOT_ID}/dsr_controller2/system/get_robot_system",
                )
                for _ in range(3):
                    for srv_name in candidates:
                        cli = self._get_robot_system_clients.get(srv_name)
                        if cli is None:
                            cli = self._dsr_node.create_client(GetRobotSystem, srv_name)
                            self._get_robot_system_clients[srv_name] = cli
                        if not cli.wait_for_service(timeout_sec=0.7):
                            continue

                        req = GetRobotSystem.Request()
                        future = cli.call_async(req)
                        end_at = time.monotonic() + 1.2
                        while time.monotonic() < end_at:
                            if future.done():
                                break
                            time.sleep(0.05)

                        if not future.done() or future.exception() is not None:
                            continue

                        res = future.result()
                        if res is None or not bool(getattr(res, "success", False)):
                            continue

                        robot_system = int(getattr(res, "robot_system", -1))
                        if robot_system == 0:
                            mode = self._set_connection_mode("REAL")
                        elif robot_system == 1:
                            mode = self._set_connection_mode("VIRTUAL")
                        else:
                            mode = self._set_connection_mode("UNKNOWN")
                        self._report_progress(
                            progress_callback,
                            88,
                            f"연결타입 판별: {mode} ({srv_name})",
                        )
                        return mode
                    time.sleep(0.15)
            except Exception:
                pass

        topic_map = {}
        try:
            if self.robot_controller is not None:
                topic_map = dict(self.robot_controller.get_topic_names_and_types())
        except Exception:
            topic_map = {}

        mode = self._set_connection_mode(self._detect_connection_mode(topic_map))
        self._report_progress(progress_callback, 88, f"연결타입 판별(토픽기준): {mode}")
        return mode

    def _select_state_topic(self, topic_map):
        state_topics = [t for t, types in topic_map.items() if "dsr_msgs2/msg/RobotState" in types]
        if not state_topics:
            return None
        preferred = [t for t in state_topics if f"/{ROBOT_ID}/" in t]
        preferred = preferred or state_topics
        canonical = [t for t in preferred if t.endswith("/state")]
        return sorted(canonical or preferred)[0]

    def _setup_position_source(self, progress_callback=None, wait_timeout_sec: float = 3.0):
        if not self._robot_stream_enabled:
            self._report_progress(progress_callback, 95, "로봇 구독 비활성화 상태")
            return
        if self._state_sub is not None or self._joint_state_sub is not None:
            return
        # bringup 직후에는 /state 토픽이 늦게 올라올 수 있어 짧게 대기 후 선택한다.
        wait_until = time.monotonic() + max(0.0, float(wait_timeout_sec))
        topic_map = {}
        while True:
            topic_map = dict(self.robot_controller.get_topic_names_and_types())
            state_topics_now = [t for t, types in topic_map.items() if "dsr_msgs2/msg/RobotState" in types]
            if state_topics_now or time.monotonic() >= wait_until:
                break
            time.sleep(0.2)

        mode = self._detect_connection_mode(topic_map)
        state_topic = self._select_state_topic(topic_map)
        if state_topic is not None:
            self._state_sub = self.robot_controller.create_subscription(
                RobotState, state_topic, self._on_robot_state, 10
            )
            self._report_progress(progress_callback, 95, f"상태 구독 시작: {state_topic} (연결타입={mode})")
            return

        for joint_topic in self._joint_state_topics:
            joint_types = topic_map.get(joint_topic, [])
            if "sensor_msgs/msg/JointState" in joint_types:
                self._joint_state_sub = self.robot_controller.create_subscription(
                    JointState, joint_topic, self._on_joint_state, 10
                )
                self._report_progress(
                    progress_callback,
                    95,
                    f"조인트 구독 시작: {joint_topic} (XYZABC는 상태 토픽 필요)",
                )
                return

        # Fallback: no known topic found.
        self._report_progress(progress_callback, 95, "상태/조인트 토픽 미발견")

    def _setup_robot_mode_source(self, progress_callback=None):
        if not self._robot_stream_enabled:
            self._report_progress(progress_callback, 96, "로봇모드 구독 비활성화 상태")
            return
        if self._robot_mode_sub is not None:
            return
        if RobotStateRt is None:
            self._report_progress(progress_callback, 96, "로봇모드 토픽 비활성화: RobotStateRt import 실패")
            return
        topic_map = dict(self.robot_controller.get_topic_names_and_types())
        rt_topics = [t for t, types in topic_map.items() if "dsr_msgs2/msg/RobotStateRt" in types]
        if not rt_topics:
            self._report_progress(progress_callback, 96, "로봇모드 토픽 미발견: RobotStateRt")
            return
        preferred = [t for t in rt_topics if f"/{ROBOT_ID}/" in t]
        topic = sorted(preferred or rt_topics)[0]
        self._robot_mode_sub = self.robot_controller.create_subscription(
            RobotStateRt, topic, self._on_robot_mode_rt, 10
        )
        self._report_progress(progress_callback, 96, f"로봇모드/제어모드 구독 시작: {topic}")

    def _try_upgrade_to_state_source(self):
        # 시작 타이밍에 /state를 못 잡았더라도, 이후 나타나면 자동 보강한다.
        if self.robot_controller is None:
            return
        if not self._robot_stream_enabled:
            return
        if self._state_sub is not None:
            return
        now = time.monotonic()
        if (now - self._last_source_scan_at) < self._source_scan_interval_sec:
            return
        self._last_source_scan_at = now
        with self._source_lock:
            if self._state_sub is not None:
                return
            topic_map = dict(self.robot_controller.get_topic_names_and_types())
            mode = self._detect_connection_mode(topic_map)
            state_topic = self._select_state_topic(topic_map)
            if state_topic is not None:
                self._state_sub = self.robot_controller.create_subscription(
                    RobotState, state_topic, self._on_robot_state, 10
                )
                self.robot_controller._log_info(f"상태 토픽 감지: {state_topic} (연결타입={mode}, 첫 수신 후 조인트 구독 정리)")

    def _try_upgrade_robot_mode_source(self):
        if RobotStateRt is None or self.robot_controller is None:
            return
        if not self._robot_stream_enabled:
            return
        if self._robot_mode_sub is not None:
            return
        now = time.monotonic()
        if (now - self._last_mode_source_scan_at) < self._mode_source_scan_interval_sec:
            return
        self._last_mode_source_scan_at = now
        with self._source_lock:
            if self._robot_mode_sub is not None:
                return
            topic_map = dict(self.robot_controller.get_topic_names_and_types())
            rt_topics = [t for t, types in topic_map.items() if "dsr_msgs2/msg/RobotStateRt" in types]
            if not rt_topics:
                return
            preferred = [t for t in rt_topics if f"/{ROBOT_ID}/" in t]
            topic = sorted(preferred or rt_topics)[0]
            self._robot_mode_sub = self.robot_controller.create_subscription(
                RobotStateRt, topic, self._on_robot_mode_rt, 10
            )
            self.robot_controller._log_info(f"로봇모드 토픽 자동 전환 완료: {topic}")

    def get_robot_state_snapshot(self):
        self._ensure_robot_state_watchdog()
        with self._robot_state_lock:
            return self._robot_state_code, self._robot_state_name, self._robot_state_seen_at

    def get_robot_state_raw_snapshot(self):
        with self._robot_state_lock:
            return self._robot_state_raw_name, self._robot_state_seen_at

    def _check_motion_precondition(self, max_state_age_sec=1.5):
        state_code, state_name, seen_at = self.get_robot_state_snapshot()
        if state_code is None:
            return False, "로봇 상태 미수신입니다. (/dsr01/state 확인 필요)"
        age = float("inf") if seen_at is None else (time.monotonic() - seen_at)
        if age > max_state_age_sec:
            return False, f"로봇 상태 수신 지연({age:.1f}s)입니다."

        code = int(state_code)
        if code in MOTION_BLOCKED_STATE_CODES:
            name = state_name or get_robot_state_name(code)
            if code in (3, 11):
                return (
                    False,
                    f"현재 상태가 {name}({code})라 동작 불가. "
                    f"로봇을 RESET/SERVO ON 후 STANDBY 상태에서 다시 시도하세요.",
                )
            return False, f"현재 상태가 {name}({code})라 동작 시작이 불가합니다."
        return True, "ok"

    def _service_name_candidates(self, relative_name: str):
        rel = str(relative_name or "").strip().lstrip("/")
        if not rel:
            return tuple()
        prefixed_rel = f"dsr_controller2/{rel}"
        candidates = [
            prefixed_rel,
            f"/{ROBOT_ID}/{prefixed_rel}",
        ]
        seen = set()
        ordered = []
        for name in candidates:
            if name in seen:
                continue
            seen.add(name)
            ordered.append(name)
        return tuple(ordered)

    def _get_or_create_service_client(self, node, cache: dict, srv_type, srv_name: str):
        cli = cache.get(srv_name)
        if cli is None:
            cli = node.create_client(srv_type, srv_name)
            cache[srv_name] = cli
        return cli

    def _call_service_with_candidates(
        self,
        *,
        node,
        srv_type,
        cache: dict,
        relative_name: str,
        req,
        timeout_sec: float,
        action_name: str,
    ):
        if node is None:
            return False, None, None, f"{action_name} 서비스 노드가 준비되지 않았습니다."

        wait_timeout = max(0.2, min(0.8, float(timeout_sec)))
        call_timeout = max(0.2, float(timeout_sec))
        last_msg = f"{relative_name} 서비스가 준비되지 않았습니다."

        for srv_name in self._service_name_candidates(relative_name):
            cli = self._get_or_create_service_client(node, cache, srv_type, srv_name)
            if not cli.wait_for_service(timeout_sec=wait_timeout):
                last_msg = f"{srv_name} 서비스가 준비되지 않았습니다."
                continue

            future = cli.call_async(req)
            end_at = time.monotonic() + call_timeout
            while time.monotonic() < end_at:
                if future.done():
                    break
                time.sleep(0.05)

            if not future.done():
                last_msg = f"{srv_name} 서비스 응답 시간 초과"
                continue
            if future.exception() is not None:
                last_msg = f"{srv_name} 서비스 예외: {future.exception()}"
                continue

            res = future.result()
            if res is None or not bool(getattr(res, "success", False)):
                last_msg = f"{srv_name} 서비스 실패"
                continue
            return True, res, srv_name, f"{action_name} 성공({srv_name})"

        return False, None, None, last_msg

    def _call_set_robot_control(self, control_value: int, timeout_sec: float = 2.0):
        if SetRobotControl is None:
            return False, "SetRobotControl 서비스 타입 import 실패"
        if self._dsr_node is None:
            return False, "DSR 노드가 없어 리셋 서비스를 호출할 수 없습니다."
        req = SetRobotControl.Request()
        req.robot_control = int(control_value)
        ok, _res, srv_name, msg = self._call_service_with_candidates(
            node=self._dsr_node,
            srv_type=SetRobotControl,
            cache=self._set_robot_control_clients,
            relative_name="system/set_robot_control",
            req=req,
            timeout_sec=timeout_sec,
            action_name="리셋 서비스",
        )
        if not ok:
            return False, f"{msg}(control={control_value})"
        return True, f"리셋 서비스 성공(control={control_value}, srv={srv_name})"

    def _call_set_robot_mode(self, mode_value: int, timeout_sec: float = 2.0):
        if SetRobotMode is None:
            return False, "SetRobotMode 서비스 타입 import 실패"
        if self._mode_node is None:
            return False, "모드 서비스 노드가 없어 모드 서비스를 호출할 수 없습니다."

        with self._robot_mode_service_lock:
            req = SetRobotMode.Request()
            req.robot_mode = int(mode_value)
            ok, _res, srv_name, msg = self._call_service_with_candidates(
                node=self._mode_node,
                srv_type=SetRobotMode,
                cache=self._set_robot_mode_clients,
                relative_name="system/set_robot_mode",
                req=req,
                timeout_sec=timeout_sec,
                action_name="모드 서비스",
            )
            if not ok:
                return False, f"{msg}(mode={mode_value})"
            return True, f"모드 서비스 성공(mode={mode_value}, srv={srv_name})"

    def _call_get_robot_mode(self, timeout_sec: float = 2.0):
        if GetRobotMode is None:
            return False, None, "GetRobotMode 서비스 타입 import 실패"
        if self._mode_node is None:
            return False, None, "모드 서비스 노드가 없어 모드 조회 서비스를 호출할 수 없습니다."

        with self._robot_mode_service_lock:
            req = GetRobotMode.Request()
            ok, res, srv_name, msg = self._call_service_with_candidates(
                node=self._mode_node,
                srv_type=GetRobotMode,
                cache=self._get_robot_mode_clients,
                relative_name="system/get_robot_mode",
                req=req,
                timeout_sec=timeout_sec,
                action_name="모드 조회 서비스",
            )
            if not ok:
                return False, None, msg
            try:
                mode_value = int(getattr(res, "robot_mode", -1))
            except Exception:
                return False, None, "모드 조회 응답 파싱 실패"
            return True, mode_value, f"모드 조회 성공(mode={mode_value}, srv={srv_name})"

    def sync_robot_mode_once(self, force: bool = False, timeout_sec: float = 1.5):
        with self._robot_mode_lock:
            if (not force) and self._robot_mode_initialized_once and (self._robot_mode_value is not None):
                return True, f"모드 유지(mode={int(self._robot_mode_value)})"

        ok, mode_value, msg = self._call_get_robot_mode(timeout_sec=timeout_sec)
        if not ok or mode_value is None:
            return False, msg
        with self._robot_mode_lock:
            self._robot_mode_value = int(mode_value)
            self._robot_mode_seen_at = time.monotonic()
            self._robot_mode_initialized_once = True
        return True, msg

    def set_robot_mode(self, mode_value: int, timeout_sec: float = 2.0):
        if not self._ready_event.is_set():
            return False, "백엔드 준비 중입니다."
        if self._busy_event.is_set():
            return False, "작업 중에는 모드를 변경할 수 없습니다."
        mode = int(mode_value)

        op_timeout = max(2.0, float(timeout_sec))
        ok_set, msg_set = self._call_set_robot_mode(mode, timeout_sec=op_timeout)
        set_result_text = f"SET={'OK' if ok_set else 'FAIL'} ({msg_set})"
        if self.robot_controller is not None:
            log_fn = self.robot_controller._log_info if ok_set else self.robot_controller._log_error
            log_fn(f"[모드변경] {set_result_text}")
        if not ok_set:
            merged = f"{set_result_text}; GET=SKIP (set 실패로 재확인 생략)"
            if self.robot_controller is not None:
                self.robot_controller._log_error(f"[모드변경] {merged}")
            return False, merged

        settle_delay_sec = 0.2
        time.sleep(settle_delay_sec)

        verify_deadline = time.monotonic() + max(3.0, op_timeout)
        last_verify_msg = "모드 재확인 실패"
        last_get_result_text = f"GET=FAIL ({last_verify_msg})"
        while time.monotonic() < verify_deadline:
            ok_get, now_mode, msg_get = self._call_get_robot_mode(timeout_sec=1.5)
            last_verify_msg = msg_get
            if ok_get and now_mode is not None:
                last_get_result_text = f"GET=OK (mode={int(now_mode)}; {msg_get})"
            else:
                last_get_result_text = f"GET=FAIL ({msg_get})"
            if ok_get and now_mode is not None:
                with self._robot_mode_lock:
                    self._robot_mode_value = int(now_mode)
                    self._robot_mode_seen_at = time.monotonic()
                    self._robot_mode_initialized_once = True
                if int(now_mode) == mode:
                    merged = f"{set_result_text}; {last_get_result_text}"
                    if mode == 1:
                        gr_ok, gr_msg = self._retry_gripper_init_after_reset()
                        merged = f"{merged}; GRIPPER={'OK' if gr_ok else 'FAIL'} ({gr_msg})"
                        if self.robot_controller is not None:
                            log_fn = self.robot_controller._log_info if gr_ok else self.robot_controller._log_error
                            log_fn(f"[모드변경] {('오토모드 확인 후 ' + gr_msg)}")
                    if self.robot_controller is not None:
                        self.robot_controller._log_info(f"[모드변경] {merged}")
                    return True, merged
            time.sleep(0.25)

        merged = f"{set_result_text}; {last_get_result_text}"
        if self.robot_controller is not None:
            self.robot_controller._log_error(f"[모드변경] {merged}")
        return False, merged

    def get_robot_mode_snapshot(self):
        with self._robot_mode_lock:
            return self._robot_mode_value, self._robot_mode_seen_at

    def get_control_mode_snapshot(self):
        with self._robot_mode_lock:
            return self._control_mode_value, self._control_mode_seen_at

    def get_connection_mode_snapshot(self):
        return self._connection_mode, self._connection_mode_seen_at

    def _call_get_current_tcp(self, timeout_sec: float = 2.0):
        if GetCurrentTcp is None:
            return False, None, "GetCurrentTcp 서비스 타입 import 실패"
        if self._mode_node is None:
            return False, None, "모드 서비스 노드가 없어 TCP 조회 서비스를 호출할 수 없습니다."

        with self._tcp_service_lock:
            req = GetCurrentTcp.Request()
            ok, res, srv_name, msg = self._call_service_with_candidates(
                node=self._mode_node,
                srv_type=GetCurrentTcp,
                cache=self._get_current_tcp_clients,
                relative_name="tcp/get_current_tcp",
                req=req,
                timeout_sec=timeout_sec,
                action_name="TCP 조회 서비스",
            )
            if not ok:
                return False, None, msg
            tcp_name = str(getattr(res, "info", "") or "").strip()
            return True, tcp_name, f"TCP 조회 성공(name='{tcp_name}', srv={srv_name})"

    def _call_set_current_tcp(self, tcp_name: str, timeout_sec: float = 2.0):
        if SetCurrentTcp is None:
            return False, "SetCurrentTcp 서비스 타입 import 실패"
        if self._mode_node is None:
            return False, "모드 서비스 노드가 없어 TCP 설정 서비스를 호출할 수 없습니다."

        name = str(tcp_name or "")
        with self._tcp_service_lock:
            req = SetCurrentTcp.Request()
            req.name = name
            ok, _res, srv_name, msg = self._call_service_with_candidates(
                node=self._mode_node,
                srv_type=SetCurrentTcp,
                cache=self._set_current_tcp_clients,
                relative_name="tcp/set_current_tcp",
                req=req,
                timeout_sec=timeout_sec,
                action_name="TCP 설정 서비스",
            )
            if not ok:
                return False, f"{msg}(name='{name}')"
            return True, f"TCP 설정 서비스 성공(name='{name}', srv={srv_name})"

    def sync_current_tcp_once(self, force: bool = False, timeout_sec: float = 1.5):
        now = time.monotonic()
        with self._tcp_state_lock:
            if (not force) and self._current_tcp_seen_at is not None and (now - self._last_tcp_sync_at) < 0.8:
                return True, f"TCP 유지(name='{self._current_tcp_name or ''}')"

        ok, tcp_name, msg = self._call_get_current_tcp(timeout_sec=timeout_sec)
        if not ok:
            return False, msg
        with self._tcp_state_lock:
            self._current_tcp_name = str(tcp_name or "").strip()
            self._current_tcp_seen_at = time.monotonic()
            self._last_tcp_sync_at = self._current_tcp_seen_at
        return True, msg

    def get_current_tcp_snapshot(self):
        with self._tcp_state_lock:
            return self._current_tcp_name, self._current_tcp_seen_at

    def set_current_tcp_name(self, tcp_name: str, timeout_sec: float = 3.0):
        if not self._ready_event.is_set():
            return False, "백엔드 준비 중입니다."
        if self._busy_event.is_set():
            return False, "작업 중에는 TCP를 변경할 수 없습니다."

        target = str(tcp_name or "").strip()
        op_timeout = max(2.0, float(timeout_sec))
        ok_set, msg_set = self._call_set_current_tcp(target, timeout_sec=op_timeout)
        set_result_text = f"SET={'OK' if ok_set else 'FAIL'} ({msg_set})"
        if not ok_set:
            merged = f"{set_result_text}; GET=SKIP (set 실패로 재확인 생략)"
            return False, merged

        time.sleep(0.15)
        ok_get, now_tcp, msg_get = self._call_get_current_tcp(timeout_sec=1.5)
        if not ok_get:
            merged = f"{set_result_text}; GET=FAIL ({msg_get})"
            return False, merged

        now_tcp = str(now_tcp or "").strip()
        with self._tcp_state_lock:
            self._current_tcp_name = now_tcp
            self._current_tcp_seen_at = time.monotonic()
            self._last_tcp_sync_at = self._current_tcp_seen_at

        get_result_text = f"GET=OK (name='{now_tcp}')"
        if now_tcp != target:
            merged = f"{set_result_text}; {get_result_text}; verify_mismatch(target='{target}')"
            return False, merged
        merged = f"{set_result_text}; {get_result_text}"
        return True, merged

    def _wait_state_any(self, target_codes, timeout_sec=3.0):
        deadline = time.monotonic() + max(0.2, timeout_sec)
        target_set = set(int(c) for c in target_codes)
        while time.monotonic() < deadline:
            state_code, state_name, seen_at = self.get_robot_state_snapshot()
            if state_code is not None and int(state_code) in target_set:
                return True, int(state_code), (state_name or get_robot_state_name(state_code))
            time.sleep(0.05)
        state_code, state_name, _ = self.get_robot_state_snapshot()
        if state_code is None:
            return False, None, "UNKNOWN"
        return False, int(state_code), (state_name or get_robot_state_name(state_code))

    def reset_robot_state(self):
        if not self._ready_event.is_set():
            return False, "백엔드 준비 중입니다."
        if self._busy_event.is_set():
            return False, "작업 중에는 리셋할 수 없습니다."

        state_code, state_name, seen_at = self.get_robot_state_snapshot()
        if state_code is None:
            return False, "현재 로봇 상태를 모릅니다. (/dsr01/state 확인 필요)"
        if seen_at is None or (time.monotonic() - seen_at) > 2.0:
            return False, "로봇 상태 수신이 지연되어 리셋을 진행할 수 없습니다."

        code = int(state_code)
        name = state_name or get_robot_state_name(code)

        def _ensure_auto_mode_for_gripper():
            mode_ok, mode_msg = self._call_set_robot_mode(1, timeout_sec=1.5)
            if self.robot_controller is not None:
                self.robot_controller._log_info(f"[리셋] {mode_msg}")
            if not mode_ok:
                return False, f"오토모드(1) 전환 실패: {mode_msg}"
            verify_ok, verify_mode, verify_msg = self._call_get_robot_mode(timeout_sec=1.2)
            if not verify_ok or verify_mode is None:
                return False, f"오토모드(1) 재확인 실패: {verify_msg}"
            try:
                verified_mode = int(verify_mode)
            except Exception:
                return False, "오토모드(1) 재확인 실패: 모드값 파싱 실패"
            with self._robot_mode_lock:
                self._robot_mode_value = verified_mode
                self._robot_mode_seen_at = time.monotonic()
                self._robot_mode_initialized_once = True
            if verified_mode != 1:
                return False, f"오토모드(1) 재확인 실패: 현재 mode={verified_mode}"
            return True, "오토모드(1) 확인 완료"

        # SetRobotControl.srv 기준:
        # 0=INIT_CONFIG, 1=ENABLE_OPERATION, 2=RESET_SAFE_STOP, 3=RESET_SAFE_OFF
        # 4=RECOVERY_SAFE_STOP, 5=RECOVERY_SAFE_OFF, 7=RESET_RECOVERY
        if code == 1:
            auto_ok, auto_msg = _ensure_auto_mode_for_gripper()
            if not auto_ok:
                return False, f"이미 STANDBY 상태입니다. {auto_msg}"
            gr_ok, gr_msg = self._retry_gripper_init_after_reset()
            if gr_ok:
                return True, f"이미 STANDBY 상태입니다. {gr_msg}"
            return False, f"이미 STANDBY 상태입니다. {gr_msg}"
        if code == 0:      # INITIALIZING -> STANDBY
            # 초기화 상태에서 동작 허용을 위해 AUTONOMOUS 모드 선행 시도
            auto_ok, auto_msg = _ensure_auto_mode_for_gripper()
            if not auto_ok:
                return False, auto_msg
            steps = [1]
        elif code == 15:   # NOT_READY -> INITIALIZING -> STANDBY
            auto_ok, auto_msg = _ensure_auto_mode_for_gripper()
            if not auto_ok:
                return False, auto_msg
            steps = [0, 1]
        elif code == 3:
            steps = [3]
        elif code == 5:
            steps = [2]
        elif code == 10:
            steps = [4, 7]
        elif code == 11:
            steps = [5, 7]
        elif code == 9:
            steps = [7]
        else:
            return False, f"현재 상태({name})에서는 자동 리셋 시퀀스를 정의하지 않았습니다."

        for idx, control in enumerate(steps):
            ok, msg = self._call_set_robot_control(control, timeout_sec=2.0)
            if not ok:
                return False, msg
            if self.robot_controller is not None:
                self.robot_controller._log_info(f"[리셋] 단계 {idx + 1}/{len(steps)} 완료: {msg}")
            time.sleep(0.15)
        ok_state, now_code, now_name = self._wait_state_any({1}, timeout_sec=3.0)
        if not ok_state:
            return (
                False,
                f"리셋 명령은 전송됐지만 STANDBY로 전환되지 않았습니다. "
                f"현재 상태={now_name}({now_code}). "
                f"제어권한/서보온/티치펜던트 상태를 확인하세요.",
            )
        auto_ok, auto_msg = _ensure_auto_mode_for_gripper()
        if not auto_ok:
            return True, f"리셋 완료: {name} -> STANDBY, 하지만 {auto_msg}"
        gr_ok, gr_msg = self._retry_gripper_init_after_reset()
        if gr_ok:
            return True, f"리셋 완료: {name} -> STANDBY, {gr_msg}"
        return True, f"리셋 완료: {name} -> STANDBY, 하지만 {gr_msg}"

    def send_pick_place(self, value):
        if not self._ready_event.is_set():
            return False, "백엔드 준비 중입니다."

        gripper_ok, gripper_msg = self._check_gripper_precondition()
        if not gripper_ok:
            return False, gripper_msg

        if not (0 <= value <= 10):
            return False, "값은 0~10 범위여야 합니다."

        if self._busy_event.is_set() or self._cmd_q.full():
            return False, "현재 작업 중입니다."

        state_ok, state_msg = self._check_motion_precondition(max_state_age_sec=1.5)
        if not state_ok:
            return False, state_msg

        # 고정 자세는 "한 번의 작업 시퀀스 내"에서만 유지한다.
        # 다음 작업 시작 시에는 get_current_posx 기준으로 다시 산출한다.
        with self._fixed_abc_lock:
            if DEFAULT_PICK_PLACE_ABC is None:
                self._fixed_pick_place_abc = None

        self._cmd_q.put(("pick_place", value))
        return True, "작업을 시작합니다."

    def send_move_home(self, vel=None, acc=None):
        if not self._ready_event.is_set():
            return False, "백엔드 준비 중입니다."

        if self._busy_event.is_set() or self._cmd_q.full():
            return False, "현재 작업 중입니다."

        # 홈 이동은 작업 중 상태 변화 영향이 적으므로 상태 수신 지연 허용 시간을 넉넉히 둔다.
        state_ok, state_msg = self._check_motion_precondition(max_state_age_sec=5.0)
        if not state_ok:
            return False, state_msg

        tv = None if vel is None else float(vel)
        ta2 = None if acc is None else float(acc)
        self._cmd_q.put(("move_home", (tv, ta2)))
        speed_text = f" 속도={tv:.0f}%" if tv is not None else ""
        return True, f"초기 위치 이동을 시작합니다.{speed_text}"

    def send_move_vision_point(self, x_mm, y_mm, z_mm, dwell_sec=1.0):
        if not self._ready_event.is_set():
            return False, "백엔드 준비 중입니다."

        if self._busy_event.is_set() or self._cmd_q.full():
            return False, "현재 작업 중입니다."

        try:
            tx = float(x_mm)
            ty = float(y_mm)
            tz = float(z_mm)
            td = float(dwell_sec)
        except Exception:
            return False, "비전 이동 좌표/대기시간 형식이 올바르지 않습니다."

        state_ok, state_msg = self._check_motion_precondition(max_state_age_sec=2.0)
        if not state_ok:
            return False, state_msg

        self._cmd_q.put(("move_vision_point", (tx, ty, tz, td)))
        return True, f"비전 좌표 이동 시작: X={tx:.2f}, Y={ty:.2f}, Z={tz:.2f} (대기 {td:.1f}s 후 홈 복귀)"

    def send_move_cartesian(self, x, y, z, a, b, c, vel=None, acc=None):
        if not self._ready_event.is_set():
            return False, "백엔드 준비 중입니다."
        if self._busy_event.is_set() or self._cmd_q.full():
            return False, "현재 작업 중입니다."
        try:
            tx, ty, tz = float(x), float(y), float(z)
            ta, tb, tc = float(a), float(b), float(c)
        except Exception:
            return False, "XYZABC 값 형식이 올바르지 않습니다."
        state_ok, state_msg = self._check_motion_precondition(max_state_age_sec=2.0)
        if not state_ok:
            return False, state_msg
        tv = None if vel is None else float(vel)
        ta2 = None if acc is None else float(acc)
        self._cmd_q.put(("move_cartesian", (tx, ty, tz, ta, tb, tc, tv, ta2)))
        speed_text = f", 속도={tv:.0f}%" if tv is not None else ""
        return True, (
            f"카테시안 이동 시작: X={tx:.2f}, Y={ty:.2f}, Z={tz:.2f}, "
            f"A={ta:.2f}, B={tb:.2f}, C={tc:.2f}{speed_text}"
        )

    def send_move_joint(self, j1, j2, j3, j4, j5, j6, vel=None, acc=None):
        if not self._ready_event.is_set():
            return False, "백엔드 준비 중입니다."
        if self._busy_event.is_set() or self._cmd_q.full():
            return False, "현재 작업 중입니다."
        try:
            vals = tuple(float(v) for v in (j1, j2, j3, j4, j5, j6))
        except Exception:
            return False, "조인트 값 형식이 올바르지 않습니다."
        state_ok, state_msg = self._check_motion_precondition(max_state_age_sec=2.0)
        if not state_ok:
            return False, state_msg
        tv = None if vel is None else float(vel)
        ta2 = None if acc is None else float(acc)
        self._cmd_q.put(("move_joint", (*vals, tv, ta2)))
        speed_text = f", 속도={tv:.0f}%" if tv is not None else ""
        return True, (
            f"조인트 이동 시작: J1={vals[0]:.2f}, J2={vals[1]:.2f}, J3={vals[2]:.2f}, "
            f"J4={vals[3]:.2f}, J5={vals[4]:.2f}, J6={vals[5]:.2f}{speed_text}"
        )

    def send_gripper_move(self, opening_distance_mm):
        if not self._ready_event.is_set():
            return False, "백엔드 준비 중입니다."

        try:
            target_mm = float(opening_distance_mm)
        except Exception:
            return False, "거리 값은 숫자(mm)여야 합니다."

        if target_mm < GRIPPER_DISTANCE_MIN_MM or target_mm > GRIPPER_DISTANCE_MAX_MM:
            return False, f"거리 범위는 {GRIPPER_DISTANCE_MIN_MM:.0f}~{GRIPPER_DISTANCE_MAX_MM:.0f} mm 입니다."

        if self._busy_event.is_set() or self._cmd_q.full():
            return False, "현재 작업 중입니다."

        if not self.use_real_gripper:
            return False, "그리퍼 비활성화 상태입니다. (STARTUP_USE_REAL_GRIPPER=1 필요)"

        gripper_ok, gripper_msg = self._check_gripper_precondition()
        if not gripper_ok:
            return False, gripper_msg

        self._cmd_q.put(("gripper_move", target_mm))
        pulse = gripper_distance_mm_to_pulse(target_mm)
        return True, f"그리퍼 이동을 시작합니다. (distance={target_mm:.2f}mm, pulse={pulse})"

    def is_ready(self):
        return self._ready_event.is_set()

    def is_busy(self):
        return self._busy_event.is_set()

    def last_error(self):
        return self._last_error

    def get_current_positions(self):
        if not self._ready_event.is_set():
            return None
        self._try_upgrade_to_state_source()
        with self._position_lock:
            return self._last_positions

    def get_position_snapshot(self):
        if not self._ready_event.is_set():
            return None, None
        self._ensure_robot_state_watchdog()
        self._try_upgrade_to_state_source()
        with self._position_lock:
            if self._last_positions is None:
                return None, None
            return self._last_positions, self._position_seen_at

    def get_current_posx_live(self):
        if not self._ready_event.is_set():
            return None, None, "백엔드 준비 중입니다."
        try:
            from DSR_ROBOT2 import get_current_posx
        except Exception as e:
            return None, None, f"get_current_posx import 실패: {e}"

        try:
            # 좌표계 전환(set_ref_coord) 영향 제거: 항상 BASE(0) 기준으로 조회한다.
            posx_value, sol = get_current_posx(0)
        except Exception as e:
            return None, None, f"get_current_posx 호출 실패: {e}"

        if posx_value is None:
            return None, None, "get_current_posx 결과가 없습니다."

        try:
            posx_list = [float(v) for v in list(posx_value)[:6]]
        except Exception as e:
            return None, None, f"get_current_posx 결과 파싱 실패: {e}"

        if len(posx_list) < 6:
            return None, None, "get_current_posx 결과 길이가 올바르지 않습니다."

        try:
            sol_value = None if sol is None else int(sol)
        except Exception:
            sol_value = None
        return posx_list, sol_value, None

    def shutdown(self):
        if not self._started:
            return

        self._stop_event.set()
        try:
            self._cmd_q.put_nowait(("quit", None))
        except queue.Full:
            pass

        if self._worker_thread is not None:
            self._worker_thread.join(timeout=2.0)

        if self.robot_controller is not None:
            try:
                if rclpy.ok():
                    self.robot_controller.terminate_gripper()
                    for _ in range(50):
                        rclpy.spin_once(self.robot_controller, timeout_sec=0.01)
                        rclpy.spin_once(self._dsr_node, timeout_sec=0.01)
                        if self._mode_node is not None:
                            rclpy.spin_once(self._mode_node, timeout_sec=0.01)
                else:
                    print("ROS 컨텍스트가 이미 종료되어 그리퍼 terminate를 생략합니다.")
            except Exception as e:
                print(f"그리퍼 terminate 중 예외(종료 과정에서 흔함): {e}")

        try:
            if self.robot_controller is not None:
                self.robot_controller.destroy_node()
        except Exception:
            pass

        try:
            if self._dsr_node is not None:
                self._dsr_node.destroy_node()
        except Exception:
            pass

        try:
            if self._mode_node is not None:
                self._mode_node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass

        self._started = False
        print("종료 완료.")


def main():
    backend = RobotBackend()
    try:
        backend.start()
    except Exception as e:
        print(e)
        return

    try:
        while True:
            s = input("오브젝트 번호 0~10 입력 (q=종료): ").strip()
            if s.lower() == "q":
                break
            try:
                value = int(s)
            except ValueError:
                print("숫자만 입력하세요.")
                continue

            ok, msg = backend.send_pick_place(value)
            print(msg)
    except KeyboardInterrupt:
        print("Ctrl+C로 종료합니다...")
    finally:
        backend.shutdown()


if __name__ == "__main__":
    main()
