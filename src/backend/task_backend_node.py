import rclpy
from rclpy.node import Node
import time
import math
import os
import glob
import csv
import json
import re
import subprocess
import sys

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
    from sensor_msgs.msg import CameraInfo as RosCameraInfoMsg
except Exception:
    RosCameraInfoMsg = None
try:
    from std_msgs.msg import String as RosStringMsg
except Exception:
    RosStringMsg = None
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
try:
    from dsr_msgs2.srv import ServoOff
except Exception:
    ServoOff = None
try:
    from dsr_msgs2.srv import MoveStop
except Exception:
    MoveStop = None

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
SRC_ROOT = os.path.join(PROJECT_ROOT, "src")
if SRC_ROOT in sys.path:
    try:
        sys.path.remove(SRC_ROOT)
    except Exception:
        pass
sys.path.insert(0, SRC_ROOT)


def _normalize_volume_condition(condition: str) -> str:
    text = str(condition or "").strip().lower()
    if text in ("gt", "gte", "eq", "lt", "lte"):
        return text
    if text == ">":
        return "gt"
    if text == ">=":
        return "gte"
    if text in ("=", "=="):
        return "eq"
    if text == "<":
        return "lt"
    if text == "<=":
        return "lte"
    return "gte"


def _format_volume_condition(condition: str, target_volume_ml: float) -> str:
    cond = _normalize_volume_condition(condition)
    op = {
        "gt": ">",
        "gte": ">=",
        "eq": "==",
        "lt": "<",
        "lte": "<=",
    }.get(cond, ">=")
    return f"{op}{float(target_volume_ml):.1f}ml"


def _resolve_volume_condition_target(condition: str, target_volume_ml):
    cond_text = str(condition or "").strip().lower()
    target = None
    if target_volume_ml is not None:
        target = float(target_volume_ml)

    if cond_text:
        norm = _normalize_volume_condition(cond_text)
        if cond_text in ("gt", "gte", "eq", "lt", "lte", ">", ">=", "<", "<=", "=", "=="):
            cond = norm
        else:
            compact = cond_text.replace(" ", "").replace("ml", "")
            match = re.match(r"^(>=|<=|>|<|==|=)(-?\d+(?:\.\d+)?)$", compact)
            if not match:
                match = re.match(r"^(-?\d+(?:\.\d+)?)(>=|<=|>|<|==|=)$", compact)
                if match:
                    target = float(match.group(1))
                    cond = _normalize_volume_condition(match.group(2))
                else:
                    raise ValueError(f"지원하지 않는 용량 조건식: {condition}")
            else:
                cond = _normalize_volume_condition(match.group(1))
                target = float(match.group(2))
    else:
        cond = "gte"

    if target is None:
        raise ValueError("target_volume_ml 또는 조건식(예: >=100, 100>)이 필요합니다.")
    return cond, float(target)


def _is_volume_condition_met(
    current_volume_ml: float,
    target_volume_ml: float,
    *,
    condition: str = "gte",
    tolerance_ml: float = 0.5,
) -> bool:
    cond = _normalize_volume_condition(condition)
    cur = float(current_volume_ml)
    tgt = float(target_volume_ml)
    tol = max(0.0, float(tolerance_ml))
    if cond == "gt":
        return cur > tgt
    if cond == "gte":
        return cur >= tgt
    if cond == "lt":
        return cur < tgt
    if cond == "lte":
        return cur <= tgt
    # eq
    return abs(cur - tgt) <= tol

# `src/backend/bartender_action` (legacy)와 `src/bartender_action`(현재) 패키지명이 겹친다.
# 우선순위를 강제로 `src/bartender_action`으로 맞춘다.
_legacy_bartender_pkg = os.path.join(CURRENT_DIR, "bartender_action")
_loaded_bartender_pkg = sys.modules.get("bartender_action")
_loaded_bartender_file = str(getattr(_loaded_bartender_pkg, "__file__", "") or "")
if _loaded_bartender_file.startswith(_legacy_bartender_pkg):
    for _mod_name in list(sys.modules.keys()):
        if _mod_name == "bartender_action" or _mod_name.startswith("bartender_action."):
            try:
                del sys.modules[_mod_name]
            except Exception:
                pass
from order_integration.voice_order_route import run_voice_order_request
from bartender_action.bartender_sequence_manager import BartenderSequenceApiServer, BartenderSequenceManager
try:
    from bartender_action.robot_action_planner import run_robot_action as run_bartender_robot_action
except Exception:
    run_bartender_robot_action = None

PARAM_DIR = os.path.join(PROJECT_ROOT, "config")
PARAM_FILE = os.path.join(PARAM_DIR, "parameter.csv")
CALIB_DIR = os.path.join(PARAM_DIR, "calibration")
MENU_OFFSET_CONFIG_PATH = os.path.join(PARAM_DIR, "menu_xyz_offsets.json")
VISION1_META_TOPIC = os.environ.get("VISION_OBJECT_META_TOPIC_1", "/camera/camera_1/detection/object/meta")
VISION1_CAMERA_INFO_TOPIC_PRIMARY = os.environ.get("CALIB_CAMERA_INFO_TOPIC", "/camera/camera_1/color/camera_info")
VISION2_META_TOPIC = os.environ.get("VISION_VOLUME_META_TOPIC_2", "/camera/camera_2/detection/volume/meta")
BARTENDER_ROBOT_ACTION_SCRIPT_PATH = os.path.join(
    PROJECT_ROOT, "src", "backend", "bartender_action", "robot_action_planner.py"
)
BARTENDER_ROBOT_ACTION_SCRIPT_TIMEOUT_SEC = max(
    2.0, float(os.environ.get("BARTENDER_ROBOT_ACTION_SCRIPT_TIMEOUT_SEC", "12.0"))
)
VOICE_ORDER_RUNTIME_TIMEOUT_SEC = max(
    10.0, float(os.environ.get("BARTENDER_VOICE_RUNTIME_TIMEOUT_SEC", "90.0"))
)
BARTENDER_ROBOT_ACTION_NATIVE = str(os.environ.get("BARTENDER_ROBOT_ACTION_NATIVE", "1")).strip().lower() in (
    "1",
    "true",
    "yes",
    "on",
)
VISION1_META_STALE_SEC = max(0.2, float(os.environ.get("BARTENDER_VISION1_META_STALE_SEC", "2.0")))
VISION2_META_STALE_SEC = max(0.2, float(os.environ.get("BARTENDER_VISION2_META_STALE_SEC", "2.0")))
VISION1_META_WAIT_SEC = max(0.5, float(os.environ.get("BARTENDER_VISION1_META_WAIT_SEC", "5.0")))
VISION2_META_WAIT_SEC = max(0.5, float(os.environ.get("BARTENDER_VISION2_META_WAIT_SEC", "5.0")))
BARTENDER_SEQUENCE_API_HOST = str(os.environ.get("BARTENDER_SEQUENCE_API_HOST", "127.0.0.1") or "").strip() or "127.0.0.1"
try:
    BARTENDER_SEQUENCE_API_PORT = int(os.environ.get("BARTENDER_SEQUENCE_API_PORT", "8765"))
except Exception:
    BARTENDER_SEQUENCE_API_PORT = 8765
ROBOT_ACTION_INGREDIENT_CLASS_ALIASES = {
    "soju": ("soju", "소주"),
    "beer": ("beer", "맥주", "비어"),
}

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
BACKEND_NODE_NAME = str(os.environ.get("BARTENDER_BACKEND_NODE_NAME", "bartender_backend") or "").strip() or "bartender_backend"
DSR_BRIDGE_NODE_NAME = (
    str(os.environ.get("BARTENDER_DSR_BRIDGE_NODE_NAME", "bartender_robot_bridge") or "").strip()
    or "bartender_robot_bridge"
)
MODE_MONITOR_NODE_NAME = (
    str(os.environ.get("BARTENDER_MODE_MONITOR_NODE_NAME", "bartender_mode_monitor") or "").strip()
    or "bartender_mode_monitor"
)
VISION_BRIDGE_NODE_NAME = (
    str(os.environ.get("BARTENDER_VISION_BRIDGE_NODE_NAME", "bartender_vision_bridge") or "").strip()
    or "bartender_vision_bridge"
)


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
        super().__init__(BACKEND_NODE_NAME)
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

    def terminate_gripper(self, best_effort: bool = False):
        if not self.use_real_gripper:
            return
        with self._gripper_lock:
            gripper = self.gripper
        if gripper is not None:
            try:
                gripper.terminate(best_effort=bool(best_effort))
            except TypeError:
                # Backward compatibility for older GripperController implementations.
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

    def move_to_cartesian_pose_async(self, x, y, z, a, b, c, vel=None, acc=None):
        from DSR_ROBOT2 import amovel
        from DR_common2 import posx
        try:
            target = [float(x), float(y), float(z), float(a), float(b), float(c)]
            velx, accx = self._cart_motion_profile(vel=vel, acc=acc)
            self.get_logger().info(
                f"카테시안 비동기 이동: XYZABC=[{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}, "
                f"{target[3]:.3f}, {target[4]:.3f}, {target[5]:.3f}], "
                f"vel=[{velx[0]:.3f}, {velx[1]:.3f}], acc=[{accx[0]:.3f}, {accx[1]:.3f}]"
            )
            ret = amovel(posx(target), vel=velx, acc=accx, radius=0.0, ra=DR_MV_RA_DUPLICATE)
            if not self._motion_ok(ret, "amovel(카테시안 비동기 이동)"):
                return False
            return True
        except Exception as e:
            self.get_logger().error(f"카테시안 비동기 이동 중 오류 발생: {e}")
            return False

    def move_to_joint_pose_async(self, j1, j2, j3, j4, j5, j6, vel=None, acc=None):
        from DSR_ROBOT2 import amovej
        from DR_common2 import posj
        try:
            target = [float(j1), float(j2), float(j3), float(j4), float(j5), float(j6)]
            self.get_logger().info(
                f"조인트 비동기 이동: J=[{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}, "
                f"{target[3]:.3f}, {target[4]:.3f}, {target[5]:.3f}]"
            )
            v = VELOCITY if vel is None else float(vel)
            a = ACC if acc is None else float(acc)
            ret = amovej(posj(*target), vel=v, acc=a, radius=0.0, ra=DR_MV_RA_DUPLICATE)
            if not self._motion_ok(ret, "amovej(조인트 비동기 이동)"):
                return False
            return True
        except Exception as e:
            self.get_logger().error(f"조인트 비동기 이동 중 오류 발생: {e}")
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


class NativeRobotActionApi:
    """Planner API 구현체(네이티브 실행).

    - planner가 호출하는 movel/movej/move_to_detection 등을 즉시 실행한다.
    - 실행시 DSR 원본 API(movel/movej + posx/posj)를 사용한다.
    """

    def __init__(self, backend, offset_map, execute_enabled: bool, move_speed):
        self.backend = backend
        self.robot_controller = backend.robot_controller
        self.offset_map = dict(offset_map or {})
        self.execute_enabled = bool(execute_enabled)
        self.move_speed = move_speed
        self.sequence_steps = []
        self._step_logs = []
        self._resolved_targets = {}

    def _append_step(self, step: dict):
        self.sequence_steps.append(dict(step))

    def _append_log(self, text: str):
        msg = str(text or "").strip()
        if msg:
            self._step_logs.append(msg)
            try:
                # 메인 로그창에서 로봇동작 중간단계/목표값을 바로 확인할 수 있게 출력
                if self.robot_controller is not None:
                    self.robot_controller._log_info(f"[로봇동작] {msg}")
                else:
                    print(f"[로봇동작] {msg}")
            except Exception:
                pass

    @staticmethod
    def _is_resolved_target_pose(pose):
        return isinstance(pose, dict) and ("__resolved_target_key__" in pose)

    def summary_tail(self, tail_count: int = 6):
        if not self._step_logs:
            return "시퀀스 실행 완료"
        return " / ".join(self._step_logs[-max(1, int(tail_count)):])

    def _check_motion(self, label: str, max_state_age_sec: float = 2.0):
        ok_state, state_msg = self.backend._check_motion_precondition(max_state_age_sec=max_state_age_sec)
        if not ok_state:
            raise RuntimeError(f"{label} 중단: {state_msg}")

    def _cart_profile(self, vel=None, acc=None):
        if self.robot_controller is None:
            raise RuntimeError("로봇 컨트롤러가 없습니다.")
        eff_vel = self.move_speed if vel is None else float(vel)
        eff_acc = self.move_speed if acc is None else float(acc)
        return self.robot_controller._cart_motion_profile(vel=eff_vel, acc=eff_acc)

    def _exec_movel_native(self, target6, label: str, vel=None, acc=None):
        from DSR_ROBOT2 import movel, wait
        from DR_common2 import posx

        target = [float(v) for v in list(target6)[:6]]
        if len(target) != 6:
            raise RuntimeError(f"{label} 실패: XYZABC 길이가 올바르지 않습니다.")
        self._check_motion(label, max_state_age_sec=2.0)
        velx, accx = self._cart_profile(vel=vel, acc=acc)
        self._append_log(
            f"{label}(XYZABC={target[0]:.3f},{target[1]:.3f},{target[2]:.3f},{target[3]:.3f},{target[4]:.3f},{target[5]:.3f}, "
            f"vel={velx[0]:.1f}, acc={accx[0]:.1f})"
        )
        ret = movel(posx(target), vel=velx, acc=accx, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        if self.robot_controller is not None and (not self.robot_controller._motion_ok(ret, f"movel({label})")):
            raise RuntimeError(f"{label} 실패: movel 반환값={ret}")
        wait(0.5)

    def _exec_movej_native(self, joints6, label: str, vel=None, acc=None):
        from DSR_ROBOT2 import movej, wait
        from DR_common2 import posj

        joints = [float(v) for v in list(joints6)[:6]]
        if len(joints) != 6:
            raise RuntimeError(f"{label} 실패: J1~J6 길이가 올바르지 않습니다.")
        self._check_motion(label, max_state_age_sec=2.0)
        eff_vel = VELOCITY if vel is None else float(vel)
        eff_acc = ACC if acc is None else float(acc)
        ret = movej(posj(*joints), eff_vel, eff_acc)
        if self.robot_controller is not None and (not self.robot_controller._motion_ok(ret, f"movej({label})")):
            raise RuntimeError(f"{label} 실패: movej 반환값={ret}")
        wait(0.5)

    def _exec_amovel_native(self, target6, label: str, vel=None, acc=None):
        from DSR_ROBOT2 import amovel
        from DR_common2 import posx

        target = [float(v) for v in list(target6)[:6]]
        if len(target) != 6:
            raise RuntimeError(f"{label} 실패: XYZABC 길이가 올바르지 않습니다.")
        self._check_motion(label, max_state_age_sec=2.0)
        velx, accx = self._cart_profile(vel=vel, acc=acc)
        ret = amovel(posx(target), vel=velx, acc=accx, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        if self.robot_controller is not None and (not self.robot_controller._motion_ok(ret, f"amovel({label})")):
            raise RuntimeError(f"{label} 실패: amovel 반환값={ret}")

    def _exec_amovej_native(self, joints6, label: str, vel=None, acc=None):
        from DSR_ROBOT2 import amovej
        from DR_common2 import posj

        joints = [float(v) for v in list(joints6)[:6]]
        if len(joints) != 6:
            raise RuntimeError(f"{label} 실패: J1~J6 길이가 올바르지 않습니다.")
        self._check_motion(label, max_state_age_sec=2.0)
        eff_vel = VELOCITY if vel is None else float(vel)
        eff_acc = ACC if acc is None else float(acc)
        ret = amovej(posj(*joints), vel=eff_vel, acc=eff_acc, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        if self.robot_controller is not None and (not self.robot_controller._motion_ok(ret, f"amovej({label})")):
            raise RuntimeError(f"{label} 실패: amovej 반환값={ret}")

    def _resolve_detection_xyz(
        self,
        *,
        ingredient_code: str,
        center_uv,
        depth_m: float,
        apply_menu_offset: bool,
        extra_offset_xyz_mm,
        debug_label: str | None = None,
    ):
        u = float(center_uv[0])
        v = float(center_uv[1])
        z_mm = float(depth_m) * 1000.0
        if (not math.isfinite(z_mm)) or z_mm <= 0.0:
            raise RuntimeError(f"depth 값 오류({depth_m})")

        robot_xyz, xyz_msg = self.backend._vision1_uvz_to_robot_xyz_mm(u, v, z_mm)
        if robot_xyz is None:
            raise RuntimeError(str(xyz_msg or "UVZ->XYZ 변환 실패"))
        tx, ty, tz = [float(vv) for vv in robot_xyz[:3]]
        base_x, base_y, base_z = tx, ty, tz
        menu_ox, menu_oy, menu_oz = 0.0, 0.0, 0.0

        if bool(apply_menu_offset):
            menu_ox, menu_oy, menu_oz = self.backend._get_menu_xyz_offset(ingredient_code, self.offset_map)
            tx += float(menu_ox)
            ty += float(menu_oy)
            tz += float(menu_oz)

        extra_offset = extra_offset_xyz_mm if isinstance(extra_offset_xyz_mm, (list, tuple)) else [0.0, 0.0, 0.0]
        ex = float(extra_offset[0]) if len(extra_offset) >= 1 else 0.0
        ey = float(extra_offset[1]) if len(extra_offset) >= 2 else 0.0
        ez = float(extra_offset[2]) if len(extra_offset) >= 3 else 0.0
        tx += ex
        ty += ey
        tz += ez
        if debug_label:
            self._append_log(
                f"{debug_label} 계산: base=({base_x:.1f},{base_y:.1f},{base_z:.1f}) + "
                f"menu_offset=({float(menu_ox):.1f},{float(menu_oy):.1f},{float(menu_oz):.1f}) + "
                f"extra_offset=({ex:.1f},{ey:.1f},{ez:.1f}) => final=({tx:.1f},{ty:.1f},{tz:.1f}), "
                f"UV=({u:.1f},{v:.1f}), depth={z_mm:.1f}mm, ingredient={str(ingredient_code or '').strip().lower() or '-'}"
            )
        return tx, ty, tz

    def set_robot_mode(self, mode: int, label: str = "로봇 오토모드 전환", enabled: bool = True):
        self._append_step(
            {
                "op": "set_robot_mode",
                "label": str(label),
                "mode": int(mode),
                "enabled": bool(enabled),
            }
        )
        if not bool(enabled):
            self._append_log(f"{label}(비활성)")
            return
        if not self.execute_enabled:
            self._append_log(f"{label}(계획 mode={int(mode)})")
            return
        # 이미 목표 모드면 불필요한 set 호출을 생략한다.
        try:
            if int(mode) == 1 and hasattr(self.backend, "sync_robot_mode_once"):
                self.backend.sync_robot_mode_once(force=False, timeout_sec=0.8)
            if hasattr(self.backend, "get_robot_mode_snapshot"):
                cur_mode, _seen_at = self.backend.get_robot_mode_snapshot()
                if cur_mode is not None and int(cur_mode) == int(mode):
                    self._append_log(f"{label}(mode={int(mode)} 이미 적용, skip)")
                    return
        except Exception:
            pass
        # NativeRobotActionApi는 run_bartender_first_ingredient_action() 내부 busy 구간에서 실행된다.
        # 여기서 public set_robot_mode()를 호출하면 busy 가드에 걸리므로, 내부 서비스 호출 함수로 우회한다.
        ok_mode, mode_msg = self.backend._call_with_retry(
            lambda: self.backend._call_set_robot_mode(int(mode), timeout_sec=6.0),
            timeout_sec=8.0,
            poll_sec=0.2,
        )
        if not ok_mode:
            raise RuntimeError(f"{label} 실패: {mode_msg}")
        self._append_log(f"{label}(mode={int(mode)})")

    def move_home(self, label: str = "홈 이동", enabled: bool = True):
        self._append_step(
            {
                "op": "backend_call",
                "label": str(label),
                "method": "send_move_home",
                "args": [],
                "kwargs": {},
                "enabled": bool(enabled),
            }
        )
        if not bool(enabled):
            self._append_log(f"{label}(비활성)")
            return
        if not self.execute_enabled:
            self._append_log(f"{label}(계획)")
            return
        home_j = self.backend.get_home_posj()
        self._exec_movej_native(home_j, label=label)
        self._append_log(label)

    def movel_posx(self, pose6, label: str = "카테시안 이동", enabled: bool = True, timeout_sec: float | None = None, vel: float | None = None, acc: float | None = None):
        if self._is_resolved_target_pose(pose6):
            payload = dict(pose6)
            self.movel_resolved_target(
                target_key=str(payload.get("__resolved_target_key__", "")),
                label=str(label),
                approach_up_mm=float(payload.get("approach_up_mm", 0.0)),
                xyzabc=payload.get("xyzabc"),
                offset_xyz_mm=payload.get("offset_xyz_mm"),
                abc=payload.get("abc"),
                enabled=bool(enabled),
                timeout_sec=timeout_sec,
                vel=vel,
                acc=acc,
            )
            return

        vals = [float(v) for v in list(pose6)[:6]]
        kwargs = {}
        if vel is not None:
            kwargs["vel"] = float(vel)
        if acc is not None:
            kwargs["acc"] = float(acc)
        step = {
            "op": "backend_call",
            "label": str(label),
            "method": "send_move_cartesian",
            "args": vals,
            "kwargs": kwargs,
            "enabled": bool(enabled),
        }
        if timeout_sec is not None:
            step["timeout_sec"] = float(timeout_sec)
        self._append_step(step)
        if not bool(enabled):
            self._append_log(f"{label}(비활성)")
            return
        if not self.execute_enabled:
            self._append_log(f"{label}(계획)")
            return
        self._exec_movel_native(vals, label=label, vel=vel, acc=acc)
        self._append_log(f"{label}(XYZ={vals[0]:.1f},{vals[1]:.1f},{vals[2]:.1f})")

    def movej_posj(self, joints6, label: str = "조인트 이동", enabled: bool = True, timeout_sec: float | None = None, vel: float | None = None, acc: float | None = None):
        vals = [float(v) for v in list(joints6)[:6]]
        if len(vals) != 6:
            raise RuntimeError("movej_posj는 6개 조인트 값이 필요합니다.")
        kwargs = {}
        if vel is not None:
            kwargs["vel"] = float(vel)
        if acc is not None:
            kwargs["acc"] = float(acc)
        step = {
            "op": "backend_call",
            "label": str(label),
            "method": "send_move_joint",
            "args": vals,
            "kwargs": kwargs,
            "enabled": bool(enabled),
        }
        if timeout_sec is not None:
            step["timeout_sec"] = float(timeout_sec)
        self._append_step(step)
        if not bool(enabled):
            self._append_log(f"{label}(비활성)")
            return
        if not self.execute_enabled:
            self._append_log(f"{label}(계획)")
            return
        self._exec_movej_native(vals, label=label, vel=vel, acc=acc)
        self._append_log(f"{label}(J={','.join(f'{v:.1f}' for v in vals)})")

    def amovel_posx(self, pose6, label: str = "카테시안 비동기 이동", enabled: bool = True, timeout_sec: float | None = None, vel: float | None = None, acc: float | None = None):
        vals = [float(v) for v in list(pose6)[:6]]
        if len(vals) != 6:
            raise RuntimeError("amovel_posx는 6개 XYZABC 값이 필요합니다.")
        kwargs = {}
        if vel is not None:
            kwargs["vel"] = float(vel)
        if acc is not None:
            kwargs["acc"] = float(acc)
        step = {
            "op": "backend_call",
            "label": str(label),
            "method": "send_move_cartesian_async",
            "args": vals,
            "kwargs": kwargs,
            "enabled": bool(enabled),
        }
        if timeout_sec is not None:
            step["timeout_sec"] = float(timeout_sec)
        self._append_step(step)
        if not bool(enabled):
            self._append_log(f"{label}(비활성)")
            return
        if not self.execute_enabled:
            self._append_log(f"{label}(계획)")
            return
        self._exec_amovel_native(vals, label=label, vel=vel, acc=acc)
        self._append_log(f"{label}(XYZ={vals[0]:.1f},{vals[1]:.1f},{vals[2]:.1f})")

    def amovej_posj(self, joints6, label: str = "조인트 비동기 이동", enabled: bool = True, timeout_sec: float | None = None, vel: float | None = None, acc: float | None = None):
        vals = [float(v) for v in list(joints6)[:6]]
        if len(vals) != 6:
            raise RuntimeError("amovej_posj는 6개 조인트 값이 필요합니다.")
        kwargs = {}
        if vel is not None:
            kwargs["vel"] = float(vel)
        if acc is not None:
            kwargs["acc"] = float(acc)
        step = {
            "op": "backend_call",
            "label": str(label),
            "method": "send_move_joint_async",
            "args": vals,
            "kwargs": kwargs,
            "enabled": bool(enabled),
        }
        if timeout_sec is not None:
            step["timeout_sec"] = float(timeout_sec)
        self._append_step(step)
        if not bool(enabled):
            self._append_log(f"{label}(비활성)")
            return
        if not self.execute_enabled:
            self._append_log(f"{label}(계획)")
            return
        self._exec_amovej_native(vals, label=label, vel=vel, acc=acc)
        self._append_log(f"{label}(J={','.join(f'{v:.1f}' for v in vals)})")

    # backward compatibility aliases
    def movel(self, pose6, label: str = "카테시안 이동", enabled: bool = True, timeout_sec: float | None = None, vel: float | None = None, acc: float | None = None):
        self.movel_posx(pose6=pose6, label=label, enabled=enabled, timeout_sec=timeout_sec, vel=vel, acc=acc)

    def movej(self, joints6, label: str = "조인트 이동", enabled: bool = True, timeout_sec: float | None = None, vel: float | None = None, acc: float | None = None):
        self.movej_posj(joints6=joints6, label=label, enabled=enabled, timeout_sec=timeout_sec, vel=vel, acc=acc)

    def amovel(self, pose6, label: str = "카테시안 비동기 이동", enabled: bool = True, timeout_sec: float | None = None, vel: float | None = None, acc: float | None = None):
        self.amovel_posx(pose6=pose6, label=label, enabled=enabled, timeout_sec=timeout_sec, vel=vel, acc=acc)

    def amovej(self, joints6, label: str = "조인트 비동기 이동", enabled: bool = True, timeout_sec: float | None = None, vel: float | None = None, acc: float | None = None):
        self.amovej_posj(joints6=joints6, label=label, enabled=enabled, timeout_sec=timeout_sec, vel=vel, acc=acc)

    def gripper(self, distance_mm: float, label: str = "그리퍼 이동", enabled: bool = True):
        self._append_step(
            {
                "op": "backend_call",
                "label": str(label),
                "method": "send_gripper_move",
                "args": [float(distance_mm)],
                "kwargs": {},
                "enabled": bool(enabled),
            }
        )
        if not bool(enabled):
            self._append_log(f"{label}(비활성)")
            return
        if not self.execute_enabled:
            self._append_log(f"{label}(계획 {float(distance_mm):.1f}mm)")
            return
        gripper_ok, gripper_msg = self.backend._check_gripper_precondition()
        if not gripper_ok:
            raise RuntimeError(f"{label} 실패: {gripper_msg}")
        ok_gripper = self.robot_controller.move_gripper_manual(float(distance_mm))
        if not ok_gripper:
            raise RuntimeError(f"{label} 실패: distance={float(distance_mm):.1f}mm")
        self._append_log(f"{label}({float(distance_mm):.1f}mm)")

    def wait_sec(self, seconds: float, label: str = "대기", enabled: bool = True):
        wait_s = max(0.0, float(seconds))
        self._append_step(
            {
                "op": "wait_sec",
                "label": str(label),
                "seconds": float(wait_s),
                "enabled": bool(enabled),
            }
        )
        if not bool(enabled):
            self._append_log(f"{label}(비활성)")
            return
        if not self.execute_enabled:
            self._append_log(f"{label}(계획 {wait_s:.2f}s)")
            return
        end_t = time.monotonic() + float(wait_s)
        while time.monotonic() < end_t:
            self._check_motion(label, max_state_age_sec=2.0)
            time.sleep(min(0.05, max(0.0, end_t - time.monotonic())))
        self._append_log(f"{label}({wait_s:.2f}s)")

    def log_event(self, message: str, level: str = "info", enabled: bool = True):
        text = str(message or "").strip()
        if not text:
            return
        lvl = str(level or "info").strip().lower() or "info"
        self._append_step(
            {
                "op": "log_event",
                "label": text,
                "level": lvl,
                "enabled": bool(enabled),
            }
        )
        if not bool(enabled):
            return
        if not self.execute_enabled:
            self._append_log(f"{text}(계획)")
            return
        if lvl in ("error", "warn", "warning"):
            self._append_log(f"[{lvl}] {text}")
            return
        self._append_log(text)

    def motion_stop(self, label: str = "모션 정지", stop_mode: int = 2, enabled: bool = True):
        mode = int(stop_mode)
        self._append_step(
            {
                "op": "backend_call",
                "label": str(label),
                "method": "send_motion_stop",
                "args": [mode],
                "kwargs": {},
                "enabled": bool(enabled),
            }
        )
        if not bool(enabled):
            self._append_log(f"{label}(비활성)")
            return
        if not self.execute_enabled:
            self._append_log(f"{label}(계획 stop_mode={mode})")
            return
        ok_stop, stop_msg = self.backend.send_motion_stop(stop_mode=mode)
        if not ok_stop:
            raise RuntimeError(f"{label} 실패: {stop_msg}")
        self._append_log(f"{label}(stop_mode={mode})")

    def emergency_stop(self, label: str = "정지", enabled: bool = True):
        # backward compatibility alias: 기존 emergency_stop 호출을 모션 정지로 연결
        self.motion_stop(label=label, stop_mode=2, enabled=enabled)

    def resolve_detection_target(
        self,
        *,
        target_key: str,
        ingredient_code: str,
        center_uv,
        depth_m: float,
        label: str = "비전 타겟 계산",
        extra_offset_xyz_mm=None,
        apply_menu_offset: bool = True,
        enabled: bool = True,
    ):
        key = str(target_key or "").strip()
        if not key:
            raise RuntimeError("target_key가 비어 있습니다.")
        self._append_step(
            {
                "op": "resolve_detection_target",
                "label": str(label),
                "target_key": key,
                "ingredient_code": str(ingredient_code),
                "target_uv": [float(center_uv[0]), float(center_uv[1])],
                "target_depth_m": float(depth_m),
                "apply_menu_offset": bool(apply_menu_offset),
                "extra_offset_xyz_mm": list(extra_offset_xyz_mm or [0.0, 0.0, 0.0]),
                "enabled": bool(enabled),
            }
        )
        if not bool(enabled):
            self._append_log(f"{label}(비활성)")
            return
        tx, ty, tz = self._resolve_detection_xyz(
            ingredient_code=str(ingredient_code),
            center_uv=center_uv,
            depth_m=float(depth_m),
            apply_menu_offset=bool(apply_menu_offset),
            extra_offset_xyz_mm=extra_offset_xyz_mm,
            debug_label=str(label or "비전 타겟 계산"),
        )
        self._resolved_targets[key] = (tx, ty, tz)
        self._append_log(f"{label}(XYZ={tx:.1f},{ty:.1f},{tz:.1f})")

    def get_resolved_target_pose(self, *, target_key: str, approach_up_mm: float = 0.0, xyzabc=None, offset_xyz_mm=None, abc=None):
        payload = {
            "__resolved_target_key__": str(target_key),
            "approach_up_mm": float(approach_up_mm),
        }
        if xyzabc is not None:
            payload["xyzabc"] = list(xyzabc)
        if offset_xyz_mm is not None:
            payload["offset_xyz_mm"] = list(offset_xyz_mm)
        if abc is not None:
            payload["abc"] = list(abc)
        return payload

    def add_pose_offset_xyz(self, pose_payload, *, dx_mm: float = 0.0, dy_mm: float = 0.0, dz_mm: float = 0.0):
        if not isinstance(pose_payload, dict):
            raise RuntimeError("pose_payload는 get_resolved_target_pose() 반환 dict여야 합니다.")
        base = pose_payload.get("offset_xyz_mm", [0.0, 0.0, 0.0])
        try:
            bx = float(base[0]) if isinstance(base, (list, tuple)) and len(base) >= 1 else 0.0
            by = float(base[1]) if isinstance(base, (list, tuple)) and len(base) >= 2 else 0.0
            bz = float(base[2]) if isinstance(base, (list, tuple)) and len(base) >= 3 else 0.0
        except Exception:
            bx, by, bz = 0.0, 0.0, 0.0
        pose_payload["offset_xyz_mm"] = [
            float(bx) + float(dx_mm),
            float(by) + float(dy_mm),
            float(bz) + float(dz_mm),
        ]
        return pose_payload

    def movel_resolved_target(
        self,
        *,
        target_key: str,
        label: str = "비전 타겟 이동",
        approach_up_mm: float = 0.0,
        xyzabc=None,
        offset_xyz_mm=None,
        abc=None,
        enabled: bool = True,
        timeout_sec: float | None = None,
        vel: float | None = None,
        acc: float | None = None,
    ):
        key = str(target_key or "").strip()
        if not key:
            raise RuntimeError("target_key가 비어 있습니다.")
        step = {
            "op": "movel_resolved_target",
            "label": str(label),
            "target_key": key,
            "approach_up_mm": float(approach_up_mm),
            "enabled": bool(enabled),
        }
        if xyzabc is not None:
            step["xyzabc"] = list(xyzabc)
        if offset_xyz_mm is not None:
            step["offset_xyz_mm"] = list(offset_xyz_mm)
        if abc is not None:
            step["abc"] = list(abc)
        if timeout_sec is not None:
            step["timeout_sec"] = float(timeout_sec)
        if vel is not None:
            step["vel"] = float(vel)
        if acc is not None:
            step["acc"] = float(acc)
        self._append_step(step)
        if not bool(enabled):
            self._append_log(f"{label}(비활성)")
            return

        if key not in self._resolved_targets:
            raise RuntimeError(f"{label} 실패: target_key={key} 미해결")
        tx, ty, tz = self._resolved_targets[key]
        xyzabc_vals = xyzabc if isinstance(xyzabc, (list, tuple)) else None
        if xyzabc_vals is not None and len(xyzabc_vals) >= 3:
            try:
                tx += float(xyzabc_vals[0])
                ty += float(xyzabc_vals[1])
                tz += float(xyzabc_vals[2])
            except Exception:
                pass
        offset = offset_xyz_mm if isinstance(offset_xyz_mm, (list, tuple)) else [0.0, 0.0, 0.0]
        try:
            tx += float(offset[0]) if len(offset) >= 1 else 0.0
            ty += float(offset[1]) if len(offset) >= 2 else 0.0
            tz += float(offset[2]) if len(offset) >= 3 else 0.0
        except Exception:
            pass
        if xyzabc_vals is not None and len(xyzabc_vals) >= 6:
            try:
                a, b, c = float(xyzabc_vals[3]), float(xyzabc_vals[4]), float(xyzabc_vals[5])
            except Exception:
                (a, b, c), _abc_source = self.backend._resolve_robot_action_abc()
        elif isinstance(abc, (list, tuple)) and len(abc) >= 3:
            a, b, c = float(abc[0]), float(abc[1]), float(abc[2])
        else:
            (a, b, c), _abc_source = self.backend._resolve_robot_action_abc()
        up_mm = max(0.0, float(approach_up_mm))
        if not self.execute_enabled:
            self._append_log(f"{label}(계획 XYZ={tx:.1f},{ty:.1f},{tz:.1f})")
            return
        if up_mm > 0.0:
            self._exec_movel_native([tx, ty, tz + up_mm, a, b, c], label=f"{label} 접근", vel=vel, acc=acc)
        self._exec_movel_native([tx, ty, tz, a, b, c], label=label, vel=vel, acc=acc)
        self._append_log(f"{label}(XYZ={tx:.1f},{ty:.1f},{tz:.1f})")

    def move_to_detection(
        self,
        *,
        ingredient_code: str,
        center_uv,
        depth_m: float,
        label: str,
        extra_offset_xyz_mm=None,
        approach_up_mm: float = 40.0,
        apply_menu_offset: bool = True,
        enabled: bool = True,
    ):
        key = f"legacy_{str(label)}_{str(ingredient_code)}"
        self.resolve_detection_target(
            target_key=key,
            ingredient_code=ingredient_code,
            center_uv=center_uv,
            depth_m=depth_m,
            label=f"{label} (타겟계산)",
            extra_offset_xyz_mm=extra_offset_xyz_mm,
            apply_menu_offset=apply_menu_offset,
            enabled=enabled,
        )
        pose = self.get_resolved_target_pose(target_key=key, approach_up_mm=approach_up_mm)
        self.movel_posx(pose, label=label, enabled=enabled)

    def wait_volume_target(
        self,
        target_volume_ml: float,
        label: str,
        timeout_sec: float = 20.0,
        poll_sec: float = 0.1,
        condition: str = "gte",
        compare_tolerance_ml: float = 0.5,
        enabled: bool = True,
    ):
        cond, target = _resolve_volume_condition_target(condition, target_volume_ml)
        tol = max(0.0, float(compare_tolerance_ml))
        cond_text = _format_volume_condition(cond, target)
        self._append_step(
            {
                "op": "wait_volume_target",
                "label": str(label),
                "target_volume_ml": float(target),
                "timeout_sec": float(timeout_sec),
                "poll_sec": float(poll_sec),
                "condition": cond,
                "condition_text": cond_text,
                "compare_tolerance_ml": tol,
                "enabled": bool(enabled),
            }
        )
        if not bool(enabled):
            self._append_log(f"{label}(비활성)")
            return
        if not self.execute_enabled:
            self._append_log(f"{label}(계획 {cond_text})")
            return

        timeout_s = max(0.5, float(timeout_sec))
        poll_s = max(0.05, float(poll_sec))
        start_at = time.monotonic()
        last_volume = None
        while (time.monotonic() - start_at) <= timeout_s:
            payload, seen_at = self.backend.get_vision2_meta_snapshot()
            if payload is None or seen_at is None:
                time.sleep(poll_s)
                continue
            age = time.monotonic() - float(seen_at)
            if age > float(VISION2_META_STALE_SEC):
                time.sleep(poll_s)
                continue
            volume_ml = self.backend._extract_vision2_volume_ml(payload)
            if volume_ml is None:
                time.sleep(poll_s)
                continue
            last_volume = float(volume_ml)
            if _is_volume_condition_met(last_volume, target, condition=cond, tolerance_ml=tol):
                self._append_log(
                    f"{label}(volume={last_volume:.1f}ml, condition={cond_text})"
                )
                return
            time.sleep(poll_s)
        if last_volume is None:
            raise RuntimeError(f"{label} 실패: vision2 용량값을 읽지 못했습니다.")
        raise RuntimeError(
            f"{label} 실패: 용량 조건 미충족(current={last_volume:.1f}ml, condition={cond_text})"
        )

    def get_current_volume_ml(self, *, max_age_sec: float | None = None) -> float:
        volume_ml, msg = self.backend.get_current_volume_ml(max_age_sec=max_age_sec)
        if volume_ml is None:
            raise RuntimeError(f"vision2 현재용량 조회 실패: {msg}")
        return float(volume_ml)


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
        self._vision_spin_thread = None
        self._worker_thread = None
        self._gripper_ready_event = threading.Event()
        self._gripper_init_started = False
        self._gripper_init_error = None
        self._gripper_init_lock = threading.Lock()

        self._executor = None
        self._vision_executor = None
        self._dsr_node = None
        self._mode_node = None
        self._vision_bridge_node = None
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
        self._servo_off_clients = {}
        self._move_stop_clients = {}
        self._move_stop_service_lock = threading.Lock()
        self._move_stop_preferred_srv = None
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
        self._voice_order_lock = threading.Lock()
        self._voice_order_last_payload = None
        self._voice_order_last_seen_at = None
        self._bartender_sequence_manager = BartenderSequenceManager(self)
        self._bartender_sequence_api_server = None
        self._vision1_meta_lock = threading.Lock()
        self._vision1_meta_payload = None
        self._vision1_meta_seen_at = None
        self._vision1_meta_sub = None
        self._vision2_meta_lock = threading.Lock()
        self._vision2_meta_payload = None
        self._vision2_meta_seen_at = None
        self._vision2_meta_sub = None
        self._vision1_intrinsics_lock = threading.Lock()
        self._vision1_intrinsics = None  # (fx, fy, cx, cy)
        self._vision1_camera_info_sub = None
        self._vision1_camera_info_topic_in_use = None
        self._vision1_affine_lock = threading.Lock()
        self._vision1_affine_3x4 = None
        self._vision1_affine_path = None
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
        self._dsr_node = rclpy.create_node(DSR_BRIDGE_NODE_NAME, namespace=ROBOT_ID)
        # 모드 set/get 서비스 전용 노드(무브 API와 분리)
        self._mode_node = rclpy.create_node(MODE_MONITOR_NODE_NAME, namespace=ROBOT_ID)
        # 비전 UI 구독 전용 노드(로봇 제어 노드와 callback 부하 분리)
        self._vision_bridge_node = rclpy.create_node(VISION_BRIDGE_NODE_NAME, namespace=ROBOT_ID)
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
        self._vision_executor = MultiThreadedExecutor(num_threads=1)
        if self._vision_bridge_node is not None:
            self._vision_executor.add_node(self._vision_bridge_node)
    
        self._spin_thread = threading.Thread(target=self._spin_bg, daemon=True)
        self._spin_thread.start()
        self._vision_spin_thread = threading.Thread(target=self._spin_vision_bg, daemon=True)
        self._vision_spin_thread.start()

        self._detect_connection_mode_once(progress_callback=progress_callback)
        self._setup_position_source(progress_callback, wait_timeout_sec=0.15)
        self._setup_robot_mode_source(progress_callback)
        self._setup_bartender_action_sources(progress_callback=progress_callback)
        self._report_gripper_startup_plan(progress_callback=progress_callback)
        self._report_progress(progress_callback, 99, "명령 워커 시작 중...")

        self._worker_thread = threading.Thread(target=self._command_loop, daemon=True)
        self._worker_thread.start()

        self._ready_event.set()
        self._started = True
        self._start_bartender_sequence_api_server(progress_callback=progress_callback)
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

    def _spin_vision_bg(self):
        last_warn_at = 0.0
        while not self._stop_event.is_set():
            try:
                if self._vision_executor is None:
                    time.sleep(0.05)
                    continue
                if not rclpy.ok():
                    break
                self._vision_executor.spin_once(timeout_sec=0.05)
            except Exception as e:
                msg = str(e)
                if ("Destroyable" in msg) or ("destruction was requested" in msg):
                    if self._stop_event.is_set():
                        break
                    time.sleep(0.01)
                    continue
                now = time.monotonic()
                if (now - last_warn_at) > 1.0:
                    print(f"vision executor.spin_once 예외: {e}")
                    last_warn_at = now
                time.sleep(0.03)

    def get_vision_subscription_node(self):
        if self._vision_bridge_node is not None:
            return self._vision_bridge_node
        return self.robot_controller

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
                finally:
                    self._busy_event.clear()
            elif cmd in ("move_cartesian", "move_cartesian_async"):
                if self._busy_event.is_set():
                    continue

                self._busy_event.set()
                try:
                    is_async = (cmd == "move_cartesian_async")
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
                    if is_async:
                        ok_move = self.robot_controller.move_to_cartesian_pose_async(x, y, z, a, b, c, vel=vel, acc=acc)
                    else:
                        ok_move = self.robot_controller.move_to_cartesian_pose(x, y, z, a, b, c, vel=vel, acc=acc)
                    if not ok_move:
                        self.robot_controller._log_error("카테시안 비동기 이동 실패" if is_async else "카테시안 이동 실패")
                finally:
                    self._busy_event.clear()
            elif cmd in ("move_joint", "move_joint_async"):
                if self._busy_event.is_set():
                    continue

                self._busy_event.set()
                try:
                    is_async = (cmd == "move_joint_async")
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
                    if is_async:
                        ok_move = self.robot_controller.move_to_joint_pose_async(j1, j2, j3, j4, j5, j6, vel=vel, acc=acc)
                    else:
                        ok_move = self.robot_controller.move_to_joint_pose(j1, j2, j3, j4, j5, j6, vel=vel, acc=acc)
                    if not ok_move:
                        self.robot_controller._log_error("조인트 비동기 이동 실패" if is_async else "조인트 이동 실패")
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
        if self._stop_event.is_set():
            return
        if not rclpy.ok():
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
            try:
                topic_map = dict(self.robot_controller.get_topic_names_and_types())
            except Exception as exc:
                self.robot_controller._log_info(f"로봇모드 토픽 자동 전환 보류: 토픽 조회 실패({exc})")
                return
            rt_topics = [
                t
                for t, types in topic_map.items()
                if "dsr_msgs2/msg/RobotStateRt" in list(types if isinstance(types, (list, tuple)) else [])
            ]
            if not rt_topics:
                return
            preferred = [t for t in rt_topics if f"/{ROBOT_ID}/" in t]
            topic = sorted(preferred or rt_topics)[0]
            try:
                self._robot_mode_sub = self.robot_controller.create_subscription(
                    RobotStateRt, topic, self._on_robot_mode_rt, 10
                )
            except Exception as exc:
                self._robot_mode_sub = None
                self.robot_controller._log_info(f"로봇모드 토픽 자동 전환 보류: 구독 생성 실패({topic}, {exc})")
                return
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

    def _call_servo_off(self, stop_type: int = 3, timeout_sec: float = 2.0):
        if ServoOff is None:
            return False, "ServoOff 서비스 타입 import 실패"
        if self._dsr_node is None:
            return False, "DSR 노드가 없어 서보오프 서비스를 호출할 수 없습니다."
        req = ServoOff.Request()
        req.stop_type = int(stop_type)
        ok, _res, srv_name, msg = self._call_service_with_candidates(
            node=self._dsr_node,
            srv_type=ServoOff,
            cache=self._servo_off_clients,
            relative_name="system/servo_off",
            req=req,
            timeout_sec=timeout_sec,
            action_name="긴급정지 서비스",
        )
        if not ok:
            return False, f"{msg}(stop_type={int(stop_type)})"
        return True, f"긴급정지 서비스 성공(stop_type={int(stop_type)}, srv={srv_name})"

    def _call_move_stop(self, stop_mode: int = 2, timeout_sec: float = 2.0):
        if MoveStop is None:
            return False, "MoveStop 서비스 타입 import 실패"
        if self._dsr_node is None:
            return False, "DSR 노드가 없어 모션정지 서비스를 호출할 수 없습니다."
        req = MoveStop.Request()
        mode = int(stop_mode)
        req.stop_mode = mode

        # move_stop은 "응답이 늦어도 실제 정지는 즉시 반영"되는 경우가 있어
        # 짧은 ACK 대기 후 상태 검증 경로(send_motion_stop)로 빠르게 넘긴다.
        call_timeout = max(0.12, min(0.9, float(timeout_sec)))
        wait_timeout = max(0.08, min(0.2, call_timeout))
        candidates = list(self._service_name_candidates("motion/move_stop"))
        with self._move_stop_service_lock:
            preferred = self._move_stop_preferred_srv
        if preferred in candidates:
            candidates = [preferred] + [name for name in candidates if name != preferred]

        last_msg = "모션정지 서비스가 준비되지 않았습니다."
        for srv_name in candidates:
            cli = self._get_or_create_service_client(self._dsr_node, self._move_stop_clients, MoveStop, srv_name)
            if not cli.wait_for_service(timeout_sec=wait_timeout):
                last_msg = f"{srv_name} 서비스가 준비되지 않았습니다."
                continue

            try:
                future = cli.call_async(req)
            except Exception as exc:
                last_msg = f"{srv_name} 서비스 호출 실패: {exc}"
                continue

            end_at = time.monotonic() + call_timeout
            while time.monotonic() < end_at:
                if future.done():
                    break
                time.sleep(0.01)

            if not future.done():
                with self._move_stop_service_lock:
                    self._move_stop_preferred_srv = srv_name
                self._watch_move_stop_future(future, srv_name=srv_name, stop_mode=mode)
                return True, f"모션정지 요청 전송(stop_mode={mode}, srv={srv_name}, ack=지연허용)"

            if future.exception() is not None:
                last_msg = f"{srv_name} 서비스 예외: {future.exception()}"
                continue

            res = future.result()
            if res is None or not bool(getattr(res, "success", False)):
                last_msg = f"{srv_name} 서비스 실패"
                continue

            with self._move_stop_service_lock:
                self._move_stop_preferred_srv = srv_name
            return True, f"모션정지 서비스 성공(stop_mode={mode}, srv={srv_name})"

        return False, f"{last_msg}(stop_mode={mode})"

    def _watch_move_stop_future(self, future, *, srv_name: str, stop_mode: int):
        def _worker():
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                if future.done():
                    break
                time.sleep(0.02)
            if not future.done():
                return
            if self.robot_controller is None:
                return
            try:
                if future.exception() is not None:
                    self.robot_controller._log_error(
                        f"[정지] move_stop 지연 응답 실패(srv={srv_name}, stop_mode={int(stop_mode)}): {future.exception()}"
                    )
                    return
                res = future.result()
                ok = bool(getattr(res, "success", False))
                if ok:
                    self.robot_controller._log_info(
                        f"[정지] move_stop 지연 응답 확인(srv={srv_name}, stop_mode={int(stop_mode)})"
                    )
            except Exception:
                return

        threading.Thread(target=_worker, daemon=True).start()

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

    def run_voice_order_runtime(
        self,
        input_text: str,
        recommend_menu: str = "",
        allow_llm: bool = True,
        request_stt: bool = False,
        audio_bytes=None,
        audio_filename: str = "recording.webm",
        timeout_sec: float = VOICE_ORDER_RUNTIME_TIMEOUT_SEC,
        event_callback=None,
        cancel_checker=None,
    ):
        text = str(input_text or "").strip()
        rec = str(recommend_menu or "").strip()
        llm_on = bool(allow_llm)
        req_stt = bool(request_stt) or (not bool(text))
        ok, payload, message = run_voice_order_request(
            input_text=text,
            recommend_menu=rec,
            allow_llm=llm_on,
            request_stt=req_stt,
            audio_bytes=audio_bytes,
            audio_filename=str(audio_filename or "recording.webm"),
            timeout_sec=float(timeout_sec),
            event_callback=event_callback,
            cancel_checker=cancel_checker,
        )
        with self._voice_order_lock:
            self._voice_order_last_payload = payload
            self._voice_order_last_seen_at = time.monotonic()
        if ok:
            return True, payload, str(message or "음성처리 결과 수신 완료")
        return False, payload, str(message or "음성처리 실패")

    def get_voice_order_snapshot(self):
        with self._voice_order_lock:
            payload = self._voice_order_last_payload
            seen_at = self._voice_order_last_seen_at
            if payload is None:
                return None, seen_at
            return dict(payload), seen_at

    def start_bartender_sequence(self, mode: str = "manual", request: dict | None = None):
        return self._bartender_sequence_manager.start(mode=mode, request=request)

    def stop_bartender_sequence(self, reason: str = ""):
        return self._bartender_sequence_manager.stop(reason=reason)

    def reset_bartender_sequence(self, reason: str = ""):
        return self._bartender_sequence_manager.reset(reason=reason)

    def get_bartender_sequence_snapshot(self):
        return self._bartender_sequence_manager.get_snapshot()

    def get_bartender_sequence_precheck(self, mode: str = "auto"):
        return self._bartender_sequence_manager.get_precheck(mode=mode)

    def notify_bartender_tts_done(self, run_id: int | None = None):
        return self._bartender_sequence_manager.notify_tts_done(run_id=run_id)

    def _start_bartender_sequence_api_server(self, progress_callback=None):
        if self._bartender_sequence_api_server is not None:
            return True, "시퀀스 API 서버 이미 실행 중"
        server = BartenderSequenceApiServer(
            self,
            host=BARTENDER_SEQUENCE_API_HOST,
            port=BARTENDER_SEQUENCE_API_PORT,
        )
        ok, msg = server.start()
        if ok:
            self._bartender_sequence_api_server = server
        self._report_progress(progress_callback, 100, msg)
        if self.robot_controller is not None:
            if ok:
                self.robot_controller._log_info(msg)
            else:
                self.robot_controller._log_error(msg)
        return ok, msg

    def _stop_bartender_sequence_api_server(self):
        server = self._bartender_sequence_api_server
        self._bartender_sequence_api_server = None
        if server is None:
            return True, "시퀀스 API 서버 미실행"
        return server.stop()

    def _setup_bartender_action_sources(self, progress_callback=None):
        if self.robot_controller is None:
            return False, "로봇 컨트롤러가 준비되지 않았습니다."

        if RosStringMsg is None:
            self._report_progress(progress_callback, 94, "vision1 메타 구독 비활성화: std_msgs/String import 실패")
        elif self._vision1_meta_sub is None:
            try:
                self._vision1_meta_sub = self.robot_controller.create_subscription(
                    RosStringMsg,
                    VISION1_META_TOPIC,
                    self._on_vision1_meta_msg,
                    30,
                )
                self._report_progress(progress_callback, 94, f"vision1 메타 구독 시작: {VISION1_META_TOPIC}")
            except Exception as e:
                if self.robot_controller is not None:
                    self.robot_controller._log_error(f"vision1 메타 구독 실패: {e}")

        if RosStringMsg is None:
            self._report_progress(progress_callback, 94, "vision2 메타 구독 비활성화: std_msgs/String import 실패")
        elif self._vision2_meta_sub is None:
            try:
                self._vision2_meta_sub = self.robot_controller.create_subscription(
                    RosStringMsg,
                    VISION2_META_TOPIC,
                    self._on_vision2_meta_msg,
                    30,
                )
                self._report_progress(progress_callback, 94, f"vision2 메타 구독 시작: {VISION2_META_TOPIC}")
            except Exception as e:
                if self.robot_controller is not None:
                    self.robot_controller._log_error(f"vision2 메타 구독 실패: {e}")

        if RosCameraInfoMsg is None:
            self._report_progress(progress_callback, 94, "vision1 카메라정보 구독 비활성화: CameraInfo import 실패")
        elif self._vision1_camera_info_sub is None:
            try:
                topic = str(VISION1_CAMERA_INFO_TOPIC_PRIMARY or "").strip()
                self._vision1_camera_info_sub = self.robot_controller.create_subscription(
                    RosCameraInfoMsg,
                    topic,
                    self._on_vision1_camera_info_msg,
                    10,
                )
                self._vision1_camera_info_topic_in_use = topic
                self._report_progress(progress_callback, 94, f"vision1 CameraInfo 구독 시작: {topic}")
            except Exception as e:
                if self.robot_controller is not None:
                    self.robot_controller._log_error(f"vision1 CameraInfo 구독 실패: {e}")

        ok_aff, msg_aff = self._load_vision1_affine_from_file(force=False)
        if not ok_aff and self.robot_controller is not None:
            self.robot_controller._log_error(msg_aff)
        return ok_aff, msg_aff

    def _stop_bartender_action_sources(self):
        with self._source_lock:
            self._destroy_sub_safe(self._vision1_meta_sub)
            self._destroy_sub_safe(self._vision2_meta_sub)
            self._destroy_sub_safe(self._vision1_camera_info_sub)
            self._vision1_meta_sub = None
            self._vision2_meta_sub = None
            self._vision1_camera_info_sub = None
            self._vision1_camera_info_topic_in_use = None
        return True, "bartender action 구독 종료 완료"

    def _on_vision1_meta_msg(self, msg):
        raw = str(getattr(msg, "data", "") or "").strip()
        if not raw:
            return
        payload = None
        try:
            decoded = json.loads(raw)
            if isinstance(decoded, dict):
                payload = dict(decoded)
        except Exception:
            payload = None
        if payload is None:
            return
        with self._vision1_meta_lock:
            self._vision1_meta_payload = payload
            self._vision1_meta_seen_at = time.monotonic()

    def _on_vision2_meta_msg(self, msg):
        raw = str(getattr(msg, "data", "") or "").strip()
        if not raw:
            return
        payload = None
        try:
            decoded = json.loads(raw)
            if isinstance(decoded, dict):
                payload = dict(decoded)
        except Exception:
            payload = None
        if payload is None:
            return
        with self._vision2_meta_lock:
            self._vision2_meta_payload = payload
            self._vision2_meta_seen_at = time.monotonic()

    def _on_vision1_camera_info_msg(self, msg):
        try:
            k = getattr(msg, "k", None)
            if k is None or len(k) < 9:
                return
            fx = float(k[0])
            fy = float(k[4])
            cx = float(k[2])
            cy = float(k[5])
            if fx <= 1e-9 or fy <= 1e-9:
                return
        except Exception:
            return
        with self._vision1_intrinsics_lock:
            self._vision1_intrinsics = (fx, fy, cx, cy)

    def _resolve_project_path(self, path_text: str):
        path = str(path_text or "").strip()
        if not path:
            return ""
        if os.path.isabs(path):
            return path
        return os.path.normpath(os.path.join(PROJECT_ROOT, path))

    def _candidate_vision1_affine_paths(self):
        candidates = []
        try:
            rows = self._load_param_rows()
            for key in ("active_calibration_path_1", "active_calibration_path"):
                values = rows.get(key, [])
                if values:
                    resolved = self._resolve_project_path(values[0])
                    if resolved and os.path.isfile(resolved):
                        candidates.append(resolved)
        except Exception:
            pass

        try:
            env_path = self._resolve_project_path(os.environ.get("BARTENDER_VISION1_AFFINE_PATH", ""))
            if env_path and os.path.isfile(env_path):
                candidates.append(env_path)
        except Exception:
            pass

        try:
            paths = sorted(glob.glob(os.path.join(CALIB_DIR, "calib_matrix_*.txt")), key=os.path.getmtime, reverse=True)
            candidates.extend([p for p in paths if os.path.isfile(p)])
        except Exception:
            pass

        deduped = []
        seen = set()
        for path in candidates:
            abs_path = os.path.abspath(path)
            if abs_path in seen:
                continue
            seen.add(abs_path)
            deduped.append(abs_path)
        return deduped

    def _parse_affine_file(self, file_path: str):
        rows = {}
        with open(file_path, "r", encoding="utf-8") as f:
            for raw in f:
                line = str(raw or "").strip()
                if (not line) or line.startswith("#") or ("=" not in line):
                    continue
                key, val = line.split("=", 1)
                key = str(key or "").strip().lower()
                if key not in ("m0", "m1", "m2", "m3", "r0", "r1", "r2", "t"):
                    continue
                parts = [p.strip() for p in str(val or "").split(",")]
                if len(parts) < 3:
                    continue
                try:
                    rows[key] = [float(parts[0]), float(parts[1]), float(parts[2])]
                except Exception:
                    continue

        if all(k in rows for k in ("m0", "m1", "m2", "m3")):
            return [rows["m0"], rows["m1"], rows["m2"], rows["m3"]]
        if all(k in rows for k in ("r0", "r1", "r2", "t")):
            return [rows["r0"], rows["r1"], rows["r2"], rows["t"]]
        raise ValueError("유효한 affine(m0~m3 또는 r0~r2,t) 데이터가 없습니다.")

    def _load_vision1_affine_from_file(self, force: bool = False):
        with self._vision1_affine_lock:
            has_cached = self._vision1_affine_3x4 is not None
            cached_path = os.path.abspath(self._vision1_affine_path) if self._vision1_affine_path else ""

        candidates = self._candidate_vision1_affine_paths()
        preferred_path = os.path.abspath(candidates[0]) if candidates else ""
        if (not force) and has_cached:
            if preferred_path and cached_path and (preferred_path == cached_path):
                return True, f"vision1 affine 유지: {self._vision1_affine_path or '-'}"
            if not preferred_path:
                return True, f"vision1 affine 유지: {self._vision1_affine_path or '-'}"

        last_err = "vision1 affine 파일을 찾지 못했습니다."
        for path in candidates:
            try:
                affine = self._parse_affine_file(path)
                with self._vision1_affine_lock:
                    self._vision1_affine_3x4 = [list(row[:3]) for row in affine[:4]]
                    self._vision1_affine_path = path
                if cached_path and (os.path.abspath(path) != cached_path):
                    return True, f"vision1 affine 갱신 로드 완료: {cached_path} -> {path}"
                return True, f"vision1 affine 로드 완료: {path}"
            except Exception as e:
                last_err = f"vision1 affine 로드 실패({path}): {e}"
                continue
        return False, last_err

    def _get_vision1_affine_3x4(self):
        with self._vision1_affine_lock:
            if self._vision1_affine_3x4 is None:
                return None
            return [list(row[:3]) for row in self._vision1_affine_3x4[:4]]

    def get_vision1_meta_snapshot(self):
        with self._vision1_meta_lock:
            payload = self._vision1_meta_payload
            seen_at = self._vision1_meta_seen_at
        if payload is None:
            return None, seen_at
        return dict(payload), seen_at

    def get_vision2_meta_snapshot(self):
        with self._vision2_meta_lock:
            payload = self._vision2_meta_payload
            seen_at = self._vision2_meta_seen_at
        if payload is None:
            return None, seen_at
        return dict(payload), seen_at

    def get_current_volume_ml(self, *, max_age_sec: float | None = None):
        payload, seen_at = self.get_vision2_meta_snapshot()
        if payload is None or seen_at is None:
            return None, "vision2 메타데이터 없음"
        try:
            age = max(0.0, time.monotonic() - float(seen_at))
        except Exception:
            age = float("inf")
        if max_age_sec is None:
            stale_limit = float(VISION2_META_STALE_SEC)
        else:
            try:
                stale_limit = max(0.0, float(max_age_sec))
            except Exception:
                stale_limit = float(VISION2_META_STALE_SEC)
        if age > stale_limit:
            return None, f"vision2 메타데이터 지연: {age * 1000.0:.0f}ms"
        volume_ml = self._extract_vision2_volume_ml(payload)
        if volume_ml is None:
            return None, "vision2 volume_ml 파싱 실패"
        return float(volume_ml), "ok"

    def _extract_vision2_volume_ml(self, payload):
        if not isinstance(payload, dict):
            return None
        candidate = payload.get("volume_ml", None)
        if candidate is None:
            liquid = payload.get("liquid", None)
            if isinstance(liquid, dict):
                candidate = liquid.get("volume_ml", None)
        try:
            value = float(candidate)
        except Exception:
            return None
        if not math.isfinite(value):
            return None
        return float(value)

    def _get_vision1_intrinsics(self):
        with self._vision1_intrinsics_lock:
            if self._vision1_intrinsics is None:
                return None
            return tuple(float(v) for v in self._vision1_intrinsics[:4])

    def _uvz_to_camera_xyz_mm_vision1(self, u: float, v: float, z_mm: float, require_intrinsics: bool = True):
        intr = self._get_vision1_intrinsics()
        if intr is None:
            if require_intrinsics:
                return None
            return float(u), float(v), float(z_mm)
        fx, fy, cx, cy = intr
        z = float(z_mm)
        x = (float(u) - float(cx)) * z / float(fx)
        y = (float(v) - float(cy)) * z / float(fy)
        return float(x), float(y), float(z)

    def _vision1_uvz_to_robot_xyz_mm(self, u: float, v: float, z_mm: float):
        affine = self._get_vision1_affine_3x4()
        if affine is None:
            return None, "vision1 affine가 준비되지 않았습니다."
        cam_xyz = self._uvz_to_camera_xyz_mm_vision1(u, v, z_mm, require_intrinsics=True)
        if cam_xyz is None:
            return None, "vision1 카메라 intrinsics가 없어 UVZ 변환을 할 수 없습니다."
        cx, cy, cz = cam_xyz
        try:
            out_x = float(cx) * float(affine[0][0]) + float(cy) * float(affine[1][0]) + float(cz) * float(affine[2][0]) + float(affine[3][0])
            out_y = float(cx) * float(affine[0][1]) + float(cy) * float(affine[1][1]) + float(cz) * float(affine[2][1]) + float(affine[3][1])
            out_z = float(cx) * float(affine[0][2]) + float(cy) * float(affine[1][2]) + float(cz) * float(affine[2][2]) + float(affine[3][2])
        except Exception as e:
            return None, f"vision1 affine 곱셈 실패: {e}"
        return (out_x, out_y, out_z), "ok"

    def _normalize_robot_action_ingredient_code(self, raw_code: str):
        code = str(raw_code or "").strip().lower()
        if not code:
            return ""
        if code in ROBOT_ACTION_INGREDIENT_CLASS_ALIASES:
            return code
        for canonical, aliases in ROBOT_ACTION_INGREDIENT_CLASS_ALIASES.items():
            for alias in tuple(aliases or ()):
                alias_text = str(alias or "").strip().lower()
                if not alias_text:
                    continue
                if code == alias_text or alias_text in code or code in alias_text:
                    return str(canonical)
        return code

    def _match_robot_action_ingredient_class(self, class_name: str, ingredient_code: str):
        name = str(class_name or "").strip().lower()
        code = self._normalize_robot_action_ingredient_code(ingredient_code)
        aliases = ROBOT_ACTION_INGREDIENT_CLASS_ALIASES.get(code, (code,))
        if not name:
            return False
        for alias in tuple(aliases or ()):
            alias_text = str(alias or "").strip().lower()
            if not alias_text:
                continue
            if name == alias_text or alias_text in name or name in alias_text:
                return True
        return False

    def _parse_robot_action_detection(self, det):
        if not isinstance(det, dict):
            return None

        class_name = str(det.get("class_name", "") or "").strip().lower()
        center_uv = det.get("center_uv")
        if not isinstance(center_uv, (list, tuple)) or len(center_uv) < 2:
            bbox = det.get("bbox_xyxy")
            if isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
                try:
                    x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
                    center_uv = [(x1 + x2) * 0.5, (y1 + y2) * 0.5]
                except Exception:
                    center_uv = None
        if not isinstance(center_uv, (list, tuple)) or len(center_uv) < 2:
            return None

        try:
            depth_m = float(det.get("depth_m"))
        except Exception:
            return None
        if (not math.isfinite(depth_m)) or depth_m <= 0.0:
            return None

        try:
            confidence = float(det.get("confidence"))
            if not math.isfinite(confidence):
                confidence = 0.0
        except Exception:
            confidence = 0.0

        try:
            u = float(center_uv[0])
            v = float(center_uv[1])
        except Exception:
            return None
        if (not math.isfinite(u)) or (not math.isfinite(v)):
            return None

        return {
            "class_name": class_name,
            "center_uv": [u, v],
            "depth_m": float(depth_m),
            "confidence": float(confidence),
        }

    def _pick_robot_action_detection_for_ingredient(self, detections, ingredient_code: str):
        code = self._normalize_robot_action_ingredient_code(ingredient_code)
        best = None
        best_score = -1.0
        for det in detections if isinstance(detections, list) else []:
            parsed = self._parse_robot_action_detection(det)
            if parsed is None:
                continue
            if not self._match_robot_action_ingredient_class(parsed.get("class_name", ""), code):
                continue
            score = float(parsed.get("confidence", 0.0))
            if score >= best_score:
                best = dict(parsed)
                best_score = score
        return best

    def _wait_fresh_vision1_meta_snapshot(self, timeout_sec: float | None = None):
        wait_sec = float(VISION1_META_WAIT_SEC if timeout_sec is None else timeout_sec)
        wait_sec = max(0.2, wait_sec)
        deadline = time.monotonic() + wait_sec
        last_age_ms = None
        while time.monotonic() <= deadline:
            payload, seen_at = self.get_vision1_meta_snapshot()
            if payload is not None and seen_at is not None:
                age_sec = time.monotonic() - float(seen_at)
                last_age_ms = max(0.0, age_sec * 1000.0)
                if age_sec <= float(VISION1_META_STALE_SEC):
                    return payload, seen_at, "ok"
            time.sleep(0.05)
        if last_age_ms is None:
            return None, None, f"vision1 메타데이터가 없습니다. 토픽 확인: {VISION1_META_TOPIC}"
        return None, None, f"vision1 메타데이터 지연: {last_age_ms:.0f}ms"

    def _ensure_vision1_intrinsics_ready(self, timeout_sec: float = 1.0):
        wait_sec = max(0.2, float(timeout_sec))
        deadline = time.monotonic() + wait_sec
        while time.monotonic() <= deadline:
            if self._get_vision1_intrinsics() is not None:
                return True, "ok"
            time.sleep(0.05)
        topic = self._vision1_camera_info_topic_in_use or VISION1_CAMERA_INFO_TOPIC_PRIMARY
        return False, f"vision1 CameraInfo 미수신: {topic}"

    def get_robot_action_vision_target_xyz(
        self,
        ingredient_code: str,
        *,
        menu_offsets=None,
        apply_menu_offset: bool = True,
    ):
        if not self._ready_event.is_set():
            return False, {}, "백엔드 준비 중입니다."

        code = self._normalize_robot_action_ingredient_code(ingredient_code)
        if code not in ROBOT_ACTION_INGREDIENT_CLASS_ALIASES:
            supported = ", ".join(sorted(ROBOT_ACTION_INGREDIENT_CLASS_ALIASES.keys()))
            return False, {}, f"지원하지 않는 재료 코드입니다: {ingredient_code} (지원: {supported})"

        ok_src, msg_src = self._setup_bartender_action_sources(progress_callback=None)
        if not ok_src:
            return False, {}, str(msg_src or "비전 소스 준비 실패")

        ok_aff, msg_aff = self._load_vision1_affine_from_file(force=False)
        if not ok_aff:
            return False, {}, str(msg_aff or "vision1 affine 로드 실패")

        ok_intr, msg_intr = self._ensure_vision1_intrinsics_ready(timeout_sec=1.0)
        if not ok_intr:
            return False, {}, str(msg_intr)

        meta_payload, _meta_seen_at, msg_meta = self._wait_fresh_vision1_meta_snapshot(timeout_sec=VISION1_META_WAIT_SEC)
        if meta_payload is None:
            return False, {}, str(msg_meta)

        detections = meta_payload.get("detections", []) if isinstance(meta_payload, dict) else []
        picked = self._pick_robot_action_detection_for_ingredient(detections, code)
        if picked is None:
            available = []
            for raw_det in detections if isinstance(detections, list) else []:
                parsed = self._parse_robot_action_detection(raw_det)
                if parsed is None:
                    continue
                name = str(parsed.get("class_name", "") or "").strip().lower()
                if name:
                    available.append(name)
            if available:
                available_text = ", ".join(sorted(set(available)))
            else:
                available_text = "-"
            return False, {}, f"vision1에서 재료({code})를 찾지 못했습니다. 감지 클래스: {available_text}"

        try:
            u = float(picked["center_uv"][0])
            v = float(picked["center_uv"][1])
            depth_m = float(picked["depth_m"])
            conf = float(picked.get("confidence", 0.0))
        except Exception:
            return False, {}, "선택된 비전 타겟의 형식이 올바르지 않습니다."

        z_mm = depth_m * 1000.0
        robot_xyz, xyz_msg = self._vision1_uvz_to_robot_xyz_mm(u, v, z_mm)
        if robot_xyz is None:
            return False, {}, str(xyz_msg or "UVZ->로봇XYZ 변환 실패")

        base_x, base_y, base_z = [float(vv) for vv in robot_xyz[:3]]
        tx, ty, tz = base_x, base_y, base_z
        menu_ox, menu_oy, menu_oz = 0.0, 0.0, 0.0
        if bool(apply_menu_offset):
            offset_map = self._load_menu_xyz_offsets(menu_offsets=menu_offsets)
            menu_ox, menu_oy, menu_oz = self._get_menu_xyz_offset(code, offset_map)
            tx += float(menu_ox)
            ty += float(menu_oy)
            tz += float(menu_oz)

        payload = {
            "ingredient_code": str(code),
            "class_name": str(picked.get("class_name", "") or ""),
            "confidence": float(conf),
            "target_uv": [float(u), float(v)],
            "target_depth_m": float(depth_m),
            "base_robot_xyz_mm": [float(base_x), float(base_y), float(base_z)],
            "menu_offset_xyz_mm": [float(menu_ox), float(menu_oy), float(menu_oz)],
            "resolved_robot_xyz_mm": [float(tx), float(ty), float(tz)],
            "apply_menu_offset": bool(apply_menu_offset),
            "affine_path": str(self._vision1_affine_path or ""),
            "camera_info_topic": str(self._vision1_camera_info_topic_in_use or VISION1_CAMERA_INFO_TOPIC_PRIMARY),
            "vision_meta_topic": str(VISION1_META_TOPIC),
        }
        msg = (
            f"비전 기준좌표 계산 완료({code}): XYZ=({tx:.1f},{ty:.1f},{tz:.1f}), "
            f"base=({base_x:.1f},{base_y:.1f},{base_z:.1f}), "
            f"menu_offset=({float(menu_ox):.1f},{float(menu_oy):.1f},{float(menu_oz):.1f}), "
            f"UV=({u:.1f},{v:.1f}), depth={z_mm:.1f}mm, conf={float(conf):.2f}"
        )
        return True, payload, msg

    def _parse_menu_offset_map(self, raw_map):
        parsed = {}
        if not isinstance(raw_map, dict):
            return parsed
        source = raw_map.get("menus", raw_map) if isinstance(raw_map.get("menus", None), dict) else raw_map
        for key, payload in source.items():
            code = str(key or "").strip().lower()
            if not code:
                continue
            xyz = None
            if isinstance(payload, dict):
                xyz = payload.get("offset_xyz_mm", None)
            elif isinstance(payload, (list, tuple)):
                xyz = payload
            if not isinstance(xyz, (list, tuple)) or len(xyz) < 3:
                continue
            try:
                parsed[code] = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
            except Exception:
                continue
        return parsed

    def _load_menu_xyz_offsets(self, menu_offsets=None):
        parsed_override = self._parse_menu_offset_map(menu_offsets)
        if parsed_override:
            return parsed_override
        try:
            if os.path.isfile(MENU_OFFSET_CONFIG_PATH):
                with open(MENU_OFFSET_CONFIG_PATH, "r", encoding="utf-8") as f:
                    decoded = json.load(f)
                parsed = self._parse_menu_offset_map(decoded)
                if parsed:
                    return parsed
        except Exception:
            pass
        return {}

    def _parse_menu_gripper_close_map(self, raw_map):
        parsed = {}
        if not isinstance(raw_map, dict):
            return parsed
        source = raw_map.get("menus", raw_map) if isinstance(raw_map.get("menus", None), dict) else raw_map
        for key, payload in source.items():
            code = str(key or "").strip().lower()
            if not code or (not isinstance(payload, dict)):
                continue
            try:
                value = float(payload.get("gripper_close_mm"))
            except Exception:
                continue
            if (not math.isfinite(value)) or value < 0.0:
                continue
            parsed[code] = float(value)
        return parsed

    def _load_menu_gripper_close_mm(self, menu_offsets=None):
        parsed_override = self._parse_menu_gripper_close_map(menu_offsets)
        if parsed_override:
            return parsed_override
        try:
            if os.path.isfile(MENU_OFFSET_CONFIG_PATH):
                with open(MENU_OFFSET_CONFIG_PATH, "r", encoding="utf-8") as f:
                    decoded = json.load(f)
                parsed = self._parse_menu_gripper_close_map(decoded)
                if parsed:
                    return parsed
        except Exception:
            pass
        return {}

    def _get_menu_xyz_offset(self, menu_code: str, offset_map: dict):
        code = str(menu_code or "").strip().lower()
        xyz = offset_map.get(code, (0.0, 0.0, 0.0))
        return float(xyz[0]), float(xyz[1]), float(xyz[2])

    def _execute_bartender_robot_action_script(self, context: dict):
        script_path = str(os.environ.get("BARTENDER_ROBOT_ACTION_SCRIPT", BARTENDER_ROBOT_ACTION_SCRIPT_PATH) or "").strip()
        if not script_path:
            script_path = BARTENDER_ROBOT_ACTION_SCRIPT_PATH
        if not os.path.isabs(script_path):
            script_path = self._resolve_project_path(script_path)
        if not os.path.isfile(script_path):
            return False, f"로봇동작 플래너 파일 없음: {script_path}", {}
        try:
            proc = subprocess.run(
                [sys.executable, script_path],
                input=json.dumps(dict(context or {}), ensure_ascii=False),
                text=True,
                capture_output=True,
                timeout=float(BARTENDER_ROBOT_ACTION_SCRIPT_TIMEOUT_SEC),
                check=False,
            )
        except Exception as e:
            return False, f"로봇동작 플래너 실행 실패: {e}", {}

        stdout_text = str(proc.stdout or "").strip()
        stderr_text = str(proc.stderr or "").strip()
        if not stdout_text:
            msg = stderr_text or f"플래너 출력 없음(returncode={proc.returncode})"
            return False, msg, {}

        out = None
        for line in reversed(stdout_text.splitlines()):
            raw = str(line or "").strip()
            if not raw:
                continue
            try:
                maybe = json.loads(raw)
            except Exception:
                continue
            if isinstance(maybe, dict):
                out = maybe
                break
        if out is None:
            return False, f"플래너 JSON 파싱 실패: {stdout_text[-240:]}", {}

        ok = bool(out.get("ok", False))
        msg = str(out.get("message") or "").strip()
        if (not ok) and stderr_text:
            msg = f"{msg} | stderr: {stderr_text}" if msg else stderr_text
        if not msg:
            msg = "완료" if ok else "실패"
        return ok, msg, out

    def _execute_bartender_robot_action_native(self, context: dict, offset_map, execute_enabled: bool, move_speed):
        if run_bartender_robot_action is None:
            return False, "로봇동작 플래너 import 실패", {}
        api = NativeRobotActionApi(
            backend=self,
            offset_map=offset_map,
            execute_enabled=bool(execute_enabled),
            move_speed=move_speed,
        )
        try:
            out = run_bartender_robot_action(dict(context or {}), api=api)
        except Exception as e:
            return False, f"로봇동작 네이티브 시퀀스 예외: {e}", {}

        if not isinstance(out, dict):
            return False, "로봇동작 플래너 결과 형식 오류", {}
        ok = bool(out.get("ok", False))
        msg = str(out.get("message") or "").strip()
        if not msg:
            msg = "완료" if ok else "실패"
        if not ok:
            return False, msg, out

        speed_txt = "-" if move_speed is None else f"{move_speed:.0f}%"
        mode_txt = "실행" if execute_enabled else "계획"
        summary = api.summary_tail()
        return True, f"{msg} | {mode_txt}시퀀스 완료({speed_txt}) | {summary}", out

    def _call_with_retry(self, fn, timeout_sec: float = 8.0, poll_sec: float = 0.2):
        deadline = time.monotonic() + max(0.5, float(timeout_sec))
        last_msg = "실행 실패"
        while time.monotonic() <= deadline:
            ok, msg = fn()
            text = str(msg or "").strip()
            if ok:
                return True, text or "완료"
            last_msg = text or last_msg
            if ("현재 작업 중" in last_msg) or ("백엔드 준비 중" in last_msg):
                time.sleep(max(0.05, float(poll_sec)))
                continue
            return False, last_msg
        return False, f"타임아웃: {last_msg}"

    def _parse_robot_action_abc(self):
        raw = str(os.environ.get("BARTENDER_ROBOT_ACTION_ABC", "180,0,180") or "").strip()
        parts = [p.strip() for p in raw.split(",")]
        if len(parts) != 3:
            return 180.0, 0.0, 180.0
        try:
            return float(parts[0]), float(parts[1]), float(parts[2])
        except Exception:
            return 180.0, 0.0, 180.0

    def _resolve_robot_action_abc(self):
        # 1) 실시간 current_posx(기본좌표계)에서 현재 TCP 자세 ABC 우선 사용
        posx_live, sol_live, err_live = self.get_current_posx_live()
        if err_live is None and posx_live is not None and len(posx_live) >= 6:
            return (
                float(posx_live[3]),
                float(posx_live[4]),
                float(posx_live[5]),
            ), (f"get_current_posx(sol={int(sol_live)})" if sol_live is not None else "get_current_posx")

        # 2) RobotState/JointState 캐시 posx에서 ABC 사용
        with self._position_lock:
            posx_cached = None if self._last_positions is None else self._last_positions[1]
        if isinstance(posx_cached, (list, tuple)) and len(posx_cached) >= 6:
            return (
                float(posx_cached[3]),
                float(posx_cached[4]),
                float(posx_cached[5]),
            ), "state_cache"

        # 3) 최종 폴백: ENV(BARTENDER_ROBOT_ACTION_ABC)
        return self._parse_robot_action_abc(), "env(BARTENDER_ROBOT_ACTION_ABC)"

    def _execute_planner_sequence_steps(self, sequence_steps, offset_map, execute_enabled: bool, move_speed):
        if not isinstance(sequence_steps, list) or not sequence_steps:
            return False, "시퀀스 step 목록이 비어 있습니다."

        def _emit_robot_action_log(message: str, *, is_error: bool = False):
            text = str(message or "").strip()
            if not text:
                return
            try:
                if self.robot_controller is not None:
                    if is_error:
                        self.robot_controller._log_error(f"[로봇동작] {text}")
                    else:
                        self.robot_controller._log_info(f"[로봇동작] {text}")
                else:
                    print(f"[로봇동작]{'[오류]' if is_error else ''} {text}")
            except Exception:
                pass

        allowed_backend_calls = {
            "set_robot_mode",
            "send_move_home",
            "send_move_cartesian",
            "send_move_cartesian_async",
            "send_move_joint",
            "send_move_joint_async",
            "send_gripper_move",
            "send_motion_stop",
            "send_emergency_stop",
        }
        step_logs = []
        resolved_targets = {}

        for idx, raw_step in enumerate(sequence_steps, start=1):
            if not isinstance(raw_step, dict):
                return False, f"step[{idx}] 형식 오류(dict 필요)"
            step = dict(raw_step)
            op = str(step.get("op", "") or "").strip().lower()
            label = str(step.get("label", f"step{idx}") or f"step{idx}").strip()
            enabled = bool(step.get("enabled", True))

            if not op:
                return False, f"step[{idx}] op 누락"

            if op == "log_event":
                level = str(step.get("level", "info") or "info").strip().lower()
                step_logs.append(f"{idx}:{label}")
                _emit_robot_action_log(step_logs[-1], is_error=level in ("error", "warn", "warning"))
                continue

            if op == "placeholder":
                note = str(step.get("note", "") or "").strip()
                step_logs.append(f"{idx}:{label}(placeholder{': ' + note if note else ''})")
                _emit_robot_action_log(step_logs[-1])
                continue

            if not enabled:
                step_logs.append(f"{idx}:{label}(비활성)")
                _emit_robot_action_log(step_logs[-1])
                continue

            if op == "set_robot_mode":
                mode = int(step.get("mode", 1))
                if execute_enabled:
                    ok_mode, mode_msg = self._call_with_retry(
                        lambda: self.set_robot_mode(mode, timeout_sec=6.0),
                        timeout_sec=8.0,
                        poll_sec=0.2,
                    )
                    if not ok_mode:
                        return False, f"{idx}:{label} 실패: {mode_msg}"
                step_logs.append(f"{idx}:{label}(mode={mode}{'' if execute_enabled else ', 계획'})")
                _emit_robot_action_log(step_logs[-1])
                continue

            if op == "resolve_detection_target":
                target_key = str(step.get("target_key", "") or "").strip()
                target_uv = step.get("target_uv")
                target_depth_m = step.get("target_depth_m")
                ingredient_code = str(step.get("ingredient_code", "") or "").strip().lower()
                if not target_key:
                    return False, f"{idx}:{label} 실패: target_key 누락"
                if not isinstance(target_uv, (list, tuple)) or len(target_uv) < 2:
                    return False, f"{idx}:{label} 실패: target_uv 형식 오류"
                try:
                    u = float(target_uv[0])
                    v = float(target_uv[1])
                    z_mm = float(target_depth_m) * 1000.0
                except Exception:
                    return False, f"{idx}:{label} 실패: target_uv/target_depth_m 파싱 실패"
                if (not math.isfinite(z_mm)) or z_mm <= 0.0:
                    return False, f"{idx}:{label} 실패: depth 값 오류({target_depth_m})"

                robot_xyz, xyz_msg = self._vision1_uvz_to_robot_xyz_mm(u, v, z_mm)
                if robot_xyz is None:
                    return False, f"{idx}:{label} 실패: {xyz_msg}"
                tx, ty, tz = [float(vv) for vv in robot_xyz[:3]]
                base_x, base_y, base_z = tx, ty, tz
                menu_ox, menu_oy, menu_oz = 0.0, 0.0, 0.0

                if bool(step.get("apply_menu_offset", True)):
                    menu_ox, menu_oy, menu_oz = self._get_menu_xyz_offset(ingredient_code, offset_map)
                    tx += float(menu_ox)
                    ty += float(menu_oy)
                    tz += float(menu_oz)

                extra_offset = step.get("extra_offset_xyz_mm", [0.0, 0.0, 0.0])
                try:
                    ex = float(extra_offset[0]) if isinstance(extra_offset, (list, tuple)) and len(extra_offset) >= 1 else 0.0
                    ey = float(extra_offset[1]) if isinstance(extra_offset, (list, tuple)) and len(extra_offset) >= 2 else 0.0
                    ez = float(extra_offset[2]) if isinstance(extra_offset, (list, tuple)) and len(extra_offset) >= 3 else 0.0
                except Exception:
                    ex, ey, ez = 0.0, 0.0, 0.0
                tx += ex
                ty += ey
                tz += ez

                resolved_targets[target_key] = (tx, ty, tz)
                step_logs.append(f"{idx}:{label}(key={target_key}, XYZ={tx:.1f},{ty:.1f},{tz:.1f})")
                _emit_robot_action_log(
                    f"{step_logs[-1]}, base=({base_x:.1f},{base_y:.1f},{base_z:.1f}), "
                    f"menu_offset=({float(menu_ox):.1f},{float(menu_oy):.1f},{float(menu_oz):.1f}), "
                    f"extra_offset=({ex:.1f},{ey:.1f},{ez:.1f}), "
                    f"UV=({u:.1f},{v:.1f}), depth={z_mm:.1f}mm, ingredient={ingredient_code or '-'}"
                )
                continue

            if op == "movel_resolved_target":
                target_key = str(step.get("target_key", "") or "").strip()
                if not target_key:
                    return False, f"{idx}:{label} 실패: target_key 누락"
                xyz = resolved_targets.get(target_key)
                if xyz is None:
                    return False, f"{idx}:{label} 실패: target_key={target_key} 미해결"
                tx, ty, tz = [float(vv) for vv in xyz[:3]]
                xyzabc = step.get("xyzabc", None)
                if isinstance(xyzabc, (list, tuple)) and len(xyzabc) >= 3:
                    try:
                        tx += float(xyzabc[0])
                        ty += float(xyzabc[1])
                        tz += float(xyzabc[2])
                    except Exception:
                        pass
                offset = step.get("offset_xyz_mm", [0.0, 0.0, 0.0])
                try:
                    ox = float(offset[0]) if isinstance(offset, (list, tuple)) and len(offset) >= 1 else 0.0
                    oy = float(offset[1]) if isinstance(offset, (list, tuple)) and len(offset) >= 2 else 0.0
                    oz = float(offset[2]) if isinstance(offset, (list, tuple)) and len(offset) >= 3 else 0.0
                except Exception:
                    ox, oy, oz = 0.0, 0.0, 0.0
                tx += ox
                ty += oy
                tz += oz

                abc = step.get("abc", None)
                if isinstance(xyzabc, (list, tuple)) and len(xyzabc) >= 6:
                    try:
                        a, b, c = float(xyzabc[3]), float(xyzabc[4]), float(xyzabc[5])
                    except Exception:
                        (a, b, c), _abc_source = self._resolve_robot_action_abc()
                elif isinstance(abc, (list, tuple)) and len(abc) >= 3:
                    try:
                        a, b, c = float(abc[0]), float(abc[1]), float(abc[2])
                    except Exception:
                        (a, b, c), _abc_source = self._resolve_robot_action_abc()
                else:
                    (a, b, c), _abc_source = self._resolve_robot_action_abc()

                try:
                    approach_up_mm = max(0.0, float(step.get("approach_up_mm", 0.0)))
                except Exception:
                    approach_up_mm = 0.0
                step_vel = step.get("vel", None)
                step_acc = step.get("acc", None)
                try:
                    eff_vel = move_speed if step_vel is None else float(step_vel)
                except Exception:
                    eff_vel = move_speed
                try:
                    eff_acc = move_speed if step_acc is None else float(step_acc)
                except Exception:
                    eff_acc = move_speed

                if execute_enabled:
                    if approach_up_mm > 0.0:
                        ok_ap, msg_ap = self._call_with_retry(
                            lambda: self.send_move_cartesian(tx, ty, tz + approach_up_mm, a, b, c, eff_vel, eff_acc),
                            timeout_sec=14.0,
                            poll_sec=0.2,
                        )
                        if not ok_ap:
                            return False, f"{idx}:{label} 접근이동 실패: {msg_ap}"
                    ok_tg, msg_tg = self._call_with_retry(
                        lambda: self.send_move_cartesian(tx, ty, tz, a, b, c, eff_vel, eff_acc),
                        timeout_sec=14.0,
                        poll_sec=0.2,
                    )
                    if not ok_tg:
                        return False, f"{idx}:{label} 타겟이동 실패: {msg_tg}"
                step_logs.append(f"{idx}:{label}(key={target_key}, XYZ={tx:.1f},{ty:.1f},{tz:.1f})")
                _emit_robot_action_log(
                    f"{step_logs[-1]}, ABC={a:.1f},{b:.1f},{c:.1f}, approach_up={approach_up_mm:.1f}mm"
                )
                continue

            if op == "move_to_detection":
                target_uv = step.get("target_uv")
                target_depth_m = step.get("target_depth_m")
                ingredient_code = str(step.get("ingredient_code", "") or "").strip().lower()
                if not isinstance(target_uv, (list, tuple)) or len(target_uv) < 2:
                    return False, f"{idx}:{label} 실패: target_uv 형식 오류"
                try:
                    u = float(target_uv[0])
                    v = float(target_uv[1])
                    z_mm = float(target_depth_m) * 1000.0
                except Exception:
                    return False, f"{idx}:{label} 실패: target_uv/target_depth_m 파싱 실패"
                if (not math.isfinite(z_mm)) or z_mm <= 0.0:
                    return False, f"{idx}:{label} 실패: depth 값 오류({target_depth_m})"

                robot_xyz, xyz_msg = self._vision1_uvz_to_robot_xyz_mm(u, v, z_mm)
                if robot_xyz is None:
                    return False, f"{idx}:{label} 실패: {xyz_msg}"
                tx, ty, tz = [float(vv) for vv in robot_xyz[:3]]
                base_x, base_y, base_z = tx, ty, tz
                menu_ox, menu_oy, menu_oz = 0.0, 0.0, 0.0

                if bool(step.get("apply_menu_offset", True)):
                    menu_ox, menu_oy, menu_oz = self._get_menu_xyz_offset(ingredient_code, offset_map)
                    tx += float(menu_ox)
                    ty += float(menu_oy)
                    tz += float(menu_oz)

                extra_offset = step.get("extra_offset_xyz_mm", [0.0, 0.0, 0.0])
                try:
                    ex = float(extra_offset[0]) if isinstance(extra_offset, (list, tuple)) and len(extra_offset) >= 1 else 0.0
                    ey = float(extra_offset[1]) if isinstance(extra_offset, (list, tuple)) and len(extra_offset) >= 2 else 0.0
                    ez = float(extra_offset[2]) if isinstance(extra_offset, (list, tuple)) and len(extra_offset) >= 3 else 0.0
                except Exception:
                    ex, ey, ez = 0.0, 0.0, 0.0
                tx += ex
                ty += ey
                tz += ez

                abc = step.get("abc", None)
                if isinstance(abc, (list, tuple)) and len(abc) >= 3:
                    try:
                        a, b, c = float(abc[0]), float(abc[1]), float(abc[2])
                    except Exception:
                        (a, b, c), _abc_source = self._resolve_robot_action_abc()
                else:
                    (a, b, c), _abc_source = self._resolve_robot_action_abc()

                try:
                    approach_up_mm = max(0.0, float(step.get("approach_up_mm", 40.0)))
                except Exception:
                    approach_up_mm = 40.0

                if execute_enabled:
                    if approach_up_mm > 0.0:
                        ok_ap, msg_ap = self._call_with_retry(
                            lambda: self.send_move_cartesian(tx, ty, tz + approach_up_mm, a, b, c, move_speed, move_speed),
                            timeout_sec=14.0,
                            poll_sec=0.2,
                        )
                        if not ok_ap:
                            return False, f"{idx}:{label} 접근이동 실패: {msg_ap}"
                    ok_tg, msg_tg = self._call_with_retry(
                        lambda: self.send_move_cartesian(tx, ty, tz, a, b, c, move_speed, move_speed),
                        timeout_sec=14.0,
                        poll_sec=0.2,
                    )
                    if not ok_tg:
                        return False, f"{idx}:{label} 타겟이동 실패: {msg_tg}"
                step_logs.append(f"{idx}:{label}(XYZ={tx:.1f},{ty:.1f},{tz:.1f})")
                _emit_robot_action_log(
                    f"{step_logs[-1]}, base=({base_x:.1f},{base_y:.1f},{base_z:.1f}), "
                    f"menu_offset=({float(menu_ox):.1f},{float(menu_oy):.1f},{float(menu_oz):.1f}), "
                    f"extra_offset=({ex:.1f},{ey:.1f},{ez:.1f}), "
                    f"UV=({u:.1f},{v:.1f}), depth={z_mm:.1f}mm, ABC={a:.1f},{b:.1f},{c:.1f}, "
                    f"ingredient={ingredient_code or '-'}"
                )
                continue

            if op == "wait_volume_target":
                try:
                    target_volume_ml = float(step.get("target_volume_ml"))
                except Exception:
                    return False, f"{idx}:{label} 실패: target_volume_ml 누락/형식오류"
                if target_volume_ml < 0.0:
                    return False, f"{idx}:{label} 실패: target_volume_ml 음수"
                timeout_sec = 20.0
                poll_sec = 0.1
                try:
                    timeout_sec = max(0.5, float(step.get("timeout_sec", 20.0)))
                except Exception:
                    timeout_sec = 20.0
                try:
                    poll_sec = max(0.05, float(step.get("poll_sec", 0.1)))
                except Exception:
                    poll_sec = 0.1

                if not execute_enabled:
                    step_logs.append(f"{idx}:{label}(계획 target={target_volume_ml:.1f}ml)")
                    continue

                start_at = time.monotonic()
                last_volume = None
                while (time.monotonic() - start_at) <= timeout_sec:
                    payload, seen_at = self.get_vision2_meta_snapshot()
                    if payload is None or seen_at is None:
                        time.sleep(poll_sec)
                        continue
                    age = time.monotonic() - float(seen_at)
                    if age > float(VISION2_META_STALE_SEC):
                        time.sleep(poll_sec)
                        continue
                    volume_ml = self._extract_vision2_volume_ml(payload)
                    if volume_ml is None:
                        time.sleep(poll_sec)
                        continue
                    last_volume = float(volume_ml)
                    if float(volume_ml) >= float(target_volume_ml):
                        break
                    time.sleep(poll_sec)
                else:
                    if last_volume is None:
                        return False, f"{idx}:{label} 실패: vision2 용량값을 읽지 못했습니다."
                    return False, f"{idx}:{label} 실패: 목표용량 미도달({last_volume:.1f}/{target_volume_ml:.1f}ml)"

                step_logs.append(f"{idx}:{label}(volume={last_volume:.1f}/{target_volume_ml:.1f}ml)")
                _emit_robot_action_log(step_logs[-1])
                continue

            if op == "wait_sec":
                try:
                    wait_s = max(0.0, float(step.get("seconds", 0.0)))
                except Exception:
                    return False, f"{idx}:{label} 실패: seconds 형식오류"

                if not execute_enabled:
                    step_logs.append(f"{idx}:{label}(계획 {wait_s:.2f}s)")
                    continue

                end_t = time.monotonic() + float(wait_s)
                while time.monotonic() < end_t:
                    ok_state, state_msg = self._check_motion_precondition(max_state_age_sec=2.0)
                    if not ok_state:
                        return False, f"{idx}:{label} 실패: {state_msg}"
                    time.sleep(min(0.05, max(0.0, end_t - time.monotonic())))
                step_logs.append(f"{idx}:{label}({wait_s:.2f}s)")
                _emit_robot_action_log(step_logs[-1])
                continue

            if op == "backend_call":
                method = str(step.get("method", "") or "").strip()
                if not method:
                    return False, f"{idx}:{label} 실패: method 누락"
                if method not in allowed_backend_calls:
                    return False, f"{idx}:{label} 실패: 미허용 backend_call method={method}"
                fn = getattr(self, method, None)
                if not callable(fn):
                    return False, f"{idx}:{label} 실패: backend method 없음({method})"

                args = step.get("args", [])
                kwargs = step.get("kwargs", {})
                if isinstance(args, tuple):
                    args = list(args)
                if not isinstance(args, list):
                    args = [args]
                if not isinstance(kwargs, dict):
                    kwargs = {}
                kwargs = dict(kwargs)
                if method in ("send_move_home", "send_move_cartesian", "send_move_cartesian_async", "send_move_joint", "send_move_joint_async"):
                    if move_speed is not None:
                        kwargs.setdefault("vel", move_speed)
                        kwargs.setdefault("acc", move_speed)

                if execute_enabled:
                    try:
                        timeout_sec = float(step.get("timeout_sec", 14.0))
                    except Exception:
                        timeout_sec = 14.0
                    ok_call, call_msg = self._call_with_retry(
                        lambda: fn(*args, **kwargs),
                        timeout_sec=max(1.0, timeout_sec),
                        poll_sec=0.2,
                    )
                    if not ok_call:
                        return False, f"{idx}:{label} 실패: {call_msg}"
                step_logs.append(f"{idx}:{label}({method})")
                if method in ("send_move_cartesian", "send_move_cartesian_async") and len(args) >= 6:
                    try:
                        _emit_robot_action_log(
                            f"{step_logs[-1]} XYZ={float(args[0]):.1f},{float(args[1]):.1f},{float(args[2]):.1f} "
                            f"ABC={float(args[3]):.1f},{float(args[4]):.1f},{float(args[5]):.1f}"
                        )
                    except Exception:
                        _emit_robot_action_log(step_logs[-1])
                elif method in ("send_move_joint", "send_move_joint_async") and len(args) >= 6:
                    try:
                        _emit_robot_action_log(
                            f"{step_logs[-1]} J={','.join(f'{float(v):.1f}' for v in list(args)[:6])}"
                        )
                    except Exception:
                        _emit_robot_action_log(step_logs[-1])
                else:
                    _emit_robot_action_log(step_logs[-1])
                continue

            return False, f"step[{idx}] 미지원 op: {op}"

        summary = " / ".join(step_logs[-6:]) if step_logs else "시퀀스 실행 완료"
        return True, summary

    def run_bartender_first_ingredient_action(
        self,
        order_result: dict,
        menu_offsets=None,
        motion_speed_percent=None,
        execute_enabled_override=None,
    ):
        if not self._ready_event.is_set():
            return False, "백엔드 준비 중입니다."
        if not isinstance(order_result, dict):
            return False, "order_result 형식이 올바르지 않습니다."

        status = str(order_result.get("status", "") or "").strip().lower()
        if status != "success":
            return False, f"주문 상태가 success가 아닙니다: {status or '-'}"

        ok_src, msg_src = self._setup_bartender_action_sources(progress_callback=None)
        if (not ok_src) and self.robot_controller is not None:
            self.robot_controller._log_error(msg_src)

        ok_aff, msg_aff = self._load_vision1_affine_from_file(force=False)
        if not ok_aff:
            return False, msg_aff
        if self.robot_controller is not None and str(msg_aff or "").strip():
            try:
                self.robot_controller._log_info(str(msg_aff))
            except Exception:
                pass

        meta_payload = None
        meta_seen_at = None
        meta_deadline = time.monotonic() + float(VISION1_META_WAIT_SEC)
        while time.monotonic() <= meta_deadline:
            meta_payload, meta_seen_at = self.get_vision1_meta_snapshot()
            if meta_payload is not None and meta_seen_at is not None:
                age = time.monotonic() - float(meta_seen_at)
                if age <= float(VISION1_META_STALE_SEC):
                    break
            time.sleep(0.05)
        if meta_payload is None or meta_seen_at is None:
            return False, f"vision1 메타데이터가 없습니다. 토픽 확인: {VISION1_META_TOPIC}"
        meta_age = time.monotonic() - float(meta_seen_at)
        if meta_age > float(VISION1_META_STALE_SEC):
            return False, f"vision1 메타데이터 지연: {meta_age * 1000.0:.0f}ms"

        intr_deadline = time.monotonic() + 1.0
        while time.monotonic() <= intr_deadline:
            if self._get_vision1_intrinsics() is not None:
                break
            time.sleep(0.05)
        if self._get_vision1_intrinsics() is None:
            topic = self._vision1_camera_info_topic_in_use or VISION1_CAMERA_INFO_TOPIC_PRIMARY
            return False, f"vision1 CameraInfo 미수신: {topic}"

        vision2_payload = None
        vision2_seen_at = None
        vision2_deadline = time.monotonic() + float(VISION2_META_WAIT_SEC)
        while time.monotonic() <= vision2_deadline:
            vision2_payload, vision2_seen_at = self.get_vision2_meta_snapshot()
            if vision2_payload is not None and vision2_seen_at is not None:
                age = time.monotonic() - float(vision2_seen_at)
                if age <= float(VISION2_META_STALE_SEC):
                    break
            time.sleep(0.05)
        if vision2_payload is None or vision2_seen_at is None:
            return False, f"vision2 메타데이터가 없습니다. 토픽 확인: {VISION2_META_TOPIC}"
        vision2_age = time.monotonic() - float(vision2_seen_at)
        if vision2_age > float(VISION2_META_STALE_SEC):
            return False, f"vision2 메타데이터 지연: {vision2_age * 1000.0:.0f}ms"

        context = {
            "order_result": dict(order_result),
            "vision1_meta": dict(meta_payload),
            "vision2_meta": dict(vision2_payload),
            "requested_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        }
        if execute_enabled_override is None:
            execute_enabled = str(os.environ.get("BARTENDER_ROBOT_ACTION_EXECUTE", "0")).strip().lower() in (
                "1",
                "true",
                "yes",
                "on",
            )
        else:
            execute_enabled = bool(execute_enabled_override)
        offset_map = self._load_menu_xyz_offsets(menu_offsets=menu_offsets)
        gripper_close_map = self._load_menu_gripper_close_mm(menu_offsets=menu_offsets)
        merged_menu_offsets = {"menus": {}}
        for code in sorted(set(list(offset_map.keys()) + list(gripper_close_map.keys()))):
            ox, oy, oz = self._get_menu_xyz_offset(code, offset_map)
            merged_menu_offsets["menus"][str(code)] = {
                "offset_xyz_mm": [float(ox), float(oy), float(oz)],
                "gripper_close_mm": float(gripper_close_map.get(code, 41.0)),
            }
        context["menu_offsets"] = merged_menu_offsets
        if self.robot_controller is not None:
            try:
                self.robot_controller._log_info(
                    f"[로봇동작] 적용 메뉴 오프셋: {json.dumps(offset_map, ensure_ascii=False)}"
                )
                self.robot_controller._log_info(
                    f"[로봇동작] 적용 메뉴 파지(mm): {json.dumps(gripper_close_map, ensure_ascii=False)}"
                )
            except Exception:
                pass
        move_speed = None
        try:
            if motion_speed_percent is not None:
                speed_v = float(motion_speed_percent)
                if math.isfinite(speed_v):
                    speed_v = max(0.0, min(100.0, speed_v))
                    move_speed = 1.0 if speed_v <= 0.0 else float(speed_v)
        except Exception:
            move_speed = None

        if BARTENDER_ROBOT_ACTION_NATIVE:
            if self._busy_event.is_set():
                return False, "현재 작업 중입니다."
            self._busy_event.set()
            try:
                ok_native, native_msg, _native_out = self._execute_bartender_robot_action_native(
                    context=context,
                    offset_map=offset_map,
                    execute_enabled=bool(execute_enabled),
                    move_speed=move_speed,
                )
                if not ok_native:
                    return False, f"로봇동작 네이티브 실패: {native_msg}"
                return True, str(native_msg or "완료")
            finally:
                self._busy_event.clear()

        ok_plan, plan_msg, plan_out = self._execute_bartender_robot_action_script(context)
        if not ok_plan:
            return False, f"로봇동작 플래너 실패: {plan_msg}"

        plan = plan_out.get("plan", {}) if isinstance(plan_out, dict) else {}
        sequence_steps = plan.get("sequence_steps")
        if isinstance(sequence_steps, list) and sequence_steps:
            ok_seq, seq_msg = self._execute_planner_sequence_steps(
                sequence_steps,
                offset_map=offset_map,
                execute_enabled=bool(execute_enabled),
                move_speed=move_speed,
            )
            if not ok_seq:
                return False, f"{plan_msg} | {seq_msg}"
            speed_txt = "-" if move_speed is None else f"{move_speed:.0f}%"
            mode_txt = "실행" if execute_enabled else "계획"
            return True, f"{plan_msg} | {mode_txt}시퀀스 완료({speed_txt}) | {seq_msg}"

        # backward compatibility: 구형 플래너 포맷(first_ingredient_code/target_uv/depth)
        first_code = str(plan.get("first_ingredient_code") or "").strip().lower()
        target_uv = plan.get("target_uv")
        target_depth_m = plan.get("target_depth_m")
        if not first_code:
            return False, "플래너 결과에 first_ingredient_code 또는 sequence_steps가 없습니다."
        if not isinstance(target_uv, (list, tuple)) or len(target_uv) < 2:
            return False, "플래너 결과에 target_uv가 없습니다."
        try:
            u = float(target_uv[0])
            v = float(target_uv[1])
            z_mm = float(target_depth_m) * 1000.0
        except Exception:
            return False, "플래너 결과의 uv/depth 형식이 올바르지 않습니다."
        if (not math.isfinite(z_mm)) or z_mm <= 0.0:
            return False, f"플래너 depth가 유효하지 않습니다: {target_depth_m}"

        robot_xyz, xyz_msg = self._vision1_uvz_to_robot_xyz_mm(u, v, z_mm)
        if robot_xyz is None:
            return False, xyz_msg
        tx, ty, tz = [float(vv) for vv in robot_xyz[:3]]
        ox, oy, oz = self._get_menu_xyz_offset(first_code, offset_map)
        tx += float(ox)
        ty += float(oy)
        tz += float(oz)
        if not execute_enabled:
            speed_txt = "-" if move_speed is None else f"{move_speed:.0f}%"
            return True, f"{plan_msg} | 실행OFF(계획): {first_code} 목표 XYZ=({tx:.1f}, {ty:.1f}, {tz:.1f}), 속도={speed_txt}"
        ok_mode, mode_msg = self._call_with_retry(
            lambda: self.set_robot_mode(1, timeout_sec=6.0),
            timeout_sec=8.0,
            poll_sec=0.2,
        )
        if not ok_mode:
            return False, f"오토모드 전환 실패: {mode_msg}"
        (a, b, c), _abc_source = self._resolve_robot_action_abc()
        ok_move, move_msg = self._call_with_retry(
            lambda: self.send_move_cartesian(tx, ty, tz, a, b, c, move_speed, move_speed),
            timeout_sec=14.0,
            poll_sec=0.2,
        )
        if not ok_move:
            return False, f"첫 재료 타겟 이동 실패: {move_msg}"
        speed_txt = "-" if move_speed is None else f"{move_speed:.0f}%"
        return True, f"{plan_msg} | 구형포맷 이동 완료({first_code}) XYZ=({tx:.1f}, {ty:.1f}, {tz:.1f}), 속도={speed_txt}"

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
        return True, f"비전 좌표 이동 시작: X={tx:.2f}, Y={ty:.2f}, Z={tz:.2f} (대기 {td:.1f}s)"

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

    def send_move_cartesian_async(self, x, y, z, a, b, c, vel=None, acc=None):
        if not self._ready_event.is_set():
            return False, "백엔드 준비 중입니다."
        if self._cmd_q.full():
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
        self._cmd_q.put(("move_cartesian_async", (tx, ty, tz, ta, tb, tc, tv, ta2)))
        speed_text = f", 속도={tv:.0f}%" if tv is not None else ""
        return True, (
            f"카테시안 비동기 이동 시작: X={tx:.2f}, Y={ty:.2f}, Z={tz:.2f}, "
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

    def send_move_joint_async(self, j1, j2, j3, j4, j5, j6, vel=None, acc=None):
        if not self._ready_event.is_set():
            return False, "백엔드 준비 중입니다."
        if self._cmd_q.full():
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
        self._cmd_q.put(("move_joint_async", (*vals, tv, ta2)))
        speed_text = f", 속도={tv:.0f}%" if tv is not None else ""
        return True, (
            f"조인트 비동기 이동 시작: J1={vals[0]:.2f}, J2={vals[1]:.2f}, J3={vals[2]:.2f}, "
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

    def send_emergency_stop(self):
        if not self._ready_event.is_set():
            return False, "백엔드 준비 중입니다."
        # ServoOff.srv: STOP_TYPE_EMERGENCY == 3
        ok, msg = self._call_servo_off(stop_type=3, timeout_sec=2.0)
        if not ok:
            return False, msg
        return True, "긴급정지 명령을 전송했습니다."

    def send_motion_stop(self, stop_mode: int = 2):
        if not self._ready_event.is_set():
            return False, "백엔드 준비 중입니다."
        # MoveStop.srv: stop_mode=2(DR_SSTO, Soft Stop)
        mode = int(stop_mode)
        op_timeout = 0.35
        ok_stop, msg_stop = self._call_move_stop(stop_mode=mode, timeout_sec=op_timeout)
        stop_result_text = f"STOP={'OK' if ok_stop else 'FAIL'} ({msg_stop})"

        # 모드변경의 GET 재확인처럼, stop 이후 상태가 MOVING(2)에서 벗어났는지 확인한다.
        verify_deadline = time.monotonic() + 2.0
        verify_ok = False
        verify_msg = "상태 미확인"
        while time.monotonic() < verify_deadline:
            state_code, state_name, seen_at = self.get_robot_state_snapshot()
            if state_code is None or seen_at is None:
                verify_msg = "로봇 상태 미수신"
                time.sleep(0.05)
                continue
            age = time.monotonic() - float(seen_at)
            if age > 1.2:
                verify_msg = f"로봇 상태 지연({age:.2f}s)"
                time.sleep(0.05)
                continue
            code = int(state_code)
            name = str(state_name or get_robot_state_name(code))
            if code != 2:  # STATE_MOVING
                verify_ok = True
                verify_msg = f"mode/state 확인: {name}({code})"
                break
            verify_msg = f"여전히 MOVING({code})"
            time.sleep(0.05)

        merged = f"{stop_result_text}; VERIFY={'OK' if verify_ok else 'FAIL'} ({verify_msg})"
        if verify_ok:
            if self.robot_controller is not None:
                self.robot_controller._log_info(f"[정지] {merged}")
            return True, merged

        # 서비스 응답이 실패/타임아웃이어도 실제 상태가 멈춘 것으로 확인되면 성공 처리한다.
        # (위 verify_ok에서 이미 처리됨) 여기까지 왔으면 둘 다 실패.
        if self.robot_controller is not None:
            self.robot_controller._log_error(f"[정지] {merged}")
        return False, merged

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

        if self.robot_controller is not None:
            # 종료 정리(워커 join/노드 파괴) 전에 먼저 그리퍼 종료를 시도해
            # drl_start 서비스가 살아있는 동안 close 요청이 나가도록 한다.
            try:
                if rclpy.ok():
                    self.robot_controller.terminate_gripper(best_effort=True)
                    for _ in range(20):
                        rclpy.spin_once(self.robot_controller, timeout_sec=0.01)
                        rclpy.spin_once(self._dsr_node, timeout_sec=0.01)
                        if self._mode_node is not None:
                            rclpy.spin_once(self._mode_node, timeout_sec=0.01)
                else:
                    print("ROS 컨텍스트가 이미 종료되어 그리퍼 terminate를 생략합니다.")
            except Exception as e:
                print(f"그리퍼 terminate 중 예외(종료 과정에서 흔함): {e}")

        # 그리퍼 종료 시도 후에 stop_event를 세팅해야,
        # background executor spin이 service 응답을 끝까지 처리할 수 있다.
        self._stop_event.set()

        try:
            self._bartender_sequence_manager.shutdown()
        except Exception:
            pass
        try:
            self._stop_bartender_sequence_api_server()
        except Exception:
            pass
        try:
            self._cmd_q.put_nowait(("quit", None))
        except queue.Full:
            pass

        if self._worker_thread is not None:
            self._worker_thread.join(timeout=2.0)
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
        if self._vision_spin_thread is not None:
            self._vision_spin_thread.join(timeout=1.0)

        try:
            if self._vision_executor is not None:
                self._vision_executor.shutdown(timeout_sec=0.5)
        except Exception:
            pass

        try:
            self._stop_bartender_action_sources()
        except Exception:
            pass

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
            if self._vision_bridge_node is not None:
                self._vision_bridge_node.destroy_node()
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
