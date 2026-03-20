import os
import sys
import glob
import json
import csv
import html
import signal
import subprocess
import threading
import time
import traceback
import warnings
import io
import wave
import re
from urllib import error as urllib_error
from urllib import request as urllib_request
from datetime import datetime

from PyQt5.QtCore import QTimer, QObject, pyqtSignal, pyqtSlot, Qt, QThread, QEvent, QUrl
from PyQt5.QtGui import (
    QFont,
    QImage,
    QPixmap,
    QBrush,
    QColor,
    QDoubleValidator,
    QPalette,
    QTransform,
    QDesktopServices,
)
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QInputDialog,
    QDialog,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QDialogButtonBox,
    QButtonGroup,
    QLabel,
    QProgressBar,
    QMessageBox,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QFrame,
    QPushButton,
    QLineEdit,
    QCheckBox,
    QSlider,
    QFileDialog,
    QSizePolicy,
    QPlainTextEdit,
    QTabWidget,
    QWidget,
)
from PyQt5 import uic

try:
    from rcl_interfaces.msg import Log as RosLogMsg
except Exception:
    RosLogMsg = None
try:
    from sensor_msgs.msg import Image as RosImageMsg
except Exception:
    RosImageMsg = None
try:
    from sensor_msgs.msg import CameraInfo as RosCameraInfoMsg
except Exception:
    RosCameraInfoMsg = None
try:
    from geometry_msgs.msg import PointStamped as RosPointStamped
except Exception:
    RosPointStamped = None
try:
    from std_msgs.msg import String as RosStringMsg
except Exception:
    RosStringMsg = None
try:
    from rclpy.qos import (
        qos_profile_sensor_data,
        QoSProfile,
        QoSHistoryPolicy,
        QoSReliabilityPolicy,
    )
except Exception:
    qos_profile_sensor_data = None
    QoSProfile = None
    QoSHistoryPolicy = None
    QoSReliabilityPolicy = None
try:
    from rclpy.callback_groups import ReentrantCallbackGroup
except Exception:
    ReentrantCallbackGroup = None
try:
    from rclpy.exceptions import RCLError
except Exception:
    RCLError = Exception
import numpy as np
try:
    import sounddevice as sd
except Exception:
    sd = None

# Project layout root
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "..", ".."))
BACKEND_DIR = os.path.abspath(os.path.join(CURRENT_DIR, "..", "backend"))
SRC_DIR = os.path.abspath(os.path.join(CURRENT_DIR, ".."))
if BACKEND_DIR not in sys.path:
    sys.path.insert(0, BACKEND_DIR)
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

# ROS2 logs to stdout so UI terminal can capture
os.environ.setdefault("RCUTILS_LOGGING_USE_STDOUT", "1")
os.environ.setdefault("RCUTILS_LOGGING_BUFFERED_STREAM", "1")
os.environ.setdefault("RCUTILS_LOGGING_SEVERITY", "WARN")

warnings.filterwarnings(
    "ignore",
    message=r"A NumPy version >=1\.17\.3 and <1\.25\.0 is required for this version of SciPy.*",
    category=UserWarning,
)

from task_backend_node import RobotBackend, ROBOT_ID, HOME_POSJ
from order_integration.openai_tts import synthesize_openai_tts

form = uic.loadUiType(os.path.join(PROJECT_ROOT, "assets", "frontend", "developer_frontend.ui"))[0]
UI_FONT_FAMILY = "Noto Sans CJK KR"
UI_FONT_SIZE = 9
UI_TERMINAL_FONT_SIZE = max(6, int(os.environ.get("UI_TERMINAL_FONT_SIZE", "9")))
UI_TABLE_FONT_SIZE = max(12, int(os.environ.get("UI_TABLE_FONT_SIZE", "12")))
UI_TABLE_ROW_HEIGHT = max(26, int(os.environ.get("UI_TABLE_ROW_HEIGHT", "30")))
UI_PANEL_TABLE_ROW_HEIGHT = max(22, int(UI_TABLE_ROW_HEIGHT * 0.875))
STARTUP_USE_REAL_GRIPPER = os.environ.get("STARTUP_USE_REAL_GRIPPER", "1") == "1"
MOTION_SPEED_MIN_PERCENT = 0
MOTION_SPEED_MAX_PERCENT = 100
DEFAULT_MOTION_SPEED_PERCENT = max(
    MOTION_SPEED_MIN_PERCENT,
    min(
        MOTION_SPEED_MAX_PERCENT,
        int(float(os.environ.get("UI_MOTION_SPEED_PERCENT", "50"))),
    ),
)
CALIBRATION_FIXED_SPEED_PERCENT = 20.0

UI_ENABLE_VISION = os.environ.get("UI_ENABLE_VISION", "1") == "1"
YOLO_EXTERNAL_NODE = os.environ.get("YOLO_EXTERNAL_NODE", "1") == "1"
CALIB_VISION_TOPIC_PRIMARY = os.environ.get("CALIB_VISION_TOPIC", "/camera/camera_1/color/image_raw")
CALIB_CAMERA_INFO_TOPIC_PRIMARY = os.environ.get("CALIB_CAMERA_INFO_TOPIC", "/camera/camera_1/color/camera_info")
CALIB_OUTPUT_META_TOPIC_1 = os.environ.get("CALIB_OUTPUT_META_TOPIC_1", "/camera/camera_1/calibration/meta")
CALIB_OUTPUT_META_TOPIC_2 = os.environ.get("CALIB_OUTPUT_META_TOPIC_2", "/camera/camera_2/calibration/meta")
VISION_OBJECT_META_TOPIC_1 = os.environ.get("VISION_OBJECT_META_TOPIC_1", "/camera/camera_1/detection/object/meta")
VISION_VOLUME_META_TOPIC_2 = os.environ.get("VISION_VOLUME_META_TOPIC_2", "/camera/camera_2/detection/volume/meta")
YOLO_AUTO_LAUNCH_NODE = os.environ.get("YOLO_AUTO_LAUNCH_NODE", "1") == "1"
YOLO_AUTO_LAUNCH_ALWAYS = os.environ.get("YOLO_AUTO_LAUNCH_ALWAYS", "0") == "1"
YOLO_AUTO_LAUNCH_CMD = os.environ.get("YOLO_AUTO_LAUNCH_CMD", "").strip()
CALIB_HELPER_AUTO_LAUNCH = os.environ.get("CALIB_HELPER_AUTO_LAUNCH", "0") == "1"
CALIB_HELPER_CMD = os.environ.get("CALIB_HELPER_CMD", "").strip()
FRONTEND_UI_PERF_LOG_PATH = os.path.abspath(
    os.environ.get("FRONTEND_UI_PERF_LOG_PATH", os.path.join(PROJECT_ROOT, "logs", "frontend_ui_perf.log"))
)

POSITION_STALE_SEC = float(os.environ.get("UI_POSITION_STALE_SEC", "2.0"))
UI_TICK_WARN_MS = max(20.0, float(os.environ.get("UI_TICK_WARN_MS", "120.0")))
UI_TICK_WARN_COOLDOWN_SEC = max(0.5, float(os.environ.get("UI_TICK_WARN_COOLDOWN_SEC", "1.0")))
UI_TICK_TRACE_ENABLED = os.environ.get("UI_TICK_TRACE", "0") == "1"
UI_TICK_TRACE_MIN_MS = max(0.0, float(os.environ.get("UI_TICK_TRACE_MIN_MS", "0.0")))
VISION_DECODE_MIN_INTERVAL_MS = max(
    0.0, float(os.environ.get("VISION_DECODE_MIN_INTERVAL_MS", "16.0"))
)
VISION_DECODE_MIN_INTERVAL_SEC = float(VISION_DECODE_MIN_INTERVAL_MS) / 1000.0
STATE_FLASH_SEC = float(os.environ.get("UI_STATE_FLASH_SEC", "1.2"))
POSITION_FLASH_SEC = float(os.environ.get("UI_POSITION_FLASH_SEC", "0.9"))
ROBOT_STARTUP_CONNECT_GRACE_SEC = float(os.environ.get("ROBOT_STARTUP_CONNECT_GRACE_SEC", "20.0"))
POSITION_VALUE_COLOR = "#000000"
STATE_COLOR_NORMAL = "#14863B"
STATE_COLOR_WARNING = "#EF6C00"
STATE_COLOR_ERROR = "#C62828"
TOP_STATUS_BAR_HEIGHT = 40
TOP_STATUS_GAP = 10
VISION_MOVE_DEFAULT_OFFSET_X_MM = 0.0
VISION_MOVE_DEFAULT_OFFSET_Y_MM = 0.0
VISION_MOVE_DEFAULT_OFFSET_Z_MM = 0.0
CALIB_SEQUENCE_VELOCITY = float(os.environ.get("CALIB_SEQUENCE_VELOCITY", "20"))
CALIB_SEQUENCE_ACC = float(os.environ.get("CALIB_SEQUENCE_ACC", "20"))
CALIB_SEQUENCE_SETTLE_SEC = float(os.environ.get("CALIB_SEQUENCE_SETTLE_SEC", "3.0"))
CALIB_SEQUENCE_NEXT_DELAY_SEC = float(os.environ.get("CALIB_SEQUENCE_NEXT_DELAY_SEC", "2.0"))
CALIB_SEQUENCE_CAPTURE_TIMEOUT_SEC = float(os.environ.get("CALIB_SEQUENCE_CAPTURE_TIMEOUT_SEC", "8.0"))
JOINT_INPUT_LIMITS_DEG = (
    (-360.0, 360.0),  # J1
    (-360.0, 360.0),  # J2
    (-155.0, 155.0),  # J3
    (-360.0, 360.0),  # J4
    (-360.0, 360.0),  # J5
    (-360.0, 360.0),  # J6
)
LOG_AREA_SHIFT_Y = 4
UI_USE_DESIGN_GEOMETRY = os.environ.get("UI_USE_DESIGN_GEOMETRY", "0") == "1"
MODE_SWITCH_GRACE_SEC = float(os.environ.get("MODE_SWITCH_GRACE_SEC", "4.0"))
VOICE_ORDER_PANEL_GAP = 10
DASHBOARD_MARGIN = 12
DASHBOARD_COL_GAP = 10
DASHBOARD_ROW_GAP = 10
DASHBOARD_BOTTOM_MIN_HEIGHT = 220
DASHBOARD_BOTTOM_EXTRA_MARGIN = 0
DASHBOARD_COL_WIDTHS = (531, 531, 840)
DASHBOARD_TOP_PANEL_HEIGHT = 600
DASHBOARD_BOTTOM_PANEL_HEIGHT = 600
DASHBOARD_LOG_HEIGHT_RATIO = 0.25
DASHBOARD_LOG_MIN_HEIGHT = 240
BARTENDER_SEQUENCE_STEPS = [
    ("boot", "시스템 준비"),
    ("mode", "모드 확인"),
    ("voice_request", "음성 입력 요청"),
    ("stt", "STT 인식"),
    ("llm", "LLM 주문 판별"),
    ("recipe", "레시피 생성"),
    ("robot_action", "로봇 동작"),
    ("vision_check", "비전 확인"),
    ("done", "완료"),
]
BARTENDER_SEQUENCE_STEP_LABEL_BY_KEY = {str(key): str(label) for key, label in BARTENDER_SEQUENCE_STEPS}
BARTENDER_SEQUENCE_STEP_MAX_HEIGHT = 36
VOICE_MIC_BUTTON_TEXT_START = "마이크 입력 시작"
VOICE_MIC_BUTTON_TEXT_STOP = "마이크 입력 중지"
VOICE_ORDER_CUSTOMER_ERROR_TTS_TEXT = "요청 처리에 실패했습니다. 다시 시도해 주세요."
VOICE_ORDER_CYCLE_INTERVAL_MS = max(
    1, int(float(os.environ.get("VOICE_ORDER_CYCLE_INTERVAL_MS", "100")))
)
VOICE_ORDER_DELAY_WARN_MS = max(
    300.0, float(os.environ.get("VOICE_ORDER_DELAY_WARN_MS", str(max(600, VOICE_ORDER_CYCLE_INTERVAL_MS * 4))))
)
VOICE_ORDER_DISCONNECT_MS = max(
    VOICE_ORDER_DELAY_WARN_MS + 200.0,
    float(os.environ.get("VOICE_ORDER_DISCONNECT_MS", str(max(1600, VOICE_ORDER_CYCLE_INTERVAL_MS * 12)))),
)
VOICE_ORDER_WEBUI_HOST = (
    str(
        os.environ.get(
            "USER_FRONTEND_HOST",
            os.environ.get("VOICE_ORDER_WEBUI_HOST", "127.0.0.1"),
        )
        or ""
    ).strip()
    or "127.0.0.1"
)
VOICE_ORDER_WEBUI_PORT = (
    str(
        os.environ.get(
            "USER_FRONTEND_PORT",
            os.environ.get("VOICE_ORDER_WEBUI_PORT", "8000"),
        )
        or ""
    ).strip()
    or "8000"
)
WEBUI_ORDER_START_SYNC_MIN_INTERVAL_SEC = max(
    0.2, float(os.environ.get("WEBUI_ORDER_START_SYNC_MIN_INTERVAL_SEC", "0.6"))
)
WEBUI_ORDER_START_HTTP_TIMEOUT_SEC = max(
    0.2, float(os.environ.get("WEBUI_ORDER_START_HTTP_TIMEOUT_SEC", "0.4"))
)
PARAM_DIR = os.path.join(PROJECT_ROOT, "config")
PARAM_FILE = os.path.join(PARAM_DIR, "parameter.csv")
MENU_OFFSET_CONFIG_PATH = os.path.join(PARAM_DIR, "menu_xyz_offsets.json")
ROBOT_ACTION_POSE_CONFIG_PATH = os.path.join(PARAM_DIR, "robot_action_pose_config.json")
BARTENDER_MENU_LABELS = {
    "soju": "소주",
    "beer": "맥주",
    "glass": "글라스잔",
    "somaek": "소맥",
}
BARTENDER_ROBOT_ACTION_TEST_MENU_ORDER = ("soju", "beer", "somaek")
BARTENDER_ROBOT_ACTION_TEST_RECIPES = {
    "soju": {"soju": 50.0},
    "beer": {"beer": 200.0},
    "somaek": {"soju": 60.0, "beer": 140.0},
}
BARTENDER_MENU_OFFSET_EXCLUDED_CODES = {"somaek"}
BARTENDER_MENU_OFFSET_EXCLUDED_CODES_NORM = {str(v).strip().lower() for v in BARTENDER_MENU_OFFSET_EXCLUDED_CODES}
BARTENDER_INGREDIENT_ALIASES = {
    "soju": {"soju", "소주"},
    "beer": {"beer", "맥주"},
}
ROBOT_ACTION_LIQUID_INGREDIENT_CODES = ("soju", "beer")
ROBOT_ACTION_POSE_TAB_LABELS = {
    "soju": "소주",
    "beer": "맥주",
    "glass": "글라스잔",
}
# 중요:
# 이 기본값은 "UI 초기 fallback"이다.
# 실제 사용 시에는 저장 파일(config/robot_action_pose_config.json) 값이 우선 로드된다.
ROBOT_ACTION_POSE_DEFAULTS = {
    "poses": {
        "service_ready_posj": [28.0, -35.0, 100.0, 77.0, 63.0, -154.0],
        "soju_service_ready_posj": [28.0, -35.0, 100.0, 77.0, 63.0, -154.0],
        "beer_service_ready_posj": [28.0, -35.0, 100.0, 77.0, 63.0, -154.0],
        "pour_start_cheers_posj": [45.0, 0.0, 135.0, 90.0, -90.0, -135.0],
        "pour_contact_posj": [45.0, 43.58, 134.19, 90.01, -90.0, -62.23],
        "pour_horizontal_posj": [42.43, 21.08, 129.85, 87.75, -88.75, -29.06],
        "pour_diagonal_posj": [41.83, -5.0, 134.35, 87.99, -87.55, -0.61],
        "pour_vertical_posj": [38.76, -35.8, 146.74, 87.76, -84.18, 22.06],
        "soju_pour_start_cheers_posj": [45.0, 0.0, 135.0, 90.0, -90.0, -135.0],
        "soju_pour_contact_posj": [40.53, 40.36, 134.88, 85.56, -89.63, -64.75],
        "soju_pour_horizontal_posj": [39.63, 18.89, 132.0, 85.3, -87.39, -29.02],
        "soju_pour_diagonal_posj": [38.12, -7.68, 137.65, 85.57, -84.73, 0.16],
        "soju_pour_vertical_posj": [36.16, -33.75, 145.95, 86.64, -81.83, 23.45],
        "beer_pour_start_cheers_posj": [45.0, 0.0, 135.0, 90.0, -90.0, -135.0],
        "beer_pour_contact_posj": [40.29, 26.46, 139.23, 85.45, -88.84, -74.27],
        "beer_pour_horizontal_posj": [37.99, 7.03, 143.01, 83.92, -86.51, -29.79],
        "beer_pour_diagonal_posj": [36.62, -6.09, 147.33, 83.45, -84.76, 11.53],
        "beer_pour_vertical_posj": [36.16, -28.4, 150.63, 85.26, -82.54, 33.54],
        "cup_pick_ready_posj": [28.0, -35.0, 100.0, 77.0, 63.0, -154.0],
        "cup_pick_approach_posx": [430.0, -110.0, 300.0, 180.0, 0.0, 180.0],
        "cup_pick_pose_posx": [430.0, -110.0, 225.0, 180.0, 0.0, 180.0],
        "cup_pick_lift_posx": [430.0, -110.0, 330.0, 180.0, 0.0, 180.0],
        "pick_lift_out_posx": [99.555, 533.322, 701.878, 88.252, 89.415, -91.978],
        "soju_pick_lift_out_posx": [99.555, 533.322, 701.878, 88.252, 89.415, -91.978],
        "beer_pick_lift_out_posx": [99.555, 533.322, 701.878, 88.252, 89.415, -91.978],
        "cup_delivery_ready_posj": [28.0, -35.0, 100.0, 77.0, 63.0, -154.0],
        "cup_delivery_approach_posx": [520.0, -20.0, 320.0, 180.0, 0.0, 180.0],
        "cup_delivery_pose_posx": [520.0, -20.0, 235.0, 180.0, 0.0, 180.0],
    },
    "offsets_xyz_mm": {
        "pick_approach_offset": [0.0, -50.0, 0.0],
        "pick_grasp_offset": [0.0, 0.0, 0.0],
        "pick_lift_offset": [0.0, 0.0, 100.0],
        "place_offset": [0.0, 0.0, 0.0],
        "retreat_offset": [-20.0, -50.0, 0.0],
        "soju_pick_approach_offset": [0.0, -50.0, 0.0],
        "soju_pick_grasp_offset": [0.0, 0.0, 0.0],
        "soju_pick_lift_offset": [0.0, 0.0, 100.0],
        "soju_place_offset": [0.0, 0.0, 0.0],
        "soju_retreat_offset": [-20.0, -50.0, 0.0],
        "beer_pick_approach_offset": [0.0, -50.0, 0.0],
        "beer_pick_grasp_offset": [0.0, 0.0, 0.0],
        "beer_pick_lift_offset": [0.0, 0.0, 100.0],
        "beer_place_offset": [0.0, 0.0, 0.0],
        "beer_retreat_offset": [-20.0, -50.0, 0.0],
    },
}
ROBOT_ACTION_POSE_LEGACY_POSE_KEY_MAP = {
    "joint_pick_ready": "service_ready_posj",
    "joint_pick_place_ready": "service_ready_posj",
    "joint_service_ready": "service_ready_posj",
    "joint_cheers": "pour_start_cheers_posj",
    "joint_contact": "pour_contact_posj",
    "joint_pour_horizontal": "pour_horizontal_posj",
    "joint_pour_diagonal": "pour_diagonal_posj",
    "joint_pour_vertical": "pour_vertical_posj",
    "cup_pick_fallback_approach": "cup_pick_approach_posx",
    "cup_pick_approach": "cup_pick_approach_posx",
    "cup_pick_fallback_pose": "cup_pick_pose_posx",
    "cup_pick_pose": "cup_pick_pose_posx",
    "cup_pick_fallback_lift": "cup_pick_lift_posx",
    "cup_pick_lift": "cup_pick_lift_posx",
    "ingredient_pick_lift_out": "pick_lift_out_posx",
    "cup_delivery_approach": "cup_delivery_approach_posx",
    "cup_delivery_pose": "cup_delivery_pose_posx",
}
ROBOT_ACTION_POSE_LEGACY_OFFSET_KEY_MAP = {
    "ingredient_pick_approach": "pick_approach_offset",
    "ingredient_pick_grasp": "pick_grasp_offset",
    "ingredient_pick_lift": "pick_lift_offset",
    "ingredient_place": "place_offset",
    "ingredient_retreat": "retreat_offset",
}
ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT = 109.0


def _canonical_robot_action_pose_key(raw_key):
    key = str(raw_key or "").strip()
    if not key:
        return ""
    return str(ROBOT_ACTION_POSE_LEGACY_POSE_KEY_MAP.get(key, key))


def _canonical_robot_action_offset_key(raw_key):
    key = str(raw_key or "").strip()
    if not key:
        return ""
    return str(ROBOT_ACTION_POSE_LEGACY_OFFSET_KEY_MAP.get(key, key))


def _build_liquid_robot_action_pose_rows(ingredient_code: str, ingredient_label: str):
    code = str(ingredient_code or "").strip().lower()
    label = str(ingredient_label or code)
    prefix_upper = str(code).upper()
    return [
        {
            "tab": code,
            "section": "[0] 재료보정",
            "type": "menu_offset_xyz",
            "ingredient_code": code,
            "var_name": f"{code}_menu_offset_xyz_mm",
            "label": f"{label} 비전 XYZ 오프셋",
            "desc": "비전 기준좌표에 더하는 재료별 XYZ 오프셋(mm)",
        },
        {
            "tab": code,
            "section": "[0] 재료보정",
            "type": "menu_gripper_close_mm",
            "ingredient_code": code,
            "var_name": f"{code}_gripper_close_mm",
            "label": f"{label} 파지 거리(mm)",
            "desc": "재료 파지 시 사용할 그리퍼 닫힘 거리(mm)",
        },
        {
            "tab": code,
            "section": "[0] 재료보정",
            "type": "menu_gripper_open_mm",
            "ingredient_code": code,
            "var_name": f"{code}_gripper_open_mm",
            "label": f"{label} 오픈 거리(mm)",
            "desc": "재료 파지 전 사용할 그리퍼 오픈 거리(mm)",
        },
        {
            "tab": code,
            "section": "[1] PICK/준비",
            "type": "posj",
            "config_group": "poses",
            "config_key": f"{code}_service_ready_posj",
            "var_name": f"{prefix_upper}_SERVICE_READY_POSJ",
            "label": f"{label} 병 집기 준비(service_ready, 기준값)",
            "desc": "기준 위치(수정/티칭 가능)",
        },
        {
            "tab": code,
            "section": "[1] PICK/준비",
            "type": "vision_target",
            "ingredient_code": code,
            "var_name": f"resolve_detection_target({code})",
            "label": f"{label} 비전 타겟 계산",
            "desc": "vision1 중심좌표/깊이에서 실시간 계산(보기 전용)",
        },
        {
            "tab": code,
            "section": "[1] PICK/준비",
            "type": "gripper_open_action",
            "ingredient_code": code,
            "var_name": f"{code}_gripper_open_before_pick",
            "label": "병 파지 전 그리퍼 오픈",
            "desc": "위치참조",
            "editable": False,
            "reference_only": True,
        },
        {
            "tab": code,
            "section": "[1] PICK",
            "type": "vision_offset",
            "ingredient_code": code,
            "config_group": "offsets_xyz_mm",
            "config_key": f"{code}_pick_approach_offset",
            "var_name": f"{code}_pick_approach_offset",
            "label": "병 접근(target_1) 오프셋",
            "desc": "비전 타겟 기준 상대 오프셋(mm)",
        },
        {
            "tab": code,
            "section": "[1] PICK",
            "type": "vision_offset",
            "ingredient_code": code,
            "config_group": "offsets_xyz_mm",
            "config_key": f"{code}_pick_grasp_offset",
            "var_name": f"{code}_pick_grasp_offset",
            "label": "병 파지(target_2) 오프셋",
            "desc": "비전 타겟 기준 상대 오프셋(mm)",
        },
        {
            "tab": code,
            "section": "[1] PICK",
            "type": "gripper_close_action",
            "ingredient_code": code,
            "var_name": f"{code}_gripper_close_pick",
            "label": "병 파지(그리퍼 닫힘)",
            "desc": "위치참조",
            "editable": False,
            "reference_only": True,
        },
        {
            "tab": code,
            "section": "[1] PICK",
            "type": "vision_offset",
            "ingredient_code": code,
            "config_group": "offsets_xyz_mm",
            "config_key": f"{code}_pick_lift_offset",
            "var_name": f"{code}_pick_lift_offset",
            "label": "병 파지 후 업(target_3) 오프셋",
            "desc": "비전 타겟 기준 상대 오프셋(mm)",
        },
        {
            "tab": code,
            "section": "[1] PICK",
            "type": "posx",
            "config_group": "poses",
            "config_key": f"{code}_pick_lift_out_posx",
            "var_name": f"{code}_pick_lift_out_posx",
            "label": "병 파지 후 업 이탈(target_4) 절대좌표",
            "desc": "절대 posx(mm/deg), 비전 오프셋 미사용",
        },
        {
            "tab": code,
            "section": "[2] POUR",
            "type": "posj",
            "config_group": "poses",
            "config_key": f"{code}_pour_start_cheers_posj",
            "var_name": f"{prefix_upper}_POUR_START_CHEERS_POSJ",
            "label": "따르기 시작 자세",
            "desc": "POUR 시작 전 조인트 자세",
        },
        {
            "tab": code,
            "section": "[2] POUR",
            "type": "posj",
            "config_group": "poses",
            "config_key": f"{code}_pour_contact_posj",
            "var_name": f"{prefix_upper}_POUR_CONTACT_POSJ",
            "label": "따르기 contact",
            "desc": "실시간 용량 피드백 단계 1",
        },
        {
            "tab": code,
            "section": "[2] POUR",
            "type": "posj",
            "config_group": "poses",
            "config_key": f"{code}_pour_horizontal_posj",
            "var_name": f"{prefix_upper}_POUR_HORIZONTAL_POSJ",
            "label": "따르기 horizontal",
            "desc": "실시간 용량 피드백 단계 2",
        },
        {
            "tab": code,
            "section": "[2] POUR",
            "type": "posj",
            "config_group": "poses",
            "config_key": f"{code}_pour_diagonal_posj",
            "var_name": f"{prefix_upper}_POUR_DIAGONAL_POSJ",
            "label": "따르기 diagonal",
            "desc": "실시간 용량 피드백 단계 3",
        },
        {
            "tab": code,
            "section": "[2] POUR",
            "type": "posj",
            "config_group": "poses",
            "config_key": f"{code}_pour_vertical_posj",
            "var_name": f"{prefix_upper}_POUR_VERTICAL_POSJ",
            "label": "따르기 vertical",
            "desc": "실시간 용량 피드백 단계 4",
        },
        {
            "tab": code,
            "section": "[3] RETURN(역순)",
            "type": "posj",
            "config_group": "poses",
            "config_key": f"{code}_service_ready_posj",
            "var_name": f"{prefix_upper}_SERVICE_READY_POSJ",
            "label": "원위치 복귀 준비(service_ready)",
            "desc": "위치참조",
            "editable": False,
            "reference_only": True,
        },
        {
            "tab": code,
            "section": "[3] RETURN(역순)",
            "type": "posx",
            "config_group": "poses",
            "config_key": f"{code}_pick_lift_out_posx",
            "var_name": f"{code}_pick_lift_out_posx(return_ref)",
            "label": "복귀 중간(target_4) 절대좌표",
            "desc": "위치참조",
            "editable": False,
            "reference_only": True,
        },
        {
            "tab": code,
            "section": "[3] RETURN(역순)",
            "type": "vision_offset",
            "ingredient_code": code,
            "config_group": "offsets_xyz_mm",
            "config_key": f"{code}_pick_lift_offset",
            "var_name": f"{code}_pick_lift_offset(return_ref)",
            "label": "복귀 중간(target_3) 오프셋",
            "desc": "위치참조",
            "editable": False,
            "reference_only": True,
        },
        {
            "tab": code,
            "section": "[3] RETURN(역순)",
            "type": "vision_offset",
            "ingredient_code": code,
            "config_group": "offsets_xyz_mm",
            "config_key": f"{code}_pick_grasp_offset",
            "var_name": f"{code}_pick_grasp_offset(return_ref)",
            "label": "병 원위치 안착(target_2) 오프셋",
            "desc": "위치참조",
            "editable": False,
            "reference_only": True,
        },
        {
            "tab": code,
            "section": "[3] RETURN(역순)",
            "type": "gripper_open_action",
            "ingredient_code": code,
            "var_name": f"{code}_gripper_open_return",
            "label": "병 놓기(그리퍼 오픈)",
            "desc": "위치참조",
            "editable": False,
            "reference_only": True,
        },
        {
            "tab": code,
            "section": "[3] RETURN(역순)",
            "type": "vision_offset",
            "ingredient_code": code,
            "config_group": "offsets_xyz_mm",
            "config_key": f"{code}_pick_approach_offset",
            "var_name": f"{code}_pick_approach_offset(return_ref)",
            "label": "병 원위치 이탈(target_1) 오프셋",
            "desc": "위치참조",
            "editable": False,
            "reference_only": True,
        },
        {
            "tab": code,
            "section": "[3] RETURN(역순)",
            "type": "posj",
            "config_group": "poses",
            "config_key": f"{code}_service_ready_posj",
            "var_name": f"{prefix_upper}_SERVICE_READY_POSJ",
            "label": "다음 재료 준비(service_ready)",
            "desc": "위치참조",
            "editable": False,
            "reference_only": True,
        },
    ]


def _build_glass_robot_action_pose_rows():
    code = "glass"
    return [
        {
            "tab": "glass",
            "section": "[0] 재료보정",
            "type": "menu_gripper_close_mm",
            "ingredient_code": code,
            "var_name": "glass_gripper_close_mm",
            "label": "글라스잔 파지 거리(mm)",
            "desc": "완성컵 파지 시 사용할 그리퍼 닫힘 거리(mm)",
        },
        {
            "tab": "glass",
            "section": "[0] 재료보정",
            "type": "menu_gripper_open_mm",
            "ingredient_code": code,
            "var_name": "glass_gripper_open_mm",
            "label": "글라스잔 오픈 거리(mm)",
            "desc": "완성컵 파지 전 사용할 그리퍼 오픈 거리(mm)",
        },
        {
            "tab": "glass",
            "section": "[4] CUP PICK",
            "type": "gripper_open_action",
            "ingredient_code": code,
            "var_name": "glass_gripper_open_pick",
            "label": "컵 집기 전 그리퍼 오픈",
            "desc": "위치참조",
            "editable": False,
            "reference_only": True,
        },
        {
            "tab": "glass",
            "section": "[4] CUP PICK",
            "type": "posj",
            "config_group": "poses",
            "config_key": "cup_pick_ready_posj",
            "var_name": "CUP_PICK_READY_POSJ",
            "label": "컵 집기 준비자세",
            "desc": "컵 집기 전 준비 posj",
        },
        {
            "tab": "glass",
            "section": "[4] CUP PICK",
            "type": "posx",
            "config_group": "poses",
            "config_key": "cup_pick_approach_posx",
            "var_name": "CUP_PICK_APPROACH_POSX",
            "label": "컵 집기 접근(고정)",
            "desc": "컵 집기 고정 posx",
        },
        {
            "tab": "glass",
            "section": "[4] CUP PICK",
            "type": "posx",
            "config_group": "poses",
            "config_key": "cup_pick_pose_posx",
            "var_name": "CUP_PICK_POSE_POSX",
            "label": "컵 집기 위치(고정)",
            "desc": "컵 집기 고정 posx",
        },
        {
            "tab": "glass",
            "section": "[4] CUP PICK",
            "type": "gripper_close_action",
            "ingredient_code": code,
            "var_name": "glass_gripper_close_pick",
            "label": "완성컵 파지(그리퍼 닫힘)",
            "desc": "위치참조",
            "editable": False,
            "reference_only": True,
        },
        {
            "tab": "glass",
            "section": "[4] CUP PICK",
            "type": "posx",
            "config_group": "poses",
            "config_key": "cup_pick_lift_posx",
            "var_name": "CUP_PICK_LIFT_POSX",
            "label": "컵 리프트(고정)",
            "desc": "컵 집기 후 고정 리프트 posx",
        },
        {
            "tab": "glass",
            "section": "[5] DELIVERY",
            "type": "posj",
            "config_group": "poses",
            "config_key": "cup_delivery_ready_posj",
            "var_name": "CUP_DELIVERY_READY_POSJ",
            "label": "전달 준비자세",
            "desc": "컵 전달 전 준비 posj",
        },
        {
            "tab": "glass",
            "section": "[5] DELIVERY",
            "type": "posx",
            "config_group": "poses",
            "config_key": "cup_delivery_approach_posx",
            "var_name": "CUP_DELIVERY_APPROACH_POSX",
            "label": "전달 위치 접근",
            "desc": "완성컵 전달 전 접근 posx",
        },
        {
            "tab": "glass",
            "section": "[5] DELIVERY",
            "type": "posx",
            "config_group": "poses",
            "config_key": "cup_delivery_pose_posx",
            "var_name": "CUP_DELIVERY_POSE_POSX",
            "label": "전달 위치 안착",
            "desc": "완성컵 전달 안착 posx",
        },
        {
            "tab": "glass",
            "section": "[5] DELIVERY",
            "type": "gripper_open_action",
            "ingredient_code": code,
            "var_name": "glass_gripper_open_release",
            "label": "완성컵 릴리즈(그리퍼 오픈)",
            "desc": "위치참조",
            "editable": False,
            "reference_only": True,
        },
        {
            "tab": "glass",
            "section": "[5] DELIVERY",
            "type": "posx",
            "config_group": "poses",
            "config_key": "cup_delivery_approach_posx",
            "var_name": "CUP_DELIVERY_APPROACH_POSX",
            "label": "전달 위치 이탈(공용)",
            "desc": "전달 후 안전 이탈(전달 접근 posx 공용)",
        },
    ]


ROBOT_ACTION_POSE_TABS = [
    ("soju", ROBOT_ACTION_POSE_TAB_LABELS.get("soju", "소주")),
    ("beer", ROBOT_ACTION_POSE_TAB_LABELS.get("beer", "맥주")),
    ("glass", ROBOT_ACTION_POSE_TAB_LABELS.get("glass", "글라스잔")),
]
ROBOT_ACTION_POSE_ROW_DEFS = (
    _build_liquid_robot_action_pose_rows("soju", ROBOT_ACTION_POSE_TAB_LABELS.get("soju", "소주"))
    + _build_liquid_robot_action_pose_rows("beer", ROBOT_ACTION_POSE_TAB_LABELS.get("beer", "맥주"))
    + _build_glass_robot_action_pose_rows()
)
CALIB_DIR = os.path.join(PARAM_DIR, "calibration")
CALIB_ROBOT_DIR = CALIB_DIR
CALIB_ROTMAT_DIR = CALIB_DIR
LEGACY_CALIB_DIR = os.path.join(PROJECT_ROOT, "calibration")
LEGACY_CALIB_ROBOT_DIR = os.path.join(LEGACY_CALIB_DIR, "Data_robot")
LEGACY_CALIB_ROTMAT_DIR = os.path.join(LEGACY_CALIB_DIR, "Data_RotMat")
LEGACY_CALIB_ACTIVE_PATH_FILE = os.path.join(LEGACY_CALIB_DIR, "active_calibration_path.txt")
CALIB_DEFAULT_PATTERN_COLS = 7
CALIB_DEFAULT_PATTERN_ROWS = 9
CALIB_DETECTION_HOLD_SEC = float(os.environ.get("CALIB_DETECTION_HOLD_SEC", "1.2"))
CALIB_DETECT_INTERVAL_SEC = float(os.environ.get("CALIB_DETECT_INTERVAL_SEC", "0.18"))
CALIB_PROCESS_HZ = float(os.environ.get("CALIB_PROCESS_HZ", "10.0"))
VISION_META_PROCESS_HZ = float(os.environ.get("VISION_META_PROCESS_HZ", "30.0"))
VISION_META_STALE_SEC = float(os.environ.get("VISION_META_STALE_SEC", "2.0"))
VISION_META_HOLD_SEC = float(os.environ.get("VISION_META_HOLD_SEC", "1.5"))
VISION_RUNTIME_UI_HOLD_SEC = float(os.environ.get("VISION_RUNTIME_UI_HOLD_SEC", "3.0"))
VISION_OVERLAY_REFRESH_MIN_INTERVAL_SEC = max(
    0.02, float(os.environ.get("VISION_OVERLAY_REFRESH_MIN_INTERVAL_SEC", "0.10"))
)
VISION_OVERLAY_REFRESH_FORCE_STALE_SEC = max(
    0.10, float(os.environ.get("VISION_OVERLAY_REFRESH_FORCE_STALE_SEC", "0.35"))
)
# 카메라 프레임 수신 지연(stale) 판정 기준(비전 전용)
VISION_CAMERA_STALE_SEC = max(0.5, float(os.environ.get("VISION_CAMERA_STALE_SEC", "5.0")))
VISION_RENDER_STALL_WARN_MS = max(1000.0, float(os.environ.get("VISION_RENDER_STALL_WARN_MS", "1000.0")))
VISION_RENDER_STALL_LOG_COOLDOWN_SEC = max(
    0.5, float(os.environ.get("VISION_RENDER_STALL_LOG_COOLDOWN_SEC", "1.0"))
)
VISION_RENDER_TRACE_ENABLED = os.environ.get("VISION_RENDER_TRACE", "0") == "1"
VISION_RENDER_TRACE_MIN_INTERVAL_SEC = max(
    0.0, float(os.environ.get("VISION_RENDER_TRACE_MIN_INTERVAL_SEC", "0.05"))
)
VISION_RENDER_INTERVAL_MS = max(10, int(float(os.environ.get("VISION_RENDER_INTERVAL_MS", "16"))))
VISION_UI_CYCLE_LABEL_INTERVAL_MS = max(
    80, int(float(os.environ.get("VISION_UI_CYCLE_LABEL_INTERVAL_MS", "160")))
)
TOP_STATUS_ANIM_INTERVAL_MS = max(
    50, int(float(os.environ.get("TOP_STATUS_ANIM_INTERVAL_MS", "90")))
)
BARTENDER_MODE_UI_REFRESH_MIN_INTERVAL_SEC = max(
    0.10, float(os.environ.get("BARTENDER_MODE_UI_REFRESH_MIN_INTERVAL_SEC", "0.25"))
)
VISION_RECOVERY_TIMER_INTERVAL_MS = max(
    80, int(float(os.environ.get("VISION_RECOVERY_TIMER_INTERVAL_MS", "120")))
)
DEFAULT_VISION1_SERIAL = os.environ.get("DEFAULT_VISION1_SERIAL", "313522301601")
DEFAULT_VISION2_SERIAL = os.environ.get("DEFAULT_VISION2_SERIAL", "311322302867")
ROBOT_MODE_HINT = (str(os.environ.get("BARTENDER_ROBOT_MODE_HINT", "real") or "").strip().lower() or "real")
ROBOT_MODEL_HINT = str(os.environ.get("BARTENDER_ROBOT_MODEL_HINT", "e0509") or "").strip() or "e0509"
ROBOT_HOST_HINT = str(os.environ.get("BARTENDER_ROBOT_HOST_HINT", "110.120.1.68") or "").strip() or "110.120.1.68"
ROBOT_RT_HOST_HINT = str(os.environ.get("BARTENDER_ROBOT_RT_HOST_HINT", "192.168.137.50") or "").strip() or "192.168.137.50"
ROBOT_GZ_HINT = (str(os.environ.get("BARTENDER_ROBOT_GZ_HINT", "false") or "").strip().lower() or "false")
CALIB_SEQUENCE_ROW_DEFS = [
    ("initial", "초기위치"),
    ("wait1", "대기위치1"),
    ("wait2", "대기위치2"),
    ("p1", "데이터1"),
    ("p2", "데이터2"),
    ("p3", "데이터3"),
    ("p4", "데이터4"),
    ("p5", "데이터5"),
    ("p6", "데이터6"),
    ("p7", "데이터7"),
    ("p8", "데이터8"),
    ("p9", "데이터9"),
    ("p10", "데이터10"),
    ("return1", "복귀위치1"),
    ("return2", "복귀위치2"),
    ("end", "끝위치"),
]

ROBOT_STATE_KR_MAP = {
    "STATE_INITIALIZING": "초기화",
    "STATE_STANDBY": "대기",
    "STATE_MOVING": "이동 중",
    "STATE_SAFE_OFF": "세이프 오프",
    "STATE_TEACHING": "티칭",
    "STATE_SAFE_STOP": "세이프 스톱",
    "STATE_EMERGENCY_STOP": "비상 정지",
    "STATE_HOMMING": "홈 동작",
    "STATE_RECOVERY": "복구",
    "STATE_SAFE_STOP2": "세이프 스톱2",
    "STATE_SAFE_OFF2": "세이프 오프2",
    "STATE_RESERVED1": "예약 상태1",
    "STATE_RESERVED2": "예약 상태2",
    "STATE_RESERVED3": "예약 상태3",
    "STATE_RESERVED4": "예약 상태4",
    "STATE_NOT_READY": "준비 안됨",
    "STATE_LAST": "마지막 상태",
}

ROBOT_MODE_KR_MAP = {
    0: "메뉴얼모드",
    1: "오토모드",
    2: "복구모드",
    3: "백드라이브모드",
    4: "측정모드",
    5: "초기화모드",
    6: "마지막모드",
}

CONTROL_MODE_KR_MAP = {
    0: "위치제어",
    1: "토크제어",
    3: "위치제어",
    4: "토크제어",
}

ROBOT_STATE_NORMAL_CODES = {1, 2}
ROBOT_STATE_WARNING_CODES = {0, 4, 5, 8, 9, 10, 12, 13, 14}
ROBOT_STATE_ERROR_CODES = {3, 6, 7, 11, 15, 16}

def _project_relative_path(path_value):
    if not path_value:
        return ""
    abs_path = os.path.abspath(str(path_value))
    try:
        rel_path = os.path.relpath(abs_path, PROJECT_ROOT)
    except ValueError:
        return abs_path
    if rel_path.startswith(".."):
        return abs_path
    return rel_path.replace(os.sep, "/")


def _resolve_repo_file(path_value, fallback_dirs=None):
    raw = str(path_value or "").strip()
    if not raw:
        return ""

    normalized = raw.replace("\\", "/")
    candidates = []
    if os.path.isabs(raw):
        candidates.append(os.path.abspath(raw))
    else:
        candidates.append(os.path.abspath(os.path.join(PROJECT_ROOT, raw)))

    basename = os.path.basename(normalized)
    if basename:
        for base_dir in fallback_dirs or ():
            candidates.append(os.path.join(base_dir, basename))

    for marker in ("config/calibration/", "param/calibration/", "calibration/"):
        if marker in normalized:
            suffix = normalized.split(marker, 1)[1]
            suffix = suffix.replace("Data_RotMat/", "").replace("Data_robot/", "")
            suffix = suffix.lstrip("/")
            if suffix:
                candidates.append(os.path.join(CALIB_DIR, suffix))
                candidates.append(os.path.join(CALIB_DIR, os.path.basename(suffix)))

    seen = set()
    for candidate in candidates:
        candidate = os.path.abspath(candidate)
        if candidate in seen:
            continue
        seen.add(candidate)
        if os.path.isfile(candidate):
            return candidate
    return ""


class EmittingStream(QObject):
    text_written = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._buffer = ""

    def write(self, text):
        if not text:
            return
        self._buffer += str(text)
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            if line:
                self.text_written.emit(line + "\n")

    def flush(self):
        if self._buffer:
            self.text_written.emit(self._buffer)
            self._buffer = ""


class BackendStartupWorker(QObject):
    progress = pyqtSignal(int, str, float)
    finished = pyqtSignal(object)
    failed = pyqtSignal(str, str)

    def __init__(self, use_real_gripper):
        super().__init__()
        self.use_real_gripper = use_real_gripper

    def run(self):
        started = time.monotonic()
        backend = RobotBackend(use_real_gripper=self.use_real_gripper)

        def progress_callback(percent, message):
            elapsed = time.monotonic() - started
            self.progress.emit(percent, message, elapsed)

        try:
            backend.start(progress_callback=progress_callback)
            elapsed = time.monotonic() - started
            self.progress.emit(100, "초기화 완료", elapsed)
            self.finished.emit(backend)
        except Exception as e:
            self.failed.emit(str(e), traceback.format_exc())


class BackendResetWorker(QObject):
    finished = pyqtSignal(bool, str)
    failed = pyqtSignal(str, str)

    def __init__(self, backend):
        super().__init__()
        self.backend = backend

    def run(self):
        try:
            ok, msg = self.backend.reset_robot_state()
            self.finished.emit(bool(ok), str(msg))
        except Exception as e:
            self.failed.emit(str(e), traceback.format_exc())


class BackendMotionStopWorker(QObject):
    finished = pyqtSignal(bool, str)
    failed = pyqtSignal(str, str)

    def __init__(self, backend, stop_mode: int = 2):
        super().__init__()
        self.backend = backend
        self.stop_mode = int(stop_mode)

    def run(self):
        try:
            ok, msg = self.backend.send_motion_stop(stop_mode=int(self.stop_mode))
            self.finished.emit(bool(ok), str(msg))
        except Exception as e:
            self.failed.emit(str(e), traceback.format_exc())


class VoiceOrderCycleWorker(QObject):
    tick = pyqtSignal(float)
    finished = pyqtSignal()

    def __init__(self, interval_ms: int):
        super().__init__()
        self._interval_sec = max(0.001, float(interval_ms) / 1000.0)
        self._stop_event = threading.Event()

    @pyqtSlot()
    def run(self):
        try:
            next_ts = time.monotonic()
            while not self._stop_event.is_set():
                now = time.monotonic()
                self.tick.emit(float(now))
                next_ts += self._interval_sec
                sleep_sec = max(0.0, next_ts - time.monotonic())
                if sleep_sec > 0.0:
                    # Keep short waits so stop requests are observed quickly.
                    self._stop_event.wait(timeout=min(sleep_sec, 0.05))
        finally:
            self.finished.emit()

    @pyqtSlot()
    def stop(self):
        self._stop_event.set()


class App(QMainWindow, form):
    ros_log_received = pyqtSignal(str)
    ros_image_received = pyqtSignal(QImage)
    calibration_ui_refresh_requested = pyqtSignal()
    vision_runtime_ui_refresh_requested = pyqtSignal(int)
    voice_order_event_received = pyqtSignal(object)

    def __init__(self, backend=None, auto_start_backend=True):
        super().__init__()
        self.setupUi(self)
        self._status_row_ready = False
        self._closing = False
        self._ui_tick_error_last_at = {}
        self._ui_tick_slow_last_at = {}
        self._vision_render_stall_last_log_at_1 = 0.0
        self._vision_render_stall_last_log_at_2 = 0.0
        self._vision_render_trace_last_log_at_1 = 0.0
        self._vision_render_trace_last_log_at_2 = 0.0
        self._ui_perf_log_path = str(FRONTEND_UI_PERF_LOG_PATH or "").strip()
        self._ui_perf_log_lock = threading.Lock()
        if self._ui_perf_log_path:
            try:
                os.makedirs(os.path.dirname(self._ui_perf_log_path), exist_ok=True)
            except Exception:
                self._ui_perf_log_path = ""

        self.backend = backend
        self._auto_start_backend = bool(auto_start_backend)
        self._backend_thread = None
        self._backend_worker = None
        self._reset_thread = None
        self._reset_worker = None
        self._motion_stop_thread = None
        self._motion_stop_worker = None
        self._mode_scan_at = 0.0
        self._mode_text_cached = "판단 중"
        self._table_value_cache = {}
        self._table_flash_until = {}
        self._pos_value_cache = {}
        self._pos_flash_until = {}
        self._current_tool_label = None
        self._current_tool_text_cache = ""
        self._current_tool_sync_lock = threading.Lock()
        self._current_tool_sync_inflight = False
        self._current_tool_sync_next_try_at = 0.0
        self._current_tool_sync_retry_deadline = 0.0
        self._backend_ready_at = None
        self._last_robot_state_text = ""
        self._robot_state_flash_until = 0.0
        self._top_status_widgets = {}
        self._top_status_toggles = {}
        self._top_status_enabled = {"vision": True, "vision2": True, "robot": True}
        self._top_status_panel = None
        self._top_status_mid_line = None
        self._top_status_state_cache = {}
        self._robot_controls_enabled_cache = None
        self._dashboard_resize_guard = False
        self._voice_panel_shift_applied = False
        self._voice_panel_shift_dx = 0
        self._voice_panel_layout_rect = None
        self._voice_order_panel = None
        self._voice_order_title_label = None
        self._voice_order_connection_dot = None
        self._voice_order_connection_label = None
        self._voice_order_connection_toggle = None
        self._voice_order_cycle_label = None
        self._voice_order_status_label = None
        self._voice_order_input_label = None
        self._voice_order_input_edit = None
        self._voice_order_last_input_text = ""
        self._voice_order_llm_label = None
        self._voice_order_llm_edit = None
        self._voice_order_last_llm_text = ""
        self._voice_order_start_button = None
        self._voice_order_log_label = None
        self._voice_order_log_view = None
        self._voice_order_result_label = None
        self._voice_order_recipe_view = None
        self._voice_order_mic_button = None
        self._voice_order_mic_state_label = None
        self._voice_order_mic_level_bar = None
        self._voice_order_mic_test_panel = None
        self._voice_order_mic_test_title_label = None
        self._voice_order_mic_test_level_bar = None
        self._voice_order_mic_test_value_label = None
        self._voice_order_mic_test_running = False
        self._voice_order_mic_test_thread = None
        self._voice_order_mic_test_stop_event = threading.Event()
        self._voice_order_mic_test_lock = threading.Lock()
        self._voice_order_mic_test_latest_level = 0
        self._voice_order_mic_test_error = ""
        self._voice_order_mic_test_poll_timer = None
        self._voice_order_mic_test_selected_index = None
        self._voice_order_mic_test_selected_name = ""
        self._voice_order_mic_level_timer = None
        self._voice_order_mic_level_phase = 0.0
        self._voice_order_mic_level_active = False
        self._voice_order_mic_level_last_at = 0.0
        self._voice_order_mic_state_text = "대기"
        self._voice_order_webui_button = None
        self._voice_order_html_badge = None
        self._webui_order_start_enabled_cached = None
        self._webui_order_start_enabled_pending = None
        self._webui_order_start_sync_lock = threading.Lock()
        self._webui_order_start_sync_cv = threading.Condition(self._webui_order_start_sync_lock)
        self._webui_order_start_sync_thread = None
        self._webui_order_start_sync_stop = threading.Event()
        self._webui_order_start_sync_last_try_at = 0.0
        self._webui_order_start_sync_error_last = ""
        self._webui_order_start_sync_error_at = 0.0
        self._voice_order_cycle_ms = None
        self._voice_prev_update_at = None
        self._voice_backend_last_seen_at = None
        self._voice_backend_poll_ok_at = None
        self._voice_order_status_text = "비활성화"
        self._voice_order_status_severity = "warn"
        self._voice_order_enabled = False
        self._voice_order_process_state = "대기"
        self._sensor_launch_proc = None
        self._sensor_launch_signature = ""
        self._robot_launch_proc = None
        self._robot_launch_signature = ""
        self._voice_cycle_thread = None
        self._voice_cycle_worker = None
        self._voice_cycle_worker_token = 0
        self._voice_worker_thread = None
        self._voice_worker_proc = None
        self._voice_worker_token = 0
        self._voice_worker_done = False
        self._voice_worker_running = False
        self._voice_retry_pending_after_tts = False
        self._voice_sequence_running = False
        self._voice_sequence_run_id = 0
        self._bartender_sequence_run_id = 0
        self._voice_ready_tone_armed = False
        self._voice_ready_tone_last_at = 0.0
        self._voice_ready_tone_warmed = False
        self._voice_last_result = None
        self._bartender_overview_panel = None
        self._bartender_overview_rect = None
        self._bartender_title_label = None
        self._bartender_mode_group = None
        self._bartender_manual_mode_button = None
        self._bartender_auto_mode_button = None
        self._bartender_settings_box = None
        self._bartender_offset_button = None
        self._bartender_pose_button = None
        self._bartender_speed_title_label = None
        self._bartender_speed_slider = None
        self._bartender_speed_value_label = None
        self._bartender_start_button = None
        self._bartender_robot_action_test_button = None
        self._bartender_mode_hint_label = None
        self._bartender_status_label = None
        self._bartender_sequence_area = None
        self._bartender_sequence_widgets = {}
        self._bartender_sequence_state = {}
        self._bartender_mode = "manual"
        self._bartender_auto_ready = False
        self._bartender_sequence_running = False
        self._bartender_active_step = ""
        self._bartender_active_started_at = 0.0
        self._bartender_manual_step_index = -1
        self._bartender_safety_last_check_at = 0.0
        self._bartender_last_safety_reason = ""
        self._bartender_last_log_id = 0
        self._bartender_last_tts_signature = ""
        self._voice_feedback_audio_gen = 0
        self._bartender_status_lock = False
        self._menu_xyz_offsets_by_code = {}
        self._menu_gripper_close_mm_by_code = {}
        self._menu_gripper_open_mm_by_code = {}
        self._menu_label_by_code = dict(BARTENDER_MENU_LABELS)
        self._robot_action_pose_config = {}
        self._robot_action_cached_vision_target_payload_by_code = {}
        self._robot_action_cached_vision_target_last_code = ""
        self._robot_action_cached_vision_target_last_label = ""
        self._motion_speed_percent = int(DEFAULT_MOTION_SPEED_PERCENT)
        self._motion_speed_slider = None
        self._motion_speed_title_label = None
        self._motion_speed_value_label = None
        self._emergency_stop_button = None

        self._setup_voice_order_panel()
        self._setup_bartender_overview_panel()
        self._reserve_top_status_space()
        self._setup_top_status_row()

        self.terminal.setReadOnly(True)
        self.terminal.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.terminal.document().setMaximumBlockCount(600)
        self.terminal.setFont(QFont(UI_FONT_FAMILY, UI_TERMINAL_FONT_SIZE))
        self.terminal.setStyleSheet(
            "QPlainTextEdit {"
            f" font-family: '{UI_FONT_FAMILY}';"
            f" font-size: {UI_TERMINAL_FONT_SIZE}pt;"
            " background: #0f172a;"
            " color: #dbeafe;"
            " border: 1px solid #334155;"
            " border-radius: 4px;"
            " padding: 6px;"
            " selection-background-color: #1f6feb;"
            " selection-color: #ffffff;"
            " }"
        )
        self._setup_log_controls()

        # Control-button wiring reads these states, so initialize before _setup_control_buttons().
        self._vision_click_dialog_enabled = False
        self._vision_move_offset_xyz_mm = (
            float(VISION_MOVE_DEFAULT_OFFSET_X_MM),
            float(VISION_MOVE_DEFAULT_OFFSET_Y_MM),
            float(VISION_MOVE_DEFAULT_OFFSET_Z_MM),
        )
        self._calibration_mode_enabled = False
        self._calibration_mode_enabled_1 = False
        self._calibration_mode_enabled_2 = False
        self._calib_pattern_cols = CALIB_DEFAULT_PATTERN_COLS
        self._calib_pattern_rows = CALIB_DEFAULT_PATTERN_ROWS
        self._calib_last_points_uvz_mm = None
        self._calib_last_points_uvz_mm_1 = None
        self._calib_last_points_uvz_mm_2 = None
        self._calib_last_points_at = None
        self._calib_last_points_at_1 = None
        self._calib_last_points_at_2 = None
        self._calib_last_grid_pts_1 = None
        self._calib_last_grid_pts_2 = None
        self._calib_last_grid_status_1 = None
        self._calib_last_grid_status_2 = None
        self._calib_grid_camera_xyz_1 = None
        self._calib_grid_camera_xyz_2 = None
        self._calib_matrix_path = None
        self._calib_matrix_path_1 = None
        self._calib_matrix_path_2 = None
        self._calib_last_reason = "대기"
        self._calib_last_reason_1 = "대기"
        self._calib_last_reason_2 = "대기"
        self._calib_full_ready_1 = False
        self._calib_full_ready_2 = False
        self._calib_full_reason_1 = "대기"
        self._calib_full_reason_2 = "대기"
        self._calib_proc_1 = None
        self._calib_proc_2 = None
        self._calib_proc_cmd_1 = None
        self._calib_proc_cmd_2 = None
        self._calib_proc_started_by_ui_1 = False
        self._calib_proc_started_by_ui_2 = False
        self._calib_status_blink_on = False
        self._calibration_sequence_running = False
        self._vision_meta_payload_1 = None
        self._vision_meta_payload_2 = None
        self._vision_meta_received_at_1 = None
        self._vision_meta_received_at_2 = None
        self._vision_meta_last_nonempty_payload_1 = None
        self._vision_meta_last_nonempty_payload_2 = None
        self._vision_meta_last_nonempty_at_1 = None
        self._vision_meta_last_nonempty_at_2 = None
        self._vision_meta_cycle_ms_1 = None
        self._vision_meta_cycle_ms_2 = None

        self._setup_control_buttons()
        self._set_robot_controls_enabled(False)

        # logging state must exist before stdout/stderr redirection starts
        self._log_buffer = []
        self._log_buffer_lock = threading.Lock()
        self._log_line_open = False
        self._last_log_line = ""
        self._last_log_line_at = 0.0
        self._stdout = EmittingStream()
        self._stderr = EmittingStream()
        self._stdout.text_written.connect(self.append_log)
        self._stderr.text_written.connect(self.append_log)
        self.ros_log_received.connect(self.append_log)
        self.ros_image_received.connect(self._update_yolo_view)
        self.calibration_ui_refresh_requested.connect(self._update_calibration_mode_ui)
        self.vision_runtime_ui_refresh_requested.connect(self._update_vision_runtime_panel_ui)
        self.voice_order_event_received.connect(self._on_voice_order_worker_event)
        sys.stdout = self._stdout
        sys.stderr = self._stderr
        sys.excepthook = self._handle_exception
        if hasattr(threading, "excepthook"):
            threading.excepthook = self._handle_thread_exception

        self._log_timer = QTimer(self)
        self._log_timer.timeout.connect(lambda: self._safe_ui_tick("log_flush", self._flush_log_buffer))
        self._log_timer.start(100)
        self.append_log("[시작] 메인창 실행 완료\n")
        self.append_log("[시작] 백엔드 초기화 대기 중...\n")

        self._robot_status_timer = QTimer(self)
        self._robot_status_timer.timeout.connect(
            lambda: self._safe_ui_tick("robot_status", self._refresh_robot_status)
        )
        self._robot_status_timer.start(250)
        self._vision_status_timer_1 = QTimer(self)
        self._vision_status_timer_1.timeout.connect(
            lambda: self._safe_ui_tick("vision_status_1", self._refresh_vision_status_panel, 1)
        )
        self._vision_status_timer_1.start(300)
        self._vision_status_timer_2 = QTimer(self)
        self._vision_status_timer_2.timeout.connect(
            lambda: self._safe_ui_tick("vision_status_2", self._refresh_vision_status_panel, 2)
        )
        self._vision_status_timer_2.start(300)
        self._vision_recovery_timer = QTimer(self)
        self._vision_recovery_timer.timeout.connect(
            lambda: self._safe_ui_tick("vision_recovery", self._process_vision_recovery_queue)
        )
        self._vision_recovery_timer.start(VISION_RECOVERY_TIMER_INTERVAL_MS)
        self._status_timer = self._robot_status_timer
        self._vision_status_timer = None
        self._top_status_anim_timer = QTimer(self)
        self._top_status_anim_timer.timeout.connect(
            lambda: self._safe_ui_tick("top_status_anim", self._tick_top_status_animation)
        )
        self._top_status_anim_timer.start(TOP_STATUS_ANIM_INTERVAL_MS)
        self._calib_status_blink_timer = QTimer(self)
        self._calib_status_blink_timer.timeout.connect(
            lambda: self._safe_ui_tick("calib_blink", self._tick_calibration_status_blink)
        )
        self._calib_status_blink_timer.start(500)

        self._pos_timer = QTimer(self)
        self._pos_timer.timeout.connect(lambda: self._safe_ui_tick("positions", self._refresh_positions))
        self._pos_timer.start(200)
        self._current_tool_timer = QTimer(self)
        self._current_tool_timer.timeout.connect(
            lambda: self._safe_ui_tick("current_tool", self._tick_current_tool_sync)
        )
        self._current_tool_timer.start(700)
        self._vision_render_timer_1 = QTimer(self)
        self._vision_render_timer_1.timeout.connect(
            lambda: self._safe_ui_tick("vision_render_1", self._drain_pending_vision_frame_1)
        )
        self._vision_render_timer_1.start(VISION_RENDER_INTERVAL_MS)
        self._vision_render_timer_2 = QTimer(self)
        self._vision_render_timer_2.timeout.connect(
            lambda: self._safe_ui_tick("vision_render_2", self._drain_pending_vision_frame_2)
        )
        self._vision_render_timer_2.start(VISION_RENDER_INTERVAL_MS)
        self._yolo_thread = None
        self._yolo_worker = None
        self._rosout_sub = None
        self._vision_sub = None
        self._vision_sub_2 = None
        self._calib_meta_sub_1 = None
        self._calib_meta_sub_2 = None
        self._vision_meta_sub_1 = None
        self._vision_meta_sub_2 = None
        self._vision_depth_sub = None
        self._vision_depth_sub_2 = None
        self._vision_cb_group_1 = ReentrantCallbackGroup() if ReentrantCallbackGroup is not None else None
        self._vision_cb_group_2 = ReentrantCallbackGroup() if ReentrantCallbackGroup is not None else None
        self._vision_point_sub = None
        self._vision_preview_sub = None
        self._vision_info_sub = None
        self._vision_info_sub_2 = None
        self._vision_image_topic_in_use = None
        self._vision_image_topic_in_use_2 = None
        self._calib_meta_topic_in_use_1 = None
        self._calib_meta_topic_in_use_2 = None
        self._vision_meta_topic_in_use_1 = None
        self._vision_meta_topic_in_use_2 = None
        self._vision_rebind_last_try_at = 0.0
        self._vision_recovery_pending_panels = set()
        self._vision_bridge_fail_last_at = 0.0
        self._vision_bridge_fail_last_msg = ""
        self._external_vision_proc = None
        self._external_vision_cmd = None
        self._external_vision_started_by_ui = False
        self._external_vision_proc_1 = None
        self._external_vision_proc_2 = None
        self._external_vision_cmd_1 = None
        self._external_vision_cmd_2 = None
        self._external_vision_started_by_ui_1 = False
        self._external_vision_started_by_ui_2 = False
        self._last_yolo_image_size = None
        self._last_yolo_image_size_2 = None
        self._vision_depth_image = None
        self._vision_depth_image_2 = None
        self._vision_depth_encoding = None
        self._vision_depth_encoding_2 = None
        self._vision_depth_shape = None
        self._vision_depth_shape_2 = None
        self._camera_intrinsics = None  # legacy alias for panel 1
        self._camera_intrinsics_1 = None  # (fx, fy, cx, cy)
        self._camera_intrinsics_2 = None  # (fx, fy, cx, cy)
        self._vision_meta_payload_1 = None
        self._vision_meta_payload_2 = None
        self._vision_meta_received_at_1 = None
        self._vision_meta_received_at_2 = None
        self._vision_meta_last_nonempty_payload_1 = None
        self._vision_meta_last_nonempty_payload_2 = None
        self._vision_meta_last_nonempty_at_1 = None
        self._vision_meta_last_nonempty_at_2 = None
        self._vision_meta_cycle_ms_1 = None
        self._vision_meta_cycle_ms_2 = None
        self._last_mouse_xy = None
        self._last_mouse_xy_2 = None
        self._last_clicked_vision_xyz_mm = None
        self._last_clicked_robot_xyz_mm = None
        self._last_clicked_robot_target_xyz_mm = None
        self._last_clicked_source_vision = 1
        self._yolo_zoom = 1.0
        self._yolo_zoom_min = 1.0
        self._yolo_zoom_max = 5.0
        # 시작 시 좌표계 원점(좌상단) 축이 보이도록 초기 포커스를 좌상단으로 둔다.
        self._yolo_pan_x = -1000000.0
        self._yolo_pan_y = -1000000.0
        self._yolo_panning = False
        self._yolo_pan_last_pos = None
        self._yolo_zoom_2 = 1.0
        self._yolo_pan_x_2 = -1000000.0
        self._yolo_pan_y_2 = -1000000.0
        self._yolo_panning_2 = False
        self._yolo_pan_last_pos_2 = None
        self._last_yolo_qimage = None
        self._last_yolo_qimage_2 = None
        self._pending_yolo_qimage = None
        self._pending_yolo_qimage_2 = None
        self._last_raw_bgr_1 = None
        self._last_raw_bgr_2 = None
        self._latest_vision_bgr_1 = None
        self._latest_vision_bgr_2 = None
        self._latest_vision_frame_at_1 = None
        self._latest_vision_frame_at_2 = None
        self._latest_vision_token_1 = 0
        self._latest_vision_token_2 = 0
        self._pending_vision_token_1 = 0
        self._pending_vision_token_2 = 0
        self._vision_frame_pending = False
        self._vision_frame_pending_2 = False
        self._vision_frame_lock_1 = threading.Lock()
        self._vision_frame_lock_2 = threading.Lock()
        self._vision_raw_frame_lock_1 = threading.Lock()
        self._vision_raw_frame_lock_2 = threading.Lock()
        self._vision_compose_lock_1 = threading.Lock()
        self._vision_compose_lock_2 = threading.Lock()
        self._vision_compose_cv_1 = threading.Condition(self._vision_compose_lock_1)
        self._vision_compose_cv_2 = threading.Condition(self._vision_compose_lock_2)
        self._vision_compose_pending_1 = False
        self._vision_compose_pending_2 = False
        self._vision_compose_thread_1 = None
        self._vision_compose_thread_2 = None
        self._vision_compose_stop = threading.Event()
        self._yolo_view_map = None
        self._yolo_view_map_2 = None
        self._last_robot_posx = None
        self._last_positions_seen_at = None
        self._last_vision_frame_at = None
        self._last_vision_frame_at_2 = None
        self._last_camera_frame_at_1 = None
        self._last_camera_frame_at_2 = None
        self._last_vision_point_at = None
        self._vision_prev_update_at = None
        self._vision_prev_update_at_2 = None
        self._vision_render_prev_at = None
        self._vision_render_prev_at_2 = None
        self._vision_overlay_refresh_last_at_1 = 0.0
        self._vision_overlay_refresh_last_at_2 = 0.0
        self._vision_cycle_label_last_update_at = 0.0
        self._bartender_mode_ui_last_update_at = 0.0
        self._robot_prev_state_seen_at = None
        self._vision_cycle_ms = None
        self._vision_cycle_ms_2 = None
        self._vision_decode_ms = None
        self._vision_decode_ms_2 = None
        self._vision_render_delay_ms = None
        self._vision_render_delay_ms_2 = None
        self._vision_render_interval_ms = None
        self._vision_render_interval_ms_2 = None
        self._vision_compose_ms = None
        self._vision_compose_ms_2 = None
        self._pending_vision_enqueued_at_1 = None
        self._pending_vision_enqueued_at_2 = None
        self._calib_proc_input_ms_1 = None
        self._calib_proc_input_ms_2 = None
        self._calib_proc_ms_1 = None
        self._calib_proc_ms_2 = None
        self._calib_proc_publish_ms_1 = None
        self._calib_proc_publish_ms_2 = None
        self._calib_proc_wait_ms_1 = None
        self._calib_proc_wait_ms_2 = None
        self._robot_cycle_ms = None
        self._vision_state_text = "끊김"
        self._vision_state_text_2 = "끊김"
        self._vision_to_robot_affine = None
        self._vision_to_robot_rmse = None
        self._vision_to_robot_affine_1 = None
        self._vision_to_robot_affine_2 = None
        self._vision_to_robot_rmse_1 = None
        self._vision_to_robot_rmse_2 = None
        self._vision_assigned_serial_1 = DEFAULT_VISION1_SERIAL
        self._vision_assigned_serial_2 = DEFAULT_VISION2_SERIAL
        self._runtime_camera_serial_1 = DEFAULT_VISION1_SERIAL
        self._runtime_camera_serial_2 = DEFAULT_VISION2_SERIAL
        self._available_camera_serials = []
        self._vision_serial_label = getattr(self, "_vision_serial_label", None)
        self._vision_serial_label_2 = getattr(self, "_vision_serial_label_2", None)
        self._vision_mode_badge_1 = getattr(self, "_vision_mode_badge_1", None)
        self._vision_mode_badge_2 = getattr(self, "_vision_mode_badge_2", None)
        self._vision_meta_rate_label_1 = getattr(self, "_vision_meta_rate_label_1", None)
        self._vision_meta_rate_label_2 = getattr(self, "_vision_meta_rate_label_2", None)
        self._vision_runtime_detail_label_1 = getattr(self, "_vision_runtime_detail_label_1", None)
        self._vision_runtime_detail_label_2 = getattr(self, "_vision_runtime_detail_label_2", None)
        self._vision_runtime_list_label_1 = getattr(self, "_vision_runtime_list_label_1", None)
        self._vision_runtime_list_label_2 = getattr(self, "_vision_runtime_list_label_2", None)
        self._vision_serial_change_button = getattr(self, "_vision_serial_change_button", None)
        self._vision_serial_change_button_2 = getattr(self, "_vision_serial_change_button_2", None)
        self._vision_rotate_left_button = getattr(self, "_vision_rotate_left_button", None)
        self._vision_rotate_zero_button = getattr(self, "_vision_rotate_zero_button", None)
        self._vision_rotate_right_button = getattr(self, "_vision_rotate_right_button", None)
        self._vision_rotation_label = getattr(self, "_vision_rotation_label", None)
        self._vision_rotate_left_button_2 = getattr(self, "_vision_rotate_left_button_2", None)
        self._vision_rotate_zero_button_2 = getattr(self, "_vision_rotate_zero_button_2", None)
        self._vision_rotate_right_button_2 = getattr(self, "_vision_rotate_right_button_2", None)
        self._vision_rotation_label_2 = getattr(self, "_vision_rotation_label_2", None)
        self._vision_rotation_deg_1 = 0
        self._vision_rotation_deg_2 = 0
        self._vision_stream_token_1 = 0
        self._vision_stream_token_2 = 0
        self._load_vision_serial_settings()
        self._set_vision_panel_controls_enabled(1, bool(self._top_status_enabled.get("vision", True)))
        self._set_vision_panel_controls_enabled(2, bool(self._top_status_enabled.get("vision2", True)))
        self._sync_vision_render_timers()
        self._start_vision_compose_workers()
        self._save_vision_serial_settings()
        self._mode_switch_grace_until = 0.0
        self._vision_mode_switch_grace_until_1 = 0.0
        self._vision_mode_switch_grace_until_2 = 0.0
        self._vision_drop_frames_until_1 = 0.0
        self._vision_drop_frames_until_2 = 0.0
        self._vision_decode_next_at_1 = 0.0
        self._vision_decode_next_at_2 = 0.0
        self._last_robot_comm_connected = False
        self._try_load_calibration_matrix_on_startup()
        self._update_calibration_mode_ui()
        self._svc_retry_log_last_at = 0.0
        self._vision_retry_notice_logged = {1: False, 2: False}

        self._vision_state_label = getattr(self, "vision_state_label", None)
        if self._vision_state_label is not None:
            self._vision_state_label.setText("끊김")
        self._vision_coord_label = getattr(self, "vision_coord_label", None)
        if self._vision_coord_label is not None:
            self._vision_coord_label.setText("")
            self._vision_coord_label.hide()
        self._vision_coord_label_2 = getattr(self, "vision_coord_label_2", None)
        if self._vision_coord_label_2 is not None:
            self._vision_coord_label_2.setText("")
            self._vision_coord_label_2.hide()

        self._robot_state_label = getattr(self, "robot_state_label", None)
        if self._robot_state_label is not None:
            self._robot_state_label.setText("초기화 중")

        self._setup_robot_state_table()
        self._setup_position_tables()
        self._hide_robot_control_title_label()
        self._setup_cycle_time_labels()
        self._update_cycle_time_labels()

        self.yolo_view.setMouseTracking(True)
        self.yolo_view.installEventFilter(self)
        self.yolo_view_2 = getattr(self, "yolo_view_2", None)
        if self.yolo_view_2 is not None:
            self.yolo_view_2.setMouseTracking(True)
            self.yolo_view_2.installEventFilter(self)

        self.statusBar().hide()
        if self._auto_start_backend:
            self._start_backend_async()

    def _shift_main_frames_for_voice_panel(self):
        # Voice-order panel is part of the main grid layout.
        self._voice_panel_shift_dx = 0
        self._voice_panel_shift_applied = True

    def _setup_voice_order_panel(self):
        self._shift_main_frames_for_voice_panel()
        panel = getattr(self, "voice_order_panel", None)
        if panel is None:
            return
        self._voice_order_panel = panel
        self._voice_order_title_label = getattr(self, "voice_order_title_label", None)
        self._voice_order_connection_dot = getattr(self, "voice_order_connection_dot", None)
        self._voice_order_connection_label = getattr(self, "voice_order_connection_label", None)
        self._voice_order_connection_toggle = getattr(self, "voice_order_connection_toggle", None)
        self._voice_order_cycle_label = getattr(self, "voice_order_cycle_label", None)
        self._voice_order_status_label = getattr(self, "voice_order_status_label", None)
        self._voice_order_input_label = getattr(self, "voice_order_input_label", None)
        self._voice_order_input_edit = getattr(self, "voice_order_input_edit", None)
        self._voice_order_llm_label = getattr(self, "voice_order_llm_label", None)
        self._voice_order_llm_edit = getattr(self, "voice_order_llm_edit", None)
        self._voice_order_mic_button = getattr(self, "voice_order_mic_button", None)
        self._voice_order_mic_state_label = getattr(self, "voice_order_mic_state_label", None)
        if self._voice_order_mic_state_label is None:
            mic_state_label = QLabel(panel)
            mic_state_label.setObjectName("voice_order_mic_state_label")
            mic_state_label.setStyleSheet("font-size: 9.2pt; font-weight: 700; color: #1f2937;")
            mic_state_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            self._voice_order_mic_state_label = mic_state_label
        self._voice_order_mic_level_bar = getattr(self, "voice_order_mic_level_bar", None)
        if self._voice_order_mic_level_bar is None:
            mic_level_bar = QProgressBar(panel)
            mic_level_bar.setObjectName("voice_order_mic_level_bar")
            mic_level_bar.setRange(0, 100)
            mic_level_bar.setTextVisible(False)
            mic_level_bar.setValue(0)
            self._voice_order_mic_level_bar = mic_level_bar
        if self._voice_order_mic_level_timer is None:
            timer = QTimer(self)
            timer.setInterval(90)
            timer.timeout.connect(self._tick_voice_order_mic_level)
            self._voice_order_mic_level_timer = timer
        if self._voice_order_mic_test_poll_timer is None:
            test_poll = QTimer(self)
            test_poll.setInterval(90)
            test_poll.timeout.connect(self._poll_voice_order_mic_test_level)
            self._voice_order_mic_test_poll_timer = test_poll
        self._voice_order_mic_test_panel = getattr(self, "voice_order_mic_test_panel", None)
        if self._voice_order_mic_test_panel is None:
            test_panel = QFrame(panel)
            test_panel.setObjectName("voice_order_mic_test_panel")
            test_panel.setStyleSheet(
                "QFrame#voice_order_mic_test_panel { background: #f8fafc; border: 1px solid #cbd5e1; border-radius: 4px; }"
            )
            self._voice_order_mic_test_panel = test_panel
        test_panel = self._voice_order_mic_test_panel
        if self._voice_order_mic_test_title_label is None:
            test_title = QLabel("마이크 테스트", test_panel)
            test_title.setStyleSheet("font-size: 7.6pt; font-weight: 700; color: #334155;")
            test_title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            self._voice_order_mic_test_title_label = test_title
        if self._voice_order_mic_test_level_bar is None:
            test_bar = QProgressBar(test_panel)
            test_bar.setRange(0, 100)
            test_bar.setTextVisible(False)
            test_bar.setValue(0)
            test_bar.setStyleSheet(
                "QProgressBar { border: 1px solid #cbd5e1; border-radius: 2px; background: #e5e7eb; }"
                "QProgressBar::chunk { background: #94a3b8; border-radius: 2px; }"
            )
            self._voice_order_mic_test_level_bar = test_bar
        if self._voice_order_mic_test_value_label is None:
            test_value = QLabel("0%", test_panel)
            test_value.setStyleSheet("font-size: 7.2pt; font-weight: 700; color: #475569;")
            test_value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self._voice_order_mic_test_value_label = test_value
        for click_widget in (
            self._voice_order_mic_test_panel,
            self._voice_order_mic_test_title_label,
            self._voice_order_mic_test_level_bar,
            self._voice_order_mic_test_value_label,
        ):
            if click_widget is None:
                continue
            try:
                click_widget.setCursor(Qt.PointingHandCursor)
                click_widget.mousePressEvent = self._on_voice_order_mic_test_clicked
            except Exception:
                pass
        self._voice_order_start_button = getattr(self, "voice_order_start_button", None)
        self._voice_order_log_label = getattr(self, "voice_order_log_label", None)
        self._voice_order_log_view = getattr(self, "voice_order_log_view", None)
        self._voice_order_result_label = getattr(self, "voice_order_result_label", None)
        self._voice_order_recipe_view = getattr(self, "voice_order_recipe_view", None)
        self._voice_order_webui_button = getattr(self, "voice_order_webui_button", None)
        self._voice_order_html_badge = getattr(self, "voice_order_html_badge", None)

        if self._voice_order_connection_toggle is not None:
            try:
                self._voice_order_connection_toggle.toggled.disconnect()
            except Exception:
                pass
            self._voice_order_connection_toggle.toggled.connect(self._on_voice_order_connection_toggled)
        if self._voice_order_mic_button is not None:
            try:
                self._voice_order_mic_button.clicked.disconnect()
            except Exception:
                pass
            self._voice_order_mic_button.clicked.connect(self._capture_voice_input_once)
        if self._voice_order_webui_button is not None:
            try:
                self._voice_order_webui_button.clicked.disconnect()
            except Exception:
                pass
            self._voice_order_webui_button.clicked.connect(self._open_voice_order_webui)
        if self._voice_order_input_edit is not None:
            self._voice_order_input_edit.setReadOnly(True)
            self._voice_order_input_edit.setFocusPolicy(Qt.NoFocus)
        if self._voice_order_llm_edit is not None:
            self._voice_order_llm_edit.setReadOnly(True)
            self._voice_order_llm_edit.setFocusPolicy(Qt.NoFocus)
        # 처리로그는 메인 로그 창으로 통합한다.
        if self._voice_order_log_label is not None:
            self._voice_order_log_label.hide()
        if self._voice_order_log_view is not None:
            self._voice_order_log_view.setReadOnly(True)
            self._voice_order_log_view.document().setMaximumBlockCount(700)
            self._voice_order_log_view.hide()
        if self._voice_order_recipe_view is not None:
            self._voice_order_recipe_view.setReadOnly(True)

        self._layout_voice_order_panel()
        self._refresh_voice_order_webui_badge()
        self._set_voice_order_connection_state(False)
        self._set_voice_order_status("비활성화", "warn")
        self._set_voice_order_mic_state("비활성화", "warn")
        self._set_voice_order_mic_visual_active(False)
        self._voice_order_process_state = "대기"
        self._set_voice_order_result(None)
        self._update_voice_mic_button_state()

    def _setup_bartender_overview_panel(self):
        panel = getattr(self, "_bartender_overview_panel", None)
        if panel is None:
            panel = QFrame(self.centralwidget)
            panel.setObjectName("bartender_overview_panel")
            panel.setFrameShape(QFrame.Box)
            panel.setFrameShadow(QFrame.Raised)
            panel.setStyleSheet(
                "QFrame#bartender_overview_panel { background: #ffffff; border: 1px solid #000000; border-radius: 6px; }"
            )
            self._bartender_overview_panel = panel

        title = QLabel("바텐더 로봇", panel)
        title.setFont(QFont(UI_FONT_FAMILY, 18, QFont.Bold))
        title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._bartender_title_label = title

        manual_btn = QPushButton("메뉴얼모드", panel)
        auto_btn = QPushButton("오토모드", panel)
        for btn in (manual_btn, auto_btn):
            btn.setCheckable(True)
            btn.setStyleSheet(
                "QPushButton { background: #edf2f7; color: #1f2937; border: 1px solid #cbd5e1; border-radius: 4px; font-size: 8.8pt; font-weight: 700; }"
                "QPushButton:checked { background: #1f6feb; color: #ffffff; border-color: #1f6feb; }"
            )

        mode_group = QButtonGroup(panel)
        mode_group.setExclusive(True)
        mode_group.addButton(manual_btn, 0)
        mode_group.addButton(auto_btn, 1)
        mode_group.buttonClicked[int].connect(self._on_bartender_mode_button_clicked)
        self._bartender_mode_group = mode_group
        self._bartender_manual_mode_button = manual_btn
        self._bartender_auto_mode_button = auto_btn

        settings_box = QFrame(panel)
        settings_box.setObjectName("bartender_settings_box")
        settings_box.setStyleSheet(
            "QFrame#bartender_settings_box { background: #f8fafc; border: 1px solid #cbd5e1; border-radius: 6px; }"
        )
        self._bartender_settings_box = settings_box

        pose_btn = QPushButton("로봇 액션 포지션/재료 오프셋 설정", settings_box)
        pose_btn.setStyleSheet(
            "QPushButton { background: #eef2ff; color: #1f2937; border: 1px solid #c7d2fe; border-radius: 4px; font-size: 8.8pt; font-weight: 700; }"
            "QPushButton:hover { background: #e0e7ff; }"
        )
        pose_btn.clicked.connect(self._open_robot_action_pose_dialog)
        self._bartender_pose_button = pose_btn

        speed_title = QLabel("로봇 시퀀스 속도 (0~100%)", settings_box)
        speed_title.setStyleSheet("color: #1f3b63; font-size: 8.5pt; font-weight: 700;")
        speed_title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._bartender_speed_title_label = speed_title

        speed_slider = QSlider(Qt.Horizontal, settings_box)
        speed_slider.setObjectName("bartender_speed_slider")
        speed_slider.setRange(MOTION_SPEED_MIN_PERCENT, MOTION_SPEED_MAX_PERCENT)
        speed_slider.setSingleStep(1)
        speed_slider.setPageStep(10)
        speed_slider.setTickInterval(10)
        speed_slider.setTickPosition(QSlider.NoTicks)
        speed_slider.valueChanged.connect(self._on_bartender_motion_speed_changed)
        self._bartender_speed_slider = speed_slider

        speed_value = QLabel(settings_box)
        speed_value.setStyleSheet("color: #1f3b63; font-size: 9pt; font-weight: 700;")
        speed_value.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._bartender_speed_value_label = speed_value

        start_btn = QPushButton("메뉴얼 시퀀스 시작", panel)
        start_btn.setStyleSheet(
            "QPushButton { background: #166534; color: #ffffff; border: 1px solid #166534; border-radius: 4px; font-size: 9.2pt; font-weight: 800; }"
            "QPushButton:hover { background: #14532d; }"
            "QPushButton:disabled { background: #94a3b8; border-color: #94a3b8; color: #f8fafc; }"
        )
        start_btn.clicked.connect(self._run_bartender_manual_sequence)
        self._bartender_start_button = start_btn

        robot_action_test_btn = QPushButton("7. 로봇 동작만 테스트", panel)
        robot_action_test_btn.setStyleSheet(
            "QPushButton { background: #0f766e; color: #ffffff; border: 1px solid #0f766e; border-radius: 4px; font-size: 8.8pt; font-weight: 800; }"
            "QPushButton:hover { background: #115e59; }"
            "QPushButton:disabled { background: #94a3b8; border-color: #94a3b8; color: #f8fafc; }"
        )
        robot_action_test_btn.clicked.connect(self._run_bartender_robot_action_only_sequence)
        self._bartender_robot_action_test_button = robot_action_test_btn

        status_label = QLabel("상태: 대기", panel)
        status_label.setStyleSheet("font-size: 10pt; font-weight: 800; color: #1f2937;")
        status_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._bartender_status_label = status_label

        hint_label = QLabel(panel)
        hint_label.setWordWrap(True)
        hint_label.setStyleSheet("font-size: 8.8pt; font-weight: 600; color: #475569;")
        hint_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self._bartender_mode_hint_label = hint_label

        seq_area = QFrame(panel)
        seq_area.setObjectName("bartender_sequence_area")
        seq_area.setStyleSheet(
            "QFrame#bartender_sequence_area { background: #f8fafc; border: 1px solid #cbd5e1; border-radius: 6px; }"
        )
        self._bartender_sequence_area = seq_area

        self._bartender_sequence_widgets = {}
        self._bartender_sequence_state = {}
        for idx, (step_key, step_label) in enumerate(BARTENDER_SEQUENCE_STEPS, start=1):
            lbl = QLabel(f"{idx}. {step_label}", seq_area)
            lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            lbl.setStyleSheet(
                "background: #ffffff; border: 1px solid #dbe2ea; border-radius: 4px; color: #334155; font-size: 8.7pt; font-weight: 700;"
            )
            self._bartender_sequence_widgets[str(step_key)] = lbl
            self._bartender_sequence_state[str(step_key)] = "pending"

        if self._bartender_manual_mode_button is not None:
            self._bartender_manual_mode_button.setChecked(True)
        self._bartender_mode = "manual"
        self._bartender_sequence_running = False
        self._bartender_active_step = ""
        self._bartender_active_started_at = 0.0
        self._bartender_manual_step_index = -1
        self._load_menu_xyz_offsets()
        self._robot_action_pose_config = self._load_robot_action_pose_config()
        self._refresh_menu_xyz_offset_button_text()
        self._sync_motion_speed_widgets()
        self._update_bartender_mode_ui()
        self._layout_bartender_overview_panel()
        self._refresh_bartender_sequence_styles()

    def _on_bartender_mode_button_clicked(self, mode_id: int):
        mode = "auto" if int(mode_id) == 1 else "manual"
        if bool(getattr(self, "_bartender_sequence_running", False)):
            self._append_voice_order_log("시퀀스 실행 중에는 모드를 변경할 수 없습니다.", level="warning")
            if self._bartender_mode_group is not None:
                self._bartender_mode_group.blockSignals(True)
                if self._bartender_mode == "auto" and self._bartender_auto_mode_button is not None:
                    self._bartender_auto_mode_button.setChecked(True)
                if self._bartender_mode == "manual" and self._bartender_manual_mode_button is not None:
                    self._bartender_manual_mode_button.setChecked(True)
                self._bartender_mode_group.blockSignals(False)
            return
        if mode == self._bartender_mode:
            self._update_bartender_mode_ui()
            return
        self._bartender_mode = mode
        self._bartender_sequence_running = False
        self._bartender_manual_step_index = -1
        self._bartender_last_safety_reason = ""
        self._bartender_safety_last_check_at = 0.0
        self._stop_voice_order_feedback_audio("모드 전환")
        backend = getattr(self, "backend", None)
        if backend is not None and hasattr(backend, "reset_bartender_sequence"):
            try:
                ok_reset, msg_reset, snap_reset = backend.reset_bartender_sequence(reason="모드 변경: 시퀀스 상태 초기화")
                self._append_voice_order_log(
                    f"시퀀스 상태 초기화 {'완료' if ok_reset else '실패'}: {msg_reset}",
                    level=("info" if ok_reset else "warning"),
                )
                if isinstance(snap_reset, dict):
                    self._apply_bartender_sequence_snapshot(snap_reset)
            except Exception as exc:
                self._append_voice_order_log(f"시퀀스 상태 초기화 예외: {exc}", level="warning")
        self._reset_bartender_sequence(log=False)
        self._voice_last_result = None
        self._set_voice_order_result(None)
        self._set_voice_order_llm_text("-")
        if mode == "auto":
            # 오토모드에서는 개발자 UI 수동 음성 루트를 중지하고 WEB UI 루트만 사용한다.
            self._voice_retry_pending_after_tts = False
            self._stop_voice_order_worker(reason="오토모드 전환: 개발자 수동 음성입력 중지")
        self._update_bartender_mode_ui()
        self._update_voice_mic_button_state()
        self._append_voice_order_log(f"바텐더 시퀀스 모드 변경: {'오토모드' if mode == 'auto' else '메뉴얼모드'}")

    def _is_bartender_auto_ready(self) -> bool:
        if self.backend is None or (not hasattr(self.backend, "is_ready")):
            return False
        try:
            if not bool(self.backend.is_ready()):
                return False
        except Exception:
            return False
        top_required = ("vision", "vision2", "robot")
        for key in top_required:
            if not bool(self._top_status_enabled.get(key, True)):
                return False
            payload = self._top_status_state_cache.get(key, None)
            if not isinstance(payload, tuple) or len(payload) != 2:
                return False
            _state_text, severity = payload
            if str(severity or "").strip().lower() not in ("normal", "ok", "success"):
                return False
        if str(getattr(self, "_voice_order_status_severity", "") or "").strip().lower() not in (
            "normal",
            "ok",
            "success",
        ):
            return False
        return True

    def _update_bartender_mode_ui(self):
        self._bartender_auto_ready = self._is_bartender_auto_ready()
        self._apply_device_toggle_lock_by_mode()
        # manual/auto 동일 조건: 오토모드에서는 WEB UI 보이스 버튼으로 시작만 허용한다.
        auto_unlock = bool(
            (str(self._bartender_mode or "").strip().lower() == "auto")
            and (not bool(getattr(self, "_bartender_sequence_running", False)))
        )
        self._set_webui_order_start_enabled(auto_unlock)
        status_label = getattr(self, "_bartender_status_label", None)
        hint_label = getattr(self, "_bartender_mode_hint_label", None)
        start_btn = getattr(self, "_bartender_start_button", None)
        robot_action_test_btn = getattr(self, "_bartender_robot_action_test_button", None)
        lock_status = bool(getattr(self, "_bartender_status_lock", False))
        mode_text = "오토모드" if self._bartender_mode == "auto" else "메뉴얼모드"
        if bool(self._bartender_sequence_running):
            self._bartender_status_lock = False
            if start_btn is not None:
                start_btn.setEnabled(True)
                start_btn.setText("시퀀스 중지")
            if robot_action_test_btn is not None:
                robot_action_test_btn.setEnabled(False)
            # 실행 중 상세 문구/단계 표시는 시퀀스 스냅샷 처리(_apply_bartender_sequence_snapshot)에서 단일 소스로 갱신한다.
            # 여기서 다시 덮어쓰면 문구가 번갈아 깜박이는 현상이 발생한다.
            return
        if self._bartender_mode == "manual":
            if start_btn is not None:
                start_btn.setEnabled(True)
                start_btn.setText("메뉴얼 시퀀스 시작")
            if robot_action_test_btn is not None:
                robot_action_test_btn.setEnabled(True)
            if hint_label is not None and (not lock_status):
                hint_label.setText("메뉴얼모드: 주변장치 상태 확인 없이 개발자 UI에서 동일 시퀀스를 테스트합니다.")
            if status_label is not None and (not self._bartender_sequence_running) and (not lock_status):
                status_label.setText(f"상태: {mode_text} 대기")
                status_label.setStyleSheet("font-size: 10pt; font-weight: 800; color: #1f2937;")
            return

        if start_btn is not None:
            start_btn.setEnabled(True)
            start_btn.setText("오토모드: WEB UI 실행")
        if robot_action_test_btn is not None:
            robot_action_test_btn.setEnabled(False)
        if hint_label is not None and (not lock_status):
            hint_label.setText("오토모드: WEB UI 보이스 버튼으로 시퀀스를 시작합니다. (메뉴얼과 동일 실행 조건)")
        if status_label is not None and (not self._bartender_sequence_running) and (not lock_status):
            status_label.setText("상태: 오토모드 대기 (WEB UI 보이스 버튼 시작)")
            status_label.setStyleSheet("font-size: 10pt; font-weight: 800; color: #166534;")

    def _apply_device_toggle_lock_by_mode(self):
        lock = str(getattr(self, "_bartender_mode", "") or "").strip().lower() == "auto"
        for _k, toggle in getattr(self, "_top_status_toggles", {}).items():
            if toggle is not None:
                toggle.setEnabled(not lock)
        voice_toggle = getattr(self, "_voice_order_connection_toggle", None)
        if voice_toggle is not None:
            voice_toggle.setEnabled(not lock)

    def _default_menu_xyz_offsets(self):
        offsets = {}
        for menu_code, menu_label in BARTENDER_MENU_LABELS.items():
            offsets[str(menu_code)] = {
                "label": str(menu_label),
                "offset_xyz_mm": [0.0, 0.0, 0.0],
                "gripper_close_mm": 41.0,
                "gripper_open_mm": float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT),
            }
        return offsets

    def _menu_codes_for_offset_ui(self):
        ordered = [code for code in BARTENDER_MENU_LABELS.keys() if str(code).strip().lower() not in BARTENDER_MENU_OFFSET_EXCLUDED_CODES_NORM]
        for code in sorted(self._menu_xyz_offsets_by_code.keys()):
            if str(code).strip().lower() in BARTENDER_MENU_OFFSET_EXCLUDED_CODES_NORM:
                continue
            if code not in ordered:
                ordered.append(code)
        return ordered

    def _load_menu_xyz_offsets(self):
        defaults = self._default_menu_xyz_offsets()
        offsets = {}
        labels = {}
        for code, payload in defaults.items():
            xyz = payload.get("offset_xyz_mm", [0.0, 0.0, 0.0])
            try:
                offsets[code] = (
                    float(xyz[0]),
                    float(xyz[1]),
                    float(xyz[2]),
                )
            except Exception:
                offsets[code] = (0.0, 0.0, 0.0)
            labels[code] = str(payload.get("label", code) or code)
        gripper_map = {
            code: float(payload.get("gripper_close_mm", 41.0))
            for code, payload in defaults.items()
        }
        gripper_open_map = {
            code: float(payload.get("gripper_open_mm", ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT))
            for code, payload in defaults.items()
        }

        try:
            if os.path.isfile(MENU_OFFSET_CONFIG_PATH):
                with open(MENU_OFFSET_CONFIG_PATH, "r", encoding="utf-8") as fp:
                    loaded = json.load(fp)
                menus = loaded.get("menus", loaded) if isinstance(loaded, dict) else {}
                if isinstance(menus, dict):
                    for raw_code, raw_payload in menus.items():
                        code = str(raw_code or "").strip()
                        if not code:
                            continue
                        if str(code).strip().lower() in BARTENDER_MENU_OFFSET_EXCLUDED_CODES_NORM:
                            continue
                        if isinstance(raw_payload, dict):
                            label = str(raw_payload.get("label", labels.get(code, code)) or code)
                            xyz = raw_payload.get("offset_xyz_mm", [0.0, 0.0, 0.0])
                            gripper_close_mm = raw_payload.get("gripper_close_mm", gripper_map.get(code, 41.0))
                            gripper_open_mm = raw_payload.get(
                                "gripper_open_mm",
                                raw_payload.get("gripper_open_mm_default", gripper_open_map.get(code, ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT)),
                            )
                        else:
                            label = labels.get(code, code)
                            xyz = raw_payload
                            gripper_close_mm = gripper_map.get(code, 41.0)
                            gripper_open_mm = gripper_open_map.get(code, ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT)
                        try:
                            x = float(xyz[0]) if isinstance(xyz, (list, tuple)) and len(xyz) >= 1 else 0.0
                            y = float(xyz[1]) if isinstance(xyz, (list, tuple)) and len(xyz) >= 2 else 0.0
                            z = float(xyz[2]) if isinstance(xyz, (list, tuple)) and len(xyz) >= 3 else 0.0
                        except Exception:
                            x, y, z = 0.0, 0.0, 0.0
                        try:
                            grip_v = float(gripper_close_mm)
                            if (not np.isfinite(grip_v)) or grip_v < 0.0:
                                grip_v = float(gripper_map.get(code, 41.0))
                        except Exception:
                            grip_v = float(gripper_map.get(code, 41.0))
                        try:
                            grip_open_v = float(gripper_open_mm)
                            if (not np.isfinite(grip_open_v)) or grip_open_v < 0.0:
                                grip_open_v = float(gripper_open_map.get(code, ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT))
                            grip_open_v = min(float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT), float(grip_open_v))
                        except Exception:
                            grip_open_v = float(gripper_open_map.get(code, ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT))
                        if not np.isfinite(np.asarray([x, y, z], dtype=np.float64)).all():
                            x, y, z = 0.0, 0.0, 0.0
                        labels[code] = label
                        offsets[code] = (float(x), float(y), float(z))
                        gripper_map[code] = float(grip_v)
                        gripper_open_map[code] = float(grip_open_v)
        except Exception as exc:
            self._append_voice_order_log(f"오프셋 설정 로드 실패: {exc}", level="warning")

        self._menu_xyz_offsets_by_code = dict(offsets)
        self._menu_gripper_close_mm_by_code = dict(gripper_map)
        self._menu_gripper_open_mm_by_code = dict(gripper_open_map)
        self._menu_label_by_code = dict(labels)
        return dict(self._menu_xyz_offsets_by_code)

    def _save_menu_xyz_offsets(self):
        payload = {"menus": {}}
        for code in self._menu_codes_for_offset_ui():
            xyz = self._menu_xyz_offsets_by_code.get(code, (0.0, 0.0, 0.0))
            label = str(self._menu_label_by_code.get(code, code) or code)
            try:
                grip_mm = float(self._menu_gripper_close_mm_by_code.get(code, 41.0))
                if (not np.isfinite(grip_mm)) or grip_mm < 0.0:
                    grip_mm = 41.0
            except Exception:
                grip_mm = 41.0
            try:
                grip_open_mm = float(self._menu_gripper_open_mm_by_code.get(code, ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT))
                if (not np.isfinite(grip_open_mm)) or grip_open_mm < 0.0:
                    grip_open_mm = float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT)
                grip_open_mm = min(float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT), float(grip_open_mm))
            except Exception:
                grip_open_mm = float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT)
            payload["menus"][str(code)] = {
                "label": label,
                "offset_xyz_mm": [float(xyz[0]), float(xyz[1]), float(xyz[2])],
                "gripper_close_mm": float(grip_mm),
                "gripper_open_mm": float(grip_open_mm),
            }
        try:
            os.makedirs(os.path.dirname(MENU_OFFSET_CONFIG_PATH), exist_ok=True)
            with open(MENU_OFFSET_CONFIG_PATH, "w", encoding="utf-8") as fp:
                json.dump(payload, fp, ensure_ascii=False, indent=2)
            return True, "저장 완료"
        except Exception as exc:
            return False, f"저장 실패: {exc}"

    def _refresh_menu_xyz_offset_button_text(self):
        btn = getattr(self, "_bartender_offset_button", None)
        if btn is None:
            return
        active_count = 0
        for xyz in self._menu_xyz_offsets_by_code.values():
            try:
                if any(abs(float(v)) > 1e-9 for v in xyz):
                    active_count += 1
            except Exception:
                continue
        btn.setText(f"술 종류별 XYZ 오프셋 설정 ({active_count}개 적용)")

    def _get_menu_xyz_offset(self, menu_code: str):
        code = str(menu_code or "").strip()
        xyz = self._menu_xyz_offsets_by_code.get(code, (0.0, 0.0, 0.0))
        try:
            return (float(xyz[0]), float(xyz[1]), float(xyz[2]))
        except Exception:
            return (0.0, 0.0, 0.0)

    def _get_menu_gripper_close_mm(self, menu_code: str):
        code = str(menu_code or "").strip()
        try:
            value = float(self._menu_gripper_close_mm_by_code.get(code, 41.0))
            if (not np.isfinite(value)) or value < 0.0:
                return 41.0
            return float(value)
        except Exception:
            return 41.0

    def _get_menu_gripper_open_mm(self, menu_code: str):
        code = str(menu_code or "").strip()
        try:
            value = float(self._menu_gripper_open_mm_by_code.get(code, ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT))
            if (not np.isfinite(value)) or value < 0.0:
                return float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT)
            return min(float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT), float(value))
        except Exception:
            return float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT)

    def _build_menu_offset_payload(self):
        payload = {"menus": {}}
        for code in self._menu_codes_for_offset_ui():
            xyz = self._get_menu_xyz_offset(code)
            payload["menus"][str(code)] = {
                "label": str(self._menu_label_by_code.get(code, code) or code),
                "offset_xyz_mm": [float(xyz[0]), float(xyz[1]), float(xyz[2])],
                "gripper_close_mm": float(self._get_menu_gripper_close_mm(code)),
                "gripper_open_mm": float(self._get_menu_gripper_open_mm(code)),
            }
        return payload

    def _open_menu_xyz_offset_dialog(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("술 종류별 XYZ 오프셋 설정")
        dialog.setModal(True)
        dialog.resize(640, 340)

        root = QVBoxLayout(dialog)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(8)

        info = QLabel(
            "비전 좌표를 로봇 좌표계로 변환한 기준값에 메뉴별 XYZ 오프셋(mm)을 더하고,\n"
            "병 파지 단계에서 사용할 그리퍼 파지/오픈 거리(mm)도 메뉴별로 지정합니다.",
            dialog,
        )
        info.setWordWrap(True)
        info.setStyleSheet("font-size: 9pt; color: #334155;")
        root.addWidget(info)

        table = QTableWidget(dialog)
        codes = self._menu_codes_for_offset_ui()
        table.setColumnCount(7)
        table.setRowCount(len(codes))
        table.setHorizontalHeaderLabels(["메뉴코드", "메뉴명", "X(mm)", "Y(mm)", "Z(mm)", "파지(mm)", "오픈(mm)"])
        table.verticalHeader().setVisible(False)
        table.setSelectionMode(QTableWidget.NoSelection)
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        for col in (2, 3, 4, 5, 6):
            table.horizontalHeader().setSectionResizeMode(col, QHeaderView.ResizeToContents)

        for row, code in enumerate(codes):
            label = str(self._menu_label_by_code.get(code, code) or code)
            xyz = self._get_menu_xyz_offset(code)

            code_item = QTableWidgetItem(code)
            code_item.setFlags(code_item.flags() & ~Qt.ItemIsEditable)
            table.setItem(row, 0, code_item)

            label_item = QTableWidgetItem(label)
            table.setItem(row, 1, label_item)

            for col, value in enumerate(xyz, start=2):
                table.setItem(row, col, QTableWidgetItem(f"{float(value):.2f}"))
            table.setItem(row, 5, QTableWidgetItem(f"{float(self._get_menu_gripper_close_mm(code)):.2f}"))
            table.setItem(row, 6, QTableWidgetItem(f"{float(self._get_menu_gripper_open_mm(code)):.2f}"))

        root.addWidget(table, 1)

        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel, Qt.Horizontal, dialog)
        reset_btn = QPushButton("0으로 초기화", dialog)
        buttons.addButton(reset_btn, QDialogButtonBox.ActionRole)
        root.addWidget(buttons)

        def _on_reset():
            for r in range(table.rowCount()):
                for c in (2, 3, 4):
                    item = table.item(r, c)
                    if item is None:
                        item = QTableWidgetItem("0.00")
                        table.setItem(r, c, item)
                    else:
                        item.setText("0.00")
                grip_item = table.item(r, 5)
                if grip_item is None:
                    grip_item = QTableWidgetItem("41.00")
                    table.setItem(r, 5, grip_item)
                else:
                    grip_item.setText("41.00")
                grip_open_item = table.item(r, 6)
                if grip_open_item is None:
                    grip_open_item = QTableWidgetItem(f"{float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT):.2f}")
                    table.setItem(r, 6, grip_open_item)
                else:
                    grip_open_item.setText(f"{float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT):.2f}")

        def _on_save():
            new_offsets = {}
            new_labels = {}
            new_gripper = {}
            new_gripper_open = {}
            for r in range(table.rowCount()):
                code_item = table.item(r, 0)
                code = str(code_item.text() if code_item is not None else "").strip()
                if not code:
                    continue
                label_item = table.item(r, 1)
                label = str(label_item.text() if label_item is not None else "").strip() or code
                vals = []
                for c in (2, 3, 4):
                    item = table.item(r, c)
                    txt = str(item.text() if item is not None else "").strip() or "0"
                    try:
                        v = float(txt)
                    except Exception:
                        QMessageBox.warning(self, "오프셋 설정", f"{code}의 오프셋 값이 숫자가 아닙니다: {txt}")
                        return
                    if not np.isfinite(v):
                        QMessageBox.warning(self, "오프셋 설정", f"{code}의 오프셋 값이 유효하지 않습니다: {txt}")
                        return
                    vals.append(float(v))
                grip_item = table.item(r, 5)
                grip_txt = str(grip_item.text() if grip_item is not None else "").strip() or "41"
                try:
                    grip_v = float(grip_txt)
                except Exception:
                    QMessageBox.warning(self, "오프셋 설정", f"{code}의 파지(mm) 값이 숫자가 아닙니다: {grip_txt}")
                    return
                if (not np.isfinite(grip_v)) or grip_v < 0.0:
                    QMessageBox.warning(self, "오프셋 설정", f"{code}의 파지(mm) 값이 유효하지 않습니다: {grip_txt}")
                    return
                grip_open_item = table.item(r, 6)
                grip_open_txt = str(
                    grip_open_item.text() if grip_open_item is not None else f"{float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT):.2f}"
                ).strip() or str(float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT))
                try:
                    grip_open_v = float(grip_open_txt)
                except Exception:
                    QMessageBox.warning(self, "오프셋 설정", f"{code}의 오픈(mm) 값이 숫자가 아닙니다: {grip_open_txt}")
                    return
                if (not np.isfinite(grip_open_v)) or grip_open_v < 0.0:
                    QMessageBox.warning(self, "오프셋 설정", f"{code}의 오픈(mm) 값이 유효하지 않습니다: {grip_open_txt}")
                    return
                grip_open_v = min(float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT), float(grip_open_v))
                new_labels[code] = label
                new_offsets[code] = (vals[0], vals[1], vals[2])
                new_gripper[code] = float(grip_v)
                new_gripper_open[code] = float(grip_open_v)

            preserved_labels = {}
            for code, label in self._menu_label_by_code.items():
                if str(code).strip().lower() in BARTENDER_MENU_OFFSET_EXCLUDED_CODES_NORM:
                    preserved_labels[str(code)] = str(label)
            self._menu_label_by_code = dict(new_labels)
            self._menu_label_by_code.update(preserved_labels)
            self._menu_xyz_offsets_by_code = dict(new_offsets)
            self._menu_gripper_close_mm_by_code = dict(new_gripper)
            self._menu_gripper_open_mm_by_code = dict(new_gripper_open)
            ok, msg = self._save_menu_xyz_offsets()
            self._refresh_menu_xyz_offset_button_text()
            if not ok:
                QMessageBox.warning(self, "오프셋 설정", msg)
                return
            self._append_voice_order_log(f"메뉴별 XYZ/파지/오픈(mm) 설정 저장: {MENU_OFFSET_CONFIG_PATH}")
            dialog.accept()

        reset_btn.clicked.connect(_on_reset)
        buttons.accepted.connect(_on_save)
        buttons.rejected.connect(dialog.reject)

        dialog.exec_()

    def _clone_default_robot_action_pose_config(self):
        payload = {
            "poses": {},
            "offsets_xyz_mm": {},
            "gripper": {"open_mm_default": float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT)},
        }
        poses = ROBOT_ACTION_POSE_DEFAULTS.get("poses", {})
        offsets = ROBOT_ACTION_POSE_DEFAULTS.get("offsets_xyz_mm", {})
        if isinstance(poses, dict):
            for key, values in poses.items():
                try:
                    vals = [float(v) for v in list(values)[:6]]
                except Exception:
                    vals = [0.0] * 6
                if len(vals) < 6:
                    vals = vals + [0.0] * (6 - len(vals))
                payload["poses"][str(key)] = vals[:6]
        if isinstance(offsets, dict):
            for key, values in offsets.items():
                try:
                    vals = [float(v) for v in list(values)[:3]]
                except Exception:
                    vals = [0.0, 0.0, 0.0]
                if len(vals) < 3:
                    vals = vals + [0.0] * (3 - len(vals))
                payload["offsets_xyz_mm"][str(key)] = vals[:3]
        return payload

    def _sanitize_robot_action_gripper_open_mm(self, value, default_mm: float = ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT):
        try:
            v = float(value)
        except Exception:
            v = float(default_mm)
        if not np.isfinite(v):
            v = float(default_mm)
        return max(0.0, min(float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT), float(v)))

    def _extract_robot_action_gripper_open_mm(self, config_payload):
        default_v = float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT)
        if not isinstance(config_payload, dict):
            return default_v
        raw = None
        gripper = config_payload.get("gripper", {})
        if isinstance(gripper, dict):
            raw = gripper.get("open_mm_default", gripper.get("gripper_open_mm_default"))
        if raw is None:
            raw = config_payload.get("gripper_open_mm_default", None)
        return float(self._sanitize_robot_action_gripper_open_mm(raw, default_mm=default_v))

    def _sanitize_robot_action_pose_config(self, raw_config):
        sanitized = self._clone_default_robot_action_pose_config()
        if not isinstance(raw_config, dict):
            return sanitized

        poses_raw = raw_config.get("poses", {})
        if isinstance(poses_raw, dict):
            normalized_poses_raw = {}
            has_legacy_shared_ready = False
            for raw_key, raw_vals in poses_raw.items():
                raw_key_txt = str(raw_key or "").strip()
                if raw_key_txt in ("joint_pick_ready", "joint_pick_place_ready", "joint_service_ready"):
                    has_legacy_shared_ready = True
                canonical_key = _canonical_robot_action_pose_key(raw_key)
                if not canonical_key:
                    continue
                if (
                    canonical_key not in normalized_poses_raw
                    or str(raw_key).strip() == canonical_key
                ):
                    normalized_poses_raw[canonical_key] = raw_vals
            for key in list(sanitized["poses"].keys()):
                raw_vals = normalized_poses_raw.get(key)
                if not isinstance(raw_vals, (list, tuple)) or len(raw_vals) < 6:
                    continue
                parsed = []
                valid = True
                for idx in range(6):
                    try:
                        v = float(raw_vals[idx])
                    except Exception:
                        valid = False
                        break
                    if not np.isfinite(v):
                        valid = False
                        break
                    parsed.append(float(v))
                if valid and len(parsed) == 6:
                    sanitized["poses"][key] = list(parsed)
            if has_legacy_shared_ready:
                ready_vals = sanitized["poses"].get("service_ready_posj")
                if isinstance(ready_vals, list) and len(ready_vals) >= 6:
                    ready_copy = [float(v) for v in list(ready_vals)[:6]]
                    for inherit_key in ("cup_pick_ready_posj", "cup_delivery_ready_posj"):
                        raw_inherit = normalized_poses_raw.get(inherit_key)
                        if isinstance(raw_inherit, (list, tuple)) and len(raw_inherit) >= 6:
                            continue
                        sanitized["poses"][inherit_key] = list(ready_copy)

            # 구버전 단일 키를 재료별 키(soju/beer)로 자동 승계한다.
            shared_pose_suffixes = (
                "service_ready_posj",
                "pick_lift_out_posx",
                "pour_start_cheers_posj",
                "pour_contact_posj",
                "pour_horizontal_posj",
                "pour_diagonal_posj",
                "pour_vertical_posj",
            )
            for suffix in shared_pose_suffixes:
                shared_key = str(suffix)
                shared_vals = sanitized["poses"].get(shared_key)
                if not isinstance(shared_vals, list) or len(shared_vals) < 6:
                    continue
                shared_copy = [float(v) for v in list(shared_vals)[:6]]
                for ingredient_code in ROBOT_ACTION_LIQUID_INGREDIENT_CODES:
                    ingredient_key = f"{ingredient_code}_{suffix}"
                    raw_specific = normalized_poses_raw.get(ingredient_key)
                    if isinstance(raw_specific, (list, tuple)) and len(raw_specific) >= 6:
                        continue
                    sanitized["poses"][ingredient_key] = list(shared_copy)

        offsets_raw = raw_config.get("offsets_xyz_mm", {})
        if isinstance(offsets_raw, dict):
            normalized_offsets_raw = {}
            for raw_key, raw_vals in offsets_raw.items():
                canonical_key = _canonical_robot_action_offset_key(raw_key)
                if not canonical_key:
                    continue
                if (
                    canonical_key not in normalized_offsets_raw
                    or str(raw_key).strip() == canonical_key
                ):
                    normalized_offsets_raw[canonical_key] = raw_vals
            for key in list(sanitized["offsets_xyz_mm"].keys()):
                raw_vals = normalized_offsets_raw.get(key)
                if not isinstance(raw_vals, (list, tuple)) or len(raw_vals) < 3:
                    continue
                parsed = []
                valid = True
                for idx in range(3):
                    try:
                        v = float(raw_vals[idx])
                    except Exception:
                        valid = False
                        break
                    if not np.isfinite(v):
                        valid = False
                        break
                    parsed.append(float(v))
                if valid and len(parsed) == 3:
                    sanitized["offsets_xyz_mm"][key] = list(parsed)

            # 구버전 단일 오프셋 키를 재료별 키(soju/beer)로 자동 승계한다.
            shared_offset_suffixes = (
                "pick_approach_offset",
                "pick_grasp_offset",
                "pick_lift_offset",
                "place_offset",
                "retreat_offset",
            )
            for suffix in shared_offset_suffixes:
                shared_key = str(suffix)
                shared_vals = sanitized["offsets_xyz_mm"].get(shared_key)
                if not isinstance(shared_vals, list) or len(shared_vals) < 3:
                    continue
                shared_copy = [float(v) for v in list(shared_vals)[:3]]
                for ingredient_code in ROBOT_ACTION_LIQUID_INGREDIENT_CODES:
                    ingredient_key = f"{ingredient_code}_{suffix}"
                    raw_specific = normalized_offsets_raw.get(ingredient_key)
                    if isinstance(raw_specific, (list, tuple)) and len(raw_specific) >= 3:
                        continue
                    sanitized["offsets_xyz_mm"][ingredient_key] = list(shared_copy)

        gripper_raw = raw_config.get("gripper", {})
        open_raw = None
        if isinstance(gripper_raw, dict):
            open_raw = gripper_raw.get("open_mm_default", gripper_raw.get("gripper_open_mm_default"))
        if open_raw is None:
            open_raw = raw_config.get("gripper_open_mm_default", None)
        if not isinstance(sanitized.get("gripper"), dict):
            sanitized["gripper"] = {}
        sanitized["gripper"]["open_mm_default"] = float(
            self._sanitize_robot_action_gripper_open_mm(open_raw, default_mm=ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT)
        )
        return sanitized

    def _load_robot_action_pose_config(self):
        cfg = self._clone_default_robot_action_pose_config()
        try:
            if os.path.isfile(ROBOT_ACTION_POSE_CONFIG_PATH):
                with open(ROBOT_ACTION_POSE_CONFIG_PATH, "r", encoding="utf-8") as fp:
                    loaded = json.load(fp)
                cfg = self._sanitize_robot_action_pose_config(loaded)
        except Exception:
            cfg = self._clone_default_robot_action_pose_config()
        return cfg

    def _save_robot_action_pose_config(self, config_payload):
        payload = self._sanitize_robot_action_pose_config(config_payload)
        payload["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        try:
            os.makedirs(os.path.dirname(ROBOT_ACTION_POSE_CONFIG_PATH), exist_ok=True)
            with open(ROBOT_ACTION_POSE_CONFIG_PATH, "w", encoding="utf-8") as fp:
                json.dump(payload, fp, ensure_ascii=False, indent=2)
            return True, "저장 완료"
        except Exception as exc:
            return False, f"저장 실패: {exc}"

    def _get_robot_action_pose_entry(self, config_payload, row_def):
        group = str(row_def.get("config_group", "") or "").strip()
        key = str(row_def.get("config_key", "") or "").strip()
        row_type = str(row_def.get("type", "") or "").strip().lower()
        if not group or not key:
            return None
        data = config_payload.get(group, {})
        if not isinstance(data, dict):
            return None
        if row_type == "global_gripper_open_mm":
            val = self._extract_robot_action_gripper_open_mm(config_payload)
            return [float(val)]
        values = data.get(key)
        if row_type in ("posj", "posx"):
            parsed = self._parse_float_values(values, expected_len=6)
            return list(parsed) if parsed is not None else None
        if row_type == "vision_offset":
            parsed = self._parse_float_values(values, expected_len=3)
            return list(parsed) if parsed is not None else None
        return None

    def _set_robot_action_pose_entry(self, config_payload, row_def, values):
        group = str(row_def.get("config_group", "") or "").strip()
        key = str(row_def.get("config_key", "") or "").strip()
        row_type = str(row_def.get("type", "") or "").strip().lower()
        if not group or not key:
            return False, "설정 키가 없습니다."
        if group not in ("poses", "offsets_xyz_mm", "gripper"):
            return False, "설정 그룹이 올바르지 않습니다."
        if not isinstance(config_payload.get(group), dict):
            config_payload[group] = {}
        if row_type == "global_gripper_open_mm":
            parsed = self._parse_float_values(values, expected_len=1)
            if parsed is None or len(parsed) < 1:
                return False, f"{key} 값이 올바르지 않습니다."
            v = float(self._sanitize_robot_action_gripper_open_mm(parsed[0], default_mm=ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT))
            config_payload[group][key] = float(v)
            return True, "ok"
        expected_len = 6 if row_type in ("posj", "posx") else 3
        parsed = self._parse_float_values(values, expected_len=expected_len)
        if parsed is None:
            return False, f"{key} 값 길이가 올바르지 않습니다."
        for v in parsed:
            if not np.isfinite(float(v)):
                return False, f"{key} 값이 유효하지 않습니다."
        config_payload[group][key] = [float(v) for v in list(parsed)[:expected_len]]
        return True, "ok"

    def _format_robot_action_pose_row_value(self, row_def, config_payload):
        row_type = str(row_def.get("type", "") or "").strip().lower()
        values = self._get_robot_action_pose_entry(config_payload, row_def)
        if row_type == "posj":
            return self._format_joint_summary(values)
        if row_type == "posx":
            return self._format_pose6_summary(values)
        if row_type == "vision_offset":
            return self._format_xyz_summary(values)
        if row_type == "vision_target":
            return "실시간 계산값(보기 전용)"
        if row_type == "gripper_open_action":
            ingredient_code = str(row_def.get("ingredient_code", "") or "").strip().lower()
            open_mm = (
                float(self._get_menu_gripper_open_mm(ingredient_code))
                if ingredient_code
                else float(self._extract_robot_action_gripper_open_mm(config_payload))
            )
            return f"{open_mm:.1f}mm"
        if row_type == "gripper_close_action":
            ingredient_code = str(row_def.get("ingredient_code", "") or "").strip().lower()
            if ingredient_code:
                close_mm = float(self._get_menu_gripper_close_mm(ingredient_code))
                ingredient_label = str(BARTENDER_MENU_LABELS.get(ingredient_code, ingredient_code) or ingredient_code)
                return f"대상={ingredient_label}({ingredient_code}), 파지={close_mm:.1f}mm"
            return "대상 선택 필요(클릭 시 선택)"
        if row_type == "gripper_actions":
            ingredient_code = str(row_def.get("ingredient_code", "") or "").strip().lower()
            close_mm = float(self._get_menu_gripper_close_mm(ingredient_code)) if ingredient_code else 41.0
            open_mm = (
                float(self._get_menu_gripper_open_mm(ingredient_code))
                if ingredient_code
                else float(self._extract_robot_action_gripper_open_mm(config_payload))
            )
            return f"오픈={open_mm:.1f}mm / 파지={float(close_mm):.1f}mm"
        if row_type == "menu_gripper_open_mm":
            ingredient_code = str(row_def.get("ingredient_code", "") or "").strip().lower()
            open_mm = float(self._get_menu_gripper_open_mm(ingredient_code))
            return f"{open_mm:.1f}mm"
        if row_type == "global_gripper_open_mm":
            open_mm = float(self._extract_robot_action_gripper_open_mm(config_payload))
            return f"{open_mm:.1f}mm"
        if row_type == "menu_offset_xyz":
            ingredient_code = str(row_def.get("ingredient_code", "") or "").strip().lower()
            xyz = self._get_menu_xyz_offset(ingredient_code)
            return self._format_xyz_summary(list(xyz))
        if row_type == "menu_gripper_close_mm":
            ingredient_code = str(row_def.get("ingredient_code", "") or "").strip().lower()
            close_mm = float(self._get_menu_gripper_close_mm(ingredient_code))
            return f"{close_mm:.1f}mm"
        return "-"

    def _open_robot_action_pose_dialog(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("로봇 액션 포지션 설정")
        dialog.setModal(True)
        dialog.setWindowFlags(Qt.Window | Qt.WindowTitleHint | Qt.WindowSystemMenuHint | Qt.WindowCloseButtonHint)
        dialog.resize(1280, 760)

        root = QVBoxLayout(dialog)
        root.setContentsMargins(14, 12, 14, 12)
        root.setSpacing(8)

        intro = QLabel(
            (
                "robot_action_planner 시퀀스 설정 화면입니다.\n"
                "상단 탭(소주/맥주/글라스잔)으로 항목을 전환할 수 있습니다.\n"
                "값 칸 클릭 수정은 지원하지 않습니다. 필요한 항목은 작업 버튼으로만 제어합니다.\n"
                "비전 오프셋 이동은 해당 탭의 [비전 타겟 계산]에서 [위치받아오기] 이후에만 가능합니다."
            ),
            dialog,
        )
        intro.setWordWrap(True)
        root.addWidget(intro)

        status_label = QLabel("대기: 필요한 항목을 작업 버튼으로 제어하세요.", dialog)
        status_label.setStyleSheet("font-weight: 700; color: #1f2937;")
        status_label.setWordWrap(True)
        root.addWidget(status_label)

        tab_widget = QTabWidget(dialog)
        tab_widget.setDocumentMode(True)
        tab_widget.setElideMode(Qt.ElideRight)
        root.addWidget(tab_widget, 1)

        action_row = QHBoxLayout()
        action_row.setSpacing(8)
        refresh_btn = QPushButton("새로고침", dialog)
        save_btn = QPushButton("저장", dialog)
        close_btn = QPushButton("닫기", dialog)
        action_row.addStretch(1)
        action_row.addWidget(refresh_btn)
        action_row.addWidget(save_btn)
        action_row.addWidget(close_btn)
        root.addLayout(action_row)

        loaded_saved_cfg = self._load_robot_action_pose_config()
        saved_cfg = self._sanitize_robot_action_pose_config(loaded_saved_cfg)
        draft_cfg = self._sanitize_robot_action_pose_config(saved_cfg)
        self._robot_action_pose_config = self._sanitize_robot_action_pose_config(saved_cfg)

        saved_menu_offsets = dict(self._menu_xyz_offsets_by_code)
        saved_menu_gripper = dict(self._menu_gripper_close_mm_by_code)
        saved_menu_gripper_open = dict(self._menu_gripper_open_mm_by_code)
        saved_menu_labels = dict(self._menu_label_by_code)
        draft_menu_offsets = dict(saved_menu_offsets)
        draft_menu_gripper = dict(saved_menu_gripper)
        draft_menu_gripper_open = dict(saved_menu_gripper_open)
        draft_menu_labels = dict(saved_menu_labels)

        rows_by_tab = {}
        for tab_code, _tab_label in ROBOT_ACTION_POSE_TABS:
            rows_by_tab[str(tab_code)] = []
        for row_def in list(ROBOT_ACTION_POSE_ROW_DEFS):
            tab_code = str(row_def.get("tab", "soju") or "soju").strip().lower()
            if tab_code not in rows_by_tab:
                rows_by_tab[tab_code] = []
            rows_by_tab[tab_code].append(dict(row_def))

        row_states = []
        table_row_states = {}
        tables_by_tab = {}
        save_state = {"saving": False, "done": False, "ok": False, "msg": "", "thread": None}
        vision_target_state = {
            "payload_by_code": dict(
                self._robot_action_cached_vision_target_payload_by_code
                if isinstance(self._robot_action_cached_vision_target_payload_by_code, dict)
                else {}
            ),
            "last_code": str(self._robot_action_cached_vision_target_last_code or "").strip().lower(),
            "last_label": str(self._robot_action_cached_vision_target_last_label or "").strip(),
        }
        gripper_target_state = {
            "ingredient_code": "",
            "ingredient_label": "",
        }
        runtime_caps = {
            "can_move_posj": False,
            "can_move_posx": False,
            "can_move_vision": False,
            "can_move_gripper": False,
            "can_teach_pose": False,
        }
        row_selection_state = {
            "table": None,
            "row_index": -1,
            "var_key": "",
        }

        def _set_status(text: str):
            status_label.setText(str(text))

        def _normalized_var_key(raw_name):
            txt = str(raw_name or "").strip()
            if not txt:
                return ""
            txt = re.sub(r"\s*\(return_ref\)\s*$", "", txt, flags=re.IGNORECASE)
            return txt.strip().lower()

        def _selection_var_key(row_def):
            row_type = str(row_def.get("type", "") or "").strip().lower()
            ingredient_code = str(row_def.get("ingredient_code", "") or "").strip().lower()
            if row_type in ("menu_gripper_open_mm", "gripper_open_action"):
                if ingredient_code:
                    return f"{ingredient_code}::gripper_open_mm"
                return "gripper_open_mm"
            if row_type in ("menu_gripper_close_mm", "gripper_close_action"):
                if ingredient_code:
                    return f"{ingredient_code}::gripper_close_mm"
                return "gripper_close_mm"
            return _normalized_var_key(row_def.get("var_name", ""))

        def _display_var_name(raw_name):
            txt = _normalized_var_key(raw_name)
            if not txt:
                return "-"
            return txt

        def _display_row_type(raw_type):
            key = str(raw_type or "").strip().lower()
            if not key:
                return "-"
            short_map = {
                "posj": "posj",
                "posx": "posx",
                "vision_target": "v_target",
                "vision_offset": "v_offset",
                "menu_offset_xyz": "m_xyz",
                "menu_gripper_close_mm": "g_close_mm",
                "menu_gripper_open_mm": "g_open_mm",
                "gripper_open_action": "g_open",
                "gripper_close_action": "g_close",
            }
            return str(short_map.get(key, key))

        def _apply_row_bg_color(row_state, color_hex: str):
            table_ref = row_state.get("table")
            row_index = int(row_state.get("index", -1))
            if table_ref is None or row_index < 0:
                return
            brush = QBrush(QColor(color_hex)) if color_hex else QBrush()
            for col_index in (0, 1, 2, 3, 4, 6, 7):
                item = table_ref.item(row_index, col_index)
                if item is not None:
                    item.setBackground(brush)
            button_wrap = row_state.get("button_wrap")
            if button_wrap is not None:
                if color_hex:
                    button_wrap.setStyleSheet(f"QFrame {{ background-color: {color_hex}; border-radius: 4px; }}")
                else:
                    button_wrap.setStyleSheet("")

        def _refresh_row_selection_highlight():
            selected_table = row_selection_state.get("table")
            try:
                selected_index = int(row_selection_state.get("row_index", -1))
            except Exception:
                selected_index = -1
            selected_key = str(row_selection_state.get("var_key", "") or "").strip().lower()

            for state in row_states:
                state_table = state.get("table")
                state_index = int(state.get("index", -1))
                state_key = str(state.get("var_key", "") or "").strip().lower()
                base_bg_hex = str(state.get("base_bg_hex", "") or "")

                if state_table is selected_table and state_index == selected_index:
                    _apply_row_bg_color(state, "#d7ebff")
                    continue
                if state_table is selected_table and selected_key and state_key == selected_key:
                    _apply_row_bg_color(state, "#eaf4ff")
                    continue
                _apply_row_bg_color(state, base_bg_hex)

        def _set_row_selected(row_state):
            row_selection_state["table"] = row_state.get("table")
            row_selection_state["row_index"] = int(row_state.get("index", -1))
            row_selection_state["var_key"] = str(row_state.get("var_key", "") or "").strip().lower()
            _refresh_row_selection_highlight()

        def _persist_vision_target_state():
            payload_by_code = vision_target_state.get("payload_by_code", {})
            if isinstance(payload_by_code, dict):
                self._robot_action_cached_vision_target_payload_by_code = dict(payload_by_code)
            else:
                self._robot_action_cached_vision_target_payload_by_code = {}
            self._robot_action_cached_vision_target_last_code = str(
                vision_target_state.get("last_code", "") or ""
            ).strip().lower()
            self._robot_action_cached_vision_target_last_label = str(
                vision_target_state.get("last_label", "") or ""
            ).strip()

        def _menu_code_from_row(row_def, *, allow_glass: bool = False):
            code = str(row_def.get("ingredient_code", "") or "").strip().lower()
            if not code:
                return ""
            if allow_glass:
                return code
            if code in ROBOT_ACTION_LIQUID_INGREDIENT_CODES:
                return code
            return ""

        def _get_draft_menu_xyz(code: str):
            c = str(code or "").strip().lower()
            raw = draft_menu_offsets.get(c, (0.0, 0.0, 0.0))
            try:
                return [float(raw[0]), float(raw[1]), float(raw[2])]
            except Exception:
                return [0.0, 0.0, 0.0]

        def _set_draft_menu_xyz(code: str, xyz):
            c = str(code or "").strip().lower()
            if not c:
                return
            try:
                x = float(xyz[0]) if isinstance(xyz, (list, tuple)) and len(xyz) >= 1 else 0.0
                y = float(xyz[1]) if isinstance(xyz, (list, tuple)) and len(xyz) >= 2 else 0.0
                z = float(xyz[2]) if isinstance(xyz, (list, tuple)) and len(xyz) >= 3 else 0.0
            except Exception:
                x, y, z = 0.0, 0.0, 0.0
            draft_menu_offsets[c] = (float(x), float(y), float(z))
            if c not in draft_menu_labels:
                draft_menu_labels[c] = str(BARTENDER_MENU_LABELS.get(c, c) or c)

        def _get_draft_menu_gripper(code: str):
            c = str(code or "").strip().lower()
            try:
                value = float(draft_menu_gripper.get(c, 41.0))
                if (not np.isfinite(value)) or value < 0.0:
                    return 41.0
                return float(value)
            except Exception:
                return 41.0

        def _set_draft_menu_gripper(code: str, value_mm: float):
            c = str(code or "").strip().lower()
            if not c:
                return
            try:
                value = float(value_mm)
            except Exception:
                value = 41.0
            if (not np.isfinite(value)) or value < 0.0:
                value = 41.0
            draft_menu_gripper[c] = float(value)
            if c not in draft_menu_labels:
                draft_menu_labels[c] = str(BARTENDER_MENU_LABELS.get(c, c) or c)

        def _get_draft_menu_gripper_open(code: str):
            c = str(code or "").strip().lower()
            try:
                value = float(draft_menu_gripper_open.get(c, ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT))
            except Exception:
                value = float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT)
            if (not np.isfinite(value)) or value < 0.0:
                value = float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT)
            return min(float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT), float(value))

        def _set_draft_menu_gripper_open(code: str, value_mm: float):
            c = str(code or "").strip().lower()
            if not c:
                return
            try:
                value = float(value_mm)
            except Exception:
                value = float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT)
            if (not np.isfinite(value)) or value < 0.0:
                value = float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT)
            value = min(float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT), float(value))
            draft_menu_gripper_open[c] = float(value)
            if c not in draft_menu_labels:
                draft_menu_labels[c] = str(BARTENDER_MENU_LABELS.get(c, c) or c)

        def _vision_payload_for_row(row_def):
            row_code = _menu_code_from_row(row_def, allow_glass=False)
            if row_code:
                payload = vision_target_state["payload_by_code"].get(row_code)
                return payload if isinstance(payload, dict) else None
            fallback_code = str(vision_target_state.get("last_code", "") or "").strip().lower()
            payload = vision_target_state["payload_by_code"].get(fallback_code)
            return payload if isinstance(payload, dict) else None

        def _is_row_editable(row_state):
            if not bool(row_state["def"].get("editable", True)):
                return False
            row_type = str(row_state["def"].get("type", "") or "").strip().lower()
            return row_type in (
                "posj",
                "posx",
                "vision_offset",
                "menu_offset_xyz",
                "menu_gripper_close_mm",
                "menu_gripper_open_mm",
                "global_gripper_open_mm",
            )

        def _is_reference_row(row_def):
            return bool(row_def.get("reference_only", False))

        def _display_row_desc(row_def):
            if _is_reference_row(row_def):
                return "위치참조"
            return str(row_def.get("desc", "") or "")

        def _refresh_runtime_caps():
            backend = self.backend
            backend_ready = False
            if backend is not None:
                try:
                    backend_ready = bool(backend.is_ready()) if hasattr(backend, "is_ready") else True
                except Exception:
                    backend_ready = False

            can_move_posj = bool(backend_ready and hasattr(backend, "send_move_joint"))
            can_move_posx = bool(backend_ready and hasattr(backend, "send_move_cartesian"))
            can_move_vision = bool(backend_ready and hasattr(backend, "get_robot_action_vision_target_xyz"))
            can_move_gripper = bool(backend_ready and hasattr(backend, "send_gripper_move"))
            mode_ok_for_move = True
            mode_value = None
            mode_seen_at = None
            if backend is not None and hasattr(backend, "get_robot_mode_snapshot"):
                try:
                    mode_value, mode_seen_at = backend.get_robot_mode_snapshot()
                except Exception:
                    mode_value, mode_seen_at = None, None
            if mode_value is None or mode_seen_at is None:
                mode_ok_for_move = False
            else:
                try:
                    mode_ok_for_move = int(mode_value) == 1
                except Exception:
                    mode_ok_for_move = False
            can_move_posj = bool(can_move_posj and mode_ok_for_move)
            can_move_posx = bool(can_move_posx and mode_ok_for_move)
            can_move_vision = bool(can_move_vision and mode_ok_for_move)
            can_move_gripper = bool(can_move_gripper and mode_ok_for_move)
            can_teach_pose = bool(
                backend_ready
                and mode_ok_for_move
                and backend is not None
                and (hasattr(backend, "get_position_snapshot") or hasattr(backend, "get_current_posx_live"))
            )
            runtime_caps["can_move_posj"] = can_move_posj
            runtime_caps["can_move_posx"] = can_move_posx
            runtime_caps["can_move_vision"] = can_move_vision
            runtime_caps["can_move_gripper"] = can_move_gripper
            runtime_caps["can_teach_pose"] = can_teach_pose

        def _is_row_teachable(row_state):
            row_type = str(row_state["def"].get("type", "") or "").strip().lower()
            return (
                bool(row_state["def"].get("editable", True))
                and row_type in ("posj", "posx")
                and bool(runtime_caps.get("can_teach_pose"))
            )

        def _is_row_movable(row_state):
            row_def = row_state["def"]
            row_type = str(row_def.get("type", "") or "").strip().lower()
            if row_type == "posj":
                return bool(runtime_caps.get("can_move_posj"))
            if row_type == "posx":
                return bool(runtime_caps.get("can_move_posx"))
            if row_type == "vision_target":
                row_code = _menu_code_from_row(row_def, allow_glass=False)
                return bool(runtime_caps.get("can_move_vision")) and bool(row_code)
            if row_type == "vision_offset":
                return bool(runtime_caps.get("can_move_vision")) and isinstance(_vision_payload_for_row(row_def), dict)
            if row_type in ("gripper_open_action", "gripper_close_action"):
                return bool(runtime_caps.get("can_move_gripper"))
            if row_type == "gripper_actions":
                return bool(runtime_caps.get("can_move_gripper"))
            return False

        def _refresh_row(row_state, status_text: str | None = None):
            row_def = row_state["def"]
            row_type = str(row_def.get("type", "") or "").strip().lower()
            value_item = row_state.get("value_item")
            state_item = row_state.get("state_item")
            row_code = _menu_code_from_row(row_def, allow_glass=True)
            row_label = str(row_def.get("label", row_def.get("var_name", "-")) or "-")

            if value_item is not None:
                if _is_reference_row(row_def):
                    value_item.setText("-")
                elif row_type == "vision_target":
                    payload = _vision_payload_for_row(row_def)
                    ingredient_label = str(BARTENDER_MENU_LABELS.get(row_code, row_code) or row_code) if row_code else "-"
                    if isinstance(payload, dict):
                        xyz = payload.get("resolved_robot_xyz_mm", [])
                        try:
                            tx = float(xyz[0])
                            ty = float(xyz[1])
                            tz = float(xyz[2])
                            value_item.setText(f"기준XYZ(mm): X={tx:.1f}, Y={ty:.1f}, Z={tz:.1f} ({ingredient_label})")
                        except Exception:
                            value_item.setText("기준XYZ 수신됨")
                    else:
                        value_item.setText("기준좌표 미수신(위치받아오기 필요)")
                elif row_type == "gripper_open_action":
                    ingredient_code = str(row_code or "").strip().lower()
                    open_mm = float(_get_draft_menu_gripper_open(ingredient_code))
                    value_item.setText(f"{open_mm:.1f}mm")
                elif row_type == "gripper_close_action":
                    ingredient_code = str(row_code or gripper_target_state.get("ingredient_code", "") or "").strip().lower()
                    ingredient_label = str(
                        BARTENDER_MENU_LABELS.get(ingredient_code, gripper_target_state.get("ingredient_label", ingredient_code))
                        or ingredient_code
                    ).strip()
                    if ingredient_code:
                        close_mm = float(_get_draft_menu_gripper(ingredient_code))
                        value_item.setText(f"대상={ingredient_label}({ingredient_code}), 파지={close_mm:.1f}mm")
                    else:
                        value_item.setText("대상 미선택(클릭 시 선택)")
                elif row_type == "gripper_actions":
                    ingredient_code = str(row_code or "").strip().lower()
                    ingredient_label = str(BARTENDER_MENU_LABELS.get(ingredient_code, ingredient_code) or ingredient_code)
                    close_mm = float(_get_draft_menu_gripper(ingredient_code)) if ingredient_code else 41.0
                    open_mm = float(_get_draft_menu_gripper_open(ingredient_code)) if ingredient_code else float(
                        ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT
                    )
                    value_item.setText(
                        f"대상={ingredient_label}({ingredient_code}), "
                        f"오픈={open_mm:.1f}mm / 파지={close_mm:.1f}mm"
                    )
                elif row_type == "menu_gripper_open_mm":
                    open_mm = float(_get_draft_menu_gripper_open(row_code))
                    value_item.setText(f"{open_mm:.1f}mm")
                elif row_type == "global_gripper_open_mm":
                    value_item.setText(f"{float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT):.1f}mm")
                elif row_type == "menu_offset_xyz":
                    xyz = _get_draft_menu_xyz(row_code)
                    value_item.setText(self._format_xyz_summary(xyz))
                elif row_type == "menu_gripper_close_mm":
                    close_mm = float(_get_draft_menu_gripper(row_code))
                    value_item.setText(f"{close_mm:.1f}mm")
                else:
                    value_item.setText(self._format_robot_action_pose_row_value(row_def, draft_cfg))

            if state_item is not None:
                if status_text is not None:
                    state_item.setText(str(status_text))
                elif _is_reference_row(row_def):
                    state_item.setText("참조")
                elif row_type == "vision_target":
                    state_item.setText("기준좌표 수신됨" if isinstance(_vision_payload_for_row(row_def), dict) else "기준좌표 필요")
                elif row_type == "vision_offset":
                    state_item.setText("저장됨" if isinstance(_vision_payload_for_row(row_def), dict) else "기준좌표 필요")
                elif row_type in ("menu_offset_xyz", "menu_gripper_close_mm", "menu_gripper_open_mm"):
                    state_item.setText("저장됨")
                elif row_type == "gripper_open_action":
                    state_item.setText("실행 가능")
                elif row_type == "gripper_close_action":
                    state_item.setText("실행 가능" if row_code else "대상 선택 필요")
                elif row_type == "gripper_actions":
                    state_item.setText("실행 가능" if row_code else "대상 선택 필요")
                elif row_type == "global_gripper_open_mm":
                    state_item.setText("저장됨")
                else:
                    state_item.setText("저장됨")

            move_btn = row_state.get("move_btn")
            teach_btn = row_state.get("teach_btn")
            edit_btn = row_state.get("edit_btn")
            extra_move_btns = row_state.get("extra_move_btns", [])
            if move_btn is not None:
                move_btn.setEnabled((not save_state["saving"]) and _is_row_movable(row_state))
            if teach_btn is not None:
                teach_btn.setEnabled((not save_state["saving"]) and _is_row_teachable(row_state))
            if edit_btn is not None:
                edit_btn.setEnabled((not save_state["saving"]) and _is_row_editable(row_state))
            for _btn in list(extra_move_btns):
                if _btn is None:
                    continue
                _btn.setEnabled((not save_state["saving"]) and _is_row_movable(row_state))

        def _refresh_all_rows(status_text: str | None = None):
            _refresh_runtime_caps()
            for row_state in row_states:
                _refresh_row(row_state, status_text=status_text if _is_row_editable(row_state) else None)
            _refresh_row_selection_highlight()

        def _default_vision_ingredient_code():
            result = self._voice_last_result if isinstance(self._voice_last_result, dict) else {}
            recipe = result.get("recipe", {})
            if isinstance(recipe, dict):
                for key, value in recipe.items():
                    code = str(key or "").strip().lower()
                    if code not in ROBOT_ACTION_LIQUID_INGREDIENT_CODES:
                        continue
                    try:
                        amount = float(value)
                    except Exception:
                        amount = 0.0
                    if amount > 0.0:
                        return code
            return ROBOT_ACTION_LIQUID_INGREDIENT_CODES[-1]

        def _ask_vision_ingredient(row_label: str, prompt_text: str = "비전 기준으로 사용할 재료를 선택하세요:"):
            options = []
            option_to_code = {}
            for code in ROBOT_ACTION_LIQUID_INGREDIENT_CODES:
                label = str(BARTENDER_MENU_LABELS.get(code, code) or code)
                item = f"{label} ({code})"
                options.append(item)
                option_to_code[item] = code
            if not options:
                return "", ""

            default_code = _default_vision_ingredient_code()
            default_index = 0
            for idx, item in enumerate(options):
                if option_to_code.get(item) == default_code:
                    default_index = idx
                    break
            selected_item, ok_select = QInputDialog.getItem(
                dialog,
                f"{row_label} 재료 선택",
                str(prompt_text or "재료를 선택하세요:"),
                options,
                int(default_index),
                False,
            )
            if not ok_select:
                return "", ""
            selected_code = str(option_to_code.get(str(selected_item), "") or "").strip().lower()
            selected_label = str(BARTENDER_MENU_LABELS.get(selected_code, selected_code) or selected_code)
            return selected_code, selected_label

        def _resolve_row_ingredient(row_def, row_label: str, prompt_text: str, *, allow_glass: bool = False):
            fixed_code = _menu_code_from_row(row_def, allow_glass=allow_glass)
            if fixed_code:
                fixed_label = str(BARTENDER_MENU_LABELS.get(fixed_code, fixed_code) or fixed_code)
                return fixed_code, fixed_label
            return _ask_vision_ingredient(row_label=row_label, prompt_text=prompt_text)

        def _build_draft_menu_offset_payload():
            payload = {"menus": {}}
            all_codes = set()
            all_codes.update(str(k).strip().lower() for k in draft_menu_offsets.keys())
            all_codes.update(str(k).strip().lower() for k in draft_menu_gripper.keys())
            all_codes.update(str(k).strip().lower() for k in draft_menu_gripper_open.keys())
            all_codes.update(str(k).strip().lower() for k in draft_menu_labels.keys())
            all_codes.update(str(k).strip().lower() for k in BARTENDER_MENU_LABELS.keys())
            for code in sorted(all_codes):
                if (not code) or code in BARTENDER_MENU_OFFSET_EXCLUDED_CODES_NORM:
                    continue
                xyz = _get_draft_menu_xyz(code)
                payload["menus"][code] = {
                    "label": str(draft_menu_labels.get(code, BARTENDER_MENU_LABELS.get(code, code)) or code),
                    "offset_xyz_mm": [float(xyz[0]), float(xyz[1]), float(xyz[2])],
                    "gripper_close_mm": float(_get_draft_menu_gripper(code)),
                    "gripper_open_mm": float(_get_draft_menu_gripper_open(code)),
                }
            return payload

        def _edit_row(row_state):
            if save_state["saving"]:
                return
            if not _is_row_editable(row_state):
                return
            row_def = row_state["def"]
            row_type = str(row_def.get("type", "") or "").strip().lower()
            row_label = str(row_def.get("label", row_def.get("var_name", "-")))
            values = self._get_robot_action_pose_entry(draft_cfg, row_def)
            edited = None

            if row_type == "posj":
                defaults = values if values is not None else [0.0] * 6
                edited = self._ask_six_values_form(
                    f"{row_label} 수정",
                    ["J1", "J2", "J3", "J4", "J5", "J6"],
                    defaults,
                    limits=JOINT_INPUT_LIMITS_DEG,
                    guide_text="안내: robot_action_planner의 조인트(posj) 변수 값을 수정합니다.",
                )
            elif row_type == "posx":
                defaults = values if values is not None else [0.0] * 6
                edited = self._ask_six_values_form(
                    f"{row_label} 수정",
                    ["X", "Y", "Z", "A", "B", "C"],
                    defaults,
                    guide_text="안내: robot_action_planner의 카테시안(posx) 변수 값을 수정합니다.",
                )
            elif row_type == "vision_offset":
                defaults = values if values is not None else [0.0, 0.0, 0.0]
                edited = self._ask_three_values_form(
                    f"{row_label} 수정",
                    ["X", "Y", "Z"],
                    defaults,
                    guide_text="안내: 비전 기준 상대 오프셋(mm)입니다. 티칭은 지원하지 않습니다.",
                )
            elif row_type == "menu_offset_xyz":
                code = _menu_code_from_row(row_def, allow_glass=True)
                defaults = _get_draft_menu_xyz(code)
                edited = self._ask_three_values_form(
                    f"{row_label} 수정",
                    ["X", "Y", "Z"],
                    defaults,
                    guide_text="안내: 재료별 비전 기준 XYZ 오프셋(mm) 값입니다.",
                )
                if edited is None:
                    return
                if isinstance(edited, str):
                    QMessageBox.warning(dialog, "포지션 설정", edited, QMessageBox.Ok)
                    return
                _set_draft_menu_xyz(code, edited)
                _refresh_row(row_state, status_text="수정됨(미저장)")
                _set_status(f"{row_label}: 값 수정 완료 (저장 필요)")
                return
            elif row_type == "menu_gripper_close_mm":
                code = _menu_code_from_row(row_def, allow_glass=True)
                current_v = float(_get_draft_menu_gripper(code))
                value, ok_edit = QInputDialog.getDouble(
                    dialog,
                    f"{row_label} 수정",
                    "그리퍼 닫힘 거리(mm):",
                    current_v,
                    0.0,
                    float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT),
                    2,
                )
                if not ok_edit:
                    return
                _set_draft_menu_gripper(code, float(value))
                _refresh_row(row_state, status_text="수정됨(미저장)")
                _set_status(f"{row_label}: 값 수정 완료 (저장 필요)")
                return
            elif row_type == "menu_gripper_open_mm":
                code = _menu_code_from_row(row_def, allow_glass=True)
                current_v = float(_get_draft_menu_gripper_open(code))
                value, ok_edit = QInputDialog.getDouble(
                    dialog,
                    f"{row_label} 수정",
                    "그리퍼 오픈 거리(mm):",
                    current_v,
                    0.0,
                    float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT),
                    2,
                )
                if not ok_edit:
                    return
                _set_draft_menu_gripper_open(code, float(value))
                _refresh_all_rows(status_text="수정됨(미저장)")
                _set_status(f"{row_label}: 값 수정 완료 (저장 필요)")
                return
            elif row_type == "global_gripper_open_mm":
                current_v = float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT)
                value, ok_edit = QInputDialog.getDouble(
                    dialog,
                    f"{row_label} 수정",
                    "그리퍼 오픈 거리(mm):",
                    current_v,
                    0.0,
                    float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT),
                    2,
                )
                if not ok_edit:
                    return
                return
            else:
                return

            if edited is None:
                return
            if isinstance(edited, str):
                QMessageBox.warning(dialog, "포지션 설정", edited, QMessageBox.Ok)
                return
            ok_set, msg_set = self._set_robot_action_pose_entry(draft_cfg, row_def, edited)
            if not ok_set:
                QMessageBox.warning(dialog, "포지션 설정", msg_set, QMessageBox.Ok)
                return
            _refresh_row(row_state, status_text="수정됨(미저장)")
            _set_status(f"{row_label}: 값 수정 완료 (저장 필요)")

        def _teach_row(row_state):
            if save_state["saving"]:
                return
            row_def = row_state["def"]
            row_type = str(row_def.get("type", "") or "").strip().lower()
            if row_type not in ("posj", "posx"):
                return
            posj, posx, err = self._capture_current_teach_pose()
            if err is not None:
                QMessageBox.warning(dialog, "포지션 티칭", err, QMessageBox.Ok)
                return
            values = posj if row_type == "posj" else posx
            ok_set, msg_set = self._set_robot_action_pose_entry(draft_cfg, row_def, values)
            if not ok_set:
                QMessageBox.warning(dialog, "포지션 티칭", msg_set, QMessageBox.Ok)
                return
            row_label = str(row_def.get("label", row_def.get("var_name", "-")))
            _refresh_row(row_state, status_text="티칭됨(미저장)")
            _set_status(f"{row_label}: 현재 위치 티칭 반영 (저장 필요)")

        def _move_row(row_state, action: str = "move"):
            if save_state["saving"]:
                return
            row_def = row_state["def"]
            row_type = str(row_def.get("type", "") or "").strip().lower()
            row_label = str(row_def.get("label", row_def.get("var_name", "-")))
            if self.backend is None:
                QMessageBox.warning(dialog, "포지션 이동", "백엔드 초기화 중입니다.", QMessageBox.Ok)
                return
            if row_type == "posj":
                values = self._get_robot_action_pose_entry(draft_cfg, row_def)
                if values is None:
                    QMessageBox.warning(dialog, "포지션 이동", f"{row_label} 값이 없습니다.", QMessageBox.Ok)
                    return
                if not hasattr(self.backend, "send_move_joint"):
                    QMessageBox.warning(dialog, "포지션 이동", "백엔드가 조인트 이동을 지원하지 않습니다.", QMessageBox.Ok)
                    return
                if not self._confirm_motion_with_values(
                    "포지션 이동 확인",
                    "안내: 로봇이 실제로 이동합니다.\n조인트 목표 위치는 아래와 같습니다.",
                    ["J1", "J2", "J3", "J4", "J5", "J6"],
                    values,
                    "액션포지션 이동",
                ):
                    return
                speed = self._motion_speed_for_command()
                ok_move, msg_move = self.backend.send_move_joint(*values, vel=speed, acc=speed)
            elif row_type == "posx":
                values = self._get_robot_action_pose_entry(draft_cfg, row_def)
                if values is None:
                    QMessageBox.warning(dialog, "포지션 이동", f"{row_label} 값이 없습니다.", QMessageBox.Ok)
                    return
                if not hasattr(self.backend, "send_move_cartesian"):
                    QMessageBox.warning(dialog, "포지션 이동", "백엔드가 카테시안 이동을 지원하지 않습니다.", QMessageBox.Ok)
                    return
                if not self._confirm_motion_with_values(
                    "포지션 이동 확인",
                    "안내: 로봇이 실제로 이동합니다.\n카테시안 목표 위치는 아래와 같습니다.",
                    ["X", "Y", "Z", "A", "B", "C"],
                    values,
                    "액션포지션 이동",
                ):
                    return
                speed = self._motion_speed_for_command()
                ok_move, msg_move = self.backend.send_move_cartesian(*values, vel=speed, acc=speed)
            elif row_type == "vision_target":
                if not hasattr(self.backend, "get_robot_action_vision_target_xyz"):
                    QMessageBox.warning(dialog, "비전 기준좌표", "백엔드가 비전 기준좌표 계산을 지원하지 않습니다.", QMessageBox.Ok)
                    return
                ingredient_code, ingredient_label = _resolve_row_ingredient(
                    row_def,
                    row_label,
                    "비전 기준으로 사용할 재료를 선택하세요:",
                    allow_glass=False,
                )
                if not ingredient_code:
                    return
                reply = QMessageBox.question(
                    dialog,
                    "비전 기준좌표 수신 확인",
                    (
                        "비전에서 선택 재료를 다시 감지해 기준좌표를 계산합니다.\n"
                        "캘리브레이션(affine + camera_info)과 메뉴 오프셋이 적용됩니다.\n\n"
                        f"재료: {ingredient_label}\n"
                        "계속할까요?"
                    ),
                    QMessageBox.Yes | QMessageBox.No,
                    QMessageBox.No,
                )
                if reply != QMessageBox.Yes:
                    return

                ok_calc, payload_calc, msg_calc = self.backend.get_robot_action_vision_target_xyz(
                    ingredient_code=ingredient_code,
                    menu_offsets=_build_draft_menu_offset_payload(),
                    apply_menu_offset=True,
                )
                if not ok_calc:
                    _refresh_all_rows()
                    QMessageBox.warning(dialog, "비전 기준좌표", str(msg_calc or "기준좌표 계산 실패"), QMessageBox.Ok)
                    _set_status(f"{row_label}: {msg_calc}")
                    self.append_log(f"[액션포지션] {row_label} 기준좌표 수신 실패: {msg_calc}\n")
                    return

                vision_target_state["payload_by_code"][ingredient_code] = dict(payload_calc) if isinstance(payload_calc, dict) else {}
                vision_target_state["last_code"] = str(ingredient_code)
                vision_target_state["last_label"] = str(ingredient_label)
                _persist_vision_target_state()
                _refresh_all_rows()
                _refresh_row(row_state, status_text="기준좌표 수신됨")
                _set_status(f"{row_label}: {msg_calc}")
                self.append_log(f"[액션포지션] {row_label} 기준좌표 수신: {msg_calc}\n")
                return
            elif row_type == "gripper_open_action":
                if not hasattr(self.backend, "send_gripper_move"):
                    QMessageBox.warning(dialog, "그리퍼 실행", "백엔드가 그리퍼 이동을 지원하지 않습니다.", QMessageBox.Ok)
                    return
                ingredient_code, ingredient_label = _resolve_row_ingredient(
                    row_def,
                    row_label,
                    "오픈할 목표 재료를 선택하세요:",
                    allow_glass=True,
                )
                if not ingredient_code:
                    return
                open_mm = float(_get_draft_menu_gripper_open(ingredient_code))
                if not self._confirm_motion_with_values(
                    "그리퍼 오픈 확인",
                    "안내: 선택 재료 기준 오픈(mm)으로 그리퍼가 실제로 이동합니다.",
                    ["Open(mm)"],
                    [open_mm],
                    f"액션포지션 그리퍼 오픈 ({ingredient_label})",
                ):
                    return
                ok_move, msg_move = self.backend.send_gripper_move(open_mm)
                msg_move = f"{msg_move} | target={ingredient_label}({ingredient_code}), open={open_mm:.1f}mm"
            elif row_type == "gripper_close_action":
                if not hasattr(self.backend, "send_gripper_move"):
                    QMessageBox.warning(dialog, "그리퍼 실행", "백엔드가 그리퍼 이동을 지원하지 않습니다.", QMessageBox.Ok)
                    return
                ingredient_code, ingredient_label = _resolve_row_ingredient(
                    row_def,
                    row_label,
                    "파지할 목표 재료를 선택하세요:",
                    allow_glass=True,
                )
                if not ingredient_code:
                    return
                close_mm = float(_get_draft_menu_gripper(ingredient_code))
                if not self._confirm_motion_with_values(
                    "그리퍼 파지 확인",
                    "안내: 선택 재료 기준 파지(mm)로 그리퍼가 실제로 이동합니다.",
                    ["Close(mm)"],
                    [close_mm],
                    f"액션포지션 그리퍼 파지 ({ingredient_label})",
                ):
                    return
                ok_move, msg_move = self.backend.send_gripper_move(close_mm)
                if ok_move:
                    gripper_target_state["ingredient_code"] = str(ingredient_code)
                    gripper_target_state["ingredient_label"] = str(ingredient_label)
                msg_move = f"{msg_move} | target={ingredient_label}({ingredient_code}), close={close_mm:.1f}mm"
            elif row_type == "gripper_actions":
                if not hasattr(self.backend, "send_gripper_move"):
                    QMessageBox.warning(dialog, "그리퍼 실행", "백엔드가 그리퍼 이동을 지원하지 않습니다.", QMessageBox.Ok)
                    return
                ingredient_code = str(_menu_code_from_row(row_def, allow_glass=True) or "").strip().lower()
                ingredient_label = str(BARTENDER_MENU_LABELS.get(ingredient_code, ingredient_code) or ingredient_code)
                if str(action or "").strip().lower() == "open":
                    open_mm = float(_get_draft_menu_gripper_open(ingredient_code))
                    if not self._confirm_motion_with_values(
                        "그리퍼 오픈 확인",
                        "안내: 선택 재료 기준 오픈(mm)으로 그리퍼가 실제로 이동합니다.",
                        ["Open(mm)"],
                        [open_mm],
                        f"액션포지션 그리퍼 오픈 ({ingredient_label})",
                    ):
                        return
                    ok_move, msg_move = self.backend.send_gripper_move(open_mm)
                    msg_move = f"{msg_move} | target={ingredient_label}({ingredient_code}), open={open_mm:.1f}mm"
                else:
                    close_mm = float(_get_draft_menu_gripper(ingredient_code))
                    if not self._confirm_motion_with_values(
                        "그리퍼 파지 확인",
                        "안내: 선택 재료 기준 파지(mm)로 그리퍼가 실제로 이동합니다.",
                        ["Close(mm)"],
                        [close_mm],
                        f"액션포지션 그리퍼 파지 ({ingredient_label})",
                    ):
                        return
                    ok_move, msg_move = self.backend.send_gripper_move(close_mm)
                    msg_move = f"{msg_move} | target={ingredient_label}({ingredient_code}), close={close_mm:.1f}mm"
            elif row_type == "vision_offset":
                values = self._get_robot_action_pose_entry(draft_cfg, row_def)
                if values is None:
                    QMessageBox.warning(dialog, "비전 오프셋 이동", f"{row_label} 값이 없습니다.", QMessageBox.Ok)
                    return
                payload = _vision_payload_for_row(row_def)
                if not isinstance(payload, dict):
                    QMessageBox.warning(dialog, "비전 오프셋 이동", "먼저 같은 탭의 '비전 타겟 계산'에서 위치받아오기를 실행하세요.", QMessageBox.Ok)
                    return
                base_xyz = payload.get("resolved_robot_xyz_mm")
                if not isinstance(base_xyz, (list, tuple)) or len(base_xyz) < 3:
                    QMessageBox.warning(dialog, "비전 오프셋 이동", "수신된 기준좌표가 올바르지 않습니다. 다시 위치받아오기를 실행하세요.", QMessageBox.Ok)
                    return
                try:
                    bx, by, bz = float(base_xyz[0]), float(base_xyz[1]), float(base_xyz[2])
                    ox, oy, oz = float(values[0]), float(values[1]), float(values[2])
                    tx, ty, tz = bx + ox, by + oy, bz + oz
                except Exception:
                    QMessageBox.warning(dialog, "비전 오프셋 이동", "기준좌표/오프셋 값이 올바르지 않습니다.", QMessageBox.Ok)
                    return

                abc = None
                abc_source = ""
                if hasattr(self.backend, "_resolve_robot_action_abc"):
                    try:
                        abc_vals, src = self.backend._resolve_robot_action_abc()
                        if isinstance(abc_vals, (list, tuple)) and len(abc_vals) >= 3:
                            abc = [float(abc_vals[0]), float(abc_vals[1]), float(abc_vals[2])]
                            abc_source = str(src or "").strip()
                    except Exception:
                        abc = None
                if abc is None:
                    _posj, posx = self._get_current_pose_defaults()
                    if not isinstance(posx, (list, tuple)) or len(posx) < 6:
                        QMessageBox.warning(dialog, "비전 오프셋 이동", "현재 TCP 자세(ABC)를 읽지 못했습니다.", QMessageBox.Ok)
                        return
                    try:
                        abc = [float(posx[3]), float(posx[4]), float(posx[5])]
                        abc_source = "UI current_posx"
                    except Exception:
                        QMessageBox.warning(dialog, "비전 오프셋 이동", "현재 TCP 자세(ABC) 파싱에 실패했습니다.", QMessageBox.Ok)
                        return

                ingredient_code = _menu_code_from_row(row_def, allow_glass=False)
                ingredient_label = str(BARTENDER_MENU_LABELS.get(ingredient_code, ingredient_code) or ingredient_code) if ingredient_code else "-"
                if not self._confirm_motion_with_values(
                    "비전 오프셋 이동 확인",
                    (
                        "안내: 로봇이 실제로 이동합니다.\n"
                        f"기준좌표({ingredient_label}) + {row_label} 오프셋을 적용한 목표입니다."
                    ),
                    ["X", "Y", "Z"],
                    [tx, ty, tz],
                    "액션포지션 비전오프셋 이동",
                ):
                    return
                speed = self._motion_speed_for_command()
                ok_move, msg_move = self.backend.send_move_cartesian(
                    float(tx),
                    float(ty),
                    float(tz),
                    float(abc[0]),
                    float(abc[1]),
                    float(abc[2]),
                    vel=speed,
                    acc=speed,
                )
                msg_move = (
                    f"{msg_move} | base=({bx:.1f},{by:.1f},{bz:.1f}) + "
                    f"offset=({ox:.1f},{oy:.1f},{oz:.1f}) -> target=({tx:.1f},{ty:.1f},{tz:.1f}), "
                    f"ABC source={abc_source or '-'}"
                )
            else:
                return
            _refresh_row(row_state, status_text=("이동요청 완료" if ok_move else "이동 실패"))
            _set_status(f"{row_label}: {msg_move}")
            self.append_log(f"[액션포지션 이동] {row_label}: {msg_move}\n")

        table_header_labels = ["순서", "구역", "변수명", "타입", "현재값", "작업", "설명", "상태"]
        for tab_code, tab_label in ROBOT_ACTION_POSE_TABS:
            page = QWidget(tab_widget)
            page_layout = QVBoxLayout(page)
            page_layout.setContentsMargins(4, 4, 4, 4)
            page_layout.setSpacing(4)
            tab_rows = list(rows_by_tab.get(tab_code, []))

            table = QTableWidget(len(tab_rows), 8, page)
            table.setEditTriggers(QTableWidget.NoEditTriggers)
            table.setSelectionMode(QTableWidget.NoSelection)
            table.verticalHeader().setVisible(False)
            table.verticalHeader().setDefaultSectionSize(max(24, int(UI_PANEL_TABLE_ROW_HEIGHT)))
            table.setHorizontalHeaderLabels(table_header_labels)
            header = table.horizontalHeader()
            if header is not None:
                header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
                header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
                header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
                header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
                header.setSectionResizeMode(4, QHeaderView.Stretch)
                header.setSectionResizeMode(5, QHeaderView.ResizeToContents)
                header.setSectionResizeMode(6, QHeaderView.Stretch)
                header.setSectionResizeMode(7, QHeaderView.ResizeToContents)
            page_layout.addWidget(table, 1)
            tab_widget.addTab(page, str(tab_label))
            tables_by_tab[tab_code] = table
            table_row_states[table] = []

            for row_index, row_def in enumerate(tab_rows):
                if _is_reference_row(row_def):
                    value_item = QTableWidgetItem("-")
                else:
                    value_item = QTableWidgetItem(self._format_robot_action_pose_row_value(row_def, draft_cfg))
                if _is_reference_row(row_def):
                    state_item = QTableWidgetItem("참조")
                else:
                    state_item = QTableWidgetItem(
                        "참조전용" if str(row_def.get("type", "")).lower() == "vision_target" else "저장됨"
                    )
                table.setItem(row_index, 0, QTableWidgetItem(str(row_index + 1)))
                table.setItem(row_index, 1, QTableWidgetItem(str(row_def.get("section", "-"))))
                table.setItem(row_index, 2, QTableWidgetItem(_display_var_name(row_def.get("var_name", "-"))))
                table.setItem(row_index, 3, QTableWidgetItem(_display_row_type(row_def.get("type", "-"))))
                table.setItem(row_index, 4, value_item)
                table.setItem(row_index, 6, QTableWidgetItem(_display_row_desc(row_def)))
                table.setItem(row_index, 7, state_item)

                button_wrap = QFrame(table)
                button_layout = QHBoxLayout(button_wrap)
                button_layout.setContentsMargins(2, 0, 2, 0)
                button_layout.setSpacing(2)
                move_btn = None
                teach_btn = None
                edit_btn = None
                extra_move_btns = []
                row_type = str(row_def.get("type", "") or "").strip().lower()
                if row_type in ("posj", "posx"):
                    move_btn = QPushButton("이동", button_wrap)
                    move_btn.setMinimumWidth(44)
                    button_layout.addWidget(move_btn)
                    if bool(row_def.get("editable", True)):
                        teach_btn = QPushButton("티칭", button_wrap)
                        teach_btn.setMinimumWidth(46)
                        button_layout.addWidget(teach_btn)
                elif row_type == "vision_target":
                    move_btn = QPushButton("위치받아오기", button_wrap)
                    move_btn.setMinimumWidth(78)
                    button_layout.addWidget(move_btn)
                elif row_type == "vision_offset":
                    move_btn = QPushButton("이동", button_wrap)
                    move_btn.setMinimumWidth(44)
                    button_layout.addWidget(move_btn)
                elif row_type == "gripper_open_action":
                    move_btn = QPushButton("오픈", button_wrap)
                    move_btn.setMinimumWidth(48)
                    button_layout.addWidget(move_btn)
                elif row_type == "gripper_close_action":
                    move_btn = QPushButton("파지", button_wrap)
                    move_btn.setMinimumWidth(48)
                    button_layout.addWidget(move_btn)
                elif row_type == "gripper_actions":
                    open_btn = QPushButton("오픈", button_wrap)
                    open_btn.setMinimumWidth(48)
                    close_btn = QPushButton("파지", button_wrap)
                    close_btn.setMinimumWidth(48)
                    button_layout.addWidget(open_btn)
                    button_layout.addWidget(close_btn)
                    extra_move_btns.extend([open_btn, close_btn])
                elif row_type in ("menu_offset_xyz", "menu_gripper_close_mm", "menu_gripper_open_mm"):
                    pass
                table.setCellWidget(row_index, 5, button_wrap)

                row_state = {
                    "index": row_index,
                    "tab": tab_code,
                    "table": table,
                    "def": row_def,
                    "value_item": value_item,
                    "state_item": state_item,
                    "move_btn": move_btn,
                    "teach_btn": teach_btn,
                    "edit_btn": edit_btn,
                    "extra_move_btns": list(extra_move_btns),
                    "button_wrap": button_wrap,
                    "base_bg_hex": "#fff9db" if str(row_def.get("section", "") or "").startswith("[0]") else "",
                    "var_key": _selection_var_key(row_def),
                }
                if move_btn is not None:
                    move_btn.clicked.connect(
                        lambda _checked=False, state=row_state: (_set_row_selected(state), _move_row(state))
                    )
                if teach_btn is not None:
                    teach_btn.clicked.connect(
                        lambda _checked=False, state=row_state: (_set_row_selected(state), _teach_row(state))
                    )
                if row_type == "gripper_actions" and len(extra_move_btns) >= 2:
                    open_btn = extra_move_btns[0]
                    close_btn = extra_move_btns[1]
                    open_btn.clicked.connect(
                        lambda _checked=False, state=row_state: (_set_row_selected(state), _move_row(state, action="open"))
                    )
                    close_btn.clicked.connect(
                        lambda _checked=False, state=row_state: (_set_row_selected(state), _move_row(state, action="close"))
                    )
                table_row_states[table].append(row_state)
                row_states.append(row_state)
                _apply_row_bg_color(row_state, row_state.get("base_bg_hex", ""))

            def _on_table_cell_clicked(row_index, column_index, _table=table):
                if save_state["saving"]:
                    return
                states = table_row_states.get(_table, [])
                if int(row_index) < 0 or int(row_index) >= len(states):
                    return
                state = states[int(row_index)]
                _set_row_selected(state)
                if int(column_index) == 4 and _is_row_editable(state):
                    _edit_row(state)

            table.cellClicked.connect(_on_table_cell_clicked)

        _refresh_all_rows()

        def _reload_saved_values():
            nonlocal saved_cfg, draft_cfg, saved_menu_offsets, saved_menu_gripper, saved_menu_gripper_open, saved_menu_labels
            nonlocal draft_menu_offsets, draft_menu_gripper, draft_menu_gripper_open, draft_menu_labels
            if save_state["saving"]:
                return
            loaded = self._load_robot_action_pose_config()
            saved_cfg = self._sanitize_robot_action_pose_config(loaded)
            draft_cfg = self._sanitize_robot_action_pose_config(saved_cfg)
            self._robot_action_pose_config = self._sanitize_robot_action_pose_config(saved_cfg)
            saved_menu_offsets = dict(self._menu_xyz_offsets_by_code)
            saved_menu_gripper = dict(self._menu_gripper_close_mm_by_code)
            saved_menu_gripper_open = dict(self._menu_gripper_open_mm_by_code)
            saved_menu_labels = dict(self._menu_label_by_code)
            draft_menu_offsets = dict(saved_menu_offsets)
            draft_menu_gripper = dict(saved_menu_gripper)
            draft_menu_gripper_open = dict(saved_menu_gripper_open)
            draft_menu_labels = dict(saved_menu_labels)
            gripper_target_state["ingredient_code"] = ""
            gripper_target_state["ingredient_label"] = ""
            _refresh_all_rows(status_text="저장값 로드")
            _set_status("저장된 포지션/오프셋 값을 다시 불러왔습니다. (위치받아오기 값 유지)")

        def _save_menu_offsets_payload(offset_map: dict, gripper_map: dict, gripper_open_map: dict, label_map: dict):
            payload = {"menus": {}}
            codes = []
            for code in self._menu_codes_for_offset_ui():
                code_txt = str(code or "").strip().lower()
                if not code_txt:
                    continue
                if code_txt not in codes:
                    codes.append(code_txt)
            extra_codes = set()
            extra_codes.update(str(k).strip().lower() for k in offset_map.keys())
            extra_codes.update(str(k).strip().lower() for k in gripper_map.keys())
            extra_codes.update(str(k).strip().lower() for k in gripper_open_map.keys())
            extra_codes.update(str(k).strip().lower() for k in label_map.keys())
            for code in sorted(extra_codes):
                if (not code) or code in BARTENDER_MENU_OFFSET_EXCLUDED_CODES_NORM:
                    continue
                if code not in codes:
                    codes.append(code)
            for code in codes:
                xyz = offset_map.get(code, (0.0, 0.0, 0.0))
                try:
                    x = float(xyz[0])
                    y = float(xyz[1])
                    z = float(xyz[2])
                except Exception:
                    x, y, z = 0.0, 0.0, 0.0
                try:
                    grip_mm = float(gripper_map.get(code, 41.0))
                    if (not np.isfinite(grip_mm)) or grip_mm < 0.0:
                        grip_mm = 41.0
                except Exception:
                    grip_mm = 41.0
                try:
                    grip_open_mm = float(gripper_open_map.get(code, ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT))
                    if (not np.isfinite(grip_open_mm)) or grip_open_mm < 0.0:
                        grip_open_mm = float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT)
                    grip_open_mm = min(float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT), float(grip_open_mm))
                except Exception:
                    grip_open_mm = float(ROBOT_ACTION_GRIPPER_OPEN_MM_DEFAULT)
                label = str(label_map.get(code, BARTENDER_MENU_LABELS.get(code, code)) or code)
                payload["menus"][code] = {
                    "label": label,
                    "offset_xyz_mm": [float(x), float(y), float(z)],
                    "gripper_close_mm": float(grip_mm),
                    "gripper_open_mm": float(grip_open_mm),
                }
            try:
                os.makedirs(os.path.dirname(MENU_OFFSET_CONFIG_PATH), exist_ok=True)
                with open(MENU_OFFSET_CONFIG_PATH, "w", encoding="utf-8") as fp:
                    json.dump(payload, fp, ensure_ascii=False, indent=2)
                return True, "저장 완료"
            except Exception as exc:
                return False, f"저장 실패: {exc}"

        def _set_dialog_enabled(enabled: bool):
            tab_widget.setEnabled(bool(enabled))
            for table in list(table_row_states.keys()):
                table.setEnabled(bool(enabled))
            refresh_btn.setEnabled(bool(enabled))
            save_btn.setEnabled(bool(enabled))
            close_btn.setEnabled(bool(enabled))

        def _finish_save():
            nonlocal saved_cfg, draft_cfg, saved_menu_offsets, saved_menu_gripper, saved_menu_gripper_open, saved_menu_labels
            nonlocal draft_menu_offsets, draft_menu_gripper, draft_menu_gripper_open, draft_menu_labels
            save_state["saving"] = False
            _set_dialog_enabled(True)
            if bool(save_state["ok"]):
                saved_cfg = self._sanitize_robot_action_pose_config(draft_cfg)
                draft_cfg = self._sanitize_robot_action_pose_config(saved_cfg)
                self._robot_action_pose_config = self._sanitize_robot_action_pose_config(saved_cfg)

                ok_menu, msg_menu = _save_menu_offsets_payload(
                    dict(draft_menu_offsets),
                    dict(draft_menu_gripper),
                    dict(draft_menu_gripper_open),
                    dict(draft_menu_labels),
                )
                if ok_menu:
                    self._menu_xyz_offsets_by_code = dict(draft_menu_offsets)
                    self._menu_gripper_close_mm_by_code = dict(draft_menu_gripper)
                    self._menu_gripper_open_mm_by_code = dict(draft_menu_gripper_open)
                    self._menu_label_by_code = dict(draft_menu_labels)
                    saved_menu_offsets = dict(self._menu_xyz_offsets_by_code)
                    saved_menu_gripper = dict(self._menu_gripper_close_mm_by_code)
                    saved_menu_gripper_open = dict(self._menu_gripper_open_mm_by_code)
                    saved_menu_labels = dict(self._menu_label_by_code)
                    draft_menu_offsets = dict(saved_menu_offsets)
                    draft_menu_gripper = dict(saved_menu_gripper)
                    draft_menu_gripper_open = dict(saved_menu_gripper_open)
                    draft_menu_labels = dict(saved_menu_labels)
                    self._refresh_menu_xyz_offset_button_text()
                    _refresh_all_rows(status_text="저장됨")
                    _set_status(f"저장 완료: {ROBOT_ACTION_POSE_CONFIG_PATH}, {MENU_OFFSET_CONFIG_PATH}")
                    self._append_voice_order_log(f"로봇 액션 포지션 설정 저장: {ROBOT_ACTION_POSE_CONFIG_PATH}")
                    self._append_voice_order_log(f"메뉴별 XYZ/파지/오픈(mm) 설정 저장: {MENU_OFFSET_CONFIG_PATH}")
                else:
                    _refresh_all_rows(status_text="부분저장")
                    _set_status(f"포지션 저장 완료, 메뉴 오프셋 저장 실패: {msg_menu}")
                    QMessageBox.warning(dialog, "포지션 설정", f"포지션 저장은 완료했지만 메뉴 오프셋 저장에 실패했습니다.\n{msg_menu}", QMessageBox.Ok)
            else:
                _set_status(str(save_state["msg"] or "저장 실패"))
                QMessageBox.warning(dialog, "포지션 설정", str(save_state["msg"] or "저장 실패"), QMessageBox.Ok)

        def _poll_save_done():
            if not bool(save_state["done"]):
                QTimer.singleShot(80, _poll_save_done)
                return
            _finish_save()

        def _save_worker(payload):
            ok_save, msg_save = self._save_robot_action_pose_config(payload)
            save_state["ok"] = bool(ok_save)
            save_state["msg"] = str(msg_save)
            save_state["done"] = True

        def _save_values_async():
            if save_state["saving"]:
                return
            payload = self._sanitize_robot_action_pose_config(draft_cfg)
            save_state["saving"] = True
            save_state["done"] = False
            save_state["ok"] = False
            save_state["msg"] = ""
            _set_dialog_enabled(False)
            _set_status("저장 중...")
            th = threading.Thread(
                target=_save_worker,
                args=(payload,),
                daemon=True,
                name="robot-action-pose-save",
            )
            save_state["thread"] = th
            th.start()
            QTimer.singleShot(80, _poll_save_done)

        refresh_btn.clicked.connect(_reload_saved_values)
        save_btn.clicked.connect(_save_values_async)
        close_btn.clicked.connect(dialog.accept)

        dialog.exec_()

    def _pull_bartender_sequence_snapshot(self):
        backend = getattr(self, "backend", None)
        if backend is None or (not hasattr(backend, "get_bartender_sequence_snapshot")):
            return None
        try:
            snap = backend.get_bartender_sequence_snapshot()
        except Exception as exc:
            self._append_voice_order_log(f"시퀀스 스냅샷 조회 실패: {exc}", level="warning")
            return None
        return dict(snap or {}) if isinstance(snap, dict) else None

    def _apply_bartender_sequence_snapshot(self, snapshot: dict):
        if not isinstance(snapshot, dict):
            return
        def _summarize_ui_text(text: str, max_len: int = 34) -> str:
            raw = str(text or "").strip()
            if not raw:
                return "-"
            short = raw
            # 로그에 있는 상세 데이터/경로/파라미터는 UI 요약에서 제거
            for sep in (" | ", "(", "["):
                if sep in short:
                    short = short.split(sep, 1)[0].strip()
            # "A: B" 형태면 A가 상태성 문구일 때 A만 표기
            if ":" in short:
                left = short.split(":", 1)[0].strip()
                if any(k in left for k in ("실패", "오류", "중지", "예외", "타임아웃", "미검출")):
                    short = left
            short = short.strip() or raw
            if len(short) > int(max_len):
                short = short[: max_len - 1].rstrip() + "…"
            return short

        running = bool(snapshot.get("running", False))
        status = str(snapshot.get("status", "") or "").strip().lower()
        message = str(snapshot.get("message", "") or "").strip()
        mode = str(snapshot.get("mode", "") or "").strip().lower()
        run_id = int(snapshot.get("run_id", 0) or 0)
        req_payload = snapshot.get("request", {})
        if not isinstance(req_payload, dict):
            req_payload = {}
        is_voice_only = bool(req_payload.get("voice_only", False))
        last_error_step = str(snapshot.get("last_error_step", "") or "").strip()
        last_error_step_label = str(snapshot.get("last_error_step_label", "") or "").strip()
        last_error_stage = str(snapshot.get("last_error_stage", "") or "").strip()
        last_error_message = str(snapshot.get("last_error_message", "") or "").strip()
        # UI 모드는 사용자가 선택한 값을 단일 진실 소스로 사용한다.
        # 백엔드 스냅샷의 mode 값으로 UI 선택 상태를 덮어쓰지 않는다.

        prev_active_step = str(getattr(self, "_bartender_active_step", "") or "").strip()
        active_step_label = ""
        if is_voice_only:
            self._voice_sequence_running = bool(running)
            if running:
                self._voice_sequence_run_id = int(run_id)
            elif int(self._voice_sequence_run_id) == int(run_id):
                self._voice_sequence_run_id = 0
            active_step = str(snapshot.get("active_step", "") or "").strip().lower()
            if running and active_step in ("voice_request", "stt"):
                if active_step == "voice_request":
                    self._set_voice_order_process_state("입력요청중")
                    self._set_voice_order_mic_state("입력 요청중", "warn")
                    self._set_voice_order_mic_visual_active(False)
                else:
                    # active_step=stt 는 stt_wait/stt_process/stt 완료를 모두 포함한다.
                    # 세부 표시는 stage 이벤트에서 처리하므로, 스냅샷 폴링이 상태를 덮어쓰지 않게 유지한다.
                    pass
            try:
                mic_level = int(snapshot.get("voice_mic_level", 0) or 0)
            except Exception:
                mic_level = 0
            self._set_voice_order_mic_level(mic_level)
            self._update_voice_mic_button_state()
        else:
            step_states = snapshot.get("step_states", {})
            if isinstance(step_states, dict):
                for key, _label in BARTENDER_SEQUENCE_STEPS:
                    k = str(key)
                    if k not in step_states:
                        continue
                    self._bartender_sequence_state[k] = str(step_states.get(k, "pending") or "pending").strip().lower()
            self._bartender_active_step = str(snapshot.get("active_step", "") or "").strip()
            self._bartender_sequence_running = running
            if running:
                self._bartender_sequence_run_id = int(run_id)
            elif int(getattr(self, "_bartender_sequence_run_id", 0) or 0) == int(run_id):
                self._bartender_sequence_run_id = 0
            if running:
                self._bartender_status_lock = False
            elif status in ("success", "error", "stopped"):
                self._bartender_status_lock = True
            active_step_label = str(
                BARTENDER_SEQUENCE_STEP_LABEL_BY_KEY.get(self._bartender_active_step, self._bartender_active_step) or ""
            ).strip()
            if (not last_error_step_label) and last_error_step:
                last_error_step_label = str(
                    BARTENDER_SEQUENCE_STEP_LABEL_BY_KEY.get(last_error_step, last_error_step) or last_error_step
                ).strip()

            # 시퀀스가 voice_request 단계로 진입하면, 수동 "마이크 입력 시작" 버튼과 동일하게 UI를 맞춘다.
            if running and self._bartender_active_step == "voice_request" and prev_active_step != "voice_request":
                self._voice_ready_tone_armed = True
                self._set_voice_order_process_state("입력요청중")
                self._set_voice_order_mic_state("입력 요청중", "warn")
                self._set_voice_order_mic_visual_active(False)

        logs = list(snapshot.get("logs", []) or [])
        if logs:
            for row in logs:
                if not isinstance(row, dict):
                    continue
                try:
                    log_id = int(row.get("id", 0))
                except Exception:
                    log_id = 0
                if log_id <= int(self._bartender_last_log_id):
                    continue
                self._bartender_last_log_id = max(int(self._bartender_last_log_id), log_id)
                level = str(row.get("level", "info") or "info").strip().lower()
                stage = str(row.get("stage", "sequence") or "sequence").strip()
                msg = str(row.get("message", "") or "").strip()
                if msg:
                    self._append_voice_order_log(f"[{stage}] {msg}", level=level)
                stage_norm = stage.lower()
                if stage_norm == "input_request":
                    if running:
                        self._voice_ready_tone_armed = True
                        self._set_voice_order_process_state("입력요청중")
                        self._set_voice_order_mic_state("입력 요청중", "warn")
                        self._set_voice_order_mic_visual_active(False)
                elif stage_norm == "stt_open":
                    if running and bool(self._voice_ready_tone_armed):
                        self._voice_ready_tone_armed = False
                        self._play_voice_order_ready_tone()
                elif stage_norm == "stt_wait":
                    if running:
                        self._set_voice_order_process_state("마이크입력대기")
                        self._set_voice_order_mic_state("입력중", "ok")
                        self._set_voice_order_mic_visual_active(True)
                elif stage_norm == "stt_process":
                    if running:
                        self._set_voice_order_process_state("STT 처리중")
                        self._set_voice_order_mic_state("STT 처리중", "warn")
                        self._set_voice_order_mic_visual_active(False)
                elif stage_norm in ("classify", "llm", "recipe"):
                    if running:
                        self._set_voice_order_process_state("주문분석중")
                        self._set_voice_order_mic_state("처리중", "warn")
                        self._set_voice_order_mic_visual_active(False)

        last_order = snapshot.get("last_order_result", {})
        if isinstance(last_order, dict) and last_order:
            merged_result = dict(last_order)
            run_id = int(snapshot.get("run_id", 0) or 0)
            latest_input_text = ""
            for key in ("input_text", "stt_text", "recognized_text", "transcript", "text"):
                raw_value = merged_result.get(key, None)
                if raw_value is None:
                    continue
                parsed = str(raw_value or "").strip()
                if parsed:
                    latest_input_text = parsed
                    break
            if latest_input_text:
                self._set_voice_order_input_text(latest_input_text)
            self._voice_last_result = merged_result
            self._set_voice_order_result(merged_result)
            tts_text = str(merged_result.get("tts_text", "") or "").strip()
            result_status = str(merged_result.get("status", "") or "").strip().lower()
            allow_tts_play = True
            if str(getattr(self, "_bartender_mode", "") or "").strip().lower() == "auto":
                # 오토모드 음성 출력은 WEB UI 담당. 개발자 UI는 진행/결과 데이터만 표시한다.
                allow_tts_play = False
            if tts_text and result_status in ("success", "retry"):
                public_tts = self._sanitize_customer_tts_text(status=result_status, raw_text=tts_text)
                tts_sig = f"{run_id}:{result_status}:{public_tts}"
                if tts_sig != str(getattr(self, "_bartender_last_tts_signature", "")):
                    self._bartender_last_tts_signature = tts_sig
                    if allow_tts_play:
                        if result_status == "retry":
                            self._voice_retry_pending_after_tts = True
                            self._play_voice_order_feedback_tone(public_tts)
                        else:
                            self._voice_retry_pending_after_tts = False
                            self._play_voice_order_feedback_tone(public_tts)

        status_label = getattr(self, "_bartender_status_label", None)
        hint_label = getattr(self, "_bartender_mode_hint_label", None)
        if (not is_voice_only) and status_label is not None:
            if status in ("success",):
                status_label.setText(f"상태: 완료{(' - ' + message) if message else ''}")
                status_label.setStyleSheet("font-size: 10pt; font-weight: 800; color: #166534;")
                if hint_label is not None:
                    hint_label.setText("시퀀스 정상 완료")
            elif status in ("error",):
                fail_step_text = str(last_error_step_label or active_step_label or "단계 미확인").strip()
                fail_reason = str(last_error_message or message or "원인 미확인").strip()
                fail_reason_short = _summarize_ui_text(fail_reason, max_len=30)
                status_label.setText(f"상태: 실패({fail_step_text}) - {fail_reason_short}")
                status_label.setStyleSheet("font-size: 10pt; font-weight: 800; color: #b91c1c;")
                if hint_label is not None:
                    fail_stage = str(last_error_stage or "-").strip()
                    hint_label.setText(f"오류 위치: {fail_step_text} (stage={fail_stage}) | 상세 원인: 로그창 확인")
            elif status in ("stopping", "stopped"):
                msg_short = _summarize_ui_text(message, max_len=30) if message else ""
                status_label.setText(f"상태: 중지{(' - ' + msg_short) if msg_short else ''}")
                status_label.setStyleSheet("font-size: 10pt; font-weight: 800; color: #b45309;")
                if hint_label is not None:
                    hint_label.setText("중지 사유: 로그창 확인")
            elif running:
                run_step = str(active_step_label or "단계 확인중").strip()
                msg_short = _summarize_ui_text(message, max_len=26) if message else ""
                status_label.setText(f"상태: 실행중({run_step}){(' - ' + msg_short) if msg_short else ''}")
                status_label.setStyleSheet("font-size: 10pt; font-weight: 800; color: #1d4ed8;")
                if hint_label is not None:
                    hint_label.setText(f"현재 진행 단계: {run_step}")
            elif message and status not in ("", "idle"):
                status_label.setText(f"상태: {_summarize_ui_text(message, max_len=34)}")
                status_label.setStyleSheet("font-size: 10pt; font-weight: 800; color: #1f2937;")

        # 시퀀스가 종료된 상태에서는 마이크 상태가 '입력중'으로 남지 않도록 정리한다.
        if is_voice_only and (not running) and (not bool(getattr(self, "_voice_order_mic_test_running", False))):
            self._set_voice_order_mic_state("대기", "info")
            self._set_voice_order_mic_visual_active(False)

    def _severity_is_normal(self, severity: str):
        return str(severity or "").strip().lower() in ("normal", "ok", "success")

    def _ingredient_aliases_for_code(self, ingredient_code: str):
        code = str(ingredient_code or "").strip().lower()
        aliases = set(BARTENDER_INGREDIENT_ALIASES.get(code, set()))
        aliases.add(code)
        label = str(self._menu_label_by_code.get(code, "") or "").strip()
        if label:
            aliases.add(label.strip().lower())
        return {a for a in aliases if a}

    def _missing_recipe_ingredients_for_runtime(self):
        result = self._voice_last_result if isinstance(self._voice_last_result, dict) else {}
        status = str(result.get("status", "") or "").strip().lower()
        if status != "success":
            return []
        recipe = result.get("recipe", {})
        if not isinstance(recipe, dict) or (not recipe):
            return []
        payload = self._current_vision_meta_payload(panel_index=1)
        detections = payload.get("detections", []) if isinstance(payload, dict) else []
        present_classes = set()
        if isinstance(detections, list):
            for det in detections:
                if not isinstance(det, dict):
                    continue
                class_name = str(det.get("class_name", "") or "").strip().lower()
                if class_name:
                    present_classes.add(class_name)
        missing = []
        for key, value in recipe.items():
            try:
                amount = float(value)
            except Exception:
                amount = 0.0
            if amount <= 0.0:
                continue
            code = str(key or "").strip().lower()
            if not code:
                continue
            aliases = self._ingredient_aliases_for_code(code)
            if not any(alias in present_classes for alias in aliases):
                missing.append(code)
        return missing

    def _detect_bartender_runtime_safety_issue(self):
        backend = getattr(self, "backend", None)
        if backend is None:
            return "백엔드가 준비되지 않았습니다."
        if hasattr(backend, "is_ready"):
            try:
                if not bool(backend.is_ready()):
                    return "백엔드 연결이 준비되지 않았습니다."
            except Exception:
                return "백엔드 상태 확인 실패"

        if hasattr(backend, "get_robot_state_snapshot"):
            try:
                state_code, state_name, _seen_at = backend.get_robot_state_snapshot()
            except Exception:
                state_code, state_name = None, ""
            if state_code is not None:
                try:
                    code = int(state_code)
                except Exception:
                    code = None
                if code is not None and code in ROBOT_STATE_ERROR_CODES:
                    name = str(state_name or f"STATE_{code}")
                    return f"로봇 에러 상태 감지: {name}({code})"
        return None

    def _stop_bartender_sequence_for_safety(self, reason: str):
        text = str(reason or "").strip() or "안전정지 조건 발생"
        if text == str(getattr(self, "_bartender_last_safety_reason", "")):
            return
        self._bartender_last_safety_reason = text
        active = str(getattr(self, "_bartender_active_step", "") or "").strip()
        if active:
            self._set_bartender_step_state(active, "error")
        self._bartender_active_step = ""
        self._bartender_sequence_running = False
        self._bartender_manual_step_index = -1
        self._voice_ready_tone_armed = False
        self._stop_voice_order_worker(reason=f"안전정지: {text}")
        if self._bartender_status_label is not None:
            self._bartender_status_label.setText(f"상태: 안전정지 - {text}")
            self._bartender_status_label.setStyleSheet("font-size: 10pt; font-weight: 800; color: #b91c1c;")
        self._bartender_status_lock = True
        self._append_voice_order_log(f"[안전정지] {text}", level="error")

    def _check_bartender_runtime_safety(self):
        if not bool(getattr(self, "_bartender_sequence_running", False)):
            return
        now = time.monotonic()
        if (now - float(getattr(self, "_bartender_safety_last_check_at", 0.0))) < 0.25:
            return
        self._bartender_safety_last_check_at = now
        reason = self._detect_bartender_runtime_safety_issue()
        if reason:
            self._stop_bartender_sequence_for_safety(reason)

    def _reset_bartender_sequence(self, log: bool = True):
        self._stop_voice_order_feedback_audio("시퀀스 초기화")
        for step_key, _step_label in BARTENDER_SEQUENCE_STEPS:
            self._bartender_sequence_state[str(step_key)] = "pending"
        self._bartender_active_step = ""
        self._bartender_active_started_at = 0.0
        self._bartender_sequence_running = False
        self._bartender_manual_step_index = -1
        self._bartender_last_safety_reason = ""
        self._bartender_safety_last_check_at = 0.0
        self._bartender_last_log_id = 0
        self._bartender_status_lock = False
        self._refresh_bartender_sequence_styles()
        if log:
            self._append_voice_order_log("바텐더 시퀀스 상태 초기화")

    def _set_bartender_step_state(self, step_key: str, state: str):
        key = str(step_key or "").strip()
        if key not in self._bartender_sequence_state:
            return
        self._bartender_sequence_state[key] = str(state or "pending").strip().lower()
        self._refresh_bartender_sequence_styles()

    def _set_bartender_active_step(self, step_key: str):
        key = str(step_key or "").strip()
        if key not in self._bartender_sequence_state:
            return
        prev = str(getattr(self, "_bartender_active_step", "") or "").strip()
        if prev and prev in self._bartender_sequence_state and self._bartender_sequence_state.get(prev) == "active":
            self._bartender_sequence_state[prev] = "done"
        self._bartender_active_step = key
        self._bartender_active_started_at = time.monotonic()
        self._bartender_sequence_state[key] = "active"
        self._refresh_bartender_sequence_styles()

    def _run_bartender_manual_sequence(self):
        backend = getattr(self, "backend", None)
        if backend is None:
            self._append_voice_order_log("백엔드가 준비되지 않아 시퀀스를 실행할 수 없습니다.", level="warning")
            return
        if not hasattr(backend, "start_bartender_sequence") or not hasattr(backend, "stop_bartender_sequence"):
            self._append_voice_order_log("백엔드 시퀀스 API를 찾지 못했습니다.", level="error")
            return

        if bool(self._bartender_sequence_running):
            ok_stop, msg_stop, _snap = backend.stop_bartender_sequence(reason="개발자 UI 중지 요청")
            self._append_voice_order_log(f"시퀀스 중지 요청: {msg_stop}", level="warning" if ok_stop else "error")
            snap_now = self._pull_bartender_sequence_snapshot()
            if isinstance(snap_now, dict):
                self._apply_bartender_sequence_snapshot(snap_now)
            else:
                self._bartender_sequence_running = False
            self._update_bartender_mode_ui()
            return

        # 오토모드 대기에서는 개발자 버튼을 WEB UI 실행 버튼으로 사용한다.
        if str(self._bartender_mode or "").strip().lower() == "auto":
            self._open_voice_order_webui()
            return

        request_payload = {
            "input_text": str(getattr(self, "_voice_order_last_input_text", "") or "").strip(),
            "request_stt": (not bool(str(getattr(self, "_voice_order_last_input_text", "") or "").strip())),
            "allow_llm": True,
            "execute_robot_action": True,
            "motion_speed_percent": float(self._motion_speed_for_command()),
            "menu_offsets": self._build_menu_offset_payload(),
        }
        if str(self._bartender_mode or "manual").strip().lower() == "manual":
            # 메뉴얼 시퀀스 시작 시에도 테스트용 마이크 입력처럼 STT를 새로 요청한다.
            request_payload["input_text"] = ""
            request_payload["request_stt"] = True
            self._voice_order_last_input_text = ""
            if self._voice_order_input_edit is not None:
                self._voice_order_input_edit.setPlainText("노트북 마이크 입력 대기중...")
            self._set_voice_order_llm_text("-")
            self._set_voice_order_result(None)
        self._bartender_status_lock = False
        self._reset_bartender_sequence(log=False)
        ok_start, msg_start, snap = backend.start_bartender_sequence(
            mode=str(self._bartender_mode or "manual"),
            request=request_payload,
        )
        if not ok_start:
            self._append_voice_order_log(f"시퀀스 시작 실패: {msg_start}", level="error")
            snap_now = self._pull_bartender_sequence_snapshot()
            if isinstance(snap_now, dict):
                self._apply_bartender_sequence_snapshot(snap_now)
            else:
                self._bartender_sequence_running = False
                self._refresh_bartender_sequence_styles()
                self._update_bartender_mode_ui()
                if self._bartender_status_label is not None:
                    self._bartender_status_label.setText(f"상태: 시작 실패 - {msg_start}")
                    self._bartender_status_label.setStyleSheet("font-size: 10pt; font-weight: 800; color: #b91c1c;")
                if self._bartender_mode_hint_label is not None:
                    self._bartender_mode_hint_label.setText(f"오류 위치: 시퀀스 시작 요청 | 원인: {msg_start}")
                self._bartender_status_lock = True
            return
        self._append_voice_order_log(f"시퀀스 시작: {msg_start}")
        self._apply_bartender_sequence_snapshot(dict(snap or {}))
        self._update_bartender_mode_ui()

    def _run_bartender_robot_action_only_sequence(self):
        backend = getattr(self, "backend", None)
        if backend is None:
            self._append_voice_order_log("백엔드가 준비되지 않아 로봇동작 테스트를 실행할 수 없습니다.", level="warning")
            return
        if not hasattr(backend, "start_bartender_sequence") or not hasattr(backend, "stop_bartender_sequence"):
            self._append_voice_order_log("백엔드 시퀀스 API를 찾지 못했습니다.", level="error")
            return
        if bool(self._bartender_sequence_running):
            self._append_voice_order_log("시퀀스 실행 중에는 로봇동작 단독 테스트를 시작할 수 없습니다.", level="warning")
            return
        if str(self._bartender_mode or "").strip().lower() != "manual":
            self._append_voice_order_log("로봇동작 단독 테스트는 메뉴얼모드에서만 실행할 수 있습니다.", level="warning")
            return

        menu_codes = [
            str(code).strip().lower()
            for code in BARTENDER_ROBOT_ACTION_TEST_MENU_ORDER
            if str(code).strip().lower() in BARTENDER_ROBOT_ACTION_TEST_RECIPES
        ]
        if not menu_codes:
            self._append_voice_order_log("로봇동작 테스트 메뉴 설정이 비어 있습니다.", level="error")
            return

        menu_items = []
        menu_code_by_item = {}
        for code in menu_codes:
            label = str(BARTENDER_MENU_LABELS.get(code, code) or code)
            recipe = dict(BARTENDER_ROBOT_ACTION_TEST_RECIPES.get(code, {}) or {})
            recipe_chunks = []
            for ingredient_code, amount in recipe.items():
                ingredient_label = str(BARTENDER_MENU_LABELS.get(str(ingredient_code), ingredient_code) or ingredient_code)
                try:
                    amount_txt = f"{float(amount):g}"
                except Exception:
                    amount_txt = str(amount)
                recipe_chunks.append(f"{ingredient_label} {amount_txt}ml")
            item_text = f"{label} ({' + '.join(recipe_chunks)})" if recipe_chunks else label
            menu_items.append(item_text)
            menu_code_by_item[item_text] = code

        current_index = 1 if len(menu_items) > 1 else 0
        selected_item, ok_select = QInputDialog.getItem(
            self,
            "로봇동작 단독 테스트 메뉴 선택",
            "테스트할 메뉴를 선택하세요:",
            menu_items,
            current_index,
            False,
        )
        if not ok_select:
            return

        selected_menu = str(menu_code_by_item.get(str(selected_item), "") or "").strip().lower()
        selected_menu_label = str(BARTENDER_MENU_LABELS.get(selected_menu, selected_menu) or selected_menu)
        selected_recipe = dict(BARTENDER_ROBOT_ACTION_TEST_RECIPES.get(selected_menu, {}) or {})
        if not selected_menu or (not selected_recipe):
            self._append_voice_order_log("선택한 메뉴의 레시피를 불러오지 못했습니다.", level="error")
            return

        result = {
            "status": "success",
            "selected_menu": selected_menu,
            "selected_menu_label": selected_menu_label,
            "recipe": selected_recipe,
        }
        recipe_text = " + ".join(
            f"{BARTENDER_MENU_LABELS.get(str(code), str(code))} {float(amount):g}ml"
            for code, amount in selected_recipe.items()
        )
        extra_note = "소맥 테스트는 소주 1회 -> 맥주 1회 순서로 실행됩니다." if selected_menu == "somaek" else ""
        confirm_message = (
            "메뉴얼모드에서 7번 로봇 동작만 실행합니다.\n"
            f"선택 메뉴: {selected_menu_label}\n"
            f"레시피: {recipe_text}"
        )
        if extra_note:
            confirm_message += f"\n{extra_note}"
        confirm_message += "\n실행하시겠습니까?"
        reply = QMessageBox.question(
            self,
            "로봇동작 단독 테스트",
            confirm_message,
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        request_payload = {
            "robot_action_only": True,
            "order_result": result,
            "request_stt": False,
            "allow_llm": False,
            "execute_robot_action": True,
            "motion_speed_percent": float(self._motion_speed_for_command()),
            "menu_offsets": self._build_menu_offset_payload(),
        }
        self._bartender_status_lock = False
        self._reset_bartender_sequence(log=False)
        ok_start, msg_start, snap = backend.start_bartender_sequence(mode="manual", request=request_payload)
        if not ok_start:
            self._append_voice_order_log(f"로봇동작 단독 테스트 시작 실패: {msg_start}", level="error")
            return
        self._append_voice_order_log(f"로봇동작 단독 테스트 시작: {msg_start}")
        self._apply_bartender_sequence_snapshot(dict(snap or {}))
        self._update_bartender_mode_ui()

    def _advance_bartender_manual_sequence(self):
        if not self._bartender_sequence_running:
            return
        steps = [str(key) for key, _ in BARTENDER_SEQUENCE_STEPS]
        idx = int(self._bartender_manual_step_index) + 1
        self._bartender_manual_step_index = idx
        if idx >= len(steps):
            self._bartender_sequence_running = False
            self._bartender_active_step = ""
            if self._bartender_status_label is not None:
                self._bartender_status_label.setText("상태: 메뉴얼 시퀀스 완료")
                self._bartender_status_label.setStyleSheet("font-size: 10pt; font-weight: 800; color: #166534;")
            self._refresh_bartender_sequence_styles()
            self._append_voice_order_log("바텐더 메뉴얼 시퀀스 완료")
            return

        key = steps[idx]
        self._set_bartender_active_step(key)
        if key == "voice_request":
            self._capture_voice_input_once()

        delay_ms = {
            "boot": 600,
            "mode": 600,
            "voice_request": 900,
            "stt": 900,
            "llm": 900,
            "recipe": 700,
            "robot_action": 900,
            "vision_check": 700,
            "done": 500,
        }.get(key, 700)
        QTimer.singleShot(int(delay_ms), self._advance_bartender_manual_sequence)

    def _layout_bartender_overview_panel(self):
        panel = getattr(self, "_bartender_overview_panel", None)
        rect = getattr(self, "_bartender_overview_rect", None)
        if panel is None or rect is None or len(rect) != 4:
            return
        x, y, w, h = rect
        panel.setGeometry(int(x), int(y), int(w), int(h))
        panel.raise_()

        margin = 12
        title_top = 10
        title_h = 34
        ctrl_h = 26
        text_h = 22
        hint_h = 32

        if self._bartender_title_label is not None:
            self._bartender_title_label.setGeometry(margin, title_top, max(120, w - (margin * 2)), title_h)

        y0 = title_top + title_h + 8
        mode_btn_w = max(96, int((w - (margin * 2) - 8) / 2))
        if self._bartender_manual_mode_button is not None:
            self._bartender_manual_mode_button.setGeometry(margin, y0, mode_btn_w, ctrl_h)
        if self._bartender_auto_mode_button is not None:
            self._bartender_auto_mode_button.setGeometry(margin + mode_btn_w + 8, y0, mode_btn_w, ctrl_h)

        y0 += ctrl_h + 8
        settings_box = getattr(self, "_bartender_settings_box", None)
        settings_h = 84
        if settings_box is not None:
            settings_box.setGeometry(margin, y0, max(120, w - (margin * 2)), settings_h)
            inner_margin = 8
            inner_w = max(120, settings_box.width() - (inner_margin * 2))
            pose_h = 22
            if self._bartender_pose_button is not None:
                self._bartender_pose_button.setGeometry(inner_margin, inner_margin, inner_w, pose_h)
            row_y = inner_margin + pose_h + 8
            if self._bartender_speed_title_label is not None:
                self._bartender_speed_title_label.setGeometry(inner_margin, row_y, 152, 18)
            slider_x = inner_margin + 156
            slider_w = max(120, inner_w - 206)
            if self._bartender_speed_slider is not None:
                self._bartender_speed_slider.setGeometry(slider_x, row_y, slider_w, 18)
            if self._bartender_speed_value_label is not None:
                self._bartender_speed_value_label.setGeometry(
                    min(settings_box.width() - 42, slider_x + slider_w + 6),
                    row_y,
                    36,
                    18,
                )
        y0 += settings_h + 8

        start_h = 34
        if self._bartender_start_button is not None:
            self._bartender_start_button.setGeometry(margin, y0, max(120, w - (margin * 2)), start_h)

        y0 += start_h + 8
        if self._bartender_status_label is not None:
            self._bartender_status_label.setGeometry(margin, y0, max(120, w - (margin * 2)), text_h)
        y0 += text_h
        if self._bartender_mode_hint_label is not None:
            self._bartender_mode_hint_label.setGeometry(margin, y0, max(120, w - (margin * 2)), hint_h)
        y0 += hint_h + 6

        seq_area = getattr(self, "_bartender_sequence_area", None)
        if seq_area is None:
            return
        action_btn_h = 30
        action_btn_gap = 6
        seq_h = max(120, h - y0 - margin - action_btn_gap - action_btn_h)
        seq_area.setGeometry(margin, y0, max(120, w - (margin * 2)), seq_h)

        inner_margin = 8
        step_count = max(1, len(BARTENDER_SEQUENCE_STEPS))
        step_gap = 6
        usable_h = max(40, seq_h - (inner_margin * 2) - (step_gap * (step_count - 1)))
        step_h = min(
            int(BARTENDER_SEQUENCE_STEP_MAX_HEIGHT),
            max(22, int(usable_h / step_count)),
        )
        py = inner_margin
        for step_key, _step_label in BARTENDER_SEQUENCE_STEPS:
            lbl = self._bartender_sequence_widgets.get(str(step_key), None)
            if lbl is None:
                continue
            lbl.setGeometry(inner_margin, py, max(80, seq_area.width() - (inner_margin * 2)), step_h)
            py += step_h + step_gap

        if self._bartender_robot_action_test_button is not None:
            btn_y = seq_area.y() + seq_h + action_btn_gap
            self._bartender_robot_action_test_button.setGeometry(
                margin, btn_y, max(120, w - (margin * 2)), action_btn_h
            )

    def _refresh_bartender_sequence_styles(self):
        if not self._bartender_sequence_widgets:
            return

        phase = time.monotonic() % 1.0
        fade = 0.5 * (1.0 + np.sin((2.0 * np.pi * phase) - (np.pi / 2.0)))

        for step_key, _step_label in BARTENDER_SEQUENCE_STEPS:
            key = str(step_key)
            lbl = self._bartender_sequence_widgets.get(key, None)
            if lbl is None:
                continue
            state = str(self._bartender_sequence_state.get(key, "pending") or "pending").strip().lower()
            if state == "done":
                lbl.setStyleSheet(
                    "QLabel { background: #dcfce7; border: 1px solid #22c55e; border-radius: 4px; color: #166534; font-size: 8.6pt; font-weight: 900; padding-left: 7px; }"
                )
                continue
            if state == "error":
                lbl.setStyleSheet(
                    "QLabel { background: #fee2e2; border: 1px solid #ef4444; border-radius: 4px; color: #991b1b; font-size: 8.6pt; font-weight: 900; padding-left: 7px; }"
                )
                continue
            if state == "active":
                alpha = int(max(92, min(255, round(104 + (fade * 151)))))
                lbl.setStyleSheet(
                    f"QLabel {{ background: rgba(22, 163, 74, {alpha}); border: 1px solid #16a34a; border-radius: 4px; color: #ffffff; font-size: 8.6pt; font-weight: 900; padding-left: 7px; }}"
                )
                continue
            lbl.setStyleSheet(
                "QLabel { background: #ffffff; border: 1px solid #dbe2ea; border-radius: 4px; color: #334155; font-size: 8.6pt; font-weight: 700; padding-left: 7px; }"
            )

    def _robot_action_backend_call(self, method_name: str, *args, **kwargs):
        backend = getattr(self, "backend", None)
        if backend is None:
            return False, "backend 없음"
        fn = getattr(backend, str(method_name or ""), None)
        if not callable(fn):
            return False, f"backend 메서드 없음: {method_name}"
        try:
            ret = fn(*args, **kwargs)
        except Exception as exc:
            return False, f"{method_name} 예외: {exc}"
        if isinstance(ret, tuple) and len(ret) >= 2:
            return bool(ret[0]), str(ret[1])
        return bool(ret), ("완료" if bool(ret) else "실패")

    def _robot_action_call_with_retry(self, method_name: str, *args, timeout_sec: float = 8.0, poll_sec: float = 0.2, **kwargs):
        deadline = time.monotonic() + max(0.5, float(timeout_sec))
        last_msg = "실행 실패"
        while time.monotonic() <= deadline:
            ok, msg = self._robot_action_backend_call(method_name, *args, **kwargs)
            text = str(msg or "").strip()
            if ok:
                return True, text or "완료"
            last_msg = text or last_msg
            if ("현재 작업 중" in last_msg) or ("백엔드 준비 중" in last_msg):
                time.sleep(max(0.05, float(poll_sec)))
                continue
            return False, last_msg
        return False, f"{method_name} 타임아웃: {last_msg}"

    def _run_robot_action_step_for_order(self):
        result = self._voice_last_result if isinstance(self._voice_last_result, dict) else {}
        status = str(result.get("status", "") or "").strip().lower()
        if status != "success":
            return False, f"주문 상태가 success가 아닙니다: {status or '-'}"
        safety_issue = self._detect_bartender_runtime_safety_issue()
        if safety_issue:
            return False, f"안전정지 조건: {safety_issue}"

        speed = self._motion_speed_for_command()
        ok, msg = self._robot_action_call_with_retry(
            "run_bartender_first_ingredient_action",
            dict(result),
            motion_speed_percent=float(speed),
            timeout_sec=26.0,
            poll_sec=0.25,
        )
        if not ok:
            return False, str(msg or "백엔드 로봇동작 실패")
        return True, str(msg or "완료")

    def _update_bartender_sequence_from_voice_event(self, evt_type: str, event: dict):
        typ = str(evt_type or "").strip().lower()
        if typ == "stage":
            stage = str(event.get("stage", "") or "").strip().lower()
            if stage in ("input_request",):
                self._set_bartender_active_step("voice_request")
            elif stage in ("stt_wait", "stt_process", "stt"):
                self._set_bartender_active_step("stt")
            elif stage in ("classify", "llm"):
                self._set_bartender_active_step("llm")
            elif stage in ("recipe",):
                self._set_bartender_active_step("recipe")
            return
        if typ == "result":
            self._set_bartender_active_step("robot_action")
            return
        if typ == "done":
            ok = bool(event.get("ok", False))
            if not ok:
                if self._bartender_active_step:
                    self._set_bartender_step_state(self._bartender_active_step, "error")
                if self._bartender_status_label is not None:
                    self._bartender_status_label.setText("상태: 시퀀스 실패")
                    self._bartender_status_label.setStyleSheet("font-size: 10pt; font-weight: 800; color: #b91c1c;")
                return
            action_ok, action_msg = self._run_robot_action_step_for_order()
            if not action_ok:
                self._set_bartender_step_state("robot_action", "error")
                self._bartender_active_step = ""
                self._bartender_sequence_running = False
                if self._bartender_status_label is not None:
                    self._bartender_status_label.setText("상태: 시퀀스 실패(로봇동작)")
                    self._bartender_status_label.setStyleSheet("font-size: 10pt; font-weight: 800; color: #b91c1c;")
                self._append_voice_order_log(f"로봇동작 단계 실패: {action_msg}", level="error")
                self._refresh_bartender_sequence_styles()
                return

            self._append_voice_order_log(f"로봇동작 단계 완료: {action_msg}")
            self._set_bartender_step_state("robot_action", "done")
            self._set_bartender_active_step("vision_check")
            self._set_bartender_step_state("vision_check", "done")
            self._set_bartender_active_step("done")
            self._set_bartender_step_state("done", "done")
            self._bartender_active_step = ""
            self._bartender_sequence_running = False
            if self._bartender_status_label is not None:
                self._bartender_status_label.setText("상태: 시퀀스 완료")
                self._bartender_status_label.setStyleSheet("font-size: 10pt; font-weight: 800; color: #166534;")
            self._refresh_bartender_sequence_styles()

    def _layout_voice_order_panel(self):
        frame_l = getattr(self, "frame", None)
        panel = getattr(self, "_voice_order_panel", None)
        if frame_l is None or panel is None:
            return
        panel_rect = getattr(self, "_voice_panel_layout_rect", None)
        if panel_rect is not None and len(panel_rect) == 4:
            x, y, w, h = panel_rect
            panel.setGeometry(int(x), int(y), int(w), int(h))
        else:
            width = int(frame_l.width())
            x = max(10, int(frame_l.x()) - width - VOICE_ORDER_PANEL_GAP)
            panel.setGeometry(x, int(frame_l.y()), width, int(frame_l.height()))
        self._layout_voice_order_panel_widgets()
        panel.raise_()

    def _layout_voice_order_panel_widgets(self):
        panel = getattr(self, "_voice_order_panel", None)
        if panel is None:
            return
        w = int(panel.width())
        h = int(panel.height())
        margin = 10
        content_w = max(80, w - (margin * 2))

        y = 10
        if self._voice_order_connection_toggle is not None:
            self._voice_order_connection_toggle.setGeometry(margin, y, 71, 20)
        mic_test_w = 168
        mic_test_h = 34
        mic_test_x = max(margin, w - margin - mic_test_w)
        if self._voice_order_cycle_label is not None:
            self._voice_order_cycle_label.setGeometry(max(margin, w - margin - 168), y, 168, 16)
        y += 26
        if self._voice_order_connection_dot is not None:
            self._voice_order_connection_dot.setGeometry(0, y, 41, 41)
        right_reserved = (mic_test_w + 8) if self._voice_order_mic_test_panel is not None else 0
        if self._voice_order_connection_label is not None:
            self._voice_order_connection_label.setGeometry(50, max(0, y - 10), max(120, content_w - 170 - right_reserved), 61)
        if self._voice_order_mic_test_panel is not None:
            self._voice_order_mic_test_panel.setGeometry(mic_test_x, y + 18, mic_test_w, mic_test_h)
            if self._voice_order_mic_test_title_label is not None:
                self._voice_order_mic_test_title_label.setGeometry(6, 2, 98, 12)
            if self._voice_order_mic_test_value_label is not None:
                self._voice_order_mic_test_value_label.setGeometry(mic_test_w - 52, 2, 46, 12)
            if self._voice_order_mic_test_level_bar is not None:
                self._voice_order_mic_test_level_bar.setGeometry(6, 17, mic_test_w - 12, 10)
        y += 54
        if self._voice_order_input_label is not None:
            self._voice_order_input_label.setGeometry(margin, y, content_w, 20)
        y += 22

        input_h = max(62, int(h * 0.13))
        if self._voice_order_input_edit is not None:
            self._voice_order_input_edit.setGeometry(margin, y, content_w, input_h)
        y += input_h + 8

        if self._voice_order_llm_label is not None:
            self._voice_order_llm_label.setGeometry(margin, y, content_w, 20)
        y += 22

        llm_h = max(54, int(h * 0.10))
        if self._voice_order_llm_edit is not None:
            self._voice_order_llm_edit.setGeometry(margin, y, content_w, llm_h)
        y += llm_h + 8

        mic_h = 28
        if self._voice_order_mic_button is not None:
            self._voice_order_mic_button.setGeometry(margin, y, content_w, mic_h)
        y += mic_h + 6

        if self._voice_order_mic_state_label is not None:
            self._voice_order_mic_state_label.setGeometry(margin, y, content_w, 18)
        y += 22
        if self._voice_order_mic_level_bar is not None:
            self._voice_order_mic_level_bar.setGeometry(margin, y, content_w, 8)
        y += 14

        if self._voice_order_result_label is not None:
            self._voice_order_result_label.setGeometry(margin, y, content_w, 20)
        y += 22

        footer_reserved = 24 + 6 + 24 + 8
        recipe_h = int(max(72, h - y - footer_reserved))
        if self._voice_order_recipe_view is not None:
            self._voice_order_recipe_view.setGeometry(margin, y, content_w, recipe_h)
        y += recipe_h + 6

        if self._voice_order_webui_button is not None:
            self._voice_order_webui_button.setGeometry(margin, y, content_w, 24)
        y += 30

        if self._voice_order_html_badge is not None:
            self._voice_order_html_badge.setGeometry(margin, y, content_w, 24)

    def _set_voice_order_connection_state(self, connected: bool):
        on = bool(connected)
        self._voice_order_enabled = bool(on)
        label = getattr(self, "_voice_order_connection_label", None)
        dot = getattr(self, "_voice_order_connection_dot", None)
        toggle = getattr(self, "_voice_order_connection_toggle", None)
        if toggle is not None:
            toggle.blockSignals(True)
            toggle.setChecked(on)
            toggle.setText("ON" if on else "OFF")
            toggle.blockSignals(False)
        if on:
            self._start_voice_cycle_worker()
            self._set_voice_order_status("연결중", "warn")
            self._set_voice_order_mic_state("대기", "info")
        else:
            self._stop_voice_order_mic_test_monitor("음성주문 연결 OFF")
            self._stop_voice_cycle_worker(reason="음성주문 연결 OFF")
            self._voice_order_cycle_ms = None
            self._voice_prev_update_at = None
            self._voice_backend_last_seen_at = None
            self._voice_backend_poll_ok_at = None
            self._voice_order_process_state = "대기"
            self._set_voice_order_mic_state("비활성화", "warn")
            self._set_voice_order_mic_visual_active(False)
        if label is not None and dot is not None:
            self._set_voice_order_status("연결중" if on else "비활성화", "warn" if on else "warn")
        self._update_voice_mic_button_state()

    def _on_voice_order_connection_toggled(self, checked: bool):
        if str(getattr(self, "_bartender_mode", "") or "").strip().lower() == "auto":
            toggle = getattr(self, "_voice_order_connection_toggle", None)
            if toggle is not None:
                toggle.blockSignals(True)
                toggle.setChecked(bool(self._voice_order_enabled))
                toggle.setText("ON" if bool(self._voice_order_enabled) else "OFF")
                toggle.blockSignals(False)
            self._append_voice_order_log("오토모드에서는 음성주문 ON/OFF를 변경할 수 없습니다.", level="warning")
            return
        on = bool(checked)
        self._voice_order_enabled = bool(on)
        self._set_voice_order_connection_state(on)
        if on:
            self._set_voice_order_status("연결중", "warn")
        else:
            self._voice_order_cycle_ms = None
            self._voice_prev_update_at = None
            self._voice_backend_last_seen_at = None
            self._voice_backend_poll_ok_at = None
            self._set_voice_order_status("비활성화", "warn")
            self._stop_voice_order_worker(reason="음성주문 연결 OFF")
            self._voice_order_process_state = "대기"
            self._set_voice_order_mic_state("비활성화", "warn")
            self._set_voice_order_result(self._voice_last_result)
        self._update_voice_mic_button_state()
        self._update_cycle_time_labels()
        try:
            self._save_vision_serial_settings()
        except Exception:
            pass

    def _is_voice_order_worker_running(self) -> bool:
        if bool(getattr(self, "_voice_sequence_running", False)):
            return True
        if bool(self._voice_worker_running):
            return True
        th = getattr(self, "_voice_worker_thread", None)
        if th is None:
            return False
        try:
            return bool(th.is_alive()) and (not bool(self._voice_worker_done))
        except Exception:
            return False

    def _update_voice_mic_button_state(self):
        btn = getattr(self, "_voice_order_mic_button", None)
        if btn is None:
            return
        if str(getattr(self, "_bartender_mode", "") or "").strip().lower() == "auto":
            btn.setEnabled(False)
            btn.setText(VOICE_MIC_BUTTON_TEXT_START)
            return
        toggle = getattr(self, "_voice_order_connection_toggle", None)
        on = bool(toggle.isChecked()) if toggle is not None else True
        running = self._is_voice_order_worker_running()
        if not on:
            btn.setEnabled(False)
            btn.setText(VOICE_MIC_BUTTON_TEXT_START)
            return
        btn.setEnabled(True)
        btn.setText(VOICE_MIC_BUTTON_TEXT_STOP if running else VOICE_MIC_BUTTON_TEXT_START)

    def _stop_voice_order_feedback_audio(self, reason: str = ""):
        try:
            self._voice_feedback_audio_gen = int(getattr(self, "_voice_feedback_audio_gen", 0) or 0) + 1
        except Exception:
            self._voice_feedback_audio_gen = 1
        if sd is not None:
            try:
                sd.stop()
            except Exception:
                pass
        if reason:
            self._append_voice_order_log(f"TTS 재생 중지: {reason}", level="warning")

    def _set_voice_order_mic_state(self, state_text: str, severity: str = "info"):
        label = getattr(self, "_voice_order_mic_state_label", None)
        if label is None:
            return
        text = str(state_text or "").strip() or "대기"
        sev = str(severity or "info").strip().lower()
        self._voice_order_mic_state_text = text
        if sev in ("error", "fail", "failed"):
            color = "#b91c1c"
        elif sev in ("warn", "warning"):
            color = "#b45309"
        elif sev in ("ok", "success"):
            color = "#166534"
        else:
            color = "#1f2937"
        label.setText(f"마이크 상태: {text}")
        label.setStyleSheet(f"font-size: 9.2pt; font-weight: 700; color: {color};")

    def _set_voice_order_mic_visual_active(self, active: bool):
        bar = getattr(self, "_voice_order_mic_level_bar", None)
        timer = getattr(self, "_voice_order_mic_level_timer", None)
        if bar is None:
            return
        on = bool(active)
        self._voice_order_mic_level_active = on
        if not on:
            self._voice_order_mic_level_phase = 0.0
            self._voice_order_mic_level_last_at = 0.0
            if timer is not None:
                try:
                    timer.stop()
                except Exception:
                    pass
            bar.setValue(0)
            self._set_voice_order_mic_test_level(0)
            bar.setStyleSheet(
                "QProgressBar { border: 1px solid #cbd5e1; border-radius: 3px; background: #e5e7eb; }"
                "QProgressBar::chunk { background: #94a3b8; border-radius: 3px; }"
            )
            test_bar = getattr(self, "_voice_order_mic_test_level_bar", None)
            if test_bar is not None:
                test_bar.setStyleSheet(
                    "QProgressBar { border: 1px solid #cbd5e1; border-radius: 2px; background: #e5e7eb; }"
                    "QProgressBar::chunk { background: #94a3b8; border-radius: 2px; }"
                )
            return
        bar.setStyleSheet(
            "QProgressBar { border: 1px solid #86efac; border-radius: 3px; background: #ecfdf5; }"
            "QProgressBar::chunk { background: #16a34a; border-radius: 3px; }"
        )
        test_bar = getattr(self, "_voice_order_mic_test_level_bar", None)
        if test_bar is not None:
            test_bar.setStyleSheet(
                "QProgressBar { border: 1px solid #86efac; border-radius: 2px; background: #ecfdf5; }"
                "QProgressBar::chunk { background: #16a34a; border-radius: 2px; }"
            )
        if timer is not None:
            try:
                if not timer.isActive():
                    timer.start()
            except Exception:
                pass
        if int(bar.value()) <= 0:
            bar.setValue(5)

    def _set_voice_order_mic_level(self, level):
        bar = getattr(self, "_voice_order_mic_level_bar", None)
        if bar is None:
            return
        if not bool(getattr(self, "_voice_order_mic_level_active", False)):
            return
        try:
            value = int(max(0, min(100, round(float(level)))))
        except Exception:
            value = 0
        self._voice_order_mic_level_last_at = time.monotonic()
        bar.setValue(value)
        self._set_voice_order_mic_test_level(value)

    def _set_voice_order_mic_test_level(self, value: int):
        try:
            v = int(max(0, min(100, int(value))))
        except Exception:
            v = 0
        test_bar = getattr(self, "_voice_order_mic_test_level_bar", None)
        if test_bar is not None:
            test_bar.setValue(v)
        test_value = getattr(self, "_voice_order_mic_test_value_label", None)
        if test_value is not None:
            test_value.setText(f"{int(v)}%")

    def _on_voice_order_mic_test_clicked(self, event):
        self._toggle_voice_order_mic_test_monitor()
        if event is not None:
            try:
                event.accept()
            except Exception:
                pass

    def _toggle_voice_order_mic_test_monitor(self):
        if bool(getattr(self, "_voice_order_mic_test_running", False)):
            self._stop_voice_order_mic_test_monitor("사용자 테스트 중지")
        else:
            self._start_voice_order_mic_test_monitor()

    def _start_voice_order_mic_test_monitor(self):
        if bool(getattr(self, "_voice_order_mic_test_running", False)):
            return
        self._voice_order_mic_test_stop_event.clear()
        with self._voice_order_mic_test_lock:
            self._voice_order_mic_test_latest_level = 0
            self._voice_order_mic_test_error = ""
        self._voice_order_mic_test_selected_index = None
        self._voice_order_mic_test_selected_name = ""
        self._voice_order_mic_test_running = True
        title = getattr(self, "_voice_order_mic_test_title_label", None)
        if title is not None:
            title.setText("마이크 테스트(ON)")
        self._set_voice_order_mic_visual_active(True)
        self._set_voice_order_mic_state("테스트중", "warn")
        test_poll = getattr(self, "_voice_order_mic_test_poll_timer", None)
        if test_poll is not None and (not test_poll.isActive()):
            test_poll.start()
        self._append_voice_order_log("[mic_test] 시작")
        th = threading.Thread(target=self._run_voice_order_mic_test_monitor, daemon=True, name="voice-mic-test-monitor")
        self._voice_order_mic_test_thread = th
        th.start()

    def _stop_voice_order_mic_test_monitor(self, reason: str = ""):
        if not bool(getattr(self, "_voice_order_mic_test_running", False)):
            return
        self._voice_order_mic_test_running = False
        self._voice_order_mic_test_stop_event.set()
        th = getattr(self, "_voice_order_mic_test_thread", None)
        if th is not None:
            try:
                th.join(timeout=0.6)
            except Exception:
                pass
        self._voice_order_mic_test_thread = None
        test_poll = getattr(self, "_voice_order_mic_test_poll_timer", None)
        if test_poll is not None:
            try:
                test_poll.stop()
            except Exception:
                pass
        with self._voice_order_mic_test_lock:
            self._voice_order_mic_test_latest_level = 0
        self._set_voice_order_mic_test_level(0)
        title = getattr(self, "_voice_order_mic_test_title_label", None)
        if title is not None:
            title.setText("마이크 테스트")
        if not self._is_voice_order_worker_running():
            self._set_voice_order_mic_visual_active(False)
            self._set_voice_order_mic_state("대기", "info")
        if reason:
            self._append_voice_order_log(f"[mic_test] 중지: {reason}")

    def _poll_voice_order_mic_test_level(self):
        if not bool(getattr(self, "_voice_order_mic_test_running", False)):
            return
        with self._voice_order_mic_test_lock:
            level = int(self._voice_order_mic_test_latest_level)
            err = str(self._voice_order_mic_test_error or "").strip()
            self._voice_order_mic_test_error = ""
        self._set_voice_order_mic_level(level)
        if err:
            self._append_voice_order_log(f"[mic_test] 오류: {err}", level="error")
            self._stop_voice_order_mic_test_monitor("장치 오류")

    def _run_voice_order_mic_test_monitor(self):
        try:
            import speech_recognition as sr
        except Exception as exc:
            with self._voice_order_mic_test_lock:
                self._voice_order_mic_test_error = f"speech_recognition 미설치: {exc}"
            return
        rec = sr.Recognizer()
        try:
            mic_names = list(sr.Microphone.list_microphone_names() or [])
        except Exception:
            mic_names = []
        # STT 워커와 동일한 우선순위로 장치를 선택해 테스트/실운영 경로의 불일치를 줄인다.
        dev_index = None
        dev_name = "default"
        raw_index = str(os.environ.get("VOICE_ORDER_MIC_DEVICE_INDEX", "") or "").strip()
        if raw_index:
            try:
                parsed = int(raw_index)
                if 0 <= parsed < len(mic_names):
                    dev_index = int(parsed)
                    dev_name = f"index:{dev_index}"
            except Exception:
                dev_index = None
        if dev_index is None:
            hint = str(os.environ.get("VOICE_ORDER_MIC_DEVICE_HINT", "") or "").strip().lower()
            if hint:
                for idx, name in enumerate(mic_names):
                    if hint in str(name or "").lower():
                        dev_index = int(idx)
                        dev_name = str(name or f"index:{idx}")
                        break
        if dev_index is None:
            auto_scan = str(os.environ.get("VOICE_ORDER_MIC_AUTO_SCAN", "0") or "").strip().lower() in (
                "1",
                "true",
                "yes",
                "on",
            )
            if auto_scan:
                for token in ("pipewire", "pulse", "usb", "mic", "default"):
                    matched = False
                    for idx, name in enumerate(mic_names):
                        if token in str(name or "").lower():
                            dev_index = int(idx)
                            dev_name = str(name or f"index:{idx}")
                            matched = True
                            break
                    if matched:
                        break
        level_gate = int(max(0.0, min(60.0, float(os.environ.get("VOICE_ORDER_MIC_LEVEL_GATE", "2") or "2"))))
        level_ref = max(400.0, float(os.environ.get("VOICE_ORDER_MIC_LEVEL_REF", "3000") or "3000"))
        try:
            self._append_voice_order_log(
                f"[mic_test] 장치 선택: index={dev_index if dev_index is not None else 'default'}, name={dev_name}"
            )
            # 진단용 선택값만 보관한다. STT 경로의 장치 환경변수는 변경하지 않는다.
            self._voice_order_mic_test_selected_index = dev_index
            self._voice_order_mic_test_selected_name = str(dev_name or "")
            with sr.Microphone(device_index=dev_index) as source:
                try:
                    rec.adjust_for_ambient_noise(source, duration=0.12)
                except Exception:
                    pass
                while not self._voice_order_mic_test_stop_event.is_set():
                    chunk = source.stream.read(source.CHUNK)
                    if not isinstance(chunk, (bytes, bytearray)) or len(chunk) <= 0:
                        level = 0
                    else:
                        samples = np.frombuffer(bytes(chunk), dtype=np.int16)
                        if samples.size <= 0:
                            rms = 0.0
                        else:
                            rms = float(np.sqrt(np.mean(np.square(samples.astype(np.float32)))))
                        if rms <= 0.0:
                            level = 0
                        else:
                            normalized = min(1.0, float(rms) / float(level_ref))
                            level = int(max(0, min(100, round((normalized ** 0.65) * 100.0))))
                    if int(level) < int(level_gate):
                        level = 0
                    with self._voice_order_mic_test_lock:
                        self._voice_order_mic_test_latest_level = int(level)
                    time.sleep(0.06)
        except Exception as exc:
            with self._voice_order_mic_test_lock:
                self._voice_order_mic_test_error = str(exc)

    def _tick_voice_order_mic_level(self):
        bar = getattr(self, "_voice_order_mic_level_bar", None)
        if bar is None:
            return
        if not bool(getattr(self, "_voice_order_mic_level_active", False)):
            bar.setValue(0)
            return
        now = time.monotonic()
        last_at = float(getattr(self, "_voice_order_mic_level_last_at", 0.0) or 0.0)
        if last_at <= 0.0:
            return
        if (now - last_at) <= 0.22:
            return
        current = int(bar.value())
        if current <= 0:
            return
        decayed = max(0, current - 9)
        bar.setValue(decayed)
        self._set_voice_order_mic_test_level(decayed)

    def _set_voice_order_input_text(self, text: str):
        view = getattr(self, "_voice_order_input_edit", None)
        if view is None:
            return
        value = str(text or "").strip()
        if value == self._voice_order_last_input_text:
            return
        self._voice_order_last_input_text = value
        view.setPlainText(value)

    def _set_voice_order_llm_text(self, text: str):
        view = getattr(self, "_voice_order_llm_edit", None)
        if view is None:
            return
        value = str(text or "").strip() or "-"
        if value == self._voice_order_last_llm_text:
            return
        self._voice_order_last_llm_text = value
        view.setPlainText(value)

    def _mark_voice_order_update(self, seen_at=None):
        now = time.monotonic() if seen_at is None else float(seen_at)
        prev = self._voice_prev_update_at
        if prev is not None:
            dt = now - float(prev)
            if dt > 0.0:
                self._voice_order_cycle_ms = dt * 1000.0
        self._voice_prev_update_at = now
        self._update_cycle_time_labels()

    def _on_voice_cycle_tick(self, _tick_at: float, token: int):
        if getattr(self, "_closing", False):
            return
        if int(token) != int(self._voice_cycle_worker_token):
            return
        toggle = getattr(self, "_voice_order_connection_toggle", None)
        if toggle is None or (not bool(toggle.isChecked())):
            return
        backend = getattr(self, "backend", None)
        if backend is None or (not hasattr(backend, "get_voice_order_snapshot")):
            return
        try:
            payload, backend_seen_at = backend.get_voice_order_snapshot()
        except Exception:
            self._set_voice_order_status("끊김", "error")
            return
        self._voice_backend_poll_ok_at = float(_tick_at)
        # 주문 결과가 아직 없으면 폴링 하트비트 간격을 표시한다.
        if payload is None or backend_seen_at is None:
            self._mark_voice_order_update(seen_at=float(_tick_at))
            if self._voice_order_cycle_ms is None:
                self._set_voice_order_status("연결중", "warn")
            elif float(self._voice_order_cycle_ms) >= VOICE_ORDER_DISCONNECT_MS:
                self._set_voice_order_status("끊김", "error")
            elif float(self._voice_order_cycle_ms) >= VOICE_ORDER_DELAY_WARN_MS:
                self._set_voice_order_status("지연", "warn")
            else:
                self._set_voice_order_status("정상연결", "ok")
            return
        backend_seen_at = float(backend_seen_at)
        prev_seen_at = self._voice_backend_last_seen_at
        if prev_seen_at is not None and abs(backend_seen_at - float(prev_seen_at)) < 1e-9:
            self._mark_voice_order_update(seen_at=float(_tick_at))
            if self._voice_order_cycle_ms is not None:
                if float(self._voice_order_cycle_ms) >= VOICE_ORDER_DISCONNECT_MS:
                    self._set_voice_order_status("끊김", "error")
                elif float(self._voice_order_cycle_ms) >= VOICE_ORDER_DELAY_WARN_MS:
                    self._set_voice_order_status("지연", "warn")
                else:
                    self._set_voice_order_status("정상연결", "ok")
            return
        self._voice_backend_last_seen_at = backend_seen_at
        self._mark_voice_order_update(seen_at=float(_tick_at))
        if self._voice_order_cycle_ms is None:
            self._set_voice_order_status("정상연결", "ok")
        elif float(self._voice_order_cycle_ms) >= VOICE_ORDER_DISCONNECT_MS:
            self._set_voice_order_status("끊김", "error")
        elif float(self._voice_order_cycle_ms) >= VOICE_ORDER_DELAY_WARN_MS:
            self._set_voice_order_status("지연", "warn")
        else:
            self._set_voice_order_status("정상연결", "ok")

    def _start_voice_cycle_worker(self):
        if getattr(self, "_closing", False):
            return
        thread = getattr(self, "_voice_cycle_thread", None)
        if thread is not None and thread.isRunning():
            return
        self._voice_cycle_worker_token += 1
        token = int(self._voice_cycle_worker_token)
        worker = VoiceOrderCycleWorker(VOICE_ORDER_CYCLE_INTERVAL_MS)
        thread = QThread(self)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.finished.connect(thread.quit)
        worker.tick.connect(lambda ts, _token=token: self._on_voice_cycle_tick(ts, _token))
        thread.start()
        self._voice_cycle_worker = worker
        self._voice_cycle_thread = thread
        self._append_voice_order_log(f"업데이트 워커 시작(interval={VOICE_ORDER_CYCLE_INTERVAL_MS}ms)")

    def _stop_voice_cycle_worker(self, reason: str = ""):
        worker = getattr(self, "_voice_cycle_worker", None)
        thread = getattr(self, "_voice_cycle_thread", None)
        if worker is None and thread is None:
            return

        if worker is not None:
            try:
                worker.tick.disconnect()
            except Exception:
                pass
            try:
                worker.stop()
            except Exception:
                pass
            try:
                worker.deleteLater()
            except Exception:
                pass
        if thread is not None:
            try:
                thread.quit()
            except Exception:
                pass
            if thread.isRunning():
                if not thread.wait(1200):
                    thread.requestInterruption()
                    thread.wait(400)
            try:
                thread.deleteLater()
            except Exception:
                pass

        self._voice_cycle_worker = None
        self._voice_cycle_thread = None
        if reason:
            self._append_voice_order_log(f"업데이트 워커 중지: {reason}")

    def _voice_order_webui_url(self):
        host = str(VOICE_ORDER_WEBUI_HOST or "").strip() or "127.0.0.1"
        if host in ("0.0.0.0", "::"):
            host = "127.0.0.1"
        port = str(VOICE_ORDER_WEBUI_PORT or "").strip() or "8000"
        return f"http://{host}:{port}"

    def _ensure_webui_order_start_sync_worker(self):
        if getattr(self, "_closing", False):
            return
        with self._webui_order_start_sync_cv:
            thread = self._webui_order_start_sync_thread
            if thread is not None and thread.is_alive():
                return
            self._webui_order_start_sync_stop.clear()
            self._webui_order_start_sync_thread = threading.Thread(
                target=self._webui_order_start_sync_loop,
                name="webui-order-start-sync",
                daemon=True,
            )
            self._webui_order_start_sync_thread.start()

    def _stop_webui_order_start_sync_worker(self):
        self._webui_order_start_sync_stop.set()
        with self._webui_order_start_sync_cv:
            self._webui_order_start_sync_cv.notify_all()
        thread = getattr(self, "_webui_order_start_sync_thread", None)
        if thread is not None and thread.is_alive():
            thread.join(timeout=0.8)
        self._webui_order_start_sync_thread = None

    def _webui_order_start_sync_loop(self):
        while not self._webui_order_start_sync_stop.is_set():
            with self._webui_order_start_sync_cv:
                while (
                    (not self._webui_order_start_sync_stop.is_set())
                    and (self._webui_order_start_enabled_pending is None)
                ):
                    self._webui_order_start_sync_cv.wait(timeout=0.5)
                if self._webui_order_start_sync_stop.is_set():
                    return
                target = bool(self._webui_order_start_enabled_pending)
                self._webui_order_start_enabled_pending = None

            now = time.monotonic()
            wait_sec = float(WEBUI_ORDER_START_SYNC_MIN_INTERVAL_SEC) - (
                now - float(self._webui_order_start_sync_last_try_at)
            )
            if wait_sec > 0.0:
                if self._webui_order_start_sync_stop.wait(wait_sec):
                    return
                with self._webui_order_start_sync_cv:
                    if self._webui_order_start_enabled_pending is not None:
                        target = bool(self._webui_order_start_enabled_pending)
                        self._webui_order_start_enabled_pending = None

            self._webui_order_start_sync_last_try_at = time.monotonic()
            url = f"{self._voice_order_webui_url()}/api/control/order_start_enabled"
            payload = json.dumps({"enabled": bool(target)}, ensure_ascii=False).encode("utf-8")
            req = urllib_request.Request(
                url=url,
                data=payload,
                method="POST",
                headers={"Content-Type": "application/json; charset=utf-8"},
            )
            try:
                with urllib_request.urlopen(req, timeout=float(WEBUI_ORDER_START_HTTP_TIMEOUT_SEC)) as resp:
                    if int(getattr(resp, "status", 200)) >= 400:
                        raise RuntimeError(f"http_{getattr(resp, 'status', '-')}")
                with self._webui_order_start_sync_cv:
                    self._webui_order_start_enabled_cached = bool(target)
                    self._webui_order_start_sync_error_last = ""
                    self._webui_order_start_sync_error_at = 0.0
            except urllib_error.URLError:
                continue
            except Exception as exc:
                with self._webui_order_start_sync_cv:
                    self._webui_order_start_sync_error_last = str(exc)
                    self._webui_order_start_sync_error_at = time.monotonic()

    def _set_webui_order_start_enabled(self, enabled: bool, force: bool = False):
        target = bool(enabled)
        with self._webui_order_start_sync_cv:
            cached = getattr(self, "_webui_order_start_enabled_cached", None)
            pending = getattr(self, "_webui_order_start_enabled_pending", None)
            if (not force) and (cached is not None) and (bool(cached) == target) and (pending is None):
                return
            if force:
                self._webui_order_start_enabled_cached = None
            self._webui_order_start_enabled_pending = target
            self._webui_order_start_sync_cv.notify_all()
        self._ensure_webui_order_start_sync_worker()

    def _notify_bartender_tts_done(self):
        run_id = int(getattr(self, "_voice_sequence_run_id", 0) or 0)
        if run_id <= 0:
            run_id = int(getattr(self, "_bartender_sequence_run_id", 0) or 0)
        if run_id <= 0:
            return

        backend = getattr(self, "backend", None)
        if backend is not None and hasattr(backend, "notify_bartender_tts_done"):
            try:
                ok, msg, _snap = backend.notify_bartender_tts_done(run_id=run_id)
                if not ok:
                    self._append_voice_order_log(f"TTS 완료 플래그 반영 실패: {msg}", level="warning")
                return
            except Exception as exc:
                self._append_voice_order_log(f"TTS 완료 플래그 반영 예외: {exc}", level="warning")
                return

    def _refresh_voice_order_webui_badge(self):
        label = getattr(self, "_voice_order_html_badge", None)
        if label is None:
            return
        label.setText("주문 WEB UI: 비활성화 (프로세스 실행중)")

    def _open_voice_order_webui(self):
        url = self._voice_order_webui_url()
        ok = bool(QDesktopServices.openUrl(QUrl(url)))
        if ok:
            self._append_voice_order_log(f"WEB UI 열기: {url}")
        else:
            self._append_voice_order_log(f"WEB UI 열기 실패: {url}")

    def _capture_voice_input_once(self):
        if str(getattr(self, "_bartender_mode", "") or "").strip().lower() == "auto":
            self._append_voice_order_log("오토모드에서는 개발자 UI 마이크 시작을 사용할 수 없습니다. WEB UI에서 시작하세요.", level="warning")
            return
        if bool(getattr(self, "_voice_order_mic_test_running", False)):
            self._stop_voice_order_mic_test_monitor("STT 입력 시작")
        toggle = getattr(self, "_voice_order_connection_toggle", None)
        if toggle is not None and (not bool(toggle.isChecked())):
            self._append_voice_order_log("마이크 입력 요청 거부: 음성주문 비활성화 상태")
            return
        backend = getattr(self, "backend", None)
        if bool(getattr(self, "_voice_sequence_running", False)) and backend is not None and hasattr(
            backend, "stop_bartender_sequence"
        ):
            self._append_voice_order_log("음성주문 시퀀스 중지 요청")
            ok_stop, msg_stop, snap = backend.stop_bartender_sequence(reason="음성주문 입력 중지")
            self._append_voice_order_log(
                f"음성주문 시퀀스 중지 {'완료' if ok_stop else '실패'}: {msg_stop}",
                level=("warning" if ok_stop else "error"),
            )
            if isinstance(snap, dict):
                self._apply_bartender_sequence_snapshot(snap)
            return
        if bool(getattr(self, "_bartender_sequence_running", False)) and backend is not None and hasattr(
            backend, "stop_bartender_sequence"
        ):
            self._append_voice_order_log("시퀀스 중지 요청: 음성주문 재입력")
            ok_stop, msg_stop, snap = backend.stop_bartender_sequence(reason="음성주문 재입력")
            self._append_voice_order_log(
                f"시퀀스 중지 {'완료' if ok_stop else '실패'}: {msg_stop}",
                level=("warning" if ok_stop else "error"),
            )
            if isinstance(snap, dict):
                self._apply_bartender_sequence_snapshot(snap)
            self._update_bartender_mode_ui()
            return
        if self._is_voice_order_worker_running():
            self._append_voice_order_log("마이크 입력 중지 요청")
            self._stop_voice_order_worker(reason="사용자 중지 요청")
            self._set_voice_order_process_state("중단")
            self._set_voice_order_mic_state("중단됨", "warn")
            self._set_voice_order_mic_visual_active(False)
            return
        self._voice_ready_tone_armed = True
        self._set_voice_order_process_state("입력요청중")
        self._set_voice_order_mic_state("입력 요청중", "warn")
        self._set_voice_order_mic_visual_active(False)
        QApplication.processEvents()
        self._start_voice_order_test(request_stt=True)

    def _set_voice_order_status(self, text: str, severity: str = "info"):
        label = getattr(self, "_voice_order_connection_label", None)
        dot = getattr(self, "_voice_order_connection_dot", None)
        if label is None or dot is None:
            return
        state_text = str(text or "").strip() or "-"
        sev = str(severity or "info").strip().lower()
        self._voice_order_status_text = state_text
        self._voice_order_status_severity = sev

        if sev in ("ok", "normal", "success"):
            bright = STATE_COLOR_NORMAL
        elif sev in ("error", "fail", "failed"):
            bright = STATE_COLOR_ERROR
        else:
            bright = STATE_COLOR_WARNING
        dim = "#ffffff"

        # 음성주문 상태도 상단 상태등처럼 점멸 표현을 적용한다.
        phase = time.monotonic() % 1.0
        fade = 0.5 * (1.0 + np.sin((2.0 * np.pi * phase) - (np.pi / 2.0)))
        should_blink = True

        def _hex_to_rgb(h: str):
            h = str(h or "#000000").lstrip("#")
            if len(h) != 6:
                return 0, 0, 0
            return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)

        def _mix(c1: str, c2: str, t: float):
            r1, g1, b1 = _hex_to_rgb(c1)
            r2, g2, b2 = _hex_to_rgb(c2)
            return (
                int(max(0, min(255, round(r1 + (r2 - r1) * t)))),
                int(max(0, min(255, round(g1 + (g2 - g1) * t)))),
                int(max(0, min(255, round(b1 + (b2 - b1) * t)))),
            )

        if should_blink:
            r, g, b = _mix(dim, bright, fade)
            alpha = int(max(0, min(255, round(255.0 * fade))))
        else:
            r, g, b = _hex_to_rgb(bright)
            alpha = 255

        label.setText(f"음성주문: {state_text}")
        label.setStyleSheet("font-size: 14pt; font-weight: 700; color: #202020;")
        dot.setStyleSheet(f"font-size: 20pt; color: rgba({r}, {g}, {b}, {alpha});")

    def _append_voice_order_log(self, message: str, level: str = "info"):
        text = str(message or "").strip()
        if not text:
            return
        lv = str(level or "info").strip().lower()
        if lv in ("warn", "warning"):
            prefix = "[음성주문][경고]"
        elif lv in ("error", "err", "fail", "failed"):
            prefix = "[음성주문][오류]"
        else:
            prefix = "[음성주문]"
        self.append_log(f"{prefix} {text}\n")

    def _set_voice_order_process_state(self, state_text: str):
        next_state = str(state_text or "").strip() or "대기"
        # STT 청취 진행 중에는 레이스로 "대기"가 덮어쓰지 못하도록 막는다.
        if next_state == "대기":
            mic_state = str(getattr(self, "_voice_order_mic_state_text", "") or "").strip()
            if mic_state == "입력중":
                return
            if bool(getattr(self, "_voice_sequence_running", False)) and bool(getattr(self, "_voice_ready_tone_armed", False)):
                return
        self._voice_order_process_state = next_state
        self._set_voice_order_result(self._voice_last_result)

    def _voice_order_result_text(self, status: str) -> str:
        value = str(status or "").strip().lower()
        if value == "success":
            return "성공"
        if value == "retry":
            return "재입력 필요"
        if value == "error":
            return "실패"
        if value in ("-", ""):
            return "-"
        return value

    def _set_voice_order_result(self, payload: dict | None):
        view = getattr(self, "_voice_order_recipe_view", None)
        if view is None:
            return
        if not payload:
            self._set_voice_order_llm_text("-")
            view.setPlainText(
                f"처리상태: {self._voice_order_process_state}\n"
                "도출결과: -\n"
                "메뉴: -\n"
                "레시피: -"
            )
            return
        menu_label = str(payload.get("selected_menu_label") or "").strip()
        menu_code = str(payload.get("selected_menu") or "").strip()
        if not menu_label:
            menu_label = menu_code or "-"
        status = str(payload.get("status") or "-")
        llm_text = str(payload.get("llm_text") or payload.get("tts_text") or "-")
        llm_text = self._sanitize_customer_tts_text(status=status, raw_text=llm_text)
        recipe = payload.get("recipe", {})
        if isinstance(recipe, dict) and recipe:
            recipe_text = ", ".join(f"{k}: {v}ml" for k, v in recipe.items())
        else:
            recipe_text = "-"
        self._set_voice_order_llm_text(llm_text)
        view.setPlainText(
            f"처리상태: {self._voice_order_process_state}\n"
            f"도출결과: {self._voice_order_result_text(status)}\n"
            f"메뉴: {menu_label}\n"
            f"레시피: {recipe_text}"
        )

    def _sanitize_customer_tts_text(self, status: str, raw_text: str) -> str:
        st = str(status or "").strip().lower()
        text = str(raw_text or "").strip()
        if st == "error":
            return str(VOICE_ORDER_CUSTOMER_ERROR_TTS_TEXT)
        return text or "-"

    def _play_voice_order_ready_tone(self):
        now = time.monotonic()
        # 버튼 클릭 직후와 stt_wait 이벤트가 근접해서 들어올 수 있어 중복음을 제한한다.
        if (now - float(getattr(self, "_voice_ready_tone_last_at", 0.0))) < 0.35:
            return
        self._voice_ready_tone_last_at = float(now)

        play_ms = int(max(80, min(400, int(float(os.environ.get("VOICE_ORDER_READY_BEEP_MS", "170"))))))
        freq_hz = float(os.environ.get("VOICE_ORDER_READY_BEEP_HZ", "1180") or "1180")
        gain = float(os.environ.get("VOICE_ORDER_READY_BEEP_GAIN", "0.35") or "0.35")
        gain = max(0.08, min(0.95, gain))
        duration_sec = float(play_ms) / 1000.0
        sample_rate = 16000
        samples = np.linspace(0.0, duration_sec, int(sample_rate * duration_sec), endpoint=False, dtype=np.float32)
        tone = (gain * np.sin(2.0 * np.pi * freq_hz * samples)).astype(np.float32)
        wave_buf = tone

        def _play_worker():
            if sd is not None:
                try:
                    if not bool(getattr(self, "_voice_ready_tone_warmed", False)):
                        warm = np.zeros(int(sample_rate * 0.04), dtype=np.float32)
                        sd.play(warm, sample_rate, blocking=True)
                        sd.stop()
                        self._voice_ready_tone_warmed = True
                    sd.play(wave_buf, sample_rate, blocking=True)
                    sd.stop()
                    return
                except Exception:
                    pass
            try:
                QApplication.beep()
            except Exception:
                pass

        threading.Thread(target=_play_worker, daemon=True).start()

    def _play_voice_order_feedback_tone(self, tts_text: str, on_complete=None):
        text = str(tts_text or "").strip()
        if not text:
            self._set_voice_order_process_state("대기")
            if callable(on_complete):
                try:
                    on_complete()
                except Exception:
                    pass
            return
        play_gen = int(getattr(self, "_voice_feedback_audio_gen", 0) or 0)
        play_ms = int(max(520, min(5200, 320 + (len(text) * 95))))
        self._set_voice_order_process_state("응답 음성 출력중")

        def _play_wav_bytes(wav_bytes: bytes):
            if sd is None:
                return False
            try:
                with io.BytesIO(bytes(wav_bytes or b"")) as bio:
                    with wave.open(bio, "rb") as wav_fp:
                        channels = int(wav_fp.getnchannels())
                        sample_width = int(wav_fp.getsampwidth())
                        sample_rate = int(wav_fp.getframerate())
                        frame_count = int(wav_fp.getnframes())
                        pcm_bytes = wav_fp.readframes(frame_count)
                if sample_width == 1:
                    samples = np.frombuffer(pcm_bytes, dtype=np.uint8).astype(np.float32)
                    samples = (samples - 128.0) / 128.0
                elif sample_width == 2:
                    samples = np.frombuffer(pcm_bytes, dtype=np.int16).astype(np.float32) / 32768.0
                elif sample_width == 4:
                    samples = np.frombuffer(pcm_bytes, dtype=np.int32).astype(np.float32) / 2147483648.0
                else:
                    return False
                if samples.size <= 0:
                    return False
                if channels > 1:
                    samples = samples.reshape(-1, channels)
                sd.play(samples, sample_rate, blocking=True)
                sd.stop()
                return True
            except Exception:
                return False

        def _play_worker():
            def _is_canceled() -> bool:
                return int(getattr(self, "_voice_feedback_audio_gen", 0) or 0) != int(play_gen)

            def _emit_tts_log(msg: str, level: str = "info"):
                try:
                    self.voice_order_event_received.emit(
                        {
                            "type": "tts_log",
                            "message": str(msg or "").strip(),
                            "level": str(level or "info").strip().lower(),
                        }
                    )
                except Exception:
                    pass

            try:
                if _is_canceled():
                    return
                spoken = False
                use_openai_tts = str(os.environ.get("VOICE_ORDER_USE_OPENAI_TTS", "1") or "").strip().lower()
                use_openai_tts = use_openai_tts not in ("0", "false", "no", "off")

                if use_openai_tts:
                    if _is_canceled():
                        return
                    try:
                        tts_result = synthesize_openai_tts(text)
                        spoken = _play_wav_bytes(tts_result.audio_bytes)
                        if spoken:
                            _emit_tts_log(
                                (
                                    "OpenAI TTS 출력 완료 "
                                    f"(model={tts_result.model}, voice={tts_result.voice}, speed={tts_result.speed:.2f})"
                                ),
                                level="info",
                            )
                    except Exception as exc:
                        err_text = str(exc or "").strip() or "unknown_error"
                        if ("insufficient_quota" in err_text.lower()) or (" 429" in f" {err_text}") or ("429" in err_text):
                            _emit_tts_log(f"OpenAI TTS 실패(쿼터 부족/결제 확인 필요): {err_text}", level="warning")
                        else:
                            _emit_tts_log(f"OpenAI TTS 실패: {err_text}", level="warning")
                        spoken = False
                    if spoken:
                        return
                else:
                    _emit_tts_log("OpenAI TTS 비활성화: fallback 출력 사용", level="warning")

                use_spd_say_fallback = str(os.environ.get("VOICE_ORDER_TTS_FALLBACK_SPD_SAY", "0") or "").strip().lower()
                use_spd_say_fallback = use_spd_say_fallback in ("1", "true", "yes", "on")
                if use_spd_say_fallback:
                    if _is_canceled():
                        return
                    try:
                        result = subprocess.run(
                            ["spd-say", "-w", text],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                            timeout=max(2.0, float(play_ms) / 1000.0 + 1.5),
                            check=False,
                        )
                        spoken = (int(getattr(result, "returncode", 1)) == 0)
                    except Exception:
                        spoken = False
                    if spoken:
                        _emit_tts_log("fallback 음성 출력(spd-say) 사용", level="warning")

                    if spoken:
                        return

                use_tone_fallback = str(os.environ.get("VOICE_ORDER_TTS_FALLBACK_TONE", "1") or "").strip().lower()
                use_tone_fallback = use_tone_fallback in ("1", "true", "yes", "on")
                if not use_tone_fallback:
                    _emit_tts_log("TTS 음성 출력 실패: fallback 톤 비활성화", level="error")
                    return

                if sd is not None:
                    if _is_canceled():
                        return
                    try:
                        duration_sec = float(play_ms) / 1000.0
                        freq_hz = 660.0
                        sample_rate = 16000
                        samples = np.linspace(
                            0.0,
                            duration_sec,
                            int(sample_rate * duration_sec),
                            endpoint=False,
                            dtype=np.float32,
                        )
                        wave = (0.22 * np.sin(2.0 * np.pi * freq_hz * samples)).astype(np.float32)
                        sd.play(wave, sample_rate, blocking=True)
                        sd.stop()
                        _emit_tts_log("TTS 음성 출력 실패: 톤 알림으로 대체", level="warning")
                        return
                    except Exception:
                        pass
                try:
                    QApplication.beep()
                    _emit_tts_log("TTS 음성 출력 실패: 시스템 beep로 대체", level="warning")
                except Exception:
                    pass
            finally:
                try:
                    self.voice_order_event_received.emit({"type": "tts_done"})
                except Exception:
                    pass
                if callable(on_complete):
                    try:
                        on_complete()
                    except Exception:
                        pass

        threading.Thread(target=_play_worker, daemon=True).start()

    def _schedule_voice_retry_auto_listen(self, tts_text: str):
        # 음성주문 테스트 경로에서 retry가 나오면, 안내 음성 종료 후 자동 재청취를 시작한다.
        if bool(getattr(self, "_bartender_sequence_running", False)):
            return
        toggle = getattr(self, "_voice_order_connection_toggle", None)
        if toggle is not None and (not bool(toggle.isChecked())):
            return
        if self._is_voice_order_worker_running():
            return
        auto_retry_on = str(os.environ.get("VOICE_ORDER_RETRY_AUTO_LISTEN", "1") or "").strip().lower()
        if auto_retry_on in ("0", "false", "no", "off"):
            return

        text = str(tts_text or "").strip()
        if text:
            play_ms = int(max(520, min(5200, 320 + (len(text) * 95))))
            delay_ms = int(play_ms + 260)
        else:
            delay_ms = 220

        def _restart():
            if bool(getattr(self, "_closing", False)):
                return
            toggle_now = getattr(self, "_voice_order_connection_toggle", None)
            if toggle_now is not None and (not bool(toggle_now.isChecked())):
                return
            if self._is_voice_order_worker_running():
                return
            self._append_voice_order_log("재입력 자동 대기 시작")
            self._start_voice_order_test(request_stt=True)

        QTimer.singleShot(delay_ms, _restart)

    def _start_voice_order_test(self, request_stt: bool = False):
        if self._voice_order_input_edit is None:
            return
        if str(getattr(self, "_bartender_mode", "") or "").strip().lower() == "auto":
            self._append_voice_order_log("오토모드에서는 개발자 UI 음성 테스트를 시작하지 않습니다. WEB UI에서 시작하세요.", level="warning")
            self._update_voice_mic_button_state()
            return
        backend = getattr(self, "backend", None)
        if backend is not None and hasattr(backend, "start_bartender_sequence") and hasattr(
            backend, "get_bartender_sequence_snapshot"
        ):
            if bool(getattr(self, "_voice_sequence_running", False)):
                self._append_voice_order_log("이미 마이크 입력이 진행 중입니다.")
                self._update_voice_mic_button_state()
                return
            input_text = self._voice_order_input_edit.toPlainText().strip()
            if (not request_stt) and (not input_text):
                QMessageBox.warning(self, "음성주문", "마이크 입력(STT 결과)이 아직 없습니다.")
                return
            if request_stt and self._voice_order_input_edit is not None:
                self._voice_order_last_input_text = ""
                self._voice_order_input_edit.setPlainText("노트북 마이크 입력 대기중...")
            self._voice_last_result = None
            self._set_voice_order_llm_text("-")
            self._set_voice_order_process_state("진행중")
            self._set_voice_order_mic_visual_active(bool(request_stt))
            self._set_voice_order_result(None)
            self._append_voice_order_log("=== 테스트 시작(시퀀스 공통루트) ===")

            request_payload = {
                "input_text": "" if bool(request_stt) else str(input_text or "").strip(),
                "request_stt": bool(request_stt),
                "allow_llm": True,
                "voice_only": True,
                # voice_only 경로는 프론트 TTS 완료 후 재청취를 시작해야 하므로
                # 백엔드 내부 자동 재시도(2/3...)는 비활성화한다.
                "voice_retry_max_attempts": 1,
            }

            self._bartender_status_lock = False
            self._voice_sequence_running = True
            self._update_voice_mic_button_state()
            ok_start, msg_start, snap = backend.start_bartender_sequence(mode="manual", request=request_payload)
            if not ok_start:
                self._voice_sequence_running = False
                self._voice_sequence_run_id = 0
                self._append_voice_order_log(f"시퀀스 시작 실패: {msg_start}", level="error")
                if isinstance(snap, dict):
                    self._apply_bartender_sequence_snapshot(snap)
                else:
                    self._set_voice_order_process_state("실패")
                    self._set_voice_order_mic_state("오류", "error")
                    self._set_voice_order_mic_visual_active(False)
                self._update_voice_mic_button_state()
                return
            self._append_voice_order_log(f"시퀀스 시작: {msg_start}")
            self._voice_sequence_running = True
            if isinstance(snap, dict):
                try:
                    self._voice_sequence_run_id = int(snap.get("run_id", 0) or 0)
                except Exception:
                    self._voice_sequence_run_id = 0
            if isinstance(snap, dict):
                self._apply_bartender_sequence_snapshot(dict(snap))
            self._update_voice_mic_button_state()
            return

        if self._is_voice_order_worker_running():
            self._append_voice_order_log("이미 마이크 입력이 진행 중입니다.")
            self._update_voice_mic_button_state()
            return
        input_text = self._voice_order_input_edit.toPlainText().strip()
        if (not request_stt) and (not input_text):
            QMessageBox.warning(self, "음성주문", "마이크 입력(STT 결과)이 아직 없습니다.")
            return
        if request_stt and self._voice_order_input_edit is not None:
            self._voice_order_last_input_text = ""
            self._voice_order_input_edit.setPlainText("노트북 마이크 입력 대기중...")
        self._voice_worker_done = False
        self._voice_ready_tone_armed = bool(request_stt)
        self._voice_last_result = None
        self._set_voice_order_llm_text("-")
        self._set_voice_order_process_state("진행중")
        self._set_voice_order_mic_visual_active(bool(request_stt))
        self._set_voice_order_result(None)
        self._append_voice_order_log("=== 테스트 시작 ===")
        payload = {"input_text": input_text, "allow_llm": True, "request_stt": bool(request_stt)}
        if not self._start_voice_order_worker(payload):
            self._voice_worker_running = False
            self._set_voice_order_process_state("실패")
            self._set_voice_order_mic_state("오류", "error")
            self._set_voice_order_mic_visual_active(False)
            self._update_voice_mic_button_state()

    def _start_voice_order_worker(self, payload: dict) -> bool:
        backend = getattr(self, "backend", None)
        if backend is None or (not hasattr(backend, "run_voice_order_runtime")):
            self._append_voice_order_log("음성주문 백엔드가 아직 준비되지 않았습니다.")
            return False

        self._voice_worker_token += 1
        token = int(self._voice_worker_token)
        self._voice_worker_proc = None
        self._voice_worker_running = True
        self._update_voice_mic_button_state()
        thread = threading.Thread(
            target=self._run_voice_order_backend_job,
            args=(token, dict(payload or {})),
            name=f"voice-backend-worker-{token}",
            daemon=True,
        )
        self._voice_worker_thread = thread
        thread.start()
        self._append_voice_order_log("백엔드 음성주문 워커 시작")
        return True

    def _run_voice_order_backend_job(self, token: int, payload: dict):
        rc = 0
        try:
            backend = getattr(self, "backend", None)
            if backend is None or (not hasattr(backend, "run_voice_order_runtime")):
                raise RuntimeError("음성주문 백엔드가 준비되지 않았습니다.")

            input_text = str((payload or {}).get("input_text", "") or "").strip()
            recommend_menu = str((payload or {}).get("recommend_menu", "") or "").strip()
            allow_llm = bool((payload or {}).get("allow_llm", True))
            request_stt = bool((payload or {}).get("request_stt", False))
            streamed_event_count = 0

            def _on_backend_event(evt):
                nonlocal streamed_event_count
                if not isinstance(evt, dict):
                    return
                event = dict(evt)
                event["_token"] = int(token)
                streamed_event_count += 1
                self.voice_order_event_received.emit(event)

            ok, out_payload, msg = backend.run_voice_order_runtime(
                input_text=input_text,
                recommend_menu=recommend_menu,
                allow_llm=allow_llm,
                request_stt=request_stt,
                event_callback=_on_backend_event,
            )
            if not isinstance(out_payload, dict):
                raise RuntimeError(str(msg or "음성주문 처리 실패"))

            if streamed_event_count <= 0:
                for item in list(out_payload.get("events", []) or []):
                    if not isinstance(item, dict):
                        continue
                    event = dict(item)
                    event["_token"] = int(token)
                    self.voice_order_event_received.emit(event)

            result = out_payload.get("result", {})
            if isinstance(result, dict):
                event = dict(result)
                event["type"] = "result"
                event["_token"] = int(token)
                self.voice_order_event_received.emit(event)

            done_ok = bool(out_payload.get("ok", ok))
            if (not ok) and done_ok:
                done_ok = False
            self.voice_order_event_received.emit({"type": "done", "ok": done_ok, "_token": int(token)})
            if not done_ok:
                rc = 1
        except Exception as e:
            rc = 1
            self.voice_order_event_received.emit(
                {"type": "error", "message": f"백엔드 음성주문 처리 예외: {e}", "_token": int(token)}
            )
            self.voice_order_event_received.emit({"type": "done", "ok": False, "_token": int(token)})
        finally:
            self.voice_order_event_received.emit({"type": "process_exit", "returncode": int(rc), "_token": int(token)})
            if int(token) == int(self._voice_worker_token):
                self._voice_worker_thread = None

    def _on_voice_order_worker_event(self, event):
        if not isinstance(event, dict):
            return
        evt_type = str(event.get("type", "") or "").strip().lower()
        if evt_type == "tts_log":
            self._append_voice_order_log(str(event.get("message", "") or "").strip(), level=event.get("level", "info"))
            return
        if evt_type == "tts_done":
            self._set_voice_order_process_state("대기")
            self._notify_bartender_tts_done()
            if bool(getattr(self, "_voice_retry_pending_after_tts", False)):
                self._voice_retry_pending_after_tts = False
                self._schedule_voice_retry_auto_listen("")
            return
        toggle = getattr(self, "_voice_order_connection_toggle", None)
        if toggle is not None and (not bool(toggle.isChecked())):
            return
        token = int(event.get("_token", -1))
        if token != int(self._voice_worker_token):
            return
        self._mark_voice_order_update()
        if evt_type == "stage":
            stage = str(event.get("stage", "") or "")
            stage_norm = stage.strip().lower()
            actor = str(event.get("actor", "") or "")
            message = str(event.get("message", "") or "")
            data = event.get("data", None)
            transcript = ""
            if isinstance(data, dict):
                for key in ("input_text", "stt_text", "recognized_text", "transcript", "text"):
                    value = data.get(key, None)
                    if value is None:
                        continue
                    transcript = str(value).strip()
                    if transcript:
                        break
            if transcript:
                self._set_voice_order_input_text(transcript)
            elif stage_norm == "stt":
                maybe_text = str(event.get("text", "") or "").strip()
                if maybe_text:
                    self._set_voice_order_input_text(maybe_text)
            if stage_norm == "stt_open":
                if bool(self._voice_ready_tone_armed):
                    self._voice_ready_tone_armed = False
                    self._play_voice_order_ready_tone()
            elif stage_norm == "stt_wait":
                self._set_voice_order_process_state("마이크입력대기")
                self._set_voice_order_mic_state("입력중", "ok")
                self._set_voice_order_mic_visual_active(True)
                if isinstance(data, dict):
                    stt_idx = data.get("device_index", None)
                    stt_name = str(data.get("device_name", "") or "").strip()
                    test_idx = getattr(self, "_voice_order_mic_test_selected_index", None)
                    test_name = str(getattr(self, "_voice_order_mic_test_selected_name", "") or "").strip()
                    if test_name or (test_idx is not None):
                        mismatch = (stt_idx != test_idx) or (test_name and stt_name and stt_name != test_name)
                        if mismatch:
                            self._append_voice_order_log(
                                f"[mic_test] 장치 불일치 감지: test(index={test_idx},name={test_name}) "
                                f"!= stt(index={stt_idx},name={stt_name})",
                                level="warning",
                            )
            elif stage_norm == "stt_process":
                self._set_voice_order_process_state("STT 처리중")
                self._set_voice_order_mic_state("STT 처리중", "warn")
                self._set_voice_order_mic_visual_active(False)
            elif stage_norm in ("classify", "llm", "recipe"):
                self._set_voice_order_process_state("주문분석중")
                self._set_voice_order_mic_state("처리중", "warn")
                self._set_voice_order_mic_visual_active(False)
            line = f"[{stage}] {actor}: {message}"
            if data is not None:
                try:
                    line += f" | {json.dumps(data, ensure_ascii=False)}"
                except Exception:
                    line += f" | {data}"
            self._append_voice_order_log(line)
            return
        if evt_type == "mic_level":
            # 입력요청중/분석중 단계에서는 게이지를 띄우지 않는다.
            if str(getattr(self, "_voice_order_mic_state_text", "") or "").strip() != "입력중":
                return
            self._set_voice_order_mic_visual_active(True)
            self._set_voice_order_mic_level(event.get("level", 0))
            return
        if evt_type == "result":
            self._voice_last_result = dict(event)
            self._set_voice_order_result(self._voice_last_result)
            menu = str(event.get("selected_menu_label") or event.get("selected_menu") or "-")
            self._append_voice_order_log(f"[result] 메뉴={menu}, status={event.get('status')}")
            return
        if evt_type == "done":
            self._voice_worker_done = True
            self._voice_worker_running = False
            self._voice_ready_tone_armed = False
            self._set_voice_order_mic_state("대기", "info")
            self._set_voice_order_mic_visual_active(False)
            self._update_voice_mic_button_state()
            ok = bool(event.get("ok", False))
            if not ok:
                self._set_voice_order_process_state("실패")
                self._append_voice_order_log("[done] 처리 실패", level="error")
                return
            if str(getattr(self, "_bartender_mode", "") or "").strip().lower() == "auto":
                # 오토모드에서는 개발자 UI가 음성 출력을 하지 않는다.
                self._voice_retry_pending_after_tts = False
                self._set_voice_order_process_state("완료")
                self._append_voice_order_log("[done] 처리 완료(오토모드: 음성출력은 WEB UI)")
                return
            status_text = str((self._voice_last_result or {}).get("status", "")).strip().lower()
            tts_text = str((self._voice_last_result or {}).get("tts_text", "") or "").strip()
            public_tts = self._sanitize_customer_tts_text(status=status_text, raw_text=tts_text)
            if status_text == "success":
                self._voice_retry_pending_after_tts = False
                self._set_voice_order_process_state("완료")
                self._append_voice_order_log("[done] 처리 완료")
                self._play_voice_order_feedback_tone(public_tts)
            elif status_text == "retry":
                self._voice_retry_pending_after_tts = True
                self._set_voice_order_process_state("완료(재입력)")
                self._append_voice_order_log("[done] 재입력 필요")
                self._play_voice_order_feedback_tone(public_tts)
            else:
                self._voice_retry_pending_after_tts = False
                self._set_voice_order_process_state("완료")
                self._append_voice_order_log("[done] 처리 완료")
                self._play_voice_order_feedback_tone(tts_text)
            return
        if evt_type == "stderr":
            sev = str(event.get("severity", "warning") or "warning").strip().lower()
            message = str(event.get("message", "") or "")
            self._append_voice_order_log(f"[stderr] {message}", level=("warning" if sev == "warning" else "error"))
            return
        if evt_type == "error":
            self._voice_worker_running = False
            self._voice_ready_tone_armed = False
            self._voice_retry_pending_after_tts = False
            self._set_voice_order_process_state("실패")
            self._set_voice_order_mic_state("오류", "error")
            self._set_voice_order_mic_visual_active(False)
            self._update_voice_mic_button_state()
            self._append_voice_order_log(f"[error] {event.get('message', '')}", level="error")
            return
        if evt_type == "process_exit":
            rc = int(event.get("returncode", -1))
            proc = self._voice_worker_proc
            if proc is not None and proc.poll() is not None:
                self._voice_worker_proc = None
            if rc != 0 and not self._voice_worker_done:
                self._set_voice_order_process_state("실패")
                self._set_voice_order_mic_state("오류", "error")
                self._append_voice_order_log(f"워커 비정상 종료(returncode={rc})", level="error")
            elif rc == 0 and not self._voice_worker_done:
                self._set_voice_order_process_state("완료")
                self._set_voice_order_mic_state("대기", "info")
                self._append_voice_order_log("워커 종료")
            self._voice_worker_running = False
            self._set_voice_order_mic_visual_active(False)
            self._update_voice_mic_button_state()
            return
        self._append_voice_order_log(str(event))

    def _stop_voice_order_worker(self, reason: str = ""):
        backend = getattr(self, "backend", None)
        if bool(getattr(self, "_voice_sequence_running", False)) and backend is not None and hasattr(
            backend, "stop_bartender_sequence"
        ):
            try:
                ok_stop, msg_stop, snap = backend.stop_bartender_sequence(reason=(str(reason or "").strip() or "음성주문 중지"))
                self._append_voice_order_log(
                    f"음성주문 시퀀스 중지 {'완료' if ok_stop else '실패'}: {msg_stop}",
                    level=("warning" if ok_stop else "error"),
                )
                if isinstance(snap, dict):
                    self._apply_bartender_sequence_snapshot(snap)
            except Exception as exc:
                self._append_voice_order_log(f"음성주문 시퀀스 중지 예외: {exc}", level="error")
            self._voice_sequence_running = False
            self._voice_sequence_run_id = 0

        self._voice_worker_token += 1
        self._voice_worker_done = True
        self._voice_worker_running = False
        self._voice_ready_tone_armed = False
        self._set_voice_order_mic_visual_active(False)
        self._voice_worker_thread = None
        proc = self._voice_worker_proc
        if proc is None:
            if reason:
                self._append_voice_order_log(f"워커 중지: {reason}")
            self._set_voice_order_process_state("대기")
            self._set_voice_order_mic_state("대기", "info")
            self._update_voice_mic_button_state()
            return
        self._voice_worker_proc = None
        if proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=1.2)
            except Exception:
                try:
                    proc.kill()
                    proc.wait(timeout=0.5)
                except Exception:
                    pass
        if reason:
            self._append_voice_order_log(f"워커 중지: {reason}")
        self._set_voice_order_process_state("대기")
        self._set_voice_order_mic_state("대기", "info")
        self._update_voice_mic_button_state()

    def _reserve_top_status_space(self):
        if self._status_row_ready:
            return
        self._layout_main_frames()
        self._status_row_ready = True

    def _layout_main_frames(self):
        if UI_USE_DESIGN_GEOMETRY:
            return
        frame_l = getattr(self, "frame", None)
        frame_m = getattr(self, "frame_4", None)
        frame_r = getattr(self, "frame_2", None)
        frame_log = getattr(self, "frame_3", None)
        term = getattr(self, "terminal", None)
        if frame_l is None or frame_r is None or frame_log is None:
            return

        margin = DASHBOARD_MARGIN
        col_gap = DASHBOARD_COL_GAP
        row_gap = DASHBOARD_ROW_GAP

        has_top_status_row = self._top_status_panel is not None
        target_y = (TOP_STATUS_BAR_HEIGHT + TOP_STATUS_GAP) if has_top_status_row else margin
        base_col_w = [int(DASHBOARD_COL_WIDTHS[0]), int(DASHBOARD_COL_WIDTHS[1]), int(DASHBOARD_COL_WIDTHS[2])]
        section_w = int(min(base_col_w[1], base_col_w[2]))
        left_w = int(min(base_col_w[0], max(400, int(section_w * 0.92))))
        col_w = [left_w, section_w, section_w]
        row1_h = int(DASHBOARD_TOP_PANEL_HEIGHT)
        row2_h = int(DASHBOARD_BOTTOM_PANEL_HEIGHT)
        combined_top_h = row1_h + row_gap + row2_h
        log_h = max(
            int(DASHBOARD_LOG_MIN_HEIGHT),
            int(round(float(combined_top_h) * float(DASHBOARD_LOG_HEIGHT_RATIO))),
        )
        content_h = row1_h + row_gap + row2_h + row_gap + log_h

        total_grid_w = col_w[0] + col_gap + col_w[1] + col_gap + col_w[2]
        required_central_w = (2 * margin) + total_grid_w
        required_central_h = target_y + content_h + margin + DASHBOARD_BOTTOM_EXTRA_MARGIN
        central_w = int(self.centralwidget.width())
        central_h = int(self.centralwidget.height())
        extra_w = max(0, int(self.width()) - central_w)
        extra_h = max(0, int(self.height()) - central_h)
        required_window_w = required_central_w + extra_w
        required_window_h = required_central_h + extra_h

        if not bool(getattr(self, "_dashboard_resize_guard", False)):
            self.setMinimumSize(required_window_w, required_window_h)
            need_fit = (
                (not self.isMaximized())
                and (not self.isFullScreen())
                and (self.width() != required_window_w or self.height() != required_window_h)
            )
            if need_fit:
                self._dashboard_resize_guard = True
                try:
                    self.resize(required_window_w, required_window_h)
                finally:
                    self._dashboard_resize_guard = False

        x_col1 = margin
        x_col2 = x_col1 + col_w[0] + col_gap
        x_col3 = x_col2 + col_w[1] + col_gap

        y_row1 = target_y
        y_row2 = y_row1 + row1_h + row_gap
        y_row3 = y_row2 + row2_h + row_gap

        self._bartender_overview_rect = (x_col1, y_row1, col_w[0], combined_top_h)
        self._voice_panel_layout_rect = (x_col2, y_row1, col_w[1], row1_h)
        frame_r.setGeometry(x_col3, y_row1, col_w[2], row1_h)
        frame_l.setGeometry(x_col2, y_row2, col_w[1], row2_h)
        if frame_m is not None:
            frame_m.setGeometry(x_col3, y_row2, col_w[2], row2_h)
        frame_log.setGeometry(x_col1, y_row3, total_grid_w, log_h)

        self._layout_bartender_overview_panel()
        self._layout_voice_order_panel()
        self._layout_vision_widgets()

        if hasattr(self, "_log_clear_button") and self._log_clear_button is not None:
            btn_w = 96
            btn_h = 24
            self._log_clear_button.setGeometry(max(10, frame_log.width() - btn_w - 12), 8, btn_w, btn_h)
        if term is not None:
            term.setGeometry(10, 40, max(120, frame_log.width() - 20), max(120, frame_log.height() - 50))

    def _layout_vision_widgets(self):
        def _layout_single(frame_obj, view_obj, calib_box):
            if frame_obj is None or view_obj is None:
                return
            top_y = 93
            side = 10
            bottom = 14
            gap = 8
            min_view_h = 120
            min_calib_h = 96
            default_calib_h = 129
            panel_w = max(220, frame_obj.width() - (side * 2))
            panel_h = int(frame_obj.height())
            view_h = max(min_view_h, panel_h - top_y - bottom)
            if calib_box is not None:
                enough_for_calib = panel_h >= (top_y + min_view_h + gap + min_calib_h + bottom)
                if enough_for_calib:
                    calib_h = min(default_calib_h, max(min_calib_h, int(panel_h * 0.28)))
                    calib_y = max(top_y + min_view_h + gap, panel_h - bottom - calib_h)
                    calib_box.setGeometry(side, calib_y, panel_w, calib_h)
                    calib_box.show()
                    view_h = max(min_view_h, calib_y - top_y - gap)
                else:
                    calib_box.hide()
            view_obj.setGeometry(side, top_y, panel_w, view_h)

        _layout_single(
            getattr(self, "frame", None),
            getattr(self, "yolo_view", None),
            getattr(self, "calibration_group_box", None),
        )
        _layout_single(
            getattr(self, "frame_4", None),
            getattr(self, "yolo_view_2", None),
            getattr(self, "calibration_group_box_2", None),
        )

    def _setup_log_controls(self):
        frame_log = getattr(self, "frame_3", None)
        if frame_log is None:
            self._log_clear_button = None
            return
        self._log_clear_button = getattr(self, "log_clear_button", None)
        if self._log_clear_button is None:
            for btn in frame_log.findChildren(QPushButton):
                if "로그" in btn.text() and "클리어" in btn.text():
                    self._log_clear_button = btn
                    break
        if self._log_clear_button is not None:
            self._log_clear_button.clicked.connect(self._clear_log_terminal)

    def _clear_log_terminal(self):
        with self._log_buffer_lock:
            self._log_buffer.clear()
            self._log_line_open = False
        self.terminal.clear()

    def _prefix_log_timestamps_locked(self, text: str) -> str:
        if not text:
            return ""
        normalized = text.replace("\r\n", "\n").replace("\r", "\n")
        parts = normalized.splitlines(keepends=True)
        if not parts:
            parts = [normalized]
        stamped = []
        for part in parts:
            if (not self._log_line_open) and part not in ("", "\n"):
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                stamped.append(f"[{ts}] {part}")
            else:
                stamped.append(part)
            self._log_line_open = not part.endswith("\n")
        return "".join(stamped)

    def append_log(self, text):
        try:
            msg = str(text)
        except Exception:
            return
        if not msg:
            return
        if not hasattr(self, "_log_buffer") or self._log_buffer is None:
            return
        with self._log_buffer_lock:
            self._log_buffer.append(self._prefix_log_timestamps_locked(msg))

    def _flush_log_buffer(self):
        if not hasattr(self, "_log_buffer"):
            return
        with self._log_buffer_lock:
            if not self._log_buffer:
                return
            chunk = "".join(self._log_buffer)
            self._log_buffer.clear()
        term = getattr(self, "terminal", None)
        if term is None:
            return
        try:
            from PyQt5.QtGui import QTextCursor

            cursor = term.textCursor()
            cursor.movePosition(QTextCursor.End)
            cursor.insertText(chunk)
            term.setTextCursor(cursor)
        except Exception:
            try:
                if hasattr(term, "insertPlainText"):
                    term.insertPlainText(chunk)
                else:
                    existing = term.toPlainText() if hasattr(term, "toPlainText") else ""
                    term.setPlainText(existing + chunk)
            except Exception:
                return
        try:
            sb = term.verticalScrollBar()
            if sb is not None:
                sb.setValue(sb.maximum())
        except Exception:
            pass

    def _fmt_ui_float(self, value, digits=2):
        try:
            v = float(value)
        except Exception:
            return "-"
        if not np.isfinite(v):
            return "-"
        return f"{v:.{int(digits)}f}"

    def _handle_exception(self, exc_type, exc_value, exc_traceback):
        try:
            text = "".join(traceback.format_exception(exc_type, exc_value, exc_traceback))
        except Exception:
            text = f"{exc_type}: {exc_value}"
        try:
            sys.__stderr__.write(text)
            sys.__stderr__.flush()
        except Exception:
            pass
        try:
            self.append_log(f"{text}\n")
        except Exception:
            pass

    def _handle_thread_exception(self, args):
        if args is None:
            return
        self._handle_exception(
            getattr(args, "exc_type", Exception),
            getattr(args, "exc_value", Exception("thread exception")),
            getattr(args, "exc_traceback", None),
        )

    def _is_shutdown_exception(self, err: Exception) -> bool:
        if isinstance(err, RCLError):
            return True
        msg = str(err or "").lower()
        if not msg:
            return False
        markers = (
            "context is invalid",
            "rcl node's context is invalid",
            "destruction was requested",
            "cannot use destroyable because destruction was requested",
            "failed to create subscription",
            "failed to create guard condition",
            "rcl_shutdown already called",
        )
        return any(marker in msg for marker in markers)

    def _log_ui_tick_exception(self, tick_name: str, err: Exception):
        now = time.monotonic()
        key = str(tick_name or "ui_tick")
        last_at = float(self._ui_tick_error_last_at.get(key, 0.0))
        if (now - last_at) < 3.0:
            return
        self._ui_tick_error_last_at[key] = now
        self.append_log(f"[UI] {key} 처리 중 예외: {err}\n")

    def _log_ui_tick_slow(self, tick_name: str, elapsed_ms: float):
        key = str(tick_name or "ui_tick")
        if key == "log_flush":
            return
        now = time.monotonic()
        last_at = float(self._ui_tick_slow_last_at.get(key, 0.0))
        if (now - last_at) < float(UI_TICK_WARN_COOLDOWN_SEC):
            return
        self._ui_tick_slow_last_at[key] = now
        self.append_log(
            f"[UI-PERF] slow tick: {key} {float(elapsed_ms):.1f}ms "
            f"(warn>={float(UI_TICK_WARN_MS):.1f}ms)\n"
        )
        self._append_ui_perf_log_file(
            f"slow tick: {key} {float(elapsed_ms):.1f}ms (warn>={float(UI_TICK_WARN_MS):.1f}ms)"
        )

    def _append_ui_perf_log_file(self, message: str):
        path = str(getattr(self, "_ui_perf_log_path", "") or "").strip()
        if not path:
            return
        text = str(message or "").strip()
        if not text:
            return
        line = f"[{time.strftime('%H:%M:%S')}] {text}\n"
        try:
            with self._ui_perf_log_lock:
                with open(path, "a", encoding="utf-8") as fp:
                    fp.write(line)
        except Exception:
            return

    def _safe_ui_tick(self, tick_name: str, fn, *args, **kwargs):
        if getattr(self, "_closing", False):
            return
        started_at = time.monotonic()
        try:
            fn(*args, **kwargs)
        except Exception as e:
            if getattr(self, "_closing", False) or self._is_shutdown_exception(e):
                return
            self._log_ui_tick_exception(str(tick_name), e)
        finally:
            elapsed_ms = (time.monotonic() - started_at) * 1000.0
            if UI_TICK_TRACE_ENABLED and float(elapsed_ms) >= float(UI_TICK_TRACE_MIN_MS):
                self._append_ui_perf_log_file(f"tick_trace: {str(tick_name)} {float(elapsed_ms):.2f}ms")
            if float(elapsed_ms) >= float(UI_TICK_WARN_MS):
                self._log_ui_tick_slow(str(tick_name), float(elapsed_ms))

    def _setup_top_status_row(self):
        self._top_status_panel = getattr(self, "top_status_panel", None)
        self._top_status_mid_line = getattr(self, "top_status_mid_line", None)
        self._top_status_widgets = {}
        if self._top_status_panel is not None:
            self._top_status_panel.setFrameShape(QFrame.NoFrame)
            self._top_status_panel.setStyleSheet(
                "QFrame#top_status_panel { border: none; background: #f8fafc; }"
            )
        if self._top_status_mid_line is not None:
            self._top_status_mid_line.hide()

        defs = [
            ("vision", "비전1", "top_status_vision_box", "top_status_vision_dot", "top_status_vision_text"),
            ("vision2", "비전2", "top_status_vision_box_2", "top_status_vision_dot_2", "top_status_vision_text_2"),
            ("robot", "로봇", "top_status_robot_box", "top_status_robot_dot", "top_status_robot_text"),
        ]
        for key, title, box_name, dot_name, text_name in defs:
            box = getattr(self, box_name, None)
            dot = getattr(self, dot_name, None)
            text = getattr(self, text_name, None)
            # New UI may remove *box and keep only dot/text.
            if dot is None or text is None:
                continue
            if box is not None:
                box.setFrameShape(QFrame.NoFrame)
                box.setStyleSheet("background: #ffffff; border: 1px solid #cfd8dc; border-radius: 3px;")
            dot.setAlignment(Qt.AlignCenter)
            dot.setText("●")
            text.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            self._top_status_widgets[key] = (title, box, dot, text)
        ui_toggles = {
            "vision": getattr(self, "top_status_vision_toggle", None),
            "vision2": getattr(self, "top_status_vision2_toggle", None),
            "robot": getattr(self, "top_status_robot_toggle", None),
        }
        for key, toggle in ui_toggles.items():
            if toggle is None:
                continue
            self._top_status_toggles[key] = toggle
            toggle.blockSignals(True)
            toggle.setChecked(bool(self._top_status_enabled.get(key, True)))
            toggle.setText("ON" if bool(self._top_status_enabled.get(key, True)) else "OFF")
            toggle.blockSignals(False)
            try:
                toggle.toggled.disconnect()
            except Exception:
                pass
            toggle.toggled.connect(lambda checked, src=key: self._on_top_status_toggle_changed(src, checked))

        self._reposition_top_status_row()
        self._set_top_status("vision", "확인중", "warning")
        self._set_top_status("vision2", "확인중", "warning")
        self._set_top_status("robot", "확인중", "warning")

    def _on_top_status_toggle_changed(self, key: str, checked: bool):
        k = str(key)
        prev_on = bool(self._top_status_enabled.get(k, True))
        if str(getattr(self, "_bartender_mode", "") or "").strip().lower() == "auto":
            toggle = self._top_status_toggles.get(k)
            if toggle is not None:
                toggle.blockSignals(True)
                toggle.setChecked(prev_on)
                toggle.setText("ON" if prev_on else "OFF")
                toggle.blockSignals(False)
            self.append_log(f"[상태토글] 오토모드에서는 {k} ON/OFF를 변경할 수 없습니다.\n")
            return
        on = bool(checked)
        self._top_status_enabled[k] = on
        toggle = self._top_status_toggles.get(k)
        if toggle is not None:
            toggle.setText("ON" if on else "OFF")
        if k == "vision":
            self._vision_state_text = "비활성화" if (not on) else "확인중"
            if not on:
                if bool(getattr(self, "_calibration_mode_enabled_1", False)):
                    sw = getattr(self, "_calibration_mode_switch", None)
                    if sw is not None:
                        sw.setChecked(False)
                    else:
                        self._calibration_mode_enabled_1 = False
                        self._calibration_mode_enabled = bool(self._calibration_mode_enabled_2)
                        self._update_calibration_mode_ui()
                        self._save_vision_serial_settings()
                self._vision_cycle_ms = None
                self._teardown_external_vision_bridge_panel(1)
                self._stop_calibration_process(1)
                self._stop_external_vision_process_panel(1)
                self._clear_vision_view_data(panel_index=1)
                if YOLO_EXTERNAL_NODE and (not bool(self._top_status_enabled.get("vision2", True))):
                    self._stop_external_vision_process()
            else:
                self._vision_cycle_ms = None
            self._set_vision_panel_controls_enabled(1, on)
            self._sync_vision_render_timers()
        elif k == "vision2":
            self._vision_state_text_2 = "비활성화" if (not on) else "확인중"
            if not on:
                if bool(getattr(self, "_calibration_mode_enabled_2", False)):
                    sw = getattr(self, "_calibration_mode_switch_2", None)
                    if sw is not None:
                        sw.setChecked(False)
                    else:
                        self._calibration_mode_enabled_2 = False
                        self._calibration_mode_enabled = bool(self._calibration_mode_enabled_1)
                        self._update_calibration_mode_ui()
                        self._save_vision_serial_settings()
                self._vision_cycle_ms_2 = None
                self._teardown_external_vision_bridge_panel(2)
                self._stop_calibration_process(2)
                self._stop_external_vision_process_panel(2)
                self._clear_vision_view_data(panel_index=2)
                if YOLO_EXTERNAL_NODE and (not bool(self._top_status_enabled.get("vision", True))):
                    self._stop_external_vision_process()
            else:
                self._vision_cycle_ms_2 = None
            self._set_vision_panel_controls_enabled(2, on)
            self._sync_vision_render_timers()
        elif k == "robot" and (not on):
            self._sync_robot_launch_from_toggle(reason="robot OFF")
            self._robot_cycle_ms = None
            if self.backend is not None and hasattr(self.backend, "stop_robot_state_subscriptions"):
                try:
                    ok, msg = self.backend.stop_robot_state_subscriptions()
                    self.append_log(f"[로봇] {msg}\n")
                except Exception as e:
                    self.append_log(f"[로봇] 구독 종료 실패: {e}\n")
            self._clear_robot_data_view()
        elif k == "robot" and on and (not prev_on):
            self._sync_robot_launch_from_toggle(reason="robot ON")
            if self.backend is not None and hasattr(self.backend, "start_robot_state_subscriptions"):
                try:
                    ok, msg = self.backend.start_robot_state_subscriptions()
                    self.append_log(f"[로봇] {msg}\n")
                except Exception as e:
                    self.append_log(f"[로봇] 구독 재시작 실패: {e}\n")
            self._robot_cycle_ms = None
            self._robot_prev_state_seen_at = None
            self._last_positions_seen_at = None
            self._set_robot_controls_enabled(self.backend is not None and self.backend.is_ready())
        self.append_log(f"[상태토글] {k} {'활성화' if on else '비활성화'}\n")
        if k in ("vision", "vision2"):
            self._sync_sensor_launch_from_vision_toggles(reason=f"{k} {'ON' if on else 'OFF'}")
        if on and (not prev_on) and k in ("vision", "vision2"):
            self._restart_vision_after_toggle_enable(k)
        self._save_vision_serial_settings()
        self._refresh_robot_status()
        self._refresh_vision_status()

    def _is_realsense_nodes_alive(self) -> bool:
        try:
            probe = subprocess.run(
                ["pgrep", "-f", "realsense2_camera_node"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=0.8,
                check=False,
            )
            return int(getattr(probe, "returncode", 1)) == 0 and bool(str(probe.stdout or "").strip())
        except Exception:
            return False

    def _stop_sensor_launch_runtime(self, reason: str = "", kill_nodes: bool = True):
        proc = getattr(self, "_sensor_launch_proc", None)
        self._sensor_launch_proc = None
        self._sensor_launch_signature = ""
        if proc is not None and proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=3.0)
            except Exception:
                try:
                    proc.kill()
                    proc.wait(timeout=1.0)
                except Exception:
                    pass
        if kill_nodes:
            # system_launch에서 올린 기존 센서 노드도 함께 내린다.
            for pattern in (
                r"realsense2_camera_node.*__ns:=/camera2",
                r"realsense2_camera_node.*__ns:=/camera",
                r"realsense2_camera_node",
            ):
                try:
                    subprocess.run(
                        ["pkill", "-f", pattern],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=0.8,
                        check=False,
                    )
                except Exception:
                    pass
            time.sleep(0.15)
        if reason:
            self.append_log(f"[센서] 런타임 센서 종료: {reason}\n")

    def _start_sensor_launch_runtime(self, run_camera1: bool, run_camera2: bool, reason: str = "") -> bool:
        launch_path = os.path.join(PROJECT_ROOT, "launch", "realsense_launch.py")
        if not os.path.isfile(launch_path):
            self.append_log(f"[센서] 재런치 실패: 런치 파일 없음 ({launch_path})\n")
            return False
        serial1 = self._normalize_serial_text(getattr(self, "_vision_assigned_serial_1", ""))
        serial2 = self._normalize_serial_text(getattr(self, "_vision_assigned_serial_2", ""))
        cmd = [
            sys.executable,
            launch_path,
            f"run_camera1:={'true' if bool(run_camera1) else 'false'}",
            f"run_camera2:={'true' if bool(run_camera2) else 'false'}",
        ]
        if serial1:
            cmd.append(f"camera1_serial_no:=_{serial1}")
        if serial2:
            cmd.append(f"camera2_serial_no:=_{serial2}")
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                cwd=PROJECT_ROOT,
                env=dict(os.environ),
                start_new_session=True,
            )
        except Exception as exc:
            self.append_log(f"[센서] 재런치 실패: {exc}\n")
            return False
        time.sleep(0.2)
        if proc.poll() is not None:
            self.append_log("[센서] 재런치 실패: 프로세스가 즉시 종료되었습니다.\n")
            return False
        self._sensor_launch_proc = proc
        self._sensor_launch_signature = f"{int(bool(run_camera1))}:{int(bool(run_camera2))}:{serial1}:{serial2}"
        self.append_log(
            f"[센서] 런타임 센서 시작: cam1={serial1 or '-'}({bool(run_camera1)}), "
            f"cam2={serial2 or '-'}({bool(run_camera2)})"
            + (f", 사유={reason}" if reason else "")
            + "\n"
        )
        return True

    def _sync_sensor_launch_from_vision_toggles(self, reason: str = ""):
        want_cam1 = bool(self._top_status_enabled.get("vision", True))
        want_cam2 = bool(self._top_status_enabled.get("vision2", True))
        serial1 = self._normalize_serial_text(getattr(self, "_vision_assigned_serial_1", ""))
        serial2 = self._normalize_serial_text(getattr(self, "_vision_assigned_serial_2", ""))
        desired_sig = f"{int(want_cam1)}:{int(want_cam2)}:{serial1}:{serial2}"

        if (not want_cam1) and (not want_cam2):
            self._stop_sensor_launch_runtime(reason=reason or "비전 토글 OFF", kill_nodes=True)
            return

        proc = getattr(self, "_sensor_launch_proc", None)
        proc_alive = bool(proc is not None and proc.poll() is None)
        nodes_alive = self._is_realsense_nodes_alive()
        current_sig = str(getattr(self, "_sensor_launch_signature", "") or "")

        if proc_alive and nodes_alive and (current_sig == desired_sig):
            return

        self._stop_sensor_launch_runtime(reason=(reason or "비전 설정 변경"), kill_nodes=True)
        self._start_sensor_launch_runtime(want_cam1, want_cam2, reason=reason)

    def _is_robot_nodes_alive(self) -> bool:
        patterns = (
            r"ros2_control_node.*__ns:=/dsr01",
            r"dsr_bringup2_gazebo.launch.py",
        )
        for pattern in patterns:
            try:
                probe = subprocess.run(
                    ["pgrep", "-f", pattern],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=0.8,
                    check=False,
                )
                if int(getattr(probe, "returncode", 1)) == 0 and bool(str(probe.stdout or "").strip()):
                    return True
            except Exception:
                continue
        return False

    def _stop_robot_launch_runtime(self, reason: str = "", kill_nodes: bool = True):
        proc = getattr(self, "_robot_launch_proc", None)
        self._robot_launch_proc = None
        self._robot_launch_signature = ""
        if proc is not None and proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=3.0)
            except Exception:
                try:
                    proc.kill()
                    proc.wait(timeout=1.0)
                except Exception:
                    pass
        if kill_nodes:
            for pattern in (
                r"dsr_bringup2_gazebo.launch.py",
                r"ros2_control_node.*__ns:=/dsr01",
                r"spawner.*dsr",
            ):
                try:
                    subprocess.run(
                        ["pkill", "-f", pattern],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=0.8,
                        check=False,
                    )
                except Exception:
                    pass
            time.sleep(0.2)
        if reason:
            self.append_log(f"[로봇] 런타임 로봇 런치 종료: {reason}\n")

    def _start_robot_launch_runtime(self, reason: str = "") -> bool:
        launch_path = os.path.join(PROJECT_ROOT, "launch", "system_launch.py")
        if not os.path.isfile(launch_path):
            self.append_log(f"[로봇] 런타임 로봇 런치 시작 실패: launch 파일 없음 ({launch_path})\n")
            return False

        mode = str(ROBOT_MODE_HINT or "real").strip().lower() or "real"
        model = str(ROBOT_MODEL_HINT or "e0509").strip() or "e0509"
        host = str(ROBOT_HOST_HINT or "110.120.1.68").strip() or "110.120.1.68"
        rt_host = str(ROBOT_RT_HOST_HINT or "192.168.137.50").strip() or "192.168.137.50"
        gz = str(ROBOT_GZ_HINT or ("false" if mode == "real" else "true")).strip().lower()
        if gz not in ("true", "false"):
            gz = "false" if mode == "real" else "true"

        cmd = [
            sys.executable,
            launch_path,
            "run_robot:=true",
            "run_sensors:=false",
            "run_frontend:=false",
            "run_user_frontend:=false",
            f"robot_mode:={mode}",
            f"robot_model:={model}",
            f"robot_host:={host}",
            f"robot_rt_host:={rt_host}",
            f"robot_gz:={gz}",
        ]
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                cwd=PROJECT_ROOT,
                env=dict(os.environ),
                start_new_session=True,
            )
        except Exception as exc:
            self.append_log(f"[로봇] 런타임 로봇 런치 시작 실패: {exc}\n")
            return False
        time.sleep(0.2)
        if proc.poll() is not None:
            self.append_log("[로봇] 런타임 로봇 런치 시작 실패: 프로세스가 즉시 종료되었습니다.\n")
            return False
        self._robot_launch_proc = proc
        self._robot_launch_signature = f"{mode}:{model}:{host}:{rt_host}:{gz}"
        self.append_log(
            f"[로봇] 런타임 로봇 런치 시작: mode={mode}, model={model}, host={host}, rt_host={rt_host}, gz={gz}"
            + (f", 사유={reason}" if reason else "")
            + "\n"
        )
        return True

    def _sync_robot_launch_from_toggle(self, reason: str = ""):
        want_robot = bool(self._top_status_enabled.get("robot", True))
        mode = str(ROBOT_MODE_HINT or "real").strip().lower() or "real"
        model = str(ROBOT_MODEL_HINT or "e0509").strip() or "e0509"
        host = str(ROBOT_HOST_HINT or "110.120.1.68").strip() or "110.120.1.68"
        rt_host = str(ROBOT_RT_HOST_HINT or "192.168.137.50").strip() or "192.168.137.50"
        gz = str(ROBOT_GZ_HINT or ("false" if mode == "real" else "true")).strip().lower()
        if gz not in ("true", "false"):
            gz = "false" if mode == "real" else "true"
        desired_sig = f"{mode}:{model}:{host}:{rt_host}:{gz}"

        if not want_robot:
            self._stop_robot_launch_runtime(reason=reason or "로봇 토글 OFF", kill_nodes=True)
            return

        proc = getattr(self, "_robot_launch_proc", None)
        proc_alive = bool(proc is not None and proc.poll() is None)
        nodes_alive = self._is_robot_nodes_alive()
        current_sig = str(getattr(self, "_robot_launch_signature", "") or "")

        if nodes_alive and ((not proc_alive) or (current_sig == desired_sig)):
            return

        self._stop_robot_launch_runtime(reason=(reason or "로봇 설정 변경"), kill_nodes=True)
        self._start_robot_launch_runtime(reason=reason)

    def _restart_vision_after_toggle_enable(self, key: str):
        if not UI_ENABLE_VISION:
            return
        now = time.monotonic()
        self._mode_switch_grace_until = now + MODE_SWITCH_GRACE_SEC
        if key == "vision":
            self._vision_state_text = "시작 중"
            self._last_vision_frame_at = now
        else:
            self._vision_state_text_2 = "시작 중"
            self._last_vision_frame_at_2 = now
        panel = 1 if key == "vision" else 2
        self._append_vision_log(f"{key} 활성화: 비전 연결 재초기화 시작", panel_index=panel)
        if self.backend is None:
            return
        try:
            if YOLO_EXTERNAL_NODE and YOLO_AUTO_LAUNCH_NODE:
                self._ensure_external_vision_process_panel(panel)
            self._sync_calibration_processes()
            panel = 1 if key == "vision" else 2
            self._setup_external_vision_bridge_panel(panel)
        except Exception as e:
            self._append_vision_log(f"재초기화 실패: {e}", panel_index=panel)

    def _set_vision_panel_controls_enabled(self, panel_index: int, enabled: bool):
        panel = 2 if int(panel_index) == 2 else 1
        on = bool(enabled)
        if panel == 2:
            targets = [
                getattr(self, "yolo_view_2", None),
                getattr(self, "_vision_rotate_left_button_2", None),
                getattr(self, "_vision_rotate_zero_button_2", None),
                getattr(self, "_vision_rotate_right_button_2", None),
                getattr(self, "_vision_serial_change_button_2", None),
                getattr(self, "_calibration_mode_switch_2", None),
                getattr(self, "_calibration_transform_button_2", None),
                getattr(self, "_calibration_load_button_2", None),
                getattr(self, "calibration_group_box_2", None),
            ]
        else:
            targets = [
                getattr(self, "yolo_view", None),
                getattr(self, "_vision_rotate_left_button", None),
                getattr(self, "_vision_rotate_zero_button", None),
                getattr(self, "_vision_rotate_right_button", None),
                getattr(self, "_vision_serial_change_button", None),
                getattr(self, "_calibration_mode_switch", None),
                getattr(self, "_calibration_transform_button", None),
                getattr(self, "_calibration_load_button", None),
                getattr(self, "calibration_group_box", None),
            ]
        for w in targets:
            if w is not None:
                w.setEnabled(on)

    def _sync_vision_render_timers(self):
        if hasattr(self, "_vision_render_timer_1") and self._vision_render_timer_1 is not None:
            want_1 = bool(self._top_status_enabled.get("vision", True))
            if want_1 and (not self._vision_render_timer_1.isActive()):
                self._vision_render_timer_1.start(VISION_RENDER_INTERVAL_MS)
            elif (not want_1) and self._vision_render_timer_1.isActive():
                self._vision_render_timer_1.stop()
        if hasattr(self, "_vision_render_timer_2") and self._vision_render_timer_2 is not None:
            want_2 = bool(self._top_status_enabled.get("vision2", True))
            if want_2 and (not self._vision_render_timer_2.isActive()):
                self._vision_render_timer_2.start(VISION_RENDER_INTERVAL_MS)
            elif (not want_2) and self._vision_render_timer_2.isActive():
                self._vision_render_timer_2.stop()

    def _start_vision_compose_workers(self):
        self._vision_compose_stop.clear()
        if self._vision_compose_thread_1 is None or not self._vision_compose_thread_1.is_alive():
            self._vision_compose_thread_1 = threading.Thread(
                target=self._vision_compose_loop,
                args=(1,),
                name="vision-compose-1",
                daemon=True,
            )
            self._vision_compose_thread_1.start()
        if self._vision_compose_thread_2 is None or not self._vision_compose_thread_2.is_alive():
            self._vision_compose_thread_2 = threading.Thread(
                target=self._vision_compose_loop,
                args=(2,),
                name="vision-compose-2",
                daemon=True,
            )
            self._vision_compose_thread_2.start()

    def _stop_vision_compose_workers(self):
        self._vision_compose_stop.set()
        for cond in (
            getattr(self, "_vision_compose_cv_1", None),
            getattr(self, "_vision_compose_cv_2", None),
        ):
            if cond is None:
                continue
            with cond:
                cond.notify_all()
        for attr in ("_vision_compose_thread_1", "_vision_compose_thread_2"):
            thread = getattr(self, attr, None)
            if thread is None:
                continue
            thread.join(timeout=0.8)
            setattr(self, attr, None)

    def _queue_vision_frame_for_compose(self, panel_index: int, bgr, now=None, stream_token=None):
        if bgr is None:
            return
        panel = 2 if int(panel_index) == 2 else 1
        t_now = time.monotonic() if now is None else float(now)
        h, w = bgr.shape[:2]
        if panel == 2:
            self._last_yolo_image_size_2 = (w, h)
            cond = self._vision_compose_cv_2
            token = int(self._vision_stream_token_2 if stream_token is None else stream_token)
            with cond:
                self._latest_vision_bgr_2 = bgr
                self._latest_vision_frame_at_2 = t_now
                self._latest_vision_token_2 = token
                self._vision_compose_pending_2 = True
                cond.notify()
        else:
            self._last_yolo_image_size = (w, h)
            cond = self._vision_compose_cv_1
            token = int(self._vision_stream_token_1 if stream_token is None else stream_token)
            with cond:
                self._latest_vision_bgr_1 = bgr
                self._latest_vision_frame_at_1 = t_now
                self._latest_vision_token_1 = token
                self._vision_compose_pending_1 = True
                cond.notify()

    def _cache_latest_raw_vision_frame(self, panel_index: int, bgr):
        if bgr is None:
            return
        panel = 2 if int(panel_index) == 2 else 1
        if panel == 2:
            with self._vision_raw_frame_lock_2:
                self._last_raw_bgr_2 = bgr
        else:
            with self._vision_raw_frame_lock_1:
                self._last_raw_bgr_1 = bgr

    def _request_vision_overlay_refresh(self, panel_index: int):
        panel = 2 if int(panel_index) == 2 else 1
        now = time.monotonic()
        if panel == 2:
            if not bool(self._top_status_enabled.get("vision2", True)):
                return
            last_refresh_at = float(getattr(self, "_vision_overlay_refresh_last_at_2", 0.0) or 0.0)
            last_camera_at = getattr(self, "_last_camera_frame_at_2", None)
            if (
                (now - last_refresh_at) < float(VISION_OVERLAY_REFRESH_MIN_INTERVAL_SEC)
                and (last_camera_at is not None)
                and ((now - float(last_camera_at)) <= float(VISION_OVERLAY_REFRESH_FORCE_STALE_SEC))
            ):
                return
            self._vision_overlay_refresh_last_at_2 = now
            token = int(getattr(self, "_vision_stream_token_2", 0))
            with self._vision_raw_frame_lock_2:
                raw_bgr = None if self._last_raw_bgr_2 is None else self._last_raw_bgr_2.copy()
        else:
            if not bool(self._top_status_enabled.get("vision", True)):
                return
            last_refresh_at = float(getattr(self, "_vision_overlay_refresh_last_at_1", 0.0) or 0.0)
            last_camera_at = getattr(self, "_last_camera_frame_at_1", None)
            if (
                (now - last_refresh_at) < float(VISION_OVERLAY_REFRESH_MIN_INTERVAL_SEC)
                and (last_camera_at is not None)
                and ((now - float(last_camera_at)) <= float(VISION_OVERLAY_REFRESH_FORCE_STALE_SEC))
            ):
                return
            self._vision_overlay_refresh_last_at_1 = now
            token = int(getattr(self, "_vision_stream_token_1", 0))
            with self._vision_raw_frame_lock_1:
                raw_bgr = None if self._last_raw_bgr_1 is None else self._last_raw_bgr_1.copy()
        if raw_bgr is None:
            return
        self._queue_vision_frame_for_compose(panel, raw_bgr, now=now, stream_token=token)

    def _compose_vision_frame_qimage(self, panel_index: int, bgr):
        panel = 2 if int(panel_index) == 2 else 1
        frame = np.ascontiguousarray(bgr)
        h, w = frame.shape[:2]
        if panel == 2:
            self._last_yolo_image_size_2 = (w, h)
            calib_on = bool(self._calibration_mode_enabled_2)
            calib_data = self._calib_last_points_uvz_mm_2
        else:
            self._last_yolo_image_size = (w, h)
            calib_on = bool(self._calibration_mode_enabled_1)
            calib_data = self._calib_last_points_uvz_mm_1
        if not calib_on:
            frame = self._draw_vision_runtime_meta_overlay(frame, panel_index=panel)
        frame = self._draw_vision_axes_overlay(frame, panel_index=panel)
        if calib_on:
            if calib_data is not None:
                frame = self._draw_calib_points_only(frame, calib_data, panel_index=panel)
            frame = self._draw_calib_info_overlay(frame, calib_data)
        rgb = np.ascontiguousarray(frame[:, :, ::-1])
        return QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()

    def _vision_compose_loop(self, panel_index: int):
        
        panel = 2 if int(panel_index) == 2 else 1
        if panel == 2:
            cond = self._vision_compose_cv_2
        else:
            cond = self._vision_compose_cv_1
        while not self._vision_compose_stop.is_set():
            with cond:
                while not self._vision_compose_stop.is_set():
                    pending = self._vision_compose_pending_2 if panel == 2 else self._vision_compose_pending_1
                    if pending:
                        break
                    cond.wait(timeout=0.2)
                if self._vision_compose_stop.is_set():
                    return
                if panel == 2:
                    bgr = self._latest_vision_bgr_2
                    frame_at = self._latest_vision_frame_at_2
                    token = self._latest_vision_token_2
                    self._latest_vision_bgr_2 = None
                    self._latest_vision_frame_at_2 = None
                    self._vision_compose_pending_2 = False
                else:
                    bgr = self._latest_vision_bgr_1
                    frame_at = self._latest_vision_frame_at_1
                    token = self._latest_vision_token_1
                    self._latest_vision_bgr_1 = None
                    self._latest_vision_frame_at_1 = None
                    self._vision_compose_pending_1 = False
            if bgr is None:
                continue
            current_token = int(getattr(self, "_vision_stream_token_2", 0)) if panel == 2 else int(getattr(self, "_vision_stream_token_1", 0))
            if int(token) != current_token:
                continue
            compose_started_at = time.monotonic()
            try:
                qimg = self._compose_vision_frame_qimage(panel, bgr)
            except Exception:
                continue
            compose_ms = (time.monotonic() - compose_started_at) * 1000.0
            if panel == 2:
                self._vision_compose_ms_2 = compose_ms
                self._enqueue_vision_frame_2(qimg, now=frame_at, stream_token=token)
            else:
                self._vision_compose_ms = compose_ms
                self._enqueue_vision_frame(qimg, now=frame_at, stream_token=token)
             

    def _clear_vision_view_data(self, panel_index: int):
        panel = 2 if int(panel_index) == 2 else 1
        if panel == 2:
            with self._vision_frame_lock_2:
                self._pending_yolo_qimage_2 = None
                self._pending_vision_token_2 = 0
                self._vision_frame_pending_2 = False
            with self._vision_raw_frame_lock_2:
                self._last_raw_bgr_2 = None
            with self._vision_compose_cv_2:
                self._latest_vision_bgr_2 = None
                self._latest_vision_frame_at_2 = None
                self._latest_vision_token_2 = 0
                self._vision_compose_pending_2 = False
            self._last_yolo_qimage_2 = None
            self._vision_prev_update_at_2 = None
            self._vision_cycle_ms_2 = None
            self._vision_decode_ms_2 = None
            self._vision_render_delay_ms_2 = None
            self._vision_render_interval_ms_2 = None
            self._vision_compose_ms_2 = None
            self._pending_vision_enqueued_at_2 = None
            self._vision_render_prev_at_2 = None
            self._vision_depth_image_2 = None
            self._vision_depth_shape_2 = None
            self._vision_depth_encoding_2 = None
            self._camera_intrinsics_2 = None
            self._vision_meta_payload_2 = None
            self._vision_meta_received_at_2 = None
            self._vision_meta_last_nonempty_payload_2 = None
            self._vision_meta_last_nonempty_at_2 = None
            self._vision_meta_cycle_ms_2 = None
        else:
            with self._vision_frame_lock_1:
                self._pending_yolo_qimage = None
                self._pending_vision_token_1 = 0
                self._vision_frame_pending = False
            with self._vision_raw_frame_lock_1:
                self._last_raw_bgr_1 = None
            with self._vision_compose_cv_1:
                self._latest_vision_bgr_1 = None
                self._latest_vision_frame_at_1 = None
                self._latest_vision_token_1 = 0
                self._vision_compose_pending_1 = False
            self._last_yolo_qimage = None
            self._vision_prev_update_at = None
            self._vision_cycle_ms = None
            self._vision_decode_ms = None
            self._vision_render_delay_ms = None
            self._vision_render_interval_ms = None
            self._vision_compose_ms = None
            self._pending_vision_enqueued_at_1 = None
            self._vision_render_prev_at = None
            self._vision_depth_image = None
            self._vision_depth_shape = None
            self._vision_depth_encoding = None
            self._camera_intrinsics_1 = None
            self._camera_intrinsics = None
            self._vision_meta_payload_1 = None
            self._vision_meta_received_at_1 = None
            self._vision_meta_last_nonempty_payload_1 = None
            self._vision_meta_last_nonempty_at_1 = None
            self._vision_meta_cycle_ms_1 = None
        if panel == 2:
            self._last_vision_frame_at_2 = None
            view = getattr(self, "yolo_view_2", None)
        else:
            self._last_vision_frame_at = None
            view = getattr(self, "yolo_view", None)
        if view is not None:
            view.clear()
            view.setPixmap(QPixmap())
            view.setAlignment(Qt.AlignCenter)
            view.setStyleSheet("background-color: #000000; color: #f3f3f3;")
            view.setText("이미지 데이터 없음")

    def _clear_vision_depth_cache(self, panel_index: int):
        panel = 2 if int(panel_index) == 2 else 1
        if panel == 2:
            self._vision_depth_image_2 = None
            self._vision_depth_shape_2 = None
            self._vision_depth_encoding_2 = None
        else:
            self._vision_depth_image = None
            self._vision_depth_shape = None
            self._vision_depth_encoding = None

    def _clear_calibration_panel_state(self, panel_index: int):
        panel = 2 if int(panel_index) == 2 else 1
        if panel == 2:
            self._calib_last_points_uvz_mm_2 = None
            self._calib_last_points_at_2 = None
            self._calib_last_grid_pts_2 = None
            self._calib_last_grid_status_2 = None
            self._calib_grid_camera_xyz_2 = None
            self._calib_last_reason_2 = "대기"
            self._calib_full_ready_2 = False
            self._calib_full_reason_2 = "대기"
        else:
            self._calib_last_points_uvz_mm_1 = None
            self._calib_last_points_at_1 = None
            self._calib_last_grid_pts_1 = None
            self._calib_last_grid_status_1 = None
            self._calib_grid_camera_xyz_1 = None
            self._calib_last_reason_1 = "대기"
            self._calib_full_ready_1 = False
            self._calib_full_reason_1 = "대기"
            self._calib_last_points_uvz_mm = None
            self._calib_last_points_at = None
            self._calib_last_reason = "대기"
        self.calibration_ui_refresh_requested.emit()

    def _advance_vision_stream_token(self, panel_index: int):
        panel = 2 if int(panel_index) == 2 else 1
        if panel == 2:
            self._vision_stream_token_2 += 1
            return self._vision_stream_token_2
        self._vision_stream_token_1 += 1
        return self._vision_stream_token_1

    def _vision_mode_switch_grace_until(self, panel_index: int):
        panel = 2 if int(panel_index) == 2 else 1
        return float(
            getattr(
                self,
                "_vision_mode_switch_grace_until_2" if panel == 2 else "_vision_mode_switch_grace_until_1",
                0.0,
            )
        )

    def _camera_status_for_panel(self, panel_index: int, now: float, enabled: bool):
        panel = 2 if int(panel_index) == 2 else 1
        if not enabled:
            return "비활성화"
        last_camera_frame_at = (
            getattr(self, "_last_camera_frame_at_2", None) if panel == 2 else getattr(self, "_last_camera_frame_at_1", None)
        )
        if last_camera_frame_at is None:
            return "확인중" if now < self._vision_mode_switch_grace_until(panel) else "끊김"
        if (now - float(last_camera_frame_at)) > VISION_CAMERA_STALE_SEC:
            return "지연"
        return "정상 수신 중"

    def _clear_robot_data_view(self):
        self._set_signal_state_label(self._robot_state_label, "비활성화", "warning", False)
        for i in range(6):
            self._set_table_value(i, "-", "", flash=False)
        self._clear_position_views()
        self._set_current_tool_text("-")

    def _clear_position_views(self):
        for table in (getattr(self, "_joint_table", None), getattr(self, "_cart_table", None)):
            if table is None:
                continue
            for row in range(table.rowCount()):
                item = table.item(row, 1)
                if item is None:
                    item = QTableWidgetItem("-")
                    table.setItem(row, 1, item)
                item.setText("-")
                item.setForeground(QBrush(QColor(POSITION_VALUE_COLOR)))
                font = item.font()
                font.setBold(False)
                item.setFont(font)
        self._pos_value_cache.clear()
        self._pos_flash_until.clear()
        self._last_positions_seen_at = None

    def _reposition_top_status_row(self):
        if not self._top_status_widgets:
            return
        # If panel/box widgets were removed in .ui, preserve Designer geometry.
        if self._top_status_panel is None:
            return
        frame_l = getattr(self, "frame", None)
        frame_r = getattr(self, "frame_2", None)
        use_frame_alignment = frame_l is not None and frame_r is not None
        frame_m = getattr(self, "frame_4", None)
        if use_frame_alignment and frame_m is not None:
            # 3열 재배치에서 frame_4/robot이 같은 x축을 공유하면 슬롯 기반 배치를 사용한다.
            if abs(int(frame_m.x()) - int(frame_r.x())) <= 1:
                use_frame_alignment = False

        left = 10
        y = 4
        if use_frame_alignment:
            panel_left = min(frame_l.x(), frame_r.x())
            panel_right = max(frame_l.x() + frame_l.width(), frame_r.x() + frame_r.width())
            left = panel_left
            total_w = max(120, panel_right - panel_left)
        else:
            total_w = max(400, self.centralwidget.width() - 20)
        panel_h = max(28, TOP_STATUS_BAR_HEIGHT - 6)
        if self._top_status_panel is not None:
            self._top_status_panel.setGeometry(left, y, total_w, panel_h)
        if use_frame_alignment and "vision" in self._top_status_widgets and "robot" in self._top_status_widgets:
            v = self._top_status_widgets.get("vision")
            v2 = self._top_status_widgets.get("vision2")
            r = self._top_status_widgets.get("robot")
            if v is None or r is None:
                return
            v_title, v_box, v_dot, v_text = v
            r_title, r_box, r_dot, r_text = r
            if v_box is None or r_box is None:
                return

            v_x = frame_l.x() - left
            m_x = frame_m.x() - left if frame_m is not None else None
            m_w = frame_m.width() if frame_m is not None else None
            r_x = frame_r.x() - left
            v_w = frame_l.width()
            r_w = frame_r.width()

            v_box.setGeometry(v_x, 1, max(90, v_w), panel_h - 2)
            v_dot.setGeometry(6, 1, 16, v_box.height() - 2)
            v_text.setGeometry(24, 1, max(64, v_box.width() - 28), v_box.height() - 2)

            if v2 is not None:
                _t2, v2_box, v2_dot, v2_text = v2
                if v2_box is not None and m_x is not None and m_w is not None:
                    v2_box.setGeometry(m_x, 1, max(90, m_w), panel_h - 2)
                    v2_dot.setGeometry(6, 1, 16, v2_box.height() - 2)
                    v2_text.setGeometry(24, 1, max(64, v2_box.width() - 28), v2_box.height() - 2)

            r_box.setGeometry(r_x, 1, max(90, r_w), panel_h - 2)
            r_dot.setGeometry(6, 1, 16, r_box.height() - 2)
            r_text.setGeometry(24, 1, max(64, r_box.width() - 28), r_box.height() - 2)
            return

        count = max(1, len(self._top_status_widgets))
        side_margin = 4
        box_gap = 8
        usable_w = max(120, total_w - (2 * side_margin) - ((count - 1) * box_gap))
        slot_w = int(usable_w / count)
        x = side_margin
        for idx, key in enumerate(self._top_status_widgets.keys()):
            title, box, dot, text = self._top_status_widgets[key]
            if box is None:
                continue
            if idx == (count - 1):
                box_w = max(90, total_w - side_margin - x)
            else:
                box_w = max(90, slot_w)
            box.setGeometry(x, 1, box_w, panel_h - 2)
            dot.setGeometry(6, 1, 16, box.height() - 2)
            text.setGeometry(24, 1, max(64, box.width() - 28), box.height() - 2)
            x += box_w + box_gap

    def _set_top_status(self, key: str, state_text: str, severity: str):
        self._top_status_state_cache[str(key)] = (str(state_text), str(severity))
        item = self._top_status_widgets.get(key)
        if item is None:
            return
        title, box, dot, text = item
        # 1-second fade cycle.
        phase = time.monotonic() % 1.0  # 0..1 per second
        # 0 -> 1 -> 0 within 1 second
        fade = 0.5 * (1.0 + np.sin((2.0 * np.pi * phase) - (np.pi / 2.0)))
        state_text_norm = str(state_text or "").strip()
        should_blink = (str(key) in {"vision", "vision2"}) or (state_text_norm in {
            "비활성화",
            "끊김",
            "대기",
            "초기화",
            "초기화중",
            "지연",
            "중지",
            "시작 중",
            "확인중",
            "연결",
            "정상 수신 중",
        })
        def _hex_to_rgb(h: str):
            h = h.lstrip("#")
            return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)

        def _rgb_to_hex(rgb):
            r, g, b = rgb
            return f"#{int(max(0, min(255, r))):02x}{int(max(0, min(255, g))):02x}{int(max(0, min(255, b))):02x}"

        def _mix(c1: str, c2: str, t: float):
            r1, g1, b1 = _hex_to_rgb(c1)
            r2, g2, b2 = _hex_to_rgb(c2)
            return _rgb_to_hex((r1 + (r2 - r1) * t, g1 + (g2 - g1) * t, b1 + (b2 - b1) * t))

        if severity == "normal":
            bright, dim = STATE_COLOR_NORMAL, "#ffffff"
            bg_bright, bg_dim = "#eef9f1", "#fbfcfb"
        elif severity == "error":
            bright, dim = STATE_COLOR_ERROR, "#ffffff"
            bg_bright, bg_dim = "#fdeeee", "#fffafb"
        else:
            bright, dim = STATE_COLOR_WARNING, "#ffffff"
            bg_bright, bg_dim = "#fff5e8", "#fffdf9"

        if should_blink:
            color = _mix(dim, bright, fade)
            bg = _mix(bg_dim, bg_bright, fade)
            alpha = int(max(0, min(255, round(255.0 * fade))))
        else:
            color = bright
            bg = bg_bright
            alpha = 255
        r, g, b = _hex_to_rgb(color)
        dot.setStyleSheet(f"color: rgba({r}, {g}, {b}, {alpha}); font-size: 20pt;")
        text.setStyleSheet("color: #202020; font-weight: 700; font-size: 14pt;")
        if box is not None:
            box.setStyleSheet(f"background: {bg}; border: 1px solid #cfd8dc; border-radius: 3px;")
        text.setText(f"{title}: {state_text}")

    def _tick_top_status_animation(self):
        if self._top_status_state_cache:
            for key, payload in list(self._top_status_state_cache.items()):
                if not isinstance(payload, tuple) or len(payload) != 2:
                    continue
                state_text, severity = payload
                self._set_top_status(key, state_text, severity)
        voice_state = str(getattr(self, "_voice_order_status_text", "") or "").strip()
        if voice_state:
            self._set_voice_order_status(
                voice_state,
                str(getattr(self, "_voice_order_status_severity", "info") or "info"),
            )
        now = time.monotonic()
        if (
            now - float(getattr(self, "_bartender_mode_ui_last_update_at", 0.0))
        ) >= float(BARTENDER_MODE_UI_REFRESH_MIN_INTERVAL_SEC):
            self._bartender_mode_ui_last_update_at = now
            self._update_bartender_mode_ui()
            self._refresh_bartender_sequence_styles()

    def _set_robot_controls_enabled(self, enabled: bool):
        enabled = bool(enabled)
        calib_seq_running = bool(getattr(self, "_calibration_sequence_running", False))
        calib_on = bool(
            getattr(self, "_calibration_mode_enabled_1", False)
            or getattr(self, "_calibration_mode_enabled_2", False)
        )
        mode_value = None
        mode_seen_at = None
        if self.backend is not None and hasattr(self.backend, "get_robot_mode_snapshot"):
            try:
                mode_value, mode_seen_at = self.backend.get_robot_mode_snapshot()
            except Exception:
                mode_value, mode_seen_at = None, None
        mode_key = None
        if mode_value is not None and mode_seen_at is not None:
            try:
                mode_key = int(mode_value)
            except Exception:
                mode_key = None
        state_key = (enabled, calib_on, mode_key, calib_seq_running)
        if self._robot_controls_enabled_cache is not None and self._robot_controls_enabled_cache == state_key:
            return
        self._robot_controls_enabled_cache = state_key
        # 자동 캘리브레이션 시퀀스 진행 중에는 로봇 조작 버튼을 잠근다.
        normal_enabled = enabled and (not calib_seq_running)
        gripper_enabled = normal_enabled and (mode_key == 1)
        tool_change_enabled = normal_enabled and (mode_key == 0)

        self.pushButton.setEnabled(normal_enabled)
        if hasattr(self, "_print_pos_button") and self._print_pos_button is not None:
            self._print_pos_button.setEnabled(normal_enabled)
        if hasattr(self, "_reset_button") and self._reset_button is not None:
            self._reset_button.setEnabled(normal_enabled)
        if hasattr(self, "_emergency_stop_button") and self._emergency_stop_button is not None:
            self._emergency_stop_button.setEnabled(normal_enabled)
        if hasattr(self, "_home_button") and self._home_button is not None:
            self._home_button.setEnabled(normal_enabled)
        if hasattr(self, "_home_save_button") and self._home_save_button is not None:
            self._home_save_button.setEnabled(normal_enabled)
        if hasattr(self, "_robot_mode_button") and self._robot_mode_button is not None:
            self._robot_mode_button.setEnabled(normal_enabled)
        if hasattr(self, "_tool_change_button") and self._tool_change_button is not None:
            self._tool_change_button.setEnabled(tool_change_enabled)
        if hasattr(self, "_gripper_move_button") and self._gripper_move_button is not None:
            self._gripper_move_button.setEnabled(gripper_enabled)
        if hasattr(self, "_gripper_stroke_input") and self._gripper_stroke_input is not None:
            self._gripper_stroke_input.setEnabled(gripper_enabled)
        if hasattr(self, "_vision_move_button") and self._vision_move_button is not None:
            self._vision_move_button.setEnabled(normal_enabled)
        for w in [
            getattr(self, "_vision_x_margin_input", None),
            getattr(self, "_vision_y_margin_input", None),
            getattr(self, "_vision_z_margin_input", None),
        ]:
            if w is not None:
                w.setEnabled(normal_enabled)
        if hasattr(self, "_vision_dialog_toggle_button") and self._vision_dialog_toggle_button is not None:
            self._vision_dialog_toggle_button.setEnabled(normal_enabled)
        if hasattr(self, "_vision_dialog_toggle_switch") and self._vision_dialog_toggle_switch is not None:
            # 클릭 확인 스위치는 백엔드 준비 전에도 사용자가 미리 바꿀 수 있게 항상 활성화한다.
            self._vision_dialog_toggle_switch.setEnabled(True)
        if hasattr(self, "_calibration_mode_switch") and self._calibration_mode_switch is not None:
            self._calibration_mode_switch.setEnabled(True)
        if hasattr(self, "_calibration_mode_switch_2") and self._calibration_mode_switch_2 is not None:
            self._calibration_mode_switch_2.setEnabled(True)
        if hasattr(self, "_calibration_transform_button") and self._calibration_transform_button is not None:
            self._calibration_transform_button.setEnabled(
                bool(normal_enabled and self._top_status_enabled.get("vision", True))
            )
        if hasattr(self, "_calibration_transform_button_2") and self._calibration_transform_button_2 is not None:
            self._calibration_transform_button_2.setEnabled(
                bool(normal_enabled and self._top_status_enabled.get("vision2", True))
            )
        if hasattr(self, "_calibration_load_button") and self._calibration_load_button is not None:
            self._calibration_load_button.setEnabled(bool((not calib_seq_running) and self._calibration_mode_enabled_1))
        if hasattr(self, "_calibration_load_button_2") and self._calibration_load_button_2 is not None:
            self._calibration_load_button_2.setEnabled(bool((not calib_seq_running) and self._calibration_mode_enabled_2))
        self._set_vision_panel_controls_enabled(1, bool(self._top_status_enabled.get("vision", True)))
        self._set_vision_panel_controls_enabled(2, bool(self._top_status_enabled.get("vision2", True)))

    def _start_backend_async(self):
        if self.backend is not None and hasattr(self.backend, "is_ready") and self.backend.is_ready():
            self._on_backend_ready(self.backend)
            return
        if self._backend_thread is not None:
            return

        self.append_log("[초기화] 백엔드 시작 중...\n")
        self._backend_thread = QThread(self)
        self._backend_worker = BackendStartupWorker(use_real_gripper=STARTUP_USE_REAL_GRIPPER)
        self._backend_worker.moveToThread(self._backend_thread)

        self._backend_thread.started.connect(self._backend_worker.run)
        self._backend_worker.progress.connect(self._on_backend_progress)
        self._backend_worker.finished.connect(self._on_backend_ready)
        self._backend_worker.failed.connect(self._on_backend_failed)

        self._backend_worker.finished.connect(self._backend_thread.quit)
        self._backend_worker.failed.connect(lambda *_: self._backend_thread.quit())
        self._backend_thread.finished.connect(self._backend_thread.deleteLater)
        self._backend_thread.finished.connect(self._backend_worker.deleteLater)
        self._backend_thread.finished.connect(self._clear_backend_worker_refs)

        self._backend_thread.start()

    def _clear_backend_worker_refs(self):
        self._backend_thread = None
        self._backend_worker = None

    def _clear_reset_worker_refs(self):
        self._reset_thread = None
        self._reset_worker = None

    def _clear_motion_stop_worker_refs(self):
        self._motion_stop_thread = None
        self._motion_stop_worker = None

    def _start_reset_async(self):
        if self.backend is None:
            self.append_log("[리셋] 백엔드 초기화 중입니다.\n")
            return
        if self._reset_thread is not None:
            self.append_log("[리셋] 이미 진행 중입니다.\n")
            return
        if not hasattr(self.backend, "reset_robot_state"):
            self.append_log("[리셋] 백엔드 리셋 기능을 지원하지 않습니다.\n")
            return

        self._set_robot_controls_enabled(False)
        self.append_log("[리셋] 요청 전송, 응답 대기 중...\n")

        self._reset_thread = QThread(self)
        self._reset_worker = BackendResetWorker(self.backend)
        self._reset_worker.moveToThread(self._reset_thread)

        self._reset_thread.started.connect(self._reset_worker.run)
        self._reset_worker.finished.connect(self._on_reset_finished)
        self._reset_worker.failed.connect(self._on_reset_failed)

        self._reset_worker.finished.connect(lambda *_: self._reset_thread.quit())
        self._reset_worker.failed.connect(lambda *_: self._reset_thread.quit())
        self._reset_thread.finished.connect(self._reset_thread.deleteLater)
        self._reset_thread.finished.connect(self._reset_worker.deleteLater)
        self._reset_thread.finished.connect(self._clear_reset_worker_refs)
        self._reset_thread.start()

    def _on_reset_finished(self, ok, msg):
        self.append_log(f"[리셋] {msg}\n")
        self._set_robot_controls_enabled(self.backend is not None and self.backend.is_ready())

    def _on_reset_failed(self, err, tb):
        self.append_log(f"[리셋] 실패: {err}\n")
        self.append_log(tb + "\n")
        self._set_robot_controls_enabled(self.backend is not None and self.backend.is_ready())

    def _start_motion_stop_async(self, stop_mode: int = 2):
        if self.backend is None:
            self.append_log("[정지] 백엔드 초기화 중입니다.\n")
            return
        if self._motion_stop_thread is not None:
            self.append_log("[정지] 이미 요청 처리 중입니다.\n")
            return
        if not hasattr(self.backend, "send_motion_stop"):
            self.append_log("[정지] 백엔드가 모션정지 기능을 지원하지 않습니다.\n")
            return

        self.append_log(f"[정지] 요청 전송(stop_mode={int(stop_mode)}), 응답 대기 중...\n")
        self._motion_stop_thread = QThread(self)
        self._motion_stop_worker = BackendMotionStopWorker(self.backend, stop_mode=int(stop_mode))
        self._motion_stop_worker.moveToThread(self._motion_stop_thread)

        self._motion_stop_thread.started.connect(self._motion_stop_worker.run)
        self._motion_stop_worker.finished.connect(self._on_motion_stop_finished)
        self._motion_stop_worker.failed.connect(self._on_motion_stop_failed)

        self._motion_stop_worker.finished.connect(lambda *_: self._motion_stop_thread.quit())
        self._motion_stop_worker.failed.connect(lambda *_: self._motion_stop_thread.quit())
        self._motion_stop_thread.finished.connect(self._motion_stop_thread.deleteLater)
        self._motion_stop_thread.finished.connect(self._motion_stop_worker.deleteLater)
        self._motion_stop_thread.finished.connect(self._clear_motion_stop_worker_refs)
        self._motion_stop_thread.start()

    def _on_motion_stop_finished(self, ok, msg):
        self.append_log(f"[정지] {msg}\n")

    def _on_motion_stop_failed(self, err, tb):
        self.append_log(f"[정지] 실패: {err}\n")
        self.append_log(tb + "\n")

    def _on_backend_progress(self, percent, message, elapsed_sec):
        self.append_log(f"[초기화] {int(percent)}% {message} ({elapsed_sec:.1f}s)\n")

    def _on_backend_ready(self, backend):
        self.backend = backend
        self._backend_ready_at = time.monotonic()
        self.append_log("[초기화] 백엔드 초기화 완료\n")
        self._request_current_tool_sync(retry_window_sec=20.0, immediate=True)
        if not bool(self._top_status_enabled.get("robot", True)):
            if hasattr(self.backend, "stop_robot_state_subscriptions"):
                try:
                    ok, msg = self.backend.stop_robot_state_subscriptions()
                    self.append_log(f"[로봇] {msg}\n")
                except Exception as e:
                    self.append_log(f"[로봇] 구독 종료 실패: {e}\n")
        self._set_robot_controls_enabled(bool(self._top_status_enabled.get("robot", True)))
        self._setup_rosout_bridge()
        self._start_yolo_camera()

    def _apply_backend_failed_state(self, err, tb, show_popup=True):
        self.append_log(f"[초기화] 실패: {err}\n")
        if tb:
            self.append_log(f"{tb}\n")
        self._set_robot_controls_enabled(False)
        self._vision_state_text = "오류"
        self._vision_state_text_2 = "오류"
        if hasattr(self, "yolo_view") and self.yolo_view is not None:
            self.yolo_view.setText("초기화 실패")
        if hasattr(self, "yolo_view_2") and self.yolo_view_2 is not None:
            self.yolo_view_2.setText("초기화 실패")
        if show_popup:
            QMessageBox.warning(self, "초기화 실패", f"백엔드 시작 실패\n\n{err}")

    def _on_backend_failed(self, err, tb):
        self._apply_backend_failed_state(err, tb, show_popup=True)

    def _refresh_status(self):
        # Backward-compat wrapper
        seq_snapshot = self._pull_bartender_sequence_snapshot()
        if isinstance(seq_snapshot, dict):
            self._apply_bartender_sequence_snapshot(seq_snapshot)
        self._refresh_robot_status()
        self._refresh_vision_status()

    def _refresh_robot_status(self):
        seq_snapshot = self._pull_bartender_sequence_snapshot()
        if isinstance(seq_snapshot, dict):
            self._apply_bartender_sequence_snapshot(seq_snapshot)

        if not bool(self._top_status_enabled.get("robot", True)):
            self._robot_cycle_ms = None
            self._set_robot_controls_enabled(False)
            self._clear_robot_data_view()
            self._set_top_status("robot", "비활성화", "warning")
            self._update_cycle_time_labels()
            return
        if self.backend is None:
            self._set_robot_controls_enabled(False)
            self._set_signal_state_label(self._robot_state_label, "초기화 중", "warning", False)
            self._set_table_value(0, "초기화", "warning", flash=False)
            self._set_table_value(1, "-", "", flash=False)
            self._set_table_value(2, "-", "", flash=False)
            self._set_table_value(3, "-", "", flash=False)
            self._set_table_value(4, "-", "", flash=False)
            self._set_table_value(5, "-", "", flash=False)
            self._set_top_status("robot", "초기화중", "warning")
            self._update_cycle_time_labels()
            return

        mode_text = self._detect_robot_mode_text()
        robot_mode_value = None
        robot_mode_seen_at = None
        if hasattr(self.backend, "get_robot_mode_snapshot"):
            robot_mode_value, robot_mode_seen_at = self.backend.get_robot_mode_snapshot()
        control_mode_value = None
        control_mode_seen_at = None
        if hasattr(self.backend, "get_control_mode_snapshot"):
            control_mode_value, control_mode_seen_at = self.backend.get_control_mode_snapshot()
        state_code = None
        state_name = ""
        state_seen_at = None
        if hasattr(self.backend, "get_robot_state_snapshot"):
            state_code, state_name, state_seen_at = self.backend.get_robot_state_snapshot()
        position_seen_at = None
        if hasattr(self.backend, "get_position_snapshot"):
            try:
                _pos_data, position_seen_at = self.backend.get_position_snapshot()
            except Exception:
                position_seen_at = None

        now = time.monotonic()
        if state_seen_at is not None:
            if self._robot_prev_state_seen_at is not None:
                dt = state_seen_at - self._robot_prev_state_seen_at
                if dt > 0.0:
                    self._robot_cycle_ms = dt * 1000.0
            self._robot_prev_state_seen_at = state_seen_at
        position_stream_alive = position_seen_at is not None and (now - position_seen_at) <= POSITION_STALE_SEC
        stale = state_seen_at is None or (now - state_seen_at) > POSITION_STALE_SEC
        startup_connecting = (
            self.backend.is_ready()
            and state_seen_at is None
            and (
                position_stream_alive
                or (
                    self._backend_ready_at is not None
                    and (now - float(self._backend_ready_at)) < ROBOT_STARTUP_CONNECT_GRACE_SEC
                )
            )
        )
        # _last_error는 과거 예외 이력이 누적될 수 있으므로, UI 상태등급은 현재 상태코드 기준으로 판정한다.
        has_error = (state_code is not None) and (int(state_code) in ROBOT_STATE_ERROR_CODES)
        comm_connected = (not stale) and (state_code is not None)
        if startup_connecting:
            stale = False
            comm_connected = False
        if now < float(getattr(self, "_mode_switch_grace_until", 0.0)):
            # 전환 직후 짧은 공백은 연결 끊김으로 보지 않는다.
            if self.backend.is_ready() and ((state_code is None) or stale):
                comm_connected = bool(getattr(self, "_last_robot_comm_connected", True))
                stale = False
        self._last_robot_comm_connected = bool(comm_connected)
        controls_enabled = self.backend.is_ready() and comm_connected
        if self._reset_thread is not None and self._reset_thread.isRunning():
            controls_enabled = False
        self._set_robot_controls_enabled(controls_enabled)

        if state_code is not None:
            state_name = (state_name or f"STATE_{state_code}").strip()
            state_text = ROBOT_STATE_KR_MAP.get(state_name, state_name)
            state_label_text = f"{state_text} ({state_name})"
            if int(state_code) == 1:
                # STANDBY(대기)는 에러 이력과 무관하게 정상색(초록)으로 유지한다.
                robot_severity = "normal"
            elif has_error or int(state_code) in ROBOT_STATE_ERROR_CODES:
                robot_severity = "error"
            elif stale or int(state_code) in ROBOT_STATE_WARNING_CODES:
                robot_severity = "warning"
            elif int(state_code) in ROBOT_STATE_NORMAL_CODES:
                robot_severity = "normal"
            else:
                robot_severity = "warning"
        else:
            if startup_connecting:
                state_text = "연결중"
                robot_severity = "warning"
            elif not self.backend.is_ready():
                state_text = "초기화 중"
                robot_severity = "warning"
            elif has_error:
                state_text = "오류"
                robot_severity = "error"
            elif self.backend.is_busy():
                # 명령 처리 중이더라도 실제 로봇 상태 미수신이면 "동작"으로 단정하지 않는다.
                state_text = "요청중"
                robot_severity = "normal"
            else:
                state_text = "대기"
                robot_severity = "warning"
            state_label_text = state_text

        if state_text != self._last_robot_state_text:
            self._last_robot_state_text = state_text
            self._robot_state_flash_until = now + STATE_FLASH_SEC
        flash_state = now < self._robot_state_flash_until

        self._set_signal_state_label(self._robot_state_label, state_label_text, robot_severity, flash_state)
        self._update_robot_state_table(
            state_code=state_code,
            state_name=state_name,
            stale=stale,
            startup_connecting=startup_connecting,
            mode_text=mode_text,
            robot_mode_value=robot_mode_value,
            robot_mode_seen_at=robot_mode_seen_at,
            control_mode_value=control_mode_value,
            control_mode_seen_at=control_mode_seen_at,
            app_ready=self.backend.is_ready(),
            app_busy=self.backend.is_busy(),
            has_error=has_error,
        )
        if startup_connecting:
            robot_link_text = "연결중"
            robot_link_sev = "warning"
        else:
            robot_link_text = "정상 수신 중" if comm_connected else "끊김"
            robot_link_sev = "normal" if comm_connected else "error"
        self._set_top_status("robot", robot_link_text, robot_link_sev)
        self._update_cycle_time_labels()

    def _refresh_vision_status(self):
        self._refresh_vision_status_panel(1)
        self._refresh_vision_status_panel(2)

    def _schedule_vision_recovery(self, panel_index: int):
        panel = 2 if int(panel_index) == 2 else 1
        pending = getattr(self, "_vision_recovery_pending_panels", None)
        if pending is None:
            self._vision_recovery_pending_panels = {panel}
            return
        pending.add(panel)

    def _process_vision_recovery_queue(self):
        pending = getattr(self, "_vision_recovery_pending_panels", None)
        if not pending:
            return
        panel = 1 if 1 in pending else min(pending)
        pending.discard(panel)
        calib_on_any = bool(
            getattr(self, "_calibration_mode_enabled_1", False)
            or getattr(self, "_calibration_mode_enabled_2", False)
        )
        if (not calib_on_any) and YOLO_EXTERNAL_NODE and YOLO_AUTO_LAUNCH_NODE:
            self._ensure_external_vision_process_panel(panel)
        self._sync_calibration_process_panel(panel)
        self._rebind_external_vision_bridge_panel_for_mode(panel)

    def _refresh_vision_status_panel(self, panel_index: int):
        panel = 2 if int(panel_index) == 2 else 1
        key = "vision2" if panel == 2 else "vision"
        now = time.monotonic()
        vision_on = bool(self._top_status_enabled.get(key, True))
        retry_needed = False
        if panel == 2:
            if not vision_on:
                self._vision_cycle_ms_2 = None
        else:
            if not vision_on:
                self._vision_cycle_ms = None
        if self.backend is None:
            if panel == 1:
                self._set_signal_state_label(self._vision_state_label, self._vision_state_text, "warning", False)
            self._set_top_status(key, "비활성화" if (not vision_on) else "끊김", "warning" if (not vision_on) else "error")
            if not vision_on:
                self._clear_vision_view_data(panel_index=panel)
            self._update_cycle_time_labels()
            return

        if not UI_ENABLE_VISION:
            vision_text = "비활성화"
        else:
            if not vision_on:
                vision_text = "비활성화"
                self._clear_vision_view_data(panel_index=panel)
            else:
                vision_text = self._camera_status_for_panel(panel, now, vision_on)

        if now < self._vision_mode_switch_grace_until(panel) and vision_text in ("지연", "끊김", "오류", "실패"):
            vision_text = "전환중"

        calib_on_any = bool(getattr(self, "_calibration_mode_enabled_1", False) or getattr(self, "_calibration_mode_enabled_2", False))
        last_camera_at = self._last_camera_frame_at_2 if panel == 2 else self._last_camera_frame_at_1
        stale_vision = bool(vision_on) and (
            (last_camera_at is None) or ((now - float(last_camera_at)) > VISION_CAMERA_STALE_SEC)
        )
        if stale_vision and ((now - self._vision_rebind_last_try_at) > 8.0):
            retry_needed = True
            self._vision_rebind_last_try_at = now
            self._schedule_vision_recovery(panel)

        if retry_needed:
            if not bool(getattr(self, "_vision_retry_notice_logged", {}).get(panel, False)):
                self._append_vision_log("연결 지연 판단: 재연결 시도를 계속 진행합니다.", panel_index=panel)
                self._vision_retry_notice_logged[panel] = True
        else:
            self._vision_retry_notice_logged[panel] = False

        if vision_text == "정상 수신 중":
            vision_severity = "normal"
        elif vision_text in ("오류", "실패", "끊김"):
            vision_severity = "error"
        else:
            vision_severity = "warning"

        if panel == 2:
            if self._last_yolo_qimage_2 is None and vision_text in ("오류", "실패", "끊김", "지연"):
                if hasattr(self, "yolo_view_2") and self.yolo_view_2 is not None:
                    self.yolo_view_2.setAlignment(Qt.AlignCenter)
                    self.yolo_view_2.setStyleSheet("background-color: #000000; color: #f3f3f3;")
                    self.yolo_view_2.setText("이미지 데이터 없음")
        else:
            if self._last_yolo_qimage is None and vision_text in ("오류", "실패", "끊김", "지연"):
                if hasattr(self, "yolo_view") and self.yolo_view is not None:
                    self.yolo_view.setAlignment(Qt.AlignCenter)
                    self.yolo_view.setStyleSheet("background-color: #000000; color: #f3f3f3;")
                    self.yolo_view.setText("이미지 데이터 없음")
            self._set_signal_state_label(self._vision_state_label, vision_text, vision_severity, False)
        self._set_top_status(key, vision_text, vision_severity)
        self._update_cycle_time_labels()
        self._update_vision_runtime_panel_ui(panel)

    def _set_signal_state_label(self, label: QLabel, text: str, severity: str, is_bold: bool = False):
        if label is None:
            return
        label.setText(text)
        if severity == "normal":
            color = STATE_COLOR_NORMAL
        elif severity == "error":
            color = STATE_COLOR_ERROR
        else:
            color = STATE_COLOR_WARNING
        weight = 700 if is_bold else 500
        label.setStyleSheet(f"color: {color}; font-weight: {weight};")

    def _set_table_value(self, row: int, value: str, severity: str = "", flash: bool = True):
        if not hasattr(self, "_robot_state_table") or self._robot_state_table is None:
            return
        now = time.monotonic()
        text_value = str(value)
        old_value = self._table_value_cache.get(row)
        if flash and old_value != text_value:
            self._table_flash_until[row] = now + STATE_FLASH_SEC
        self._table_value_cache[row] = text_value

        item = self._robot_state_table.item(row, 1)
        if item is None:
            item = QTableWidgetItem("")
            self._robot_state_table.setItem(row, 1, item)
        item.setText(text_value)

        if severity == "normal":
            color = QColor(STATE_COLOR_NORMAL)
        elif severity == "error":
            color = QColor(STATE_COLOR_ERROR)
        elif severity == "warning":
            color = QColor(STATE_COLOR_WARNING)
        else:
            color = self.palette().text().color()
        item.setForeground(QBrush(color))

        is_bold = now < self._table_flash_until.get(row, 0.0)
        font = item.font()
        font.setBold(is_bold)
        item.setFont(font)

    def _format_state_like_text(self, label: str, raw_value):
        text = str(label or "").strip()
        if raw_value is None:
            return text or "-"
        raw_text = str(raw_value).strip()
        if not raw_text:
            return text or "-"
        if not text:
            return f"({raw_text})"
        return f"{text} ({raw_text})"

    def _update_robot_state_table(
        self,
        state_code,
        state_name,
        stale,
        startup_connecting,
        mode_text,
        robot_mode_value,
        robot_mode_seen_at,
        control_mode_value,
        control_mode_seen_at,
        app_ready,
        app_busy,
        has_error,
    ):
        connection_text = {
            "REAL": "실제로봇",
            "VIRTUAL": "가상로봇",
            "UNKNOWN": "알수없음",
        }.get(str(mode_text or "UNKNOWN").upper(), str(mode_text or "UNKNOWN"))
        connection_sev = "normal" if str(mode_text or "").upper() in ("REAL", "VIRTUAL") else "warning"

        if startup_connecting:
            comm_text = "연결중"
            comm_sev = "warning"
        else:
            comm_text = "끊김" if stale else "정상 수신 중"
            comm_sev = "error" if stale else "normal"

        if state_code is None:
            robot_state_text = "초기화" if not app_ready else "대기"
            robot_state_sev = "warning" if not app_ready else ("error" if has_error else "warning")
        else:
            normalized_name = str(state_name or f"STATE_{state_code}").strip()
            robot_state_text = self._format_state_like_text(
                ROBOT_STATE_KR_MAP.get(normalized_name, normalized_name),
                int(state_code),
            )
            if int(state_code) in ROBOT_STATE_ERROR_CODES:
                robot_state_sev = "error"
            elif int(state_code) in ROBOT_STATE_NORMAL_CODES:
                robot_state_sev = "normal"
            else:
                robot_state_sev = "warning"

        if robot_mode_value is None or robot_mode_seen_at is None:
            robot_mode_text = "-"
            robot_mode_sev = "warning"
        else:
            try:
                robot_mode_raw = int(robot_mode_value)
                robot_mode_text = self._format_state_like_text(
                    ROBOT_MODE_KR_MAP.get(robot_mode_raw, f"알수없음모드"),
                    robot_mode_raw,
                )
            except Exception:
                robot_mode_text = str(robot_mode_value)
            robot_mode_sev = "normal"

        if control_mode_value is None or control_mode_seen_at is None:
            control_mode_text = "-"
            control_mode_sev = "warning"
        else:
            try:
                control_mode_raw = int(control_mode_value)
                control_mode_text = self._format_state_like_text(
                    CONTROL_MODE_KR_MAP.get(control_mode_raw, "알수없음제어"),
                    control_mode_raw,
                )
            except Exception:
                control_mode_text = str(control_mode_value)
            control_mode_sev = "normal"

        if has_error:
            work_text = "오류"
            work_sev = "error"
        elif not app_ready:
            work_text = "초기화"
            work_sev = "warning"
        elif app_busy:
            work_text = "동작중"
            work_sev = "normal"
        else:
            work_text = "대기"
            work_sev = "normal"

        self._set_table_value(0, connection_text, connection_sev, flash=False)
        self._set_table_value(1, comm_text, comm_sev)
        self._set_table_value(2, robot_state_text, robot_state_sev)
        self._set_table_value(3, robot_mode_text, robot_mode_sev, flash=False)
        self._set_table_value(4, control_mode_text, control_mode_sev, flash=False)
        self._set_table_value(5, work_text, work_sev)

    def _run_vision_target_move(self, vision_xyz_mm, robot_xyz_mm, source_panel: int = 1):
        src = 2 if int(source_panel) == 2 else 1
        tag = f"[비전{src}이동]"
        if self.backend is None:
            self.append_log(f"{tag} 백엔드 초기화 중입니다.\n")
            return
        if not hasattr(self.backend, "send_move_vision_point"):
            self.append_log(f"{tag} 백엔드가 비전 좌표 이동 기능을 지원하지 않습니다.\n")
            return

        rx, ry, rz = [float(v) for v in robot_xyz_mm]
        offset_x, offset_y, offset_z = self._get_vision_move_offset_xyz_mm()
        target_x = rx + float(offset_x)
        target_y = ry + float(offset_y)
        target_z = rz + float(offset_z)
        self._last_clicked_robot_target_xyz_mm = (float(target_x), float(target_y), float(target_z))
        ok, msg = self.backend.send_move_vision_point(target_x, target_y, target_z, dwell_sec=1.0)
        if vision_xyz_mm is not None:
            vx, vy, vz = vision_xyz_mm
            self.append_log(
                f"{tag} 비전 XYZ(mm): X={vx:.1f}, Y={vy:.1f}, Z={vz:.1f} -> "
                f"Robot XYZ(mm): X={rx:.2f}, Y={ry:.2f}, Z={rz:.2f} -> "
                f"이동목표 XYZ(mm): X={target_x:.2f}, Y={target_y:.2f}, Z={target_z:.2f} "
                f"[옵셋 X={offset_x:.1f}, Y={offset_y:.1f}, Z={offset_z:.1f}]\n"
            )
        self.append_log(f"{tag} {msg}\n")

    def on_move_to_last_vision_point(self):
        if self._last_clicked_robot_xyz_mm is None or self._last_clicked_vision_xyz_mm is None:
            self.append_log("[비전이동] 마지막 클릭 좌표가 없습니다. 비전 화면을 먼저 클릭하세요.\n")
            return
        self._run_vision_target_move(
            self._last_clicked_vision_xyz_mm,
            self._last_clicked_robot_xyz_mm,
            source_panel=getattr(self, "_last_clicked_source_vision", 1),
        )

    def _detect_robot_mode_text(self):
        if hasattr(self.backend, "get_connection_mode_snapshot"):
            mode_text, _ = self.backend.get_connection_mode_snapshot()
            mode_text = str(mode_text or "").upper().strip()
            if mode_text in ("REAL", "VIRTUAL", "UNKNOWN"):
                self._mode_text_cached = mode_text
                return self._mode_text_cached
        if not self._mode_text_cached:
            self._mode_text_cached = "UNKNOWN"
        return self._mode_text_cached

    def _setup_rosout_bridge(self):
        if RosLogMsg is None:
            self.append_log("[시스템] /rosout 브리지 비활성화: rcl_interfaces.msg.Log import 실패\n")
            return
        node = getattr(self.backend, "robot_controller", None)
        if node is None:
            self.append_log("[시스템] /rosout 브리지 비활성화: robot_controller 노드 없음\n")
            return
        try:
            self._rosout_sub = node.create_subscription(RosLogMsg, "/rosout", self._on_rosout_msg, 200)
            self.append_log("[시스템] /rosout 로그 미러링 시작\n")
        except Exception as e:
            self.append_log(f"[시스템] /rosout 브리지 생성 실패: {e}\n")

    def _on_rosout_msg(self, msg):
        name = str(getattr(msg, "name", ""))
        ros_log_allow = {
            "bartender_backend",
            "bartender_robot_app",      # legacy
            "bartender_robot_bridge",
            "bartender_robot_mode",     # legacy
            "bartender_mode_monitor",
            str(os.environ.get("BARTENDER_BACKEND_NODE_NAME", "") or "").strip(),
            str(os.environ.get("BARTENDER_DSR_BRIDGE_NODE_NAME", "") or "").strip(),
            str(os.environ.get("BARTENDER_MODE_MONITOR_NODE_NAME", "") or "").strip(),
        }
        ros_log_allow.discard("")
        if name not in ros_log_allow:
            return

        level = int(getattr(msg, "level", 0))
        if level >= 50:
            level_text = "FATAL"
        elif level >= 40:
            level_text = "ERROR"
        elif level >= 30:
            level_text = "WARN"
        elif level >= 20:
            level_text = "INFO"
        else:
            level_text = "DEBUG"

        text = str(getattr(msg, "msg", ""))
        self.ros_log_received.emit(f"[ROS:{level_text}] [{name}] {text}\n")

    def _setup_robot_state_table(self):
        self._robot_state_table = getattr(self, "robot_state_table", None)
        if self._robot_state_table is None:
            return

        self._robot_state_table.setRowCount(6)
        self._robot_state_table.setColumnCount(2)
        self._robot_state_table.setHorizontalHeaderLabels(["항목", "값"])
        self._robot_state_table.verticalHeader().setVisible(False)
        self._robot_state_table.horizontalHeader().setMinimumSectionSize(20)
        self._robot_state_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self._robot_state_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self._robot_state_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._robot_state_table.setSelectionMode(QTableWidget.NoSelection)
        self._robot_state_table.setFocusPolicy(Qt.NoFocus)
        self._robot_state_table.setWordWrap(False)
        self._robot_state_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._robot_state_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._robot_state_table.setFont(QFont(UI_FONT_FAMILY, UI_TABLE_FONT_SIZE))
        self._robot_state_table.verticalHeader().setDefaultSectionSize(UI_PANEL_TABLE_ROW_HEIGHT)

        keys = ["연결타입", "통신상태", "로봇상태", "로봇모드", "제어모드", "작업상태"]
        for row, key in enumerate(keys):
            self._robot_state_table.setItem(row, 0, QTableWidgetItem(key))
            self._robot_state_table.setItem(row, 1, QTableWidgetItem("-"))
        self._apply_table_font_scale(self._robot_state_table, 1.0)
        self._fit_table_height(self._robot_state_table)

    def _setup_position_tables(self):
        hide_names = [
            "j1_label", "j2_label", "j3_label", "j4_label", "j5_label", "j6_label",
            "j1_value", "j2_value", "j3_value", "j4_value", "j5_value", "j6_value",
            "x_label", "y_label", "z_label", "a_label", "b_label", "c_label",
            "x_value", "y_value", "z_value", "a_value", "b_value", "c_value",
        ]
        for name in hide_names:
            w = getattr(self, name, None)
            if w is not None:
                w.hide()

        self._joint_table = getattr(self, "joint_table", None)
        self._cart_table = getattr(self, "cart_table", None)
        if self._joint_table is None or self._cart_table is None:
            return

        self._setup_position_table_common(self._joint_table, ["J1", "J2", "J3", "J4", "J5", "J6"])
        self._setup_position_table_common(self._cart_table, ["X", "Y", "Z", "A", "B", "C"])
        self._setup_current_tool_label()

    def _refresh_positions(self):
        if getattr(self, "_joint_table", None) is None or getattr(self, "_cart_table", None) is None:
            return
        if self.backend is None or (not bool(self._top_status_enabled.get("robot", True))):
            self._clear_position_views()
            return
        if hasattr(self.backend, "get_position_snapshot"):
            data, seen_at = self.backend.get_position_snapshot()
        else:
            data = self.backend.get_current_positions() if hasattr(self.backend, "get_current_positions") else None
            seen_at = None
        if not data or len(data) < 2:
            self._clear_position_views()
            return
        now = time.monotonic()
        if seen_at is not None and (now - float(seen_at)) > POSITION_STALE_SEC:
            self._clear_position_views()
            return
        posj, posx = data
        if posj is None or posx is None or len(posj) < 6 or len(posx) < 6:
            self._clear_position_views()
            return

        def _update_table(table, prefix, values):
            for row in range(6):
                item = table.item(row, 1)
                if item is None:
                    item = QTableWidgetItem("-")
                    table.setItem(row, 1, item)
                text_value = self._fmt_ui_float(values[row], 2)
                cache_key = (prefix, row)
                if self._pos_value_cache.get(cache_key) != text_value:
                    self._pos_flash_until[cache_key] = now + POSITION_FLASH_SEC
                self._pos_value_cache[cache_key] = text_value
                item.setText(text_value)
                item.setForeground(QBrush(QColor(POSITION_VALUE_COLOR)))
                font = item.font()
                font.setBold(now < self._pos_flash_until.get(cache_key, 0.0))
                item.setFont(font)

        _update_table(self._joint_table, "joint", [float(v) for v in list(posj)[:6]])
        _update_table(self._cart_table, "cart", [float(v) for v in list(posx)[:6]])
        self._last_positions_seen_at = seen_at if seen_at is not None else now
        if hasattr(self.backend, "get_current_tcp_snapshot"):
            tcp_name, _ = self.backend.get_current_tcp_snapshot()
            if tcp_name is not None:
                self._set_current_tool_text(tcp_name)

    def _setup_current_tool_label(self):
        if self._joint_table is None:
            return
        if self._current_tool_label is not None:
            return

        ui_label = getattr(self, "current_tool_label", None)
        if isinstance(ui_label, QLabel):
            self._current_tool_label = ui_label
            try:
                parent = self._joint_table.parentWidget()
                if parent is not None and self._current_tool_label.parentWidget() is not parent:
                    self._current_tool_label.setParent(parent)
                g_joint = self._joint_table.geometry()
                g_cart = self._cart_table.geometry() if self._cart_table is not None else g_joint
                left = g_joint.x() + 2
                right = max(left + 110, g_cart.x() + g_cart.width() - 2)
                self._current_tool_label.setGeometry(left, max(0, g_joint.y() - 22), right - left, 18)
                self._current_tool_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            except Exception:
                pass
            self._current_tool_label.show()
            self._current_tool_label.raise_()
            return

        label = QLabel("현재툴(TCP): -", self._joint_table.parentWidget())
        label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        inserted = False
        parent = self._joint_table.parentWidget()
        if parent is not None and parent.layout() is not None:
            lay = parent.layout()
            idx = lay.indexOf(self._joint_table)
            if idx >= 0:
                lay.insertWidget(idx, label)
                inserted = True

        if not inserted:
            try:
                g_joint = self._joint_table.geometry()
                g_cart = self._cart_table.geometry() if self._cart_table is not None else g_joint
                h = 18
                y = max(0, g_joint.y() - 22)
                left = g_joint.x() + 2
                right = max(left + 110, g_cart.x() + g_cart.width() - 2)
                label.setGeometry(left, y, right - left, h)
            except Exception:
                pass

        self._current_tool_label = label
        self._current_tool_label.show()
        self._current_tool_label.raise_()

    def _request_current_tool_sync(self, retry_window_sec: float = 8.0, immediate: bool = False):
        now = time.monotonic()
        self._current_tool_sync_retry_deadline = max(
            float(self._current_tool_sync_retry_deadline),
            now + max(0.0, float(retry_window_sec)),
        )
        if immediate:
            self._current_tool_sync_next_try_at = 0.0
            self._tick_current_tool_sync()
            return
        self._current_tool_sync_next_try_at = min(
            float(self._current_tool_sync_next_try_at or now),
            now,
        )

    def _sync_current_tool_in_background(self):
        try:
            backend = self.backend
            if backend is None:
                return
            if not hasattr(backend, "sync_current_tcp_once"):
                return
            backend.sync_current_tcp_once(force=True, timeout_sec=1.0)
        except Exception:
            pass
        finally:
            with self._current_tool_sync_lock:
                self._current_tool_sync_inflight = False

    def _tick_current_tool_sync(self):
        if getattr(self, "_closing", False):
            return
        backend = self.backend
        if backend is None:
            return
        if not hasattr(backend, "get_current_tcp_snapshot"):
            return
        tcp_name, tcp_seen_at = backend.get_current_tcp_snapshot()
        if tcp_seen_at is not None:
            self._set_current_tool_text(tcp_name)
            self._current_tool_sync_retry_deadline = 0.0
            return

        now = time.monotonic()
        if now > float(self._current_tool_sync_retry_deadline):
            return
        if now < float(self._current_tool_sync_next_try_at):
            return

        with self._current_tool_sync_lock:
            if self._current_tool_sync_inflight:
                return
            self._current_tool_sync_inflight = True
        self._current_tool_sync_next_try_at = now + 1.2
        threading.Thread(target=self._sync_current_tool_in_background, daemon=True).start()

    def _set_current_tool_text(self, tcp_name):
        tcp = str(tcp_name or "").strip()
        display = "flange" if tcp == "" else tcp
        text = f"현재툴(TCP): {display}"
        if text == self._current_tool_text_cache:
            return
        self._current_tool_text_cache = text
        if self._current_tool_label is not None:
            self._current_tool_label.setText(text)

    def _refresh_current_tool_label_once(self, log_fail: bool = False):
        if self.backend is None:
            self._set_current_tool_text("-")
            return False
        if not hasattr(self.backend, "sync_current_tcp_once") or not hasattr(self.backend, "get_current_tcp_snapshot"):
            self._set_current_tool_text("-")
            return False
        ok_tcp, msg_tcp = self.backend.sync_current_tcp_once(force=True, timeout_sec=1.0)
        if not ok_tcp:
            self._request_current_tool_sync(retry_window_sec=10.0, immediate=False)
            if log_fail:
                self.append_log(f"[툴표시] 현재 TCP 조회 실패: {msg_tcp}\n")
            return False
        tcp_name, _tcp_seen_at = self.backend.get_current_tcp_snapshot()
        self._set_current_tool_text(tcp_name)
        self._current_tool_sync_retry_deadline = 0.0
        return True

    def _setup_cycle_time_labels(self):
        self._vision_cycle_label = getattr(self, "vision_cycle_label", None)
        self._vision_cycle_label_2 = getattr(self, "vision_cycle_label_2", None)
        self._robot_cycle_label = getattr(self, "robot_cycle_label", None)
        self._voice_order_cycle_label = getattr(self, "_voice_order_cycle_label", None)
        style = f"color: #555; font-size: {max(6, UI_TERMINAL_FONT_SIZE)}pt;"
        if self._vision_cycle_label is not None:
            self._vision_cycle_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self._vision_cycle_label.setStyleSheet(style)
            self._vision_cycle_label.show()
        if self._vision_cycle_label_2 is not None:
            self._vision_cycle_label_2.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self._vision_cycle_label_2.setStyleSheet(style)
            self._vision_cycle_label_2.show()
        if self._robot_cycle_label is not None:
            self._robot_cycle_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self._robot_cycle_label.setStyleSheet(style)
        if self._voice_order_cycle_label is not None:
            self._voice_order_cycle_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self._voice_order_cycle_label.setStyleSheet(style)
            self._voice_order_cycle_label.show()

        serial_style = style
        self._vision_serial_label = getattr(self, "vision_serial_label", None)
        self._vision_serial_label_2 = getattr(self, "vision_serial_label_2", None)
        self._vision_serial_change_button = getattr(self, "vision_serial_change_button", None)
        self._vision_serial_change_button_2 = getattr(self, "vision_serial_change_button_2", None)
        self._vision_rotate_left_button = getattr(self, "vision_rotate_left_button", None)
        self._vision_rotate_zero_button = getattr(self, "vision_rotate_zero_button", None)
        self._vision_rotate_right_button = getattr(self, "vision_rotate_right_button", None)
        self._vision_rotation_label = getattr(self, "vision_rotation_label", None)
        self._vision_rotate_left_button_2 = getattr(self, "vision_rotate_left_button_2", None)
        self._vision_rotate_zero_button_2 = getattr(self, "vision_rotate_zero_button_2", None)
        self._vision_rotate_right_button_2 = getattr(self, "vision_rotate_right_button_2", None)
        self._vision_rotation_label_2 = getattr(self, "vision_rotation_label_2", None)

        if self._vision_serial_label is not None:
            self._vision_serial_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self._vision_serial_label.setStyleSheet(serial_style)
        if self._vision_serial_label_2 is not None:
            self._vision_serial_label_2.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self._vision_serial_label_2.setStyleSheet(serial_style)
        if self._vision_rotation_label is not None:
            self._vision_rotation_label.setStyleSheet(serial_style)
        if self._vision_rotation_label_2 is not None:
            self._vision_rotation_label_2.setStyleSheet(serial_style)
        if self._vision_serial_change_button is not None:
            try:
                self._vision_serial_change_button.clicked.disconnect()
            except Exception:
                pass
            self._vision_serial_change_button.clicked.connect(lambda: self._on_change_vision_camera(1))
        if self._vision_serial_change_button_2 is not None:
            try:
                self._vision_serial_change_button_2.clicked.disconnect()
            except Exception:
                pass
            self._vision_serial_change_button_2.clicked.connect(lambda: self._on_change_vision_camera(2))
        if self._vision_rotate_left_button is not None:
            try:
                self._vision_rotate_left_button.clicked.disconnect()
            except Exception:
                pass
            self._vision_rotate_left_button.clicked.connect(lambda: self._set_vision_rotation(1, -90))
        if self._vision_rotate_right_button is not None:
            try:
                self._vision_rotate_right_button.clicked.disconnect()
            except Exception:
                pass
            self._vision_rotate_right_button.clicked.connect(lambda: self._set_vision_rotation(1, 90))
        if self._vision_rotate_zero_button is not None:
            try:
                self._vision_rotate_zero_button.clicked.disconnect()
            except Exception:
                pass
            self._vision_rotate_zero_button.clicked.connect(lambda: self._set_vision_rotation_absolute(1, 0))
        if self._vision_rotate_left_button_2 is not None:
            try:
                self._vision_rotate_left_button_2.clicked.disconnect()
            except Exception:
                pass
            self._vision_rotate_left_button_2.clicked.connect(lambda: self._set_vision_rotation(2, -90))
        if self._vision_rotate_zero_button_2 is not None:
            try:
                self._vision_rotate_zero_button_2.clicked.disconnect()
            except Exception:
                pass
            self._vision_rotate_zero_button_2.clicked.connect(lambda: self._set_vision_rotation_absolute(2, 0))
        if self._vision_rotate_right_button_2 is not None:
            try:
                self._vision_rotate_right_button_2.clicked.disconnect()
            except Exception:
                pass
            self._vision_rotate_right_button_2.clicked.connect(lambda: self._set_vision_rotation(2, 90))
        self._refresh_vision_rotation_labels()

        self._reposition_cycle_labels()
        self._refresh_vision_serial_labels()

    def _hide_robot_control_title_label(self):
        panel = getattr(self, "frame_2", None)
        if panel is None:
            return
        try:
            for w in panel.findChildren(QLabel):
                txt = (w.text() or "").strip()
                if txt in ("[로봇 조작]", "로봇 조작"):
                    w.hide()
        except Exception:
            return

    def _vision_uvz_to_robot_xyz_mm(self, u: float, v: float, z_mm: float, panel_index: int = 1):
        panel = 2 if int(panel_index) == 2 else 1
        affine = self._vision_to_robot_affine_2 if panel == 2 else self._vision_to_robot_affine_1
        if affine is None:
            affine = self._vision_to_robot_affine
        if affine is None:
            return None
        # 비전 클릭 이동은 반드시 UVZ -> Camera XYZ(mm) 변환이 되어야 한다.
        cam_xyz = self._uvz_to_camera_xyz_mm(u, v, z_mm, require_intrinsics=True, panel_index=panel)
        if cam_xyz is None:
            return None
        cx, cy, cz = cam_xyz
        vec = np.array([float(cx), float(cy), float(cz), 1.0], dtype=np.float64)
        out = vec @ affine
        return float(out[0]), float(out[1]), float(out[2])

    def _apply_vision_to_robot_affine(self, mat, rmse=None, path=None, panel_index: int = 1):
        arr = np.asarray(mat, dtype=np.float64)
        if arr.shape != (4, 3):
            raise ValueError("vision affine must be 4x3")
        panel = 2 if int(panel_index) == 2 else 1
        resolved_path = _resolve_repo_file(path, fallback_dirs=(CALIB_DIR, CALIB_ROTMAT_DIR)) if path else None
        if panel == 2:
            self._vision_to_robot_affine_2 = np.asarray(arr, dtype=np.float64)
            self._vision_to_robot_rmse_2 = None if rmse is None else float(rmse)
            self._calib_matrix_path_2 = resolved_path
        else:
            self._vision_to_robot_affine_1 = np.asarray(arr, dtype=np.float64)
            self._vision_to_robot_rmse_1 = None if rmse is None else float(rmse)
            self._calib_matrix_path_1 = resolved_path
            self._vision_to_robot_affine = np.asarray(arr, dtype=np.float64)
            self._vision_to_robot_rmse = None if rmse is None else float(rmse)
            self._calib_matrix_path = resolved_path
        if self._vision_to_robot_affine is None:
            self._vision_to_robot_affine = np.asarray(arr, dtype=np.float64)
            self._vision_to_robot_rmse = None if rmse is None else float(rmse)
            self._calib_matrix_path = resolved_path
        if resolved_path:
            self._save_active_calibration_path(resolved_path, panel_index=panel)
        self.calibration_ui_refresh_requested.emit()

    def _reposition_cycle_labels(self):
        if hasattr(self, "_vision_cycle_label") and self._vision_cycle_label is not None:
            parent = self._vision_cycle_label.parentWidget()
            w = parent.width() if parent is not None else 511
            right_x = max(220, w - 180)
            self._vision_cycle_label.setGeometry(right_x, 8, 168, 14)
            if self._vision_serial_label is not None:
                self._vision_serial_label.setGeometry(right_x - 52, 24, 220, 14)
            if self._vision_rotate_left_button is not None:
                self._vision_rotate_left_button.raise_()
            if self._vision_rotate_zero_button is not None:
                self._vision_rotate_zero_button.raise_()
            if self._vision_rotate_right_button is not None:
                self._vision_rotate_right_button.raise_()
        if hasattr(self, "_vision_cycle_label_2") and self._vision_cycle_label_2 is not None:
            parent = self._vision_cycle_label_2.parentWidget()
            w = parent.width() if parent is not None else 511
            right_x2 = max(220, w - 180)
            self._vision_cycle_label_2.setGeometry(right_x2, 8, 168, 14)
            if self._vision_serial_label_2 is not None:
                self._vision_serial_label_2.setGeometry(right_x2 - 52, 24, 220, 14)
            if self._vision_rotate_left_button_2 is not None:
                self._vision_rotate_left_button_2.raise_()
            if self._vision_rotate_zero_button_2 is not None:
                self._vision_rotate_zero_button_2.raise_()
            if self._vision_rotate_right_button_2 is not None:
                self._vision_rotate_right_button_2.raise_()
        if hasattr(self, "_robot_cycle_label") and self._robot_cycle_label is not None:
            parent = self._robot_cycle_label.parentWidget()
            w = parent.width() if parent is not None else 531
            self._robot_cycle_label.setGeometry(max(240, w - 190), 8, 178, 14)

    def _maybe_update_cycle_time_labels_from_vision(self):
        now = time.monotonic()
        last_at = float(getattr(self, "_vision_cycle_label_last_update_at", 0.0) or 0.0)
        min_dt = max(0.08, float(VISION_UI_CYCLE_LABEL_INTERVAL_MS) / 1000.0)
        if (now - last_at) < min_dt:
            return
        self._vision_cycle_label_last_update_at = now
        self._update_cycle_time_labels()

    def _update_cycle_time_labels(self):
        if hasattr(self, "_vision_cycle_label") and self._vision_cycle_label is not None:
            if self._vision_cycle_ms is None:
                self._vision_cycle_label.setText("업데이트: - ms")
            else:
                self._vision_cycle_label.setText(f"업데이트: {self._vision_cycle_ms:.1f} ms")
        if hasattr(self, "_vision_cycle_label_2") and self._vision_cycle_label_2 is not None:
            if self._vision_cycle_ms_2 is None:
                self._vision_cycle_label_2.setText("업데이트: - ms")
            else:
                self._vision_cycle_label_2.setText(f"업데이트: {self._vision_cycle_ms_2:.1f} ms")
        if hasattr(self, "_robot_cycle_label") and self._robot_cycle_label is not None:
            if self._robot_cycle_ms is None:
                self._robot_cycle_label.setText("업데이트: - ms")
            else:
                self._robot_cycle_label.setText(f"업데이트: {self._robot_cycle_ms:.1f} ms")
        if hasattr(self, "_voice_order_cycle_label") and self._voice_order_cycle_label is not None:
            if self._voice_order_cycle_ms is None:
                self._voice_order_cycle_label.setText("업데이트: - ms")
            else:
                self._voice_order_cycle_label.setText(f"업데이트: {self._voice_order_cycle_ms:.1f} ms")
        self._refresh_voice_order_webui_badge()
        self._refresh_vision_serial_labels()

    def performance_snapshot(self):
        return {
            "vision1": {
                "frontend_receive_interval_ms": self._vision_cycle_ms,
                "frontend_decode_ms": self._vision_decode_ms,
                "frontend_compose_ms": self._vision_compose_ms,
                "frontend_render_delay_ms": self._vision_render_delay_ms,
                "frontend_render_interval_ms": self._vision_render_interval_ms,
                "calib_input_interval_ms": self._calib_proc_input_ms_1,
                "calib_processing_ms": self._calib_proc_ms_1,
                "calib_publish_interval_ms": self._calib_proc_publish_ms_1,
                "calib_frame_wait_ms": self._calib_proc_wait_ms_1,
                "calib_reason": self._calib_last_reason_1,
            },
            "vision2": {
                "frontend_receive_interval_ms": self._vision_cycle_ms_2,
                "frontend_decode_ms": self._vision_decode_ms_2,
                "frontend_compose_ms": self._vision_compose_ms_2,
                "frontend_render_delay_ms": self._vision_render_delay_ms_2,
                "frontend_render_interval_ms": self._vision_render_interval_ms_2,
                "calib_input_interval_ms": self._calib_proc_input_ms_2,
                "calib_processing_ms": self._calib_proc_ms_2,
                "calib_publish_interval_ms": self._calib_proc_publish_ms_2,
                "calib_frame_wait_ms": self._calib_proc_wait_ms_2,
                "calib_reason": self._calib_last_reason_2,
            },
        }

    def _on_change_vision_camera(self, panel_index: int):
        discovered = self._discover_connected_camera_serials()
        # 카메라 변경 목록은 "현재 검색된" 시리얼만 노출한다.
        self._available_camera_serials = [s for s in discovered if s]
        options = list(self._available_camera_serials)
        if not options:
            QMessageBox.warning(self, "카메라 변경", "검색된 카메라 시리얼이 없습니다.")
            return

        current = self._vision_assigned_serial_1 if int(panel_index) == 1 else self._vision_assigned_serial_2
        try:
            idx = options.index(current)
        except Exception:
            idx = 0
        selected, ok = QInputDialog.getItem(
            self,
            "카메라 변경",
            f"비전{int(panel_index)}에 할당할 카메라 시리얼을 선택하세요.",
            options,
            idx,
            False,
        )
        if not ok:
            return
        picked = self._normalize_serial_text(selected)
        if not picked:
            return

        if int(panel_index) == 1:
            prev = self._vision_assigned_serial_1
            self._vision_assigned_serial_1 = picked
            if picked == self._vision_assigned_serial_2:
                self._vision_assigned_serial_2 = prev
        else:
            prev = self._vision_assigned_serial_2
            self._vision_assigned_serial_2 = picked
            if picked == self._vision_assigned_serial_1:
                self._vision_assigned_serial_1 = prev

        self._save_vision_serial_settings()
        self._refresh_vision_serial_labels()
        self._clear_calibration_panel_state(1)
        self._clear_calibration_panel_state(2)
        self._clear_vision_view_data(1)
        self._clear_vision_view_data(2)
        self._advance_vision_stream_token(1)
        self._advance_vision_stream_token(2)
        now = time.monotonic()
        self._vision_mode_switch_grace_until_1 = now + MODE_SWITCH_GRACE_SEC
        self._vision_mode_switch_grace_until_2 = now + MODE_SWITCH_GRACE_SEC
        self._vision_drop_frames_until_1 = now + 1.2
        self._vision_drop_frames_until_2 = now + 1.2
        self._vision_state_text = "전환중"
        self._vision_state_text_2 = "전환중"
        self._sync_sensor_launch_from_vision_toggles(reason="카메라 시리얼 변경")
        self._sync_calibration_processes()
        self._rebind_external_vision_bridge_for_mode()
        self.append_log(
            f"[{self._vision_log_tag()}] 카메라 할당 변경: 비전1={self._vision_assigned_serial_1}, 비전2={self._vision_assigned_serial_2} (런타임 camera={self._runtime_camera_serial_1}, camera2={self._runtime_camera_serial_2})\n"
        )
        self.append_log(
            f"[{self._vision_log_tag()}] 카메라 시리얼 변경 저장 완료: 센서 런타임을 재시작해 즉시 반영했습니다.\n"
        )
        QMessageBox.information(
            self,
            "카메라 변경",
            "시리얼 변경을 저장했습니다.\n센서 런타임을 재시작하여 현재 설정을 즉시 반영했습니다.",
        )

    def _setup_position_table_common(self, table: QTableWidget, keys):
        table.setRowCount(len(keys))
        table.setColumnCount(2)
        table.setHorizontalHeaderLabels(["항목", "값"])
        table.verticalHeader().setVisible(False)
        table.horizontalHeader().setMinimumSectionSize(20)
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.setSelectionMode(QTableWidget.NoSelection)
        table.setFocusPolicy(Qt.NoFocus)
        table.setWordWrap(False)
        table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        table.setFont(QFont(UI_FONT_FAMILY, UI_TABLE_FONT_SIZE))
        table.verticalHeader().setDefaultSectionSize(UI_PANEL_TABLE_ROW_HEIGHT)
        for row, key in enumerate(keys):
            table.setItem(row, 0, QTableWidgetItem(key))
            table.setItem(row, 1, QTableWidgetItem("-"))
        self._apply_table_font_scale(table, 1.0)
        self._fit_table_height(table)

    def _fit_table_height(self, table: QTableWidget):
        if table is None:
            return
        try:
            total_h = table.horizontalHeader().height()
            for row in range(table.rowCount()):
                total_h += table.rowHeight(row)
            total_h += table.frameWidth() * 2 + 2
            table.setMinimumHeight(total_h)
            table.setMaximumHeight(total_h)
        except Exception:
            pass

    def _apply_table_font_scale(self, table: QTableWidget, scale: float):
        if table is None:
            return
        try:
            scale = float(scale)
        except Exception:
            scale = 1.0
        scale = max(0.3, min(1.0, scale))

        base_font = table.font()
        base_pt = base_font.pointSizeF() if base_font.pointSizeF() > 0 else float(UI_FONT_SIZE)
        new_pt = max(6.0, base_pt * scale)

        f = QFont(base_font)
        f.setPointSizeF(new_pt)
        table.setFont(f)

        hh = table.horizontalHeader()
        if hh is not None:
            hf = QFont(hh.font())
            hpt = hf.pointSizeF() if hf.pointSizeF() > 0 else base_pt
            hf.setPointSizeF(max(6.0, hpt * scale))
            hh.setFont(hf)

        for r in range(table.rowCount()):
            for c in range(table.columnCount()):
                item = table.item(r, c)
                if item is None:
                    continue
                if c == 1:
                    # 값 컬럼은 상태 강조(bold) 업데이트 로직이 있으므로 크기만 고정.
                    if item.font().pointSizeF() <= 0:
                        item.setFont(QFont(f))
                    else:
                        itf = QFont(item.font())
                        itf.setPointSizeF(new_pt)
                        item.setFont(itf)
                else:
                    item.setFont(QFont(f))

    def _setup_control_buttons(self):
        def _pick_button(panel, names, text_keywords):
            for n in names:
                w = getattr(self, n, None)
                if isinstance(w, QPushButton):
                    return w
            if panel is not None:
                for w in panel.findChildren(QPushButton):
                    txt = str(w.text())
                    if any(k in txt for k in text_keywords):
                        return w
            return None

        def _pick_line_edit(panel, names, hint_keywords):
            for n in names:
                w = getattr(self, n, None)
                if isinstance(w, QLineEdit):
                    return w
            if panel is not None:
                for w in panel.findChildren(QLineEdit):
                    blob = f"{w.placeholderText()} {w.toolTip()} {w.objectName()}".lower()
                    if any(k in blob for k in hint_keywords):
                        return w
            return None

        def _pick_label(panel, names, text_keywords):
            for n in names:
                w = getattr(self, n, None)
                if isinstance(w, QLabel):
                    return w
            if panel is not None:
                for w in panel.findChildren(QLabel):
                    txt = str(w.text())
                    if any(k in txt for k in text_keywords):
                        return w
            return None

        def _pick_checkbox(panel, names, text_keywords):
            for n in names:
                w = getattr(self, n, None)
                if isinstance(w, QCheckBox):
                    return w
            if panel is not None:
                for w in panel.findChildren(QCheckBox):
                    txt = str(w.text())
                    if any(k in txt for k in text_keywords):
                        return w
            return None

        panel = getattr(self, "frame_2", None)
        self.pushButton = _pick_button(panel, ["pushButton", "start_button", "work_start_button"], ["작업 시작", "시작"])
        start_btn = self.pushButton
        self._print_pos_button = _pick_button(
            panel,
            ["print_pos_button", "print_button", "position_print_button"],
            ["좌표 출력", "좌표", "출력"],
        )
        self._reset_button = _pick_button(panel, ["reset_button", "robot_reset_button"], ["리셋", "초기화"])
        self._emergency_stop_button = _pick_button(
            panel,
            ["emergency_stop_button", "robot_emergency_stop_button"],
            ["정지", "긴급정지", "긴급 정지", "비상정지", "비상 정지"],
        )
        self._home_button = _pick_button(panel, ["home_button", "move_home_button"], ["홈위치", "홈", "원점"])
        self._home_save_button = _pick_button(
            panel,
            ["home_button_2", "home_save_button", "save_home_button"],
            ["홈위치 저장", "홈 저장"],
        )
        self._robot_mode_button = _pick_button(
            panel,
            ["print_pos_button_2", "robot_mode_button", "mode_change_button"],
            ["모드변경", "모드 변경", "로봇모드"],
        )
        self._tool_change_button = _pick_button(
            panel,
            ["tool_change_button", "tcp_change_button"],
            ["툴 변경", "tcp 변경", "tool"],
        )

        start_btn = self.pushButton
        if start_btn is not None:
            try:
                start_btn.clicked.disconnect()
            except Exception:
                pass
            start_btn.clicked.connect(self.on_move_xyzabc_dialog)
        self._motion_group_box = getattr(self, "motion_group_box", None)
        self._motion_group_title = _pick_label(panel, ["motion_group_title"], ["기본 동작", "기본동작"])
        self._vision_group_box = getattr(self, "vision_group_box", None)
        self._vision_group_title = _pick_label(panel, ["vision_group_title"], ["비전 이동", "비전이동"])
        self._gripper_manual_box = getattr(self, "gripper_manual_box", None)
        self._gripper_range_title_label = _pick_label(
            panel,
            ["gripper_range_title_label"],
            ["그리퍼 사이 거리 입력", "그리퍼", "집게"],
        )
        self._gripper_range_value_label = _pick_label(
            panel,
            ["gripper_range_value_label"],
            ["허용 범위", "0.00 ~ 109.00"],
        )
        self._gripper_stroke_input = _pick_line_edit(
            panel,
            ["gripper_stroke_input", "gripper_input"],
            ["gripper", "그리퍼", "집게", "109.00"],
        )
        self._gripper_move_button = _pick_button(
            panel,
            ["gripper_move_button", "gripper_button"],
            ["그리퍼 이동", "집게 이동", "그리퍼"],
        )
        self._vision_move_button = _pick_button(
            panel,
            ["vision_move_button", "vision_coord_move_button"],
            ["비전 좌표 이동", "비전 이동", "좌표 이동"],
        )
        self._vision_x_axis_label = _pick_label(panel, ["vision_x_axis_label"], [])
        self._vision_y_axis_label = _pick_label(panel, ["vision_y_axis_label"], [])
        self._vision_z_axis_label = _pick_label(panel, ["vision_z_axis_label"], [])
        self._vision_x_margin_input = _pick_line_edit(
            panel,
            ["vision_x_margin_input", "vision_x_offset_input"],
            [],
        )
        self._vision_y_margin_input = _pick_line_edit(
            panel,
            ["vision_y_margin_input", "vision_y_offset_input"],
            [],
        )
        self._vision_z_margin_input = _pick_line_edit(
            panel,
            ["vision_z_margin_input", "z_margin_input", "vision_z_offset_input"],
            ["z", "옵셋", "offset", "vision"],
        )
        self._vision_z_title_label = _pick_label(
            panel,
            ["vision_z_title_label"],
            ["Z 옵셋 입력", "Z+ 옵셋", "옵셋"],
        )
        self._vision_z_range_label = _pick_label(
            panel,
            ["vision_z_range_label"],
            ["-300.0 ~ 300.0", "허용 범위"],
        )
        self._vision_dialog_toggle_button = _pick_button(
            panel,
            ["vision_dialog_toggle_button", "vision_click_toggle_button"],
            ["비전클릭이동", "비전좌표 클릭"],
        )
        self._vision_dialog_toggle_switch = _pick_checkbox(
            panel,
            ["vision_dialog_toggle_switch", "vision_click_dialog_switch"],
            ["비전클릭이동", "비전좌표 클릭"],
        )
        self._calibration_mode_switch = _pick_checkbox(
            panel,
            ["calibration_mode_switch", "vision_calibration_mode_switch"],
            ["캘리브레이션 모드", "calibration mode"],
        )
        self._calibration_mode_switch_2 = getattr(self, "calibration_mode_switch_2", None)
        self._calibration_transform_button = _pick_button(
            panel,
            ["calibration_transform_button", "vision_robot_transform_button"],
            ["비전/로봇 좌표변환", "좌표변환", "변환행렬"],
        )
        self._calibration_transform_button_2 = getattr(self, "calibration_transform_button_2", None)
        self._calibration_load_button = _pick_button(
            panel,
            ["calibration_load_button", "load_rotation_matrix_button"],
            ["회전행렬 불러오기", "행렬 불러오기", "불러오기"],
        )
        self._calibration_load_button_2 = getattr(self, "calibration_load_button_2", None)
        self._calibration_status_label = _pick_label(
            panel,
            ["calibration_status_label"],
            ["캘리브레이션", "calibration"],
        )
        self._calibration_status_label_2 = getattr(self, "calibration_status_label_2", None)
        self._calibration_matrix_file_label = _pick_label(
            panel,
            ["calibration_matrix_file_label"],
            ["행렬:", "matrix"],
        )
        self._calibration_matrix_file_label_2 = getattr(self, "calibration_matrix_file_label_2", None)

        def _ensure_runtime_info_widgets(box, panel_index: int):
            if box is None:
                return
            badge_name = "_vision_mode_badge_2" if int(panel_index) == 2 else "_vision_mode_badge_1"
            rate_name = "_vision_meta_rate_label_2" if int(panel_index) == 2 else "_vision_meta_rate_label_1"
            detail_name = "_vision_runtime_detail_label_2" if int(panel_index) == 2 else "_vision_runtime_detail_label_1"
            list_name = "_vision_runtime_list_label_2" if int(panel_index) == 2 else "_vision_runtime_list_label_1"
            badge = getattr(self, badge_name, None)
            if badge is None:
                badge = QLabel(box)
                badge.setObjectName(f"vision_mode_badge_{int(panel_index)}")
                badge.setGeometry(318, 12, 158, 20)
                badge.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                setattr(self, badge_name, badge)
            rate = getattr(self, rate_name, None)
            if rate is None:
                rate = QLabel(box)
                rate.setObjectName(f"vision_meta_rate_label_{int(panel_index)}")
                rate.setGeometry(172, 14, 54, 16)
                rate.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                setattr(self, rate_name, rate)
            detail = getattr(self, detail_name, None)
            if detail is None:
                detail = QLabel(box)
                detail.setObjectName(f"vision_runtime_detail_label_{int(panel_index)}")
                detail.setGeometry(14, 56, 462, 22)
                detail.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                detail.setWordWrap(False)
                setattr(self, detail_name, detail)
            data = getattr(self, list_name, None)
            if data is None:
                data = QLabel(box)
                data.setObjectName(f"vision_runtime_list_label_{int(panel_index)}")
                data.setGeometry(14, 84, 462, 34)
                data.setAlignment(Qt.AlignLeft | Qt.AlignTop)
                data.setWordWrap(True)
                setattr(self, list_name, data)

        _ensure_runtime_info_widgets(getattr(self, "calibration_group_box", None), 1)
        _ensure_runtime_info_widgets(getattr(self, "calibration_group_box_2", None), 2)

        if self._print_pos_button is not None:
            try:
                self._print_pos_button.clicked.disconnect()
            except Exception:
                pass
            self._print_pos_button.clicked.connect(self.on_move_joint_dialog)
        if self._reset_button is not None:
            self._reset_button.clicked.connect(self.on_reset_robot)
        if self._emergency_stop_button is not None:
            try:
                self._emergency_stop_button.clicked.disconnect()
            except Exception:
                pass
            self._emergency_stop_button.clicked.connect(self.on_stop_motion)
        if self._home_button is not None:
            self._home_button.clicked.connect(self.on_move_home)
        if self._home_save_button is not None:
            try:
                self._home_save_button.clicked.disconnect()
            except Exception:
                pass
            self._home_save_button.clicked.connect(self.on_save_home_position_dialog)
        if self._robot_mode_button is not None:
            try:
                self._robot_mode_button.clicked.disconnect()
            except Exception:
                pass
            self._robot_mode_button.clicked.connect(self.on_change_robot_mode_dialog)
        if self._tool_change_button is not None:
            try:
                self._tool_change_button.clicked.disconnect()
            except Exception:
                pass
            self._tool_change_button.clicked.connect(self.on_change_tool_dialog)
        if self._motion_group_box is not None:
            if self._motion_speed_title_label is None:
                self._motion_speed_title_label = QLabel(self._motion_group_box)
                self._motion_speed_title_label.setObjectName("motion_speed_title_label")
                self._motion_speed_title_label.setStyleSheet("color: #1f3b63; font-size: 8.5pt; font-weight: 600;")
                self._motion_speed_title_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self._motion_speed_title_label.setGeometry(220, 8, 116, 18)
            if self._motion_speed_slider is None:
                self._motion_speed_slider = QSlider(Qt.Horizontal, self._motion_group_box)
                self._motion_speed_slider.setObjectName("motion_speed_slider")
                self._motion_speed_slider.setGeometry(342, 8, 116, 18)
                self._motion_speed_slider.setRange(MOTION_SPEED_MIN_PERCENT, MOTION_SPEED_MAX_PERCENT)
                self._motion_speed_slider.setSingleStep(1)
                self._motion_speed_slider.setPageStep(10)
                self._motion_speed_slider.setTickInterval(10)
                self._motion_speed_slider.setTickPosition(QSlider.NoTicks)
                self._motion_speed_slider.valueChanged.connect(self._on_motion_speed_changed)
            if self._motion_speed_value_label is None:
                self._motion_speed_value_label = QLabel(self._motion_group_box)
                self._motion_speed_value_label.setObjectName("motion_speed_value_label")
                self._motion_speed_value_label.setStyleSheet("color: #1f3b63; font-size: 9pt; font-weight: 700;")
                self._motion_speed_value_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                self._motion_speed_value_label.setGeometry(464, 8, 37, 18)
            self._motion_speed_slider.blockSignals(True)
            self._motion_speed_slider.setValue(self._clamp_motion_speed_percent(self._motion_speed_percent))
            self._motion_speed_slider.blockSignals(False)
            self._sync_motion_speed_widgets()
        if self._gripper_stroke_input is not None:
            validator = QDoubleValidator(0.0, 109.0, 2, self._gripper_stroke_input)
            validator.setNotation(QDoubleValidator.StandardNotation)
            self._gripper_stroke_input.setValidator(validator)
        if self._gripper_move_button is not None:
            self._gripper_move_button.clicked.connect(self.on_gripper_move)

        if self._vision_move_button is not None:
            self._vision_move_button.clicked.connect(self.on_move_to_last_vision_point)
        for edit, default_v in (
            (self._vision_x_margin_input, VISION_MOVE_DEFAULT_OFFSET_X_MM),
            (self._vision_y_margin_input, VISION_MOVE_DEFAULT_OFFSET_Y_MM),
            (self._vision_z_margin_input, VISION_MOVE_DEFAULT_OFFSET_Z_MM),
        ):
            if edit is None:
                continue
            edit.setText(f"{float(default_v):.1f}")
            validator = QDoubleValidator(-1000.0, 3000.0, 1, edit)
            validator.setNotation(QDoubleValidator.StandardNotation)
            edit.setValidator(validator)
        if self._vision_dialog_toggle_button is not None:
            self._vision_dialog_toggle_button.hide()
        if self._vision_dialog_toggle_switch is not None:
            self._vision_dialog_toggle_switch.setChecked(bool(getattr(self, "_vision_click_dialog_enabled", True)))
            self._vision_dialog_toggle_switch.toggled.connect(self._on_vision_click_dialog_toggled)
        if self._calibration_mode_switch is not None:
            self._calibration_mode_switch.setChecked(bool(self._calibration_mode_enabled_1))
            self._calibration_mode_switch.toggled.connect(self._on_calibration_mode_toggled)
        if self._calibration_mode_switch_2 is not None:
            self._calibration_mode_switch_2.setChecked(bool(self._calibration_mode_enabled_2))
            self._calibration_mode_switch_2.toggled.connect(self._on_calibration_mode_toggled)
        if self._calibration_transform_button is not None:
            try:
                self._calibration_transform_button.clicked.disconnect()
            except Exception:
                pass
            self._calibration_transform_button.clicked.connect(self.on_calibration_transform)
        if self._calibration_transform_button_2 is not None:
            try:
                self._calibration_transform_button_2.clicked.disconnect()
            except Exception:
                pass
            self._calibration_transform_button_2.clicked.connect(self.on_calibration_transform)
        if self._calibration_load_button is not None:
            self._calibration_load_button.clicked.connect(self.on_calibration_load_matrix)
        if self._calibration_load_button_2 is not None:
            self._calibration_load_button_2.clicked.connect(self.on_calibration_load_matrix)
        if self._motion_group_box is not None:
            self._motion_group_box.lower()
        if self._gripper_manual_box is not None:
            self._gripper_manual_box.lower()
        if self._vision_group_box is not None:
            self._vision_group_box.lower()
        for w in [
            self._motion_group_title,
            self._vision_group_title,
            self._home_button,
            self._home_save_button,
            self._robot_mode_button,
            self._tool_change_button,
            self._motion_speed_title_label,
            self._motion_speed_slider,
            self._motion_speed_value_label,
            self.pushButton,
            self._print_pos_button,
            self._reset_button,
            self._emergency_stop_button,
            self._gripper_range_title_label,
            self._gripper_range_value_label,
            self._gripper_stroke_input,
            self._gripper_move_button,
            *self._iter_vision_move_widgets(),
            self._calibration_mode_switch,
            self._calibration_mode_switch_2,
            self._calibration_transform_button,
            self._calibration_transform_button_2,
            self._calibration_load_button,
            self._calibration_load_button_2,
            self._calibration_matrix_file_label,
            self._calibration_matrix_file_label_2,
            self._calibration_status_label,
            self._calibration_status_label_2,
        ]:
            if w is not None:
                w.raise_()
        self._update_calibration_mode_ui()
        self._layout_control_buttons()

    def _setup_gripper_manual_controls(self):
        # UI widgets are now defined in developer_frontend.ui
        return

    def _layout_control_buttons(self):
        # Geometry is controlled by developer_frontend.ui; enforce only z-order here.
        if self._calibration_mode_switch is not None and self._calibration_mode_switch_2 is not None:
            try:
                g1 = self._calibration_mode_switch.geometry()
                g2 = self._calibration_mode_switch_2.geometry()
                # Keep switch_2 on the same row as switch_1.
                self._calibration_mode_switch_2.setGeometry(g2.x(), g1.y(), g2.width(), g1.height())
            except Exception:
                pass
        if self._motion_group_box is not None:
            self._motion_group_box.lower()
        if self._gripper_manual_box is not None:
            self._gripper_manual_box.lower()
        if self._vision_group_box is not None:
            self._vision_group_box.lower()
        for w in [
            self._motion_group_title,
            self._vision_group_title,
            self._home_button,
            self._home_save_button,
            self.pushButton,
            self._print_pos_button,
            self._reset_button,
            self._emergency_stop_button,
            self._tool_change_button,
            self._motion_speed_title_label,
            self._motion_speed_slider,
            self._motion_speed_value_label,
            self._gripper_range_title_label,
            self._gripper_range_value_label,
            self._gripper_stroke_input,
            self._gripper_move_button,
            *self._iter_vision_move_widgets(),
            self._vision_dialog_toggle_button,
            self._calibration_mode_switch,
            self._calibration_mode_switch_2,
            self._calibration_transform_button,
            self._calibration_transform_button_2,
            self._calibration_load_button,
            self._calibration_load_button_2,
            self._calibration_matrix_file_label,
            self._calibration_matrix_file_label_2,
            self._calibration_status_label,
            self._calibration_status_label_2,
        ]:
            if w is not None:
                w.raise_()

    def _clamp_motion_speed_percent(self, value):
        try:
            speed = int(round(float(value)))
        except Exception:
            speed = int(DEFAULT_MOTION_SPEED_PERCENT)
        return max(MOTION_SPEED_MIN_PERCENT, min(MOTION_SPEED_MAX_PERCENT, speed))

    def _sync_motion_speed_widgets(self):
        speed = self._clamp_motion_speed_percent(self._motion_speed_percent)
        self._motion_speed_percent = speed
        if self._motion_speed_title_label is not None:
            self._motion_speed_title_label.setText("이동속도 (0~100%)")
        if self._motion_speed_slider is not None:
            self._motion_speed_slider.blockSignals(True)
            self._motion_speed_slider.setValue(int(speed))
            self._motion_speed_slider.blockSignals(False)
        if self._motion_speed_value_label is not None:
            self._motion_speed_value_label.setText(f"{speed}%")
        if self._bartender_speed_title_label is not None:
            self._bartender_speed_title_label.setText("로봇 시퀀스 속도 (0~100%)")
        if self._bartender_speed_slider is not None:
            self._bartender_speed_slider.blockSignals(True)
            self._bartender_speed_slider.setValue(int(speed))
            self._bartender_speed_slider.blockSignals(False)
        if self._bartender_speed_value_label is not None:
            self._bartender_speed_value_label.setText(f"{speed}%")

    def _on_motion_speed_changed(self, value):
        self._motion_speed_percent = self._clamp_motion_speed_percent(value)
        self._sync_motion_speed_widgets()

    def _on_bartender_motion_speed_changed(self, value):
        self._motion_speed_percent = self._clamp_motion_speed_percent(value)
        self._sync_motion_speed_widgets()

    def _motion_speed_for_command(self):
        speed = self._clamp_motion_speed_percent(
            self._motion_speed_slider.value()
            if self._motion_speed_slider is not None
            else (
                self._bartender_speed_slider.value()
                if self._bartender_speed_slider is not None
                else self._motion_speed_percent
            )
        )
        self._motion_speed_percent = speed
        if speed <= 0:
            self.append_log("[속도] 0%는 실행할 수 없어 1%로 적용합니다.\n")
            return 1.0
        return float(speed)

    def _calibration_motion_speed_for_command(self):
        return float(CALIBRATION_FIXED_SPEED_PERCENT)

    def _wait_with_ui_pump(self, delay_sec: float, status_callback=None, message_prefix: str = ""):
        remain = max(0.0, float(delay_sec))
        if remain <= 0.0:
            return
        end_at = time.monotonic() + remain
        while True:
            left = end_at - time.monotonic()
            if left <= 0.0:
                break
            if status_callback is not None and message_prefix:
                try:
                    status_callback(f"{message_prefix} ({left:.1f}초)")
                except Exception:
                    pass
            QApplication.processEvents()
            time.sleep(min(0.1, max(0.02, left)))

    def _vision_log_tag(self, panel_index=None):
        if panel_index is None:
            return "비전"
        return "비전2" if int(panel_index) == 2 else "비전1"

    def _iter_vision_move_widgets(self):
        return [
            getattr(self, "_vision_dialog_toggle_switch", None),
            getattr(self, "_vision_z_title_label", None),
            getattr(self, "_vision_z_range_label", None),
            getattr(self, "_vision_x_axis_label", None),
            getattr(self, "_vision_y_axis_label", None),
            getattr(self, "_vision_z_axis_label", None),
            getattr(self, "_vision_x_margin_input", None),
            getattr(self, "_vision_y_margin_input", None),
            getattr(self, "_vision_z_margin_input", None),
            getattr(self, "_vision_move_button", None),
        ]

    def _append_vision_log(self, message: str, panel_index=None):
        self.append_log(f"[{self._vision_log_tag(panel_index)}] {message}\n")

    def _read_vision_offset_input_mm(self, widget, default_v):
        if widget is None:
            return float(default_v)
        raw = widget.text().strip()
        if not raw:
            value = float(default_v)
        else:
            try:
                value = float(raw)
            except Exception:
                value = float(default_v)
        widget.setText(f"{value:.1f}")
        return float(value)

    def _get_vision_move_offset_xyz_mm(self):
        offsets = (
            self._read_vision_offset_input_mm(getattr(self, "_vision_x_margin_input", None), VISION_MOVE_DEFAULT_OFFSET_X_MM),
            self._read_vision_offset_input_mm(getattr(self, "_vision_y_margin_input", None), VISION_MOVE_DEFAULT_OFFSET_Y_MM),
            self._read_vision_offset_input_mm(getattr(self, "_vision_z_margin_input", None), VISION_MOVE_DEFAULT_OFFSET_Z_MM),
        )
        self._vision_move_offset_xyz_mm = offsets
        return self._vision_move_offset_xyz_mm

    def _get_vision_move_z_margin_mm(self):
        return float(self._get_vision_move_offset_xyz_mm()[2])

    def _toggle_vision_click_dialog(self):
        self._vision_click_dialog_enabled = not bool(self._vision_click_dialog_enabled)
        if hasattr(self, "_vision_dialog_toggle_button") and self._vision_dialog_toggle_button is not None:
            self._vision_dialog_toggle_button.setText(
                "비전클릭이동 ON" if self._vision_click_dialog_enabled else "비전클릭이동 OFF"
            )
        if hasattr(self, "_vision_dialog_toggle_switch") and self._vision_dialog_toggle_switch is not None:
            self._vision_dialog_toggle_switch.blockSignals(True)
            self._vision_dialog_toggle_switch.setChecked(bool(self._vision_click_dialog_enabled))
            self._vision_dialog_toggle_switch.blockSignals(False)
        self._append_vision_log(f"클릭 이동 다이얼로그: {'ON' if self._vision_click_dialog_enabled else 'OFF'}")
        if self.backend is not None:
            self._rebind_external_vision_bridge_for_mode()

    def _on_vision_click_dialog_toggled(self, checked):
        self._vision_click_dialog_enabled = bool(checked)
        self._append_vision_log(f"클릭 이동 다이얼로그: {'ON' if self._vision_click_dialog_enabled else 'OFF'}")
        if self.backend is not None:
            self._rebind_external_vision_bridge_for_mode()

    def on_change_robot_mode_dialog(self):
        if self.backend is None:
            self.append_log("[로봇모드] 백엔드 초기화 중입니다.\n")
            return
        if not hasattr(self.backend, "sync_robot_mode_once") or not hasattr(self.backend, "set_robot_mode"):
            self.append_log("[로봇모드] 백엔드가 모드 변경 기능을 지원하지 않습니다.\n")
            return

        ok_mode, mode_msg = self.backend.sync_robot_mode_once(force=False, timeout_sec=1.0)
        if not ok_mode:
            self.append_log(f"[로봇모드] 현재 모드 조회 실패: {mode_msg}\n")
        mode_value, _seen_at = self.backend.get_robot_mode_snapshot()
        cur_value = -1 if mode_value is None else int(mode_value)
        cur_text = {
            0: "메뉴얼모드",
            1: "오토모드",
            2: "측정모드",
        }.get(cur_value, "알수없음")

        dialog = QDialog(self)
        dialog.setWindowTitle("로봇 모드 변경")
        dialog.setModal(True)
        dialog.resize(500, 280)

        layout = QVBoxLayout(dialog)
        layout.setContentsMargins(20, 18, 20, 16)
        layout.setSpacing(14)
        info = QLabel(f"현재 모드: {cur_text} ({cur_value})\n변경할 모드를 선택하세요.", dialog)
        layout.addWidget(info)
        layout.addSpacing(8)

        mode_row = QHBoxLayout()
        mode_row.setSpacing(16)
        mode_row.addStretch(1)
        btn_manual = QPushButton("메뉴얼모드", dialog)
        btn_auto = QPushButton("오토모드", dialog)
        for b in (btn_manual, btn_auto):
            b.setCheckable(True)
            b.setMinimumSize(150, 58)
            b.setStyleSheet(
                "QPushButton { border: 1px solid #9aa0a6; border-radius: 6px; padding: 8px 12px; }"
                "QPushButton:checked { background: #1f6feb; color: #ffffff; border: 1px solid #1f6feb; }"
            )
        group = QButtonGroup(dialog)
        group.setExclusive(True)
        group.addButton(btn_manual, 0)
        group.addButton(btn_auto, 1)
        if cur_value == 1:
            btn_auto.setChecked(True)
        else:
            btn_manual.setChecked(True)
        mode_row.addWidget(btn_manual)
        mode_row.addWidget(btn_auto)
        mode_row.addStretch(1)
        layout.addStretch(1)
        layout.addLayout(mode_row)
        layout.addStretch(1)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, parent=dialog)
        buttons.setCenterButtons(False)

        def _on_accept():
            if group.checkedId() not in (0, 1):
                QMessageBox.warning(dialog, "로봇 모드 변경", "메뉴얼모드 또는 오토모드를 선택하세요.")
                return
            dialog.accept()

        buttons.accepted.connect(_on_accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)

        if dialog.exec_() != QDialog.Accepted:
            self.append_log("[로봇모드] 변경 취소\n")
            return

        target = int(group.checkedId())
        target_name = "메뉴얼모드" if target == 0 else "오토모드"

        def _format_mode_result_details(raw_msg: str) -> str:
            text = str(raw_msg or "").strip()
            parts = [p.strip() for p in text.split(";") if p.strip()]
            set_part = "-"
            get_part = "-"
            extra_parts = []

            def _simplify_status(value: str) -> str:
                v = str(value or "").strip()
                up = v.upper()
                if up.startswith("OK"):
                    return "OK"
                if up.startswith("FAIL"):
                    if "(" in v and ")" in v:
                        try:
                            reason = v[v.find("(") + 1 : v.rfind(")")].strip()
                            if reason:
                                return reason
                        except Exception:
                            pass
                    reason = v[4:].strip()
                    return reason if reason else "실패"
                return v if v else "-"

            for p in parts:
                up = p.upper()
                if up.startswith("SET="):
                    set_part = _simplify_status(p.split("=", 1)[1].strip())
                elif up.startswith("GET="):
                    get_part = _simplify_status(p.split("=", 1)[1].strip())
                else:
                    extra_parts.append(p)
            lines = [f"- SET: {set_part}", f"- GET: {get_part}"]
            if extra_parts:
                lines.append(f"- 기타: {'; '.join(extra_parts)}")
            return "\n".join(lines)

        wait = QDialog(self)
        wait.setWindowTitle("로봇 모드 변경")
        wait.setWindowModality(Qt.WindowModal)
        wait.setModal(True)
        wait_layout = QVBoxLayout(wait)
        wait_layout.addWidget(QLabel("모드 변경 적용중입니다...\n모드 변경 확인중입니다...", wait))
        wait.setFixedSize(260, 90)
        wait.show()
        QApplication.processEvents()
        try:
            ok, msg = self.backend.set_robot_mode(target, timeout_sec=8.0)
        finally:
            wait.done(0)
            wait.hide()
            wait.deleteLater()
            QApplication.processEvents()
        if ok:
            self._request_current_tool_sync(retry_window_sec=12.0, immediate=True)
            self.append_log(f"[로봇모드] {target_name}({target})로 변경을 성공하였습니다. ({msg})\n")
            QMessageBox.information(
                self,
                "로봇 모드 변경",
                (
                    f"{target_name}({target})로 변경을 성공하였습니다.\n\n"
                    f"상세\n{_format_mode_result_details(msg)}"
                ),
                QMessageBox.Ok,
            )
        else:
            self.append_log(f"[로봇모드] {target_name}({target})로 변경을 실패하였습니다. ({msg})\n")
            QMessageBox.warning(
                self,
                "로봇 모드 변경",
                (
                    f"{target_name}({target})로 변경을 실패하였습니다.\n\n"
                    f"상세\n{_format_mode_result_details(msg)}"
                ),
                QMessageBox.Ok,
            )

    def on_change_tool_dialog(self):
        if self.backend is None:
            self.append_log("[툴변경] 백엔드 초기화 중입니다.\n")
            return
        if not hasattr(self.backend, "sync_robot_mode_once") or not hasattr(self.backend, "get_robot_mode_snapshot"):
            self.append_log("[툴변경] 백엔드가 로봇모드 조회 기능을 지원하지 않습니다.\n")
            return
        if not hasattr(self.backend, "sync_current_tcp_once") or not hasattr(self.backend, "set_current_tcp_name"):
            self.append_log("[툴변경] 백엔드가 TCP 변경 기능을 지원하지 않습니다.\n")
            return

        ok_mode, mode_msg = self.backend.sync_robot_mode_once(force=False, timeout_sec=1.0)
        if not ok_mode:
            self.append_log(f"[툴변경] 로봇모드 조회 실패: {mode_msg}\n")
            QMessageBox.warning(self, "툴 변경", f"로봇모드 조회 실패\n\n{mode_msg}")
            return

        mode_value, _mode_seen_at = self.backend.get_robot_mode_snapshot()
        try:
            mode_int = int(mode_value) if mode_value is not None else -1
        except Exception:
            mode_int = -1
        if mode_int != 0:
            self.append_log(f"[툴변경] 메뉴얼모드(0)에서만 변경할 수 있습니다. (현재 mode={mode_int})\n")
            QMessageBox.warning(
                self,
                "툴 변경",
                f"메뉴얼모드(0)에서만 진입할 수 있습니다.\n현재 모드: {mode_int}",
            )
            return

        ok_tcp, tcp_msg = self.backend.sync_current_tcp_once(force=True, timeout_sec=1.0)
        if not ok_tcp:
            self.append_log(f"[툴변경] 현재 TCP 조회 실패: {tcp_msg}\n")
        cur_name, _cur_seen = self.backend.get_current_tcp_snapshot() if hasattr(self.backend, "get_current_tcp_snapshot") else ("", None)
        current_display = "flange" if str(cur_name or "").strip() == "" else str(cur_name).strip()

        dialog = QDialog(self)
        dialog.setWindowTitle("툴 변경")
        dialog.setModal(True)
        dialog.resize(520, 280)

        layout = QVBoxLayout(dialog)
        layout.setContentsMargins(20, 18, 20, 16)
        layout.setSpacing(14)

        info = QLabel(
            f"현재 툴(TCP): {current_display}\n변경할 항목을 선택하세요.\n(메뉴얼모드에서만 변경 가능)",
            dialog,
        )
        layout.addWidget(info)
        layout.addSpacing(8)

        options = [("flange", ""), ("tool_gripper", "tool_gripper"), ("tool_checkerboard", "tool_checkerboard")]
        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)
        btn_row.addStretch(1)
        group = QButtonGroup(dialog)
        group.setExclusive(True)
        buttons = {}
        for idx, (label, _target) in enumerate(options):
            btn = QPushButton(label, dialog)
            btn.setCheckable(True)
            btn.setMinimumSize(140, 54)
            btn.setStyleSheet(
                "QPushButton { border: 1px solid #9aa0a6; border-radius: 6px; padding: 8px 12px; }"
                "QPushButton:checked { background: #1f6feb; color: #ffffff; border: 1px solid #1f6feb; }"
            )
            group.addButton(btn, idx)
            buttons[label] = btn
            btn_row.addWidget(btn)
        btn_row.addStretch(1)
        layout.addStretch(1)
        layout.addLayout(btn_row)
        layout.addStretch(1)

        if current_display in buttons:
            buttons[current_display].setChecked(True)
        else:
            buttons["flange"].setChecked(True)

        buttons_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, parent=dialog)
        buttons_box.setCenterButtons(False)
        buttons_box.accepted.connect(dialog.accept)
        buttons_box.rejected.connect(dialog.reject)
        layout.addWidget(buttons_box)

        if dialog.exec_() != QDialog.Accepted:
            self.append_log("[툴변경] 변경 취소\n")
            return

        selected_id = int(group.checkedId())
        if selected_id < 0 or selected_id >= len(options):
            QMessageBox.warning(self, "툴 변경", "변경할 툴을 선택하세요.")
            return

        selected_label, target_name = options[selected_id]
        wait = QDialog(self)
        wait.setWindowTitle("툴 변경")
        wait.setWindowModality(Qt.WindowModal)
        wait.setModal(True)
        wait_layout = QVBoxLayout(wait)
        wait_layout.addWidget(QLabel("툴(TCP) 적용중입니다...\n결과 확인중입니다...", wait))
        wait.setFixedSize(280, 90)
        wait.show()
        QApplication.processEvents()
        try:
            ok_set, msg_set = self.backend.set_current_tcp_name(target_name, timeout_sec=5.0)
            self.backend.sync_current_tcp_once(force=True, timeout_sec=1.0)
        finally:
            wait.done(0)
            wait.hide()
            wait.deleteLater()
            QApplication.processEvents()

        now_name, _now_seen = self.backend.get_current_tcp_snapshot() if hasattr(self.backend, "get_current_tcp_snapshot") else ("", None)
        now_display = "flange" if str(now_name or "").strip() == "" else str(now_name).strip()
        self._set_current_tool_text(now_name)

        if ok_set:
            self.append_log(f"[툴변경] 적용 성공: 요청={selected_label}, 현재={now_display} ({msg_set})\n")
            QMessageBox.information(
                self,
                "툴 변경",
                f"툴(TCP) 적용 성공\n\n요청: {selected_label}\n현재: {now_display}\n\n상세: {msg_set}",
                QMessageBox.Ok,
            )
        else:
            self.append_log(f"[툴변경] 적용 실패: 요청={selected_label}, 현재={now_display} ({msg_set})\n")
            QMessageBox.warning(
                self,
                "툴 변경",
                f"툴(TCP) 적용 실패\n\n요청: {selected_label}\n현재: {now_display}\n\n상세: {msg_set}",
                QMessageBox.Ok,
            )

    def _set_manual_mode_and_checkerboard_tcp(self, parent=None):
        if self.backend is None:
            return False, "백엔드 초기화 중입니다."
        if not hasattr(self.backend, "sync_robot_mode_once") or not hasattr(self.backend, "get_robot_mode_snapshot"):
            return False, "백엔드가 로봇모드 조회 기능을 지원하지 않습니다."
        if not hasattr(self.backend, "set_robot_mode"):
            return False, "백엔드가 로봇모드 변경 기능을 지원하지 않습니다."
        if not hasattr(self.backend, "sync_current_tcp_once") or not hasattr(self.backend, "set_current_tcp_name"):
            return False, "백엔드가 TCP 변경 기능을 지원하지 않습니다."

        holder = parent if parent is not None else self
        wait = QDialog(holder)
        wait.setWindowTitle("모드/TCP 설정")
        wait.setWindowModality(Qt.WindowModal)
        wait.setModal(True)
        wait_layout = QVBoxLayout(wait)
        wait_layout.addWidget(QLabel("수동모드 및 체커보드 TCP 적용중입니다...", wait))
        wait.setFixedSize(320, 86)
        wait.show()
        QApplication.processEvents()
        try:
            ok_mode_sync, msg_mode_sync = self.backend.sync_robot_mode_once(force=True, timeout_sec=1.2)
            if not ok_mode_sync:
                return False, f"로봇모드 조회 실패: {msg_mode_sync}"

            mode_value, _mode_seen_at = self.backend.get_robot_mode_snapshot()
            try:
                mode_int = int(mode_value) if mode_value is not None else -1
            except Exception:
                mode_int = -1

            mode_detail = "SKIP(이미 메뉴얼모드)"
            if mode_int != 0:
                ok_mode_set, msg_mode_set = self.backend.set_robot_mode(0, timeout_sec=8.0)
                mode_detail = msg_mode_set
                if not ok_mode_set:
                    return False, f"수동모드 전환 실패: {msg_mode_set}"

            ok_tcp_set, msg_tcp_set = self.backend.set_current_tcp_name("tool_checkerboard", timeout_sec=5.0)
            if not ok_tcp_set:
                return False, f"체커보드 TCP 적용 실패: {msg_tcp_set}"

            ok_tcp_sync, msg_tcp_sync = self.backend.sync_current_tcp_once(force=True, timeout_sec=1.2)
            if not ok_tcp_sync:
                return False, f"TCP 재조회 실패: {msg_tcp_sync}"
            tcp_name, _tcp_seen_at = self.backend.get_current_tcp_snapshot()
            tcp_name = str(tcp_name or "").strip()
            if tcp_name != "tool_checkerboard":
                return False, f"TCP 검증 실패: 현재 tcp='{tcp_name or 'flange'}'"

            self._refresh_current_tool_label_once(log_fail=False)
            return True, f"MODE={mode_detail}; TCP={msg_tcp_set}"
        finally:
            wait.done(0)
            wait.hide()
            wait.deleteLater()
            QApplication.processEvents()

    def _on_calibration_mode_toggled(self, checked):
        sender = self.sender()
        src_panel = 1
        if sender is getattr(self, "_calibration_mode_switch_2", None):
            checked = bool(self._calibration_mode_switch_2.isChecked())
            self._calibration_mode_enabled_2 = bool(checked)
            src_panel = 2
        elif hasattr(self, "_calibration_mode_switch") and self._calibration_mode_switch is not None:
            checked = bool(self._calibration_mode_switch.isChecked())
            self._calibration_mode_enabled_1 = bool(checked)
            src_panel = 1
        self._calibration_mode_enabled = bool(self._calibration_mode_enabled_1 or self._calibration_mode_enabled_2)
        self._clear_calibration_panel_state(src_panel)
        self._clear_vision_view_data(src_panel)
        if src_panel == 2:
            self._vision_state_text_2 = "전환중"
        else:
            self._vision_state_text = "전환중"
        now = time.monotonic()
        self._advance_vision_stream_token(src_panel)
        if src_panel == 2:
            self._vision_drop_frames_until_2 = now + 1.2
        else:
            self._vision_drop_frames_until_1 = now + 1.2
        if (not checked) and self._vision_to_robot_affine is None:
            self._try_load_calibration_matrix_on_startup()
        if src_panel == 2:
            self._vision_mode_switch_grace_until_2 = now + MODE_SWITCH_GRACE_SEC
        else:
            self._vision_mode_switch_grace_until_1 = now + MODE_SWITCH_GRACE_SEC
        self._rebind_external_vision_bridge_panel_for_mode(src_panel)
        self._update_calibration_mode_ui()
        base_enabled = False
        cache = self._robot_controls_enabled_cache
        if isinstance(cache, tuple):
            base_enabled = bool(cache[0])
        elif cache is not None:
            base_enabled = bool(cache)
        self._set_robot_controls_enabled(base_enabled)
        self.append_log(f"[캘리브레이션{src_panel}] 모드: {'ON' if bool(checked) else 'OFF'}\n")
        self._save_vision_serial_settings()

    def _tick_calibration_status_blink(self):
        self._calib_status_blink_on = not bool(getattr(self, "_calib_status_blink_on", False))
        if bool(getattr(self, "_calibration_mode_enabled_1", False) or getattr(self, "_calibration_mode_enabled_2", False)):
            self._update_calibration_mode_ui()

    def _is_calibration_board_detected(self, panel_index=None):
        try:
            panel = int(panel_index) if panel_index is not None else 1
        except Exception:
            panel = 1
        if panel == 2:
            pts = getattr(self, "_calib_last_grid_pts_2", None)
            seen_at = getattr(self, "_calib_last_points_at_2", None)
        else:
            pts = getattr(self, "_calib_last_grid_pts_1", None)
            seen_at = getattr(self, "_calib_last_points_at_1", None)
        if pts is None:
            return False
        if seen_at is None:
            return True
        return (time.monotonic() - float(seen_at)) <= max(0.2, float(CALIB_DETECTION_HOLD_SEC) + 0.2)

    def _is_calibration_full_data_ready(self, panel_index=None):
        try:
            panel = int(panel_index) if panel_index is not None else 1
        except Exception:
            panel = 1
        if panel == 2:
            ready = bool(getattr(self, "_calib_full_ready_2", False))
            seen_at = getattr(self, "_calib_last_points_at_2", None)
        else:
            ready = bool(getattr(self, "_calib_full_ready_1", False))
            seen_at = getattr(self, "_calib_last_points_at_1", None)
        if not ready:
            return False
        if seen_at is None:
            return True
        return (time.monotonic() - float(seen_at)) <= max(0.2, float(CALIB_DETECTION_HOLD_SEC) + 0.2)

    def _vision_mode_name(self, panel_index: int = 1):
        return "용량인식모드" if int(panel_index) == 2 else "객체인식모드"

    def _vision_panel_mode_name(self, panel_index: int = 1, calib_on=None):
        panel = 2 if int(panel_index) == 2 else 1
        if calib_on is None:
            calib_on = bool(self._calibration_mode_enabled_2) if panel == 2 else bool(self._calibration_mode_enabled_1)
        if bool(calib_on):
            return "TF"
        return "용량인식" if panel == 2 else "객체인식"

    def _layout_vision_mode_box(self, panel_index: int = 1):
        panel = 2 if int(panel_index) == 2 else 1
        box = getattr(self, "calibration_group_box_2", None) if panel == 2 else getattr(self, "calibration_group_box", None)
        if box is None:
            return
        calib_on = bool(self._calibration_mode_enabled_2) if panel == 2 else bool(self._calibration_mode_enabled_1)
        switch = self._calibration_mode_switch_2 if panel == 2 else self._calibration_mode_switch
        status_label = self._calibration_status_label_2 if panel == 2 else self._calibration_status_label
        matrix_label = self._calibration_matrix_file_label_2 if panel == 2 else self._calibration_matrix_file_label
        load_button = self._calibration_load_button_2 if panel == 2 else self._calibration_load_button
        transform_button = self._calibration_transform_button_2 if panel == 2 else self._calibration_transform_button
        badge = self._vision_mode_badge_2 if panel == 2 else self._vision_mode_badge_1
        rate_label = self._vision_meta_rate_label_2 if panel == 2 else self._vision_meta_rate_label_1
        detail = self._vision_runtime_detail_label_2 if panel == 2 else self._vision_runtime_detail_label_1
        data = self._vision_runtime_list_label_2 if panel == 2 else self._vision_runtime_list_label_1

        margin_x = 14
        margin_right = 8
        top_y = 10
        box_w = max(460, int(box.width()))
        box_h = max(110, int(box.height()))
        content_w = max(220, box_w - (margin_x * 2))
        switch_w = 50
        gap = 10
        meta_rate_w = 86
        badge_w = min(max(118, content_w - switch_w - (gap * 2) - 8), max(118, content_w - 100))
        if badge is not None:
            badge.setGeometry(margin_x, top_y, badge_w, 24)
            badge.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            badge.raise_()
        if rate_label is not None:
            if calib_on:
                rate_label.setGeometry(0, 0, 0, 0)
            else:
                rate_label.setGeometry(box_w - margin_right - meta_rate_w, box_h - 24, meta_rate_w, 16)
            rate_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            rate_label.raise_()
        if switch is not None:
            switch.setGeometry(box_w - margin_right - switch_w, top_y + 2, switch_w, 18)
            switch.raise_()
        if status_label is not None:
            status_label.setGeometry(margin_x, 36, content_w, 86 if not calib_on else 20)
            status_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)
            status_label.setWordWrap(True)
            status_label.raise_()
        if calib_on:
            if matrix_label is not None:
                matrix_label.setGeometry(margin_x, 56, content_w - 92, 22)
                matrix_label.raise_()
            if load_button is not None:
                load_button.setGeometry(box_w - margin_x - 82, 54, 82, 26)
                load_button.raise_()
            if transform_button is not None:
                transform_button.setGeometry(margin_x, 86, content_w, 32)
                transform_button.raise_()
            if detail is not None:
                detail.setGeometry(0, 0, 0, 0)
                detail.hide()
            if data is not None:
                data.setGeometry(0, 0, 0, 0)
                data.hide()
        else:
            if detail is not None:
                detail.setGeometry(0, 0, 0, 0)
                detail.hide()
            if data is not None:
                data.setGeometry(0, 0, 0, 0)
                data.hide()

    def _vision_runtime_summary(self, panel_index: int = 1):
        panel = 2 if int(panel_index) == 2 else 1
        mode_name = self._vision_mode_name(panel)
        vision_enabled = bool(self._top_status_enabled.get("vision2" if panel == 2 else "vision", True))
        meta_cycle_ms = self._vision_meta_cycle_ms_2 if panel == 2 else self._vision_meta_cycle_ms_1
        payload = self._current_vision_meta_payload_for_ui(panel)
        if meta_cycle_ms is not None:
            meta_rate_text = f"{meta_cycle_ms:.0f}ms"
        else:
            proc_ms = None
            source_age_ms = None
            if isinstance(payload, dict):
                try:
                    proc_ms = float(payload.get("processing_ms"))
                except Exception:
                    proc_ms = None
                try:
                    source_age_ms = float(payload.get("source_age_ms"))
                except Exception:
                    source_age_ms = None
            if proc_ms is not None:
                meta_rate_text = f"{proc_ms:.0f}ms"
            elif source_age_ms is not None:
                meta_rate_text = f"{source_age_ms:.0f}ms"
            elif isinstance(payload, dict):
                meta_rate_text = f"{(1000.0 / max(1.0, float(VISION_META_PROCESS_HZ))):.0f}ms"
            else:
                meta_rate_text = "-"
        if not vision_enabled:
            return {
                "mode_name": mode_name,
                "status_text": f"[{mode_name}] 비전 비활성화",
                "status_style": "color: #8d6e63; font-size: 9.5pt; font-weight: 800;",
                "detail_text": "패널이 꺼져 있습니다.",
                "list_text": "데이터 리스트: -",
                "meta_text": "",
                "status_color": "#8d6e63",
                "summary_lines": [],
                "meta_rate_text": meta_rate_text,
                "detail_visible": False,
                "list_visible": False,
                "meta_visible": False,
            }
        if not isinstance(payload, dict):
            return {
                "mode_name": mode_name,
                "status_text": f"[{mode_name}] 메타 대기 중",
                "status_style": "color: #8d6e63; font-size: 9.5pt; font-weight: 800;",
                "detail_text": "최근 감지 데이터 없음",
                "list_text": "데이터 리스트: -",
                "meta_text": "",
                "status_color": "#8d6e63",
                "summary_lines": [],
                "meta_rate_text": meta_rate_text,
                "detail_visible": False,
                "list_visible": False,
                "meta_visible": False,
            }
        detections = payload.get("detections")
        if not isinstance(detections, list):
            detections = []
        if panel == 1:
            if not detections:
                return {
                    "mode_name": mode_name,
                    "status_text": f"[{mode_name}] 객체 미감지",
                    "status_style": "color: #ef6c00; font-size: 9.5pt; font-weight: 800;",
                    "detail_text": "감지 결과 없음",
                    "list_text": "데이터 리스트: -",
                    "meta_text": "",
                    "status_color": "#ef6c00",
                    "summary_lines": [],
                    "meta_rate_text": meta_rate_text,
                    "detail_visible": False,
                    "list_visible": False,
                    "meta_visible": False,
                }
            top = detections[0]
            class_name = str(top.get("class_name", "-"))
            conf = top.get("confidence")
            depth_m = top.get("depth_m")
            detail_parts = [f"대표 {class_name}"]
            if conf is not None:
                try:
                    detail_parts.append(f"정확도 {float(conf):.2f}")
                except Exception:
                    pass
            if depth_m is not None:
                try:
                    detail_parts.append(f"거리 {float(depth_m):.2f}m")
                except Exception:
                    pass
            detail = " / ".join(detail_parts)
            line_parts = []
            for idx, det in enumerate(detections[:3], start=1):
                name = str(det.get("class_name", "-"))
                depth_text = ""
                try:
                    if det.get("depth_m") is not None:
                        depth_text = f" {float(det.get('depth_m')):.2f}m"
                except Exception:
                    pass
                line_parts.append(f"{idx}.{name}{depth_text}")
            return {
                "mode_name": mode_name,
                "status_text": f"[{mode_name}] 객체 {len(detections)}개 감지",
                "status_style": "color: #2e7d32; font-size: 9.5pt; font-weight: 800;",
                "detail_text": detail,
                "list_text": " / ".join(line_parts) if line_parts else "데이터 리스트: -",
                "meta_text": "메타: class / conf / bbox / center / depth",
                "status_color": "#2e7d32",
                "summary_lines": [
                    f"대표: {class_name} / {float(conf):.2f}" if conf is not None else f"대표: {class_name}",
                    "감지: " + " / ".join(line_parts[:3]) if line_parts else "",
                    "메타: class / conf / bbox / center / depth",
                ],
                "meta_rate_text": meta_rate_text,
                "detail_visible": True,
                "list_visible": bool(line_parts),
                "meta_visible": True,
            }

        bottle = payload.get("bottle") if isinstance(payload.get("bottle"), dict) else None
        liquid = payload.get("liquid") if isinstance(payload.get("liquid"), dict) else None
        volume_ml = liquid.get("volume_ml") if liquid else None
        if volume_ml is not None:
            try:
                status_text = f"[{mode_name}] 현재 용량 {float(volume_ml):.1f}ml"
                status_style = "color: #2e7d32; font-size: 9.5pt; font-weight: 800;"
            except Exception:
                status_text = f"[{mode_name}] 용량 계산 완료"
                status_style = "color: #2e7d32; font-size: 9.5pt; font-weight: 800;"
        elif bottle or liquid:
            status_text = f"[{mode_name}] 병/액체 감지 중"
            status_style = "color: #ef6c00; font-size: 9.5pt; font-weight: 800;"
        else:
            status_text = f"[{mode_name}] 용량 대상 미감지"
            status_style = "color: #8d6e63; font-size: 9.5pt; font-weight: 800;"
        detail_parts = []
        if bottle:
            detail_parts.append(f"병 {str(bottle.get('class_name', '-'))}")
            if bottle.get("depth_m") is not None:
                try:
                    detail_parts.append(f"병거리 {float(bottle.get('depth_m')):.2f}m")
                except Exception:
                    pass
        if liquid:
            detail_parts.append(f"액체 {str(liquid.get('class_name', '-'))}")
        if liquid and liquid.get("depth_m") is not None:
            try:
                detail_parts.append(f"액체거리 {float(liquid.get('depth_m')):.2f}m")
            except Exception:
                pass
        if volume_ml is not None:
            try:
                detail_parts.insert(0, f"현재용량 {float(volume_ml):.1f}ml")
            except Exception:
                pass
        detail_text = " / ".join(detail_parts) if detail_parts else "대표 대상 없음"
        list_parts = []
        if bottle and bottle.get("bottom_y") is not None:
            try:
                list_parts.append(f"병바닥Y {int(bottle.get('bottom_y'))}")
            except Exception:
                pass
        if liquid:
            if liquid.get("waterline_y") is not None:
                list_parts.append(f"수면선Y {int(liquid.get('waterline_y'))}")
            if liquid.get("height_px") is not None:
                list_parts.append(f"높이 {int(liquid.get('height_px'))}px")
            if liquid.get("height_px_ema") is not None:
                try:
                    list_parts.append(f"EMA {float(liquid.get('height_px_ema')):.1f}px")
                except Exception:
                    pass
            if liquid.get("volume_ml") is not None:
                try:
                    list_parts.append(f"용량 {float(liquid.get('volume_ml')):.1f}ml")
                except Exception:
                    pass
        list_text = " / ".join(list_parts) if list_parts else "데이터 리스트: -"
        return {
            "mode_name": mode_name,
            "status_text": status_text,
            "status_style": status_style,
            "detail_text": detail_text,
            "list_text": list_text,
            "meta_text": "메타: bottle / liquid / contour / bbox / depth / waterline / height / volume",
            "status_color": "#2e7d32" if volume_ml is not None else ("#ef6c00" if (bottle or liquid) else "#8d6e63"),
            "summary_lines": [
                detail_text,
                "값: " + " / ".join(list_parts[:3]) if list_parts else "",
                "메타: bottle / liquid / contour / bbox / depth / waterline / height / volume",
            ],
            "meta_rate_text": meta_rate_text,
            "detail_visible": bool(detail_parts),
            "list_visible": bool(list_parts),
            "meta_visible": True,
        }

    def _update_vision_runtime_panel_ui(self, panel_index: int = 1):
        panel = 2 if int(panel_index) == 2 else 1
        calib_on = bool(self._calibration_mode_enabled_2) if panel == 2 else bool(self._calibration_mode_enabled_1)
        switch = self._calibration_mode_switch_2 if panel == 2 else self._calibration_mode_switch
        status_label = self._calibration_status_label_2 if panel == 2 else self._calibration_status_label
        matrix_label = self._calibration_matrix_file_label_2 if panel == 2 else self._calibration_matrix_file_label
        load_button = self._calibration_load_button_2 if panel == 2 else self._calibration_load_button
        transform_button = self._calibration_transform_button_2 if panel == 2 else self._calibration_transform_button
        badge = self._vision_mode_badge_2 if panel == 2 else self._vision_mode_badge_1
        rate_label = self._vision_meta_rate_label_2 if panel == 2 else self._vision_meta_rate_label_1
        detail = self._vision_runtime_detail_label_2 if panel == 2 else self._vision_runtime_detail_label_1
        data = self._vision_runtime_list_label_2 if panel == 2 else self._vision_runtime_list_label_1
        runtime = self._vision_runtime_summary(panel)

        self._layout_vision_mode_box(panel)

        if switch is not None:
            switch.blockSignals(True)
            switch.setText("TF")
            switch.setToolTip("TF 모드 변경 스위치")
            switch.blockSignals(False)
            switch.setStyleSheet("font-size: 7pt; font-weight: 700; padding-left: 2px;")
        if badge is not None:
            badge_title = self._vision_panel_mode_name(panel, calib_on=calib_on)
            badge.setTextFormat(Qt.PlainText)
            badge.setText(badge_title)
            badge.setStyleSheet("color: #1f2937; background: transparent; border: none; font-size: 11pt; font-weight: 800;")
            badge.show()
        if rate_label is not None:
            if calib_on:
                rate_label.clear()
                rate_label.hide()
            else:
                rate_text = str(runtime.get("meta_rate_text", "") or "-")
                rate_label.setText(f"메타 {rate_text}")
                rate_label.setStyleSheet("color: #475569; background: transparent; border: none; font-size: 7.8pt; font-weight: 700;")
                rate_label.show()
        if status_label is not None and (not calib_on):
            status_label.setVisible(True)
            status_text = str(runtime.get("status_text", "") or "")
            summary_lines = [str(line).strip() for line in list(runtime.get("summary_lines", []) or []) if str(line).strip()]
            if summary_lines:
                status_html = (
                    f"<div style='font-size:9.5pt; font-weight:800; color:{html.escape(str(runtime.get('status_color', '#2e7d32')))};'>"
                    f"{html.escape(status_text)}</div>"
                )
                for idx, line in enumerate(summary_lines):
                    font_size = "7.6pt" if idx == 0 else "7.2pt"
                    status_html += (
                        f"<div style='font-size:{font_size}; font-weight:600; color:#475569; margin-top:1px;'>"
                        f"{html.escape(line)}</div>"
                    )
                status_label.setTextFormat(Qt.RichText)
                status_label.setText(status_html)
                status_label.setStyleSheet("background: transparent;")
            else:
                status_label.setTextFormat(Qt.PlainText)
                status_label.setText(status_text)
                status_label.setStyleSheet(runtime["status_style"])
        if matrix_label is not None:
            if calib_on:
                matrix_label.show()
            else:
                matrix_label.hide()
        if detail is not None:
            detail.hide()
            detail.clear()
        if data is not None:
            data.hide()
            data.clear()
        if load_button is not None and (not calib_on):
            load_button.hide()
        if transform_button is not None and (not calib_on):
            transform_button.hide()

    def _update_calibration_mode_ui(self):
        calib_on_1 = bool(getattr(self, "_calibration_mode_enabled_1", False))
        calib_on_2 = bool(getattr(self, "_calibration_mode_enabled_2", False))
        calib_on = bool(calib_on_1 or calib_on_2)
        calib_seq_running = bool(getattr(self, "_calibration_sequence_running", False))
        self._calibration_mode_enabled = calib_on
        calib_board_detected_1 = bool(self._is_calibration_board_detected(panel_index=1))
        calib_board_detected_2 = bool(self._is_calibration_board_detected(panel_index=2))
        calib_detected_1 = bool(self._is_calibration_full_data_ready(panel_index=1))
        calib_detected_2 = bool(self._is_calibration_full_data_ready(panel_index=2))
        if hasattr(self, "_calibration_mode_switch") and self._calibration_mode_switch is not None:
            self._calibration_mode_switch.blockSignals(True)
            self._calibration_mode_switch.setChecked(calib_on_1)
            self._calibration_mode_switch.setText("TF")
            self._calibration_mode_switch.setToolTip("객체인식/TF 모드 변경")
            self._calibration_mode_switch.blockSignals(False)
            self._calibration_mode_switch.setStyleSheet("font-size: 7pt; font-weight: 700; padding-left: 2px;")
        if hasattr(self, "_calibration_mode_switch_2") and self._calibration_mode_switch_2 is not None:
            self._calibration_mode_switch_2.blockSignals(True)
            self._calibration_mode_switch_2.setChecked(calib_on_2)
            self._calibration_mode_switch_2.setText("TF")
            self._calibration_mode_switch_2.setToolTip("용량인식/TF 모드 변경")
            self._calibration_mode_switch_2.blockSignals(False)
            self._calibration_mode_switch_2.setStyleSheet("font-size: 7pt; font-weight: 700; padding-left: 2px;")
        if hasattr(self, "_calibration_transform_button") and self._calibration_transform_button is not None:
            self._calibration_transform_button.setVisible(calib_on_1)
            self._calibration_transform_button.setEnabled(
                bool(calib_on_1 and (not calib_seq_running) and self._top_status_enabled.get("vision", True))
            )
        if hasattr(self, "_calibration_transform_button_2") and self._calibration_transform_button_2 is not None:
            self._calibration_transform_button_2.setVisible(calib_on_2)
            self._calibration_transform_button_2.setEnabled(
                bool(calib_on_2 and (not calib_seq_running) and self._top_status_enabled.get("vision2", True))
            )
        if hasattr(self, "_calibration_load_button") and self._calibration_load_button is not None:
            self._calibration_load_button.setVisible(calib_on_1)
            self._calibration_load_button.setEnabled(bool(calib_on_1 and (not calib_seq_running)))
        if hasattr(self, "_calibration_load_button_2") and self._calibration_load_button_2 is not None:
            self._calibration_load_button_2.setVisible(calib_on_2)
            self._calibration_load_button_2.setEnabled(bool(calib_on_2 and (not calib_seq_running)))
        if hasattr(self, "_calibration_matrix_file_label") and self._calibration_matrix_file_label is not None:
            base = os.path.basename(self._calib_matrix_path_1) if self._calib_matrix_path_1 else "(없음)"
            self._calibration_matrix_file_label.setText(f"현재 적용행렬 : {base}")
            self._calibration_matrix_file_label.setStyleSheet("color: #666666; font-size: 8pt;")
        if hasattr(self, "_calibration_matrix_file_label_2") and self._calibration_matrix_file_label_2 is not None:
            base = os.path.basename(self._calib_matrix_path_2) if self._calib_matrix_path_2 else "(없음)"
            self._calibration_matrix_file_label_2.setText(f"현재 적용행렬 : {base}")
            self._calibration_matrix_file_label_2.setStyleSheet("color: #666666; font-size: 8pt;")
        if hasattr(self, "_calibration_status_label") and self._calibration_status_label is not None:
            self._calibration_status_label.setVisible(calib_on_1)
            if calib_on_1:
                status = "[실패] 켈리브레이션 보드 인식 실패"
                reason = str(getattr(self, "_calib_full_reason_1", "") or getattr(self, "_calib_last_reason_1", "") or "").strip()
                if calib_detected_1:
                    status = "[성공] 켈리브레이션 보드 인식 성공"
                elif calib_board_detected_1:
                    status = "[대기] 켈리브레이션 데이터 수집 중"
                    if reason:
                        status = f"{status} ({reason})"
                self._calibration_status_label.setText(status)
                if calib_detected_1:
                    self._calibration_status_label.setStyleSheet(
                        "color: #2e7d32; font-size: 10pt; font-weight: 800;"
                    )
                elif calib_board_detected_1:
                    self._calibration_status_label.setStyleSheet(
                        "color: #ef6c00; font-size: 10pt; font-weight: 800;"
                    )
                else:
                    blink_on = bool(getattr(self, "_calib_status_blink_on", False))
                    self._calibration_status_label.setStyleSheet(
                        "color: #d32f2f; font-size: 10pt; font-weight: 800;"
                        if blink_on
                        else "color: #f9a825; font-size: 10pt; font-weight: 800;"
                    )
            else:
                self._calibration_status_label.setText("")
                self._calibration_status_label.setStyleSheet(
                    "color: #5f5a1e; font-size: 9pt; font-weight: 700;"
                )
        if hasattr(self, "_vision_mode_badge_1") and self._vision_mode_badge_1 is not None:
            self._vision_mode_badge_1.setText(self._vision_panel_mode_name(1, calib_on=calib_on_1))
        if hasattr(self, "_calibration_status_label_2") and self._calibration_status_label_2 is not None:
            self._calibration_status_label_2.setVisible(calib_on_2)
            if calib_on_2:
                status = "[실패] 켈리브레이션 보드 인식 실패"
                reason = str(getattr(self, "_calib_full_reason_2", "") or getattr(self, "_calib_last_reason_2", "") or "").strip()
                if calib_detected_2:
                    status = "[성공] 켈리브레이션 보드 인식 성공"
                elif calib_board_detected_2:
                    status = "[대기] 켈리브레이션 데이터 수집 중"
                    if reason:
                        status = f"{status} ({reason})"
                self._calibration_status_label_2.setText(status)
                if calib_detected_2:
                    self._calibration_status_label_2.setStyleSheet(
                        "color: #2e7d32; font-size: 10pt; font-weight: 800;"
                    )
                elif calib_board_detected_2:
                    self._calibration_status_label_2.setStyleSheet(
                        "color: #ef6c00; font-size: 10pt; font-weight: 800;"
                    )
                else:
                    blink_on = bool(getattr(self, "_calib_status_blink_on", False))
                    self._calibration_status_label_2.setStyleSheet(
                        "color: #d32f2f; font-size: 10pt; font-weight: 800;"
                        if blink_on
                        else "color: #f9a825; font-size: 10pt; font-weight: 800;"
                    )
            else:
                self._calibration_status_label_2.setText("")
                self._calibration_status_label_2.setStyleSheet(
                    "color: #5f5a1e; font-size: 9pt; font-weight: 700;"
                )
        if hasattr(self, "_vision_mode_badge_2") and self._vision_mode_badge_2 is not None:
            self._vision_mode_badge_2.setText(self._vision_panel_mode_name(2, calib_on=calib_on_2))
        for panel, detail, data, calib_on_panel in (
            (1, getattr(self, "_vision_runtime_detail_label_1", None), getattr(self, "_vision_runtime_list_label_1", None), calib_on_1),
            (2, getattr(self, "_vision_runtime_detail_label_2", None), getattr(self, "_vision_runtime_list_label_2", None), calib_on_2),
        ):
            _ = panel
            if detail is not None and calib_on_panel:
                detail.hide()
                detail.clear()
            if data is not None and calib_on_panel:
                data.hide()
                data.clear()

        # yolo_view 배경색은 프레임 렌더링/미수신 상태에서만 관리한다.
        # (캘리브레이션 상태 갱신 타이머와 충돌해 깜박이는 현상 방지)

        # Normal vision move UI is hidden in calibration mode.
        for w in self._iter_vision_move_widgets():
            if w is not None:
                w.setVisible(True)
        if hasattr(self, "_vision_dialog_toggle_button") and self._vision_dialog_toggle_button is not None:
            self._vision_dialog_toggle_button.hide()
        self._update_vision_runtime_panel_ui(1)
        self._update_vision_runtime_panel_ui(2)

    def _parse_point_file_sections(self, file_path):
        sections = {}
        current = "default"
        sections[current] = {}
        with open(file_path, "r", encoding="utf-8") as f:
            for raw in f:
                line = raw.strip()
                if not line:
                    continue
                if line.startswith("#"):
                    head = line.lstrip("#").strip().lower()
                    if head.startswith("robot"):
                        current = "robot"
                    elif head.startswith("vision"):
                        current = "vision"
                    elif head.startswith("rotationmat"):
                        current = "rotationmat"
                    else:
                        current = "default"
                    sections.setdefault(current, {})
                    continue
                if "=" not in line:
                    continue
                k, v = line.split("=", 1)
                key = k.strip().lower()
                vals = [x.strip() for x in v.split(",")]
                if len(vals) < 1:
                    continue
                try:
                    numbers = [float(x) for x in vals]
                except Exception:
                    continue
                sections.setdefault(current, {})[key] = numbers
        return sections

    def _parse_bool_text(self, raw_value, default=True):
        text = str(raw_value if raw_value is not None else "").strip().lower()
        if text in ("1", "true", "on", "yes", "y"):
            return True
        if text in ("0", "false", "off", "no", "n"):
            return False
        return bool(default)

    def _parse_float_values(self, raw_values, expected_len=None):
        if raw_values is None:
            return None
        if not isinstance(raw_values, (list, tuple)):
            raw_values = [raw_values]
        try:
            vals = [float(v) for v in list(raw_values)]
        except Exception:
            return None
        if expected_len is not None:
            if len(vals) < int(expected_len):
                return None
            vals = vals[: int(expected_len)]
        return vals

    def _format_pose6_summary(self, pose6):
        vals = self._parse_float_values(pose6, expected_len=6)
        if vals is None:
            return "-"
        labels = ("X", "Y", "Z", "A", "B", "C")
        return ", ".join(f"{labels[i]}={self._fmt_ui_float(vals[i], 2)}" for i in range(6))

    def _format_joint_summary(self, joints):
        vals = self._parse_float_values(joints, expected_len=6)
        if vals is None:
            return "-"
        labels = ("J1", "J2", "J3", "J4", "J5", "J6")
        return ", ".join(f"{labels[i]}={self._fmt_ui_float(vals[i], 2)}" for i in range(6))

    def _format_xyz_summary(self, xyz):
        vals = self._parse_float_values(xyz, expected_len=3)
        if vals is None:
            return "-"
        return ", ".join(
            (
                f"X={self._fmt_ui_float(vals[0], 3)}",
                f"Y={self._fmt_ui_float(vals[1], 3)}",
                f"Z={self._fmt_ui_float(vals[2], 3)}",
            )
        )

    def _format_xyz_compact(self, xyz):
        vals = self._parse_float_values(xyz, expected_len=3)
        if vals is None:
            return "-"
        return f"({self._fmt_ui_float(vals[0], 2)}, {self._fmt_ui_float(vals[1], 2)}, {self._fmt_ui_float(vals[2], 2)})"

    def _predict_robot_xyz_from_vision_xyz(self, vision_xyz, panel_index: int = 1):
        vals = self._parse_float_values(vision_xyz, expected_len=3)
        if vals is None:
            return None
        panel = 2 if int(panel_index) == 2 else 1
        affine = self._vision_to_robot_affine_2 if panel == 2 else self._vision_to_robot_affine_1
        if affine is None:
            affine = self._vision_to_robot_affine
        if affine is None:
            return None
        try:
            vec = np.array([float(vals[0]), float(vals[1]), float(vals[2]), 1.0], dtype=np.float64)
            out = vec @ np.asarray(affine, dtype=np.float64)
            if out.shape[0] < 3 or (not np.isfinite(out[:3]).all()):
                return None
            return np.asarray(out[:3], dtype=np.float64)
        except Exception:
            return None

    def _format_calibration_measurement_summary(self, vision_xyz=None, robot_xyz=None, predicted_robot_xyz=None):
        parts = []
        if self._parse_float_values(vision_xyz, expected_len=3) is not None:
            parts.append(f"V{self._format_xyz_compact(vision_xyz)}")
        if self._parse_float_values(robot_xyz, expected_len=3) is not None:
            parts.append(f"R{self._format_xyz_compact(robot_xyz)}")
        if self._parse_float_values(predicted_robot_xyz, expected_len=3) is not None:
            parts.append(f"C{self._format_xyz_compact(predicted_robot_xyz)}")
        return " | ".join(parts) if parts else "-"

    def _load_calibration_sequence_rows(self, panel_index: int = 1):
        rows = self._load_parameter_rows()
        panel = 2 if int(panel_index) == 2 else 1
        prefix = f"vision{panel}_calib_seq_"
        home_posj = None
        if self.backend is not None and hasattr(self.backend, "get_home_posj"):
            try:
                home_posj = self._parse_float_values(self.backend.get_home_posj(), expected_len=6)
            except Exception:
                home_posj = None
        if home_posj is None:
            home_posj = self._parse_float_values(HOME_POSJ, expected_len=6)

        def _resolve_sequence_storage_key(row_key: str):
            key_text = str(row_key or "").strip()
            if key_text == "initial":
                explicit_keys = (
                    f"{prefix}initial_enabled",
                    f"{prefix}initial_j",
                    f"{prefix}initial_pose6",
                    f"{prefix}initial_xyzabc",
                )
                if any(rows.get(name) for name in explicit_keys):
                    return "initial"
                legacy_home_keys = (
                    f"{prefix}home_enabled",
                    f"{prefix}home_j",
                    f"{prefix}home_pose6",
                    f"{prefix}home_xyzabc",
                )
                if any(rows.get(name) for name in legacy_home_keys):
                    return "home"
                legacy_end_home_keys = (
                    f"{prefix}end_home_enabled",
                    f"{prefix}end_home_j",
                    f"{prefix}end_home_pose6",
                    f"{prefix}end_home_xyzabc",
                )
                if any(rows.get(name) for name in legacy_end_home_keys):
                    return "end_home"
                return "initial"
            if key_text == "end":
                explicit_keys = (
                    f"{prefix}end_enabled",
                    f"{prefix}end_j",
                    f"{prefix}end_pose6",
                    f"{prefix}end_xyzabc",
                )
                if any(rows.get(name) for name in explicit_keys):
                    return "end"
                legacy_end_home_keys = (
                    f"{prefix}end_home_enabled",
                    f"{prefix}end_home_j",
                    f"{prefix}end_home_pose6",
                    f"{prefix}end_home_xyzabc",
                )
                if any(rows.get(name) for name in legacy_end_home_keys):
                    return "end_home"
                legacy_home_keys = (
                    f"{prefix}home_enabled",
                    f"{prefix}home_j",
                    f"{prefix}home_pose6",
                    f"{prefix}home_xyzabc",
                )
                if any(rows.get(name) for name in legacy_home_keys):
                    return "home"
                return "end"
            if key_text not in ("return1", "return2"):
                return key_text
            explicit_keys = (
                f"{prefix}{key_text}_enabled",
                f"{prefix}{key_text}_j",
                f"{prefix}{key_text}_pose6",
                f"{prefix}{key_text}_xyzabc",
            )
            if any(rows.get(name) for name in explicit_keys):
                return key_text
            legacy_keys = (
                f"{prefix}return_home_enabled",
                f"{prefix}return_home_j",
                f"{prefix}return_home_pose6",
                f"{prefix}return_home_xyzabc",
            )
            if any(rows.get(name) for name in legacy_keys):
                return "return_home"
            return key_text

        items = []
        for key, label in CALIB_SEQUENCE_ROW_DEFS:
            storage_key = _resolve_sequence_storage_key(key)
            entry = {
                "key": str(key),
                "label": str(label),
                "enabled": True,
                "capture": str(key).startswith("p"),
                "move_kind": "pose6",
                "target_pose6": None,
                "target_joint": None,
                "target_text": "-",
            }
            if key in ("initial", "end"):
                entry["enabled"] = self._parse_bool_text(((rows.get(f"{prefix}{storage_key}_enabled") or ["1"])[0]), default=True)
                entry["move_kind"] = "home"
                entry["target_joint"] = home_posj
                entry["target_text"] = self._format_joint_summary(home_posj)
                items.append(entry)
                continue

            entry["enabled"] = self._parse_bool_text(((rows.get(f"{prefix}{storage_key}_enabled") or ["1"])[0]), default=True)
            pose6 = self._parse_float_values(rows.get(f"{prefix}{storage_key}_pose6"), expected_len=6)
            if pose6 is None:
                pose6 = self._parse_float_values(rows.get(f"{prefix}{storage_key}_xyzabc"), expected_len=6)
            target_joint = self._parse_float_values(rows.get(f"{prefix}{storage_key}_j"), expected_len=6)
            entry["target_pose6"] = pose6
            entry["target_joint"] = target_joint
            if pose6 is not None:
                entry["move_kind"] = "pose6"
                entry["target_text"] = self._format_pose6_summary(pose6)
            elif target_joint is not None:
                entry["move_kind"] = "joint"
                entry["target_text"] = self._format_joint_summary(target_joint)
            else:
                entry["enabled"] = False
                entry["target_text"] = "미설정"
            items.append(entry)
        return items

    def _save_calibration_sequence_row_config(self, panel_index: int, row_key: str, enabled=None, joints=None, pose6=None):
        panel = 2 if int(panel_index) == 2 else 1
        key = str(row_key or "").strip()
        if not key:
            return False, "시퀀스 키가 없습니다."
        rows = self._load_parameter_rows()
        prefix = f"vision{panel}_calib_seq_{key}"
        if enabled is not None:
            rows[f"{prefix}_enabled"] = ["1" if bool(enabled) else "0"]

        if key in ("initial", "end"):
            if joints is not None:
                if self.backend is None or not hasattr(self.backend, "set_home_posj"):
                    return False, "백엔드 초기위치 저장 기능을 지원하지 않습니다."
                ok_home, msg_home = self.backend.set_home_posj(joints)
                if not ok_home:
                    return False, msg_home
            self._save_parameter_rows(rows)
            return True, "초기위치 설정 저장 완료"

        if joints is not None:
            joint_vals = self._parse_float_values(joints, expected_len=6)
            if joint_vals is None:
                return False, "조인트 값 형식이 올바르지 않습니다."
            rows[f"{prefix}_j"] = [f"{v:.6f}" for v in joint_vals]
        if pose6 is not None:
            pose_vals = self._parse_float_values(pose6, expected_len=6)
            if pose_vals is None:
                return False, "좌표 값 형식이 올바르지 않습니다."
            saved = [f"{v:.6f}" for v in pose_vals]
            rows[f"{prefix}_pose6"] = list(saved)
            rows[f"{prefix}_xyzabc"] = list(saved)
        self._save_parameter_rows(rows)
        return True, "시퀀스 설정 저장 완료"

    def _capture_current_teach_pose(self):
        if self.backend is None:
            return None, None, "백엔드 초기화 중입니다."
        posj = None
        posx = None
        seen_at = None
        last_err = "현재 좌표를 얻지 못했습니다."
        if hasattr(self.backend, "get_position_snapshot"):
            try:
                data, seen_at = self.backend.get_position_snapshot()
            except Exception:
                data, seen_at = (None, None)
            if data and len(data) >= 2:
                try:
                    if data[0] is not None:
                        posj = [float(v) for v in list(data[0])[:6]]
                except Exception:
                    posj = None
                try:
                    if data[1] is not None:
                        posx = [float(v) for v in list(data[1])[:6]]
                except Exception:
                    posx = None
        if posx is None and hasattr(self.backend, "get_current_posx_live"):
            posx_live, _sol_live, err_live = self.backend.get_current_posx_live()
            if posx_live is not None:
                try:
                    posx = [float(v) for v in list(posx_live)[:6]]
                except Exception:
                    posx = None
            elif err_live:
                last_err = str(err_live)
        if posj is None or posx is None:
            return None, None, last_err
        if seen_at is not None and (time.monotonic() - float(seen_at)) > max(1.0, POSITION_STALE_SEC):
            return None, None, "현재 좌표 수신이 지연되어 티칭할 수 없습니다."
        return posj, posx, None

    def _get_robot_mode_int(self):
        if self.backend is None or not hasattr(self.backend, "sync_robot_mode_once") or not hasattr(self.backend, "get_robot_mode_snapshot"):
            return None
        try:
            self.backend.sync_robot_mode_once(force=True, timeout_sec=1.2)
        except Exception:
            pass
        try:
            mode_value, mode_seen_at = self.backend.get_robot_mode_snapshot()
            if mode_value is None or mode_seen_at is None:
                return None
            return int(mode_value)
        except Exception:
            return None

    def _get_current_tcp_name(self, force_sync: bool = False):
        if self.backend is None or not hasattr(self.backend, "get_current_tcp_snapshot"):
            return "", None
        if force_sync and hasattr(self.backend, "sync_current_tcp_once"):
            try:
                self.backend.sync_current_tcp_once(force=True, timeout_sec=1.0)
            except Exception:
                pass
        try:
            tcp_name, seen_at = self.backend.get_current_tcp_snapshot()
        except Exception:
            return "", None
        return str(tcp_name or "").strip(), seen_at

    def _is_checkerboard_tcp_ready(self, force_sync: bool = False):
        tcp_name, seen_at = self._get_current_tcp_name(force_sync=force_sync)
        display = "flange" if not tcp_name else tcp_name
        return bool(tcp_name == "tool_checkerboard"), display, seen_at

    def _prepare_calibration_motion_ready(self, parent=None, status_callback=None):
        original_mode = self._get_robot_mode_int()
        if status_callback is not None:
            status_callback("체커보드 TCP 확인중...")
        tcp_ready, tcp_display, _tcp_seen_at = self._is_checkerboard_tcp_ready(force_sync=True)
        if tcp_ready:
            return True, f"현재 TCP가 {tcp_display} 상태라 추가 모드/TCP 변경 없이 진행합니다.", None
        if status_callback is not None:
            status_callback("체커보드 TCP 적용 준비중...")
        ok_setup, msg_setup = self._set_manual_mode_and_checkerboard_tcp(parent=parent)
        if not ok_setup:
            return False, msg_setup, original_mode
        if self.backend is None or not hasattr(self.backend, "set_robot_mode"):
            return False, "백엔드가 로봇모드 변경 기능을 지원하지 않습니다.", original_mode
        if status_callback is not None:
            status_callback("오토모드 전환중...")
        ok_auto, msg_auto = self.backend.set_robot_mode(1, timeout_sec=8.0)
        if not ok_auto:
            return False, f"오토모드 전환 실패: {msg_auto}", original_mode
        return True, f"{msg_setup}; {msg_auto}", original_mode

    def _restore_robot_mode(self, mode_value):
        if self.backend is None or not hasattr(self.backend, "set_robot_mode"):
            return False, "백엔드가 로봇모드 변경 기능을 지원하지 않습니다."
        try:
            target_mode = int(mode_value)
        except Exception:
            return False, "복원할 로봇모드가 없습니다."
        ok, msg = self.backend.set_robot_mode(target_mode, timeout_sec=8.0)
        if ok and target_mode == 0:
            self._request_current_tool_sync(retry_window_sec=6.0, immediate=True)
        return bool(ok), str(msg)

    def _prepare_calibration_gripper_closed(self, status_callback=None):
        if self.backend is None:
            return False, "백엔드 초기화 중입니다."
        if not hasattr(self.backend, "send_gripper_move"):
            return True, "그리퍼 제어 미지원: 생략"
        if not bool(getattr(self.backend, "use_real_gripper", False)):
            return True, "그리퍼 비활성화: 생략"
        if status_callback is not None:
            status_callback("그리퍼 0mm 닫는 중...")
        ok_move, msg_move = self.backend.send_gripper_move(0.0)
        if not ok_move:
            if "그리퍼 비활성화" in str(msg_move):
                return True, str(msg_move)
            return False, f"그리퍼 0mm 닫기 실패: {msg_move}"
        ok_wait, wait_err = self._wait_for_backend_motion_done(timeout_sec=20.0)
        if not ok_wait:
            return False, f"그리퍼 0mm 닫기 대기 실패: {wait_err}"
        return True, str(msg_move)

    def _compute_rigid_xyz_to_xyz(self, src_xyz_arr, dst_xyz_arr):
        if src_xyz_arr is None or dst_xyz_arr is None:
            return None, "소스/타겟 XYZ 데이터가 부족합니다."
        src = np.asarray(src_xyz_arr, dtype=np.float64)
        dst = np.asarray(dst_xyz_arr, dtype=np.float64)
        if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 3 or src.shape[0] < 4:
            return None, "XYZ 포인트 형식이 맞지 않거나 데이터 개수가 부족합니다."
        if not np.isfinite(src).all():
            return None, "비전 XYZ 포인트에 유효하지 않은 값이 있습니다."
        if not np.isfinite(dst).all():
            return None, "로봇 XYZ 포인트에 유효하지 않은 값이 있습니다."

        src_c = np.mean(src, axis=0)
        dst_c = np.mean(dst, axis=0)
        src0 = src - src_c
        dst0 = dst - dst_c
        h = src0.T @ dst0
        u, _, vt = np.linalg.svd(h)
        r_col = vt.T @ u.T
        if np.linalg.det(r_col) < 0:
            vt[-1, :] *= -1.0
            r_col = vt.T @ u.T
        r_row = r_col.T
        t_row = dst_c - (src_c @ r_row)
        pred = (src @ r_row) + t_row
        err = np.linalg.norm(pred - dst, axis=1)
        rmse = float(np.sqrt(np.mean(err ** 2)))
        affine = np.vstack([r_row, t_row.reshape(1, 3)])
        return (affine, r_row, t_row, rmse), None

    def _extract_calib_grid_camera_xyz(self, panel_index: int = 1):
        panel = 2 if int(panel_index) == 2 else 1
        cached_xyz = self._calib_grid_camera_xyz_2 if panel == 2 else self._calib_grid_camera_xyz_1
        if cached_xyz is not None:
            try:
                arr = np.asarray(cached_xyz, dtype=np.float64).reshape(-1, 3)
                if arr.size > 0 and np.isfinite(arr).all():
                    return arr, None
            except Exception:
                pass

        pts_grid = self._calib_last_grid_pts_2 if panel == 2 else self._calib_last_grid_pts_1
        if pts_grid is None:
            return None, "체커보드 그리드 데이터가 없습니다."
        try:
            grid = np.asarray(pts_grid, dtype=np.float64)
        except Exception:
            return None, "체커보드 그리드 데이터 파싱 실패"
        if grid.ndim != 3 or grid.shape[2] < 2 or grid.shape[0] < 2 or grid.shape[1] < 2:
            return None, "체커보드 그리드 형식이 올바르지 않습니다."

        cam_rows = []
        bad = []
        flat = grid.reshape(-1, 2)
        for idx, pt in enumerate(flat):
            u_f = float(pt[0])
            v_f = float(pt[1])
            z_m = self._depth_m_from_image_coord(int(round(u_f)), int(round(v_f)), panel_index=panel)
            if z_m is None or (not np.isfinite(z_m)) or float(z_m) <= 0.0:
                bad.append(f"g{idx + 1}")
                continue
            z_mm = float(z_m) * 1000.0
            cxyz = self._uvz_to_camera_xyz_mm(u_f, v_f, z_mm, require_intrinsics=True, panel_index=panel)
            if cxyz is None:
                bad.append(f"g{idx + 1}")
                continue
            cxyz_arr = np.asarray(cxyz, dtype=np.float64).reshape(3,)
            if not np.isfinite(cxyz_arr).all():
                bad.append(f"g{idx + 1}")
                continue
            cam_rows.append(cxyz_arr)
        total = int(flat.shape[0])
        if len(cam_rows) != total:
            sample = ", ".join(bad[:8])
            if len(bad) > 8:
                sample += ", ..."
            return None, f"체커보드 전체 XYZ 취득 실패: {len(bad)}/{total}개({sample})"
        return np.asarray(cam_rows, dtype=np.float64), None

    def _robust_center_from_grid_xyz(self, grid_xyz):
        pts = np.asarray(grid_xyz, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < 4:
            return None, 0
        if not np.isfinite(pts).all():
            return None, 0
        med = np.median(pts, axis=0)
        dist = np.linalg.norm(pts - med.reshape(1, 3), axis=1)
        mad = np.median(np.abs(dist - np.median(dist)))
        if not np.isfinite(mad) or mad <= 1e-9:
            inlier = pts
        else:
            thr = max(5.0, 3.5 * 1.4826 * mad)
            inlier = pts[dist <= thr]
            if inlier.shape[0] < max(4, int(0.5 * pts.shape[0])):
                inlier = pts
        center = np.mean(inlier, axis=0)
        if not np.isfinite(center).all():
            return None, int(inlier.shape[0])
        return center.astype(np.float64), int(inlier.shape[0])

    def _capture_current_robot_xyz_mm(self):
        if self.backend is None:
            return None, "백엔드 초기화 중입니다."
        last_err = "현재 TCP 좌표를 얻지 못했습니다."
        if hasattr(self.backend, "get_current_posx_live"):
            posx_live, _sol_live, err_live = self.backend.get_current_posx_live()
            if posx_live is not None:
                try:
                    arr = np.asarray(list(posx_live)[:3], dtype=np.float64)
                    if arr.shape == (3,) and np.isfinite(arr).all():
                        return arr, None
                except Exception:
                    pass
            if err_live:
                last_err = err_live

        if hasattr(self.backend, "get_position_snapshot"):
            data, seen_at = self.backend.get_position_snapshot()
            if data and len(data) >= 2 and data[1] is not None:
                if seen_at is None or (time.monotonic() - float(seen_at)) <= max(0.5, POSITION_STALE_SEC):
                    try:
                        arr = np.asarray(list(data[1])[:3], dtype=np.float64)
                        if arr.shape == (3,) and np.isfinite(arr).all():
                            return arr, None
                    except Exception:
                        pass
        return None, last_err

    def _wait_for_backend_motion_done(self, timeout_sec: float = 90.0):
        if self.backend is None:
            return False, "백엔드 초기화 중입니다."
        end_at = time.monotonic() + max(1.0, float(timeout_sec))
        started_busy = False
        idle_since = None
        while time.monotonic() < end_at:
            QApplication.processEvents()
            try:
                busy = bool(self.backend.is_busy())
            except Exception:
                busy = False
            if busy:
                started_busy = True
                idle_since = None
            else:
                if (not started_busy) and ((time.monotonic() + 0.35) < end_at):
                    time.sleep(0.05)
                    continue
                if idle_since is None:
                    idle_since = time.monotonic()
                elif (time.monotonic() - idle_since) >= 0.35:
                    return True, None
            time.sleep(0.05)
        return False, "이동 완료 대기 시간 초과"

    def _wait_for_calibration_center_mm(self, panel_index: int = 1, timeout_sec: float = 6.0, min_seen_at=None):
        panel = 2 if int(panel_index) == 2 else 1
        end_at = time.monotonic() + max(0.5, float(timeout_sec))
        last_center = None
        stable_count = 0
        last_err = "체커보드 전체 데이터 대기중"
        while time.monotonic() < end_at:
            QApplication.processEvents()
            seen_at = self._calib_last_points_at_2 if panel == 2 else self._calib_last_points_at_1
            if min_seen_at is not None:
                if seen_at is None or float(seen_at) < float(min_seen_at):
                    last_err = "새 체커보드 데이터 대기중"
                    time.sleep(0.15)
                    continue
            grid_xyz, err = self._extract_calib_grid_camera_xyz(panel)
            if grid_xyz is not None:
                center, inlier_count = self._robust_center_from_grid_xyz(grid_xyz)
                if center is not None:
                    if last_center is not None and np.linalg.norm(center - last_center) <= 2.5:
                        stable_count += 1
                    else:
                        stable_count = 1
                    last_center = center
                    if stable_count >= 2:
                        return center, int(grid_xyz.shape[0]), int(inlier_count), None
            last_err = err or (
                self._calib_full_reason_2 if panel == 2 else self._calib_full_reason_1
            ) or last_err
            time.sleep(0.15)
        return None, 0, 0, last_err

    def _save_calibration_xyz_file(self, out_path, samples, affine, rmse, panel_index: int = 1):
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        aff = np.asarray(affine, dtype=np.float64)
        if aff.shape != (4, 3):
            raise ValueError("vision affine must be 4x3")
        r_row = aff[:3, :]
        t_row = aff[3, :].reshape(3,)
        with open(out_path, "w", encoding="utf-8") as f:
            f.write("#VisionXYZ (x,y,z_mm)\n")
            for idx, sample in enumerate(samples, start=1):
                sx, sy, sz = np.asarray(sample["vision_xyz"], dtype=np.float64).reshape(3,)
                f.write(f"s{idx:02d}={sx:.6f},{sy:.6f},{sz:.6f}\n")
            f.write("\n#Robot (x,y,z_mm)\n")
            for idx, sample in enumerate(samples, start=1):
                rx, ry, rz = np.asarray(sample["robot_xyz"], dtype=np.float64).reshape(3,)
                f.write(f"s{idx:02d}={rx:.6f},{ry:.6f},{rz:.6f}\n")
            f.write("\n#Sequence (panel,row,label)\n")
            for idx, sample in enumerate(samples, start=1):
                row_key = str(sample.get("row_key", ""))
                label = str(sample.get("label", "")).replace(",", " ")
                f.write(f"s{idx:02d}=panel{int(panel_index)},{row_key},{label}\n")
            f.write("\n#SampleLog (manual check only)\n")
            f.write("#format=sXX,vision_xyz(mm),robot_xyz(mm),pred_robot_xyz(mm),residual(mm)\n")
            for idx, sample in enumerate(samples, start=1):
                src = np.asarray(sample["vision_xyz"], dtype=np.float64).reshape(1, 3)
                dst = np.asarray(sample["robot_xyz"], dtype=np.float64).reshape(1, 3)
                pred = (src @ r_row) + t_row.reshape(1, 3)
                err = float(np.linalg.norm(pred.reshape(3,) - dst.reshape(3,)))
                sv = src.reshape(3,)
                dv = dst.reshape(3,)
                pv = pred.reshape(3,)
                f.write(
                    (
                        f"s{idx:02d},"
                        f"V=({sv[0]:.3f},{sv[1]:.3f},{sv[2]:.3f}),"
                        f"R=({dv[0]:.3f},{dv[1]:.3f},{dv[2]:.3f}),"
                        f"P=({pv[0]:.3f},{pv[1]:.3f},{pv[2]:.3f}),"
                        f"E={err:.3f}\n"
                    )
                )
            f.write("\n#RotationMat (rigid: R(3x3), t(1x3), row-vector form)\n")
            for i in range(3):
                f.write(f"r{i}={r_row[i,0]:.12g},{r_row[i,1]:.12g},{r_row[i,2]:.12g}\n")
            f.write(f"t={t_row[0]:.12g},{t_row[1]:.12g},{t_row[2]:.12g}\n")
            for r in range(4):
                f.write(f"m{r}={aff[r,0]:.12g},{aff[r,1]:.12g},{aff[r,2]:.12g}\n")
            f.write(f"rmse={float(rmse):.6f}\n")

    def _load_affine_from_file(self, file_path):
        sections = self._parse_point_file_sections(file_path)
        rot = sections.get("rotationmat", {})
        if all((f"r{i}" in rot) for i in range(3)) and ("t" in rot):
            try:
                r_row = np.asarray([rot["r0"][:3], rot["r1"][:3], rot["r2"][:3]], dtype=np.float64)
                t_row = np.asarray(rot["t"][:3], dtype=np.float64).reshape(1, 3)
                mat = np.vstack([r_row, t_row])
                rmse = None
                if "rmse" in rot and len(rot["rmse"]) > 0:
                    try:
                        rmse = float(rot["rmse"][0])
                    except Exception:
                        rmse = None
                if rmse is None:
                    base = sections.get("default", {})
                    if "rmse" in base and len(base["rmse"]) > 0:
                        try:
                            rmse = float(base["rmse"][0])
                        except Exception:
                            rmse = None
                return mat, rmse
            except Exception:
                pass
        rows = []
        for i in range(4):
            k = f"m{i}"
            if k in rot:
                rows.append(rot[k][:3])
        if len(rows) == 4:
            mat = np.asarray(rows, dtype=np.float64)
            rmse = None
            if "rmse" in rot and len(rot["rmse"]) > 0:
                try:
                    rmse = float(rot["rmse"][0])
                except Exception:
                    rmse = None
            if rmse is None:
                base = sections.get("default", {})
                if "rmse" in base and len(base["rmse"]) > 0:
                    try:
                        rmse = float(base["rmse"][0])
                    except Exception:
                        rmse = None
            return mat, rmse
        return None, None

    def _save_active_calibration_path(self, src_path, panel_index: int = 1):
        try:
            if not src_path:
                return
            resolved_path = _resolve_repo_file(src_path, fallback_dirs=(CALIB_DIR, CALIB_ROTMAT_DIR))
            stored_path = _project_relative_path(resolved_path or src_path)
            rows = self._load_parameter_rows()
            panel = 2 if int(panel_index) == 2 else 1
            if panel == 2:
                rows["active_calibration_path_2"] = [stored_path]
            else:
                rows["active_calibration_path_1"] = [stored_path]
                # backward compatibility
                rows["active_calibration_path"] = [stored_path]
            self._save_parameter_rows(rows)
            if os.path.isfile(LEGACY_CALIB_ACTIVE_PATH_FILE):
                try:
                    os.remove(LEGACY_CALIB_ACTIVE_PATH_FILE)
                except Exception:
                    pass
        except Exception:
            return

    def on_calibration_load_matrix(self):
        sender = self.sender()
        panel = 2 if sender is getattr(self, "_calibration_load_button_2", None) else 1
        start_dir = CALIB_ROTMAT_DIR if os.path.isdir(CALIB_ROTMAT_DIR) else CALIB_DIR
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            f"캘리브레이션 행렬 불러오기 (비전{panel})",
            start_dir,
            "Text Files (*.txt);;All Files (*)",
        )
        if not file_path:
            return
        mat, rmse = self._load_affine_from_file(file_path)
        if mat is None:
            self.append_log(f"[캘리브레이션{panel}] 행렬 불러오기 실패: {file_path}\n")
            QMessageBox.warning(self, "행렬 불러오기", "선택한 파일에서 변환행렬을 읽지 못했습니다.")
            return
        self._apply_vision_to_robot_affine(mat, rmse=rmse, path=file_path, panel_index=panel)
        self.append_log(f"[캘리브레이션{panel}] 행렬 불러오기 성공: {file_path}\n")

    def on_calibration_transform(self):
        sender = self.sender()
        panel = 2 if sender is getattr(self, "_calibration_transform_button_2", None) else 1
        if self.backend is None:
            self.append_log(f"[캘리브레이션{panel}] 백엔드 초기화 중입니다.\n")
            return
        if not hasattr(self.backend, "send_move_cartesian") or not hasattr(self.backend, "send_move_home"):
            self.append_log(f"[캘리브레이션{panel}] 백엔드가 캘리브레이션 이동 기능을 지원하지 않습니다.\n")
            QMessageBox.warning(self, "캘리브레이션", "백엔드가 캘리브레이션 이동 기능을 지원하지 않습니다.")
            return
        vision_enabled = bool(self._top_status_enabled.get("vision2" if panel == 2 else "vision", True))
        if not vision_enabled:
            self.append_log(f"[캘리브레이션{panel}] 비전 패널이 비활성화되어 실행할 수 없습니다.\n")
            QMessageBox.warning(self, "캘리브레이션", f"비전{panel} 패널이 비활성화 상태입니다.")
            return

        seq_rows = self._load_calibration_sequence_rows(panel_index=panel)

        dialog = QDialog(self)
        dialog.setWindowTitle(f"캘리브레이션 실행 (비전{panel})")
        dialog.setModal(True)
        dialog.setWindowFlags(Qt.Window | Qt.WindowTitleHint | Qt.WindowSystemMenuHint | Qt.WindowCloseButtonHint)
        dialog.resize(1160, 660)

        layout = QVBoxLayout(dialog)
        layout.setContentsMargins(16, 16, 16, 14)
        layout.setSpacing(10)

        intro = QLabel(
            (
                f"비전{panel} 캘리브레이션 준비 화면입니다.\n"
                "여기서 각 위치의 사용 여부를 고르고, 개별 이동/티칭을 수행한 뒤, 아래 시작 버튼을 눌러야만 실제 시퀀스가 실행됩니다.\n"
                "목표 위치 칸을 누르면 값을 직접 수정할 수 있습니다.\n"
                "현재 TCP가 tool_checkerboard가 아니면 필요할 때만 메뉴얼모드에서 TCP를 적용한 뒤 진행합니다."
            ),
            dialog,
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)

        status_label = QLabel("대기중: 행별 사용 여부, 목표위치, 티칭 상태를 확인한 뒤 시작을 누르세요.", dialog)
        status_label.setStyleSheet("font-weight: 700; color: #1f1f1f;")
        status_label.setWordWrap(True)
        layout.addWidget(status_label)

        tcp_status_label = QLabel("현재 TCP 확인중...", dialog)
        tcp_status_label.setStyleSheet("font-size: 9pt; color: #666666;")
        layout.addWidget(tcp_status_label)

        progress_bar = QProgressBar(dialog)
        progress_bar.setRange(0, max(1, len(seq_rows)))
        progress_bar.setValue(0)
        layout.addWidget(progress_bar)

        table = QTableWidget(len(seq_rows), 6, dialog)
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.setSelectionMode(QTableWidget.NoSelection)
        table.verticalHeader().setVisible(False)
        table.verticalHeader().setDefaultSectionSize(max(24, int(UI_PANEL_TABLE_ROW_HEIGHT)))
        table.setHorizontalHeaderLabels(["단계", "사용", "목표 위치", "작업", "수집 좌표(V/R/C)", "상태"])
        header = table.horizontalHeader()
        if header is not None:
            header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
            header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
            header.setSectionResizeMode(2, QHeaderView.Stretch)
            header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
            header.setSectionResizeMode(4, QHeaderView.Stretch)
            header.setSectionResizeMode(5, QHeaderView.ResizeToContents)
        dialog_rows = []
        layout.addWidget(table, 1)

        action_row = QHBoxLayout()
        action_row.setSpacing(8)
        start_button = QPushButton("시작", dialog)
        start_button.setMinimumWidth(110)
        close_button = QPushButton("닫기", dialog)
        close_button.setMinimumWidth(90)
        close_button.clicked.connect(dialog.accept)
        action_row.addStretch(1)
        action_row.addWidget(start_button)
        action_row.addWidget(close_button)
        layout.addLayout(action_row)

        run_state = {"running": False}

        def _set_status(text: str):
            status_label.setText(str(text))
            QApplication.processEvents()

        def _set_row_state(row_state, status_text: str, vision_xyz=None, robot_xyz=None, predicted_robot_xyz=None):
            if row_state["vision_item"] is not None and any(v is not None for v in (vision_xyz, robot_xyz, predicted_robot_xyz)):
                row_state["vision_item"].setText(
                    self._format_calibration_measurement_summary(
                        vision_xyz=vision_xyz,
                        robot_xyz=robot_xyz,
                        predicted_robot_xyz=predicted_robot_xyz,
                    )
                )
            if row_state["status_item"] is not None:
                row_state["status_item"].setText(str(status_text))
            QApplication.processEvents()

        def _active_rows():
            return [row_state for row_state in dialog_rows if row_state["check_box"].isChecked()]

        def _active_capture_rows():
            return [row_state for row_state in _active_rows() if bool(row_state["data"].get("capture"))]

        def _refresh_tcp_status(force_sync: bool = False):
            tcp_ready, tcp_display, _tcp_seen_at = self._is_checkerboard_tcp_ready(force_sync=force_sync)
            if tcp_ready:
                tcp_status_label.setText(f"시작 조건 TCP: {tcp_display} (OK)")
                tcp_status_label.setStyleSheet("font-size: 9pt; color: #2e7d32; font-weight: 700;")
            else:
                tcp_status_label.setText(
                    f"시작 조건 TCP: {tcp_display} (tool_checkerboard 필요, 툴 변경에서 먼저 적용)"
                )
                tcp_status_label.setStyleSheet("font-size: 9pt; color: #c62828; font-weight: 700;")
            return tcp_ready

        def _refresh_start_button():
            active_capture_count = len(_active_capture_rows())
            tcp_ready = _refresh_tcp_status(force_sync=False)
            start_button.setEnabled((not run_state["running"]) and active_capture_count >= 4 and tcp_ready)

        def _refresh_row_controls(row_state):
            seq_row = row_state["data"]
            has_target = bool(
                seq_row.get("move_kind") == "home"
                or seq_row.get("target_pose6") is not None
                or seq_row.get("target_joint") is not None
            )
            row_state["check_box"].setEnabled(not run_state["running"])
            row_state["teach_button"].setEnabled(not run_state["running"])
            row_state["move_button"].setEnabled((not run_state["running"]) and has_target)

        def _refresh_all_row_controls():
            for row_state in dialog_rows:
                _refresh_row_controls(row_state)
            _refresh_start_button()

        def _confirm_real_motion(title: str, row_label: str, target_text: str):
            msg = QMessageBox(self)
            msg.setWindowTitle(title)
            msg.setIcon(QMessageBox.Warning)
            msg.setText(
                (
                    f"{row_label} 위치로 실제 이동합니다.\n\n"
                    f"목표 위치: {target_text}\n\n"
                    "현재 TCP가 tool_checkerboard가 아니면 필요 시 메뉴얼모드에서 TCP를 적용한 뒤 진행합니다.\n"
                    "계속할까요?"
                )
            )
            msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
            msg.setDefaultButton(QMessageBox.No)
            return msg.exec_() == QMessageBox.Yes

        def _confirm_manual_row_motion_second(row_label: str):
            answer = QMessageBox.question(
                dialog,
                "캘리브레이션 위치 이동 재확인",
                f"{row_label} 위치로 실제 이동합니다.\n한 번 더 확인합니다. 계속할까요?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            return answer == QMessageBox.Yes

        def _values_changed(old_values, new_values, tol: float = 1e-6):
            old_vals = self._parse_float_values(old_values, expected_len=6)
            new_vals = self._parse_float_values(new_values, expected_len=6)
            if old_vals is None or new_vals is None:
                return True
            return any(abs(float(a) - float(b)) > float(tol) for a, b in zip(old_vals[:6], new_vals[:6]))

        def _save_row_enabled(row_state, checked: bool):
            ok_save, msg_save = self._save_calibration_sequence_row_config(
                panel,
                row_state["data"]["key"],
                enabled=checked,
            )
            if not ok_save:
                row_state["check_box"].blockSignals(True)
                row_state["check_box"].setChecked(not bool(checked))
                row_state["check_box"].blockSignals(False)
                QMessageBox.warning(dialog, "캘리브레이션", msg_save, QMessageBox.Ok)
                return
            row_state["data"]["enabled"] = bool(checked)
            _set_row_state(row_state, "대기" if checked else "미사용")
            self.append_log(
                f"[캘리브레이션{panel}] {row_state['data']['label']} 사용여부 저장: {'ON' if checked else 'OFF'}\n"
            )
            _refresh_start_button()

        def _move_row(row_state):
            seq_row = row_state["data"]
            row_label = str(seq_row.get("label", seq_row.get("key", "-")))
            target_text = str(seq_row.get("target_text", "-"))
            if run_state["running"]:
                return
            tcp_ready, tcp_display, _tcp_seen_at = self._is_checkerboard_tcp_ready(force_sync=True)
            if not tcp_ready:
                QMessageBox.warning(
                    dialog,
                    "캘리브레이션",
                    f"현재 TCP가 tool_checkerboard가 아닙니다.\n현재: {tcp_display}",
                    QMessageBox.Ok,
                )
                _refresh_start_button()
                return
            if not _confirm_real_motion("캘리브레이션 위치 이동", row_label, target_text):
                return
            if not _confirm_manual_row_motion_second(row_label):
                return
            run_state["running"] = True
            close_button.setEnabled(False)
            _refresh_all_row_controls()
            previous_mode = None
            restore_enabled = bool(self.backend is not None and self.backend.is_ready())
            self._calibration_sequence_running = True
            self._set_robot_controls_enabled(restore_enabled)
            try:
                ok_ready, msg_ready, previous_mode = self._prepare_calibration_motion_ready(
                    parent=dialog,
                    status_callback=_set_status,
                )
                if not ok_ready:
                    raise RuntimeError(msg_ready)
                _set_status(f"{row_label} 이동중...")
                _set_row_state(row_state, "이동중")
                move_speed = self._calibration_motion_speed_for_command()
                if seq_row.get("move_kind") == "home":
                    ok_move, move_msg = self.backend.send_move_home(vel=move_speed, acc=move_speed)
                elif seq_row.get("move_kind") == "joint":
                    target_joint = seq_row.get("target_joint")
                    if target_joint is None:
                        raise RuntimeError(f"{row_label} 조인트 위치가 없습니다.")
                    ok_move, move_msg = self.backend.send_move_joint(*target_joint, vel=move_speed, acc=move_speed)
                else:
                    target_pose6 = seq_row.get("target_pose6")
                    if target_pose6 is None:
                        raise RuntimeError(f"{row_label} 좌표 위치가 없습니다.")
                    ok_move, move_msg = self.backend.send_move_cartesian(*target_pose6, vel=move_speed, acc=move_speed)
                if not ok_move:
                    raise RuntimeError(move_msg)
                ok_wait, wait_err = self._wait_for_backend_motion_done(timeout_sec=90.0)
                if not ok_wait:
                    raise RuntimeError(wait_err)
                if float(CALIB_SEQUENCE_SETTLE_SEC) > 0.0:
                    _set_row_state(row_state, "정지 안정화")
                    self._wait_with_ui_pump(
                        float(CALIB_SEQUENCE_SETTLE_SEC),
                        status_callback=_set_status,
                        message_prefix=f"{row_label} 정지 안정화 중",
                    )
                if bool(seq_row.get("capture")):
                    _set_row_state(row_state, "비전 데이터 수집중")
                    if row_state["vision_item"] is not None:
                        row_state["vision_item"].setText("수집중...")
                    fresh_after = time.monotonic()
                    center_xyz, corner_count, inlier_count, center_err = self._wait_for_calibration_center_mm(
                        panel_index=panel,
                        timeout_sec=float(CALIB_SEQUENCE_CAPTURE_TIMEOUT_SEC),
                        min_seen_at=fresh_after,
                    )
                    if center_xyz is None:
                        raise RuntimeError(f"비전 데이터 수집 실패: {center_err}")
                    predicted_xyz = self._predict_robot_xyz_from_vision_xyz(center_xyz, panel_index=panel)
                    _set_row_state(
                        row_state,
                        f"이동완료 ({int(corner_count)}pt/{int(inlier_count)}in)",
                        vision_xyz=center_xyz,
                        predicted_robot_xyz=predicted_xyz,
                    )
                else:
                    _set_row_state(row_state, "이동완료")
                _set_status(f"{row_label} 이동 완료")
                self.append_log(f"[캘리브레이션{panel}] {row_label} 개별 이동 완료\n")
            except Exception as e:
                _set_row_state(row_state, f"실패: {e}")
                _set_status(f"실패: {e}")
                self.append_log(f"[캘리브레이션{panel}] {row_label} 개별 이동 실패: {e}\n")
                QMessageBox.warning(dialog, "캘리브레이션", str(e), QMessageBox.Ok)
            finally:
                if previous_mode is not None and int(previous_mode) != 1:
                    ok_restore, msg_restore = self._restore_robot_mode(previous_mode)
                    self.append_log(
                        f"[캘리브레이션{panel}] 개별 이동 후 모드 복원({'성공' if ok_restore else '실패'}): {msg_restore}\n"
                    )
                self._calibration_sequence_running = False
                self._set_robot_controls_enabled(restore_enabled)
                run_state["running"] = False
                close_button.setEnabled(True)
                _refresh_all_row_controls()

        def _teach_row(row_state):
            if run_state["running"]:
                return
            row_label = str(row_state["data"].get("label", row_state["data"].get("key", "-")))
            posj, posx, err = self._capture_current_teach_pose()
            if err is not None:
                QMessageBox.warning(dialog, "캘리브레이션", err, QMessageBox.Ok)
                return
            ok_save, msg_save = self._save_calibration_sequence_row_config(
                panel,
                row_state["data"]["key"],
                enabled=row_state["check_box"].isChecked(),
                joints=posj,
                pose6=posx,
            )
            if not ok_save:
                QMessageBox.warning(dialog, "캘리브레이션", msg_save, QMessageBox.Ok)
                return
            if row_state["data"]["key"] in ("initial", "end"):
                row_state["data"]["move_kind"] = "home"
                row_state["data"]["target_joint"] = list(posj)
                row_state["data"]["target_pose6"] = None
                row_state["data"]["target_text"] = self._format_joint_summary(posj)
            else:
                row_state["data"]["move_kind"] = "pose6"
                row_state["data"]["target_joint"] = list(posj)
                row_state["data"]["target_pose6"] = list(posx)
                row_state["data"]["target_text"] = self._format_pose6_summary(posx)
            row_state["target_item"].setText(str(row_state["data"]["target_text"]))
            _set_row_state(row_state, "티칭 저장")
            self.append_log(f"[캘리브레이션{panel}] {row_label} 티칭 저장: {msg_save}\n")
            _refresh_row_controls(row_state)
            _refresh_start_button()

        def _edit_row_target(row_state):
            if run_state["running"]:
                return
            seq_row = row_state["data"]
            row_label = str(seq_row.get("label", seq_row.get("key", "-")))
            move_kind = str(seq_row.get("move_kind", "pose6") or "pose6").strip().lower()
            current_posj, current_posx = self._get_current_pose_defaults()
            if move_kind in ("home", "joint"):
                defaults = self._parse_float_values(seq_row.get("target_joint"), expected_len=6)
                if defaults is None:
                    defaults = current_posj
                vals = self._ask_six_values_form(
                    f"{row_label} 목표 위치 수정",
                    ["J1", "J2", "J3", "J4", "J5", "J6"],
                    defaults,
                    limits=JOINT_INPUT_LIMITS_DEG,
                    guide_text="안내: 이 값은 캘리브레이션 시퀀스 목표 위치로 저장됩니다.",
                )
                if vals is None:
                    return
                if isinstance(vals, str):
                    QMessageBox.warning(dialog, "캘리브레이션", vals, QMessageBox.Ok)
                    return
                if not _values_changed(seq_row.get("target_joint"), vals):
                    _set_row_state(row_state, "변경 없음")
                    self.append_log(f"[캘리브레이션{panel}] {row_label} 목표 위치 변경 없음\n")
                    return
                ok_save, msg_save = self._save_calibration_sequence_row_config(
                    panel,
                    seq_row["key"],
                    enabled=row_state["check_box"].isChecked(),
                    joints=vals,
                )
                if not ok_save:
                    QMessageBox.warning(dialog, "캘리브레이션", msg_save, QMessageBox.Ok)
                    return
                if seq_row["key"] in ("initial", "end"):
                    seq_row["move_kind"] = "home"
                    seq_row["target_pose6"] = None
                else:
                    seq_row["move_kind"] = "joint"
                seq_row["target_joint"] = list(vals)
                seq_row["target_text"] = self._format_joint_summary(vals)
            else:
                defaults = self._parse_float_values(seq_row.get("target_pose6"), expected_len=6)
                if defaults is None:
                    defaults = current_posx
                vals = self._ask_six_values_form(
                    f"{row_label} 목표 위치 수정",
                    ["X", "Y", "Z", "A", "B", "C"],
                    defaults,
                    guide_text="안내: 이 값은 캘리브레이션 시퀀스 목표 위치로 저장됩니다.",
                )
                if vals is None:
                    return
                if isinstance(vals, str):
                    QMessageBox.warning(dialog, "캘리브레이션", vals, QMessageBox.Ok)
                    return
                if not _values_changed(seq_row.get("target_pose6"), vals):
                    _set_row_state(row_state, "변경 없음")
                    self.append_log(f"[캘리브레이션{panel}] {row_label} 목표 위치 변경 없음\n")
                    return
                ok_save, msg_save = self._save_calibration_sequence_row_config(
                    panel,
                    seq_row["key"],
                    enabled=row_state["check_box"].isChecked(),
                    pose6=vals,
                )
                if not ok_save:
                    QMessageBox.warning(dialog, "캘리브레이션", msg_save, QMessageBox.Ok)
                    return
                seq_row["move_kind"] = "pose6"
                seq_row["target_pose6"] = list(vals)
                seq_row["target_text"] = self._format_pose6_summary(vals)

            row_state["target_item"].setText(str(seq_row["target_text"]))
            _set_row_state(row_state, "목표값 저장")
            self.append_log(f"[캘리브레이션{panel}] {row_label} 목표 위치 직접 수정: {msg_save}\n")
            _refresh_row_controls(row_state)
            _refresh_start_button()

        for row_index, seq_row in enumerate(seq_rows):
            label_item = QTableWidgetItem(str(seq_row.get("label", "-")))
            target_item = QTableWidgetItem(str(seq_row.get("target_text", "-")))
            target_item.setToolTip("클릭하여 목표 위치 값을 직접 수정")
            vision_item = QTableWidgetItem("-")
            status_item = QTableWidgetItem("대기" if bool(seq_row.get("enabled")) else "미사용")
            table.setItem(row_index, 0, label_item)
            table.setItem(row_index, 2, target_item)
            table.setItem(row_index, 4, vision_item)
            table.setItem(row_index, 5, status_item)

            check_wrap = QFrame(table)
            check_layout = QHBoxLayout(check_wrap)
            check_layout.setContentsMargins(0, 0, 0, 0)
            check_layout.setAlignment(Qt.AlignCenter)
            check_box = QCheckBox(check_wrap)
            check_box.setChecked(bool(seq_row.get("enabled")))
            check_layout.addWidget(check_box)
            table.setCellWidget(row_index, 1, check_wrap)

            button_wrap = QFrame(table)
            button_layout = QHBoxLayout(button_wrap)
            button_layout.setContentsMargins(4, 0, 4, 0)
            button_layout.setSpacing(6)
            move_button = QPushButton("이동", button_wrap)
            move_button.setMinimumWidth(54)
            teach_button = QPushButton("티칭", button_wrap)
            teach_button.setMinimumWidth(60)
            button_layout.addWidget(move_button)
            button_layout.addWidget(teach_button)
            table.setCellWidget(row_index, 3, button_wrap)

            row_state = {
                "row_index": row_index,
                "data": seq_row,
                "check_box": check_box,
                "move_button": move_button,
                "teach_button": teach_button,
                "target_item": target_item,
                "vision_item": vision_item,
                "status_item": status_item,
            }
            check_box.toggled.connect(lambda checked, state=row_state: _save_row_enabled(state, checked))
            move_button.clicked.connect(lambda _checked=False, state=row_state: _move_row(state))
            teach_button.clicked.connect(lambda _checked=False, state=row_state: _teach_row(state))
            dialog_rows.append(row_state)
            _refresh_row_controls(row_state)

        def _on_table_cell_clicked(row_index, column_index):
            if int(column_index) != 2:
                return
            if row_index < 0 or row_index >= len(dialog_rows):
                return
            _edit_row_target(dialog_rows[row_index])

        table.cellClicked.connect(_on_table_cell_clicked)

        def _run_sequence():
            if run_state["running"]:
                return
            active_rows = _active_rows()
            active_capture_rows = [row_state for row_state in active_rows if bool(row_state["data"].get("capture"))]
            if len(active_capture_rows) < 4:
                QMessageBox.warning(dialog, "캘리브레이션", "사용 체크된 데이터 포인트가 4개 이상 필요합니다.", QMessageBox.Ok)
                return
            tcp_ready, tcp_display, _tcp_seen_at = self._is_checkerboard_tcp_ready(force_sync=True)
            if not tcp_ready:
                _refresh_start_button()
                QMessageBox.warning(
                    dialog,
                    "캘리브레이션 시작",
                    f"시작 조건 미충족: 현재 TCP가 tool_checkerboard가 아닙니다.\n현재: {tcp_display}",
                    QMessageBox.Ok,
                )
                return
            answer = QMessageBox.question(
                dialog,
                "캘리브레이션 시작",
                (
                    f"선택된 {len(active_rows)}개 단계를 실제로 실행합니다.\n"
                    f"이 중 데이터 수집 포인트는 {len(active_capture_rows)}개입니다.\n\n"
                    "현재 TCP가 tool_checkerboard가 아니면 필요 시 메뉴얼모드에서 TCP를 적용한 뒤 진행합니다.\n"
                    "시작할까요?"
                ),
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return

            run_state["running"] = True
            close_button.setEnabled(False)
            _refresh_all_row_controls()

            restore_enabled = bool(self.backend is not None and self.backend.is_ready())
            self._calibration_sequence_running = True
            self._set_robot_controls_enabled(restore_enabled)

            previous_mode = None
            try:
                progress_bar.setRange(0, max(1, len(active_rows)))
                progress_bar.setValue(0)
                _set_status("캘리브레이션 모드 준비중...")
                enabled_attr = "_calibration_mode_enabled_2" if panel == 2 else "_calibration_mode_enabled_1"
                if not bool(getattr(self, enabled_attr, False)):
                    switch = self._calibration_mode_switch_2 if panel == 2 else self._calibration_mode_switch
                    if switch is None:
                        raise RuntimeError("캘리브레이션 모드 스위치를 찾지 못했습니다.")
                    switch.setChecked(True)
                    deadline = time.monotonic() + 2.0
                    while time.monotonic() < deadline:
                        QApplication.processEvents()
                        if bool(getattr(self, enabled_attr, False)):
                            break
                        time.sleep(0.05)
                    if not bool(getattr(self, enabled_attr, False)):
                        raise RuntimeError("캘리브레이션 모드를 활성화하지 못했습니다.")
                self._ensure_calibration_process(panel)

                ok_ready, msg_ready, previous_mode = self._prepare_calibration_motion_ready(
                    parent=dialog,
                    status_callback=_set_status,
                )
                if not ok_ready:
                    raise RuntimeError(msg_ready)
                self.append_log(f"[캘리브레이션{panel}] 준비 완료: {msg_ready}\n")
                ok_gripper, msg_gripper = self._prepare_calibration_gripper_closed(status_callback=_set_status)
                if not ok_gripper:
                    raise RuntimeError(msg_gripper)
                self.append_log(f"[캘리브레이션{panel}] 시작 전 그리퍼 준비: {msg_gripper}\n")

                samples = []
                completed_steps = 0
                move_speed = self._calibration_motion_speed_for_command()
                for active_index, row_state in enumerate(active_rows, start=1):
                    seq_row = row_state["data"]
                    row_label = str(seq_row.get("label", seq_row.get("key", "-")))
                    _set_status(f"{row_label} 이동중...")
                    _set_row_state(row_state, "이동중")

                    if seq_row.get("move_kind") == "home":
                        ok_move, move_msg = self.backend.send_move_home(vel=move_speed, acc=move_speed)
                    elif seq_row.get("move_kind") == "joint":
                        target_joint = seq_row.get("target_joint")
                        if target_joint is None:
                            raise RuntimeError(f"{row_label} 조인트 위치가 없습니다.")
                        ok_move, move_msg = self.backend.send_move_joint(*target_joint, vel=move_speed, acc=move_speed)
                    else:
                        target_pose6 = seq_row.get("target_pose6")
                        if target_pose6 is None:
                            raise RuntimeError(f"{row_label} 좌표 위치가 없습니다.")
                        ok_move, move_msg = self.backend.send_move_cartesian(*target_pose6, vel=move_speed, acc=move_speed)
                    if not ok_move:
                        raise RuntimeError(f"{row_label} 이동 실패: {move_msg}")
                    ok_wait, wait_err = self._wait_for_backend_motion_done(timeout_sec=90.0)
                    if not ok_wait:
                        raise RuntimeError(f"{row_label} {wait_err}")

                    completed_steps += 1
                    progress_bar.setValue(completed_steps)
                    QApplication.processEvents()

                    if float(CALIB_SEQUENCE_SETTLE_SEC) > 0.0:
                        _set_row_state(row_state, "정지 안정화")
                        self._wait_with_ui_pump(
                            float(CALIB_SEQUENCE_SETTLE_SEC),
                            status_callback=_set_status,
                            message_prefix=f"{row_label} 정지 안정화 중",
                        )

                    if not bool(seq_row.get("capture")):
                        _set_row_state(row_state, "완료")
                    else:
                        _set_status(f"{row_label} 데이터 수집중...")
                        _set_row_state(row_state, "비전 데이터 수집중")
                        if row_state["vision_item"] is not None:
                            row_state["vision_item"].setText("수집중...")
                        fresh_after = time.monotonic()
                        center_xyz, corner_count, inlier_count, center_err = self._wait_for_calibration_center_mm(
                            panel_index=panel,
                            timeout_sec=float(CALIB_SEQUENCE_CAPTURE_TIMEOUT_SEC),
                            min_seen_at=fresh_after,
                        )
                        if center_xyz is None:
                            raise RuntimeError(f"{row_label} 비전 데이터 수집 실패: {center_err}")
                        robot_xyz, robot_err = self._capture_current_robot_xyz_mm()
                        if robot_xyz is None:
                            raise RuntimeError(f"{row_label} 로봇 TCP 수집 실패: {robot_err}")
                        predicted_xyz = self._predict_robot_xyz_from_vision_xyz(center_xyz, panel_index=panel)

                        samples.append(
                            {
                                "row_key": str(seq_row.get("key", "")),
                                "label": row_label,
                                "vision_xyz": np.asarray(center_xyz, dtype=np.float64).reshape(3,),
                                "robot_xyz": np.asarray(robot_xyz, dtype=np.float64).reshape(3,),
                            }
                        )
                        _set_row_state(
                            row_state,
                            f"수집완료 ({int(corner_count)}pt/{int(inlier_count)}in)",
                            vision_xyz=center_xyz,
                            robot_xyz=robot_xyz,
                            predicted_robot_xyz=predicted_xyz,
                        )
                        self.append_log(
                            (
                                f"[캘리브레이션{panel}] {row_label} 수집: "
                                f"vision={self._format_xyz_summary(center_xyz)}, "
                                f"robot={self._format_xyz_summary(robot_xyz)}\n"
                            )
                        )

                    if active_index < len(active_rows) and float(CALIB_SEQUENCE_NEXT_DELAY_SEC) > 0.0:
                        self._wait_with_ui_pump(
                            float(CALIB_SEQUENCE_NEXT_DELAY_SEC),
                            status_callback=_set_status,
                            message_prefix=f"{row_label} 완료, 다음 이동 대기",
                        )

                if len(samples) < 4:
                    raise RuntimeError("유효 샘플이 4개 미만입니다.")

                src_xyz = np.asarray([sample["vision_xyz"] for sample in samples], dtype=np.float64)
                dst_xyz = np.asarray([sample["robot_xyz"] for sample in samples], dtype=np.float64)
                result, err = self._compute_rigid_xyz_to_xyz(src_xyz, dst_xyz)
                if result is None:
                    raise RuntimeError(str(err))
                affine, _r_row, _t_row, rmse = result

                timestamp = time.strftime("%y%m%d_%H%M%S", time.localtime())
                out_path = os.path.join(CALIB_ROTMAT_DIR, f"calib_matrix_{timestamp}.txt")
                if os.path.exists(out_path):
                    out_path = os.path.join(CALIB_ROTMAT_DIR, f"calib_matrix_{timestamp}_panel{panel}.txt")
                self._save_calibration_xyz_file(out_path, samples, affine, rmse, panel_index=panel)
                self._apply_vision_to_robot_affine(affine, rmse=rmse, path=out_path, panel_index=panel)
                self.append_log(f"[캘리브레이션{panel}] 행렬 계산 완료 (RMSE={rmse:.3f}mm): {out_path}\n")
                _set_status(f"완료: RMSE={rmse:.3f}mm, 저장={os.path.basename(out_path)}")
                QMessageBox.information(
                    dialog,
                    "캘리브레이션",
                    (
                        f"비전{panel} 캘리브레이션이 완료되었습니다.\n\n"
                        f"샘플 수: {len(samples)}\n"
                        f"RMSE: {rmse:.3f} mm\n"
                        f"저장 파일: {out_path}"
                    ),
                    QMessageBox.Ok,
                )
            except Exception as e:
                self.append_log(f"[캘리브레이션{panel}] 실행 실패: {e}\n")
                _set_status(f"실패: {e}")
                QMessageBox.warning(dialog, "캘리브레이션", str(e), QMessageBox.Ok)
            finally:
                if previous_mode is not None and int(previous_mode) != 1:
                    ok_restore, msg_restore = self._restore_robot_mode(previous_mode)
                    self.append_log(
                        f"[캘리브레이션{panel}] 시퀀스 종료 후 모드 복원({'성공' if ok_restore else '실패'}): {msg_restore}\n"
                    )
                self._calibration_sequence_running = False
                self._set_robot_controls_enabled(restore_enabled)
                run_state["running"] = False
                close_button.setEnabled(True)
                _refresh_all_row_controls()

        start_button.clicked.connect(_run_sequence)
        self._request_current_tool_sync(retry_window_sec=6.0, immediate=True)
        tcp_timer = QTimer(dialog)
        tcp_timer.timeout.connect(_refresh_start_button)
        tcp_timer.start(700)
        _refresh_all_row_controls()
        dialog.exec_()

    def _load_parameter_rows(self):
        rows = {}
        if not os.path.isfile(PARAM_FILE):
            return rows
        try:
            with open(PARAM_FILE, "r", encoding="utf-8", newline="") as f:
                for row in csv.reader(f):
                    if not row:
                        continue
                    key = str(row[0]).strip()
                    if not key or key.lower() == "name":
                        continue
                    rows[key] = [str(v).strip() for v in row[1:] if str(v).strip() != ""]
        except Exception:
            return {}
        return rows

    def _save_parameter_rows(self, rows):
        try:
            os.makedirs(PARAM_DIR, exist_ok=True)
            with open(PARAM_FILE, "w", encoding="utf-8", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["name", "v1", "v2", "v3", "v4", "v5", "v6"])
                for key in sorted(rows.keys()):
                    vals = rows.get(key, [])
                    if not isinstance(vals, (list, tuple)):
                        vals = [str(vals)]
                    writer.writerow([key, *list(vals)])
        except Exception:
            return

    def _normalize_serial_text(self, value):
        s = str(value or "").strip()
        if not s:
            return ""
        if s.startswith("_"):
            s = s[1:]
        return s

    def _discover_connected_camera_serials(self):
        serials = []
        rs_cmd_candidates = [
            ["rs-enumerate-devices", "-s"],
            ["/opt/ros/jazzy/bin/rs-enumerate-devices", "-s"],
        ]
        for cmd in rs_cmd_candidates:
            try:
                env = dict(os.environ)
                prev_ld = str(env.get("LD_LIBRARY_PATH", "") or "")
                extra_ld = ["/opt/ros/jazzy/lib/x86_64-linux-gnu", "/opt/ros/jazzy/lib"]
                merged_ld = ":".join([p for p in extra_ld + [prev_ld] if p])
                if merged_ld:
                    env["LD_LIBRARY_PATH"] = merged_ld
                proc = subprocess.run(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=2.5,
                    check=False,
                    env=env,
                )
            except Exception:
                continue
            # rs-enumerate-devices의 표준 출력(장치 목록)만 우선 파싱한다.
            # stderr에는 내부 로그/스레드 ID 숫자가 섞여 오탐이 발생할 수 있다.
            text = str(proc.stdout or "")
            if not text.strip():
                text = (proc.stdout or "") + "\n" + (proc.stderr or "")
            for line in str(text).splitlines():
                low = line.lower()
                # 1) "Serial Number: xxxxx" 형식
                if "serial number" in low and ":" in line:
                    raw = line.split(":", 1)[1].strip()
                    serial = self._normalize_serial_text(raw)
                    if serial.isdigit() and 10 <= len(serial) <= 14 and serial not in serials:
                        serials.append(serial)
                    continue
                # 2) 테이블 형식: "Intel RealSense D455F  252222300723  5.17.0.10"
                m = re.search(
                    r"Intel\s+RealSense.*?\s(?P<sn>\d{10,14})\s+\d+\.\d+\.\d+\.\d+",
                    line,
                    flags=re.IGNORECASE,
                )
                if m:
                    serial = self._normalize_serial_text(m.group("sn"))
                    if serial and serial not in serials:
                        serials.append(serial)
            if serials:
                break
        if serials:
            return serials

        # Fallback 1) /dev/v4l/by-id 링크명에서 RealSense 시리얼 추출
        try:
            for path in sorted(glob.glob("/dev/v4l/by-id/*")):
                name = os.path.basename(path)
                low = name.lower()
                if ("realsense" not in low) and ("intel" not in low):
                    continue
                head = name.split("-video-index", 1)[0]
                tokens = [t for t in head.split("_") if t]
                serial = ""
                for tok in reversed(tokens):
                    t = self._normalize_serial_text(tok)
                    if len(t) >= 6 and t.isdigit():
                        serial = t
                        break
                if serial and serial not in serials:
                    serials.append(serial)
        except Exception:
            pass
        if serials:
            return serials

        # Fallback 2) udevadm 속성에서 Intel(8086) 장치 시리얼 추출
        try:
            for dev in sorted(glob.glob("/dev/video*")):
                try:
                    proc = subprocess.run(
                        ["udevadm", "info", "--query=property", "--name", dev],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        timeout=0.7,
                        check=False,
                    )
                except Exception:
                    continue
                if int(getattr(proc, "returncode", 1)) != 0:
                    continue
                props = {}
                for line in str(proc.stdout or "").splitlines():
                    if "=" not in line:
                        continue
                    k, v = line.split("=", 1)
                    props[str(k).strip()] = str(v).strip()
                if str(props.get("ID_VENDOR_ID", "")).strip() != "8086":
                    continue
                serial = self._normalize_serial_text(props.get("ID_SERIAL_SHORT", ""))
                if not serial:
                    raw_serial = self._normalize_serial_text(props.get("ID_SERIAL", ""))
                    if raw_serial:
                        for tok in reversed([t for t in raw_serial.replace("-", "_").split("_") if t]):
                            t = self._normalize_serial_text(tok)
                            if len(t) >= 6 and t.isdigit():
                                serial = t
                                break
                if serial and serial not in serials:
                    serials.append(serial)
        except Exception:
            pass
        return serials

    def _load_vision_serial_settings(self):
        rows = self._load_parameter_rows()
        def _as_bool(key: str, default: bool) -> bool:
            raw = (rows.get(key) or [str(int(default))])[0] if rows.get(key) else str(int(default))
            s = str(raw).strip().lower()
            if s in ("1", "true", "on", "yes", "y"):
                return True
            if s in ("0", "false", "off", "no", "n"):
                return False
            return bool(default)

        vis1 = self._normalize_serial_text((rows.get("vision1_serial") or [""])[0] if rows.get("vision1_serial") else "")
        vis2 = self._normalize_serial_text((rows.get("vision2_serial") or [""])[0] if rows.get("vision2_serial") else "")
        if not vis1:
            vis1 = DEFAULT_VISION1_SERIAL
        if not vis2:
            vis2 = DEFAULT_VISION2_SERIAL
        if vis1 == vis2:
            vis2 = DEFAULT_VISION2_SERIAL if vis1 != DEFAULT_VISION2_SERIAL else DEFAULT_VISION1_SERIAL
        self._vision_assigned_serial_1 = vis1
        self._vision_assigned_serial_2 = vis2
        self._runtime_camera_serial_1 = vis1
        self._runtime_camera_serial_2 = vis2

        discovered = self._discover_connected_camera_serials()
        merged = []
        for s in discovered + [vis1, vis2]:
            if s and s not in merged:
                merged.append(s)
        self._available_camera_serials = merged
        try:
            rot1 = int(float((rows.get("vision1_view_rotation_deg") or ["0"])[0]))
        except Exception:
            rot1 = 0
        try:
            rot2 = int(float((rows.get("vision2_view_rotation_deg") or ["0"])[0]))
        except Exception:
            rot2 = 0
        self._vision_rotation_deg_1 = self._normalize_rotation_deg(rot1)
        self._vision_rotation_deg_2 = self._normalize_rotation_deg(rot2)
        self._top_status_enabled["vision"] = _as_bool("top_status_vision_enabled", True)
        self._top_status_enabled["vision2"] = _as_bool("top_status_vision2_enabled", True)
        self._top_status_enabled["robot"] = _as_bool("top_status_robot_enabled", True)
        self._voice_order_enabled = _as_bool("top_status_voice_enabled", False)
        # Do not auto-restore calibration ON state on startup.
        self._calibration_mode_enabled_1 = False
        self._calibration_mode_enabled_2 = False
        self._calibration_mode_enabled = bool(self._calibration_mode_enabled_1 or self._calibration_mode_enabled_2)
        for k, toggle in self._top_status_toggles.items():
            if toggle is None:
                continue
            on = bool(self._top_status_enabled.get(k, True))
            toggle.blockSignals(True)
            toggle.setChecked(on)
            toggle.setText("ON" if on else "OFF")
            toggle.blockSignals(False)
        voice_toggle = getattr(self, "_voice_order_connection_toggle", None)
        if voice_toggle is not None:
            voice_toggle.blockSignals(True)
            voice_toggle.setChecked(bool(self._voice_order_enabled))
            voice_toggle.setText("ON" if bool(self._voice_order_enabled) else "OFF")
            voice_toggle.blockSignals(False)
            self._set_voice_order_connection_state(bool(self._voice_order_enabled))

    def _save_vision_serial_settings(self):
        rows = self._load_parameter_rows()
        rows["vision1_serial"] = [self._normalize_serial_text(self._vision_assigned_serial_1)]
        rows["vision2_serial"] = [self._normalize_serial_text(self._vision_assigned_serial_2)]
        rows["vision1_view_rotation_deg"] = [str(int(self._vision_rotation_deg_1))]
        rows["vision2_view_rotation_deg"] = [str(int(self._vision_rotation_deg_2))]
        rows["top_status_vision_enabled"] = ["1" if bool(self._top_status_enabled.get("vision", True)) else "0"]
        rows["top_status_vision2_enabled"] = ["1" if bool(self._top_status_enabled.get("vision2", True)) else "0"]
        rows["top_status_robot_enabled"] = ["1" if bool(self._top_status_enabled.get("robot", True)) else "0"]
        voice_toggle = getattr(self, "_voice_order_connection_toggle", None)
        voice_on = bool(voice_toggle.isChecked()) if voice_toggle is not None else bool(self._voice_order_enabled)
        rows["top_status_voice_enabled"] = ["1" if voice_on else "0"]
        # Keep startup behavior deterministic: calibration mode starts OFF after restart.
        rows["vision1_calibration_enabled"] = ["0"]
        rows["vision2_calibration_enabled"] = ["0"]
        self._save_parameter_rows(rows)

    def _normalize_rotation_deg(self, value):
        try:
            v = int(value)
        except Exception:
            v = 0
        v = v % 360
        candidates = (0, 90, 180, 270)
        return min(candidates, key=lambda x: abs(x - v))

    def _rotate_image_for_panel(self, image: QImage, panel_index: int):
        if image is None:
            return None
        deg = self._vision_rotation_deg_2 if int(panel_index) == 2 else self._vision_rotation_deg_1
        deg = self._normalize_rotation_deg(deg)
        if deg == 0:
            return image
        tf = QTransform()
        tf.rotate(float(deg))
        return image.transformed(tf, Qt.FastTransformation)

    def _map_rotated_to_source_coords(self, x_rot: int, y_rot: int, src_w: int, src_h: int, rot_deg: int):
        rot = self._normalize_rotation_deg(rot_deg)
        x_r = int(max(0, min(max(0, (src_h - 1) if rot in (90, 270) else (src_w - 1)), x_rot)))
        y_r = int(max(0, min(max(0, (src_w - 1) if rot in (90, 270) else (src_h - 1)), y_rot)))
        if rot == 90:  # clockwise
            return int(max(0, min(src_w - 1, y_r))), int(max(0, min(src_h - 1, (src_h - 1) - x_r)))
        if rot == 180:
            return int(max(0, min(src_w - 1, (src_w - 1) - x_r))), int(max(0, min(src_h - 1, (src_h - 1) - y_r)))
        if rot == 270:  # clockwise
            return int(max(0, min(src_w - 1, (src_w - 1) - y_r))), int(max(0, min(src_h - 1, x_r)))
        return int(max(0, min(src_w - 1, x_r))), int(max(0, min(src_h - 1, y_r)))

    def _map_source_to_rotated_coords(self, x_src: int, y_src: int, src_w: int, src_h: int, rot_deg: int):
        rot = self._normalize_rotation_deg(rot_deg)
        xs = int(max(0, min(max(0, src_w - 1), int(x_src))))
        ys = int(max(0, min(max(0, src_h - 1), int(y_src))))
        if rot == 90:   # clockwise
            return int(max(0, min(src_h - 1, (src_h - 1) - ys))), int(max(0, min(src_w - 1, xs)))
        if rot == 180:
            return int(max(0, min(src_w - 1, (src_w - 1) - xs))), int(max(0, min(src_h - 1, (src_h - 1) - ys)))
        if rot == 270:  # clockwise
            return int(max(0, min(src_h - 1, ys))), int(max(0, min(src_w - 1, (src_w - 1) - xs)))
        return xs, ys

    def _to_display_uv(self, u_src: float, v_src: float, panel_index: int = 1, src_w: int = None, src_h: int = None):
        panel = 2 if int(panel_index) == 2 else 1
        if src_w is None or src_h is None:
            size = self._last_yolo_image_size_2 if panel == 2 else self._last_yolo_image_size
            if size is None or len(size) < 2:
                return float(u_src), float(v_src)
            src_w = int(size[0])
            src_h = int(size[1])
        rot = self._vision_rotation_deg_2 if panel == 2 else self._vision_rotation_deg_1
        du, dv = self._map_source_to_rotated_coords(int(round(float(u_src))), int(round(float(v_src))), int(src_w), int(src_h), int(rot))
        return float(du), float(dv)

    def _map_source_to_canvas_coords(self, x_src: float, y_src: float, panel_index: int = 1):
        panel = 2 if int(panel_index) == 2 else 1
        m = self._yolo_view_map_2 if panel == 2 else self._yolo_view_map
        if not m:
            return None
        src_orig_w = int(m.get("src_orig_w", m.get("src_w", 0)))
        src_orig_h = int(m.get("src_orig_h", m.get("src_h", 0)))
        if src_orig_w <= 0 or src_orig_h <= 0:
            return None
        rot_deg = int(m.get("rotation_deg", 0))
        xr, yr = self._map_source_to_rotated_coords(
            int(round(float(x_src))),
            int(round(float(y_src))),
            src_orig_w,
            src_orig_h,
            rot_deg,
        )
        scale = float(m.get("scale", 1.0))
        ax = float(xr) * scale
        ay = float(yr) * scale
        draw_w = int(m.get("draw_w", 0))
        draw_h = int(m.get("draw_h", 0))
        view_w = int(m.get("view_w", 0))
        view_h = int(m.get("view_h", 0))
        crop_x = int(m.get("crop_x", 0))
        crop_y = int(m.get("crop_y", 0))
        pad_x = int(m.get("pad_x", 0))
        pad_y = int(m.get("pad_y", 0))
        vx = (ax - float(crop_x)) if draw_w > view_w else (ax + float(pad_x))
        vy = (ay - float(crop_y)) if draw_h > view_h else (ay + float(pad_y))
        return int(round(vx)), int(round(vy))

    def _set_vision_rotation(self, panel_index: int, delta_deg: int):
        if int(panel_index) == 2:
            self._vision_rotation_deg_2 = self._normalize_rotation_deg(self._vision_rotation_deg_2 + int(delta_deg))
            # 회전 후 원점이 보이도록 포커스를 원점 기준으로 초기화한다.
            self._yolo_pan_x_2 = -1000000.0
            self._yolo_pan_y_2 = -1000000.0
            self._render_yolo_view_2()
            self.append_log(f"[비전2] 화면 회전: {self._vision_rotation_deg_2}도\n")
        else:
            self._vision_rotation_deg_1 = self._normalize_rotation_deg(self._vision_rotation_deg_1 + int(delta_deg))
            self._yolo_pan_x = -1000000.0
            self._yolo_pan_y = -1000000.0
            self._render_yolo_view()
            self.append_log(f"[비전1] 화면 회전: {self._vision_rotation_deg_1}도\n")
        if self.backend is not None and YOLO_EXTERNAL_NODE and YOLO_AUTO_LAUNCH_NODE:
            self._ensure_external_vision_process_panel(panel_index)
        self._refresh_vision_rotation_labels()
        self._save_vision_serial_settings()

    def _set_vision_rotation_absolute(self, panel_index: int, absolute_deg: int):
        target = self._normalize_rotation_deg(absolute_deg)
        if int(panel_index) == 2:
            self._vision_rotation_deg_2 = target
            self._yolo_pan_x_2 = -1000000.0
            self._yolo_pan_y_2 = -1000000.0
            self._render_yolo_view_2()
            self.append_log(f"[비전2] 화면 회전: {self._vision_rotation_deg_2}도\n")
        else:
            self._vision_rotation_deg_1 = target
            self._yolo_pan_x = -1000000.0
            self._yolo_pan_y = -1000000.0
            self._render_yolo_view()
            self.append_log(f"[비전1] 화면 회전: {self._vision_rotation_deg_1}도\n")
        if self.backend is not None and YOLO_EXTERNAL_NODE and YOLO_AUTO_LAUNCH_NODE:
            self._ensure_external_vision_process_panel(panel_index)
        self._refresh_vision_rotation_labels()
        self._save_vision_serial_settings()

    def _runtime_camera_slot_for_serial(self, serial):
        s = self._normalize_serial_text(serial)
        if s and s == self._normalize_serial_text(self._runtime_camera_serial_1):
            return 1
        if s and s == self._normalize_serial_text(self._runtime_camera_serial_2):
            return 2
        return 1

    def _assigned_topic_for_serial(self, serial):
        slot = self._runtime_camera_slot_for_serial(serial)
        return "/camera/camera_2/color/image_raw" if slot == 2 else CALIB_VISION_TOPIC_PRIMARY

    def _assigned_depth_topic_for_serial(self, serial, node=None):
        slot = self._runtime_camera_slot_for_serial(serial)
        _ = node
        if slot == 2:
            return "/camera/camera_2/aligned_depth_to_color/image_raw"
        return "/camera/camera_1/aligned_depth_to_color/image_raw"

    def _assigned_raw_topic_for_serial(self, serial, node):
        slot = self._runtime_camera_slot_for_serial(serial)
        _ = node
        if slot == 2:
            return "/camera/camera_2/color/image_raw"
        return CALIB_VISION_TOPIC_PRIMARY

    def _assigned_info_topic_for_serial(self, serial, node):
        slot = self._runtime_camera_slot_for_serial(serial)
        _ = node
        if slot == 2:
            return "/camera/camera_2/color/camera_info"
        return CALIB_CAMERA_INFO_TOPIC_PRIMARY

    def _vision_panel_needs_depth(self, panel_index: int = 1):
        panel = 2 if int(panel_index) == 2 else 1
        calib_on = bool(self._calibration_mode_enabled_2) if panel == 2 else bool(self._calibration_mode_enabled_1)
        return not calib_on

    def _vision_panel_needs_camera_info(self, panel_index: int = 1):
        # camera_info는 저부하이며 축/좌표 정확도에 직접 필요하므로 active panel에서는 유지한다.
        _ = panel_index
        return True

    def _refresh_vision_serial_labels(self):
        text1 = f"시리얼: {self._normalize_serial_text(self._vision_assigned_serial_1) or '-'}"
        text2 = f"시리얼: {self._normalize_serial_text(self._vision_assigned_serial_2) or '-'}"
        if self._vision_serial_label is not None:
            self._vision_serial_label.setText(text1)
        if self._vision_serial_label_2 is not None:
            self._vision_serial_label_2.setText(text2)

    def _refresh_vision_rotation_labels(self):
        if self._vision_rotation_label is not None:
            self._vision_rotation_label.setText(f"각도: {int(self._vision_rotation_deg_1)}°")
        if self._vision_rotation_label_2 is not None:
            self._vision_rotation_label_2.setText(f"각도: {int(self._vision_rotation_deg_2)}°")

    def _try_load_calibration_matrix_on_startup(self):
        try:
            # Legacy(calibration/*) -> New(config/calibration/*) one-way migration
            try:
                if os.path.isdir(LEGACY_CALIB_ROBOT_DIR):
                    os.makedirs(CALIB_ROBOT_DIR, exist_ok=True)
                    for p in glob.glob(os.path.join(LEGACY_CALIB_ROBOT_DIR, "*")):
                        if not os.path.isfile(p):
                            continue
                        dst = os.path.join(CALIB_ROBOT_DIR, os.path.basename(p))
                        if not os.path.isfile(dst):
                            try:
                                import shutil
                                shutil.copy2(p, dst)
                            except Exception:
                                pass
                if os.path.isdir(LEGACY_CALIB_ROTMAT_DIR):
                    os.makedirs(CALIB_ROTMAT_DIR, exist_ok=True)
                    for p in glob.glob(os.path.join(LEGACY_CALIB_ROTMAT_DIR, "*")):
                        if not os.path.isfile(p):
                            continue
                        dst = os.path.join(CALIB_ROTMAT_DIR, os.path.basename(p))
                        if not os.path.isfile(dst):
                            try:
                                import shutil
                                shutil.copy2(p, dst)
                            except Exception:
                                pass
            except Exception:
                pass

            env_path = os.environ.get("CALIB_MATRIX_PATH", "").strip()
            candidates = []
            if env_path:
                candidates.append(env_path)
            migrated_legacy_path = ""
            if os.path.isfile(LEGACY_CALIB_ACTIVE_PATH_FILE):
                try:
                    with open(LEGACY_CALIB_ACTIVE_PATH_FILE, "r", encoding="utf-8") as f:
                        p = f.readline().strip()
                    if p:
                        migrated_legacy_path = p
                    os.remove(LEGACY_CALIB_ACTIVE_PATH_FILE)
                except Exception:
                    pass
            if migrated_legacy_path:
                self._save_active_calibration_path(migrated_legacy_path)
                candidates.append(migrated_legacy_path)
            rows = self._load_parameter_rows()
            active_path_1 = (rows.get("active_calibration_path_1") or [""])[0] if rows.get("active_calibration_path_1") else ""
            active_path_2 = (rows.get("active_calibration_path_2") or [""])[0] if rows.get("active_calibration_path_2") else ""
            legacy_active = (rows.get("active_calibration_path") or [""])[0] if rows.get("active_calibration_path") else ""
            if not active_path_1:
                active_path_1 = legacy_active
            if not active_path_2:
                active_path_2 = active_path_1

            path_for_panel = {
                1: active_path_1 if active_path_1 else (candidates[0] if candidates else ""),
                2: active_path_2 if active_path_2 else active_path_1,
            }
            loaded_any = False
            for panel, raw_path in path_for_panel.items():
                p = _resolve_repo_file(raw_path, fallback_dirs=(CALIB_DIR, CALIB_ROTMAT_DIR))
                if not p:
                    continue
                mat, rmse = self._load_affine_from_file(p)
                if mat is None:
                    continue
                self._apply_vision_to_robot_affine(mat, rmse=rmse, path=p, panel_index=panel)
                self.append_log(f"[캘리브레이션{panel}] 시작 시 행렬 자동 로드: {p}\n")
                loaded_any = True
            if not loaded_any:
                # 시작 시점에 로드 가능한 행렬이 없으면 명시적으로 없음 상태를 유지/표시
                self._calib_matrix_path = None
                self._calib_matrix_path_1 = None
                self._calib_matrix_path_2 = None
                self.append_log("[캘리브레이션] 시작 시 적용 가능한 행렬 없음 (현재 적용행렬: 없음)\n")
        except Exception:
            return

    def _build_vision_image_qos(self):
        if (
            QoSProfile is not None
            and QoSHistoryPolicy is not None
            and QoSReliabilityPolicy is not None
        ):
            return QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
            )
        return qos_profile_sensor_data if qos_profile_sensor_data is not None else 1

    def _is_topic_alive(self, node, topic_name):
        if node is None or not topic_name:
            return False
        try:
            if int(node.count_publishers(str(topic_name))) > 0:
                return True
        except Exception:
            pass
        try:
            infos = node.get_publishers_info_by_topic(str(topic_name))
            if infos:
                return True
        except Exception:
            pass
        return False

    def _is_image_topic_alive(self, node, topic_name):
        return self._is_topic_alive(node, topic_name)

    def _calibration_output_meta_topic(self, panel_index: int = 1):
        return CALIB_OUTPUT_META_TOPIC_2 if int(panel_index) == 2 else CALIB_OUTPUT_META_TOPIC_1

    def _vision_output_meta_topic(self, panel_index: int = 1):
        return VISION_VOLUME_META_TOPIC_2 if int(panel_index) == 2 else VISION_OBJECT_META_TOPIC_1

    def _stop_foreign_vision_helpers(self, panel_index: int, keep_pid=None):
        panel = 2 if int(panel_index) == 2 else 1
        meta_topic = self._vision_output_meta_topic(panel)
        script_name = "glass_fill_level.py" if panel == 2 else "drink_detection.py"
        legacy_script_name = "realsense_panel_meta_process.py"
        keep = set()
        try:
            if keep_pid is not None:
                keep.add(int(keep_pid))
        except Exception:
            pass
        keep.add(int(os.getpid()))
        try:
            outputs = []
            for name in (script_name, legacy_script_name):
                try:
                    outputs.append(subprocess.check_output(["pgrep", "-af", name], text=True))
                except Exception:
                    continue
        except Exception:
            return
        for output in outputs:
            for raw in str(output).splitlines():
                line = str(raw).strip()
                if not line:
                    continue
                parts = line.split(None, 1)
                if not parts:
                    continue
                try:
                    pid = int(parts[0])
                except Exception:
                    continue
                if pid in keep:
                    continue
                cmdline = parts[1] if len(parts) > 1 else ""
                if meta_topic not in cmdline:
                    continue
                try:
                    os.kill(pid, signal.SIGINT)
                except Exception:
                    continue
                try:
                    os.kill(pid, signal.SIGTERM)
                except Exception:
                    pass
                self.append_log(f"[비전{panel}] 기존 메타 헬퍼 정리: PID {parts[0]}\n")

    def _stop_foreign_calibration_helpers(self, panel_index: int, keep_pid=None):
        panel = 2 if int(panel_index) == 2 else 1
        meta_topic = self._calibration_output_meta_topic(panel)
        keep = set()
        try:
            if keep_pid is not None:
                keep.add(int(keep_pid))
        except Exception:
            pass
        keep.add(int(os.getpid()))
        try:
            output = subprocess.check_output(
                ["pgrep", "-af", "camera_eye_to_hand_robot_calibration.py"],
                text=True,
            )
        except Exception:
            return
        for raw in str(output).splitlines():
            line = str(raw).strip()
            if not line:
                continue
            parts = line.split(None, 1)
            if not parts:
                continue
            try:
                pid = int(parts[0])
            except Exception:
                continue
            if pid in keep:
                continue
            cmdline = parts[1] if len(parts) > 1 else ""
            if meta_topic not in cmdline:
                continue
            try:
                os.kill(pid, signal.SIGINT)
            except Exception:
                continue
            try:
                os.kill(pid, signal.SIGTERM)
            except Exception:
                pass
            self.append_log(f"[캘리브레이션{panel}] 기존 헬퍼 프로세스 정리: PID {parts[0]}\n")

    def _vision_subscription_node(self):
        if self.backend is None:
            return None
        getter = getattr(self.backend, "get_vision_subscription_node", None)
        if callable(getter):
            try:
                node = getter()
            except Exception:
                node = None
            if node is not None:
                return node
        return getattr(self.backend, "robot_controller", None)

    def _current_vision_image_topic(self):
        node = self._vision_subscription_node()
        if node is not None:
            return self._assigned_raw_topic_for_serial(self._vision_assigned_serial_1, node)
        return self._assigned_topic_for_serial(self._vision_assigned_serial_1)

    def _current_vision_image_topic_2(self):
        node = self._vision_subscription_node()
        if node is not None:
            return self._assigned_raw_topic_for_serial(self._vision_assigned_serial_2, node)
        return self._assigned_topic_for_serial(self._vision_assigned_serial_2)

    def _build_vision_meta_process_cmd(self, panel_index: int):
        panel = 2 if int(panel_index) == 2 else 1
        node = self._vision_subscription_node()
        if node is None:
            return None
        serial = self._vision_assigned_serial_2 if panel == 2 else self._vision_assigned_serial_1
        script_name = "glass_fill_level.py" if panel == 2 else "drink_detection.py"
        vision_py = str(os.environ.get("BARTENDER_VISION_PYTHON", "") or "").strip()
        py_exec = vision_py if (vision_py and os.path.isfile(vision_py)) else sys.executable
        return [
            py_exec,
            os.path.join(PROJECT_ROOT, "src", "vision", script_name),
            "--image-topic",
            self._assigned_raw_topic_for_serial(serial, node),
            "--depth-topic",
            self._assigned_depth_topic_for_serial(serial, node),
            "--output-meta-topic",
            self._vision_output_meta_topic(panel),
            "--process-hz",
            str(float(VISION_META_PROCESS_HZ)),
            "--rotation-deg",
            str(int(self._vision_rotation_deg_2 if panel == 2 else self._vision_rotation_deg_1)),
        ]

    def _ensure_external_vision_process_panel(self, panel_index: int):
        panel = 2 if int(panel_index) == 2 else 1
        calib_on = bool(self._calibration_mode_enabled_2) if panel == 2 else bool(self._calibration_mode_enabled_1)
        if calib_on:
            self._stop_external_vision_process_panel(panel)
            return False
        if not bool(self._top_status_enabled.get("vision2" if panel == 2 else "vision", True)):
            self._stop_external_vision_process_panel(panel)
            return False
        cmd = self._build_vision_meta_process_cmd(panel)
        if not cmd:
            return False
        proc = self._external_vision_proc_2 if panel == 2 else self._external_vision_proc_1
        prev_cmd = self._external_vision_cmd_2 if panel == 2 else self._external_vision_cmd_1
        keep_pid = proc.pid if (proc is not None and proc.poll() is None) else None
        self._stop_foreign_vision_helpers(panel, keep_pid=keep_pid)
        if proc is not None and proc.poll() is None and prev_cmd == cmd:
            return True
        self._stop_external_vision_process_panel(panel)
        try:
            proc = subprocess.Popen(
                cmd,
                cwd=PROJECT_ROOT,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            self.append_log(f"[비전{panel}] 메타 프로세스 시작 실패: {e}\n")
            return False
        if panel == 2:
            self._external_vision_proc_2 = proc
            self._external_vision_cmd_2 = list(cmd)
            self._external_vision_started_by_ui_2 = True
        else:
            self._external_vision_proc_1 = proc
            self._external_vision_cmd_1 = list(cmd)
            self._external_vision_started_by_ui_1 = True
        self.append_log(f"[비전{panel}] 메타 프로세스 시작 (python={cmd[0]})\n")
        return True

    def _stop_external_vision_process_panel(self, panel_index: int):
        panel = 2 if int(panel_index) == 2 else 1
        if panel == 2:
            proc = self._external_vision_proc_2
            started_by_ui = bool(self._external_vision_started_by_ui_2)
        else:
            proc = self._external_vision_proc_1
            started_by_ui = bool(self._external_vision_started_by_ui_1)
        if proc is not None and proc.poll() is None and started_by_ui:
            try:
                proc.send_signal(signal.SIGINT)
            except Exception:
                pass
            try:
                if proc.poll() is None:
                    proc.terminate()
            except Exception:
                pass
        if panel == 2:
            self._external_vision_proc_2 = None
            self._external_vision_cmd_2 = None
            self._external_vision_started_by_ui_2 = False
        else:
            self._external_vision_proc_1 = None
            self._external_vision_cmd_1 = None
            self._external_vision_started_by_ui_1 = False

    def _teardown_external_vision_bridge_panel(self, panel_index: int):
        node = self._vision_subscription_node()

        def _destroy(sub):
            if sub is None:
                return
            try:
                if node is not None:
                    node.destroy_subscription(sub)
                    return
            except Exception:
                pass
            try:
                sub.destroy()
            except Exception:
                pass

        panel = 2 if int(panel_index) == 2 else 1
        if panel == 2:
            _destroy(getattr(self, "_vision_sub_2", None))
            _destroy(getattr(self, "_calib_meta_sub_2", None))
            _destroy(getattr(self, "_vision_meta_sub_2", None))
            _destroy(getattr(self, "_vision_depth_sub_2", None))
            _destroy(getattr(self, "_vision_info_sub_2", None))
            self._vision_sub_2 = None
            self._calib_meta_sub_2 = None
            self._vision_meta_sub_2 = None
            self._vision_depth_sub_2 = None
            self._vision_info_sub_2 = None
            self._vision_image_topic_in_use_2 = None
            self._calib_meta_topic_in_use_2 = None
            self._vision_meta_topic_in_use_2 = None
        else:
            _destroy(getattr(self, "_vision_sub", None))
            _destroy(getattr(self, "_calib_meta_sub_1", None))
            _destroy(getattr(self, "_vision_meta_sub_1", None))
            _destroy(getattr(self, "_vision_depth_sub", None))
            _destroy(getattr(self, "_vision_info_sub", None))
            self._vision_sub = None
            self._calib_meta_sub_1 = None
            self._vision_meta_sub_1 = None
            self._vision_depth_sub = None
            self._vision_info_sub = None
            self._vision_image_topic_in_use = None
            self._calib_meta_topic_in_use_1 = None
            self._vision_meta_topic_in_use_1 = None

    def _teardown_external_vision_bridge(self):
        self._teardown_external_vision_bridge_panel(1)
        self._teardown_external_vision_bridge_panel(2)

    def _build_calibration_process_cmd(self, panel_index: int):
        panel = 2 if int(panel_index) == 2 else 1
        node = self._vision_subscription_node()
        if node is None:
            return None
        serial = self._vision_assigned_serial_2 if panel == 2 else self._vision_assigned_serial_1
        return [
            sys.executable,
            os.path.join(PROJECT_ROOT, "src", "vision", "camera_eye_to_hand_robot_calibration.py"),
            "--image-topic",
            self._assigned_raw_topic_for_serial(serial, node),
            "--depth-topic",
            self._assigned_depth_topic_for_serial(serial, node),
            "--camera-info-topic",
            self._assigned_info_topic_for_serial(serial, node),
            "--output-meta-topic",
            self._calibration_output_meta_topic(panel),
            "--cols",
            str(int(self._calib_pattern_cols)),
            "--rows",
            str(int(self._calib_pattern_rows)),
            "--panel",
            str(panel),
            "--hold-sec",
            str(float(CALIB_DETECTION_HOLD_SEC)),
            "--detect-interval-sec",
            str(float(CALIB_DETECT_INTERVAL_SEC)),
            "--process-hz",
            str(float(CALIB_PROCESS_HZ)),
        ]

    def _ensure_calibration_process(self, panel_index: int):
        panel = 2 if int(panel_index) == 2 else 1
        node = self._vision_subscription_node()
        if node is None:
            return False
        cmd = self._build_calibration_process_cmd(panel)
        if not cmd:
            return False
        proc = self._calib_proc_2 if panel == 2 else self._calib_proc_1
        prev_cmd = self._calib_proc_cmd_2 if panel == 2 else self._calib_proc_cmd_1
        keep_pid = proc.pid if (proc is not None and proc.poll() is None) else None
        self._stop_foreign_calibration_helpers(panel, keep_pid=keep_pid)
        if proc is not None and proc.poll() is None and prev_cmd == cmd:
            return True
        self._stop_calibration_process(panel)
        try:
            proc = subprocess.Popen(
                cmd,
                cwd=PROJECT_ROOT,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            self.append_log(f"[캘리브레이션{panel}] 프로세스 시작 실패: {e}\n")
            return False
        if panel == 2:
            self._calib_proc_2 = proc
            self._calib_proc_cmd_2 = list(cmd)
            self._calib_proc_started_by_ui_2 = True
        else:
            self._calib_proc_1 = proc
            self._calib_proc_cmd_1 = list(cmd)
            self._calib_proc_started_by_ui_1 = True
        self.append_log(f"[캘리브레이션{panel}] 프로세스 시작\n")
        return True

    def _stop_calibration_process(self, panel_index: int):
        panel = 2 if int(panel_index) == 2 else 1
        if panel == 2:
            proc = self._calib_proc_2
            started_by_ui = bool(self._calib_proc_started_by_ui_2)
        else:
            proc = self._calib_proc_1
            started_by_ui = bool(self._calib_proc_started_by_ui_1)
        if proc is not None and proc.poll() is None and started_by_ui:
            try:
                proc.send_signal(signal.SIGINT)
            except Exception:
                pass
            try:
                if proc.poll() is None:
                    proc.terminate()
            except Exception:
                pass
        if panel == 2:
            self._calib_proc_2 = None
            self._calib_proc_cmd_2 = None
            self._calib_proc_started_by_ui_2 = False
        else:
            self._calib_proc_1 = None
            self._calib_proc_cmd_1 = None
            self._calib_proc_started_by_ui_1 = False

    def _sync_calibration_processes(self):
        if self.backend is None:
            return
        if bool(getattr(self, "_calibration_mode_enabled_1", False)) and bool(self._top_status_enabled.get("vision", True)):
            self._ensure_calibration_process(1)
        else:
            self._stop_calibration_process(1)
        if bool(getattr(self, "_calibration_mode_enabled_2", False)) and bool(self._top_status_enabled.get("vision2", True)):
            self._ensure_calibration_process(2)
        else:
            self._stop_calibration_process(2)

    def _sync_calibration_process_panel(self, panel_index: int):
        panel = 2 if int(panel_index) == 2 else 1
        if self.backend is None:
            return
        if panel == 2:
            if bool(getattr(self, "_calibration_mode_enabled_2", False)) and bool(self._top_status_enabled.get("vision2", True)):
                self._ensure_calibration_process(2)
            else:
                self._stop_calibration_process(2)
        else:
            if bool(getattr(self, "_calibration_mode_enabled_1", False)) and bool(self._top_status_enabled.get("vision", True)):
                self._ensure_calibration_process(1)
            else:
                self._stop_calibration_process(1)

    def _apply_calibration_meta(self, panel_index: int, raw: str, stream_token=None):
        panel = 2 if int(panel_index) == 2 else 1
        if stream_token is not None:
            current = int(getattr(self, "_vision_stream_token_2", 0)) if panel == 2 else int(getattr(self, "_vision_stream_token_1", 0))
            if int(stream_token) != current:
                return
        try:
            meta = json.loads(raw or "{}")
        except Exception:
            return
        now = time.monotonic()
        points = meta.get("points")
        if not isinstance(points, dict):
            points = {}
        center = meta.get("center")
        if center is not None and isinstance(center, (list, tuple)) and len(center) >= 3:
            points["center"] = [float(center[0]), float(center[1]), float(center[2])]
        grid_points = meta.get("grid_points_uv")
        grid_status = meta.get("grid_status")
        grid_xyz = meta.get("grid_camera_xyz_mm")
        detected = bool(meta.get("detected", False))
        full_ready = bool(meta.get("full_ready", False))
        reason = str(meta.get("reason", "대기"))
        full_reason = str(meta.get("full_reason", reason))
        raw_input_interval_ms = meta.get("raw_input_interval_ms")
        processing_ms = meta.get("processing_ms")
        publish_interval_ms = meta.get("publish_interval_ms")
        frame_wait_ms = meta.get("frame_wait_ms")
        try:
            grid_points_arr = np.asarray(grid_points, dtype=np.float64) if grid_points is not None else None
        except Exception:
            grid_points_arr = None
        try:
            grid_xyz_arr = np.asarray(grid_xyz, dtype=np.float64) if grid_xyz is not None else None
        except Exception:
            grid_xyz_arr = None
        if panel == 2:
            self._calib_last_points_uvz_mm_2 = points if points else None
            self._calib_last_points_at_2 = now if detected else None
            self._calib_last_grid_pts_2 = grid_points_arr
            self._calib_last_grid_status_2 = list(grid_status) if isinstance(grid_status, list) else None
            self._calib_grid_camera_xyz_2 = grid_xyz_arr
            self._calib_last_reason_2 = reason
            self._calib_full_ready_2 = bool(full_ready)
            self._calib_full_reason_2 = full_reason
            self._calib_proc_input_ms_2 = float(raw_input_interval_ms) if raw_input_interval_ms is not None else None
            self._calib_proc_ms_2 = float(processing_ms) if processing_ms is not None else None
            self._calib_proc_publish_ms_2 = float(publish_interval_ms) if publish_interval_ms is not None else None
            self._calib_proc_wait_ms_2 = float(frame_wait_ms) if frame_wait_ms is not None else None
        else:
            self._calib_last_points_uvz_mm_1 = points if points else None
            self._calib_last_points_at_1 = now if detected else None
            self._calib_last_grid_pts_1 = grid_points_arr
            self._calib_last_grid_status_1 = list(grid_status) if isinstance(grid_status, list) else None
            self._calib_grid_camera_xyz_1 = grid_xyz_arr
            self._calib_last_reason_1 = reason
            self._calib_full_ready_1 = bool(full_ready)
            self._calib_full_reason_1 = full_reason
            self._calib_proc_input_ms_1 = float(raw_input_interval_ms) if raw_input_interval_ms is not None else None
            self._calib_proc_ms_1 = float(processing_ms) if processing_ms is not None else None
            self._calib_proc_publish_ms_1 = float(publish_interval_ms) if publish_interval_ms is not None else None
            self._calib_proc_wait_ms_1 = float(frame_wait_ms) if frame_wait_ms is not None else None
            self._calib_last_points_uvz_mm = self._calib_last_points_uvz_mm_1
            self._calib_last_points_at = self._calib_last_points_at_1
            self._calib_last_reason = self._calib_last_reason_1
        self._request_vision_overlay_refresh(panel)
        self.calibration_ui_refresh_requested.emit()

    def _on_calibration_meta_msg(self, msg, stream_token=None):
        try:
            self._apply_calibration_meta(1, getattr(msg, "data", ""), stream_token=stream_token)
        except Exception:
            return

    def _on_calibration_meta_msg_2(self, msg, stream_token=None):
        try:
            self._apply_calibration_meta(2, getattr(msg, "data", ""), stream_token=stream_token)
        except Exception:
            return

    def _apply_vision_meta(self, panel_index: int, raw: str, stream_token=None):
        panel = 2 if int(panel_index) == 2 else 1
        if stream_token is not None:
            current = int(getattr(self, "_vision_stream_token_2", 0)) if panel == 2 else int(getattr(self, "_vision_stream_token_1", 0))
            if int(stream_token) != current:
                return
        try:
            meta = json.loads(raw or "{}")
        except Exception:
            return
        meta_payload = meta if isinstance(meta, dict) else None
        now = time.monotonic()
        prev_received_at = self._vision_meta_received_at_2 if panel == 2 else self._vision_meta_received_at_1
        fallback_ms = None
        if isinstance(meta_payload, dict):
            for key in ("processing_ms", "source_age_ms"):
                try:
                    value = float(meta_payload.get(key))
                except Exception:
                    value = None
                if value is not None and value >= 0.0:
                    fallback_ms = value
                    break
            if fallback_ms is None:
                try:
                    fallback_ms = 1000.0 / max(1.0, float(VISION_META_PROCESS_HZ))
                except Exception:
                    fallback_ms = None
        if prev_received_at is not None:
            try:
                interval_ms = max(0.0, (now - float(prev_received_at)) * 1000.0)
            except Exception:
                interval_ms = fallback_ms
        else:
            interval_ms = fallback_ms
        if panel == 2:
            self._vision_meta_payload_2 = meta_payload
            self._vision_meta_received_at_2 = now
            self._vision_meta_cycle_ms_2 = interval_ms
            if self._vision_payload_has_runtime_data(2, meta_payload):
                self._vision_meta_last_nonempty_payload_2 = meta_payload
                self._vision_meta_last_nonempty_at_2 = now
        else:
            self._vision_meta_payload_1 = meta_payload
            self._vision_meta_received_at_1 = now
            self._vision_meta_cycle_ms_1 = interval_ms
            if self._vision_payload_has_runtime_data(1, meta_payload):
                self._vision_meta_last_nonempty_payload_1 = meta_payload
                self._vision_meta_last_nonempty_at_1 = now
        self._request_vision_overlay_refresh(panel)
        self.vision_runtime_ui_refresh_requested.emit(panel)

    def _on_vision_meta_msg(self, msg, stream_token=None):
        try:
            self._apply_vision_meta(1, getattr(msg, "data", ""), stream_token=stream_token)
        except Exception:
            return

    def _on_vision_meta_msg_2(self, msg, stream_token=None):
        try:
            self._apply_vision_meta(2, getattr(msg, "data", ""), stream_token=stream_token)
        except Exception:
            return

    def _ensure_external_vision_process(self):
        if self.backend is None or not UI_ENABLE_VISION or (not YOLO_EXTERNAL_NODE):
            return False
        ok = False
        if bool(self._top_status_enabled.get("vision", True)):
            ok = self._ensure_external_vision_process_panel(1) or ok
        else:
            self._stop_external_vision_process_panel(1)
        if bool(self._top_status_enabled.get("vision2", True)):
            ok = self._ensure_external_vision_process_panel(2) or ok
        else:
            self._stop_external_vision_process_panel(2)
        return ok

    def _stop_external_vision_process(self):
        self._stop_external_vision_process_panel(1)
        self._stop_external_vision_process_panel(2)
        self._external_vision_proc = None
        self._external_vision_cmd = None
        self._external_vision_started_by_ui = False

    def _start_yolo_camera(self):
        if not UI_ENABLE_VISION or self.backend is None:
            return
        try:
            if YOLO_EXTERNAL_NODE and YOLO_AUTO_LAUNCH_NODE:
                self._ensure_external_vision_process()
            self._setup_external_vision_bridge()
            return
        except Exception as e:
            self._append_vision_log(f"시작 실패: {e}")
        self._append_vision_log("객체인식 메타 프로세스 미사용 상태입니다.")

    def _setup_external_vision_bridge_panel(self, panel_index: int):
        if RosImageMsg is None or self.backend is None:
            return False
        node = self._vision_subscription_node()
        if node is None:
            return False
        panel = 2 if int(panel_index) == 2 else 1
        calib_on = bool(self._calibration_mode_enabled_2) if panel == 2 else bool(self._calibration_mode_enabled_1)
        image_qos = self._build_vision_image_qos()
        if panel == 2:
            sub_kwargs = {}
            if self._vision_cb_group_2 is not None:
                sub_kwargs["callback_group"] = self._vision_cb_group_2
            image_topic = self._current_vision_image_topic_2()
            self._teardown_external_vision_bridge_panel(2)
            stream_token = self._vision_stream_token_2
            self._vision_sub_2 = node.create_subscription(
                RosImageMsg, image_topic, lambda msg, token=stream_token: self._on_vision_image_msg_2(msg, token), image_qos, **sub_kwargs
            )
            self._vision_image_topic_in_use_2 = image_topic
            if (not calib_on) and RosStringMsg is not None:
                meta_topic_runtime_2 = self._vision_output_meta_topic(2)
                self._vision_meta_sub_2 = node.create_subscription(
                    RosStringMsg,
                    meta_topic_runtime_2,
                    lambda msg, token=stream_token: self._on_vision_meta_msg_2(msg, token),
                    10,
                    **sub_kwargs,
                )
                self._vision_meta_topic_in_use_2 = meta_topic_runtime_2
            if calib_on and RosStringMsg is not None:
                meta_topic_2 = self._calibration_output_meta_topic(2)
                self._calib_meta_sub_2 = node.create_subscription(
                    RosStringMsg,
                    meta_topic_2,
                    lambda msg, token=stream_token: self._on_calibration_meta_msg_2(msg, token),
                    10,
                    **sub_kwargs,
                )
                self._calib_meta_topic_in_use_2 = meta_topic_2
            if self._vision_panel_needs_depth(2):
                depth_topic_2 = self._assigned_depth_topic_for_serial(self._vision_assigned_serial_2, node)
                self._vision_depth_sub_2 = node.create_subscription(
                    RosImageMsg, depth_topic_2, lambda msg, token=stream_token: self._on_vision_depth_msg_2(msg, token), image_qos, **sub_kwargs
                )
            else:
                depth_topic_2 = None
            if self._vision_panel_needs_camera_info(2) and RosCameraInfoMsg is not None:
                info_topic_2 = self._assigned_info_topic_for_serial(self._vision_assigned_serial_2, node)
                self._vision_info_sub_2 = node.create_subscription(
                    RosCameraInfoMsg,
                    info_topic_2,
                    lambda msg, token=stream_token: self._on_vision_camera_info_msg(msg, token, panel_index=2),
                    image_qos,
                    **sub_kwargs,
                )
                self.append_log(f"[비전2] 카메라정보 구독 시작: {info_topic_2}\n")
            self._vision_state_text_2 = "끊김"
            self.append_log(f"[비전2] 비전 원본 구독 시작({'CALIB' if self._calibration_mode_enabled_2 else 'RAW'}): {image_topic}\n")
            if (not calib_on) and self._vision_meta_topic_in_use_2:
                self.append_log(f"[비전2] 메타 구독 시작: {self._vision_meta_topic_in_use_2}\n")
            if calib_on and self._calib_meta_topic_in_use_2:
                self.append_log(f"[캘리브레이션2] 메타 구독 시작: {self._calib_meta_topic_in_use_2}\n")
            if depth_topic_2:
                self.append_log(f"[비전2] 뎁스 구독 시작: {depth_topic_2}\n")
        else:
            sub_kwargs = {}
            if self._vision_cb_group_1 is not None:
                sub_kwargs["callback_group"] = self._vision_cb_group_1
            image_topic = self._current_vision_image_topic()
            self._teardown_external_vision_bridge_panel(1)
            stream_token = self._vision_stream_token_1
            self._vision_sub = node.create_subscription(
                RosImageMsg, image_topic, lambda msg, token=stream_token: self._on_vision_image_msg(msg, token), image_qos, **sub_kwargs
            )
            self._vision_image_topic_in_use = image_topic
            if (not calib_on) and RosStringMsg is not None:
                meta_topic_runtime_1 = self._vision_output_meta_topic(1)
                self._vision_meta_sub_1 = node.create_subscription(
                    RosStringMsg,
                    meta_topic_runtime_1,
                    lambda msg, token=stream_token: self._on_vision_meta_msg(msg, token),
                    10,
                    **sub_kwargs,
                )
                self._vision_meta_topic_in_use_1 = meta_topic_runtime_1
            if calib_on and RosStringMsg is not None:
                meta_topic_1 = self._calibration_output_meta_topic(1)
                self._calib_meta_sub_1 = node.create_subscription(
                    RosStringMsg,
                    meta_topic_1,
                    lambda msg, token=stream_token: self._on_calibration_meta_msg(msg, token),
                    10,
                    **sub_kwargs,
                )
                self._calib_meta_topic_in_use_1 = meta_topic_1
            if self._vision_panel_needs_depth(1):
                depth_topic = self._assigned_depth_topic_for_serial(self._vision_assigned_serial_1, node)
                self._vision_depth_sub = node.create_subscription(
                    RosImageMsg, depth_topic, lambda msg, token=stream_token: self._on_vision_depth_msg(msg, token), image_qos, **sub_kwargs
                )
            else:
                depth_topic = None
            if self._vision_panel_needs_camera_info(1) and RosCameraInfoMsg is not None:
                info_topic = self._assigned_info_topic_for_serial(self._vision_assigned_serial_1, node)
                self._vision_info_sub = node.create_subscription(
                    RosCameraInfoMsg,
                    info_topic,
                    lambda msg, token=stream_token: self._on_vision_camera_info_msg(msg, token, panel_index=1),
                    image_qos,
                    **sub_kwargs,
                )
                self._append_vision_log(f"카메라정보 구독 시작: {info_topic}", panel_index=1)
            self._vision_state_text = "끊김"
            self._append_vision_log(
                f"비전 원본 구독 시작({'CALIB' if self._calibration_mode_enabled_1 else 'RAW'}): {image_topic}",
                panel_index=1,
            )
            if (not calib_on) and self._vision_meta_topic_in_use_1:
                self._append_vision_log(f"메타 구독 시작: {self._vision_meta_topic_in_use_1}", panel_index=1)
            if calib_on and self._calib_meta_topic_in_use_1:
                self.append_log(f"[캘리브레이션1] 메타 구독 시작: {self._calib_meta_topic_in_use_1}\n")
            if depth_topic:
                self._append_vision_log(f"뎁스 구독 시작: {depth_topic}", panel_index=1)
        return True

    def _rebind_external_vision_bridge_for_mode(self):
        if self.backend is None:
            return
        node = self._vision_subscription_node()
        if node is None:
            return
        self._teardown_external_vision_bridge()
        if not self._vision_panel_needs_depth(1):
            self._clear_vision_depth_cache(1)
        if not self._vision_panel_needs_depth(2):
            self._clear_vision_depth_cache(2)
        self._sync_calibration_processes()
        if YOLO_EXTERNAL_NODE and YOLO_AUTO_LAUNCH_NODE:
            self._ensure_external_vision_process()
        self._setup_external_vision_bridge()

    def _rebind_external_vision_bridge_panel_for_mode(self, panel_index: int):
        panel = 2 if int(panel_index) == 2 else 1
        if self.backend is None:
            return
        node = self._vision_subscription_node()
        if node is None:
            return
        self._teardown_external_vision_bridge_panel(panel)
        if not self._vision_panel_needs_depth(panel):
            self._clear_vision_depth_cache(panel)
        self._sync_calibration_process_panel(panel)
        if YOLO_EXTERNAL_NODE and YOLO_AUTO_LAUNCH_NODE:
            if panel == 2:
                if bool(self._top_status_enabled.get("vision2", True)):
                    self._ensure_external_vision_process_panel(2)
                else:
                    self._stop_external_vision_process_panel(2)
            else:
                if bool(self._top_status_enabled.get("vision", True)):
                    self._ensure_external_vision_process_panel(1)
                else:
                    self._stop_external_vision_process_panel(1)
        self._setup_external_vision_bridge_panel(panel)

    def _setup_external_vision_bridge(self):
        if RosImageMsg is None:
            self._append_vision_log("sensor_msgs/Image import 실패")
            self._vision_state_text = "오류"
            self._vision_state_text_2 = "오류"
            return False

        node = self._vision_subscription_node()
        if node is None:
            self._append_vision_log("ROS 노드 없음: 비전 원본 구독 실패")
            self._vision_state_text = "오류"
            self._vision_state_text_2 = "오류"
            return False

        try:
            ok = False
            if bool(self._top_status_enabled.get("vision", True)):
                ok = self._setup_external_vision_bridge_panel(1) or ok
            else:
                self._teardown_external_vision_bridge_panel(1)
            if bool(self._top_status_enabled.get("vision2", True)):
                ok = self._setup_external_vision_bridge_panel(2) or ok
            else:
                self._teardown_external_vision_bridge_panel(2)
            return ok
        except Exception as e:
            now = time.monotonic()
            msg = str(e)
            if (msg != self._vision_bridge_fail_last_msg) or ((now - self._vision_bridge_fail_last_at) > 5.0):
                if self._calibration_mode_enabled_1 or self._calibration_mode_enabled_2:
                    self._append_vision_log(f"캘리브레이션 비전 대기: {e}")
                else:
                    self._append_vision_log(f"비전 원본 구독 실패: {e}")
                self._vision_bridge_fail_last_msg = msg
                self._vision_bridge_fail_last_at = now
            self._vision_state_text = "대기" if self._calibration_mode_enabled_1 else "오류"
            self._vision_state_text_2 = "대기" if self._calibration_mode_enabled_2 else "오류"
            return False


    def _draw_vision_axes_overlay(self, bgr, panel_index: int = 1):
        try:
            import cv2
        except Exception:
            return bgr
        h, w = bgr.shape[:2]
        if h <= 2 or w <= 2:
            return bgr
        # 비전 XYZ(mm) 좌표계(카메라 광학좌표)를 영상 위에 투영해 표시한다.
        # 원점은 광학중심(cx, cy), 방향은 X+ 우측 / Y+ 하향.
        panel = 2 if int(panel_index) == 2 else 1
        intr = self._camera_intrinsics_2 if panel == 2 else self._camera_intrinsics_1
        if intr is None and panel == 1:
            intr = self._camera_intrinsics
        if intr is not None and len(intr) >= 4:
            fx, fy, cx, cy = [float(v) for v in intr[:4]]
            ox = int(round(cx))
            oy = int(round(cy))
        else:
            fx = fy = None
            ox = int(w // 2)
            oy = int(h // 2)
        ox = max(1, min(w - 2, ox))
        oy = max(1, min(h - 2, oy))
        color = (0, 255, 0)
        tip_color = (0, 0, 255)
        thickness = 2

        # 오버레이 크기는 사용자 인지용으로 고정하고, 축선은 점선으로 화면 끝까지 연장한다.
        axis_len_px = max(14, int(min(w, h) * (1.0 / 3.0)))
        x_end = min(w - 2, max(1, ox + axis_len_px))
        y_end = min(h - 2, max(1, oy + axis_len_px))

        # Dashed axis lines to both edges
        dash = max(6, int(min(w, h) * 0.015))
        gap = max(4, int(dash * 0.7))
        xx = 1
        while xx < (w - 1):
            x2 = min(w - 2, xx + dash)
            cv2.line(bgr, (xx, oy), (x2, oy), color, thickness, cv2.LINE_AA)
            xx = x2 + gap
        yy = 1
        while yy < (h - 1):
            y2 = min(h - 2, yy + dash)
            cv2.line(bgr, (ox, yy), (ox, y2), color, thickness, cv2.LINE_AA)
            yy = y2 + gap
        # 화살표 촉만 빨간색으로 강조
        tip_len_x = max(8, int(w * 0.04))
        tip_len_y = max(8, int(h * 0.04))
        cv2.arrowedLine(
            bgr,
            (max(ox, (w - 2) - tip_len_x), oy),
            (w - 2, oy),
            tip_color,
            thickness,
            cv2.LINE_AA,
            tipLength=0.45,
        )
        cv2.arrowedLine(
            bgr,
            (ox, max(oy, (h - 2) - tip_len_y)),
            (ox, h - 2),
            tip_color,
            thickness,
            cv2.LINE_AA,
            tipLength=0.45,
        )

        # 원점 표시(빨간색)
        cv2.circle(bgr, (ox, oy), 4, (0, 0, 255), -1, cv2.LINE_AA)

        cv2.putText(
            bgr,
            "X+",
            (max(8, min(w - 40, (w - 2) - 24)), max(18, min(h - 8, oy + 26))),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.80,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            bgr,
            "X+",
            (max(8, min(w - 40, (w - 2) - 24)), max(18, min(h - 8, oy + 26))),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.80,
            color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            bgr,
            "Y+",
            (max(8, min(w - 40, ox + 8)), max(18, min(h - 8, (h - 2) - 8))),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.80,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            bgr,
            "Y+",
            (max(8, min(w - 40, ox + 8)), max(18, min(h - 8, (h - 2) - 8))),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.80,
            color,
            2,
            cv2.LINE_AA,
        )
        return bgr

    def _draw_calib_info_overlay(self, bgr, data_dict):
        # 캘리브레이션 좌표 텍스트는 Qt 캔버스(좌하단)에서 정방향으로 별도 표시한다.
        return bgr

    def _vision_payload_has_runtime_data(self, panel_index: int, payload):
        if not isinstance(payload, dict):
            return False
        panel = 2 if int(panel_index) == 2 else 1
        detections = payload.get("detections")
        if isinstance(detections, list) and len(detections) > 0:
            return True
        if panel == 2:
            bottle = payload.get("bottle")
            liquid = payload.get("liquid")
            if isinstance(bottle, dict) or isinstance(liquid, dict):
                return True
            if bool(payload.get("volume_ready", False)):
                return True
        return False

    def _current_vision_meta_payload(self, panel_index: int = 1):
        panel = 2 if int(panel_index) == 2 else 1
        payload = getattr(self, "_vision_meta_payload_2", None) if panel == 2 else getattr(self, "_vision_meta_payload_1", None)
        received_at = getattr(self, "_vision_meta_received_at_2", None) if panel == 2 else getattr(self, "_vision_meta_received_at_1", None)
        last_nonempty = getattr(self, "_vision_meta_last_nonempty_payload_2", None) if panel == 2 else getattr(self, "_vision_meta_last_nonempty_payload_1", None)
        last_nonempty_at = getattr(self, "_vision_meta_last_nonempty_at_2", None) if panel == 2 else getattr(self, "_vision_meta_last_nonempty_at_1", None)
        if payload is not None:
            if self._vision_payload_has_runtime_data(panel, payload):
                return payload
            if (
                last_nonempty is not None
                and last_nonempty_at is not None
                and received_at is not None
                and float(last_nonempty_at) > float(received_at)
            ):
                return last_nonempty
            return payload
        if last_nonempty is not None and last_nonempty_at is not None:
            return last_nonempty
        return None

    def _current_vision_meta_payload_for_ui(self, panel_index: int = 1):
        panel = 2 if int(panel_index) == 2 else 1
        payload = self._current_vision_meta_payload(panel)
        if isinstance(payload, dict):
            return payload
        now = time.monotonic()
        if panel == 2:
            last_nonempty = getattr(self, "_vision_meta_last_nonempty_payload_2", None)
            last_nonempty_at = getattr(self, "_vision_meta_last_nonempty_at_2", None)
        else:
            last_nonempty = getattr(self, "_vision_meta_last_nonempty_payload_1", None)
            last_nonempty_at = getattr(self, "_vision_meta_last_nonempty_at_1", None)
        if last_nonempty is not None and last_nonempty_at is not None:
            if (now - float(last_nonempty_at)) <= float(max(VISION_META_STALE_SEC, VISION_META_HOLD_SEC, VISION_RUNTIME_UI_HOLD_SEC)):
                return last_nonempty
        return None

    def _draw_detection_contour(self, overlay, contour_uv, color, thickness=2):
        try:
            import cv2
        except Exception:
            return
        if not contour_uv:
            return
        try:
            pts = np.asarray(contour_uv, dtype=np.int32).reshape((-1, 1, 2))
        except Exception:
            return
        if pts.shape[0] < 3:
            return
        cv2.polylines(overlay, [pts], True, color, thickness, cv2.LINE_AA)

    def _fill_detection_contour(self, overlay, contour_uv, color, alpha=0.18):
        try:
            import cv2
        except Exception:
            return
        if not contour_uv:
            return
        try:
            pts = np.asarray(contour_uv, dtype=np.int32).reshape((-1, 1, 2))
        except Exception:
            return
        if pts.shape[0] < 3:
            return
        layer = overlay.copy()
        cv2.fillPoly(layer, [pts], color, cv2.LINE_AA)
        cv2.addWeighted(layer, float(alpha), overlay, 1.0 - float(alpha), 0.0, dst=overlay)

    def _rotate_bgr_for_overlay(self, bgr, rot_deg: int):
        try:
            import cv2
        except Exception:
            return bgr
        rot = self._normalize_rotation_deg(rot_deg)
        if bgr is None or rot == 0:
            return bgr
        if rot == 90:
            return cv2.rotate(bgr, cv2.ROTATE_90_CLOCKWISE)
        if rot == 180:
            return cv2.rotate(bgr, cv2.ROTATE_180)
        if rot == 270:
            return cv2.rotate(bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return bgr

    def _inverse_rotation_deg(self, rot_deg: int):
        rot = self._normalize_rotation_deg(rot_deg)
        return (360 - rot) % 360

    def _map_source_bbox_to_rotated(self, bbox_xyxy, src_w: int, src_h: int, rot_deg: int):
        if not isinstance(bbox_xyxy, (list, tuple)) or len(bbox_xyxy) < 4:
            return None
        try:
            x1, y1, x2, y2 = [int(round(float(v))) for v in bbox_xyxy[:4]]
        except Exception:
            return None
        corners = [
            self._map_source_to_rotated_coords(x1, y1, src_w, src_h, rot_deg),
            self._map_source_to_rotated_coords(x2, y1, src_w, src_h, rot_deg),
            self._map_source_to_rotated_coords(x2, y2, src_w, src_h, rot_deg),
            self._map_source_to_rotated_coords(x1, y2, src_w, src_h, rot_deg),
        ]
        xs = [int(p[0]) for p in corners]
        ys = [int(p[1]) for p in corners]
        return [int(min(xs)), int(min(ys)), int(max(xs)), int(max(ys))]

    def _map_source_contour_to_rotated(self, contour_uv, src_w: int, src_h: int, rot_deg: int):
        if not contour_uv:
            return None
        out = []
        for pt in contour_uv:
            if not isinstance(pt, (list, tuple)) or len(pt) < 2:
                continue
            try:
                mapped = self._map_source_to_rotated_coords(
                    int(round(float(pt[0]))),
                    int(round(float(pt[1]))),
                    int(src_w),
                    int(src_h),
                    int(rot_deg),
                )
            except Exception:
                continue
            out.append([int(mapped[0]), int(mapped[1])])
        return out if len(out) >= 3 else None

    def _draw_outlined_text(self, overlay, text: str, origin, color, font_scale=0.8, thickness=2):
        try:
            import cv2
        except Exception:
            return
        if not text:
            return
        try:
            x = int(round(float(origin[0])))
            y = int(round(float(origin[1])))
        except Exception:
            return
        cv2.putText(overlay, str(text), (x, y), cv2.FONT_HERSHEY_SIMPLEX, float(font_scale), (0, 0, 0), int(thickness) + 2, cv2.LINE_AA)
        cv2.putText(overlay, str(text), (x, y), cv2.FONT_HERSHEY_SIMPLEX, float(font_scale), color, int(thickness), cv2.LINE_AA)

    def _measure_text_rect(self, shape, text: str, origin, font_scale=0.8, thickness=2, pad_x=0, pad_y=0):
        try:
            import cv2
        except Exception:
            return None
        if not text or shape is None or len(shape) < 2:
            return None
        try:
            x = int(round(float(origin[0])))
            y = int(round(float(origin[1])))
        except Exception:
            return None
        (text_w, text_h), baseline = cv2.getTextSize(str(text), cv2.FONT_HERSHEY_SIMPLEX, float(font_scale), int(thickness))
        width = int(shape[1])
        height = int(shape[0])
        x1 = max(0, x - int(pad_x))
        y1 = max(0, y - text_h - int(pad_y))
        x2 = min(width - 1, x + text_w + int(pad_x))
        y2 = min(height - 1, y + baseline + int(pad_y))
        return (x1, y1, x2, y2)

    def _rects_overlap(self, rect_a, rect_b):
        if rect_a is None or rect_b is None:
            return False
        ax1, ay1, ax2, ay2 = rect_a
        bx1, by1, bx2, by2 = rect_b
        return not (ax2 < bx1 or bx2 < ax1 or ay2 < by1 or by2 < ay1)

    def _draw_text_badge(self, overlay, text: str, origin, color, font_scale=0.8, thickness=2, bg_color=(0, 0, 0)):
        try:
            import cv2
        except Exception:
            return
        if not text:
            return
        try:
            x = int(round(float(origin[0])))
            y = int(round(float(origin[1])))
        except Exception:
            return
        font = cv2.FONT_HERSHEY_SIMPLEX
        (text_w, text_h), baseline = cv2.getTextSize(str(text), font, float(font_scale), int(thickness))
        pad_x = 8
        pad_y = 6
        x1 = max(0, x - pad_x)
        y1 = max(0, y - text_h - pad_y)
        x2 = min(overlay.shape[1] - 1, x + text_w + pad_x)
        y2 = min(overlay.shape[0] - 1, y + baseline + 2)
        layer = overlay.copy()
        cv2.rectangle(layer, (x1, y1), (x2, y2), bg_color, -1, cv2.LINE_AA)
        cv2.addWeighted(layer, 0.65, overlay, 0.35, 0.0, dst=overlay)
        cv2.putText(overlay, str(text), (x, y), font, float(font_scale), color, int(thickness), cv2.LINE_AA)

    def _draw_vision_runtime_meta_overlay(self, bgr, panel_index: int = 1):
        try:
            import cv2
        except Exception:
            return bgr
        payload = self._current_vision_meta_payload(panel_index)
        if not isinstance(payload, dict):
            return bgr
        panel = 2 if int(panel_index) == 2 else 1
        rot_deg = int(self._vision_rotation_deg_2) if panel == 2 else int(self._vision_rotation_deg_1)
        src_h, src_w = bgr.shape[:2]
        overlay = self._rotate_bgr_for_overlay(bgr.copy(), rot_deg)
        if panel == 1:
            detections = payload.get("detections")
            if not isinstance(detections, list):
                return bgr
            for det in detections:
                if not isinstance(det, dict):
                    continue
                bbox = self._map_source_bbox_to_rotated(det.get("bbox_xyxy"), src_w, src_h, rot_deg)
                if bbox is None:
                    continue
                x1, y1, x2, y2 = bbox
                class_name = str(det.get("class_name", "?"))
                depth_m = det.get("depth_m")
                label = class_name
                if depth_m is not None:
                    try:
                        label += f"/{float(depth_m):.2f}m"
                    except Exception:
                        pass
                color = (252, 119, 30)
                cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
                self._draw_outlined_text(overlay, label, (x1, max(18, y1 - 10)), color, font_scale=0.8, thickness=2)
            return self._rotate_bgr_for_overlay(overlay, self._inverse_rotation_deg(rot_deg))

        color_map = {
            "bottle": (255, 0, 0),
            "soju": (0, 0, 255),
            "beer": (0, 255, 0),
            "juice": (0, 255, 0),
        }
        occupied_label_rects = []
        detections = payload.get("detections")
        if isinstance(detections, list):
            for det in detections:
                if not isinstance(det, dict):
                    continue
                class_name = str(det.get("class_name", "") or "")
                color = color_map.get(class_name, (255, 255, 255))
                contour_uv = self._map_source_contour_to_rotated(det.get("contour_uv"), src_w, src_h, rot_deg)
                self._fill_detection_contour(overlay, contour_uv, color, alpha=0.20)
                self._draw_detection_contour(overlay, contour_uv, color, thickness=2)
                bbox = self._map_source_bbox_to_rotated(det.get("bbox_xyxy"), src_w, src_h, rot_deg)
                if bbox is None:
                    continue
                x1, y1, x2, y2 = bbox
                cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
                label = class_name
                conf = det.get("confidence")
                if conf is not None:
                    try:
                        label += f" {float(conf):.2f}"
                    except Exception:
                        pass
                label_origin = (x1, max(18, y1 - 10))
                self._draw_outlined_text(overlay, label, label_origin, color, font_scale=0.8, thickness=2)
                occupied_label_rects.append(
                    self._measure_text_rect(overlay.shape, label, label_origin, font_scale=0.8, thickness=2, pad_x=4, pad_y=4)
                )

        liquid = payload.get("liquid")
        if isinstance(liquid, dict):
            bbox = self._map_source_bbox_to_rotated(liquid.get("bbox_xyxy"), src_w, src_h, rot_deg)
            volume_ml = liquid.get("volume_ml")
            if bbox is not None and volume_ml is not None:
                try:
                    x1, y1, x2, _y2 = bbox
                    liquid_color = color_map.get(str(liquid.get("class_name", "")), (0, 255, 0))
                    volume_text = f"volume={float(volume_ml):.1f}ml"
                    candidate_origins = [
                        (max(12, overlay.shape[1] - 190), 38),
                        (max(8, min(overlay.shape[1] - 190, x2 + 12)), max(30, y1 + 24)),
                        (max(8, x1 + 12), max(30, _y2 + 28)),
                        (max(8, x1 + 12), max(30, y1 - 16)),
                    ]
                    picked_origin = candidate_origins[0]
                    for origin in candidate_origins:
                        rect = self._measure_text_rect(
                            overlay.shape,
                            volume_text,
                            origin,
                            font_scale=0.8,
                            thickness=2,
                            pad_x=8,
                            pad_y=6,
                        )
                        if rect is None:
                            continue
                        if any(self._rects_overlap(rect, occ) for occ in occupied_label_rects):
                            continue
                        picked_origin = origin
                        break
                    self._draw_text_badge(
                        overlay,
                        volume_text,
                        picked_origin,
                        liquid_color,
                        font_scale=0.8,
                        thickness=2,
                        bg_color=(24, 24, 24),
                    )
                except Exception:
                    pass
        return self._rotate_bgr_for_overlay(overlay, self._inverse_rotation_deg(rot_deg))

    def _extract_calib_center_uvz(self, data_dict):
        if not isinstance(data_dict, dict):
            return None
        vals = data_dict.get("center")
        if vals is not None and len(vals) >= 3:
            try:
                return float(vals[0]), float(vals[1]), float(vals[2])
            except Exception:
                pass
        pts = []
        for k in ("p1", "p2", "p3", "p4"):
            v = data_dict.get(k)
            if v is None or len(v) < 2:
                return None
            try:
                pts.append((float(v[0]), float(v[1])))
            except Exception:
                return None
        if len(pts) != 4:
            return None
        cu = sum(p[0] for p in pts) / 4.0
        cv = sum(p[1] for p in pts) / 4.0
        z_vals = []
        for k in ("p1", "p2", "p3", "p4", "p5"):
            v = data_dict.get(k)
            if v is None or len(v) < 3:
                continue
            try:
                zz = float(v[2])
                if np.isfinite(zz):
                    z_vals.append(zz)
            except Exception:
                continue
        cz = float(np.mean(z_vals)) if z_vals else float("nan")
        return cu, cv, cz

    def _draw_calib_points_only(self, bgr, data_dict, panel_index: int = 1):
        try:
            import cv2
        except Exception:
            return bgr
        if not isinstance(data_dict, dict):
            return bgr
        panel = 2 if int(panel_index) == 2 else 1
        pts_grid = self._calib_last_grid_pts_2 if panel == 2 else self._calib_last_grid_pts_1
        grid_status = self._calib_last_grid_status_2 if panel == 2 else self._calib_last_grid_status_1
        # 체커보드 그물 구조(인접 코너 연결선)
        if pts_grid is not None:
            try:
                grid = np.asarray(pts_grid, dtype=np.float32)
                rows, cols = grid.shape[:2]
                # OpenCV 체커보드는 내부 코너만 반환하므로, 표시용으로 바깥 1칸을 외삽해
                # 실제 보드의 가장자리까지 포함한 그물 구조를 그린다.
                if rows >= 2 and cols >= 2:
                    expanded = np.zeros((rows + 2, cols + 2, 2), dtype=np.float32)
                    expanded[1 : rows + 1, 1 : cols + 1] = grid
                    # 내부 코너 기반이므로 바깥 가장자리선은 반칸(0.5 step) 외삽으로 그린다.
                    expanded[0, 1 : cols + 1] = 1.5 * grid[0, :] - 0.5 * grid[1, :]
                    expanded[rows + 1, 1 : cols + 1] = 1.5 * grid[rows - 1, :] - 0.5 * grid[rows - 2, :]
                    expanded[:, 0] = 1.5 * expanded[:, 1] - 0.5 * expanded[:, 2]
                    expanded[:, cols + 1] = 1.5 * expanded[:, cols] - 0.5 * expanded[:, cols - 1]
                    grid = expanded
                    rows, cols = grid.shape[:2]
                for rr in range(rows):
                    for cc in range(cols - 1):
                        p1 = grid[rr, cc]
                        p2 = grid[rr, cc + 1]
                        cv2.line(
                            bgr,
                            (int(round(float(p1[0]))), int(round(float(p1[1])))),
                            (int(round(float(p2[0]))), int(round(float(p2[1])))),
                            (0, 220, 0),
                            1,
                            cv2.LINE_AA,
                        )
                for cc in range(cols):
                    for rr in range(rows - 1):
                        p1 = grid[rr, cc]
                        p2 = grid[rr + 1, cc]
                        cv2.line(
                            bgr,
                            (int(round(float(p1[0]))), int(round(float(p1[1])))),
                            (int(round(float(p2[0]))), int(round(float(p2[1])))),
                            (0, 220, 0),
                            1,
                            cv2.LINE_AA,
                        )
            except Exception:
                pass
        # 실제 켈 시퀀스 데이터 취득점 상태 표시: 정상=초록, 비정상=빨강
        if grid_status:
            try:
                for u_f, v_f, ok_pt in list(grid_status):
                    cx = int(round(float(u_f)))
                    cy = int(round(float(v_f)))
                    color = (0, 220, 0) if bool(ok_pt) else (0, 0, 255)
                    cv2.circle(bgr, (cx, cy), 2, color, -1, cv2.LINE_AA)
            except Exception:
                pass
        center_uvz = self._extract_calib_center_uvz(data_dict)
        if center_uvz is None:
            return bgr
        cu, cv, _ = center_uvz
        cx = int(round(cu))
        cy = int(round(cv))
        # 중심점 1개만 강조
        cv2.circle(bgr, (cx, cy), 4, (0, 0, 255), -1, cv2.LINE_AA)
        return bgr

    def _draw_vision_update_ms_overlay(self, bgr):
        try:
            import cv2
        except Exception:
            return bgr
        h, w = bgr.shape[:2]
        if self._vision_cycle_ms is None or (not np.isfinite(self._vision_cycle_ms)):
            txt = "-ms"
        else:
            txt = f"{self._vision_cycle_ms:.1f}ms"
        fscale = 0.52
        thick = 1
        tsize, _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, fscale, thick)
        x = max(6, w - tsize[0] - 10)
        y = 20
        cv2.putText(
            bgr,
            txt,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            fscale,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            bgr,
            txt,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            fscale,
            (0, 255, 0),
            thick,
            cv2.LINE_AA,
        )
        return bgr

    def _draw_robot_frame_icon_overlay(self, bgr):
        try:
            import cv2
        except Exception:
            return bgr
        h, w = bgr.shape[:2]
        if h < 90 or w < 180:
            return bgr

        # 로봇 프레임 원점을 비전 좌표계 Y축 최상단 우측에 둔다.
        cx = int(w * 0.5)
        oy = 22
        ox = min(w - 28, cx + 64)

        frame_color = (0, 255, 255)  # yellow

        cv2.putText(
            bgr,
            "[ROBOT FRAME]",
            (max(8, ox - 82), max(18, oy - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            frame_color,
            2,
            cv2.LINE_AA,
        )

        # axis origin
        cv2.circle(bgr, (ox, oy), 2, frame_color, -1, cv2.LINE_AA)
        # draw X+/Y+ directions from current vision->robot transform
        # row-vector model: robot = camera @ R_row + t
        # robot-axis in camera frame = columns of R_row
        r = None
        try:
            aff = getattr(self, "_vision_to_robot_affine", None)
            if aff is not None:
                arr = np.asarray(aff, dtype=np.float64)
                if arr.shape == (4, 3):
                    r = arr[:3, :]
        except Exception:
            r = None

        def _dir2_from_cam3(v3):
            vx = float(v3[0])
            vy = float(v3[1])
            n = float(np.hypot(vx, vy))
            if n < 1e-9:
                return None
            return np.array([vx / n, vy / n], dtype=np.float64)

        x2 = None
        y2 = None
        if r is not None:
            x_cam = r[:, 0]
            y_cam = r[:, 1]
            x2 = _dir2_from_cam3(x_cam)
            y2 = _dir2_from_cam3(y_cam)

        if x2 is None:
            x2 = np.array([1.0, 0.0], dtype=np.float64)
        if y2 is None:
            y2 = np.array([0.0, 1.0], dtype=np.float64)

        len_x = 62.0
        len_y = 50.0
        x_end = (int(round(ox + x2[0] * len_x)), int(round(oy + x2[1] * len_x)))
        y_end = (int(round(ox + y2[0] * len_y)), int(round(oy + y2[1] * len_y)))

        cv2.arrowedLine(bgr, (ox, oy), x_end, frame_color, 2, cv2.LINE_AA, tipLength=0.2)
        cv2.arrowedLine(bgr, (ox, oy), y_end, frame_color, 2, cv2.LINE_AA, tipLength=0.2)
        cv2.putText(
            bgr,
            "X+",
            (x_end[0] + 4, x_end[1] + 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            frame_color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            bgr,
            "Y+",
            (y_end[0] + 4, y_end[1] + 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            frame_color,
            2,
            cv2.LINE_AA,
        )
        return bgr

    def _on_vision_image_msg(self, msg, stream_token=None):
        if not bool(self._top_status_enabled.get("vision", True)):
            return
        try:
            decode_started_at = time.monotonic()
            if stream_token is not None and int(stream_token) != int(getattr(self, "_vision_stream_token_1", 0)):
                return
            now = time.monotonic()
            self._last_camera_frame_at_1 = now
            if now < float(getattr(self, "_vision_drop_frames_until_1", 0.0)):
                return
            if VISION_DECODE_MIN_INTERVAL_SEC > 0.0 and now < float(getattr(self, "_vision_decode_next_at_1", 0.0)):
                return
            if VISION_DECODE_MIN_INTERVAL_SEC > 0.0:
                self._vision_decode_next_at_1 = now + float(VISION_DECODE_MIN_INTERVAL_SEC)

            h = int(msg.height)
            w = int(msg.width)
            step = int(msg.step)
            if h <= 0 or w <= 0:
                return

            buf = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding in ("rgb8", "bgr8"):
                need = w * 3
                if step < need or buf.size < h * step:
                    return
                arr = buf.reshape((h, step))[:, :need].reshape((h, w, 3))
                bgr = arr[:, :, ::-1] if msg.encoding == "rgb8" else arr
                bgr = np.ascontiguousarray(bgr)
                self._last_yolo_image_size = (w, h)
                self._cache_latest_raw_vision_frame(1, bgr)
            elif msg.encoding in ("rgba8", "bgra8"):
                need = w * 4
                if step < need or buf.size < h * step:
                    return
                arr = buf.reshape((h, step))[:, :need].reshape((h, w, 4))
                if msg.encoding == "bgra8":
                    bgr = np.ascontiguousarray(arr[:, :, :3])
                else:
                    bgr = np.ascontiguousarray(arr[:, :, :3][:, :, ::-1])
                self._last_yolo_image_size = (w, h)
                self._cache_latest_raw_vision_frame(1, bgr)
            else:
                return

            self._vision_decode_ms = (time.monotonic() - decode_started_at) * 1000.0
            self._queue_vision_frame_for_compose(1, bgr, now=now, stream_token=stream_token)
        except Exception:
            return

    def _on_vision_image_msg_2(self, msg, stream_token=None):
        if not bool(self._top_status_enabled.get("vision2", True)):
            return
        try:
            decode_started_at = time.monotonic()
            if stream_token is not None and int(stream_token) != int(getattr(self, "_vision_stream_token_2", 0)):
                return
            now = time.monotonic()
            self._last_camera_frame_at_2 = now
            if now < float(getattr(self, "_vision_drop_frames_until_2", 0.0)):
                return
            if VISION_DECODE_MIN_INTERVAL_SEC > 0.0 and now < float(getattr(self, "_vision_decode_next_at_2", 0.0)):
                return
            if VISION_DECODE_MIN_INTERVAL_SEC > 0.0:
                self._vision_decode_next_at_2 = now + float(VISION_DECODE_MIN_INTERVAL_SEC)
            h = int(msg.height)
            w = int(msg.width)
            step = int(msg.step)
            if h <= 0 or w <= 0:
                return

            buf = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding in ("rgb8", "bgr8"):
                need = w * 3
                if step < need or buf.size < h * step:
                    return
                arr = buf.reshape((h, step))[:, :need].reshape((h, w, 3))
                bgr = arr[:, :, ::-1] if msg.encoding == "rgb8" else arr
                bgr = np.ascontiguousarray(bgr)
                self._last_yolo_image_size_2 = (w, h)
                self._cache_latest_raw_vision_frame(2, bgr)
            elif msg.encoding in ("rgba8", "bgra8"):
                need = w * 4
                if step < need or buf.size < h * step:
                    return
                arr = buf.reshape((h, step))[:, :need].reshape((h, w, 4))
                if msg.encoding == "bgra8":
                    bgr = np.ascontiguousarray(arr[:, :, :3])
                else:
                    bgr = np.ascontiguousarray(arr[:, :, :3][:, :, ::-1])
                self._last_yolo_image_size_2 = (w, h)
                self._cache_latest_raw_vision_frame(2, bgr)
            else:
                return

            self._vision_decode_ms_2 = (time.monotonic() - decode_started_at) * 1000.0
            self._queue_vision_frame_for_compose(2, bgr, now=now, stream_token=stream_token)
        except Exception:
            return

    def _on_vision_depth_msg(self, msg, stream_token=None):
        try:
            if stream_token is not None and int(stream_token) != int(getattr(self, "_vision_stream_token_1", 0)):
                return
            self._last_camera_frame_at_1 = time.monotonic()
            h = int(msg.height)
            w = int(msg.width)
            step = int(msg.step)
            if h <= 0 or w <= 0:
                return

            enc = str(msg.encoding).lower()
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            if enc in ("16uc1", "mono16"):
                need = w * 2
                if step < need or buf.size < h * step:
                    return
                arr = buf.reshape((h, step))[:, :need].reshape((h, w, 2))
                depth = arr[:, :, 0].astype(np.uint16) | (arr[:, :, 1].astype(np.uint16) << 8)
                self._vision_depth_image = np.ascontiguousarray(depth)
                self._vision_depth_encoding = "16UC1"
                self._vision_depth_shape = (h, w)
                return

            if enc == "32fc1":
                need = w * 4
                if step < need or buf.size < h * step:
                    return
                depth = np.frombuffer(msg.data, dtype=np.float32, count=h * w)
                depth = depth.reshape((h, w))
                self._vision_depth_image = np.ascontiguousarray(depth)
                self._vision_depth_encoding = "32FC1"
                self._vision_depth_shape = (h, w)
                return
        except Exception:
            return

    def _on_vision_depth_msg_2(self, msg, stream_token=None):
        try:
            if stream_token is not None and int(stream_token) != int(getattr(self, "_vision_stream_token_2", 0)):
                return
            self._last_camera_frame_at_2 = time.monotonic()
            h = int(msg.height)
            w = int(msg.width)
            step = int(msg.step)
            if h <= 0 or w <= 0:
                return

            enc = str(msg.encoding).lower()
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            if enc in ("16uc1", "mono16"):
                need = w * 2
                if step < need or buf.size < h * step:
                    return
                arr = buf.reshape((h, step))[:, :need].reshape((h, w, 2))
                depth = arr[:, :, 0].astype(np.uint16) | (arr[:, :, 1].astype(np.uint16) << 8)
                self._vision_depth_image_2 = np.ascontiguousarray(depth)
                self._vision_depth_encoding_2 = "16UC1"
                self._vision_depth_shape_2 = (h, w)
                return

            if enc == "32fc1":
                need = w * 4
                if step < need or buf.size < h * step:
                    return
                depth = np.frombuffer(msg.data, dtype=np.float32, count=h * w)
                depth = depth.reshape((h, w))
                self._vision_depth_image_2 = np.ascontiguousarray(depth)
                self._vision_depth_encoding_2 = "32FC1"
                self._vision_depth_shape_2 = (h, w)
                return
        except Exception:
            return

    def _on_vision_camera_info_msg(self, msg, stream_token=None, panel_index: int = 1):
        try:
            panel = 2 if int(panel_index) == 2 else 1
            current_token = int(getattr(self, "_vision_stream_token_2", 0)) if panel == 2 else int(getattr(self, "_vision_stream_token_1", 0))
            if stream_token is not None and int(stream_token) != current_token:
                return
            k = getattr(msg, "k", None)
            if k is None or len(k) < 9:
                return
            fx = float(k[0])
            fy = float(k[4])
            cx = float(k[2])
            cy = float(k[5])
            if fx <= 1e-9 or fy <= 1e-9:
                return
            intr = (fx, fy, cx, cy)
            if panel == 2:
                self._camera_intrinsics_2 = intr
            else:
                self._camera_intrinsics_1 = intr
                self._camera_intrinsics = intr
        except Exception:
            return

    def _uvz_to_camera_xyz_mm(self, u, v, z_mm, require_intrinsics=False, panel_index: int = 1):
        panel = 2 if int(panel_index) == 2 else 1
        intr = self._camera_intrinsics_2 if panel == 2 else self._camera_intrinsics_1
        if intr is None and panel == 1:
            intr = self._camera_intrinsics
        if intr is None:
            if require_intrinsics:
                return None
            return float(u), float(v), float(z_mm)
        fx, fy, cx, cy = intr
        z = float(z_mm)
        x = (float(u) - cx) * z / fx
        # Camera optical frame: X right(+), Y down(+), Z forward(+)
        y = (float(v) - cy) * z / fy
        return float(x), float(y), float(z)

    def _depth_m_from_image_coord(self, ix: int, iy: int, panel_index: int = 1):
        if int(panel_index) == 2:
            depth_image = self._vision_depth_image_2
            depth_shape = self._vision_depth_shape_2
            depth_encoding = self._vision_depth_encoding_2
            image_size = self._last_yolo_image_size_2
        else:
            depth_image = self._vision_depth_image
            depth_shape = self._vision_depth_shape
            depth_encoding = self._vision_depth_encoding
            image_size = self._last_yolo_image_size
        if depth_image is None or depth_shape is None:
            return None
        if image_size is None:
            return None

        src_w, src_h = image_size
        dep_h, dep_w = depth_shape
        if src_w <= 0 or src_h <= 0 or dep_w <= 0 or dep_h <= 0:
            return None

        dx = int(round(ix * (dep_w - 1) / max(1, src_w - 1)))
        dy = int(round(iy * (dep_h - 1) / max(1, src_h - 1)))
        dx = max(0, min(dep_w - 1, dx))
        dy = max(0, min(dep_h - 1, dy))

        half = 2
        x1 = max(0, dx - half)
        x2 = min(dep_w, dx + half + 1)
        y1 = max(0, dy - half)
        y2 = min(dep_h, dy + half + 1)
        roi = depth_image[y1:y2, x1:x2]
        if roi.size == 0:
            return None

        valid = roi[np.isfinite(roi) & (roi > 0)]
        if valid.size < 3:
            return None

        z = float(np.median(valid))
        if depth_encoding == "16UC1":
            z = z / 1000.0
        elif depth_encoding == "32FC1":
            z = z
        else:
            return None

        if z < 0.05 or z > 5.0:
            return None
        return z

    def _on_selected_point_msg(self, msg):
        try:
            self._last_vision_point_at = time.monotonic()
            if self._vision_coord_label is not None:
                self._vision_coord_label.setText(
                    f"선택점 X,Y[m] / Z[mm] : {msg.point.x:+.4f}, {msg.point.y:+.4f}, {msg.point.z * 1000.0:.1f}"
                )
                self._vision_coord_label.show()
        except Exception:
            return

    def _on_preview_point_msg(self, msg):
        try:
            # selected_point 직후 잠깐은 선택 좌표 표기를 유지한다.
            now = time.monotonic()
            if self._last_vision_point_at is not None and (now - self._last_vision_point_at) < 0.7:
                return
            if self._vision_coord_label is not None:
                self._vision_coord_label.setText(
                    f"프리뷰 X,Y[m] / Z[mm] : {msg.point.x:+.4f}, {msg.point.y:+.4f}, {msg.point.z * 1000.0:.1f}"
                )
                self._vision_coord_label.show()
        except Exception:
            return

    def _update_yolo_view(self, image: QImage):
        self._enqueue_vision_frame(image)

    def _enqueue_vision_frame(self, image: QImage, now=None, stream_token=None):
        if not hasattr(self, "yolo_view"):
            return
        t_now = time.monotonic() if now is None else float(now)
        if self._vision_prev_update_at is not None:
            dt = t_now - self._vision_prev_update_at
            if dt > 0.0:
                self._vision_cycle_ms = dt * 1000.0
        self._vision_prev_update_at = t_now
        self._last_yolo_image_size = (image.width(), image.height())
        with self._vision_frame_lock_1:
            self._pending_yolo_qimage = image
            self._pending_vision_token_1 = int(self._vision_stream_token_1 if stream_token is None else stream_token)
            self._pending_vision_enqueued_at_1 = t_now
            self._vision_frame_pending = True
        self._last_vision_frame_at = t_now
        self._vision_state_text = "정상 수신 중"

    def _enqueue_vision_frame_2(self, image: QImage, now=None, stream_token=None):
        if not hasattr(self, "yolo_view_2"):
            return
        t_now = time.monotonic() if now is None else float(now)
        if self._vision_prev_update_at_2 is not None:
            dt = t_now - self._vision_prev_update_at_2
            if dt > 0.0:
                self._vision_cycle_ms_2 = dt * 1000.0
        self._vision_prev_update_at_2 = t_now
        self._last_yolo_image_size_2 = (image.width(), image.height())
        with self._vision_frame_lock_2:
            self._pending_yolo_qimage_2 = image
            self._pending_vision_token_2 = int(self._vision_stream_token_2 if stream_token is None else stream_token)
            self._pending_vision_enqueued_at_2 = t_now
            self._vision_frame_pending_2 = True
        self._last_vision_frame_at_2 = t_now
        self._vision_state_text_2 = "정상 수신 중"

    def _maybe_log_vision_render_stall(self, panel_index: int, render_delay_ms):
        if render_delay_ms is None:
            return
        delay_ms = float(render_delay_ms)
        if delay_ms < float(VISION_RENDER_STALL_WARN_MS):
            return
        panel = 2 if int(panel_index) == 2 else 1
        now = time.monotonic()
        attr = "_vision_render_stall_last_log_at_2" if panel == 2 else "_vision_render_stall_last_log_at_1"
        last_at = float(getattr(self, attr, 0.0) or 0.0)
        if (now - last_at) < float(VISION_RENDER_STALL_LOG_COOLDOWN_SEC):
            return
        setattr(self, attr, now)
        decode_ms = self._vision_decode_ms_2 if panel == 2 else self._vision_decode_ms
        compose_ms = self._vision_compose_ms_2 if panel == 2 else self._vision_compose_ms
        self.append_log(
            f"[UI-PERF][비전{panel}] render_delay={delay_ms:.1f}ms "
            f"(decode={decode_ms if decode_ms is not None else '-'}ms, "
            f"compose={compose_ms if compose_ms is not None else '-'}ms)\n"
        )
        self._append_ui_perf_log_file(
            f"[비전{panel}] render_delay={delay_ms:.1f}ms "
            f"(decode={decode_ms if decode_ms is not None else '-'}ms, "
            f"compose={compose_ms if compose_ms is not None else '-'}ms)"
        )

    def _maybe_trace_vision_render_frame(self, panel_index: int):
        if not bool(VISION_RENDER_TRACE_ENABLED):
            return
        panel = 2 if int(panel_index) == 2 else 1
        now = time.monotonic()
        attr = "_vision_render_trace_last_log_at_2" if panel == 2 else "_vision_render_trace_last_log_at_1"
        last_at = float(getattr(self, attr, 0.0) or 0.0)
        if (now - last_at) < float(VISION_RENDER_TRACE_MIN_INTERVAL_SEC):
            return
        setattr(self, attr, now)
        if panel == 2:
            delay_ms = self._vision_render_delay_ms_2
            decode_ms = self._vision_decode_ms_2
            compose_ms = self._vision_compose_ms_2
            receive_ms = self._vision_cycle_ms_2
            render_interval_ms = self._vision_render_interval_ms_2
        else:
            delay_ms = self._vision_render_delay_ms
            decode_ms = self._vision_decode_ms
            compose_ms = self._vision_compose_ms
            receive_ms = self._vision_cycle_ms
            render_interval_ms = self._vision_render_interval_ms
        self._append_ui_perf_log_file(
            f"vision_frame_trace[{panel}] "
            f"delay={float(delay_ms) if delay_ms is not None else -1.0:.1f}ms "
            f"decode={float(decode_ms) if decode_ms is not None else -1.0:.1f}ms "
            f"compose={float(compose_ms) if compose_ms is not None else -1.0:.1f}ms "
            f"receive={float(receive_ms) if receive_ms is not None else -1.0:.1f}ms "
            f"render_interval={float(render_interval_ms) if render_interval_ms is not None else -1.0:.1f}ms"
        )

    def _drain_pending_vision_frame_1(self):
        with self._vision_frame_lock_1:
            image1 = self._pending_yolo_qimage if self._vision_frame_pending else None
            token1 = self._pending_vision_token_1 if self._vision_frame_pending else 0
            enqueued_at_1 = self._pending_vision_enqueued_at_1 if self._vision_frame_pending else None
            self._pending_yolo_qimage = None
            self._pending_vision_token_1 = 0
            self._pending_vision_enqueued_at_1 = None
            self._vision_frame_pending = False
        if image1 is not None and int(token1) == int(getattr(self, "_vision_stream_token_1", 0)):
            now = time.monotonic()
            if enqueued_at_1 is not None:
                self._vision_render_delay_ms = max(0.0, (now - float(enqueued_at_1)) * 1000.0)
                self._maybe_log_vision_render_stall(1, self._vision_render_delay_ms)
            if self._vision_render_prev_at is not None:
                dt = now - self._vision_render_prev_at
                if dt > 0.0:
                    self._vision_render_interval_ms = dt * 1000.0
            self._vision_render_prev_at = now
            self._last_yolo_qimage = image1
            self._maybe_trace_vision_render_frame(1)
            self._render_yolo_view()
            self._maybe_update_cycle_time_labels_from_vision()

    def _drain_pending_vision_frame_2(self):
        with self._vision_frame_lock_2:
            image2 = self._pending_yolo_qimage_2 if self._vision_frame_pending_2 else None
            token2 = self._pending_vision_token_2 if self._vision_frame_pending_2 else 0
            enqueued_at_2 = self._pending_vision_enqueued_at_2 if self._vision_frame_pending_2 else None
            self._pending_yolo_qimage_2 = None
            self._pending_vision_token_2 = 0
            self._pending_vision_enqueued_at_2 = None
            self._vision_frame_pending_2 = False
        if image2 is not None and int(token2) == int(getattr(self, "_vision_stream_token_2", 0)):
            now = time.monotonic()
            if enqueued_at_2 is not None:
                self._vision_render_delay_ms_2 = max(0.0, (now - float(enqueued_at_2)) * 1000.0)
                self._maybe_log_vision_render_stall(2, self._vision_render_delay_ms_2)
            if self._vision_render_prev_at_2 is not None:
                dt = now - self._vision_render_prev_at_2
                if dt > 0.0:
                    self._vision_render_interval_ms_2 = dt * 1000.0
            self._vision_render_prev_at_2 = now
            self._last_yolo_qimage_2 = image2
            self._maybe_trace_vision_render_frame(2)
            self._render_yolo_view_2()
            self._maybe_update_cycle_time_labels_from_vision()

    def _render_yolo_view(self):
        if not hasattr(self, "yolo_view"):
            return
        if self._last_yolo_qimage is None:
            self.yolo_view.clear()
            self.yolo_view.setPixmap(QPixmap())
            self.yolo_view.setAlignment(Qt.AlignCenter)
            self.yolo_view.setStyleSheet("background-color: #000000; color: #f3f3f3;")
            self.yolo_view.setText("이미지 데이터 없음")
            return
        view_main = getattr(self, "yolo_view", None)
        view_w = max(1, view_main.width())
        view_h = max(1, view_main.height())
        src_orig_w = max(1, self._last_yolo_qimage.width())
        src_orig_h = max(1, self._last_yolo_qimage.height())
        img_disp = self._rotate_image_for_panel(self._last_yolo_qimage, 1)
        src_w = max(1, img_disp.width())
        src_h = max(1, img_disp.height())

        fit_scale = max(view_w / float(src_w), view_h / float(src_h))
        scale = fit_scale * float(self._yolo_zoom)
        draw_w = int(round(src_w * scale))
        draw_h = int(round(src_h * scale))
        scaled = QPixmap.fromImage(img_disp).scaled(
            max(1, draw_w), max(1, draw_h), Qt.KeepAspectRatio, Qt.FastTransformation
        )

        canvas = QPixmap(view_w, view_h)
        canvas.fill(Qt.black)
        painter_x = 0
        painter_y = 0
        crop_x = 0
        crop_y = 0
        if self._yolo_pan_x <= -999999.0 or self._yolo_pan_y <= -999999.0:
            rot = self._normalize_rotation_deg(self._vision_rotation_deg_1)
            if rot == 90:
                org_x, org_y = src_w - 1, 0
            elif rot == 180:
                org_x, org_y = src_w - 1, src_h - 1
            elif rot == 270:
                org_x, org_y = 0, src_h - 1
            else:
                org_x, org_y = 0, 0
            base_x = float(max(0, draw_w - view_w)) / 2.0
            base_y = float(max(0, draw_h - view_h)) / 2.0
            desired_x = max(0.0, min(max(0.0, float(draw_w - view_w)), (org_x * scale) - 18.0))
            desired_y = max(0.0, min(max(0.0, float(draw_h - view_h)), (org_y * scale) - 18.0))
            self._yolo_pan_x = desired_x - base_x
            self._yolo_pan_y = desired_y - base_y
        if draw_w > view_w:
            base_x = float(draw_w - view_w) / 2.0
            crop_x = int(round(base_x + self._yolo_pan_x))
            crop_x = max(0, min(draw_w - view_w, crop_x))
        else:
            painter_x = int((view_w - draw_w) / 2)
            self._yolo_pan_x = 0.0
        if draw_h > view_h:
            base_y = float(draw_h - view_h) / 2.0
            crop_y = int(round(base_y + self._yolo_pan_y))
            crop_y = max(0, min(draw_h - view_h, crop_y))
        else:
            painter_y = int((view_h - draw_h) / 2)
            self._yolo_pan_y = 0.0

        if draw_w > view_w or draw_h > view_h:
            shown = scaled.copy(crop_x, crop_y, min(view_w, draw_w), min(view_h, draw_h))
            from PyQt5.QtGui import QPainter
            p = QPainter(canvas)
            p.drawPixmap(painter_x, painter_y, shown)
            p.end()
        else:
            from PyQt5.QtGui import QPainter
            p = QPainter(canvas)
            p.drawPixmap(painter_x, painter_y, scaled)
            p.end()

        self._yolo_view_map = {
            "scale": scale,
            "draw_w": draw_w,
            "draw_h": draw_h,
            "view_w": view_w,
            "view_h": view_h,
            "crop_x": crop_x,
            "crop_y": crop_y,
            "pad_x": painter_x,
            "pad_y": painter_y,
            "src_w": src_w,
            "src_h": src_h,
            "src_orig_w": src_orig_w,
            "src_orig_h": src_orig_h,
            "rotation_deg": int(self._vision_rotation_deg_1),
        }
        self._draw_calib_text_overlay_on_canvas(canvas, panel_index=1)
        view_main.setPixmap(canvas)

    def _render_yolo_view_2(self):
        view2 = getattr(self, "yolo_view_2", None)
        if view2 is None:
            return
        if self._last_yolo_qimage_2 is None:
            view2.clear()
            view2.setPixmap(QPixmap())
            view2.setAlignment(Qt.AlignCenter)
            view2.setStyleSheet("background-color: #000000; color: #f3f3f3;")
            view2.setText("이미지 데이터 없음")
            return
        view2_w = max(1, view2.width())
        view2_h = max(1, view2.height())
        src2_orig_w = max(1, self._last_yolo_qimage_2.width())
        src2_orig_h = max(1, self._last_yolo_qimage_2.height())
        img_disp_2 = self._rotate_image_for_panel(self._last_yolo_qimage_2, 2)
        src2_w = max(1, img_disp_2.width())
        src2_h = max(1, img_disp_2.height())
        fit_scale2 = max(view2_w / float(src2_w), view2_h / float(src2_h))
        scale2 = fit_scale2 * float(self._yolo_zoom_2)
        draw2_w = int(round(src2_w * scale2))
        draw2_h = int(round(src2_h * scale2))
        scaled2 = QPixmap.fromImage(img_disp_2).scaled(
            max(1, draw2_w), max(1, draw2_h), Qt.KeepAspectRatio, Qt.FastTransformation
        )
        canvas2 = QPixmap(view2_w, view2_h)
        canvas2.fill(Qt.black)
        painter2_x = 0
        painter2_y = 0
        crop2_x = 0
        crop2_y = 0
        if self._yolo_pan_x_2 <= -999999.0 or self._yolo_pan_y_2 <= -999999.0:
            rot2 = self._normalize_rotation_deg(self._vision_rotation_deg_2)
            if rot2 == 90:
                org2_x, org2_y = src2_w - 1, 0
            elif rot2 == 180:
                org2_x, org2_y = src2_w - 1, src2_h - 1
            elif rot2 == 270:
                org2_x, org2_y = 0, src2_h - 1
            else:
                org2_x, org2_y = 0, 0
            base2_x = float(max(0, draw2_w - view2_w)) / 2.0
            base2_y = float(max(0, draw2_h - view2_h)) / 2.0
            desired2_x = max(0.0, min(max(0.0, float(draw2_w - view2_w)), (org2_x * scale2) - 18.0))
            desired2_y = max(0.0, min(max(0.0, float(draw2_h - view2_h)), (org2_y * scale2) - 18.0))
            self._yolo_pan_x_2 = desired2_x - base2_x
            self._yolo_pan_y_2 = desired2_y - base2_y
        if draw2_w > view2_w:
            base2_x = float(draw2_w - view2_w) / 2.0
            crop2_x = int(round(base2_x + self._yolo_pan_x_2))
            crop2_x = max(0, min(draw2_w - view2_w, crop2_x))
        else:
            painter2_x = int((view2_w - draw2_w) / 2)
            self._yolo_pan_x_2 = 0.0
        if draw2_h > view2_h:
            base2_y = float(draw2_h - view2_h) / 2.0
            crop2_y = int(round(base2_y + self._yolo_pan_y_2))
            crop2_y = max(0, min(draw2_h - view2_h, crop2_y))
        else:
            painter2_y = int((view2_h - draw2_h) / 2)
            self._yolo_pan_y_2 = 0.0

        from PyQt5.QtGui import QPainter
        p2 = QPainter(canvas2)
        if draw2_w > view2_w or draw2_h > view2_h:
            shown2 = scaled2.copy(crop2_x, crop2_y, min(view2_w, draw2_w), min(view2_h, draw2_h))
            p2.drawPixmap(painter2_x, painter2_y, shown2)
        else:
            p2.drawPixmap(painter2_x, painter2_y, scaled2)
        p2.end()
        self._yolo_view_map_2 = {
            "scale": scale2,
            "draw_w": draw2_w,
            "draw_h": draw2_h,
            "view_w": view2_w,
            "view_h": view2_h,
            "crop_x": crop2_x,
            "crop_y": crop2_y,
            "pad_x": painter2_x,
            "pad_y": painter2_y,
            "src_w": src2_w,
            "src_h": src2_h,
            "src_orig_w": src2_orig_w,
            "src_orig_h": src2_orig_h,
            "rotation_deg": int(self._vision_rotation_deg_2),
        }
        self._draw_calib_text_overlay_on_canvas(canvas2, panel_index=2)
        view2.setPixmap(canvas2)

    def _map_view_to_image_coords(self, vx: int, vy: int, panel_index: int = 1):
        if int(panel_index) == 2:
            image_size = self._last_yolo_image_size_2
            view_map = self._yolo_view_map_2
        else:
            image_size = self._last_yolo_image_size
            view_map = self._yolo_view_map
        if image_size is None or view_map is None:
            return None
        m = view_map
        scale = float(m["scale"])
        draw_w = int(m["draw_w"])
        draw_h = int(m["draw_h"])
        view_w = int(m["view_w"])
        view_h = int(m["view_h"])
        crop_x = int(m["crop_x"])
        crop_y = int(m["crop_y"])
        pad_x = int(m["pad_x"])
        pad_y = int(m["pad_y"])
        src_w = int(m["src_w"])
        src_h = int(m["src_h"])

        if draw_w > view_w:
            ax = vx + crop_x
        else:
            if vx < pad_x or vx >= (pad_x + draw_w):
                return None
            ax = vx - pad_x

        if draw_h > view_h:
            ay = vy + crop_y
        else:
            if vy < pad_y or vy >= (pad_y + draw_h):
                return None
            ay = vy - pad_y

        ix = int(max(0, min(src_w - 1, ax / max(1e-6, scale))))
        iy = int(max(0, min(src_h - 1, ay / max(1e-6, scale))))
        src_orig_w = int(m.get("src_orig_w", src_w))
        src_orig_h = int(m.get("src_orig_h", src_h))
        rot_deg = int(m.get("rotation_deg", 0))
        return self._map_rotated_to_source_coords(ix, iy, src_orig_w, src_orig_h, rot_deg)

    def _draw_calib_text_overlay_on_canvas(self, canvas: QPixmap, panel_index: int = 1):
        if int(panel_index) == 2:
            calib_on = bool(getattr(self, "_calibration_mode_enabled_2", False))
            data = getattr(self, "_calib_last_points_uvz_mm_2", None)
            rot_deg = int(getattr(self, "_vision_rotation_deg_2", 0))
        else:
            calib_on = bool(getattr(self, "_calibration_mode_enabled_1", False))
            data = getattr(self, "_calib_last_points_uvz_mm_1", None)
            rot_deg = int(getattr(self, "_vision_rotation_deg_1", 0))
        if not calib_on:
            return
        if not isinstance(data, dict) or not data:
            return
        lines = []
        center_uvz = self._extract_calib_center_uvz(data)
        center_canvas_xy = None
        if center_uvz is not None:
            u, v, z = center_uvz
            center_canvas_xy = self._map_source_to_canvas_coords(u, v, panel_index=int(panel_index))
            cxyz = self._uvz_to_camera_xyz_mm(
                u, v, z, require_intrinsics=True, panel_index=int(panel_index)
            ) if np.isfinite(z) else None
            if cxyz is not None and np.isfinite(np.asarray(cxyz, dtype=np.float64)).all():
                cx, cy, cz = cxyz
                lines.append(f"CENTER XYZ(mm): X={cx:.1f}, Y={cy:.1f}, Z={cz:.1f}")
            elif np.isfinite(z):
                lines.append(f"CENTER Z(mm): Z={z:.1f} (camera_info 미수신: XY 환산 불가)")
            else:
                lines.append("CENTER: 깊이 미수신")
        if not lines:
            return
        from PyQt5.QtGui import QPainter, QPen
        text_layer = QPixmap(canvas.size())
        text_layer.fill(Qt.transparent)
        p = QPainter(text_layer)
        try:
            font = QFont(UI_FONT_FAMILY, max(7, UI_TERMINAL_FONT_SIZE - 3))
            p.setFont(font)
            line_h = 13
            base_x = 8
            start_y = max(14, canvas.height() - 8 - ((len(lines) - 1) * line_h))
            shadow_pen = QPen(QColor(0, 0, 0))
            text_pen = QPen(QColor(230, 230, 230))
            for i, line in enumerate(lines):
                y = start_y + i * line_h
                p.setPen(shadow_pen)
                p.drawText(base_x + 1, y + 1, line)
                p.setPen(text_pen)
                p.drawText(base_x, y, line)
            if center_canvas_xy is not None:
                cx, cy = center_canvas_xy
                center_line = lines[0]
                tx = max(6, min(canvas.width() - 260, int(cx) + 10))
                ty = max(18, min(canvas.height() - 10, int(cy) - 8))
                p.setPen(shadow_pen)
                p.drawText(tx + 1, ty + 1, center_line)
                p.setPen(text_pen)
                p.drawText(tx, ty, center_line)
        finally:
            p.end()
        rot = self._normalize_rotation_deg(rot_deg)
        if rot != 0:
            tf = QTransform()
            tf.rotate(float(rot))
            rotated = text_layer.transformed(tf, Qt.SmoothTransformation)
            layer = QPixmap(canvas.size())
            layer.fill(Qt.transparent)
            p_layer = QPainter(layer)
            try:
                dx = int(round((canvas.width() - rotated.width()) / 2.0))
                dy = int(round((canvas.height() - rotated.height()) / 2.0))
                p_layer.drawPixmap(dx, dy, rotated)
            finally:
                p_layer.end()
            text_layer = layer
        p_canvas = QPainter(canvas)
        try:
            p_canvas.drawPixmap(0, 0, text_layer)
        finally:
            p_canvas.end()

    def _set_yolo_message_if_needed(self, msg: str):
        if not hasattr(self, "yolo_view"):
            return
        if "실패" in msg or "없음" in msg:
            self.yolo_view.setText(msg)
            self._vision_state_text = "오류"
        elif "시작" in msg:
            self._vision_state_text = "실행 중"
        elif "종료" in msg:
            self._vision_state_text = "종료"
        if self._last_yolo_qimage is None:
            self.yolo_view.setAlignment(Qt.AlignCenter)
            self.yolo_view.setText("비전1 프레임 대기중...")

    def _update_bottom_status(self):
        return

    def eventFilter(self, obj, event):
        panel_index = 1 if obj is self.yolo_view else (2 if obj is getattr(self, "yolo_view_2", None) else 0)
        if panel_index in (1, 2):
            log_tag = f"[비전{panel_index}]"
            if event.type() == QEvent.MouseMove:
                if panel_index == 1 and self._yolo_panning and self._yolo_pan_last_pos is not None:
                    px, py = self._yolo_pan_last_pos
                    cx, cy = event.pos().x(), event.pos().y()
                    self._yolo_pan_x -= float(cx - px)
                    self._yolo_pan_y -= float(cy - py)
                    self._yolo_pan_last_pos = (cx, cy)
                    self._render_yolo_view()
                    return True
                if panel_index == 2 and self._yolo_panning_2 and self._yolo_pan_last_pos_2 is not None:
                    px, py = self._yolo_pan_last_pos_2
                    cx, cy = event.pos().x(), event.pos().y()
                    self._yolo_pan_x_2 -= float(cx - px)
                    self._yolo_pan_y_2 -= float(cy - py)
                    self._yolo_pan_last_pos_2 = (cx, cy)
                    self._render_yolo_view_2()
                    return True
                # Hover move 이벤트에서는 좌표 갱신/상태 갱신을 생략해 UI 부하를 줄인다.
                # (클릭/휠/팬 드래그 시에만 필요한 동작을 수행)
                return False
            elif event.type() == QEvent.MouseButtonPress and event.button() == Qt.LeftButton:
                mapped = self._map_view_to_image_coords(event.pos().x(), event.pos().y(), panel_index=panel_index)
                if mapped is not None:
                    x, y = mapped
                    if panel_index == 1:
                        self._last_mouse_xy = (x, y)
                    else:
                        self._last_mouse_xy_2 = (x, y)
                    z_m = self._depth_m_from_image_coord(x, y, panel_index=panel_index)
                    if z_m is None:
                        self.append_log(f"{log_tag} 클릭 좌표: 깊이 미수신(Z 없음)으로 mm 환산 불가\n")
                        if panel_index == 2 and self._vision_coord_label_2 is not None:
                            self._vision_coord_label_2.hide()
                    else:
                        z_mm = z_m * 1000.0
                        cam_xyz = self._uvz_to_camera_xyz_mm(x, y, z_mm, require_intrinsics=True, panel_index=panel_index)
                        if cam_xyz is None or (not np.isfinite(np.asarray(cam_xyz, dtype=np.float64)).all()):
                            self.append_log(
                                f"{log_tag} 클릭 좌표: Z={z_mm:.1f}mm "
                                f"(camera_info 미수신: 비전 XYZ(mm) 환산 불가)\n"
                            )
                        else:
                            vx, vy, vz = [float(vv) for vv in cam_xyz]
                            robot_xyz = self._vision_uvz_to_robot_xyz_mm(x, y, z_mm, panel_index=panel_index)
                            if robot_xyz is None:
                                self.append_log(
                                    f"{log_tag} 클릭 비전 XYZ(mm): X={vx:.1f}, Y={vy:.1f}, Z={vz:.1f} "
                                    f"(로봇 변환행렬 미적용/변환 실패)\n"
                                )
                            else:
                                rx, ry, rz = robot_xyz
                                offset_x, offset_y, offset_z = self._get_vision_move_offset_xyz_mm()
                                rx_target = float(rx) + float(offset_x)
                                ry_target = float(ry) + float(offset_y)
                                rz_target = float(rz) + float(offset_z)
                                self._last_clicked_vision_xyz_mm = (float(vx), float(vy), float(vz))
                                self._last_clicked_robot_xyz_mm = (float(rx), float(ry), float(rz))
                                self._last_clicked_robot_target_xyz_mm = (float(rx_target), float(ry_target), float(rz_target))
                                self._last_clicked_source_vision = panel_index
                                self.append_log(
                                    f"{log_tag} 클릭 비전 XYZ(mm): X={vx:.1f}, Y={vy:.1f}, Z={vz:.1f} -> "
                                    f"로봇 XYZ(mm): X={rx:.2f}, Y={ry:.2f}, Z={rz:.2f} -> "
                                    f"이동목표 XYZ(mm): X={rx_target:.2f}, Y={ry_target:.2f}, Z={rz_target:.2f} "
                                    f"[옵셋 X={offset_x:.1f}, Y={offset_y:.1f}, Z={offset_z:.1f}]\n"
                                )
                                if panel_index == 2 and self._vision_coord_label_2 is not None:
                                    self._vision_coord_label_2.hide()
                                if self._vision_click_dialog_enabled:
                                    dialog_text = (
                                        f"비전 클릭 좌표\n"
                                        f"- 비전 XYZ(mm): X={vx:.1f}, Y={vy:.1f}, Z={vz:.1f}\n"
                                        f"- Robot XYZ(mm): X={rx:.2f}, Y={ry:.2f}, Z={rz:.2f}\n"
                                        f"- 이동 옵셋 XYZ(mm): X={offset_x:.1f}, Y={offset_y:.1f}, Z={offset_z:.1f}\n"
                                        f"- 이동 목표 XYZ(mm): X={rx_target:.2f}, Y={ry_target:.2f}, Z={rz_target:.2f}\n\n"
                                        f"로봇을 이동하시겠습니까?"
                                    )
                                    answer = QMessageBox.question(
                                        self,
                                        "비전 좌표 이동 확인",
                                        dialog_text,
                                        QMessageBox.Yes | QMessageBox.No,
                                        QMessageBox.No,
                                    )
                                    if answer == QMessageBox.Yes:
                                        self._run_vision_target_move(
                                            self._last_clicked_vision_xyz_mm,
                                            self._last_clicked_robot_xyz_mm,
                                            source_panel=panel_index,
                                        )
                else:
                    self.append_log(f"{log_tag} 클릭 좌표: 영상 영역 밖\n")
                self._update_bottom_status()
            elif panel_index == 1 and event.type() == QEvent.MouseButtonPress and event.button() == Qt.MiddleButton:
                self._yolo_panning = True
                self._yolo_pan_last_pos = (event.pos().x(), event.pos().y())
                return True
            elif panel_index == 2 and event.type() == QEvent.MouseButtonPress and event.button() == Qt.MiddleButton:
                self._yolo_panning_2 = True
                self._yolo_pan_last_pos_2 = (event.pos().x(), event.pos().y())
                return True
            elif panel_index == 1 and event.type() == QEvent.Wheel:
                delta = event.angleDelta().y()
                if delta > 0:
                    self._yolo_zoom = min(self._yolo_zoom_max, self._yolo_zoom * 1.12)
                elif delta < 0:
                    self._yolo_zoom = max(self._yolo_zoom_min, self._yolo_zoom / 1.12)
                if self._yolo_zoom <= (self._yolo_zoom_min + 1e-6):
                    self._yolo_pan_x = 0.0
                    self._yolo_pan_y = 0.0
                self._render_yolo_view()
                return True
            elif panel_index == 2 and event.type() == QEvent.Wheel:
                delta = event.angleDelta().y()
                if delta > 0:
                    self._yolo_zoom_2 = min(self._yolo_zoom_max, self._yolo_zoom_2 * 1.12)
                elif delta < 0:
                    self._yolo_zoom_2 = max(self._yolo_zoom_min, self._yolo_zoom_2 / 1.12)
                if self._yolo_zoom_2 <= (self._yolo_zoom_min + 1e-6):
                    self._yolo_pan_x_2 = 0.0
                    self._yolo_pan_y_2 = 0.0
                self._render_yolo_view_2()
                return True
            elif panel_index == 1 and event.type() == QEvent.MouseButtonRelease and event.button() == Qt.MiddleButton:
                self._yolo_panning = False
                self._yolo_pan_last_pos = None
                return True
            elif panel_index == 2 and event.type() == QEvent.MouseButtonRelease and event.button() == Qt.MiddleButton:
                self._yolo_panning_2 = False
                self._yolo_pan_last_pos_2 = None
                return True
            elif event.type() == QEvent.Leave:
                if panel_index == 1:
                    self._last_mouse_xy = None
                else:
                    self._last_mouse_xy_2 = None
                self._update_bottom_status()
        return super().eventFilter(obj, event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if not UI_USE_DESIGN_GEOMETRY:
            self._layout_main_frames()
        self._layout_voice_order_panel()
        self._layout_bartender_overview_panel()
        self._reposition_top_status_row()
        self._reposition_cycle_labels()
        self._layout_control_buttons()
        if hasattr(self, "yolo_view"):
            self._render_yolo_view()
        if hasattr(self, "yolo_view_2"):
            self._render_yolo_view_2()

    def _stop_yolo_camera(self):
        if self._yolo_worker is not None:
            self._yolo_worker.stop()
        if self._yolo_thread is not None:
            self._yolo_thread.quit()
            if not self._yolo_thread.wait(1200):
                # Fallback: camera worker thread can hang on device read.
                self._yolo_thread.terminate()
                self._yolo_thread.wait(500)
        self._yolo_worker = None
        self._yolo_thread = None

    def on_pick_place(self):
        if self.backend is None:
            self.append_log("[작업] 백엔드 초기화 중입니다.\n")
            return
        value, ok = QInputDialog.getInt(self, "오브젝트 번호", "0~10 입력:", 0, 0, 10, 1)
        if not ok:
            return

        ok, msg = self.backend.send_pick_place(value)
        self.append_log(msg + "\n")

    def _get_current_pose_defaults(self):
        posj = [0.0] * 6
        posx = [0.0] * 6
        if self.backend is None:
            return posj, posx
        data = None
        if hasattr(self.backend, "get_position_snapshot"):
            data, _ = self.backend.get_position_snapshot()
        elif hasattr(self.backend, "get_current_positions"):
            data = self.backend.get_current_positions()
        if data and len(data) >= 2:
            pj, px = data
            if pj is not None and len(pj) >= 6:
                posj = [float(v) for v in list(pj)[:6]]
            if px is not None and len(px) >= 6:
                posx = [float(v) for v in list(px)[:6]]
        return posj, posx

    def _ask_six_values_form(self, title, labels, defaults, limits=None, guide_text=None):
        if labels is None or len(labels) < 6:
            return "라벨 설정 오류: 6개 라벨이 필요합니다."
        dialog = QDialog(self)
        dialog.setWindowTitle(str(title))
        dialog.setModal(True)
        dialog.resize(360, 320)

        layout = QVBoxLayout(dialog)
        if guide_text:
            guide_label = QLabel(str(guide_text), dialog)
            guide_label.setWordWrap(True)
            guide_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            layout.addWidget(guide_label)

        grid = QGridLayout()
        edits = []
        for i in range(6):
            lbl = QLabel(str(labels[i]), dialog)
            edit = QLineEdit(dialog)
            edit.setStyleSheet(
                "QLineEdit {"
                " selection-background-color: #1f6feb;"
                " selection-color: #ffffff;"
                "}"
            )
            edit_palette = edit.palette()
            for group in (QPalette.Active, QPalette.Inactive, QPalette.Disabled):
                edit_palette.setColor(group, QPalette.Highlight, QColor("#1f6feb"))
                edit_palette.setColor(group, QPalette.HighlightedText, QColor("#ffffff"))
            edit.setPalette(edit_palette)
            v = float(defaults[i]) if defaults is not None and len(defaults) > i else 0.0
            edit.setText(f"{v:.2f}")
            edit.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            edit.setMaxLength(16)
            min_v, max_v = (-999999.0, 999999.0)
            if limits is not None and len(limits) > i:
                min_v, max_v = limits[i]
            validator = QDoubleValidator(float(min_v), float(max_v), 3, edit)
            validator.setNotation(QDoubleValidator.StandardNotation)
            edit.setValidator(validator)
            grid.addWidget(lbl, i, 0)
            grid.addWidget(edit, i, 1)
            edits.append(edit)
        layout.addLayout(grid)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, parent=dialog)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)

        if edits:
            edits[0].setFocus()
            edits[0].selectAll()

        if dialog.exec_() != QDialog.Accepted:
            return None

        vals = []
        for i, edit in enumerate(edits):
            raw = edit.text().strip()
            if raw == "":
                return "입력 형식 오류: 빈 칸 없이 숫자를 입력하세요."
            try:
                v = float(raw)
            except Exception:
                return "입력 형식 오류: 숫자만 입력하세요."
            if limits is not None and len(limits) > i:
                min_v, max_v = limits[i]
                if v < float(min_v) or v > float(max_v):
                    label = str(labels[i]) if labels is not None and len(labels) > i else f"항목{i+1}"
                    return f"입력 범위 오류: {label}는 {float(min_v):.1f} ~ {float(max_v):.1f} 범위여야 합니다."
            vals.append(v)
        return tuple(vals)

    def _ask_three_values_form(self, title, labels, defaults, guide_text=None):
        if labels is None or len(labels) < 3:
            return "라벨 설정 오류: 3개 라벨이 필요합니다."
        dialog = QDialog(self)
        dialog.setWindowTitle(str(title))
        dialog.setModal(True)
        dialog.resize(340, 240)

        layout = QVBoxLayout(dialog)
        if guide_text:
            guide_label = QLabel(str(guide_text), dialog)
            guide_label.setWordWrap(True)
            guide_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            layout.addWidget(guide_label)

        grid = QGridLayout()
        edits = []
        for i in range(3):
            lbl = QLabel(str(labels[i]), dialog)
            edit = QLineEdit(dialog)
            edit.setStyleSheet(
                "QLineEdit {"
                " selection-background-color: #1f6feb;"
                " selection-color: #ffffff;"
                "}"
            )
            edit_palette = edit.palette()
            for group in (QPalette.Active, QPalette.Inactive, QPalette.Disabled):
                edit_palette.setColor(group, QPalette.Highlight, QColor("#1f6feb"))
                edit_palette.setColor(group, QPalette.HighlightedText, QColor("#ffffff"))
            edit.setPalette(edit_palette)
            v = float(defaults[i]) if defaults is not None and len(defaults) > i else 0.0
            edit.setText(f"{v:.3f}")
            edit.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            edit.setMaxLength(16)
            validator = QDoubleValidator(-999999.0, 999999.0, 3, edit)
            validator.setNotation(QDoubleValidator.StandardNotation)
            edit.setValidator(validator)
            grid.addWidget(lbl, i, 0)
            grid.addWidget(edit, i, 1)
            edits.append(edit)
        layout.addLayout(grid)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, parent=dialog)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)

        if edits:
            edits[0].setFocus()
            edits[0].selectAll()

        if dialog.exec_() != QDialog.Accepted:
            return None

        vals = []
        for edit in edits:
            raw = edit.text().strip()
            if raw == "":
                return "입력 형식 오류: 빈 칸 없이 숫자를 입력하세요."
            try:
                v = float(raw)
            except Exception:
                return "입력 형식 오류: 숫자만 입력하세요."
            if not np.isfinite(v):
                return "입력 형식 오류: 유효한 숫자를 입력하세요."
            vals.append(float(v))
        return tuple(vals)

    def _confirm_motion_with_values(self, title, intro, labels, values, log_prefix):
        if labels is None or values is None:
            return False
        lines = [str(intro), ""]
        n = min(len(labels), len(values), 6)
        for i in range(n):
            try:
                v = float(values[i])
                lines.append(f"{labels[i]}: {v:.2f}")
            except Exception:
                lines.append(f"{labels[i]}: {values[i]}")
        msg = QMessageBox(self)
        msg.setWindowTitle(str(title))
        msg.setIcon(QMessageBox.Warning)
        msg.setText("\n".join(lines))
        msg.setTextInteractionFlags(Qt.TextSelectableByMouse)
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        msg.setDefaultButton(QMessageBox.No)
        msg.setStyleSheet(
            "QLabel {"
            " selection-background-color: #1f6feb;"
            " selection-color: #ffffff;"
            "}"
        )
        answer = msg.exec_()
        if answer != QMessageBox.Yes:
            self.append_log(f"[{log_prefix}] 이동 취소\n")
            return False
        return True

    def on_move_xyzabc_dialog(self):
        if self.backend is None:
            self.append_log("[좌표계 무브] 백엔드 초기화 중입니다.\n")
            return
        if not hasattr(self.backend, "send_move_cartesian"):
            self.append_log("[좌표계 무브] 백엔드가 좌표계 이동 기능을 지원하지 않습니다.\n")
            return
        _posj, posx = self._get_current_pose_defaults()
        vals = self._ask_six_values_form(
            "좌표계 무브",
            ["X", "Y", "Z", "A", "B", "C"],
            posx,
            guide_text="안내: 입력한 좌표로 로봇이 실제 이동합니다.",
        )
        if vals is None:
            return
        if isinstance(vals, str):
            self.append_log(f"[좌표계 무브] {vals}\n")
            return
        x, y, z, a, b, c = vals
        if not self._confirm_motion_with_values(
            "좌표계 무브 확인",
            "안내: 로봇이 실제로 이동합니다.\n좌표계로 이동할까요? 목표 위치는 아래와 같습니다.",
            ["X", "Y", "Z", "A", "B", "C"],
            (x, y, z, a, b, c),
            "좌표계 무브",
        ):
            return
        speed = self._motion_speed_for_command()
        ok, msg = self.backend.send_move_cartesian(x, y, z, a, b, c, vel=speed, acc=speed)
        self.append_log(f"[좌표계 무브] {msg}\n")

    def on_move_joint_dialog(self):
        if self.backend is None:
            self.append_log("[조인트 무브] 백엔드 초기화 중입니다.\n")
            return
        if not hasattr(self.backend, "send_move_joint"):
            self.append_log("[조인트 무브] 백엔드가 조인트 이동 기능을 지원하지 않습니다.\n")
            return
        posj, _posx = self._get_current_pose_defaults()
        vals = self._ask_six_values_form(
            "조인트 무브",
            ["J1", "J2", "J3", "J4", "J5", "J6"],
            posj,
            limits=JOINT_INPUT_LIMITS_DEG,
            guide_text="안내: 입력한 조인트 각도로 로봇이 실제 이동합니다.",
        )
        if vals is None:
            return
        if isinstance(vals, str):
            self.append_log(f"[조인트 무브] {vals}\n")
            return
        j1, j2, j3, j4, j5, j6 = vals
        if not self._confirm_motion_with_values(
            "조인트 무브 확인",
            "안내: 로봇이 실제로 이동합니다.\n조인트로 이동할까요? 목표 위치는 아래와 같습니다.",
            ["J1", "J2", "J3", "J4", "J5", "J6"],
            (j1, j2, j3, j4, j5, j6),
            "조인트 무브",
        ):
            return
        speed = self._motion_speed_for_command()
        ok, msg = self.backend.send_move_joint(j1, j2, j3, j4, j5, j6, vel=speed, acc=speed)
        self.append_log(f"[조인트 무브] {msg}\n")

    def on_print_positions(self):
        if self.backend is None:
            self.append_log("[좌표] 백엔드 초기화 중입니다.\n")
            return
        if hasattr(self.backend, "get_position_snapshot"):
            data, seen_at = self.backend.get_position_snapshot()
        else:
            data = self.backend.get_current_positions()
            seen_at = None

        if not data:
            self.append_log("[좌표] 수신된 좌표가 없습니다.\n")
            return
        if seen_at is not None and (time.monotonic() - seen_at) > POSITION_STALE_SEC:
            self.append_log("[좌표] 좌표 수신이 지연되어 출력하지 않습니다.\n")
            return

        posj, posx = data
        j_text = ", ".join(f"J{i+1}={self._fmt_ui_float(posj[i], 2)}" for i in range(6))
        p_text = ", ".join(
            (
                f"X={self._fmt_ui_float(posx[0], 2)}",
                f"Y={self._fmt_ui_float(posx[1], 2)}",
                f"Z={self._fmt_ui_float(posx[2], 2)}",
                f"A={self._fmt_ui_float(posx[3], 2)}",
                f"B={self._fmt_ui_float(posx[4], 2)}",
                f"C={self._fmt_ui_float(posx[5], 2)}",
            )
        )
        self.append_log(f"[좌표] {j_text}\n")
        self.append_log(f"[좌표] {p_text}\n")

        self.append_log("[좌표] 기준소스: RobotState snapshot(current_posx 우선)\n")

    def on_reset_robot(self):
        self._start_reset_async()

    def on_stop_motion(self):
        self._start_motion_stop_async(stop_mode=2)

    def on_emergency_stop(self):
        # backward compatibility alias
        self.on_stop_motion()

    def on_move_home(self):
        if self.backend is None:
            self.append_log("[홈] 백엔드 초기화 중입니다.\n")
            return
        if not hasattr(self.backend, "send_move_home"):
            self.append_log("[홈] 백엔드 홈 이동 기능을 지원하지 않습니다.\n")
            return
        if hasattr(self.backend, "get_home_posj"):
            target = tuple(float(v) for v in list(self.backend.get_home_posj())[:6])
        else:
            target = tuple(float(v) for v in list(HOME_POSJ)[:6])
        if not self._confirm_motion_with_values(
            "홈위치이동 확인",
            "홈위치로 이동할까요? 목표 위치는 아래와 같습니다.",
            ["J1", "J2", "J3", "J4", "J5", "J6"],
            target,
            "홈",
        ):
            return
        speed = self._motion_speed_for_command()
        ok, msg = self.backend.send_move_home(vel=speed, acc=speed)
        self.append_log(f"[홈] {msg}\n")

    def on_save_home_position_dialog(self):
        if self.backend is None:
            self.append_log("[홈저장] 백엔드 초기화 중입니다.\n")
            return
        if not hasattr(self.backend, "set_home_posj"):
            self.append_log("[홈저장] 백엔드 홈 저장 기능을 지원하지 않습니다.\n")
            return
        posj, _posx = self._get_current_pose_defaults()
        vals = self._ask_six_values_form(
            "홈위치 저장",
            ["J1", "J2", "J3", "J4", "J5", "J6"],
            posj,
            limits=JOINT_INPUT_LIMITS_DEG,
            guide_text="안내: 현재/입력한 조인트 각도를 홈위치로 저장합니다.",
        )
        if vals is None:
            self.append_log("[홈저장] 저장 취소\n")
            return
        if isinstance(vals, str):
            self.append_log(f"[홈저장] {vals}\n")
            return
        ok, msg = self.backend.set_home_posj(vals)
        self.append_log(f"[홈저장] {msg}\n")

    def on_gripper_move(self):
        if self.backend is None:
            self.append_log("[그리퍼] 백엔드 초기화 중입니다.\n")
            return
        if not hasattr(self.backend, "send_gripper_move"):
            self.append_log("[그리퍼] 백엔드 그리퍼 수동 이동 기능을 지원하지 않습니다.\n")
            return
        if hasattr(self.backend, "get_robot_mode_snapshot"):
            mode_value, mode_seen_at = self.backend.get_robot_mode_snapshot()
            if mode_seen_at is None or mode_value is None:
                self.append_log("[그리퍼] 로봇모드 확인 중입니다. 잠시 후 다시 시도하세요.\n")
                return
            if int(mode_value) != 1:
                self.append_log("[그리퍼] 오토모드(1)에서만 실행할 수 있습니다.\n")
                return
        raw = ""
        if hasattr(self, "_gripper_stroke_input") and self._gripper_stroke_input is not None:
            raw = self._gripper_stroke_input.text().strip()
        if not raw:
            self.append_log("[그리퍼] 벌어진 거리(mm)를 입력하세요. (0~109)\n")
            return
        try:
            distance_mm = float(raw)
        except ValueError:
            self.append_log("[그리퍼] 거리 값은 숫자(mm)여야 합니다. (0~109)\n")
            return
        ok, msg = self.backend.send_gripper_move(distance_mm)
        self.append_log(f"[그리퍼] {msg}\n")

    def closeEvent(self, event):
        self._closing = True
        self._bartender_sequence_running = False
        backend_shutdown_done = False
        try:
            if hasattr(self, "_log_timer") and self._log_timer is not None:
                self._log_timer.stop()
            if hasattr(self, "_status_timer") and self._status_timer is not None:
                self._status_timer.stop()
            if hasattr(self, "_robot_status_timer") and self._robot_status_timer is not None:
                self._robot_status_timer.stop()
            if hasattr(self, "_vision_status_timer_1") and self._vision_status_timer_1 is not None:
                self._vision_status_timer_1.stop()
            if hasattr(self, "_vision_status_timer_2") and self._vision_status_timer_2 is not None:
                self._vision_status_timer_2.stop()
            if hasattr(self, "_top_status_anim_timer") and self._top_status_anim_timer is not None:
                self._top_status_anim_timer.stop()
            if hasattr(self, "_pos_timer") and self._pos_timer is not None:
                self._pos_timer.stop()
            if hasattr(self, "_current_tool_timer") and self._current_tool_timer is not None:
                self._current_tool_timer.stop()

            if self.backend is not None and hasattr(self.backend, "stop_bartender_sequence"):
                try:
                    self.backend.stop_bartender_sequence(reason="개발자 UI 종료")
                except Exception:
                    pass
            # 중요: 런치 하위 노드를 강제 종료하기 전에 backend.shutdown()을 먼저 호출해
            # drl_start 서비스가 살아있는 동안 그리퍼 terminate를 선행한다.
            if self.backend is not None:
                try:
                    self.backend.shutdown()
                    backend_shutdown_done = True
                except Exception:
                    pass
            self._stop_voice_order_mic_test_monitor("앱 종료")
            self._stop_voice_cycle_worker(reason="앱 종료")
            self._stop_voice_order_worker(reason="앱 종료")
            self._stop_webui_order_start_sync_worker()
            if self._reset_thread is not None:
                self._reset_thread.quit()
                if not self._reset_thread.wait(800):
                    self._reset_thread.terminate()
                    self._reset_thread.wait(300)
            if self._motion_stop_thread is not None:
                self._motion_stop_thread.quit()
                if not self._motion_stop_thread.wait(500):
                    self._motion_stop_thread.terminate()
                    self._motion_stop_thread.wait(200)
            self._stop_vision_compose_workers()
            # 앱 종료 시 런타임으로 띄운 launch 하위 노드까지 정리한다.
            self._stop_sensor_launch_runtime(reason="앱 종료", kill_nodes=True)
            self._stop_robot_launch_runtime(reason="앱 종료", kill_nodes=True)
            self._stop_yolo_camera()
            self._stop_calibration_process(1)
            self._stop_calibration_process(2)
            self._teardown_external_vision_bridge()
            self._stop_external_vision_process()

            if self._backend_thread is not None:
                self._backend_thread.quit()
                if not self._backend_thread.wait(800):
                    self._backend_thread.terminate()
                    self._backend_thread.wait(300)
            if (not backend_shutdown_done) and self.backend is not None:
                try:
                    self.backend.shutdown()
                except Exception:
                    pass
        finally:
            event.accept()
            super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setFont(QFont(UI_FONT_FAMILY, UI_FONT_SIZE))
    palette = app.palette()
    for group in (QPalette.Active, QPalette.Inactive, QPalette.Disabled):
        palette.setColor(group, QPalette.Highlight, QColor("#1f6feb"))
        palette.setColor(group, QPalette.HighlightedText, QColor("#ffffff"))
    app.setPalette(palette)
    app.setStyleSheet(
        f"""
        * {{
            font-family: '{UI_FONT_FAMILY}';
            font-size: {UI_FONT_SIZE}pt;
        }}
        QWidget, QDialog, QMessageBox {{
            selection-background-color: #1f6feb;
            selection-color: #ffffff;
        }}
        QLineEdit, QTextEdit, QPlainTextEdit, QTextBrowser, QLabel {{
            selection-background-color: #1f6feb;
            selection-color: #ffffff;
        }}
        """
    )

    window = App(backend=None, auto_start_backend=True)
    window.setFont(QFont(UI_FONT_FAMILY, UI_FONT_SIZE))
    window.show()
    window.raise_()
    window.activateWindow()
    window.setFocus(Qt.ActiveWindowFocusReason)
    sys.exit(app.exec_())
