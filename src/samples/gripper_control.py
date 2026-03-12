#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class GripperControl(Node):
    """
    'rh_r1' 관절의 목표 각도를 받아 부드럽게 보간하며
    /dsr01/joint_states 토픽을 지속적으로 발행하는 노드
    """
    def __init__(self, node_name='gripper_control_node'):
        super().__init__(node_name)
        
        self.publisher_ = self.create_publisher(JointState, '/dsr01/joint_states', 10)
        
        self.joint_names = ['rh_l1', 'rh_r1', 'rh_l2', 'rh_r2']
        self.default_pos = [0.5976, 0.5382, 0.5533, 0.4811]
        self.control_joint_index = 1  # 'rh_r1'의 인덱스

        self.current_position = list(self.default_pos)
        self.source_position = list(self.default_pos)
        self.target_position = list(self.default_pos)

        self.timer_period = 0.1  # 0.1초 (10Hz)
        self.movement_duration = 3.0  # 목표 도달 시간 (초)
        self.switch_interval_ticks = int(self.movement_duration / self.timer_period)
        self.counter = self.switch_interval_ticks # 시작 시 움직이지 않도록 카운터 초기화

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('GripperControl 모듈이 시작되었습니다. /dsr01/joint_states 발행 중...')

    def move(self, rh_r1_angle_deg):
        """
        [Public API] 'rh_r1' 관절의 새로운 목표 각도(degree)를 설정합니다.
        """
        self.get_logger().info(f'새로운 목표 수신: rh_r1 = {rh_r1_angle_deg} deg')

        self.source_position = list(self.current_position)

        new_target = list(self.default_pos) # 기본 위치에서 시작
        new_target[self.control_joint_index] = math.radians(rh_r1_angle_deg) # rh_r1 값만 덮어쓰기
        self.target_position = new_target

        self.counter = 0

    def open(self, rh_r1_angle_deg=0):
        """
        [Public API] 'rh_r1' 관절의 새로운 목표 각도(degree)를 설정합니다.
        """
        self.get_logger().info(f'새로운 목표 수신: rh_r1 = {rh_r1_angle_deg} deg')

        self.source_position = list(self.current_position)

        new_target = list(self.default_pos) # 기본 위치에서 시작
        new_target[self.control_joint_index] = math.radians(rh_r1_angle_deg) # rh_r1 값만 덮어쓰기
        self.target_position = new_target

        self.counter = 0

    def close(self, rh_r1_angle_deg=60):
        """
        [Public API] 'rh_r1' 관절의 새로운 목표 각도(degree)를 설정합니다.
        """
        self.get_logger().info(f'새로운 목표 수신: rh_r1 = {rh_r1_angle_deg} deg')

        self.source_position = list(self.current_position)

        new_target = list(self.default_pos) # 기본 위치에서 시작
        new_target[self.control_joint_index] = math.radians(rh_r1_angle_deg) # rh_r1 값만 덮어쓰기
        self.target_position = new_target

        self.counter = 0


    def timer_callback(self):
        """
        [Private] 10Hz로 실행되며 실제 보간 및 메시지 발행을 담당합니다.
        """
        
        if self.counter < self.switch_interval_ticks:
            alpha = self.counter / self.switch_interval_ticks
            alpha = min(alpha, 1.0) # 1.0을 넘지 않도록
            self.counter += 1
        else:
            alpha = 1.0 # 목표 도달 후에는 항상 1.0

        interpolated_position = []
        for i in range(len(self.joint_names)):
            start_val = self.source_position[i]
            end_val = self.target_position[i]
            interp_val = (1.0 - alpha) * start_val + alpha * end_val
            interpolated_position.append(interp_val)

        self.current_position = interpolated_position

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        msg.name = self.joint_names
        msg.position = self.current_position
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.publisher_.publish(msg)