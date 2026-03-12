#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
import DR_init
import sys

#from rclpy.executors import SingleThreadedExecutor 
# isaac sim gripper
from gripper_control import GripperControl 
# real gripper
from dsr_example.simple.gripper_drl_controller import GripperController

def main(args=None):
    rclpy.init(args=args)

    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "e0509"
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    # --- 노드 2개 생성 ---
    gripper = GripperControl()
    dsr_node = rclpy.create_node('example_py', namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node 

    # gripper의 콜백(타이머)만 처리할 별도 Executor 생성
    #gripper_executor = SingleThreadedExecutor()
    #gripper_executor.add_node(gripper)

    #spin_thread = threading.Thread(target=gripper_executor.spin)
    #spin_thread.daemon = True
    #spin_thread.start()
    gripper.get_logger().info("GripperControl 노드가 별도 Executor 스레드에서 실행 중입니다.")

    from DSR_ROBOT2 import movej, movel, wait, posj, posx, set_robot_mode, ROBOT_MODE_AUTONOMOUS

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    drl_gripper = None 

    drl_gripper = GripperController(node=dsr_node, namespace=ROBOT_ID)


    target_pos = posj(0, 0, 90.0, 0, 90.0, 0)
    x1 = [420, -11, 165.0, 63.0, -175.0, 61.0]
    x2 = [420, -11, 365.0, 63.0, -175.0, 61.0]
    x3 = [420, -11, 165.0, 63.0, -175.0, 61.0]

    while rclpy.ok():

        movej(target_pos, vel=100, acc=100)
        # gripper.move(0.0)
        drl_gripper.move(0)
        #gripper.open()
        #drl_gripper.open()
        wait(5.0)

        movel(x1, vel=100, acc=100)
        #gripper.move(60.0)
        drl_gripper.move(700)
        #gripper.close()
        #drl_gripper.close()
        wait(5.0)

        movel(x2, vel=100, acc=100)
        wait(5.0)

        movel(x3, vel=100, acc=100)
        #gripper.move(0.0)
        drl_gripper.move(0)
        #gripper.open()
        #drl_gripper.open()
        wait(5.0)

    
    dsr_node.get_logger().info("사용자에 의해 중지됨")
    dsr_node.get_logger().error(f"DSR 제어 중 오류 발생: {e}")
    dsr_node.get_logger().info("--- 작업 완료 ---")
    
    gripper_executor.shutdown() # 스핀 중지
    drl_gripper.terminate()
    rclpy.shutdown()
    dsr_node.get_logger().info("모든 노드 종료.")
    spin_thread.join() 


if __name__ == '__main__':
    main()