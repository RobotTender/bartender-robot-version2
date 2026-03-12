import rclpy
import DR_init
import sys

from bartender_test.gripper_controller import GripperController


def main(args=None):
    rclpy.init(args=args)

    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "e0509"
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    VELOCITY = 20
    ACC = 20

    node = rclpy.create_node('example_py', namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    from DSR_ROBOT2 import(
        DR_BASE, DR_TOOL, DR_LINE, DR_CIRCLE, DR_MV_RA_OVERRIDE, DR_AXIS_Z,
        movej, movel, moveb, movejx, movec, move_spiral, move_periodic,
        amovej,
        wait, mwait,
        posj, posx, posb, trans,
        get_current_posx,
        set_robot_mode, set_user_cart_coord, set_velx, set_accx,
        ROBOT_MODE_AUTONOMOUS
    ) 

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    gripper = None
    try:
        from DSR_ROBOT2 import wait
        gripper = GripperController(node=node, namespace=ROBOT_ID)
        
        # gripper.move(0)
        # wait(3)
        
    except Exception as e:
        print(f"Gripper error: {e}")
        rclpy.shutdown()
        return

    # base_posj = posj(0, -45, 90, 0, 135, -180.0)
    print("기본 자세")
    base_posj = posj(69.5, -43.0, 102.0, 101.0, -72.0, -213.0)
    movej(base_posj, VELOCITY, ACC)
    
    wait(1)
    print("잡기 전 자세")
    P1 = posj(28.0, -35.0, 100.0, 77.0, 63.0, -154.0)
    movej(P1, VELOCITY, ACC)

    current_pos = get_current_posx()[0]
    print(f"current_pos : {current_pos}")
    target_pos = posx(424.0, 630.0, 790.5, current_pos[3], current_pos[4], current_pos[5])
    wait(2)

    print("타겟 좌표로 이동")
    ret = movel(target_pos, vel=VELOCITY, acc=ACC)
    print(f"movel ret = {ret}, target = {target_pos}")
    wait(1)

    # wait(1)
    # print("중간 자세")
    # P2 = posj(61.0, -20.0, 97.0, 96.0, -63.0, -195.0)
    # movej(P2, VELOCITY, ACC)

    # wait(1)
    # print("최종 자세")
    # target_posj = posj(45.0, 0.0, 135.0, 90.0, -90.0, -135.0)
    # movej(target_posj, VELOCITY, ACC)

    print("예제 프로그램이 종료되었습니다.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()