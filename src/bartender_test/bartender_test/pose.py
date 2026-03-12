import rclpy
import sys
from rclpy.node import Node
import DR_init
from .defines import HOME_POSE, CHEERS_POSE, POLE_POSE, POUR_HORIZONTAL, POUR_DIAGONAL, POUR_VERTICAL, CONTACT_POSE

def main(args=None):
    if len(sys.argv) < 2:
        print("Usage:")
        print("  ros2 run bartender_test pose home")
        print("  ros2 run bartender_test pose cheers")
        print("  ros2 run bartender_test pose contact")
        print("  ros2 run bartender_test pose pour_horizontal")
        print("  ros2 run bartender_test pose pour_diagonal")
        print("  ros2 run bartender_test pose pour_vertical")
        print("  ros2 run bartender_test pose pole")
        return

    rclpy.init(args=args)
    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "e0509"
    node = rclpy.create_node('pose_node', namespace=ROBOT_ID)
    DR_init.__dsr__id, DR_init.__dsr__model, DR_init.__dsr__node = ROBOT_ID, ROBOT_MODEL, node

    from DSR_ROBOT2 import (movej, set_robot_mode, ROBOT_MODE_AUTONOMOUS, wait)

    try:
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        wait(0.5)

        cmd = sys.argv[1].lower()
        
        if cmd == 'home':
            target_pose = HOME_POSE
        elif cmd == 'cheers':
            target_pose = CHEERS_POSE
        elif cmd == 'contact':
            target_pose = CONTACT_POSE
        elif cmd in ['pour_mid', 'pour_horizontal']:
            target_pose = POUR_HORIZONTAL
        elif cmd == 'pour_diagonal':
            target_pose = POUR_DIAGONAL
        elif cmd in ['pour_end', 'pour_vertical']:
            target_pose = POUR_VERTICAL
        elif cmd == 'pole':
            target_pose = POLE_POSE
        else:
            print(f"Unknown pose: {cmd}")
            return

        node.get_logger().info(f"Moving to pose: {cmd} ({target_pose})")
        ret = movej(target_pose, vel=20, acc=20)
        node.get_logger().info(f"Move complete with return code: {ret}")

    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
