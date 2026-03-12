import rclpy
import sys
from rclpy.node import Node
import DR_init

def main(args=None):
    rclpy.init(args=args)
    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "e0509"
    node = rclpy.create_node('movej_node', namespace=ROBOT_ID)
    DR_init.__dsr__id, DR_init.__dsr__model, DR_init.__dsr__node = ROBOT_ID, ROBOT_MODEL, node

    from DSR_ROBOT2 import (movej, set_robot_mode, ROBOT_MODE_AUTONOMOUS, wait, get_current_posj)

    def print_usage():
        print("\nUsage:")
        print("  ros2 run bartender_test movej j<num> [rel|abs] <val>")
        print("  ros2 run bartender_test movej <j1> <j2> <j3> <j4> <j5> <j6>")
        print("\nJoint Limits (approx):")
        print("  J1, J2, J4, J5, J6: +/- 360.0°")
        print("  J3: +/- 155.0°")
        print("\nExample:")
        print("  ros2 run bartender_test movej j4 rel 10")

    try:
        current_pose = list(get_current_posj())
        
        # --- CASE 1: No arguments ---
        if len(sys.argv) < 2:
            print_usage()
            print("\n--- Current Joint Values ---")
            for i, val in enumerate(current_pose):
                print(f"  J{i+1}: {val:7.3f}°")
            return

        cmd = sys.argv[1].lower()

        # --- CASE 2: Only joint name (e.g., j4) ---
        if cmd.startswith('j') and len(cmd) <= 2 and cmd[1:].isdigit() and len(sys.argv) == 2:
            joint_index = int(cmd[1:]) - 1
            if 0 <= joint_index <= 5:
                print_usage()
                print(f"\n--- Current Value for {cmd.upper()} ---")
                print(f"  {cmd.upper()}: {current_pose[joint_index]:7.3f}°")
            else:
                print(f"Invalid joint number: {cmd}")
            return

        # --- CASE 3: Execute Movement ---
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        wait(0.5)
        target_pose = None

        if cmd.startswith('j') and len(cmd) <= 2 and cmd[1:].isdigit():
            joint_index = int(cmd[1:]) - 1
            if 0 <= joint_index <= 5 and len(sys.argv) >= 4:
                mode = sys.argv[2].lower()
                val = float(sys.argv[3])
                target_pose = list(current_pose)
                if mode == 'rel': 
                    target_pose[joint_index] += val
                else: 
                    target_pose[joint_index] = val
            else:
                print("Invalid joint command format. Use: j<num> [rel|abs] <val>")
                return

        elif len(sys.argv) >= 7:
            try:
                target_pose = [float(x) for x in sys.argv[1:7]]
            except ValueError:
                print("Invalid joint list. Please provide 6 floats.")
                return
        else:
            print(f"Unknown command: {cmd}")
            return

        if target_pose:
            node.get_logger().info(f"Moving to: {target_pose}")
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
