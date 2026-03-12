import rclpy
import sys
from rclpy.node import Node
from dsr_msgs2.srv import SetRobotMode
from .gripper_controller import GripperController

def main(args=None):
    if len(sys.argv) < 2:
        print("Usage:")
        print("  ros2 run bartender_test gripper open")
        print("  ros2 run bartender_test gripper close <force>")
        return

    cmd = sys.argv[1].lower()
    stroke = 0
    force = 400 # Default for open/manual

    # 1. Strict 'open' logic
    if cmd == 'open':
        stroke = 0
        force = 400

    # 2. Strict 'close' logic
    elif cmd == 'close':
        # Default to max safe force (800) if not specified
        if len(sys.argv) >= 3:
            try:
                force = int(sys.argv[2])
            except ValueError:
                print(f"Invalid force value: '{sys.argv[2]}'. Must be a number.")
                return
        else:
            force = 800 # Maximum Safe Force for RH-P12-RN
        
        stroke = 700 # Fully closed stroke


    # 3. Handle manual stroke (if user provides a raw number first)
    else:
        try:
            stroke = int(cmd)
            force = int(sys.argv[2]) if len(sys.argv) >= 3 else 400
        except ValueError:
            print(f"Invalid command: '{cmd}'. Use 'open' or 'close <number>'.")
            return

    # ROS execution
    rclpy.init(args=args)
    ROBOT_ID = "dsr01"
    node = rclpy.create_node('gripper_command_node', namespace=ROBOT_ID)

    mode_cli = node.create_client(SetRobotMode, f"/{ROBOT_ID}/system/set_robot_mode")
    if mode_cli.wait_for_service(timeout_sec=2.0):
        req = SetRobotMode.Request()
        req.robot_mode = 1 
        future = mode_cli.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)

    try:
        gripper = GripperController(node=node, namespace=ROBOT_ID)
        node.get_logger().info(f"Executing: stroke={stroke}, force={force}")
        if gripper.move(stroke, current=force):
            node.get_logger().info("Success.")
        else:
            node.get_logger().error("Failed.")
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
