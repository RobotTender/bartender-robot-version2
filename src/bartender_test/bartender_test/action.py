import rclpy
import sys
from rclpy.node import Node
import DR_init
from .gripper_controller import GripperController
from .defines import (HOME_POSE, CHEERS_POSE, CONTACT_POSE, POUR_HORIZONTAL, POUR_DIAGONAL, POUR_VERTICAL, POLE_POSE,
                            POS1_XYZ, POS2_XYZ, POS3_XYZ, POS4_XYZ, POS5_XYZ)

def main(args=None):
    if len(sys.argv) < 2:
        print("Usage:")
        print("  ros2 run bartender_test action pour move [horizontal|vertical] [repeat=<n>]")
        print("  ros2 run bartender_test action pour orbit [<start_idx>] [<end_idx>] [repeat=<n>]")
        print("  ros2 run bartender_test action warmup [repeat=<n>]")
        return

    # Initialize ROS2
    rclpy.init(args=args)
    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "e0509"
    main_node = rclpy.create_node('action_node', namespace=ROBOT_ID)
    DR_init.__dsr__id, DR_init.__dsr__model, DR_init.__dsr__node = ROBOT_ID, ROBOT_MODEL, main_node

    from DSR_ROBOT2 import (movec, movej, movel, movesx, moveb, posx, set_robot_mode, ROBOT_MODE_AUTONOMOUS, wait, 
                            get_current_posx, fkin, ikin, get_joint_torque, read_data_rt, DR_MVS_VEL_CONST, DR_MVS_VEL_NONE)
    from DR_common2 import posb, DR_CIRCLE, DR_LINE
    
    try:
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        wait(0.5)

        cmd = sys.argv[1].lower()
        sub_cmd = sys.argv[2].lower() if len(sys.argv) > 2 else ""
        move_type = sys.argv[3].lower() if len(sys.argv) > 3 and not sys.argv[3].startswith('repeat=') and not sys.argv[3].isdigit() else ""
        
        def get_args_info():
            repeat = 1
            range_indices = None
            nums = []
            for arg in sys.argv[2:]:
                if arg.startswith('repeat='):
                    try: repeat = int(arg.split('=')[1])
                    except: pass
                elif arg.isdigit():
                    nums.append(int(arg))
            
            if sub_cmd == 'orbit':
                if len(nums) >= 2:
                    range_indices = (nums[0], nums[1])
                    if len(nums) >= 3: repeat = nums[2]
                elif len(nums) == 1:
                    range_indices = (1, nums[0])
            elif len(nums) >= 1:
                repeat = nums[0]
            return repeat, range_indices

        count, orbit_range = get_args_info()

        def get_trajectory_points(s):
            # Point 1: CHEERS
            p1_raw = [float(x) for x in fkin(CHEERS_POSE, ref=0)]
            p1 = posx(p1_raw)
            
            # Point 2: POS2_XYZ position with CONTACT_POSE orientation
            p2_raw = [float(x) for x in fkin(CONTACT_POSE, ref=0)]
            p2 = posx(POS2_XYZ + p2_raw[3:])
            
            # Point 3: POUR_HORIZONTAL
            pm_raw = [float(x) for x in fkin(POUR_HORIZONTAL, ref=0)]
            p3 = posx(POS3_XYZ + pm_raw[3:])
            
            # Point 5: POUR_VERTICAL
            p5_raw = [float(x) for x in fkin(POUR_VERTICAL, ref=0)]
            p5 = posx(p5_raw)
            
            # Midpoint 2 (Point 4): POS4_XYZ with target orientation from POUR_DIAGONAL
            pd_raw = [float(x) for x in fkin(POUR_DIAGONAL, ref=0)]
            p4 = posx(list(POS4_XYZ) + pd_raw[3:])
            
            return [p1, p2, p3, p4, p5]

        def do_pour_move():
            cx_full, s = get_current_posx()
            p = get_trajectory_points(s)

            for i in range(count):
                if count > 1: main_node.get_logger().info(f"--- Cycle {i+1}/{count} ---")
                
                if move_type == 'horizontal':
                    main_node.get_logger().info("Action: POUR MOVE HORIZONTAL")
                    movej(CHEERS_POSE, vel=60, acc=60)
                    movec(p[1], p[2], vel=50, acc=50)
                elif move_type == 'vertical':
                    main_node.get_logger().info("Action: POUR MOVE VERTICAL")
                    movej(POUR_HORIZONTAL, vel=60, acc=60)
                    movec(p[3], p[4], vel=50, acc=50)
                else:
                    main_node.get_logger().info("Action: POUR MOVE FULL")
                    movej(CHEERS_POSE, vel=60, acc=60)
                    # Use radius for continuous motion (zero pause)
                    movec(p[1], p[2], vel=50, acc=50, radius=20)
                    movec(p[3], p[4], vel=50, acc=50)
            return 0

        def do_pour_orbit():
            cx_full, s = get_current_posx()
            full_path = get_trajectory_points(s)
            path = full_path
            msg_suffix = "Full"
            if orbit_range:
                start_idx, end_idx = orbit_range
                s_idx = max(0, start_idx - 1)
                e_idx = min(len(full_path), end_idx)
                path = full_path[s_idx:e_idx]
                msg_suffix = f"Range {start_idx}->{end_idx}"

            if len(path) < 2:
                print("Error: Path range must include at least 2 points.")
                return

            main_node.get_logger().info(f"Action: POUR ORBIT {msg_suffix} ({count} cycles)")
            for i in range(count):
                if count > 1: main_node.get_logger().info(f"--- Cycle {i+1}/{count} ---")
                main_node.get_logger().info(f"Moving to start of range...")
                if orbit_range and orbit_range[0] == 1 or not orbit_range:
                    movej(CHEERS_POSE, vel=60, acc=60)
                else:
                    movel(path[0], vel=60, acc=60)
                wait(0.2)
                movesx(path, vel=150, acc=150, vel_opt=DR_MVS_VEL_NONE)
            return 0

        def do_warmup():
            main_node.get_logger().info(f"Action: WARMUP ({count} cycles)")
            gripper = GripperController(main_node, namespace=ROBOT_ID)
            gripper.move(0)
            wait(1.0)
            poses = [("HOME", HOME_POSE), ("CHEERS", CHEERS_POSE), ("CONTACT", CONTACT_POSE), 
                     ("POUR_HORIZONTAL", POUR_HORIZONTAL), ("POLE", POLE_POSE)]
            for i in range(count):
                if count > 1: main_node.get_logger().info(f"--- Cycle {i+1}/{count} ---")
                for name, pose in poses:
                    movej(pose, vel=60, acc=60)
                wait(1.0)
                rt_data = read_data_rt()
                temps = rt_data.joint_temperature if rt_data else [0.0]*6
                msg = f"Cycle {i+1} Status | Temps: " + "/".join([f"{t:.1f}" for t in temps])
                main_node.get_logger().info(msg)
            return 0

        if cmd == 'pour':
            if sub_cmd == 'move': do_pour_move()
            elif sub_cmd == 'orbit': do_pour_orbit()
            else: print("Invalid pour command. Use: move [horizontal|vertical], or orbit")
        elif cmd == 'warmup': do_warmup()
        else: print(f"Unknown command: {cmd}")

    except Exception as e:
        main_node.get_logger().error(f"Action Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            main_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
