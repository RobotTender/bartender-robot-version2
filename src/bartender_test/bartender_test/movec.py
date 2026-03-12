import rclpy
import sys
from rclpy.node import Node
import DR_init
from .defines import (HOME_POSE, CHEERS_POSE, POUR_MID, POUR_END, POLE_POSE,
                            POS1_XYZ, POS2_XYZ, POS3_XYZ)

def main(args=None):
    if len(sys.argv) < 4:
        print("Usage:")
        print("  ros2 run bartender_test movec <from> <to> <midpoint>")
        print("Example:")
        print("  ros2 run bartender_test movec pos1 pos2 pos3")
        print("\nAvailable Named Poses/Markers:")
        print("  Joint: home, cheers, contact, pour_mid, pole")
        print("  Cartesian: pos1, pos2, pos3")
        return

    rclpy.init(args=args)
    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "e0509"
    node = rclpy.create_node('movec_node', namespace=ROBOT_ID)
    DR_init.__dsr__id, DR_init.__dsr__model, DR_init.__dsr__node = ROBOT_ID, ROBOT_MODEL, node

    from DSR_ROBOT2 import (movej, movec, set_robot_mode, ROBOT_MODE_AUTONOMOUS, 
                            wait, fkin, get_current_posx)

    # 1. Map Names to Joint Poses or Cartesian XYZ
    joint_map = {
        'home': HOME_POSE,
        'cheers': CHEERS_POSE,
        'contact': CONTACT_POSE,
        'pour_mid': POUR_MID,
        'pour_end': POUR_END,
        'pole': POLE_POSE
    }
    marker_map = {
        'pos1': POS1_XYZ,
        'pos2': POS2_XYZ,
        'pos3': POS3_XYZ,
        'mid_p2p3': MID_P2P3
    }

    def get_xyz_and_j(name):
        name = name.lower()
        if name in joint_map:
            j = joint_map[name]
            x_full = fkin(j, ref=0)
            return list(x_full), list(j)
        elif name in marker_map:
            xyz = marker_map[name]
            # Use current orientation for marker targets if not in joint map
            curr_x, sol = get_current_posx()
            return list(xyz) + list(curr_x[3:]), None
        else:
            print(f"Unknown point name: {name}")
            return None, None

    try:
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        wait(0.5)

        start_name = sys.argv[1]
        end_name = sys.argv[2]
        mid_name = sys.argv[3]

        start_x, start_j = get_xyz_and_j(start_name)
        end_x, end_j = get_xyz_and_j(end_name)
        mid_x, _ = get_xyz_and_j(mid_name)

        if any(p is None for p in [start_x, end_x, mid_x]):
            return

        # 2. Move to Start Position first
        node.get_logger().info(f"Moving to start position: {start_name}")
        if start_j:
            movej(start_j, vel=40, acc=40)
        else:
            from DSR_ROBOT2 import movel
            movel(start_x, vel=60, acc=60)
        wait(0.5)

        # 3. Execute movec through midpoint
        node.get_logger().info(f"Executing Circular Move: {start_name} -> {mid_name} -> {end_name}")
        node.get_logger().info(f"START: {start_x}")
        node.get_logger().info(f"MID:   {mid_x}")
        node.get_logger().info(f"END:   {end_x}")
        ret = movec(mid_x, end_x, vel=30, acc=30)
        node.get_logger().info(f"Movec complete with return code: {ret}")

    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
