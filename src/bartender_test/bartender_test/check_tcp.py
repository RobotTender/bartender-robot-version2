import rclpy
import sys
from rclpy.node import Node
import DR_init

def main(args=None):
    rclpy.init(args=args)
    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "e0509"
    node = rclpy.create_node('tcp_check_node', namespace=ROBOT_ID)
    DR_init.__dsr__id, DR_init.__dsr__model, DR_init.__dsr__node = ROBOT_ID, ROBOT_MODEL, node

    from DSR_ROBOT2 import (get_current_posx, set_tcp, wait)
    
    # TCP OFFSET: zero (grasp point)
    BOTTLE_TCP = [0, 0, 0, 0, 0, 0]

    try:
        # Define and set TCP
        from DSR_ROBOT2 import add_tcp
        try:
            add_tcp("grasp_tcp", BOTTLE_TCP)
        except:
            pass
        set_tcp("grasp_tcp")
        wait(0.5)

        # Get Cartesian Pos
        posx = get_current_posx()
        from DSR_ROBOT2 import get_current_posj
        posj = get_current_posj()
        print(f"\nCURRENT JOINTS: {posj}")
        print(f"CURRENT GRASP TCP POSITION (zero offset): {posx}")
        print("-" * 30)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
