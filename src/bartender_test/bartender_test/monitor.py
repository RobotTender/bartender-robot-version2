#!/usr/bin/env python3
import rclpy
import sys
import time
from rclpy.node import Node
import DR_init

def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ['j', 'xyz']:
        print("Usage:")
        print("  ros2 run bartender_test monitor j      (Joint values)")
        print("  ros2 run bartender_test monitor xyz    (Cartesian values)")
        return

    monitor_type = sys.argv[1]
    
    # Initialize ROS2
    rclpy.init()
    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "e0509"
    
    node = rclpy.create_node('robot_monitor', namespace=ROBOT_ID)
    DR_init.__dsr__id, DR_init.__dsr__model, DR_init.__dsr__node = ROBOT_ID, ROBOT_MODEL, node

    # Import Doosan library after node initialization
    from DSR_ROBOT2 import get_current_posx, get_current_posj, DR_BASE
    
    print("\n" + "="*50)
    if monitor_type == 'j':
        print("MONITORING: Joint Values [J1, J2, J3, J4, J5, J6]")
    else:
        print("MONITORING: Cartesian Values [X, Y, Z, RX, RY, RZ]")
    print("Press Ctrl+C to stop")
    print("="*50 + "\n")

    try:
        while rclpy.ok():
            try:
                if monitor_type == 'j':
                    curr = get_current_posj()
                    # current_posj returns a list of 6 floats (posj class)
                    vals = [round(float(x), 3) for x in curr]
                else:
                    curr, sol = get_current_posx(ref=DR_BASE)
                    # current_posx returns (posx, sol)
                    vals = [round(float(x), 3) for x in curr]
                
                print(f"{vals},")
            except Exception as e:
                # print(f"Error: {e}") # Debug: uncomment to see why it fails
                pass
            
            # Polling at 1Hz
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
