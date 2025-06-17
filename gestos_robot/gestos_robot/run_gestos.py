# gestos_robot/run_gestos.py

import rclpy
from gestos_robot import GestosNode

def main(args=None):
    rclpy.init(args=args)
    node = GestosNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()