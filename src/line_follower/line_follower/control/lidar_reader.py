import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class lidar_reader(Node):
    def __init__(self):
        super().__init__('lidar_reader')

        self.publisher = self.create_publisher(Bool, "/emergency_stop", 10)

def main(args=None):
    rclpy.init(args=args)
    node = lidar_reader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()