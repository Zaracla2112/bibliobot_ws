import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

class line_velocity_publisher(Node):
    def __init__(self):
        super().__init__('line_vel_publisher')

        #self.line_vel_pub = self.create_publisher(Float32, '/line_vel', 10)

        self.pub = self.create_publisher(Float32MultiArray, '/wheel_velocities_setpoint', 10)

        self.pub_pwm = self.create_publisher(Float32, '/left/pwm', 10)

        self.line_vel = Float32MultiArray()
        self.line_vel.data = [0.0,0.0]

        self.vel = Float32()

        self.timer = self.create_timer(0.1, self.timer_callback)


    def timer_callback(self):

        #self.line_vel_pub.publish(self.vel)
        
        self.pub_pwm.publish(self.vel)
        #self.pub.publish(self.line_vel)

    def update_line_vel(self):
        while rclpy.ok():
                new_line_vel = input("Introduce el valor de line_vel (en m/s): ")

                self.vel.data = float(new_line_vel)
                #self.line_vel = Float32MultiArray()
                #self.line_vel.data = [float(new_line_vel.split(";")[0]), float(new_line_vel.split(";")[1])]
                self.get_logger().info(f"Nuevo valor de line_vel: {new_line_vel}")
            

def main(args=None):
    rclpy.init(args=args)
    node = line_velocity_publisher()

    import threading
    input_thread = threading.Thread(target=node.update_line_vel)
    input_thread.daemon = True
    input_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
