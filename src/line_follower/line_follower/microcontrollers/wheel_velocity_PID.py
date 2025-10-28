import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32,Float32MultiArray

class WheelSpeedPIDController(Node):

    Kp_izq = 0.001
    Ki_izq = 0.01
    Kd_izq = 0.005

    Kp_der = 0.00
    Ki_der = 0.01
    Kd_der = 0.0005

    min_pwm = 1000 
    max_pwm = 2000  

    def __init__(self):
        super().__init__('wheel_velocity_PID')

        self.create_subscription(Float32MultiArray, '/wheel_velocities_setpoint', self.wheel_velocities_callback, 10)

        self.create_subscription(Float32, '/left/encoder_velocity', self.left_encoder_velocity_callback, 10)
        self.create_subscription(Float32, '/right/encoder_velocity', self.right_encoder_velocity_callback, 10)

        self.left_pwm_pub = self.create_publisher(Float32, '/left/pwm', 10)
        self.right_pwm_pub = self.create_publisher(Float32, '/right/pwm', 10)

        self.left_setpoint = 0.0
        self.right_setpoint = 0.0
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        self.left_error = 0.0
        self.right_error = 0.0
        self.left_integral = 0.0
        self.right_integral = 0.0
        self.previous_left_error = 0.0
        self.previous_right_error = 0.0

        
        self.setpoints = [0.0,0.0]
        
        self.timer = self.create_timer(0.1, self.control_pid)



    def wheel_velocities_callback(self, msg: Float32MultiArray):
        self.setpoints = msg.data

    def left_encoder_velocity_callback(self, msg: Float32):
        self.left_wheel_velocity = (msg.data) 

    def right_encoder_velocity_callback(self, msg: Float32):
        self.right_wheel_velocity = (msg.data)

    def control_pid(self):
        left_error = self.setpoints[0] - self.left_wheel_velocity
        right_error = self.setpoints[1] - self.right_wheel_velocity

        self.left_integral += left_error
        self.right_integral += right_error

        left_derivative = left_error - self.previous_left_error
        right_derivative = right_error - self.previous_right_error

        left_output = (self.Kp_izq * left_error + self.Ki_izq * self.left_integral + self.Kd_izq * left_derivative)
        right_output =  (self.Kp_der * right_error + self.Ki_der * self.right_integral + self.Kd_der * right_derivative)

        self.previous_left_error = left_error
        self.previous_right_error = -right_error

        self.publish_pwm(self.map_pid_to_pwm(left_output), self.map_pid_to_pwm(right_output))


    def map_pid_to_pwm(self, pid_output):
        return self.min_pwm + (pid_output + 1) * (self.max_pwm - self.min_pwm) / 2
    
    def publish_pwm(self, left_pwm, right_pwm):
        msg_left = Float32()
        msg_right = Float32()

        msg_left.data = float(max(1000, min(left_pwm, 2000)))
        msg_right.data = float(max(1000, min(right_pwm, 2000)))
        
        self.left_pwm_pub.publish(msg_left)
        self.right_pwm_pub.publish(msg_right)

def main(args=None):
    rclpy.init(args=args)
    node = WheelSpeedPIDController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
