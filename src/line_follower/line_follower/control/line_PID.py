#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32, Float32MultiArray, Bool

class line_follower_PID(Node):

    wheel_radius = 0.495576
    wheel_separation = 0.0762

    Kp = 1.0
    Ki = 0.0
    Kd = 0.1

    def __init__(self):
        super().__init__('line_follower_PID')

        self.previousError = 0.0
        self.integral = 0.0

        self.create_subscription(Int32MultiArray, '/sensor/line_follow_sensor', self.pid, 10)
        
        self.pub = self.create_publisher(Float32MultiArray, '/wheel_velocities_setpoint', 10)

        self.create_subscription(Float32, '/line_vel', self.cmd_vel_callback, 10)
        
        self.create_subscription(Bool, '/emergency_stop', self.cb, 10)

        self.line_vel = Float32()

        self.msg = Float32MultiArray()

    def cb(self, msg):
        pass

    def cmd_vel_callback(self, msg : Float32):    
        self.line_vel = msg

    def pid(self, msg : Int32MultiArray):
        v = self.line_vel.data      # Velocidad seteada en el vel_input.py

        error = self.getError(msg.data)

        P = error
        self.integral += error
        D = error - self.previousError

        output = self.Kp * P + self.Ki * self.integral + self.Kd * D
        self.previousError = error

        wr = (v / self.wheel_radius) + (self.wheel_separation * output) / (2 * self.wheel_radius)
        wl = (v / self.wheel_radius) - (self.wheel_separation * output) / (2 * self.wheel_radius)

        self.msg.data = [wr , wl]

        self.pub.publish(self.msg)

        

    def getError(self, values : list): 
        
        error = 0

        total_active = 0

        weights = [-5, -4, -3, -2, -1, 1, 2, 3, 4, 5]

        for i,val in enumerate(values):
            if val:
                error += weights[i]
                total_active += 1

        if total_active > 0:
            error = error / total_active
        else:
            error = 0

        return error



def main(args=None):
    rclpy.init(args=args)
    node = line_follower_PID()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
