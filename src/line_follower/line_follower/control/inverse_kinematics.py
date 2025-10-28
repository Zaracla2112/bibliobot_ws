#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math
# Este nodo se encarga de recibir comandos de velocidad y calcular las velocidades de las ruedas
# utilizando la cinemática inversa diferencial. Publica las velocidades en rad/s y rpm.
# La cinemática inversa diferencial se basa en la siguiente fórmula:
# wr = (v / r) + (d * ω) / (2 * r)
# wl = (v / r) - (d * ω) / (2 * r)
# Donde:
# wr: Velocidad de la rueda derecha en rad/s
# wl: Velocidad de la rueda izquierda en rad/s
# v: Velocidad lineal del robot en m/s
# ω: Velocidad angular del robot en rad/s

class DiffDriveKinematicsNode(Node):
    def __init__(self):
        super().__init__('diff_drive_ik_node')

        self.declare_parameter('wheel_radius', 0.495576)
        self.declare_parameter('wheel_separation', 0.0762)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value


        # Publicadores para la ESP32 (micro-ROS)
        self.pub = self.create_publisher(Float32MultiArray, '/wheel_velocities_setpoint', 10)

        # Suscripción al cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info('Nodo cinemática inversa diferencial iniciado.')

    def cmd_vel_callback(self, msg):
        v = msg.linear.x      # Velocidad lineal en m/s
        omega = msg.angular.z # Velocidad angular en rad/s

        # Cinemática inversa
        wr = (v / self.wheel_radius) + (self.wheel_separation * omega) / (2 * self.wheel_radius)
        wl = (v / self.wheel_radius) - (self.wheel_separation * omega) / (2 * self.wheel_radius)


        msg = Float32MultiArray()
        msg.data = [wl, wr]
        
        self.pub.publish(msg)

        self.get_logger().info(f'cmd_vel: v={v:.2f}, ω={omega:.2f} -> wr={wr:.2f} rad/s, wl={wl:.2f} rad/s')

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
