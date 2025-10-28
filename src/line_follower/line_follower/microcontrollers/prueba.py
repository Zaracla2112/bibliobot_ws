#!/usr/bin/env python3
import struct
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

AXES = {
    0: 'LX', 
    1: 'LY',
    5: 'RT', 
}
BUTTONS = {
    0: 'A',
    1: 'B',
    2: 'X',
    3: 'Y',
    4: 'LB',
    5: 'RB',
    6: 'Back',
    7: 'Start',
    8: 'Xbox',
    9: 'LStick',
    10: 'RStick',
}

EVENT_FORMAT = "IhBB"  # tiempo (uint32), valor (short), tipo (byte), número (byte)
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

class XboxToCmdVel(Node):
    def __init__(self):
        super().__init__('xbox_to_cmdvel')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.jsdev = '/dev/input/js0'
        self.axes = {'LX': 0, 'LY': 0, 'RT': 0}
        self.last_pub_time = time.time()

        self.get_logger().info(f"📡 Escuchando control Xbox en {self.jsdev}")
        self.run()

    def run(self):
        with open(self.jsdev, "rb") as js:
            while rclpy.ok():
                evbuf = js.read(EVENT_SIZE)
                if not evbuf:
                    continue

                time_sec, value, etype, number = struct.unpack(EVENT_FORMAT, evbuf)
                # Ejes
                if etype == 2 and number in AXES:
                    axis = AXES[number]
                    self.axes[axis] = value
                    self.publish_if_ready()

                # Botones (si quieres usar más adelante)
                elif etype == 1 and number in BUTTONS:
                    pass

    def publish_if_ready(self):
        # El eje de los sticks suele ir de -32768 a 32767
        lx = self.axes['LX'] / 32768.0
        ly = -self.axes['LY'] / 32768.0  # invertido: arriba positivo
        rt = self.axes['RT']

        # Algunos controladores dan RT de -32768 (no presionado) a 32767 (presionado)
        # Normalizamos:
        rt_norm = (rt + 32768) / 65535.0

        if rt_norm >= 0.95:  # solo si está casi completamente presionado
            twist = Twist()
            twist.linear.x = ly * 1.0  # escala de velocidad
            twist.angular.z = lx * 1.0
            self.publisher_.publish(twist)

            # Opcional: evitar saturar consola
            if time.time() - self.last_pub_time > 0.2:
                self.get_logger().info(f"🚗 Publicando -> linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")
                self.last_pub_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = XboxToCmdVel()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
