#!/usr/bin/env python3
import struct
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Mapeo de ejes
AXES = {
    0: 'LX',
    1: 'LY',   # Stick izquierdo Y -> /left/pwm
    3: 'RX',
    4: 'RY',   # Stick derecho Y -> /right/pwm
    5: 'RT',   # Gatillo derecho (dead man switch)
}

BUTTONS = {
    0: 'A', 1: 'B', 2: 'X', 3: 'Y',
    4: 'LB', 5: 'RB', 6: 'Back', 7: 'Start',
    8: 'Xbox', 9: 'LStick', 10: 'RStick'
}

EVENT_FORMAT = "IhBB"
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

class XboxToPWM(Node):
    def __init__(self):
        super().__init__('xbox_to_pwm')
        self.left_pub = self.create_publisher(Float32, '/left/pwm', 10)
        self.right_pub = self.create_publisher(Float32, '/right/pwm', 10)
        self.jsdev = '/dev/input/js0'

        # Inicialización de ejes
        self.axes = {'LY': 0, 'RY': 0, 'RT': -32768}
        self.last_pub_time = time.time()

        self.get_logger().info(f"🎮 Escuchando control Xbox en {self.jsdev}")
        self.run()

    def run(self):
        with open(self.jsdev, "rb") as js:
            while rclpy.ok():
                evbuf = js.read(EVENT_SIZE)
                if not evbuf:
                    continue

                time_sec, value, etype, number = struct.unpack(EVENT_FORMAT, evbuf)
                if etype == 2 and number in AXES:
                    axis = AXES[number]
                    self.axes[axis] = value
                    self.publish_pwm()
                elif etype == 1 and number in BUTTONS:
                    pass

    def map_axis_to_pwm(self, val):
        """Convierte un valor de joystick (-32768 a 32767) a rango 1000–2000."""
        val_norm = (val + 32768) / 65535.0  # Normaliza a [0, 1]
        return 1000.0 + val_norm * 1000.0   # Escala a [1000, 2000]

    def publish_pwm(self):
        ly = -self.axes['LY']     # Invertido: arriba positivo
        ry = self.axes['RY']     # Invertido: arriba positivo
        rt = self.axes['RT']

        # Normaliza RT (-32768 = suelto, 32767 = presionado)
        rt_norm = (rt + 32768) / 65535.0

        left_msg = Float32()
        right_msg = Float32()

        if rt_norm >= 0.95:
            left_msg.data = self.map_axis_to_pwm(ly)
            right_msg.data = self.map_axis_to_pwm(ry)
        else:
            # Dead-man suelto -> neutro 1500
            left_msg.data = 1500.0
            right_msg.data = 1500.0

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

        # Log cada 0.1 s
        if time.time() - self.last_pub_time > 0.05:
            self.get_logger().info(
                f"PWM -> Left={left_msg.data:.0f}, Right={right_msg.data:.0f}, RT={rt_norm:.2f}"
            )
            self.last_pub_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = XboxToPWM()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
