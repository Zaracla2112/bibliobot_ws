#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32
import serial
from serial.tools import list_ports
from math import pi

BAUDRATE = 115200

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

QOS_FAST = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE
)

class SerialControllerNode(Node):
    def __init__(self):
        super().__init__('serial_controller')

        self.left_position_publisher  = self.create_publisher(Float32, '/left/encoder_position',  10)
        self.left_encoder_publisher   = self.create_publisher(Float32, '/left/encoder_velocity',  10)
        self.right_position_publisher = self.create_publisher(Float32, '/right/encoder_position', 10)
        self.right_encoder_publisher  = self.create_publisher(Float32, '/right/encoder_velocity', 10)
        self.line_follow_publisher    = self.create_publisher(Int32MultiArray, '/sensor/line_follow_sensor', 10)


        self.create_subscription(Float32, '/left/pwm', self.left_wheel_callback, 10)
        self.create_subscription(Float32, '/right/pwm', self.right_wheel_callback, 10)

        self.ser_devices = []
        self.connect_serial()

        self.create_timer(0.01, self.timer_callback)

        self.create_timer(0.1, self.publishers_callback)
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0
        
        self.left_data = [0.0,0.0]
        self.right_data = [0.0,0.0]
        self.sensor_data = [0]*8

    def publishers_callback(self):
        msg = Float32()

        msg.data = self.left_data[0]*0.10472
        self.left_encoder_publisher.publish(msg)

        msg.data = self.left_data[1]
        self.left_position_publisher.publish(msg)

        msg.data = self.right_data[0]*0.10472
        self.right_encoder_publisher.publish(msg)

        msg.data = self.right_data[1]
        self.right_position_publisher.publish(msg)

        msg = Int32MultiArray()
        msg.data = self.sensor_data
        self.line_follow_publisher.publish(msg)

    def timer_callback(self):
        self.read_serial()
        self.print_serial()

    def left_wheel_callback(self, msg : Float32):
        self.left_wheel_velocity = msg.data

    def right_wheel_callback(self, msg : Float32):
        self.right_wheel_velocity = msg.data

    def connect_serial(self):
        devices = self.find_serial_devices()
        if devices:
            for device in devices:
                try:
                    
                    self.get_logger().info(f"Conectando a {device}")
                    ser = serial.Serial(device, BAUDRATE, timeout=0.02)
                    
                    self.ser_devices.append(ser)
                except Exception as e:
                    self.get_logger().error(f"Error abriendo puerto {device}: {e}")
        else:
            self.get_logger().warn("No se encontraron dispositivos seriales")

    def find_serial_devices(self):
        ports = list_ports.comports()
        devices = []
        for port in ports:
            if 'ACM' in port.description or 'USB' in port.description:
                devices.append(port.device)
                initialSerial = serial.Serial(port.device, BAUDRATE, timeout=0.02)
                initialSerial.close()

        return devices

    def read_serial(self):
        for ser in self.ser_devices:
            try:
                if ser.in_waiting > 0:
                    
                    line = ser.readline().decode('utf-8', errors='ignore').strip()

                    if line:
                        values = []

                        if line.startswith("left"):
                            ser.esp = "left"
                            values = self.process_sensor_data(line)
                            if values:
                                self.left_data[0] = float(values[0]) # velocidad

                                self.left_data[1] = float(values[1]) #posicion

                        elif line.startswith("right"):
                            ser.esp = "right"
                            values = self.process_sensor_data(line)
                            if values:
                                self.right_data[0] = float(values[0]) # velocidad

                                self.right_data[1] = float(values[1]) #posicion

                        elif line.startswith("sensor"):
                            ser.esp = "sensor"
                            ser
                            values = self.process_sensor_data(line)
                            if values:
                                self.sensor_data = values

            except Exception as e:
                self.get_logger().error(f"Error leyendo puerto: {e}")


    def print_serial(self):
        for ser in self.ser_devices:
            try:
                if not hasattr(ser, 'esp'):
                    continue
                
                if ser.esp == "left":
                    line = str(self.left_wheel_velocity)
                    ser.write(line.encode('utf-8') + b'\n')  

                if ser.esp == "right":
                    line = str(self.right_wheel_velocity)
                    ser.write(line.encode('utf-8') + b'\n')  

            except Exception as e:
                self.get_logger().error(f"Error escribiendo en el puerto serial: {e}")

    def process_sensor_data(self, line):
        values = []
        parts = line.split(';')
        if len(line) > 1:
            for x in parts[1:]:
                try:
                    values.append(int(x))
                except ValueError:
                    pass
        return values

def main(args=None):
    rclpy.init(args=args)
    node = SerialControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cerrar todos los puertos abiertos
        for ser in node.ser_devices:
            if ser and ser.is_open:
                ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
