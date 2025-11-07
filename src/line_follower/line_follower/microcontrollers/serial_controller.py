#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32
import serial
from serial.tools import list_ports
import time

BAUDRATE = 115200


class SerialControllerNode(Node):
    def __init__(self):
        super().__init__('serial_controller')

        # Publishers
        self.left_position_publisher  = self.create_publisher(Float32, '/left/encoder_position',  10)
        self.left_encoder_publisher   = self.create_publisher(Float32, '/left/encoder_velocity',  10)
        self.right_position_publisher = self.create_publisher(Float32, '/right/encoder_position', 10)
        self.right_encoder_publisher  = self.create_publisher(Float32, '/right/encoder_velocity', 10)
        self.line_follow_publisher    = self.create_publisher(Int32MultiArray, '/sensor/line_follow_sensor', 10)

        # Subscribers
        self.create_subscription(Float32, '/left/pwm', self.left_wheel_callback, 10)
        self.create_subscription(Float32, '/right/pwm', self.right_wheel_callback, 10)

        # Timers
        self.create_timer(0.01, self.timer_callback)
        self.create_timer(0.1, self.publishers_callback)
        self.create_timer(1.0, self.reconnect_timer)  # <-- NUEVO timer para revisar conexión

        # Estados
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0
        self.left_data = [0.0, 0.0]
        self.right_data = [0.0, 0.0]
        self.sensor_data = [0] * 8
        self.serial_devices = set()

        # Conexión inicial
        self.connection()

    # -------------------------
    #  CONEXIÓN AUTOMÁTICA
    # -------------------------
    def connection(self):
        attempt = 0
        while len(self.serial_devices) < 3:
            attempt += 1
            try:
                devices = self.find_serial_candidates()
                devices = [d for d in devices if d not in [s.port for s in self.serial_devices if hasattr(s, 'port')]]

                for device in devices:
                    try:
                        ser = serial.Serial(device, BAUDRATE, timeout=0.02)
                        role = self.try_serial(ser)
                        if role:
                            ser.esp = role
                            self.serial_devices.add(ser)
                            self.get_logger().info(f"Conectado {device} como {role}")
                        else:
                            ser.close()
                    except Exception as e:
                        self.get_logger().warn(f"No se pudo abrir {device}: {e}")
                        continue

                if len(self.serial_devices) >= 3:
                    break

                self.get_logger().info(f"Intento {attempt}: {len(self.serial_devices)}/3 ESP32 conectadas, reintentando...")
                time.sleep(1.0)

            except Exception as e:
                self.get_logger().error(f"Error general en conexión: {e}")
                time.sleep(1.0)

    def find_serial_candidates(self):
        try:
            ports = list_ports.comports()
            return [
                p.device for p in ports
                if ('USB' in p.device) or ('ACM' in p.device)
                or ('USB' in (p.description or '')) or ('ACM' in (p.description or ''))
            ]
        except Exception as e:
            self.get_logger().error(f"Error buscando puertos: {e}")
            return []

    def try_serial(self, serial_obj: serial.Serial):
        start = time.time()
        while time.time() - start < 0.5:
            try:
                if serial_obj.in_waiting > 0:
                    line = serial_obj.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("left"):
                        return "left"
                    elif line.startswith("right"):
                        return "right"
                    elif line.startswith("sensor"):
                        return "sensor"
            except Exception:
                break
        return False

    # -------------------------
    #  RECONEXIÓN AUTOMÁTICA
    # -------------------------
    def reconnect_timer(self):
        """
        Revisa cada segundo si las 3 ESP32 siguen conectadas.
        Si falta alguna, ejecuta nuevamente connection().
        """
        try:
            # Filtra solo las conexiones abiertas
            active_devices = [ser for ser in self.serial_devices if ser.is_open]
            roles = {getattr(ser, 'esp', None) for ser in active_devices}
            missing = [r for r in ['left', 'right', 'sensor'] if r not in roles]

            if missing:
                self.get_logger().warn(f"Faltan {len(missing)} ESP32 ({', '.join(missing)}). Reintentando conexión...")
                # Cierra y limpia los que estén caídos
                for ser in list(self.serial_devices):
                    if not ser.is_open:
                        try:
                            ser.close()
                        except Exception:
                            pass
                        self.serial_devices.discard(ser)
                # Reintenta conectar los que faltan
                self.connection()
        except Exception as e:
            self.get_logger().error(f"Error en reconnect_timer: {e}")

    # -------------------------
    #  LOOP PRINCIPAL
    # -------------------------
    def timer_callback(self):
        try:
            self.read_serial()
            self.print_serial()
        except Exception as e:
            self.get_logger().error(f"Error en timer_callback: {e}")

    def read_serial(self):
        to_remove = []
        for ser in list(self.serial_devices):
            try:
                if not ser.is_open:
                    to_remove.append(ser)
                    continue

                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue

                    if line.startswith("left"):
                        ser.esp = "left"
                        vals = self.process_sensor_data(line)
                        if vals:
                            self.left_data = vals

                    elif line.startswith("right"):
                        ser.esp = "right"
                        vals = self.process_sensor_data(line)
                        if vals:
                            self.right_data = vals

                    elif line.startswith("sensor"):
                        ser.esp = "sensor"
                        vals = self.process_sensor_data(line)
                        if vals:
                            self.sensor_data = vals

            except Exception as e:
                self.get_logger().error(f"Error leyendo {getattr(ser, 'port', '?')}: {e}")
                to_remove.append(ser)

        for ser in to_remove:
            try:
                ser.close()
            except Exception:
                pass
            self.serial_devices.discard(ser)
            self.get_logger().warn(f"Puerto {getattr(ser, 'port', '?')} desconectado")

    def print_serial(self):
        to_remove = []
        for ser in list(self.serial_devices):
            try:
                if not hasattr(ser, 'esp') or not ser.is_open:
                    continue
                if ser.esp == "left":
                    ser.write(f"{self.left_wheel_velocity}\n".encode('utf-8'))
                elif ser.esp == "right":
                    ser.write(f"{self.right_wheel_velocity}\n".encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Error escribiendo en {getattr(ser, 'port', '?')}: {e}")
                to_remove.append(ser)
        for ser in to_remove:
            try:
                ser.close()
            except Exception:
                pass
            self.serial_devices.discard(ser)

    # -------------------------
    #  PUBLICADORES
    # -------------------------
    def publishers_callback(self):
        try:
            msg = Float32()
            msg.data = self.left_data[0] 
            self.left_encoder_publisher.publish(msg)

            msg.data = self.left_data[1]
            self.left_position_publisher.publish(msg)

            msg.data = self.right_data[0] 
            self.right_encoder_publisher.publish(msg)

            msg.data = self.right_data[1]
            self.right_position_publisher.publish(msg)

            imsg = Int32MultiArray()
            imsg.data = self.sensor_data
            self.line_follow_publisher.publish(imsg)
        except Exception as e:
            self.get_logger().error(f"Error publicando datos: {e}")

    # -------------------------
    #  UTILIDADES
    # -------------------------
    def left_wheel_callback(self, msg: Float32):
        self.left_wheel_velocity = msg.data

    def right_wheel_callback(self, msg: Float32):
        self.right_wheel_velocity = msg.data

    def process_sensor_data(self, line):
        try:
            parts = line.split(';')[1:]
            if line.startswith("sensor"):
                return [int(x) for x in parts if x.isdigit()]
            else:
                return [float(x) for x in parts if x]
        except Exception:
            return []


def main(args=None):
    rclpy.init(args=args)
    node = SerialControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for ser in node.serial_devices:
            try:
                if ser and ser.is_open:
                    ser.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
