import serial
from serial.tools import list_ports
import time

class SerialController():
    def __init__(self):
        
        self.serial_devices = set()

        self.connection()

    def connection(self):
        devices = self.find_serial_candidates()

        for ser in self.serial_devices:
            devices.remove(ser)

        for device in devices:
            ser = serial.Serial(device ,115200,timeout=0.02)
            
            ret = self.try_serial(ser)
            if ret:
                self.serial_devices.add(ser)
                print(f"Añadiendo {device} a Serial as {ret}")
            else:
                ser.close()
        
        if len(self.serial_devices)>2:
            return True
        else:
            return self.connection()

    def find_serial_candidates(self):
        ports = list_ports.comports()
        out = []
        for p in ports:
            if ('USB' in p.device) or ('ACM' in p.device) or \
                ('USB' in (p.description or '')) or ('ACM' in (p.description or '')):
                out.append(p.device)
        return out

    def try_serial(self,serial : serial.Serial):
        begin = time.time()
        while 0.1 > time.time() - begin:
            if serial.in_waiting > 0:    
                line = serial.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("left"):
                    return "left"
                if line.startswith("right"):
                    return "right"
                if line.startswith("sensor"):
                    return "sensor"

        return False 
    

if __name__ == "__main__":
    hola = SerialController()