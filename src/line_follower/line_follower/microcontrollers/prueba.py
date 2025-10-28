import struct
import time

# Dispositivo del joystick
jsdev = '/dev/input/js0'

# Códigos de botones/axes según documentación del driver xboxdrv
AXES = {
    0: 'LX',  # Stick izquierdo X
    1: 'LY',
    2: 'LT',  # Gatillo izquierdo
    3: 'RX',  # Stick derecho X
    4: 'RY',
    5: 'RT',  # Gatillo derecho
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

# Formato de los eventos del joystick
EVENT_FORMAT = "IhBB"  # tiempo (uint32), valor (short), tipo (byte), número (byte)
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

print("Escuchando control Xbox en", jsdev)

with open(jsdev, "rb") as js:
    while True:
        evbuf = js.read(EVENT_SIZE)
        if evbuf:
            time_sec, value, etype, number = struct.unpack(EVENT_FORMAT, evbuf)
            if etype == 1:  # Botón
                if number in BUTTONS:
                    print(f"Botón {BUTTONS[number]} {'PRESIONADO' if value else 'LIBERADO'}")
            elif etype == 2:  # Eje analógico
                if number in AXES:
                    print(f"Eje {AXES[number]} valor: {value}")
