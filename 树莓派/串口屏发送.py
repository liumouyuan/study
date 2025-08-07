import serial

def screen_send_once(command: str, port: str = '/dev/ttyUSB0', baud: int = 9600):  # command-> 't0.txt="2"'   'page 1'

    try:
        with serial.Serial(port, baud, timeout=1) as ser:
            ser.write(command.encode('utf-8'))
            ser.write(b'\xff\xff\xff')
    except serial.SerialException as e:
        print('閿欒 错误', e)


while True:
    screen_send_once('t0.txt="2"')