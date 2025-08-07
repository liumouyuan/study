import serial

ser = serial.Serial("/dev/ttyUSB0", 9600)


def sends_stm32(ss):
    stm32 = [ss]
    ser.write(bytearray(stm32))


def receive_stm32():
    ch = ser.read(2)
    return ch if ch else None


sends_stm32(0x02)
while True:
    if receive_stm32() == b'11':
        print("OK")
