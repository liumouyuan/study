import serial, time
ser = serial.Serial('/dev/rfcomm0', 115200)
while True:
    ser.write(b'2')
    time.sleep(1)
