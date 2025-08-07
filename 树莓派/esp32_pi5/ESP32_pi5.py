import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)


def send_data(data):
    ser.write(f"{data}\n".encode())
    print(f": {data}")


def receive_and_execute():
    if ser.in_waiting:
        msg = ser.readline().decode().strip()
        if msg == "1":
            print("a")
        elif msg == "2":
            print("b")
        return msg
    return None


try:
    while True:
        receive_and_execute()
        time.sleep(0.1)

except KeyboardInterrupt:
    ser.close()
    print("22")