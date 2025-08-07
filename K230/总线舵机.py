import time
from machine import UART
from machine import FPIOA

# UART 配置
UART_PORT = UART.UART1
UART_TX_PIN = 3
UART_RX_PIN = 4
BAUD_RATE = 9600

# LOBOT 舵机控制协议相关定义(脉冲大-逆时针     脉冲小-顺时针)
LOBOT_FRAME_HEADER = 0x55
LOBOT_CMD_SERVO_MOVE = 3
LOBOT_CMD_ACTION_GROUP_RUN = 6
LOBOT_CMD_ACTION_GROUP_STOP = 7
LOBOT_CMD_ACTION_GROUP_SPEED = 11
LOBOT_CMD_GET_BATTERY_VOLTAGE = 15

# 初始化 UART
fpioa = FPIOA()
fpioa.set_function(UART_TX_PIN, FPIOA.UART1_TXD)
fpioa.set_function(UART_RX_PIN, FPIOA.UART1_RXD)
uart = UART(UART_PORT, BAUD_RATE)  # 设置串口号和波特率

# 控制单个总线舵机转动
def setBusServoMove(servo_id, servo_pulse, time):
    buf = bytearray(b'\x55\x55')  # 帧头
    buf.append(0x08)  # 数据长度
    buf.append(LOBOT_CMD_SERVO_MOVE)  # 指令
    buf.append(0x01)  # 要控制的舵机个数

    time = 0 if time < 0 else time
    time = 30000 if time > 30000 else time
    time_list = list(time.to_bytes(2, 'little'))  # 时间
    buf.append(time_list[0])
    buf.append(time_list[1])

    servo_id = 254 if (servo_id < 1 or servo_id > 254) else servo_id
    buf.append(servo_id)  # 舵机 ID

    servo_pulse = 0 if servo_pulse < 0 else servo_pulse
    servo_pulse = 1000 if servo_pulse > 1000 else servo_pulse
    pulse_list = list(servo_pulse.to_bytes(2, 'little'))  # 位置
    buf.append(pulse_list[0])
    buf.append(pulse_list[1])

    uart.write(buf)  # 发送数据

# 控制多个总线舵机转动
def setMoreBusServoMove(servos, servos_count, time):
    buf = bytearray(b'\x55\x55')  # 帧头
    buf.append(servos_count * 3 + 5)  # 数据长度
    buf.append(LOBOT_CMD_SERVO_MOVE)  # 指令

    servos_count = 1 if servos_count < 1 else servos_count
    servos_count = 254 if servos_count > 254 else servos_count
    buf.append(servos_count)  # 要控制的舵机个数

    time = 0 if time < 0 else time
    time = 30000 if time > 30000 else time
    time_list = list(time.to_bytes(2, 'little'))
    buf.append(time_list[0])  # 时间
    buf.append(time_list[1])

    for i in range(servos_count):
        buf.append(servos[i * 2])  # 舵机 ID
        pos = servos[i * 2 + 1]
        pos = 0 if pos < 0 else pos
        pos = 1000 if pos > 1000 else pos
        pos_list = list(pos.to_bytes(2, 'little'))
        buf.append(pos_list[0])  # 位置
        buf.append(pos_list[1])

    uart.write(buf)  # 发送数据

# 主循环：测试舵机控制
if __name__ == '__main__':
    while True:
        # 控制单个舵机
        setBusServoMove(1, 1000, 1000)  # 舵机 ID=1，脉冲=500，时间=1000ms
        time.sleep(2)

        #setBusServoMove(1, 1000, 1000)  # 舵机 ID=1，脉冲=1000，时间=1000ms
        #time.sleep(2)

        # 控制多个舵机
        #servos = [1, 500, 2, 1000]  # 舵机 ID=1，脉冲=500；舵机 ID=2，脉冲=1000
        #setMoreBusServoMove(servos, 2, 1000)  # 控制 2 个舵机，时间=1000ms
        #time.sleep(2)

        #servos = [1, 1000, 2, 500]  # 舵机 ID=1，脉冲=1000；舵机 ID=2，脉冲=500
        #setMoreBusServoMove(servos, 2, 1000)  # 控制 2 个舵机，时间=1000ms
        #time.sleep(2)