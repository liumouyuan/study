#include <Arduino.h>
const int RX_PIN = 16;   // 连接树莓派TX
const int TX_PIN = 17;   // 连接树莓派RX

void send(int num) {
  Serial2.println(num);  // 发送到树莓派
  Serial.print("ESP32发送: ");
  Serial.println(num);   // 本地打印
}
// 接收数据并执行对应动作
void receive_and_execute() 
{
  if (Serial2.available()) 
  {
    String msg = Serial2.readStringUntil('\n');
    if (msg == "1")
    {
      Serial.println("ESP32执行A"); 
    } 
    else if (msg == "2") {
      Serial.println("ESP32执行B"); 
    }
  }
}
void setup() 
{
  Serial.begin(115200);       // 初始化USB串口（用于调试打印）
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // 初始化硬件串口
}
void loop() 
{
 send(1);
}
