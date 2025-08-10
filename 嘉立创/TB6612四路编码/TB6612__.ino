const int PIN_PWMA = 32;  
const int PIN_AIN1 = 33;
const int PIN_AIN2 = 25;
const int PIN_STBY = 26;
 
// PWM频率和分辨率设置
const int pwmFrequency = 10000;  // Hz
const int bitResolution = 8;     // pwm值范围: 0~255

void setup() {
  Serial.begin(115200);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);
 
  // 电机PWM输出设置(通道、频率、位深)
  ledcAttach(PIN_PWMA, pwmFrequency, bitResolution);
}
 
void loop() {
  
  digitalWrite(PIN_AIN1, HIGH); 
  digitalWrite(PIN_AIN2, LOW);  
  ledcWrite(PIN_PWMA, 250); 
  delay(1000);  
  ledcWrite(PIN_PWMA, 0);
  delay(1000);
  digitalWrite(PIN_AIN1, LOW);  
  digitalWrite(PIN_AIN2, HIGH);   
  ledcWrite(PIN_PWMA, 50); 
  delay(1000);
  ledcWrite(PIN_PWMA, 0);
  delay(1000);

}
 
