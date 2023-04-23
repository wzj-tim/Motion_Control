#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

#define I2C_DEV_ADDR 0x78
#define RGB_BUILTIN 2
#define CW_pin 17
#define CCW_pin 16
#define pwm_out 15
#define phaseA_pin 26
#define phaseB_pin 27


unsigned long recordedMillis, currentMillis;
unsigned int msCounter = 0;//计时用变量，决定控制频率

//pwm输出用
uint16_t freq = 60;

//中断与编码器
uint8_t outFlag = 0;
uint8_t phaseA = 0;
uint8_t phaseB = 0;
int32_t EncoderTotal = 0;
float currentAngle = 0;

//OLED test
uint8_t m = 24;

U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   
// the setup function runs once when you press reset or power the board

void setup() {
  // No need to initialize the RGB LED
  Serial.begin(115200);
  pinMode(RGB_BUILTIN, OUTPUT);
  pinMode(CW_pin, OUTPUT);
  pinMode(CCW_pin, OUTPUT);
  // Wire.begin();
  u8g2.begin();//OLED setup

  recordedMillis = currentMillis = millis();//Init timers

  ledcSetup(0,freq,8);//PWM setup
  ledcAttachPin(pwm_out,0);

  attachInterrupt(digitalPinToInterrupt(phaseA_pin), InterruptA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(phaseB_pin), InterruptB, CHANGE);

}

void InterruptA(){
  // outFlag = 1;
  phaseA = digitalRead(phaseA_pin);
  phaseB = digitalRead(phaseB_pin);
  if(phaseA && phaseB)EncoderTotal++;
  else if(phaseA)EncoderTotal--;
  else if(phaseB)EncoderTotal--;
  else EncoderTotal++;
  // Serial.printf("%d, %d \n", phaseA, phaseB);
}
void InterruptB(){
  // outFlag = 1;
  phaseA = digitalRead(phaseA_pin);
  phaseB = digitalRead(phaseB_pin);
  if(phaseB && phaseA)EncoderTotal--;
  else if(phaseB)EncoderTotal++;
  else if(phaseA)EncoderTotal++;
  else EncoderTotal--;
  // Serial.printf("%d, %d \n", phaseA, phaseB);
}

void SetClockwise(){
  digitalWrite(CW_pin, HIGH);
  digitalWrite(CCW_pin, LOW);
}

void SetCounterClockwise(){
  digitalWrite(CCW_pin, HIGH);
  digitalWrite(CW_pin, LOW);
}

// the loop function runs over and over again forever
void loop() {
  currentMillis = millis();
  // if(outFlag = 1){
  //   outFlag = 0;
  //   Serial.printf("%d, %d \n", phaseA, phaseB);
  // }
  if(currentMillis - recordedMillis >= 1){
    msCounter += currentMillis - recordedMillis;
    recordedMillis = currentMillis;
    //execute every millisecond

    if(u8g2.nextPage()){
      u8g2.drawBox(64-20, 32-10, 40, 20);
    } else{
      u8g2.firstPage();
    }

  }
  if(msCounter>=1000){
    msCounter = msCounter - 1000;
    ///don't change above
    // Serial.printf("%d \n",currentMillis);//print time
    SetClockwise();
    ledcWrite(0,200);
    // Serial.printf("%d \n",EncoderTotal);//print encoder
    currentAngle = EncoderTotal*360.0f/810.0f/44.0f;
    Serial.printf("%f \n",currentAngle);
    // if(freq<32000)freq = freq+1000;
    // Serial.printf("%d \n",freq);
    // ledcChangeFrequency(0,freq,8);
    // u8g2.firstPage();
    // do {
    //   u8g2.drawBox(64, 32, 20, 10);
    // } while ( u8g2.nextPage() );

  /*
    char m_str[3];
    strcpy(m_str, u8x8_u8toa(m, 2));    
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_logisoso62_tn);
      u8g2.drawStr(0,63,"9");
      u8g2.drawStr(33,63,":");
       u8g2.drawStr(50,63,m_str);
    } while ( u8g2.nextPage() );
    m++;
    if ( m == 60 )m = 0;
    */


    // Serial.printf("%d \n",currentMillis);
    //execute every second
    //Write message to the slave
    
    // Wire.beginTransmission(I2C_DEV_ADDR);
    // Wire.printf("Hello World! %u", i++);
    // uint8_t error = Wire.endTransmission(true);
    // Serial.printf("endTransmission: %u\n", error);

    // u8g2.clearBuffer();					// clear the internal memory
    // u8g2.setFont(u8g2_font_ncenB08_tr);	// choose a suitable font
    // u8g2.drawStr(0,10,"Hello World!");	// write something to the internal memory
    // u8g2.sendBuffer();					// transfer internal memory to the display

  }
  
// #ifdef RGB_BUILTIN
  // digitalWrite(RGB_BUILTIN, HIGH);   // Turn the RGB LED white
  // // Serial.print("off\n");
  // delay(1000);
  // digitalWrite(RGB_BUILTIN, LOW);    // Turn the RGB LED off
  // // Serial.print("on\n");
  // delay(1000);


}
