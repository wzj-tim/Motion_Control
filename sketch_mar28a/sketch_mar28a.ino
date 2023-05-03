#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

#define PI 3.14159265f

#define I2C_DEV_ADDR 0x78
#define RGB_BUILTIN 2
#define CW_pin 17
#define CCW_pin 16
#define PWM_Channel 0
#define PWM_Bits 10
#define PWM_Out 15
#define phaseA_pin 26
#define phaseB_pin 27

typedef struct{
  uint16_t frequency;
  float speed;
} FreqRecord;

typedef struct{
  uint16_t duty;
  float speed;
} DutyRecord;

FreqRecord freqRecord[400];
DutyRecord dutyRecord[200];



unsigned long recordedMillis, currentMillis;
unsigned int msCounter = 0;//计时用变量，决定控制频率

//pwm输出用
uint16_t freq = 100;

//中断与编码器
uint8_t outFlag = 0;
uint8_t phaseA = 0;
uint8_t phaseB = 0;
int32_t EncoderTotal = 0;
int32_t lastEncoder = 0;
float currentAngle = 0;
float currentSpeed = 0;

//OLED test
uint8_t m = 24;

//Test output memort
uint8_t mem[200];
uint8_t memIdx = 0;

U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   
// the setup function runs once when you press reset or power the board

void setup() {
  // No need to initialize the RGB LED
  Serial.begin(115200);
  pinMode(RGB_BUILTIN, OUTPUT);
  pinMode(CW_pin, OUTPUT);
  pinMode(CCW_pin, OUTPUT);

  u8g2.begin();//OLED setup

  recordedMillis = currentMillis = millis();//Init timers

  ledcSetup(PWM_Channel,freq,PWM_Bits);//PWM setup
  ledcAttachPin(PWM_Out,PWM_Channel);

  attachInterrupt(digitalPinToInterrupt(phaseA_pin), InterruptA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(phaseB_pin), InterruptB, CHANGE);

  // ledcChangeFrequency(PWM_Channel,100,PWM_Bits);
  ledcWrite(PWM_Channel,50);

}

void InterruptA(){
  phaseA = digitalRead(phaseA_pin);
  phaseB = digitalRead(phaseB_pin);
  if(phaseA && phaseB)EncoderTotal++;
  else if(phaseA)EncoderTotal--;
  else if(phaseB)EncoderTotal--;
  else EncoderTotal++;
  // Serial.printf("%d, %d \n", phaseA, phaseB);
}
void InterruptB(){
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

void FreqTest(){
  static uint16_t testFreq = 5;
  static uint16_t testIndex = 0;
  static uint16_t freqTime = 5000;
  static float testSpeed = 0;

  if(freqTime > 0){
    freqTime--;
    // Serial.printf("%d\n",freqTime);
    testSpeed = 0.95f*testSpeed + 0.05f*currentSpeed;
  }
  else{
    freqRecord[testIndex].frequency = testFreq;
    freqRecord[testIndex].speed = testSpeed;
    testIndex++;
    testSpeed = 0;
    testFreq+=5;
    freqTime=200;
    Serial.printf("%d\n",testFreq);
    ledcChangeFrequency(PWM_Channel,testFreq,PWM_Bits);
    ledcWrite(PWM_Channel,800);
  }

  if(testFreq > 1805){
    freqTime = 200;
    PrintMemory();
  }

}

void DutyTest(){
  static uint16_t testDuty = 50;
  static uint16_t testIndex = 0;
  static uint16_t dutyTime = 3000;
  static float testSpeed = 0;

  if(dutyTime > 0){
    dutyTime--;
    // Serial.printf("%d\n",freqTime);
    testSpeed = 0.95f*testSpeed + 0.05f*currentSpeed;
  }
  else{
    dutyRecord[testIndex].duty = testDuty;
    dutyRecord[testIndex].speed = testSpeed;
    testIndex++;
    testSpeed = 0;
    testDuty += 5;
    dutyTime=400;
    // Serial.printf("%d\n",testDuty);
    ledcWrite(PWM_Channel,testDuty);
  }

  if(testDuty > 1020){
    dutyTime = 400;
    PrintMemory();
  }

}

void PrintMemory(){
  static uint16_t outIndex = 0;
  // if(outIndex<361){
  //   Serial.printf("%d, %f \n",freqRecord[outIndex].frequency,freqRecord[outIndex].speed);
  //   outIndex++;
  // }

  if(dutyRecord[outIndex].duty>0 && outIndex<199){
    Serial.printf("%d, %f \n",dutyRecord[outIndex].duty,dutyRecord[outIndex].speed);
    outIndex++;
  }

}

// the loop function runs over and over again forever
void loop() {
  currentMillis = millis();

  if(currentMillis - recordedMillis >= 1){
    msCounter += currentMillis - recordedMillis;
    
    //execute every millisecond
    currentAngle = EncoderTotal/99.0f*PI/180;//*360.0f/810.0f/44.0f*PI/180;//rad
    currentSpeed = (float)(EncoderTotal-lastEncoder)/(float)(currentMillis - recordedMillis)/99.0f*PI/180.0f;//rad/s
    // Serial.printf("%d,%d \n",EncoderTotal-lastEncoder,currentMillis - recordedMillis);
    lastEncoder = EncoderTotal;
    
    // FreqTest();
    DutyTest();

    // if(u8g2.nextPage()){
    //   u8g2.drawBox(64-20, 32-10, 40, 20);
    // } else{
    //   u8g2.firstPage();
    // }

    recordedMillis = currentMillis;
  }
  if(msCounter>=1000){
    msCounter = msCounter - 1000;
    ///don't change above
    // Serial.printf("%d \n",currentMillis);//print time
    SetCounterClockwise();
    // SetClockwise();
    // ledcWrite(0,200);
    // Serial.printf("%f \n",currentAngle);
    // if(freq<32000)freq = freq+1000;
    // Serial.printf("%d \n",freq);
    // ledcChangeFrequency(0,freq,8);

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

    // u8g2.clearBuffer();					// clear the internal memory
    // u8g2.setFont(u8g2_font_ncenB08_tr);	// choose a suitable font
    // u8g2.drawStr(0,10,"Hello World!");	// write something to the internal memory
    // u8g2.sendBuffer();					// transfer internal memory to the display

  }
}
