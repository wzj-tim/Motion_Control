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
unsigned int tenMsCounter = 0;

//pwm输出用
uint16_t freq = 100;

//中断与编码器
uint8_t outFlag = 0;
uint8_t phaseA = 0;
uint8_t phaseB = 0;
int32_t EncoderTotal = 0;
int32_t lastEncoder = 0;
int32_t tenLastEncoder = 0;
float currentAngle = 0;
float currentSpeed = 0;
// bool isClockWise = false;

uint8_t profileType  = 0;
float recordAngle;
float expSpeed, expAngle, expAcc;
int16_t expDuty;
float time_f;
float record_time;
uint8_t record_index;
uint8_t new_graph;

//OLED test
uint8_t m = 24;
uint8_t recV[128];
uint8_t display_index = 0;

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
  ledcWrite(PWM_Channel,0);

  u8g2.firstPage();
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
    dutyTime=800;
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

float GetTrapSpeed(float t){
    if(t<=2.0f) return 0.4*t;
    else if(t<=3.927f)return 0.8f;
    else if(t<=5.927f)return 0.8f-0.4*(t-3.927f);
    else return 0;
}

float GetTrapAngle(float t){
    if(t<=2.0f) return 0.2*t*t;
    else if(t<=3.927f)return 0.8f+0.8f*(t-2);
    else if(t<=5.927f)return -0.8f+0.8f*t-0.2f*(t-3.927f)*(t-3.927f);
    else return 3.14159f;
}

float GetSSpeed(float t){
    if(t<=2.0f) return 0.075f*t*t;
    else if(t<=4.0f)return 0.6f-0.075f*(4-t)*(4-t);
    else if(t<=5.23f)return 0.6f;
    else if(t<=7.23f)return 0.6f-0.075f*(t-5.23f)*(t-5.23f);
    else if(t<=9.23f)return 0.075f*(9.23f-t)*(9.23f-t);
    else return 0;
}

float GetSAngle(float t){
    if(t<=2.0f) return 0.025*powf(t,3);
    else if(t<=4.0f)return 0.6f*(t-2)+0.025f*pow((4-t),3);
    else if(t<=5.23f)return 1.2f+0.6f*(t-4);
    else if(t<=7.23f)return 3.14f-0.6f*(7.23f-t)-0.025f*pow((t-5.23f),3);
    else if(t<=9.23f)return 3.14f-0.025f*pow((9.23f-t),3);
    else return 3.14159f;
}

uint16_t Acc2Duty(float acc, float vel){
  // float vmax = ((((0.01364f*vel-0.0448f)*vel+0.0579f)*vel-0.0372f)*vel+0.0123f)*vel-0.00068f;
  // int16_t duty = (int)(acc/(0.00773f*(vmax-vel)));
  int16_t duty = (int)(sqrtf(acc/0.00773f));
  if(duty<0)duty=0;
  duty = duty+50;
  if(duty>1023)duty=1023;
  return duty;
}

int16_t StateFeedback(float targetSpeed, float targetAngle){
  expAcc = 2400 * (targetSpeed - currentSpeed) + 7200 * (targetAngle-currentAngle);
  // return Acc2Duty(expAcc, currentSpeed);
  int16_t Duty = (int)expAcc;
  if(abs(Duty)<10)Duty=0;
  if(Duty>0)Duty += 60;
  else if(Duty<0)Duty -= 60;
  if(Duty>1023)Duty=1023;
  if(Duty<-1023)Duty=-1023;
  return Duty;
}

// the loop function runs over and over again forever
void loop() {
  currentMillis = millis();

  if(currentMillis - recordedMillis >= 1){
    msCounter += currentMillis - recordedMillis;
    tenMsCounter += currentMillis - recordedMillis;
    
    //execute every millisecond
    currentAngle = EncoderTotal/99.0f*PI/180;//*360.0f/810.0f/44.0f*PI/180;//rad
    // currentSpeed = (float)(EncoderTotal-lastEncoder)/(float)(currentMillis - recordedMillis)/99.0f*PI/180.0f;//rad/s
    // Serial.printf("%d,%d \n",EncoderTotal-lastEncoder,currentMillis - recordedMillis);
    lastEncoder = EncoderTotal;
    // Serial.printf("%f\n", currentAngle);
    if(tenMsCounter >= 10){//100Hz
      currentSpeed = (float)(EncoderTotal-tenLastEncoder)/(float)(tenMsCounter)/99.0f*PI/180.0f *1000.0f;//rad/s
      tenMsCounter -= 10;
      tenLastEncoder = EncoderTotal;
      // Serial.printf("%d\n",profileType);

      if(currentMillis > 2000){
        time_f = ((float)((currentMillis-2000)%40000)/1000.0f);
        if(time_f <20){
          if(profileType){
            profileType = 0;
            recordAngle = currentAngle;
            record_time = 0;
            record_index = 0;
            
            // Serial.printf("Set%f\n",currentAngle);
          }
          new_graph = 0;
          expSpeed = GetTrapSpeed(time_f);
          expAngle = GetTrapAngle(time_f)+recordAngle;
          expDuty = StateFeedback(expSpeed,expAngle);
          // Serial.printf("%d\n",expDuty);
          if(time_f > 10){
            if(time_f >= 12 && time_f < 15)new_graph = 1;
            else new_graph = 0;
            // if(record_index){
            //   if(new_graph){
            //     new_graph = 0;
            //     if(!u8g2.nextPage()){
            //       u8g2.firstPage();
            //       new_graph = 1;
            //     };
            //   }
            //   else{
            //     u8g2.drawPixel(record_index, 50);
            //     u8g2.drawPixel(record_index, 50);
            //     // u8g2.drawPixel(record_index, recV[record_index]);
            //     Serial.printf("%d,%d\n",record_index,recV[record_index]);
            //     record_index = (record_index+99)%100;
            //   }
            // // }
          } else{
              if(time_f-record_time>0.1f){
                record_time = time_f;
                recV[record_index++] = (int)(currentSpeed*64);
              }
          }
        } else {
          if(!profileType){
            profileType = 1;
            recordAngle = currentAngle;
            record_time = 0;
            record_index = 0;
            
            // Serial.printf("Set%f\n",currentAngle);
          }
          new_graph = 0;
          expSpeed = GetSSpeed(time_f-20.0f);
          expAngle = GetSAngle(time_f-20.0f)+recordAngle;
          expDuty = StateFeedback(expSpeed,expAngle);
          if(time_f > 30){
            if(time_f >= 32 && time_f < 35)new_graph = 1;
            else new_graph = 0;
            // if(record_index){
              // if(new_graph){
              //   new_graph = 0;
              //   if(!u8g2.nextPage()){
              //     u8g2.firstPage();
              //     new_graph = 1;
              //   };
              // }
              // else{
              //   u8g2.drawPixel(record_index, 50);
              //   u8g2.drawPixel(record_index, 50);
              //   // u8g2.drawPixel(record_index, recV[record_index]);
              //   Serial.printf("%d,%d\n",record_index,recV[record_index]);
              //   record_index = (record_index+99)%100;
              // }
            // }
          } else{
              if(time_f-record_time>0.1f){
                record_time = time_f;
                recV[record_index++] = (int)(currentSpeed*64);
              }
          }
        }
        
        if(expDuty>=0){
          SetClockwise();
          ledcWrite(PWM_Channel,expDuty);
        } else if(expDuty<0){
          SetCounterClockwise();
          ledcWrite(PWM_Channel,-expDuty);
        }
      }
    

      Serial.printf("/*%f,%f,%f,%f,%f,%f,%f*/\n",currentSpeed,expSpeed,currentAngle,expAngle,expAcc,(float)expDuty,time_f);

    //   if(u8g2.nextPage()){
    //   // u8g2.drawBox(64-20, 32-10, 40, 20);
    //   u8g2.drawPixel(10, 10);
    // } else{
    //   u8g2.firstPage();
    // }

    }
    
    // FreqTest();
    // DutyTest();

    

    recordedMillis = currentMillis;
  }
  if(msCounter>=1000){
    msCounter = msCounter - 1000;
    ///don't change above
  if(new_graph){
    u8g2.firstPage();
    do {
      for(int i = 0;i<100;i++){
        u8g2.drawPixel(i, 63-recV[i]);
      }
      
      // Serial.printf("%d,%d\n",record_index,recV[record_index]);
      // record_index = (record_index+99)%100;
    } while ( u8g2.nextPage() );
  }
    

    
    // Serial.printf("%d \n",currentMillis);//print time
    // SetClockwise();
    // if(currentSpeed <= 0)SetClockwise();
    // else SetCounterClockwise();
    // ledcWrite(0,200);
    // Serial.printf("%f \n",currentAngle);
    // if(freq<32000)freq = freq+1000;
    // Serial.printf("%d \n",freq);
    // ledcChangeFrequency(0,freq,8);

  
    // char m_str[3];
    // strcpy(m_str, u8x8_u8toa(m, 2));    
    // u8g2.firstPage();
    // do {
    //   u8g2.setFont(u8g2_font_logisoso62_tn);
    //   u8g2.drawStr(0,63,"9");
    //   u8g2.drawStr(33,63,":");
    //    u8g2.drawStr(50,63,m_str);
    // } while ( u8g2.nextPage() );
    // m++;
    // if ( m == 60 )m = 0;
  
    // u8g2.clearBuffer();					// clear the internal memory
    // u8g2.setFont(u8g2_font_ncenB08_tr);	// choose a suitable font
    // u8g2.drawStr(0,10,"Hello World!");	// write something to the internal memory
    // u8g2.sendBuffer();					// transfer internal memory to the display

  }
}
