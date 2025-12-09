
#include <Wire.h>
#include <SPI.h>

// #include <utility/imumaths.h>

#include <Adafruit_NeoPixel.h>
#include "RP2040_PWM.h"
#include "Adafruit_VL6180X.h"
#include <Arduino_LSM6DS3.h>
#include <atomic>

// On board RGB LED 
#define NEO 16

// GPIO PINS
#define SDA0 0
#define SCL0 1

#define LINRL_PIN 4
#define OBSRL_PIN 5
#define OBSFL_PIN 6
#define LINFL_PIN 7

#define VREG_EN   8

#define PIN_MOTOR_L1      10
#define PIN_MOTOR_L2      11
#define PIN_MOTOR_R1      12
#define PIN_MOTOR_R2      13

#define LINFR_PIN 15
#define OBSFR_PIN 26
#define OBSRR_PIN 27
#define LINRR_PIN 28

#define VBAT_PIN  29


///////////////// On Board LED ////////////////////////////
// Create led instance
Adafruit_NeoPixel neo(1, NEO, NEO_GRB + NEO_KHZ800);

bool neoRed   = 0;
bool neoBlue  = 0;
bool neoGreen = 0;

void SetNeo(int r=0, int g=150, int b=0) {
  neo.clear();
  neo.setPixelColor(0, neo.Color(r, g, b));
  neo.show();   // Send the updated pixel colors to the hardware.
}


void InitNeo(int r=0, int g=150, int b=0) {
  neo.begin();
  SetNeo();
}

///////////// Wheel motors ///////////////////
//creates pwm instance for motor drive
RP2040_PWM* PWM_MOTOR_L1;
RP2040_PWM* PWM_MOTOR_L2;
RP2040_PWM* PWM_MOTOR_R1;
RP2040_PWM* PWM_MOTOR_R2;

// Motor PWM motorPwmFreq
float motorPwmFreq = 5000;

void InitMotorDrivers(){
  // initialize to stop-braked (100PCT FOR STOP BRAKED)
  PWM_MOTOR_L1 = new RP2040_PWM(PIN_MOTOR_L1, motorPwmFreq, 100);
  PWM_MOTOR_L2 = new RP2040_PWM(PIN_MOTOR_L2, motorPwmFreq, 100);
  PWM_MOTOR_R1 = new RP2040_PWM(PIN_MOTOR_R1, motorPwmFreq, 100);
  PWM_MOTOR_R2 = new RP2040_PWM(PIN_MOTOR_R2, motorPwmFreq, 100);
}

// +pct = fwd, -pct = rev, 0pct = stop-braked
// pct limited to +-100%
//void MotorDrivePct(int pctL, int pctR) {
void MotorDrivePct(float pctL, float pctR) {
  pctL = -pctL; // opposite rotation as Right Wheel
  // Limit to +-100 pct
  if(pctL>100) pctL=100;
  if(pctL<-100) pctL=-100;
  if(pctL>=0) {
    // Motor Left Forward
    PWM_MOTOR_L1->setPWM(PIN_MOTOR_L1, motorPwmFreq, 100);
    PWM_MOTOR_L2->setPWM(PIN_MOTOR_L2, motorPwmFreq, 100-pctL);
  } else {
    // Motor Left Reverse
     PWM_MOTOR_L1->setPWM(PIN_MOTOR_L1, motorPwmFreq, 100+pctL);
     PWM_MOTOR_L2->setPWM(PIN_MOTOR_L2, motorPwmFreq, 100);
  }

  pctR = +pctR;
  // Limit to +-100 pct
  if(pctR>100) pctR=100;
  if(pctR<-100) pctR=-100;
  if(pctR>=0) {
    // Motor Right Forward
    PWM_MOTOR_R1->setPWM(PIN_MOTOR_R1, motorPwmFreq, 100);
    PWM_MOTOR_R2->setPWM(PIN_MOTOR_R2, motorPwmFreq, 100-pctR);
  } else {
    // Motor Right Reverse
    PWM_MOTOR_R1->setPWM(PIN_MOTOR_R1, motorPwmFreq, 100+pctR);
    PWM_MOTOR_R2->setPWM(PIN_MOTOR_R2, motorPwmFreq, 100);
  }

}

/////////////// I2C ///////////////////////////
void Core2_InitI2c(int hz = 100000) {
  Wire.setClock(hz);
  Wire.setSCL(SCL0);
  Wire.setSDA(SDA0);
  Wire.begin();
}

/////////////// TOF sensor ////////////////////
// TOF sensor read and processed in core 2
Adafruit_VL6180X vl = Adafruit_VL6180X();

// buffer range data for inter-process transfer
#define TOFSTRUCT_LEN 2
volatile uint8_t tofStructWrIdx = 0;
volatile uint8_t tofStructRdIdx = 0;
volatile std::atomic<uint8_t> tofStructCnt = 0;
volatile struct {
  uint8_t range = 255;
} tofStruct[TOFSTRUCT_LEN];
uint8_t currentRange = 255;

void Core2_InitTof() {
  Serial.println("Adafruit VL6180x TOF sensor init");
  if (! vl.begin(&Wire)) {
    Serial.println("Failed to find TOF sensor");
    while (1);
  }
  Serial.println("TOF sensor found!");
  for(int i=0; i<TOFSTRUCT_LEN; i++) {
    tofStruct[i].range = 255;
  }
  tofStructCnt = 0; /* atomic */
  tofStructWrIdx = tofStructRdIdx = 0;
}

// Put TOF sensor range values value into the buffer
void Core2_GetTofSensor() {
  uint8_t range  = vl.readRange(); /* blocking */
  uint8_t status = vl.readRangeStatus();
  // if(status==0) Serial.println(range);
  // else Serial.println("---");
  if(status != 0) range = 255;
  // wait for buffer not full - core1 needs to read a value
  // TODO: timeout?
  while(tofStructCnt>=TOFSTRUCT_LEN){
    Serial.println("TOF struct full - can't write new value");
    delay(100);
  }
  tofStruct[tofStructWrIdx].range = range;
  tofStructCnt++; /* atomic */
  if(tofStructWrIdx<TOFSTRUCT_LEN) tofStructWrIdx++;
  else tofStructWrIdx = 0;
}

// gets current (or last) range in mm (typ 0 to 200), 255 if no range detected
// range value is saved in global variable
bool GetNewRange(){
  if(tofStructCnt>0){
    currentRange = tofStruct[tofStructRdIdx].range;
    tofStructCnt--; /* atomic */
    if(tofStructRdIdx<TOFSTRUCT_LEN) tofStructRdIdx++;
    else tofStructRdIdx = 0;
    return true;
  } 
  return false;
}

/////////////////// IMU ////////////////////////
// IMU read and processed in core 2
// The IMU is instanced in the class - there can only be 1
// Calibration offsets created during init
float ax0=0;
float ay0=0;
float az0=0;
float gx0=0;
float gy0=0;
float gz0=0;
#define IMU_CAL_CNT 100
#define IMU_DIS_CNT 100
void CalImuOffsets() {
  // discard some readings to clear out whatever
  float x, y, z;
  for(int i=0;i<IMU_DIS_CNT;i++){
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(x, y, z);
    }
  }

  // Average lots of readings to cal offsets
  gx0=0;
  gy0=0;
  gz0=0;
  int cnt = 0;
  while(cnt<IMU_CAL_CNT) {
    if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
      IMU.readGyroscope(x, y, z);
      gx0+=x;
      gy0+=y;
      gz0+=z;

     IMU.readAcceleration(x, y, z);
      ax0+=x;
      ay0+=y;
      az0+=z;

      cnt++;
    }
  }
  gx0/=IMU_CAL_CNT;
  gy0/=IMU_CAL_CNT;
  gz0/=IMU_CAL_CNT;
  ax0/=IMU_CAL_CNT;
  ay0/=IMU_CAL_CNT;
  az0/=IMU_CAL_CNT;
}

void InitImu() {
  Serial.println("IMU Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    return; //while (1);
  }

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");

  CalImuOffsets();
}

void ProcImu() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    return;

    float x, y, z;

    IMU.readAcceleration(x, y, z);
    x -= ax0;
    y -= ay0;
    z -= az0;
    Serial.print(x*200);
    Serial.print('\t');
    Serial.print(y*200);
    Serial.print('\t');
    Serial.print(z*200);

    IMU.readGyroscope(x, y, z);
    x -= gx0;
    y -= gy0;
    z -= gz0;
    Serial.print('\t');
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.print(z);
    Serial.println();
  }
}

////////////////////// IR detectors ////////////////

void InitIRdet() {
  pinMode(LINFR_PIN, INPUT);
  pinMode(LINFL_PIN, INPUT);
  pinMode(LINRR_PIN, INPUT);
  pinMode(LINRL_PIN, INPUT);
  pinMode(OBSFR_PIN, INPUT);
  pinMode(OBSFL_PIN, INPUT);
  pinMode(OBSRR_PIN, INPUT);
  pinMode(OBSRL_PIN, INPUT);
}

int16_t GetIRdet() {
  int16_t det = 0;
  det |= digitalRead(OBSFR_PIN)<<0;
  det |= digitalRead(OBSFL_PIN)<<1;
  det |= digitalRead(OBSRR_PIN)<<2;
  det |= digitalRead(OBSRL_PIN)<<3;
  det |= digitalRead(LINFR_PIN)<<8;
  det |= digitalRead(LINFL_PIN)<<9;
  det |= digitalRead(LINRR_PIN)<<10;
  det |= digitalRead(LINRL_PIN)<<11;
  return(det);
}

///////////////// 3.3V VREG /////////////////////
void VregEnable(bool on=true) {
  digitalWrite(VREG_EN, on);
  delay(10);
}

void InitVreg() {
  pinMode(VREG_EN, OUTPUT);
  VregEnable(true);
}

/////////////// VBAT measurement ////////////////
#define VBAT_MIN 2*3.3
#define VBAT_PIN_MAX 1023
#define VBAT_SCALE (3.3/(15.0/(22+15)))

void InitVbatMeas() {

}

float GetVbat() {
  int vbatPinValue = analogRead(VBAT_PIN);
  float vbat = vbatPinValue/float(VBAT_PIN_MAX) * VBAT_SCALE;
  return(vbat);
}

/////////////// Movement /////////////////
// stop before hitting for debug testing
#define minRange 20
#define rangeWheelSpeed 100
#define obsFRLWheelSpeed 50

// 2nd core stuff
volatile std::atomic<uint8_t> core1InitBasicsFinished = false;
volatile std::atomic<uint8_t> core2InitFinished = false;


void setup() {
  Serial.begin(115200);
  delay(2000);

  InitVbatMeas();
  InitVreg();

  // signal core2 that the basics are initialized
  core1InitBasicsFinished = true; /* atomic */

  InitNeo(150,150,150);
  InitMotorDrivers();
  InitIRdet();

  // wait for core2 init
  while(!core2InitFinished) delay(1); /* atomic */

}

uint32_t timer0 = 0;
void loop() {
  uint32_t t0 = millis();
  uint8_t range = 255;
  int16_t det = 0xFFFF;
  float vbat = 0.0;

  vbat = GetVbat();

  if(vbat<=VBAT_MIN && vbat>1.0 ) {
    // vbat<1 is DEBUG when no battery installed
    // remove power from sensors and motors
    // RP2040 still active
    // TODO: RP2040 lower power mode?
    Serial.printf("Vbat =  %f - shutting off sensors and motors", vbat);
    VregEnable(false);
    delay(1000);
    return;
  }

  det = ~GetIRdet();
  GetNewRange(); // update current range is new range is available
  range = currentRange;

  int r = 150*(((det>>0)&0x0F)!=0x00);
  int g = 150*(((det>>8)&0x0F)!=0x00);
  int b = int(150*((200-range)/200.0));
  if (b<0) b=0;
  SetNeo(r,g,b);


  float wheelR = 0.0;
  float wheelL = 0.0;

  // no movement when finger over sensor opening
  if(range>=minRange) {
    uint16_t edgeSensors = (det&0x0F00)>>8;
    if(edgeSensors || timer0!=0) {
      // the edge is detected
      if(timer0 == 0) timer0 = millis();
      uint32_t dt = millis()-timer0;
      // move away from edge for a bit
      if(dt<100) {
        if(edgeSensors&0x2) {
          // Front Left sensor - move back and rotate CW
          wheelR = -100.0;
          wheelL = -50.0;
        }
        else if(edgeSensors&0x1) {
          // Front Right sensor - move back and rotate CCW
          wheelR = -50.0;
          wheelL = -100.0;
        }
        else if(edgeSensors&0x8) {
          // Rear Left sensor - move Forward and rotate CCW
          wheelR = 50.0;
          wheelL = 100.0;
        }
        else if(edgeSensors&0x4) {
          // Rear Left sensor - move Forward and rotate CCW
          wheelR = 100.0;
          wheelL = 500.0;
        }
      } else {
        // reset edge timer
        timer0 = 0;
        wheelR = 0.0;
        wheelL = 0.0;
      }
    } else {
      if(range<255) {
        wheelR = rangeWheelSpeed;
        wheelL = rangeWheelSpeed;
      }
      
      // front sensors
      if((det&0x0001) == 0x0001) {
        // front right sensor only turn right
        wheelR -= obsFRLWheelSpeed;
        wheelL += obsFRLWheelSpeed;
      }
      if((det&0x0002) == 0x0002) {
        // front left sensor only turn left
        wheelR += obsFRLWheelSpeed;
        wheelL -= obsFRLWheelSpeed;
      }
      if((det&0x0003) == 0x0003) {
        // drive forward
        wheelR = rangeWheelSpeed;
        wheelL = rangeWheelSpeed;
      }

      // rear sensors
      if(det&0x000C) {
        // drive reverse
        wheelR = -rangeWheelSpeed;
        wheelL = -rangeWheelSpeed;
      }
      if(det&0x0004) {
        // rear right sensor only turn right
        wheelR += obsFRLWheelSpeed;
        wheelL -= obsFRLWheelSpeed;
      }
      if(det&0x0008) {
        // rear left sensor only turn left
        wheelR -= obsFRLWheelSpeed;
        wheelL += obsFRLWheelSpeed;
      }
    }
  }

  MotorDrivePct(wheelL, wheelR);

  uint32_t t = millis()-t0;
  Serial.printf("%Ld: %f, %d, %f, %f, %X\n", t, vbat, range, wheelR, wheelL, det&0x0F0F);

  delay(0);
}

//////////////////////////// CORE 2 ///////////////////////////////

void setup1() {
  while(!core1InitBasicsFinished) delay(1); /* atomic */

  // core2 code
  Core2_InitI2c();
  Core2_InitTof();
  InitImu(); // TODO: update code for core2
  core2InitFinished = true; /* atomic */
}

void loop1(){
  // code to put in 2nd core
  Core2_GetTofSensor();
  ProcImu();
}
