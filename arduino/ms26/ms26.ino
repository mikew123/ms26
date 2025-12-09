
#include <Wire.h>
#include <SPI.h>

// #include <utility/imumaths.h>

#include <Adafruit_NeoPixel.h>
#include "RP2040_PWM.h"
#include "Adafruit_VL6180X.h"
#include <Arduino_LSM6DS3.h>

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
void InitI2c(int hz = 100000) {
  Wire.setClock(hz);
  Wire.setSCL(SCL0);
  Wire.setSDA(SDA0);
  Wire.begin();
}

/////////////// TOF sensor ////////////////////
Adafruit_VL6180X vl = Adafruit_VL6180X();

void InitTof() {
  Serial.println("Adafruit VL6180x TOF sensor init");
  if (! vl.begin(&Wire)) {
    Serial.println("Failed to find TOF sensor");
    while (1);
  }
  Serial.println("TOF sensor found!");
}

// Returns range in mm (typ 0 to 200), 255 if no range detected
uint8_t GetTof() {
  uint8_t range  = vl.readRange();
  uint8_t status = vl.readRangeStatus();
  // if(status==0) Serial.println(range);
  // else Serial.println("---");
  if(status != 0) range = 255;
  return(range);
}

/////////////////// IMU ////////////////////////
// The IMU is already instanced in the class - there can only be 1
// Calibration offsets read during init
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


void setup() {
  Serial.begin(115200);
  delay(2000);

  InitVbatMeas();
  InitVreg();

  InitI2c();

  InitNeo(150,150,150);
  InitMotorDrivers();
  InitTof();
  InitImu();
  InitIRdet();
}

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
  ProcImu();
  range = GetTof();

  int r = 150*(((det>>0)&0x0F)!=0x00);
  int g = 150*(((det>>8)&0x0F)!=0x00);
  int b = int(150*((200-range)/200.0));
  if (b<0) b=0;
  SetNeo(r,g,b);


  float wheelR = 0.0;
  float wheelL = 0.0;
  if(range<255 && range>minRange) {
    wheelR = rangeWheelSpeed;
    wheelL = rangeWheelSpeed;
  }
  
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

  // stop when edge is detected
  if((det&0x0F00) != 0x0000) {
    // front left sensor only turn left
    wheelR = 0.0;
    wheelL = 0.0;
  }

  MotorDrivePct(wheelL, wheelR);

  uint32_t t = millis()-t0;
//  Serial.print(t);

  Serial.printf("%Ld: %f, %d, %f, %f, %X\n", t, vbat, range, wheelR, wheelL, det&0x0F0F);

  delay(0);
}

