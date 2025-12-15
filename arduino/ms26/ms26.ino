
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

// 2nd core interprocess stuff
volatile std::atomic<bool> core1InitBasicsFinished = false;
volatile std::atomic<bool> core2InitFinished = false;
volatile std::atomic<bool> core2CalImu = false;

// buffer TOF range data for inter-process transfer
#define TOFSTRUCT_LEN 2
volatile uint8_t tofStructWrIdx = 0;
volatile uint8_t tofStructRdIdx = 0;
volatile std::atomic<uint8_t> tofStructCnt = 0;
volatile struct {
  uint8_t range = 255;
} tofStruct[TOFSTRUCT_LEN];

// buffer TOF range data for inter-process transfer
#define IMUSTRUCT_LEN 2
volatile uint8_t imuStructWrIdx = 0;
volatile uint8_t imuStructRdIdx = 0;
volatile std::atomic<uint8_t> imuStructCnt = 0;
volatile struct {
  float yaw = 0;
} imuStruct[IMUSTRUCT_LEN];


///////////////// On Board LED ////////////////////////////
// Create led instance
Adafruit_NeoPixel neo(1, NEO, NEO_GRB + NEO_KHZ800);

bool neoRed   = 0;
bool neoGreen = 0;
bool neoBlue  = 0;

// update the RGB colors on the built in NEO LED
// Only updates when colors change
void setNeo(int r=0, int g=150, int b=0) {
  if(r!=neoRed | g!=neoGreen | b!=neoBlue) {
    neoRed   = r;
    neoGreen = g;
    neoBlue  = b;
    neo.clear();
    neo.setPixelColor(0, neo.Color(r, g, b));
    neo.show();   // Send the updated pixel colors to the hardware.
  }
}


void InitNeo(int r=0, int g=150, int b=0) {
  neo.begin();
  setNeo();
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
  // Pass TOF data to core1
  tofStruct[tofStructWrIdx].range = range;
  tofStructCnt++; /* atomic */
  if(tofStructWrIdx<TOFSTRUCT_LEN) tofStructWrIdx++;
  else tofStructWrIdx = 0;
}

uint8_t currentRange = 255;
// get current (or last) range in mm (typ 0 to 200), 255 if no range detected
// range value is saved in global variable
bool GetTofData(){
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
void Core2_CalImuOffsets() {
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

uint32_t lastYawMillis = 0;
float imuYaw = 0.0;

void Core2_InitImu() {
  Serial.println("IMU Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    return; //while (1);
  }

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");

  Core2_CalImuOffsets();

  // initialize IMU data buffer
  imuStructCnt = 0; /* atomic */
  imuStructWrIdx = imuStructRdIdx = 0;
}

void Core2_ProcImu() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
//    return;

    float ax, ay, az;
    float gx, gy, gz;

    IMU.readAcceleration(ax, ay, az);
    ax -= ax0;
    ay -= ay0;
    az -= az0;
    // Serial.print(ax*200);
    // Serial.print('\t');
    // Serial.print(ay*200);
    // Serial.print('\t');
    // Serial.print(az*200);

    IMU.readGyroscope(gx, gy, gz);
    gx -= gx0;
    gy -= gy0;
    gz -= gz0;
    // Serial.print('\t');
    // Serial.print(gx);
    // Serial.print('\t');
    // Serial.print(gy);
    // Serial.print('\t');
    // Serial.print(gz);
    // Serial.println();

    // Integrate angular velocity to get yaw
    // TODO: Use usec time?
    uint32_t currentMillis = millis();
    float dt = (currentMillis - lastYawMillis)/1000.0;
    lastYawMillis = currentMillis;
    if(dt<100) {
      // seems valid time difference integrate yaw with ang velocity
      imuYaw += dt*gy;
      // save into buffer for core1
      if(imuStructCnt < IMUSTRUCT_LEN) {
        imuStruct[imuStructWrIdx].yaw = imuYaw;
        imuStructWrIdx++;
        if(imuStructWrIdx >= IMUSTRUCT_LEN) imuStructWrIdx = 0;
        imuStructCnt++;
      }
    }
  }
}

float currentYaw = 0.0;
// Get IMU data from core2
void GetImuData(){
  if(imuStructCnt > 0){
    currentYaw = imuStruct[imuStructRdIdx].yaw;
    imuStructRdIdx++;
    if(imuStructRdIdx >= IMUSTRUCT_LEN) imuStructRdIdx = 0;
    imuStructCnt--;
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
#define VBAT_SCALE (3.3/(15.0/(22+15)))*(6.5/6.6)

void InitVbatMeas() {

}

float GetVbat() {
  int vbatPinValue = analogRead(VBAT_PIN);
  float vbat = vbatPinValue/float(VBAT_PIN_MAX) * VBAT_SCALE;
  return(vbat);
}

/////////////// Sensors /////////////////

#define FSM_TOF_MIN_RANGE  12
#define FSM_FWD_SPEED      100
#define FSM_REV_SPEED      FSM_FWD_SPEED
#define FSM_SPIN_SPEED     FSM_FWD_SPEED
#define FSM_MAX_SPIN_TIME  1.5*((25*1000)/FSM_SPIN_SPEED)
#define FSM_SPIN_ANGLE     100.0
#define FSM_BACKUP_TIME    ((25*500)/FSM_REV_SPEED)
#define FSM_TRIG_DET_TIME  1000 /* 1 second trigger "filter" to minimize random stops */
#define FSM_START_DELAY    5000 /* 5 sec time before starting movement */

/////////////////////// FSM for movement /////////////////////

enum fsmState{
INIT, IDLE0, IDLE1, WAIT_START, FWD, BACKUP, SPIN
};


struct {
  // trigger input
  bool trig = false;
  // edge sensors
  bool esFR = false;
  bool esFL = false;
  bool esRR = false;
  bool esRL = false;
  // object sensors
  uint8_t range = 0; // Front Center TOF sensor
  bool osFC = false;
  bool osFR = false;
  bool osFL = false;
  bool osRR = false;
  bool osRL = false;
  // spin direction +/- latched based on edge sensor
  int spinDir = 0; 
  // yaw value when SPIN begins - used to measure SPIN angle
  float yaw0 = 0.0;
  // state timer
  uint32_t timer = 0;
  // state info
  fsmState currentState = INIT;
  fsmState nextState = IDLE0;
} fsmData;

// Set the sensor data in the global FSM data structure
void setFsmSensorData(uint8_t range, uint16_t det) {
  static uint32_t timer = 0;
  fsmData.range = range;
  // edge sensors
  fsmData.esFR = det&0x0100;
  fsmData.esFL = det&0x0200;
  fsmData.esRR = det&0x0400;
  fsmData.esRL = det&0x0800;
  // object sensors
  fsmData.osFC = range!=255 && range>=FSM_TOF_MIN_RANGE; // 255 is no det
  fsmData.osFR = det&0x0001;
  fsmData.osFL = det&0x0002;
  fsmData.osRR = det&0x0004;
  fsmData.osRL = det&0x0008;

  // set spin direction based on the edge sensor tripped
  if     (fsmData.esFR) fsmData.spinDir = +1;
  else if(fsmData.esFL) fsmData.spinDir = -1;
  else if(fsmData.esRR) fsmData.spinDir = +1;
  else if(fsmData.esFL) fsmData.spinDir = -1;

  // trigger on range < min ie. finger covers FC sensor hole
  // this could help false triggers stopping the motion
  // trigger is set when range<min for 1 sec and then 255
  // reset timer when range > min including 255
  if(range>=2*FSM_TOF_MIN_RANGE) {
    // finger not covering TOF hole
    fsmData.trig = false;
    timer = millis();
  }
  else if(range<FSM_TOF_MIN_RANGE){
    // finger is covering TOF hole
    if(!fsmData.trig && ((millis() - timer) >= FSM_TRIG_DET_TIME)) {
      fsmData.trig = true;
    }
  }
}

void fsm(float &wheelR, float &wheelL) {
  static bool lastTrig = false;
  static bool esFR_latch = false;
  static bool esFL_latch = false;
  bool trig0 = false;
  bool trig1 = false;
  // set triggers when trig value changes
  if(fsmData.trig != lastTrig) {
    trig1 = fsmData.trig; // finger on FC
    trig0 = !fsmData.trig; // finger off FC
  }
  lastTrig = fsmData.trig;

  int r = 100*(fsmData.esFR|fsmData.esFL|fsmData.esRR|fsmData.esRL);
  int g = 100*(fsmData.osFR|fsmData.osFL|fsmData.osRR|fsmData.osRL);
  int b = int(100*((200-fsmData.range)/200.0));
  if (b<0) b=0;

  // default motor-wheel speed 0 percent
  wheelR = 0;
  wheelL = 0;

  bool stateChange = false;
  if(fsmData.nextState != fsmData.currentState) {
    stateChange = true;
  }
  fsmState state = fsmData.nextState;
  fsmState nextState = state; // default no state change

  // FSM
  switch(state) {
    case IDLE0: 
      // wait for finger on FC before start sequence
      if(trig1) {
        nextState = IDLE1;
      }
      // continuous show sensor status
      setNeo(r,g,b);
      break;

    case IDLE1: 
      if(stateChange) {
        setNeo(0,0,255); // BLUE
      }
      // wait for finger is off the FC to start delay before moving
      if(trig0) {
        nextState = WAIT_START;
      }
      break;

    case WAIT_START: 
      if(stateChange) {
        // wait before moving
        fsmData.timer = millis()+FSM_START_DELAY;
        // calibrate IMU on the mat
        core2CalImu = true;
        setNeo(255,0,0); // RED
      }

      if(trig1) {
        // stop and return to idle when finger on fc
        nextState = IDLE0;
      }
      else if(millis() >= fsmData.timer) {
        // 
        nextState = FWD;
      }
      break;
    
    case FWD: 
      if(stateChange) {
        esFR_latch = false;
        esFL_latch = false;
        setNeo(0,255,0); // GREEN
      }
      // latch front edge sensors
      esFR_latch |= fsmData.esFR;
      esFL_latch |= fsmData.esFL;

      if(trig1) {
        // stop and idle when finger on FC
        nextState = IDLE0;
      }
      // else if(fsmData.esFR | fsmData.esFL | fsmData.esRR | fsmData.esRL) {
      // ignore rear sensors for now - i may need a backup state for spin
      else if(!fsmData.osFC && (fsmData.esFR | fsmData.esFL)) {
        // edge sensor detected - brake immediately then start spin
        // TODO: check which sensors tripped and do whatever
        wheelR = 0;
        wheelL = 0;
        nextState = BACKUP;
//        nextState = SPIN;
      } 
      else if(fsmData.osFC && (esFR_latch & esFL_latch)) {
        // edge sensor detected - brake immediately then start spin
        // TODO: check which sensors tripped and do whatever
        wheelR = 0;
        wheelL = 0;
        nextState = BACKUP;
//        nextState = SPIN;
      } 
      else {
        // drive forward
        wheelR = FSM_FWD_SPEED;
        wheelL = FSM_FWD_SPEED;
        // Vear towards an object in front not in center
        if(fsmData.osFR & !fsmData.osFL) wheelR *= 0.8;
        if(fsmData.osFL & !fsmData.osFR) wheelL *= 0.8;
      }
      break;

    case BACKUP: 
      if(stateChange) {
        // backup a bit
        fsmData.timer = millis()+FSM_BACKUP_TIME;
        setNeo(0,0,255); // BLUE
      }
      if(trig1) {
        // stop and idle when finger on FC
        nextState = IDLE0;
      }
      else if(millis()>=fsmData.timer) {
        nextState = SPIN;
      }
      else if(fsmData.esRR | fsmData.esRL) {
        // react to rear edge sensors by spinning
        nextState = SPIN;
      }
      else {
        // drive reverse
        wheelR = -FSM_REV_SPEED;
        wheelL = -FSM_REV_SPEED;
      }
      break;

    case SPIN: 
      if(stateChange) {
        // wait then stop spinning
        fsmData.timer = millis()+FSM_MAX_SPIN_TIME;
        fsmData.yaw0 = currentYaw;
        setNeo(150,0,150); // PURPLE
      }

      // spin away from edge before driving forward again
      wheelR = +fsmData.spinDir*FSM_SPIN_SPEED;
      wheelL = -fsmData.spinDir*FSM_SPIN_SPEED;

      if(trig1) {
        // stop and idle when finger on FC
        nextState = IDLE0;
      }
      else if(millis()>=fsmData.timer) {
        nextState = FWD;
      }
      else if(fabs(currentYaw - fsmData.yaw0) >= FSM_SPIN_ANGLE) {
        // stop spinning when spin angle achieved
        nextState = FWD;
      }
      else if(fsmData.osFC | fsmData.osFR | fsmData.osFL) {
        // stop spinning when an object is detected at front
        nextState = FWD;
      }
      break;
    
    default: 
      setNeo(150,150,150); // WHITE
      // stop movement - freeze in unknown state until trigger
      wheelR = 0;
      wheelL = 0;
      if(trig1) {
        nextState = IDLE0;
      }
      break;
  }

  fsmData.nextState    = nextState;
  fsmData.currentState = state;
}


///////////////////////////////// SETUP - LOOP //////////////////////////

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
    setNeo(50,0,0); // RED
    delay(1000);
    return;
  }

  det = ~GetIRdet(); // invert so that det bit value 1 = active
  GetTofData(); // update current range is new range is available
  range = currentRange;
  GetImuData();

  setFsmSensorData(range, det);

  float wheelR = 0.0;
  float wheelL = 0.0;

  fsm(wheelR, wheelL);

  MotorDrivePct(wheelL, wheelR);

  uint32_t t = millis()-t0;
  Serial.printf("%Ld: %f, %d, %f, %f, %X, %f\n", t, vbat, range, wheelR, wheelL, det&0x0F0F, currentYaw);

  delay(0);
}

//////////////////////////// CORE 2 ///////////////////////////////

void setup1() {
  while(!core1InitBasicsFinished) delay(1); /* atomic */

  // core2 code
  Core2_InitI2c();
  Core2_InitTof();
  Core2_InitImu(); // TODO: update code for core2
  core2InitFinished = true; /* atomic */
}

void loop1(){
  // calibrate IMU command from core1
  if(core2CalImu) {
    Core2_CalImuOffsets();
    core2CalImu = false;
  }

  // code to put in 2nd core
  Core2_GetTofSensor();
  Core2_ProcImu();
}
