#include "SimpleRSLK.h"


// Debug Control
#define DEBUG 0
#define PLX 0

#if DEBUG == 1
#define debugln(x) Serial.println(x)
#define output(x) Serial.println(x) // Print newline char when debugging for clarity
#define debugDelay(x) delay(x)      // For Debug Serial Monitor Delay
#else
#define debugln(x)
#define output(x) Serial.print(x) // Don't print newline char when not debugging
#define debugDelay(x)
#endif

#if PLX == 1
#define PLXout(x) Serial.print(x)
#define PLXoutln(x) Serial.println(x)
#else
#define PLXout(x)
#define PLXoutln(x)
#endif

void setupEncoder(uint8_t ela_pin, uint8_t elb_pin, uint8_t era_pin, uint8_t erb_pin); // initalize the encoders

const int trigPin = 32; // This is Port Pin 3.5 on the MSP432 Launchpad
const int echoPin = 33; // This is Port Pin 5.1 on the MSP432 Launchpad

uint8_t lineColor = DARK_LINE; // DARK_LINE or LIGHT_LINE
bool isCalibrationComplete = false;

uint8_t Speed = 20;
uint8_t LSpeed = 20;
uint8_t RSpeed = 20;

uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];

// P Controller
double Ke = (0.001); // Range of output/max error
double DR = 0.5;
double correction = 100; // Correction for Distance from Axle
int center = 3500;

void setup()
{
  Serial.begin(9600);

  PLXoutln("CLEARDATA");
  PLXoutln("LABEL, Time, Time From Start, Sensor Output, Drive Ratio, Error");
  PLXoutln("RESETTIMER");

  setupRSLK();
  setupWaitBtn(LP_LEFT_BTN);  /* Left button on Launchpad */
  setupLed(RED_LED);   /* Red led in rgb led */
  clearMinMax(sensorMinVal, sensorMaxVal);

  delay(1000);
}

void simpleCalibrate()
{
  //Calibrate with Multiple Data
  for (int x = 0; x < 100; x++)
  {
    readLineSensor(sensorVal);
    setSensorMinMax(sensorVal, sensorMinVal, sensorMaxVal);
 
  }
}

void floorCalibration()
{
  /* Place Robot On Floor (no line) */
  delay(2000);
  String btnMsg = "Push left button on Launchpad to begin calibration.\n";
  btnMsg += "Make sure the robot is on the floor away from the line.\n";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN, btnMsg, RED_LED); //Turn off for system ID

  delay(1000);

  debugln("Running calibration on floor");
  simpleCalibrate();
  debugln("Reading floor values complete");

  btnMsg = "Push left button on Launchpad to begin line following.\n";
  btnMsg += "Make sure the robot is on the line.\n";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN, btnMsg, RED_LED); //Turn off for system ID
  delay(1000);

  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
}

uint32_t getPosition()
{
  readLineSensor(sensorVal);
  readCalLineSensor(sensorVal, sensorCalVal, sensorMinVal, sensorMaxVal, lineColor);
  // could use average
  return getLinePosition(sensorCalVal, lineColor);
}


void loop()
{
  uint32_t lastPos;
  /* Run this setup only once */
  if (isCalibrationComplete == false)
  {
    floorCalibration();
    isCalibrationComplete = true;
    lastPos = getPosition();
  }

//  uint32_t linePos1 = getPosition();
//  uint32_t linePos2 = getPosition();
//  uint32_t linePos3 = getPosition();
//  uint32_t linePos4 = getPosition();
//  uint32_t linePos5 = getPosition();
//  
//  uint32_t avgPos = (linePos1 + linePos2 + linePos3 + linePos4 + linePos5)/5;
  int error =  getPosition() - center;


 // lastPos = avgPos;
//  if (error > 0)
//  {
//    error = error - correction;
//  }
//  else
//  {
//    error = error + correction;
//  }
  
  double adjustment = error * Ke;
  double DRnow = DR + adjustment;
  if (DRnow > 1){
    DRnow = 1;
  }
  if(DRnow < 0){
    DRnow = 0;
  }
  float p = error * Ke;
  
  uint8_t NewLS = constrain(LSpeed + p, 0, 100);
  uint8_t NewRS = constrain(RSpeed - p, 0, 100);
  
//  uint8_t Left = (DRnow)*Speed;
//  uint8_t Right = (1-DRnow)*Speed;

   

  setMotorSpeed(LEFT_MOTOR, (NewLS));
  setMotorSpeed(RIGHT_MOTOR, (NewRS));

  uint32_t LeftCount = getEncoderLeftCnt();
  uint32_t RightCount = getEncoderRightCnt();
  debugln("linePos = " + String(linePos) + " DR = " + String(DRnow) + " Error = " + String(error));
  debugln(" LSpeed = " + String(DRnow * Speed) + " RSpeed = " + String((1 - DRnow) * Speed));
  debugln(" Left E Count: " + String(getEncoderLeftCnt()) + " Right E Count: " + String(RightCount));
  

  // PLX Data Out
  volatile unsigned long timeNow = millis();
  PLXout("DATA, TIME,");
  PLXout(timeNow);
  PLXout(" ,");
  PLXout(avgPos);
  PLXout(" ,");
  PLXout(DRnow);
  PLXout(" ,");
  PLXout(error);
  PLXoutln(" ,");
  delay(500);
}
