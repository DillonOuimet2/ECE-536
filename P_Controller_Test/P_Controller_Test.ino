#include "SimpleRSLK.h"
#include "Encoder.h"

// Debug Control
#define DEBUG 0
#define PLX 1

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

uint16_t Speed = 30;

uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];

//
int numRuns;

// P Controller
double Ke = 0.5 / 3500; // Range of output/max error
double DR = 0.5;
double correction = 10; // Correction for Distance from Axle
volatile int center = 2500;

void setup()
{
  Serial.begin(9600);

  PLXoutln("CLEARDATA");
  PLXoutln("LABEL, Time, Time From Start, Sensor Output, Drive Ratio, Error, Center");
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

  uint32_t linePos = getPosition();
  uint32_t estPos = (lastPos + linePos + getPosition())/3;
  int error = estPos - center;

  lastPos = linePos;
  if (estPos > 0)
  {
    error = error - correction;
  }
  else
  {
    error = error + correction;
  }
  
  double adjustment = error * Ke;
  double DRnow = DR + adjustment;
  DRnow= constrain(DRnow, 0, 1);
  setMotorSpeed(LEFT_MOTOR, ((DRnow)*Speed));
  setMotorSpeed(RIGHT_MOTOR, ((1 - DRnow) * Speed));

//  debugln("linePos = " + String(linePos) + " DR = " + String(DRnow) + " Error = " + String(error));
//  debugln(" LSpeed = " + String(DRnow * Speed) + " RSpeed = " + String((1 - DRnow) * Speed));

  // PLX Data Out
  volatile unsigned long timeNow = millis();
  PLXout("DATA, TIME,");
  PLXout(timeNow);
  PLXout(" ,");
  PLXout(linePos);
  PLXout(" ,");
  PLXout(DRnow);
  PLXout(" ,");
  PLXout(error);
  PLXout(" ,");
  PLXout(center);
  PLXoutln(" ,");
  //delay(500);
  
  numRuns = numRuns + 1;
//   debugln("Number of Loop it. = " + String(numRuns))
}
