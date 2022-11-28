#include "SimpleRSLK.h"
#include "Encoder.h"
#include "Romi_Motor_Power.h"

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

// Function Prototypes
void setupEncoder(uint8_t ela_pin, uint8_t elb_pin, uint8_t era_pin, uint8_t erb_pin); // initalize the encoders
void setRawMotorSpeed(uint8_t motorNum, uint8_t speed);

// Pin Configurations
const int trigPin = 32; // This is Port Pin 3.5 on the MSP432 Launchpad
const int echoPin = 33; // This is Port Pin 5.1 on the MSP432 Launchpad

// Calibration Global Variables
uint8_t lineColor = DARK_LINE; // DARK_LINE or LIGHT_LINE
bool isCalibrationComplete = false;

// Motor Speed Global Variables

uint16_t Speed = 0.3*255;
double DR = 0.5;
double DRnow;

// Sensor Global Variables
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];

// Num Runs
int numRuns = 0;
bool stepEn = false;

/* PID Controller Variables */
uint32_t linePos;
uint32_t lastLine; 
int error;
float adjustment;

// P Controller
float Ke = 0.0011664; // OG Ke = 1.5*(0.5 / 3500); 
volatile int center = 3500;

// I and D controllers
float Ki = 0.00092462;
float totError = 0;

float Kd = Ki*0.1;
float dError;
float errorLast;

volatile unsigned long timeLast = millis();
volatile unsigned long timeNow= millis();

float ErrorEnd;
float i;
float iold;
float d;

float maxOS;
float minOS;
volatile unsigned long stepTime;
volatile unsigned long timeSettle;
bool step;
bool settled;

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

  floorCalibration();
}

void CheckOS(void) {
  if (linePos > maxOS) {
    maxOS = linePos;
  }
  if (linePos < minOS) {
    minOS = linePos;
  }
}

void CheckSettle(void) {
  if ((linePos >= center*0.9) && (linePos <= center*1.1) && !settled) {
    settled = true;
    timeSettle = (millis()-stepTime)/1000;
  }
  else if ((linePos >= center*0.9) && (linePos <= center*1.1) && settled) {
   }
  else {
    settled = false;
  }
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
  String btnMsg = "";
  /* Place Robot On Floor (no line) */
  delay(2000);
 
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN, btnMsg, RED_LED); //Turn off for system ID

  delay(1000);

  debugln("Running calibration on floor");
  simpleCalibrate();
  debugln("Reading floor values complete");
 
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

void CalculatePID(void) 
{
  // Calculate time from last calculation
  unsigned long timeNow = millis();
  unsigned long dTime = timeNow - timeLast;

  // Compute Working Error Variables
  linePos = getPosition();

  //Check to see if the bot has lost the line.
  if (linePos == 0){
    Pivot();
  }
  error = linePos - center;
//  totError += error;
  // i = iold + (Ki*(dTime * error));
  i = iold + dTime*error;
  dError = error - errorLast;
  d = (dError)/dTime;

  // Compute PID Adjustment
  adjustment = (error * (Ke)) + (constrain(i*Ki, 0, 1)) + (d*Kd);
  iold = i;
  // Save time and error
  errorLast = error;
  timeLast = timeNow;

  lastLine = linePos;
   
}

void SetMotorSpeedPID(void) 
{
  DRnow = DR + adjustment;
  DRnow = constrain(DRnow, 0, 1);
  setRawMotorSpeed(LEFT_MOTOR, ((DRnow)*Speed));
  setRawMotorSpeed(RIGHT_MOTOR, ((1 - DRnow) * Speed));
}

void PLXSerialOuput(void)
{
  debugln("linePos = " + String(linePos) + " DR = " + String(DRnow) + " Error = " + String(error));
  debugln(" LSpeed = " + String(DRnow * Speed) + " RSpeed = " + String((1 - DRnow) * Speed));

  // PLX Data Out
  volatile unsigned long timeNow = millis();
  PLXout("DATA, TIME,");
  PLXout(timeNow);
  PLXout(" ,");
  PLXout(linePos);
  PLXout(" ,");
  PLXoutln(DRnow);
  PLXout(" ,");
  PLXout(error);
  PLXout(" ,");
  PLXout(center);
  PLXoutln(" ,");

  //   debugln("Number of Loop it. = " + String(numRuns))
}

// Function to pivot 90 degrees.
void Pivot(){

  disableMotor(BOTH_MOTORS);
  delay(10);
  //if the position was last >3500 turn cw otherwise turn ccw

  if (lastLine > 3500) {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD); //In order to turn CW, the right motor needs to turn backwards and the left forward.
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    enableMotor(BOTH_MOTORS);
    setRawMotorSpeed(LEFT_MOTOR, 0.5 * Speed);
    setRawMotorSpeed(RIGHT_MOTOR, 0.5 * Speed);
  }
  else {
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD); //In order to turn CCW, the left motor needs to turn backwards and the righ forward.
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    enableMotor(BOTH_MOTORS);
    setRawMotorSpeed(LEFT_MOTOR, 0.5 * Speed);
    setRawMotorSpeed(RIGHT_MOTOR, 0.5 * Speed);
  }

  //Now turn until the robot has reached the center. 
  while ( (linePos > (center + 100)) || (linePos < (center - 100)) ){
    linePos = getPosition();
  }
  disableMotor(BOTH_MOTORS);
  delay (10);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  enableMotor(BOTH_MOTORS);
  setRawMotorSpeed(LEFT_MOTOR, ((DRnow)*Speed));
  setRawMotorSpeed(RIGHT_MOTOR, ((1 - DRnow) * Speed)); 
}

void loop()
{
  uint32_t lastPos;

  if (numRuns == 500 && stepEn) {
    center = 5500;
    step = true;
    stepTime = millis();
  }

  if (numRuns == 1000 && stepEn) {
    Serial.print("Max Overshoot: ");
    Serial.println(maxOS);
    Serial.print("Min Overshoot: ");
    Serial.println(minOS);
    Serial.print("Settling Time: ");
    Serial.println(timeSettle);
    Serial.print("Last Position: ");
    Serial.println(linePos);
    Serial.print("Center: ");
    Serial.println(center);
  }

  CalculatePID();

  SetMotorSpeedPID();

  PLXSerialOuput();

  if (step) {
    CheckOS();
    CheckSettle();
  }
  
  numRuns = numRuns + 1;

//  Serial.println(numRuns);
//  delay(500);
 
/* Timing Function
//int itTime = 9;
//if ( (millis() - timeNow) < itTime) {
//  delay(itTime-(millis() - timeNow));
//}
// PLXout(millis() - timeNow);
//  PLXoutln(" s");
*/

//Test
}
