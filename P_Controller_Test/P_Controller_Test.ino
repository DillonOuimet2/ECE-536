#include "SimpleRSLK.h"
#include "Encoder.h"
#include "Romi_Motor_Power.h"

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
uint16_t SM = 1;
uint16_t Speed = SM*0.3*255;
double DR = 0.5;
double DRnow;

// Sensor Global Variables
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];

// Num Runs
int numRuns = 0;

/* PID Controller Variables */
uint32_t linePos;
uint32_t lastLine; 
int error;
double adjustment;

// P Controller
double Ke = 0.00091083; // OG Ke = 1.5*(0.5 / 3500); 
volatile int center = 3500;

// I and D controllers
double Ki = 1.6641;
double totError = 0;

double Kd = 0;
double dError;
double errorLast;

volatile unsigned long timeLast = millis();
volatile unsigned long timeNow= millis();

volatile unsigned long ErrorEnd;
volatile unsigned long derror;
volatile unsigned long i;
volatile unsigned long d;

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
  totError += error;
  i = dTime*totError;
  dError = error - errorLast;
  d = (dError)/dTime;

  // Compute PID Adjustment
  adjustment = (error * (Ke/SM)) + (constrain(i * Ki,0, 1)) + (d*Kd);

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
  PLXout(DRnow);
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
  while ( (linePos > (center + 50)) || (linePos < (center - 50)) ){
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

  if (numRuns == 100) {
    center = 3500;
  }

  CalculatePID();

  SetMotorSpeedPID();

  PLXSerialOuput();
  
  numRuns = numRuns + 1;
 
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
