#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

#define SRX D5
#define STX D6

Adafruit_PWMServoDriver servoDriver;
SoftwareSerial espSerial(SRX, STX);

char MOVEMENT_CONTROL = 0b00000001;
#define MOVEMENT_ENABLED 7
#define MOVEMENT_IDLE 6
#define MOVEMENT_SELECT 5
#define MOVEMENT_FORWARD 4
#define MOVEMENT_ROTATION 3

#define NUM_SERVOS 8
#define SERVO_MIN 200
#define SERVO_MAX 500
const int servoPins[8] = {0, 1, 2, 3, 4, 5, 6, 7};

int forwardSwitch = 0, backwardSwitch = 0, leftSwitch = 0, rightSwitch = 0;

//0: min - to its right (facing forward); max - to its left
//1: min - upward; max - downward
//2: same as 0
//3: same as 1
//4: min - to its right (facing backward); max - to its left
//5: min - downward; max - upward
//6: same as 4
//7: same as 5

const int idleValues[NUM_SERVOS] = {350, 350, 350, 350, 350, 350, 350, 350};
const int errorValues[NUM_SERVOS] = {20, -70, -25, -75, -10, -10, -70, -10};
unsigned long lastValueLoad = 0;
#define TEST_VALUES_WAIT 4000
const int testValues[8][NUM_SERVOS] = {{250, 250, 250, 250, 250, 250, 250, 250},
                                      {300, 300, 300, 300, 300, 300, 300, 300},
                                      {350, 350, 350, 350, 350, 350, 350, 350},
                                      {400, 400, 400, 400, 400, 400, 400, 400},
                                      {450, 450, 450, 450, 450, 450, 450, 450},
                                      {400, 400, 400, 400, 400, 400, 400, 400},
                                      {350, 350, 350, 350, 350, 350, 350, 350},
                                      {300, 300, 300, 300, 300, 300, 300, 300}};

#define MOVING_VALUES_WAIT 500
const int movingForwardValues[9][NUM_SERVOS] = {{350, 350, 350, 275, 350, 425, 350, 350},
                                          {350, 350, 275, 275, 425, 425, 350, 350},
                                          {350, 350, 275, 350, 425, 350, 350, 350},
                                          {350, 275, 350, 350, 350, 350, 350, 425},
                                          {275, 275, 425, 350, 275, 350, 425, 425},
                                          {275, 350, 425, 350, 275, 350, 425, 350},
                                          {350, 350, 350, 275, 350, 425, 350, 350},
                                          {425, 350, 275, 275, 425, 425, 275, 350},
                                          {425, 350, 275, 350, 425, 350, 275, 350}};
const int movingBackwardValues[9][NUM_SERVOS] = {{350, 350, 350, 275, 350, 425, 350, 350},
                                          {350, 350, 425, 275, 275, 425, 350, 350},
                                          {350, 350, 425, 350, 275, 350, 350, 350},
                                          {350, 275, 350, 350, 350, 350, 350, 425},
                                          {425, 275, 275, 350, 425, 350, 275, 425},
                                          {425, 350, 275, 350, 425, 350, 275, 350},
                                          {350, 350, 350, 275, 350, 425, 350, 350},
                                          {275, 350, 425, 275, 275, 425, 425, 350},
                                          {275, 350, 425, 350, 275, 350, 425, 350}};

#define ROTATING_VALUES_WAIT 500
const int rotatingLeftValues[6][NUM_SERVOS] = {{350, 350, 350, 275, 350, 425, 350, 350},
                                                {350, 350, 425, 275, 425, 425, 350, 350},
                                                {350, 350, 425, 350, 425, 350, 350, 350},
                                                {350, 275, 425, 350, 425, 350, 350, 425},
                                                {350, 275, 350, 350, 350, 350, 350, 425},
                                                {350, 350, 350, 350, 350, 350, 350, 350}};

const int rotatingRightValues[6][NUM_SERVOS] = {{350, 275, 350, 350, 350, 350, 350, 425},
                                                {275, 275, 350, 350, 350, 350, 275, 425},
                                                {275, 350, 350, 350, 350, 350, 275, 350},
                                                {275, 350, 350, 275, 350, 425, 275, 350},
                                                {350, 350, 350, 275, 350, 425, 350, 350},
                                                {350, 350, 350, 350, 350, 350, 350, 350}};

unsigned long lastDriverUpdate = 0;
#define DRIVER_UPDATE_DELAY 200

void updateServo(const int newValues[NUM_SERVOS])
{
  for(int i=0; i<NUM_SERVOS; i++)
  {
    servoDriver.setPWM(servoPins[i], 0, newValues[i] + errorValues[i]);
  }
}

void movementTest()
{
  static int step = 0;

  if(millis() - lastValueLoad > TEST_VALUES_WAIT)
  {
    lastValueLoad = millis();
    updateServo(testValues[step]);
    
    step++;
    if(step % 8 == 0)
      step = 3;
  }
}

void movementIdle()
{
  updateServo(idleValues);
}

void movementForward()
{
  static int step = 0;
  if(forwardSwitch == 2)
    step = 0;

  if(millis() - lastValueLoad > MOVING_VALUES_WAIT)
  {
    lastValueLoad = millis();
    updateServo(movingForwardValues[step]);
    
    step++;
    if(step % 9 == 0)
      step = 3;
  }
}

void movementBackward()
{
  static int step = 0;
  if(backwardSwitch == 2)
    step = 0;

  if(millis() - lastValueLoad > MOVING_VALUES_WAIT)
  {
    lastValueLoad = millis();
    updateServo(movingBackwardValues[step]);
    
    step++;
    if(step % 9 == 0)
      step = 3;
  }
}

void movementRotateLeft()
{
  static int step = 0;
  if(leftSwitch == 2)
    step = 0;

  if(millis() - lastValueLoad > ROTATING_VALUES_WAIT)
  {
    lastValueLoad = millis();
    updateServo(rotatingLeftValues[step]);
    
    step++;
    if(step % 6 == 0)
      step = 0;
  }
}

void movementRotateRight()
{
  static int step = 0;
  if(rightSwitch == 2)
    step = 0;

  if(millis() - lastValueLoad > ROTATING_VALUES_WAIT)
  {
    lastValueLoad = millis();
    updateServo(rotatingRightValues[step]);
    
    step++;
    if(step % 6 == 0)
      step = 0;
  }
}

void setup() {
  Serial.begin(115200);
  espSerial.begin(115200);

  Wire.begin();
  
  servoDriver.begin();
  servoDriver.setPWMFreq(50);

  while(!Serial);
  Serial.println("Setup complete!");

  movementIdle();
}

void loop() {
  
  if(millis() - lastDriverUpdate > DRIVER_UPDATE_DELAY)
  {
    lastDriverUpdate = millis();
    if(MOVEMENT_CONTROL & (1<<MOVEMENT_ENABLED))
    {
      if(MOVEMENT_CONTROL & (1<<MOVEMENT_IDLE))
      {
        movementIdle();
      }else
      {
        if(MOVEMENT_CONTROL & (1<<MOVEMENT_SELECT))
        {
          if(MOVEMENT_CONTROL & (1<<MOVEMENT_FORWARD))
          {
            movementForward();
            forwardSwitch = 1;
          }
          else{
            movementBackward();
            backwardSwitch = 1;
          }
        }else
        {
          if(MOVEMENT_CONTROL & (1<<MOVEMENT_ROTATION))
          {
            movementRotateRight();
            rightSwitch = 1;
          }
          else
          {
            movementRotateLeft();
            leftSwitch = 1;
          }
        }
      }
    }
  }

  if(espSerial.available())
  {
    if(forwardSwitch == 1)
      forwardSwitch = 2;
    if(backwardSwitch == 1)
      backwardSwitch = 2;
    if(rightSwitch == 1)
      rightSwitch = 2;
    if(leftSwitch == 1)
      leftSwitch = 2;
    char receivedValue = espSerial.read();
    MOVEMENT_CONTROL = receivedValue;
    Serial.println(int(receivedValue));
    for(int i=0; i<2; i++)
      receivedValue = espSerial.read();
    movementIdle();
    lastValueLoad = millis();
  }
}