#include <Arduino.h>
#include <Ticker.h>
#include <AccelStepper.h>
#include <Wire.h>
#include "ArduinoNunchuk/ArduinoNunchuk.h"

#define MOTOR_LEFT_STEP  2
#define MOTOR_LEFT_DIR   3
#define MOTOR_LEFT_EN    4
#define MOTOR_RIGHT_STEP 5
#define MOTOR_RIGHT_DIR  6
#define MOTOR_RIGHT_EN   7

const long MOTOR_FWD_DST = INT16_MAX;
const long MOTOR_REV_DST = INT16_MIN;

#define DEADZONE_X 12
#define DEADZONE_Y 12
#define CENTER_X 128
#define CENTER_Y 135
#define MIN_X -80
#define MIN_Y -74
#define MAX_X 80
#define MAX_Y 66

#define MAX_SPEED 900
#define MAX_TURN 300

void blinkLight();
void motorReverse();
void joyGet();

AccelStepper motorLeft(AccelStepper::DRIVER, MOTOR_LEFT_STEP, MOTOR_LEFT_DIR);
AccelStepper motorRight(AccelStepper::DRIVER, MOTOR_RIGHT_STEP, MOTOR_RIGHT_DIR);

Ticker blinky(blinkLight, 200);
// Ticker motorSwitch(motorReverse, 7000);
Ticker joystick(joyGet, 40);

ArduinoNunchuk nunchuk = ArduinoNunchuk();

// Joystick X range: 27 - 128 - 227
// Joystick Y range: 30 - 135 - 231

void setup() {

  Serial.begin(115200);
  // put your setup code here, to run once:

  motorLeft.setEnablePin(MOTOR_LEFT_EN);
  motorRight.setEnablePin(MOTOR_RIGHT_EN);
  motorLeft.setPinsInverted(false, false, true);
  motorRight.setPinsInverted(false, false, true);
  motorLeft.disableOutputs();
  motorRight.disableOutputs();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  nunchuk.init();

  blinky.start();
  // motorSwitch.start();
  joystick.start();

  motorLeft.setMaxSpeed(3000);
  motorLeft.setAcceleration(200);
  motorRight.setMaxSpeed(3000);
  motorRight.setAcceleration(200);
  // motorLeft.moveTo(500);

  // motorLeft.enableOutputs();
  // motorRight.enableOutputs();
}

void loop() {
  // put your main code here, to run repeatedly:
  blinky.update();
  // motorSwitch.update();
  motorLeft.runSpeed();
  motorRight.runSpeed();
  joystick.update();
}

void blinkLight() {
  uint8_t state = digitalRead(LED_BUILTIN);
  digitalWrite(LED_BUILTIN, !state);
}

void motorReverse() {
  Serial.println("Motor Reverse");
  motorLeft.moveTo(-motorLeft.currentPosition());
}

void joyGet() {
  int16_t speedLeft;
  int16_t speedRight;
  int16_t YPos;
  int16_t XPos;

  nunchuk.update();

  Serial.print(nunchuk.analogX, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.analogY, DEC);
  Serial.println();

  YPos = nunchuk.analogY - CENTER_Y;
  XPos = nunchuk.analogX - CENTER_X;

  if (YPos > 0 && abs(YPos) > DEADZONE_Y) {
    YPos = YPos - DEADZONE_Y;

    Serial.print("Speed PreMap: ");
    Serial.println(YPos);

    speedLeft = map(YPos, 0, MAX_Y, 0, MAX_SPEED);
    speedRight = map(YPos, 0, MAX_Y, 0, MAX_SPEED);

  } else if (YPos < 0 && abs(YPos) > DEADZONE_Y) {

    YPos = YPos + DEADZONE_Y;

    Serial.print("Speed PreMap: ");
    Serial.println(YPos);

    speedLeft = map(YPos, 0, MIN_Y, 0, -MAX_SPEED);
    speedRight = map(YPos, 0, MIN_Y, 0, -MAX_SPEED);

  } else {
    speedLeft = 0;
    speedRight = 0;
  }

  if (XPos > 0 && abs(XPos) > DEADZONE_X) {
    XPos = XPos - DEADZONE_X;

    Serial.print("XSpeed PreMap: ");
    Serial.println(XPos);

    XPos = map(XPos, 0, MAX_X, 0, MAX_TURN);

    speedLeft -= XPos;
    speedRight += XPos;

  } else if (XPos < 0 && abs(XPos) > DEADZONE_X) {
    XPos = XPos + DEADZONE_X;

    Serial.print("XSpeed PreMap: ");
    Serial.println(XPos);

    XPos = map(XPos, 0, MIN_X, 0, -MAX_TURN);

    speedLeft -= XPos;
    speedRight += XPos;

  }

  if (speedLeft!=0 || speedRight!=0) {
    motorLeft.enableOutputs();
    motorRight.enableOutputs();

    motorLeft.setSpeed(speedLeft);
    motorRight.setSpeed(-speedRight);
  } else {
    motorLeft.setCurrentPosition(0);
    motorLeft.disableOutputs();

    motorRight.setCurrentPosition(0);
    motorRight.disableOutputs();
  }

}