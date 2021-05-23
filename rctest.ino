#include <Servo.h>
#include "RCReceiver.h"
#include "SparkFunLSM6DS3.h"
#include <math.h>

const uint16_t NEUTRAL_STE = 1500;
const uint16_t NEUTRAL_THR = 1500;
const uint16_t TOLERANCE = 50;
const uint16_t AREA = 1000;
const uint16_t HAREA = AREA / 2;

const byte D9 = 9;
const byte D10 = 10;

//PIN_CTR = Controll pins for the Motor right and left
const byte PIN_CTR_R = D9;
const byte PIN_CTR_L = D10;

const byte D2 = 2;
const byte D3 = 3;

//PIN_RC = Receiving pins for the Remote
const byte PIN_RC_STE = D2;
const byte PIN_RC_THR = D3;

const int8_t UP = 1;
const int8_t DOWN = -UP;

int8_t rotation = UP;

RCReceiver steReceive;
RCReceiver thrReceive;
LSM6DS3 GyroscopeIMU;
Servo lMotor;
Servo rMotor;

long ms = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println("FlipBoatController");

  pinMode(PIN_RC_STE, INPUT);
  pinMode(PIN_RC_THR, INPUT);
  Serial.println("Attaching motors");

  lMotor.attach(PIN_CTR_L);
  rMotor.attach(PIN_CTR_R);
  Serial.println("Attaching receivers");

  steReceive.attach(PIN_RC_STE);
  thrReceive.attach(PIN_RC_THR);

  Serial.println("Attaching gyroscope");

  GyroscopeIMU.begin();

  Serial.println("initialzing motors");
  lMotor.writeMicroseconds(NEUTRAL_STE);
  rMotor.writeMicroseconds(NEUTRAL_STE);

  Serial.println("done");
}

void loop()
{
  int16_t ste = steReceive.getValue();
  int16_t thr = thrReceive.getValue();

  if (GyroscopeIMU.readFloatAccelZ() > 0)
    rotation = UP;
  else
    rotation = DOWN;

  if (millis() - ms < 19)
    return;

  Serial.println(millis() - ms);
  ms = millis();
  lMotor.writeMicroseconds(calcMotorLSpeed(ste, thr));
  rMotor.writeMicroseconds(calcMotorRSpeed(ste, thr));
}

int16_t calcMotorLSpeed(float ste, float thr)
{
  float motor = calcMotorSpeed(ste, thr, UP);
  return max(min(motor, 2000), 1500);
}

int16_t calcMotorRSpeed(float ste, float thr)
{
  float motor = calcMotorSpeed(ste, thr, DOWN);
  return min(max(motor, 1000), 1500);
}

float calcMotorSpeed(float ste, float thr, int8_t fac)
{
  if (fabs(thr - NEUTRAL_THR) < TOLERANCE)
    return 1500;

  float thrfactor = ((thr - NEUTRAL_THR) / HAREA);
  float stefactor = 1 - ((ste - NEUTRAL_STE) / HAREA) * (rotation * fac);

  return 1500 + thrfactor * stefactor * HAREA * fac;
}