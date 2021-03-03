#include <Servo.h>
#include "RCReceiver.h"
#include "SparkFunLSM6DS3.h"

uint16_t neutralSTE = 1500;
uint16_t neutralTHR = 1500;
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
  lMotor.writeMicroseconds(neutralSTE);
  rMotor.writeMicroseconds(neutralSTE);

  Serial.println("done");

}

void loop()
{
  doWork();
}

void doWork()
{
  int16_t ste = steReceive.getValue();
  int16_t thr = thrReceive.getValue();

  if (GyroscopeIMU.readFloatAccelZ() > 0)
  {
    rotation = UP;
  }
  else
  {
    rotation = DOWN;
  }
  if (millis() - ms > 19)
  {
    Serial.println(millis() - ms);
    ms = millis();
    lMotor.writeMicroseconds(calcMotorLSpeed(ste, thr));
    rMotor.writeMicroseconds(calcMotorRSpeed(ste, thr));
  }
}

int16_t calcMotorLSpeed(float ste, float thr)
{
  float motor = calcMotorSpeed(ste, thr, UP);
  if (motor < 1500)
  {
    return 1500;
  }
  return min(motor, 2000);
}

int16_t calcMotorRSpeed(float ste, float thr)
{
  float motor = calcMotorSpeed(ste, thr, DOWN);
  if (motor > 1500)
  {
    return 1500;
  }
  return max(motor, 1000);
}

float calcMotorSpeed(float ste, float thr, int8_t fac)
{
  if (thr < neutralTHR + TOLERANCE && thr > neutralTHR - TOLERANCE)
    return 1500;

  float thrfactor;
  float stefactor;

  thrfactor = ((thr - neutralTHR) / HAREA);
  stefactor = 1 - ((ste - neutralSTE) / HAREA) * (rotation * fac);
  return 1500 + thrfactor * stefactor * HAREA * fac;
}