#include <Servo.h>
#include "RCReceive.h"
#include "SparkFunLSM6DS3.h"

uint16_t neutralSTE = 1500;
uint16_t neutralTHR = 1500;
const uint16_t TOLERANCE = 50;
const uint16_t NPT = neutralSTE + TOLERANCE;
const uint16_t NMT = neutralSTE - TOLERANCE;
const uint16_t AREA = 1000;
const uint16_t HAREA = AREA / 2;

const byte D9 = 9;
const byte D10 = 10;

//PIN_CTR = Controll pins for the Motor right and left
const byte PIN_CTR_R = D9;
const byte PIN_CTR_L = D10;

const byte D7 = 7;
const byte D8 = 8;

//PIN_RC = Receiving pins for the Remote
const byte PIN_RC_STE = D7;
const byte PIN_RC_THR = D8;

int8_t rotation = 1;

RCReceive steReceive;
RCReceive thrReceive;
LSM6DS3 GyroscopeIMU;
Servo lMotor;
Servo rMotor;

void setup()
{
  Serial.begin(9600);

  pinMode(PIN_RC_STE, INPUT);
  pinMode(PIN_RC_THR, INPUT);

  lMotor.attach(PIN_CTR_L);
  rMotor.attach(PIN_CTR_R);

  steReceive.attachInt(PIN_RC_STE);
  thrReceive.attachInt(PIN_RC_THR);

  GyroscopeIMU.begin();

  lMotor.writeMicroseconds(neutralSTE);
  rMotor.writeMicroseconds(neutralSTE);
}

void loop()
{
  // Read current RC value
  steReceive.poll();
  thrReceive.poll();

  if (steReceive.hasNP() && !steReceive.hasError())
  {
    doWork();
  }
  else if (steReceive.hasError())
  {
    Serial.println("Failed to read RC");
  }
}

void doWork()
{
  int16_t ste = steReceive.getMsValue();
  int16_t thr = thrReceive.getMsValue();

  neutralSTE = steReceive.getMSNP();
  neutralTHR = thrReceive.getMSNP();

  if (GyroscopeIMU.readFloatAccelZ() > 0)
  {
    rotation = 1;
  }
  else
  {
    rotation = -1;
  }

  Serial.print((int)ste);
  Serial.print(" ");
  Serial.print((int)thr);
  Serial.print(" ");
  Serial.print(calcMotorLSpeed(ste, thr));
  Serial.print(" ");
  Serial.print(calcMotorRSpeed(ste, thr));
  Serial.println();
  lMotor.writeMicroseconds(calcMotorLSpeed(ste, thr));
  rMotor.writeMicroseconds(calcMotorRSpeed(ste, thr));
}

int16_t calcMotorLSpeed(float ste, float thr)
{
  float motor = calcMotorSpeed(ste, thr, 1);
  if (motor < 1500)
  {
    return 1500;
  }
  return min(motor, 2000);
}

int16_t calcMotorRSpeed(float ste, float thr)
{
  float motor = calcMotorSpeed(ste, thr, -1);
  if (motor > 1500)
  {
    return 1500;
  }
  return max(motor, 1000);
}

float calcMotorSpeed(float ste, float thr, int8_t fac)
{
  if (ste < NPT && ste > NMT && thr < NPT && thr > NMT)
    return 1500;

  float thrfactor;
  float stefactor;

  thrfactor = ((thr - neutralTHR) / HAREA);
  stefactor = 1 - ((ste - neutralSTE) / HAREA) * (rotation * fac);
  return 1500 + thrfactor * stefactor * HAREA * fac;
}