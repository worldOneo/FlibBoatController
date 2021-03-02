//include <makros.h>
//include <debug.h>
#include "RCReceive.h"
#include "SparkFunLSM6DS3.h"
/*
  RC_Template.ino - Template for RC Receiver enabled programs - Version 0.2
  Copyright (c) 2012 Wilfried Klaas.  All right reserved.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

uint16_t neutralSTE = 1500;
uint16_t neutralTHR = 1500;
const uint16_t TOLERANCE = 50;
const uint16_t NPT = neutralSTE + TOLERANCE;
const uint16_t NMT = neutralSTE - TOLERANCE;
const uint16_t AREA = 1000;
const uint16_t HAREA = AREA / 2;

const byte D9 = 9;
const byte D10 = 10;

const byte PIN_RC_STE = D9;
const byte PIN_RC_THR = D10;

int8_t rotation = 1;

RCReceive steReceive;
RCReceive thrReceive;
LSM6DS3 GyroscopeIMU;

void setup()
{
  Serial.begin(9600);
  pinMode(PIN_RC_STE, INPUT);
  pinMode(PIN_RC_THR, INPUT);
  steReceive.attachInt(PIN_RC_STE);
  thrReceive.attachInt(PIN_RC_THR);

  GyroscopeIMU.begin();

  // put your setup code here, to run once:
}

void loop()
{
  // Read current RC value
  steReceive.poll();
  thrReceive.poll();

  // zero point determination?
  if (steReceive.hasNP() && !steReceive.hasError())
  {
    doWork();
  }
  else if (steReceive.hasError())
  {
    Serial.println("Failed to read RC");
    // Failure handling failsafe or something ...
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

  calcMotorRSpeed(ste, thr);
  Serial.print((int)ste);
  Serial.print(" ");
  Serial.print((int)thr);
  Serial.print(" ");
  Serial.print(calcMotorLSpeed(ste, thr));
  Serial.print(" ");
  Serial.print(calcMotorRSpeed(ste, thr));
  Serial.println();
  /*Serial.print(" ");
  Serial.println(calcMotorRSpeed(ste, thr));*/
  // put your main code here, to run repeatedly:
}

int16_t calcMotorLSpeed(float ste, float thr)
{
  /* if (ste < NPT && ste > NMT && thr < NPT && thr > NMT)
    return 1500;

  float thrfactor;
  float stefactor;

  thrfactor = ((thr - neutralTHR) / HAREA);
  stefactor = 1 - ((ste - neutralSTE) / HAREA) * rotation;

  float motor = 1500 + thrfactor * stefactor * HAREA; */
  float motor = calcMotorSpeed(ste, thr, 1);
  if (motor < 1500)
  {
    return 1500;
  }
  return min(motor, 2000);
}

int16_t calcMotorRSpeed(float ste, float thr)
{
  /*if (ste < NPT && ste > NMT && thr < NPT && thr > NMT)
    return 1500;

  float thrfactor;
  float stefactor;

  thrfactor = ((thr - neutralTHR) / HAREA);
  stefactor = 1 - ((ste - neutralSTE) / HAREA) * -rotation;

  float motor = 1500 - thrfactor * stefactor * HAREA;*/
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