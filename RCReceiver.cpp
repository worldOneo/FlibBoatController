#include "Arduino.h"
#include "RCReceiver.h"

void pinStatusChange(RCReceiver *rc)
{
  noInterrupts();

  if (digitalRead(rc->getPin()) == HIGH)
    rc->impulseStart();
  else
    rc->impulseStop();

  interrupts();
}

void RCReceiver::attach(uint8_t pin)
{
  attachInterrupt(pin, pinStatusChange, CHANGE, this);
  this->pin = pin;
}

void RCReceiver::impulseStart()
{
  start = micros();
}

void RCReceiver::impulseStop()
{
  value = micros() - start;
}

uint8_t RCReceiver::getPin()
{
  return this->pin;
}

uint16_t RCReceiver::getValue()
{ // If value is 0 return 1500
  return this->value + (this->value == 0) * 1500;
}
