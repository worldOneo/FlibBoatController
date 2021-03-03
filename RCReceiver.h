#ifndef RCReceiver_H
#define RCReceiver_H
class RCReceiver
{
public:
  void attach(uint8_t pin);
  uint16_t getValue();
  void impulseStart();
  void impulseStop();

  uint8_t getPin();

protected:
  uint16_t start;
  uint16_t value;
  uint8_t pin;
};
#endif