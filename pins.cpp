#include "pins.h"

bool checkDigitalPin(int pin)
{
  const int pins[9] = {0, 1, 2, 9, 10, 14, 15, 16, 17};
  int i;
  for (i = 0; i < 9; i++)
  {
    if (pin == pins[i])
    {
      return true;
    }
  }
  return false;
}

bool checkAnalogPin(int pin)
{
  const int pins[4] = {14, 15, 16, 17};
  int i;
  for (i = 0; i < 4; i++)
  {
    if (pin == pins[i])
    {
      return true;
    }
  }
  return false;
}

bool checkPWMPin(int pin)
{
  const int pins[2] = {9, 10};
  int i;
  for (i = 0; i < 2; i++)
  {
    if (pin == pins[i])
    {
      return true;
    }
  }
  return false;
}

bool checkNotBusyPin(int pin, int sp, int in, int out)
{
  return ((pin != sp) && (pin != in) && (pin != out));
}
