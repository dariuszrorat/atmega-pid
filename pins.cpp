#include "pins.h"

bool checkDigitalPin(int pin)
{
  const int pins[9] = {3, 5, 7, 8, 9, 14, 15, 16, 17};
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
  const int pins[3] = {3, 5, 9};
  int i;
  for (i = 0; i < 3; i++)
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
