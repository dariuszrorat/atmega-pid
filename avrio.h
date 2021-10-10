#ifndef AVRIO_H
#define AVRIO_H

#include <arduino.h>

int digitalReadOutputPin(uint8_t pin);
void setPWMDivisor(byte timer, byte value);

#endif
