#include "avrio.h"

int digitalReadOutputPin(uint8_t pin)
{
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    if (port == NOT_A_PIN)
        return LOW;

    return (*portOutputRegister(port) & bit) ? HIGH : LOW;
}

void setPWMDivisor(byte timer, byte value)
{
    byte mask = B11111000;
    switch (timer)
    {
        case 0:
        {
            TCCR0B &= mask;
            TCCR0B |= value;
        }
        break;
        case 1:
        {
            TCCR1B &= mask;
            TCCR1B |= value;
        }
        break;
        case 2:
        {
            TCCR2B &= mask;
            TCCR2B |= value;
        }
        break;
    }
}
