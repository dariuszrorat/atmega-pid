# MMI PID regulator

This directory contains simple MMI PID regulator for ATMEGA328/Arduino.

### Features

  - continous mode regulation
  - solid state relay mode regulation
  - extended dual PWM 16 bit resolutions
  - Ziegler-Nichols relay method tuning

### MMI codes

      *00*#           repeat the last code
      *01*#           print Kp, Ki, Kd and PID mode params
      *10*XXXX#       set P param value divided by 100
      *11*XXXX#       set I param value divided by 100
      *12*XXXX#       set D param value divided by 100
      *13*X#          set proportional on measurement X=0 or error X=1
      *14*X#          set direction X=0 means direct, X=1 means reverse
      *15*X#          set PID mode 0 = PWM, 1 = SSR, 2 = 16 bit dual PWM
      *16*X#          set relay high state X = 0 or 1
      *17*XXXX#       set relay mode window size ms
      *18*XXXX#       set PID sample time ms or tuner interval
      *20*XX#         set input analog pin
      *21*XX#         set setpoint analog pin
      *22*XX#         set output digital relay or PWM pin
      *23*X#          set use analog input as setpoint X = 0 or 1
      *24*XX*Y#       set digital pin as 0 or 1
      *25*XXXX#       set desired analog input 0..1023
      *26*XX*YY#      set 16 bit mode dual PWM output pair, H = XX and L = YY
      *31*XX*YY#      set the clip alarm LED pin XX to XX SP percent
      *32*XX*YY#      set too low input value LED pin XX to XX SP percent
      *33*XX#         set SSR mode relay enable information LED pin
      *34*XX#         set tuner target input value 0..1023, default 511
      *35*XX#         set tuner output range, 0..255 or 0..65535 for dual PWM, default 255
      *36*X#          set tuning mode, 0 = basic, 1 = less overshoot, 2 = no overshoot, default 0
      *37*XX#         set tuning cycles, default 10
      *41*#           enable PID regulator
      *42*#           disable PID regulator
      *43*#           start auto tuner, press any key to abort
      *51*#           save settings to EEPROM
      *52*#           load settings from EEPROM

### Other informations

This project using Arduino PID library
https://playground.arduino.cc/Code/PIDLibrary/

The Ziegler-Nichols relay tuning library
https://github.com/jackw01/arduino-pid-autotuner

Auto tuner works only in single analog PWM mode, not in SSR or Dual PWM modes.

The dual PWM 16 use 2 PWM 8 bit outputs
http://www.openmusiclabs.com/learning/digital/pwm-dac/dual-pwm-circuits/index.html

Default values are P=1.0, I=0.0, D=0.0, standard 8 bit PWM output, sample time 100ms.