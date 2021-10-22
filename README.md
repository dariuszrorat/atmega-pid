# MMI PID regulator

This directory contains simple MMI PID regulator for ATMEGA328/Arduino.

### Features

  - continous mode regulation
  - solid state relay mode regulation
  - extended 16 bit PWM
  - extended dual PWM 16 bit resolutions

### MMI codes

      *00*#           repeat the last code
      *01*#           print Kp, Ki, Kd and PID mode params
      *10*XXXX#       set P param value divided by 100
      *11*XXXX#       set I param value divided by 100
      *12*XXXX#       set D param value divided by 100
      *13*X#          set proportional on measurement X=0 or error X=1
      *14*X#          set direction X=0 means direct, X=1 means reverse
      *15*X#          set PID mode 0 = PWM, 1 = PWM16, 2 = Dual PWM, 3 = SSR
      *16*X#          set relay high state X = 0 or 1
      *17*XXXX#       set relay mode window size ms
      *18*XXXX#       set PID sample time ms
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
      *41*#           enable PID regulator
      *42*#           disable PID regulator
      *51*#           save settings to EEPROM
      *52*#           load settings from EEPROM

### Other informations

This project using Arduino PID library
https://playground.arduino.cc/Code/PIDLibrary/

The dual PWM 16 use 2 PWM 8 bit outputs
http://www.openmusiclabs.com/learning/digital/pwm-dac/dual-pwm-circuits/index.html

Default values are P=1.0, I=0.0, D=0.0, standard 8 bit PWM output, sample time 100ms.