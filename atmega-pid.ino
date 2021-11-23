
/*
    ATMEGA328 PID MMI code regulator

    MMI codes:

      00*#           repeat the last code
      01*#           print Kp, Ki, Kd and PID mode params
      10*XXXX#       set P param value divided by 100
      11*XXXX#       set I param value divided by 100
      12*XXXX#       set D param value divided by 100
      13*X#          set proportional on measurement X=0 or error X=1
      14*X#          set direction X=0 means direct, X=1 means reverse
      15*X#          set PID mode 0 = PWM, 1 = PWM16, 2 = Dual PWM, 3 = SSR
      16*X#          set relay high state X = 0 or 1
      17*XXXX#       set relay mode window size ms
      18*XXXX#       set PID sample time ms
      20*XX#         set input analog pin
      21*XX#         set setpoint analog pin
      22*XX#         set output digital relay or PWM pin
      23*X#          set use analog input as setpoint X = 0 or 1
      24*XX*Y#       set digital pin as 0 or 1
      25*XXXX#       set desired analog input 0..1023
      26*XX*YY#      set 16 bit mode PWM output MSB (XX) and LSB (YY) pair
      31*XX*YY#      set the clip alarm LED pin XX to XX SP percent
      32*XX*YY#      set too low input value LED pin XX to XX SP percent
      33*XX#         set SSR mode relay enable information LED pin
      41*#           enable PID regulator
      42*#           disable PID regulator
      51*#           save settings to EEPROM
      52*#           load settings from EEPROM

*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <PID_v1.h>
#include <EEPROM.h>

#include "strings.h"
#include "limits.h"
#include "pins.h"
#include "pwmio.h"
#include "mmicode.h"
#include "scheduler.h"

#define BUZZ_PIN 6
#define DISPLAY_TIME 3 // 3x2s
#define IDLE_TIME 75 // 1,25*60s
#define DEFAULT_INPUT_PIN 14
#define DEFAULT_SETPOINT_PIN 15
#define DEFAULT_OUTPUT_PIN_PWM16_H 9
#define DEFAULT_OUTPUT_PIN_PWM16_L 10
#define DEFAULT_OUTPUT_PIN 9
#define DEFAULT_WINDOW_SIZE 5000
#define DEFAULT_SAMPLE_TIME 100
#define DISABLED_LED_PIN 255
#define DEFAULT_P 1.0
#define DEFAULT_I 0.0
#define DEFAULT_D 0.0

LiquidCrystal_I2C lcd(0x3F, 16, 2);

const byte ROWS = 4;
const byte COLS = 4;

char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte rowPins[ROWS] = {13, 12, 11, 8};
byte colPins[COLS] = {7, 5, 4, 3};

Keypad keypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

byte upArrow[] = {
  B00100,
  B01110,
  B10101,
  B00100,
  B00100,
  B00100,
  B00100,
  B00100
};

byte downArrow[] = {
  B00100,
  B00100,
  B00100,
  B00100,
  B00100,
  B10101,
  B01110,
  B00100
};

byte switchOff[] = {
  B01110,
  B00100,
  B11111,
  B00000,
  B00000,
  B11111,
  B11111,
  B01010
};

byte switchOn[] = {
  B00000,
  B00000,
  B01110,
  B00100,
  B11111,
  B11111,
  B11111,
  B01010
};

byte lock = 0;
byte blnk = 0;
byte idle = 0;
byte entering = 0;
byte displayInfo = 0;
byte invalidMMI = 0;
byte invalidPin = 0;
byte dispCounter = 0;
byte idleCounter = 0;
byte swOn = 0;
byte pidEnabled = 0;
unsigned long windowStartTime;
int setpointValue = 0;

double Setpoint = 0, Input = 0, Output = 0;
double Kp = DEFAULT_P, Ki = DEFAULT_I, Kd = DEFAULT_D;
double LastInput = 0;

struct Settings
{
  int pinInput;
  int pinOutput;
  int pinSetpoint;
  byte pinOutputPWM16H;
  byte pinOutputPWM16L;
  byte inputAsSetpoint;
  byte pidMode;
  int windowSize;
  int sampleTime;
  byte relayHigh;
  double Kp;
  double Ki;
  double Kd;
  byte pOnE;
  byte controllerDirection;
  int pinClip;
  int percentClip;
  int pinTooLow;
  int percentTooLow;
  int pinSSRActive;
};

Settings settings;

PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

String shortcode = "";
String lastShortcode = "";
String displayTitle = "";
String displayStr = "";

void setup()
{
  loadDefaults();

  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(settings.pinInput, INPUT);
  pinMode(settings.pinSetpoint, INPUT);
  pinMode(settings.pinOutput, OUTPUT);
  pinMode(settings.pinOutputPWM16H, OUTPUT);
  pinMode(settings.pinOutputPWM16L, OUTPUT);
  digitalWrite(settings.pinOutput, LOW);
  digitalWrite(settings.pinOutputPWM16H, LOW);
  digitalWrite(settings.pinOutputPWM16L, LOW);

  Input = analogRead(settings.pinInput);
  Setpoint = 0;
  pid.SetTunings(settings.Kp, settings.Ki, settings.Kd, settings.pOnE);
  pid.SetControllerDirection(settings.controllerDirection);
  pid.SetSampleTime(settings.sampleTime);
  pid.SetMode(AUTOMATIC);

  windowStartTime = millis();

  lcd.init();
  lcd.createChar(0, upArrow);
  lcd.createChar(1, downArrow);
  lcd.createChar(2, switchOff);
  lcd.createChar(3, switchOn);
  lcd.backlight();

  Sch.init();
  Sch.add(ledUpdate, 500);
  Sch.add(keyUpdate, 1);
  Sch.add(pidUpdate, 1);

  Mmi.init();
  Mmi.add("*00*", repeatLastCode);
  Mmi.add("*01*", printParams);
  Mmi.add("*10*", setKpValue);
  Mmi.add("*11*", setKiValue);
  Mmi.add("*12*", setKdValue);
  Mmi.add("*13*", setPOnE);
  Mmi.add("*14*", setDirection);
  Mmi.add("*15*", setPIDMode);
  Mmi.add("*16*", setRelayHighState);
  Mmi.add("*17*", setRelayWindow);
  Mmi.add("*18*", setSampleTime);
  Mmi.add("*20*", setInputPin);
  Mmi.add("*21*", setSetpointPin);
  Mmi.add("*22*", setOutputPin);
  Mmi.add("*23*", setUseInputAsSetpoint);
  Mmi.add("*24*", setDigitalPinState);
  Mmi.add("*25*", setDesiredInput);
  Mmi.add("*26*", setDualPWMPair);
  Mmi.add("*31*", setClipAlarmPin);
  Mmi.add("*32*", setTooLowAlarmPin);
  Mmi.add("*33*", setSSRInfoPin);
  Mmi.add("*41*", enableRegulator);
  Mmi.add("*42*", disableRegulator);
  Mmi.add("*51*", saveSettings);
  Mmi.add("*52*", loadSettings);

  printFilledStr("WELCOME TO MMI", 0);
  printFilledStr("PID CONTROLLER", 1);
  delay(2000);
}

/* Main Program */
void loop()
{
  Sch.dispatch();
}

void keyUpdate()
{

  char key = keypad.getKey();
  if (key)
  {
    if (idle == 1)
    {
      lcd.setBacklight(1);
      tone(BUZZ_PIN, 1000, 50);
      idleCounter = 0;
      idle = 0;
      return;
    }

    entering = 1;
    invalidPin = 0;
    displayInfo = 0;
    idleCounter = 0;

    switch (key)
    {
      case 'A':
        {
          shortcode = "";
          entering = 0;
        }
        break;
      case 'B':
        {
          shortcode = shortcode.substring(0, shortcode.length() - 1);
        }
        break;
      case 'C': shortcode = ""; break;
      case 'D':
        {
          scanMMI();
          lastShortcode = shortcode;
          shortcode = "";
        }
        break;
      default:
        {
          if (shortcode.length() < 16)
          {
            shortcode += key;
          }
        }
        break;
    }

    tone(BUZZ_PIN, 1000, 50);
  }
}

void ledUpdate()
{
  idleCounter += 1;
  if (idleCounter == (2 * IDLE_TIME))
  {
    lcd.setBacklight(0);
    idleCounter = 0;
    idle = 1;
  }

  if (invalidMMI == 1)
  {
    printError("ERROR", "INVALID CODE", &invalidMMI);
    return;
  }

  if (invalidPin == 1)
  {
    printError("ERROR", "INVALID PIN", &invalidPin);
    return;
  }

  if (entering == 1)
  {
    printEntering();
    return;
  }

  if (displayInfo == 1)
  {
    printInfo();
    return;
  }

  printValues();
}

void pidUpdate()
{
  if (settings.inputAsSetpoint == 1)
  {
    setpointValue = analogRead(settings.pinSetpoint);
    Setpoint = (double) setpointValue;
  }

  LastInput = Input;
  Input = analogRead(settings.pinInput);

  if (pidEnabled == 1)
  {
    pid.Compute();
    doOutput();
    doWarningLEDS();
  }
}

void scanMMI()
{
  invalidMMI = 1;
  entering = 1;
  bool execResult = Mmi.exec(shortcode);
  invalidMMI = execResult ? 0 : 1;
  entering = execResult ? 0 : 1;
}

void repeatLastCode(long val0, long val1, long val2)
{
  if (lastShortcode != "")
  {
    shortcode = lastShortcode;
    scanMMI();
  }
}

void printParams(long val0, long val1, long val2)
{
  if (pidEnabled == 0)
  {
    const char* modes[4] = {"PWM", "PWM 16", "DUAL PWM", "SSR"};
    String s = String(modes[settings.pidMode]);
    printFilledStr("SETTINGS", 0);
    printFilledStr("KP=" + String(settings.Kp), 1);
    delay(2000);
    printFilledStr("KI=" + String(settings.Ki), 1);
    delay(2000);
    printFilledStr("KD=" + String(settings.Kd), 1);
    delay(2000);
    printFilledStr("MODE: " + s, 1);
    delay(2000);
  }
  else
  {
    displayTitle = "ERROR";
    displayStr = "PID IS ACTIVE";
    displayInfo = 1;
  }
}

void setKpValue(long val0, long val1, long val2)
{
  long value = limitValue(val0, 0, 2147483647);
  settings.Kp = (double) value / 100.0;
  pid.SetTunings(settings.Kp, settings.Ki, settings.Kd, settings.pOnE);
}

void setKiValue(long val0, long val1, long val2)
{
  long value = limitValue(val0, 0, 2147483647);
  settings.Ki = (double) value / 100.0;
  pid.SetTunings(settings.Kp, settings.Ki, settings.Kd, settings.pOnE);
}

void setKdValue(long val0, long val1, long val2)
{
  long value = limitValue(val0, 0, 2147483647);
  settings.Kd = (double) value / 100.0;
  pid.SetTunings(settings.Kp, settings.Ki, settings.Kd, settings.pOnE);
}

void setPOnE(long val0, long val1, long val2)
{
  int value = limitValue(val0, 0, 1);
  settings.pOnE = value;
  pid.SetTunings(settings.Kp, settings.Ki, settings.Kd, settings.pOnE);
}

void setDirection(long val0, long val1, long val2)
{
  int value = limitValue(val0, 0, 1);
  settings.controllerDirection = value;
  pid.SetControllerDirection(settings.controllerDirection);
}

void setPIDMode(long val0, long val1, long val2)
{
  int value = limitValue(val0, 0, 3);
  settings.pidMode = value;
  switch (settings.pidMode)
  {
    case 0: pid.SetOutputLimits(0, 255); break;
    case 1: {
        pid.SetOutputLimits(0, 65535);
        setupPWM16();
      } break;
    case 2: pid.SetOutputLimits(0, 65535); break;
    case 3: pid.SetOutputLimits(0, settings.windowSize); break;
  }

  windowStartTime = millis();
}

void setRelayHighState(long val0, long val1, long val2)
{
  int value = limitValue(val0, 0, 1);
  settings.relayHigh = value;
}

void setRelayWindow(long val0, long val1, long val2)
{
  int value = limitValue(val0, 0, 32767);
  settings.windowSize = value;
  switch (settings.pidMode)
  {
    case 0: pid.SetOutputLimits(0, 255); break;
    case 1: {
        pid.SetOutputLimits(0, 65535);
        setupPWM16();
      } break;
    case 2: pid.SetOutputLimits(0, 65535); break;
    case 3: pid.SetOutputLimits(0, settings.windowSize); break;
  }
}

void setSampleTime(long val0, long val1, long val2)
{
  int value = limitValue(val0, 50, 32767);
  settings.sampleTime = value;
  pid.SetSampleTime(settings.sampleTime);
}

void setInputPin(long val0, long val1, long val2)
{
  int pin = val0;
  invalidPin = checkAnalogPin(pin) ? 0 : 1;
  if (invalidPin == 0)
  {
    settings.pinInput = pin;
    pinMode(pin, INPUT);
  }
}

void setSetpointPin(long val0, long val1, long val2)
{
  int pin = val0;
  invalidPin = checkAnalogPin(pin) ? 0 : 1;
  if (invalidPin == 0)
  {
    settings.pinSetpoint = pin;
    pinMode(pin, INPUT);
  }
}

void setOutputPin(long val0, long val1, long val2)
{
  int pin = val0;
  switch (settings.pidMode)
  {
    case 0: {
        invalidPin = checkPWMPin(pin) ? 0 : 1;
      } break;
    case 1: {
        invalidPin = checkPWMPin(pin) ? 0 : 1;
        setupPWM16();
      } break;
    case 2: {
        invalidPin = checkPWMPin(pin) ? 0 : 1;
      } break;
    case 3: {
        invalidPin = checkDigitalPin(pin) ? 0 : 1;
      } break;
  }

  if (invalidPin == 0)
  {
    settings.pinOutput = pin;
    pinMode(pin, OUTPUT);
  }
}

void setUseInputAsSetpoint(long val0, long val1, long val2)
{
  int value = limitValue(val0, 0, 1);
  settings.inputAsSetpoint = value;
}

void setDigitalPinState(long val0, long val1, long val2)
{
  int pin = val0;
  invalidPin = checkDigitalPin(pin) ? 0 : 1;
  if (invalidPin == 0)
  {
    int value = limitValue(val1, 0, 1);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, value);
  }
}

void setDesiredInput(long val0, long val1, long val2)
{
  int value = limitValue(val0, 0, 1023);
  Setpoint = (double) value;
}

void setDualPWMPair(long val0, long val1, long val2)
{
  int pinH = val0;
  int pinL = val1;
  invalidPin = (checkPWMPin(pinH) && checkPWMPin(pinL)) ? 0 : 1;
  if (invalidPin == 0)
  {
    settings.pinOutputPWM16H = pinH;
    settings.pinOutputPWM16L = pinL;
    pinMode(pinH, OUTPUT);
    pinMode(pinL, OUTPUT);
  }
}

void setClipAlarmPin(long val0, long val1, long val2)
{
  int pin = val0;
  int percent = val1;
  bool validPin = ((checkDigitalPin(pin)
                    && checkNotBusyPin(pin, settings.pinSetpoint, settings.pinInput, settings.pinOutput)) || (pin == DISABLED_LED_PIN));
  invalidPin = validPin ? 0 : 1;

  if (invalidPin == 0)
  {
    if (settings.pinClip != DISABLED_LED_PIN)
    {
      pinMode(settings.pinClip, OUTPUT);
      digitalWrite(settings.pinClip, LOW);
    }
    if (pin != DISABLED_LED_PIN)
      pinMode(pin, OUTPUT);
    settings.pinClip = pin;
    settings.percentClip = percent;
  }
}

void setTooLowAlarmPin(long val0, long val1, long val2)
{
  int pin = val0;
  int percent = val1;
  bool validPin = ((checkDigitalPin(pin)
                    && checkNotBusyPin(pin, settings.pinSetpoint, settings.pinInput, settings.pinOutput)) || (pin == DISABLED_LED_PIN));
  invalidPin = validPin ? 0 : 1;

  if (invalidPin == 0)
  {
    if (settings.pinTooLow != DISABLED_LED_PIN)
    {
      pinMode(settings.pinTooLow, OUTPUT);
      digitalWrite(settings.pinTooLow, LOW);
    }
    if (pin != DISABLED_LED_PIN)
      pinMode(pin, OUTPUT);
    settings.pinTooLow = pin;
    settings.percentTooLow = percent;
  }
}

void setSSRInfoPin(long val0, long val1, long val2)
{
  int pin = val0;
  bool validPin = ((checkDigitalPin(pin)
                    && checkNotBusyPin(pin, settings.pinSetpoint, settings.pinInput, settings.pinOutput)) || (pin == DISABLED_LED_PIN));
  invalidPin = validPin ? 0 : 1;

  if (invalidPin == 0)
  {
    if (settings.pinSSRActive != DISABLED_LED_PIN)
    {
      pinMode(settings.pinSSRActive, OUTPUT);
      digitalWrite(settings.pinSSRActive, LOW);
    }
    if (pin != DISABLED_LED_PIN)
      pinMode(pin, OUTPUT);
    settings.pinSSRActive = pin;
  }
}

void enableRegulator(long val0, long val1, long val2)
{
  pidEnabled = 1;
  displayTitle = "INFO";
  displayStr = "PID ENABLED";
  displayInfo = 1;
}

void disableRegulator(long val0, long val1, long val2)
{
  pidEnabled = 0;
  displayTitle = "INFO";
  displayStr = "PID DISABLED";
  displayInfo = 1;
  Output = 0;
  disableAllOutputs();
}

void saveSettings(long val0, long val1, long val2)
{
  EEPROM.put(0, settings);
  displayTitle = "SAVE SETTINGS";
  displayStr = "OK";
  displayInfo = 1;
}

void loadSettings(long val0, long val1, long val2)
{
  if (pidEnabled == 0)
  {
    EEPROM.get(0, settings);
    restoreSettings();
    displayTitle = "LOAD SETTINGS";
    displayStr = "OK";
  }
  else
  {
    displayTitle = "ERROR";
    displayStr = "PID IS ENABLED";
  }
  displayInfo = 1;
}

void disableAllOutputs()
{
  int valoff = (settings.pidMode == 3) ? (1 - settings.relayHigh) : 0;
  digitalWrite(settings.pinOutput, valoff);
  if (settings.pidMode == 2)
  {
    digitalWrite(settings.pinOutputPWM16H, LOW);
    digitalWrite(settings.pinOutputPWM16L, LOW);
  }
  if (settings.pinClip != DISABLED_LED_PIN)
    digitalWrite(settings.pinClip, LOW);
  if (settings.pinTooLow != DISABLED_LED_PIN)
    digitalWrite(settings.pinTooLow, LOW);
  if (settings.pinSSRActive != DISABLED_LED_PIN)
    digitalWrite(settings.pinSSRActive, LOW);
}

void loadDefaults()
{
  settings.pinInput = DEFAULT_INPUT_PIN;
  settings.pinSetpoint = DEFAULT_SETPOINT_PIN;
  settings.pinOutput = DEFAULT_OUTPUT_PIN;
  settings.pinOutputPWM16H = DEFAULT_OUTPUT_PIN_PWM16_H;
  settings.pinOutputPWM16L = DEFAULT_OUTPUT_PIN_PWM16_L;
  settings.inputAsSetpoint = 0;
  settings.pidMode = 0;
  settings.windowSize = DEFAULT_WINDOW_SIZE;
  settings.sampleTime = DEFAULT_SAMPLE_TIME;
  settings.relayHigh = 1;
  settings.Kp = DEFAULT_P;
  settings.Ki = DEFAULT_I;
  settings.Kd = DEFAULT_D;
  settings.pOnE = 1;
  settings.controllerDirection = 0;
  settings.pinClip = DISABLED_LED_PIN; // means that pin is not set
  settings.percentClip = 0;
  settings.pinTooLow = DISABLED_LED_PIN; // means that pin is not set
  settings.percentTooLow = 0;
  settings.pinSSRActive = DISABLED_LED_PIN; // means that pin is not set
}

void restoreSettings()
{
  if (settings.pidMode > 3)
  {
    settings.pidMode = 0;
  }

  if (!checkAnalogPin(settings.pinInput))
  {
    settings.pinInput = DEFAULT_INPUT_PIN;
  }
  pinMode(settings.pinInput, INPUT);

  switch (settings.pidMode)
  {
    case 0:
      {
        if (!checkPWMPin(settings.pinOutput))
        {
          settings.pinOutput = DEFAULT_OUTPUT_PIN;
        }
        pinMode(settings.pinOutput, OUTPUT);
      }
      break;
    case 1:
      {
        if (!checkPWMPin(settings.pinOutput))
        {
          settings.pinOutput = DEFAULT_OUTPUT_PIN;
        }
        pinMode(settings.pinOutput, OUTPUT);
        setupPWM16();
      }
      break;
    case 2:
      {
        if (!checkPWMPin(settings.pinOutputPWM16H))
        {
          settings.pinOutputPWM16H = DEFAULT_OUTPUT_PIN_PWM16_H;
        }
        pinMode(settings.pinOutputPWM16H, OUTPUT);
        if (!checkPWMPin(settings.pinOutputPWM16L))
        {
          settings.pinOutputPWM16L = DEFAULT_OUTPUT_PIN_PWM16_L;
        }
        pinMode(settings.pinOutputPWM16L, OUTPUT);
      }
      break;
    case 3:
      {
        if (!checkDigitalPin(settings.pinOutput))
        {
          settings.pinOutput = DEFAULT_OUTPUT_PIN;
        }
        pinMode(settings.pinOutput, OUTPUT);
      }
      break;
  }

  if (!checkAnalogPin(settings.pinSetpoint))
  {
    settings.pinSetpoint = DEFAULT_SETPOINT_PIN;
  }
  pinMode(settings.pinSetpoint, INPUT);

  if (settings.inputAsSetpoint > 1)
  {
    settings.inputAsSetpoint = 0;
  }

  if ((settings.windowSize < 0) || (settings.windowSize > 32767))
  {
    settings.windowSize = DEFAULT_WINDOW_SIZE;
  }

  if ((settings.sampleTime < 0) || (settings.sampleTime > 32767))
  {
    settings.sampleTime = DEFAULT_SAMPLE_TIME;
  }

  switch (settings.pidMode)
  {
    case 0:
      pid.SetOutputLimits(0, 255);
      break;
    case 1:
    case 2: pid.SetOutputLimits(0, 65535);
      break;
    case 3:
      pid.SetOutputLimits(0, settings.windowSize);
      break;
  }

  if (settings.relayHigh > 1)
  {
    settings.relayHigh = 1;
  }

  if (isnan(settings.Kp))
  {
    settings.Kp = DEFAULT_P;
  }

  if (isnan(settings.Ki))
  {
    settings.Ki = DEFAULT_I;
  }

  if (isnan(settings.Kd))
  {
    settings.Kd = DEFAULT_D;
  }

  if (settings.pOnE > 1)
  {
    settings.pOnE = 1;
  }

  if (settings.controllerDirection > 1)
  {
    settings.controllerDirection = 0;
  }

  if ((settings.pinClip < 0) || (settings.pinClip > DISABLED_LED_PIN))
  {
    settings.pinClip = DISABLED_LED_PIN;
  }

  if (settings.percentClip < 0)
  {
    settings.percentClip = 0;
  }

  if ((settings.pinTooLow < 0) || (settings.pinTooLow > DISABLED_LED_PIN))
  {
    settings.pinTooLow = DISABLED_LED_PIN;
  }

  if ((settings.percentTooLow < 0) || (settings.percentTooLow > 100))
  {
    settings.percentTooLow = 100;
  }

  if ((settings.pinSSRActive < 0) || (settings.pinSSRActive > DISABLED_LED_PIN))
  {
    settings.pinSSRActive = DISABLED_LED_PIN;
  }

  if ((settings.pinClip != DISABLED_LED_PIN) && (checkDigitalPin(settings.pinClip)))
  {
    pinMode(settings.pinClip, OUTPUT);
  }
  if ((settings.pinTooLow != DISABLED_LED_PIN) && (checkDigitalPin(settings.pinTooLow)))
  {
    pinMode(settings.pinTooLow, OUTPUT);
  }
  if ((settings.pinSSRActive != DISABLED_LED_PIN) && (checkDigitalPin(settings.pinSSRActive)))
  {
    pinMode(settings.pinSSRActive, OUTPUT);
  }


  pid.SetTunings(settings.Kp, settings.Ki, settings.Kd, settings.pOnE);
  pid.SetControllerDirection(settings.controllerDirection);
  pid.SetSampleTime(settings.sampleTime);

}

void doOutput()
{
  switch (settings.pidMode)
  {
    case 0:
      {
        analogWrite(settings.pinOutput, Output);
      }
      break;
    case 1:
      {
        analogWrite16(settings.pinOutput, Output);
      }
      break;
    case 2:
      {
        unsigned int outInt = (unsigned int) Output;
        unsigned int outH = (outInt >> 8) & 0xFF;
        unsigned int outL = outInt & 0xFF;
        analogWrite(settings.pinOutputPWM16H, outH);
        analogWrite(settings.pinOutputPWM16L, outL);
      }
      break;
    case 3:
      {
        if (millis() - windowStartTime > settings.windowSize)
        { //time to shift the Relay Window
          windowStartTime += settings.windowSize;
        }
        if (Output < millis() - windowStartTime)
        {
          digitalWrite(settings.pinOutput, (1 - settings.relayHigh));
          if (settings.pinSSRActive != DISABLED_LED_PIN) digitalWrite(settings.pinSSRActive, LOW);
          swOn = 0;
        }
        else
        {
          digitalWrite(settings.pinOutput, settings.relayHigh);
          if (settings.pinSSRActive != DISABLED_LED_PIN) digitalWrite(settings.pinSSRActive, HIGH);
          swOn = 1;
        }
      }
      break;
  }

}

void doWarningLEDS()
{
  if (settings.pinClip != DISABLED_LED_PIN)
  {
    double percent = (double) settings.percentClip;
    if ((Setpoint > 0) && ((100.0 * Input / Setpoint) > percent))
    {
      digitalWrite(settings.pinClip, HIGH);
    }
    else
    {
      digitalWrite(settings.pinClip, LOW);
    }
  }

  if (settings.pinTooLow != DISABLED_LED_PIN)
  {
    double percent = (double) settings.percentTooLow;
    if ((Setpoint > 0) && ((100.0 * Input / Setpoint) < percent))
    {
      digitalWrite(settings.pinTooLow, HIGH);
    }
    else
    {
      digitalWrite(settings.pinTooLow, LOW);
    }
  }
}

void printFilledStr(String s, int row)
{
  lcd.setCursor(0, row);
  lcd.print(rightFilledStr(s, 16));
}

void printError(String caption, String message, byte *code)
{
  printFilledStr(caption, 0);
  printFilledStr(message, 1);
  dispCounter += 1;
  if (dispCounter >= DISPLAY_TIME)
  {
    *code = 0;
    dispCounter = 0;
  }
}

void printEntering()
{
  printFilledStr("ENTER MMI CODE", 0);
  if (blnk == 0)
  {
    printFilledStr(shortcode, 1);
  }
  else
  {
    printFilledStr(shortcode + '_', 1);
  }
  blnk += 1;
  if (blnk == 2) blnk = 0;
}

void printInfo()
{
  printFilledStr(displayTitle, 0);
  printFilledStr(displayStr, 1);
  dispCounter += 1;
  if (dispCounter >= (DISPLAY_TIME * 2))
  {
    displayInfo = 0;
    dispCounter = 0;
  }
}

void printCustomChar(byte ch, byte col, byte row)
{
  lcd.setCursor(col, row);
  lcd.write(ch);
}

void printValues()
{
  String s;
  s = "SV:" + leftFilledStr(String((unsigned int) Setpoint), 4);
  s += " PV:" + leftFilledStr(String((unsigned int) Input), 4);
  printFilledStr(s, 0);
  byte len = settings.pidMode == 0 ? 4 : 5;
  s = "OV:" + leftFilledStr(String((unsigned int) Output), len);
  printFilledStr(s, 1);

  if (pidEnabled == 1)
  {
    if (Input > LastInput)
    {
      printCustomChar(0, 15, 0);
    }
    else if (Input < LastInput)
    {
      printCustomChar(1, 15, 0);
    }

    if (settings.pidMode == 3) //SSR
    {
      if (swOn == 0)
      {
        printCustomChar(2, 9, 1);
      }
      else
      {
        printCustomChar(3, 9, 1);
      }
    }
  }

}
