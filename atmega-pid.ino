
/*
    ATMEGA328 PID MMI code regulator

    MMI codes:

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
      *19*XXXX#       set backlight IDLE time s
      *20*XX#         set input analog pin
      *21*XX#         set setpoint analog pin
      *22*XX#         set output digital relay or PWM pin
      *23*X#          set use analog input as setpoint X = 0 or 1
      *24*XX*Y#       set digital pin as 0 or 1
      *25*XXXX#       set desired analog input 0..1023
      *26*XX*YY#      set 16 bit mode PWM output MSB (XX) and LSB (YY) pair
      *31*XX*YY#      set the clip alarm LED pin XX to YY SP percent
      *32*XX*YY#      set too low input value LED pin XX to YY SP percent
      *33*XX#         set SSR mode relay enable information LED pin
      *41*#           enable PID regulator
      *42*#           disable PID regulator
      *51*#           save settings to EEPROM
      *52*#           load settings from EEPROM

*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <EEPROM.h>

#include "strings.h"
#include "limits.h"
#include "pins.h"
#include "pwmio.h"
#include "mmicode.h"
#include "scheduler.h"

#define BUZZ_PIN 6
#define DISPLAY_TIME 3  // 3x2s
#define DEFAULT_INPUT_PIN 14
#define DEFAULT_SETPOINT_PIN 15
#define DEFAULT_OUTPUT_PIN_PWM16_H 9
#define DEFAULT_OUTPUT_PIN_PWM16_L 10
#define DEFAULT_OUTPUT_PIN 9
#define DEFAULT_WINDOW_SIZE 5000
#define DEFAULT_SAMPLE_TIME 100
#define DEFAULT_IDLE_TIME 60
#define DISABLED_LED_PIN 255
#define DEFAULT_SCALING_FACTOR 255.0 / 1023.0
#define DEFAULT_P 1.0
#define DEFAULT_I 0.0
#define DEFAULT_D 0.0

LiquidCrystal_I2C lcd(0x3F, 16, 2);

const byte ROWS = 4;
const byte COLS = 4;

char hexaKeys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

byte rowPins[ROWS] = { 13, 12, 11, 8 };
byte colPins[COLS] = { 7, 5, 4, 3 };

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

byte powerOn[] = {
	B00100,
	B01110,
	B10101,
	B10101,
	B10101,
	B10001,
	B10001,
	B01110
};

byte bell[] = {
  B00100,
  B01110,
  B01110,
  B01110,
  B11111,
  B00000,
  B00100,
  B00000
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

double ValueSetpoint = 0, ValueInput = 0, ValueLastInput = 0;
double Setpoint = 0, Input = 0, Output = 0, lastInput = 0;
double outputSum = 0, outMin = 0, outMax = 255;
unsigned long lastTime = 0;
double Kp = DEFAULT_P, Ki = DEFAULT_I, Kd = DEFAULT_D;
double ScalingFactor = DEFAULT_SCALING_FACTOR;

struct Settings {
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
  int idleTime;
};

Settings settings;

String shortcode = "";
String lastShortcode = "";
String displayTitle = "";
String displayStr = "";

void setup() {
  loadDefaults();
  scalePidParams();

  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(settings.pinInput, INPUT);
  pinMode(settings.pinSetpoint, INPUT);
  pinMode(settings.pinOutput, OUTPUT);
  pinMode(settings.pinOutputPWM16H, OUTPUT);
  pinMode(settings.pinOutputPWM16L, OUTPUT);
  digitalWrite(settings.pinOutput, LOW);
  digitalWrite(settings.pinOutputPWM16H, LOW);
  digitalWrite(settings.pinOutputPWM16L, LOW);

  ValueInput = analogRead(settings.pinInput);
  ValueSetpoint = 0;

  windowStartTime = millis();
  lastTime = millis()-settings.sampleTime;

  lcd.init();
  lcd.createChar(0, upArrow);
  lcd.createChar(1, downArrow);
  lcd.createChar(2, switchOff);
  lcd.createChar(3, switchOn);
  lcd.createChar(4, powerOn);
  lcd.createChar(5, bell);
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
  Mmi.add("*19*", setIdleTime);
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
}

/* Main Program */
void loop() {
  Sch.dispatch();
}

void keyUpdate() {

  char key = keypad.getKey();
  if (key) {
    if (idle == 1) {
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

    switch (key) {
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
          if (shortcode.length() < 16) {
            shortcode += key;
          }
        }
        break;
    }

    tone(BUZZ_PIN, 1000, 50);
  }
}

void ledUpdate() {
  idleCounter += 1;
  //experimental 1.55 * idleTime to real default 60s
  if ((float)idleCounter >= (float)(1.55 * settings.idleTime)) {
    lcd.setBacklight(0);
    idleCounter = 0;
    idle = 1;
  }

  if (invalidMMI == 1) {
    printError("ERROR", "INVALID CODE", &invalidMMI);
    return;
  }

  if (invalidPin == 1) {
    printError("ERROR", "INVALID PIN", &invalidPin);
    return;
  }

  if (entering == 1) {
    printEntering();
    return;
  }

  if (displayInfo == 1) {
    printInfo();
    return;
  }

  printValues();
}

void pidUpdate() {
  if (settings.inputAsSetpoint == 1) {
    setpointValue = analogRead(settings.pinSetpoint);
    ValueSetpoint = (double)setpointValue;
  }

  ValueLastInput = ValueInput;
  ValueInput = analogRead(settings.pinInput);

  Setpoint = (double) (ValueSetpoint * ScalingFactor);
  Input = (double) (ValueInput * ScalingFactor);

  if (pidEnabled == 1) {
    pidCompute();
    doOutput();
    doWarningLEDS();
  }
}

void scanMMI() {
  invalidMMI = 1;
  entering = 1;
  bool execResult = Mmi.exec(shortcode);
  invalidMMI = execResult ? 0 : 1;
  entering = execResult ? 0 : 1;
}

void repeatLastCode(long val0, long val1, long val2) {
  if (lastShortcode != "") {
    shortcode = lastShortcode;
    scanMMI();
  }
}

void printParams(long val0, long val1, long val2) {
  if (pidEnabled == 0) {
    const char* modes[4] = { "PWM", "PWM 16", "DUAL PWM", "SSR" };
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
  } else {
    displayTitle = "ERROR";
    displayStr = "PID IS ACTIVE";
    displayInfo = 1;
  }
}

void setKpValue(long val0, long val1, long val2) {
  long value = limitValue(val0, 0, 2147483647);
  settings.Kp = (double)value / 100.0;
  scalePidParams();
}

void setKiValue(long val0, long val1, long val2) {
  long value = limitValue(val0, 0, 2147483647);
  settings.Ki = (double)value / 100.0;
  scalePidParams();
}

void setKdValue(long val0, long val1, long val2) {
  long value = limitValue(val0, 0, 2147483647);
  settings.Kd = (double)value / 100.0;
  scalePidParams();
}

void setPOnE(long val0, long val1, long val2) {
  int value = limitValue(val0, 0, 1);
  settings.pOnE = value;
  scalePidParams();
}

void setDirection(long val0, long val1, long val2) {
  int value = limitValue(val0, 0, 1);
  settings.controllerDirection = value;
  scalePidParams();
}

void setPIDMode(long val0, long val1, long val2) {
  int value = limitValue(val0, 0, 3);
  settings.pidMode = value;
  switch (settings.pidMode) {
    case 0:
      {
        outMin = 0; outMax = 255;  
        ScalingFactor = DEFAULT_SCALING_FACTOR;
      }
      break;
    case 1:
      {
        outMin = 0; outMax = 65535;  
        setupPWM16();
        ScalingFactor = 65535.0 / 1023.0;
      }
      break;
    case 2:
      {
        outMin = 0; outMax = 65535;            
        ScalingFactor = 65535.0 / 1023.0;
      }
      break;
    case 3:
      {
        outMin = 0; outMax = settings.windowSize;            
        ScalingFactor = (double)settings.windowSize / 1023.0;
      }
      break;
  }

  windowStartTime = millis();
}

void setRelayHighState(long val0, long val1, long val2) {
  int value = limitValue(val0, 0, 1);
  settings.relayHigh = value;
}

void setRelayWindow(long val0, long val1, long val2) {
  int value = limitValue(val0, 0, 32767);
  settings.windowSize = value;
  switch (settings.pidMode) {
    case 0:
      {
        outMin = 0; outMax = 255;    
        ScalingFactor = DEFAULT_SCALING_FACTOR;
      }
      break;
    case 1:
      {
        outMin = 0; outMax = 65535;            
        setupPWM16();
        ScalingFactor = 65535.0 / 1023.00;
      }
      break;
    case 2:
      {
        outMin = 0; outMax = 65535;            
        ScalingFactor = 65535.0 / 1023.00;
      }
      break;
    case 3:
      {
        outMin = 0; outMax = settings.windowSize;            
        ScalingFactor = (double)settings.windowSize / 1023.00;
      }
      break;
  }
}

void setSampleTime(long val0, long val1, long val2) {
  int value = limitValue(val0, 50, 32767);
  settings.sampleTime = value;  
  scalePidParams();
}

void setIdleTime(long val0, long val1, long val2) {
  int value = limitValue(val0, 10, 32767);
  settings.idleTime = value;
}

void setInputPin(long val0, long val1, long val2) {
  int pin = val0;
  invalidPin = checkAnalogPin(pin) ? 0 : 1;
  if (invalidPin == 0) {
    settings.pinInput = pin;
    pinMode(pin, INPUT);
  }
}

void setSetpointPin(long val0, long val1, long val2) {
  int pin = val0;
  invalidPin = checkAnalogPin(pin) ? 0 : 1;
  if (invalidPin == 0) {
    settings.pinSetpoint = pin;
    pinMode(pin, INPUT);
  }
}

void setOutputPin(long val0, long val1, long val2) {
  int pin = val0;
  switch (settings.pidMode) {
    case 0:
      {
        invalidPin = checkPWMPin(pin) ? 0 : 1;
      }
      break;
    case 1:
      {
        invalidPin = checkPWMPin(pin) ? 0 : 1;
        setupPWM16();
      }
      break;
    case 2:
      {
        invalidPin = checkPWMPin(pin) ? 0 : 1;
      }
      break;
    case 3:
      {
        invalidPin = checkDigitalPin(pin) ? 0 : 1;
      }
      break;
  }

  if (invalidPin == 0) {
    settings.pinOutput = pin;
    pinMode(pin, OUTPUT);
  }
}

void setUseInputAsSetpoint(long val0, long val1, long val2) {
  int value = limitValue(val0, 0, 1);
  settings.inputAsSetpoint = value;
}

void setDigitalPinState(long val0, long val1, long val2) {
  int pin = val0;
  invalidPin = checkDigitalPin(pin) ? 0 : 1;
  if (invalidPin == 0) {
    int value = limitValue(val1, 0, 1);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, value);
  }
}

void setDesiredInput(long val0, long val1, long val2) {
  int value = limitValue(val0, 0, 1023);
  ValueSetpoint = (double)value;
}

void setDualPWMPair(long val0, long val1, long val2) {
  int pinH = val0;
  int pinL = val1;
  invalidPin = (checkPWMPin(pinH) && checkPWMPin(pinL)) ? 0 : 1;
  if (invalidPin == 0) {
    settings.pinOutputPWM16H = pinH;
    settings.pinOutputPWM16L = pinL;
    pinMode(pinH, OUTPUT);
    pinMode(pinL, OUTPUT);
  }
}

void setClipAlarmPin(long val0, long val1, long val2) {
  int pin = val0;
  int percent = val1;
  bool validPin = ((checkDigitalPin(pin)
                    && checkNotBusyPin(pin, settings.pinSetpoint, settings.pinInput, settings.pinOutput))
                   || (pin == DISABLED_LED_PIN));
  invalidPin = validPin ? 0 : 1;

  if (invalidPin == 0) {
    if (settings.pinClip != DISABLED_LED_PIN) {
      pinMode(settings.pinClip, OUTPUT);
      digitalWrite(settings.pinClip, LOW);
    }
    if (pin != DISABLED_LED_PIN)
      pinMode(pin, OUTPUT);
    settings.pinClip = pin;
    settings.percentClip = percent;
  }
}

void setTooLowAlarmPin(long val0, long val1, long val2) {
  int pin = val0;
  int percent = val1;
  bool validPin = ((checkDigitalPin(pin)
                    && checkNotBusyPin(pin, settings.pinSetpoint, settings.pinInput, settings.pinOutput))
                   || (pin == DISABLED_LED_PIN));
  invalidPin = validPin ? 0 : 1;

  if (invalidPin == 0) {
    if (settings.pinTooLow != DISABLED_LED_PIN) {
      pinMode(settings.pinTooLow, OUTPUT);
      digitalWrite(settings.pinTooLow, LOW);
    }
    if (pin != DISABLED_LED_PIN)
      pinMode(pin, OUTPUT);
    settings.pinTooLow = pin;
    settings.percentTooLow = percent;
  }
}

void setSSRInfoPin(long val0, long val1, long val2) {
  int pin = val0;
  bool validPin = ((checkDigitalPin(pin)
                    && checkNotBusyPin(pin, settings.pinSetpoint, settings.pinInput, settings.pinOutput))
                   || (pin == DISABLED_LED_PIN));
  invalidPin = validPin ? 0 : 1;

  if (invalidPin == 0) {
    if (settings.pinSSRActive != DISABLED_LED_PIN) {
      pinMode(settings.pinSSRActive, OUTPUT);
      digitalWrite(settings.pinSSRActive, LOW);
    }
    if (pin != DISABLED_LED_PIN)
      pinMode(pin, OUTPUT);
    settings.pinSSRActive = pin;
  }
}

void enableRegulator(long val0, long val1, long val2) {
  pidEnabled = 1;
  lastTime = millis()-settings.sampleTime;
}

void disableRegulator(long val0, long val1, long val2) {
  pidEnabled = 0;
  Output = 0;
  outputSum = 0;
  disableAllOutputs();
}

void saveSettings(long val0, long val1, long val2) {
  EEPROM.put(0, settings);
}

void loadSettings(long val0, long val1, long val2) {
  if (pidEnabled == 0) {
    EEPROM.get(0, settings);
    restoreSettings();
  } else {
    displayTitle = "ERROR";
    displayStr = "PID IS ENABLED";
    displayInfo = 1;
  }
}

void disableAllOutputs() {
  int valoff = (settings.pidMode == 3) ? (1 - settings.relayHigh) : 0;
  digitalWrite(settings.pinOutput, valoff);
  if (settings.pidMode == 2) {
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

void loadDefaults() {
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
  settings.pinClip = DISABLED_LED_PIN;  // means that pin is not set
  settings.percentClip = 0;
  settings.pinTooLow = DISABLED_LED_PIN;  // means that pin is not set
  settings.percentTooLow = 0;
  settings.pinSSRActive = DISABLED_LED_PIN;  // means that pin is not set
  settings.idleTime = DEFAULT_IDLE_TIME;
  ScalingFactor = DEFAULT_SCALING_FACTOR;
}

void restoreSettings() {
  if (settings.pidMode > 3) {
    settings.pidMode = 0;
  }

  if (!checkAnalogPin(settings.pinInput)) {
    settings.pinInput = DEFAULT_INPUT_PIN;
  }
  pinMode(settings.pinInput, INPUT);

  switch (settings.pidMode) {
    case 0:
      {
        if (!checkPWMPin(settings.pinOutput)) {
          settings.pinOutput = DEFAULT_OUTPUT_PIN;
        }
        pinMode(settings.pinOutput, OUTPUT);
      }
      break;
    case 1:
      {
        if (!checkPWMPin(settings.pinOutput)) {
          settings.pinOutput = DEFAULT_OUTPUT_PIN;
        }
        pinMode(settings.pinOutput, OUTPUT);
        setupPWM16();
      }
      break;
    case 2:
      {
        if (!checkPWMPin(settings.pinOutputPWM16H)) {
          settings.pinOutputPWM16H = DEFAULT_OUTPUT_PIN_PWM16_H;
        }
        pinMode(settings.pinOutputPWM16H, OUTPUT);
        if (!checkPWMPin(settings.pinOutputPWM16L)) {
          settings.pinOutputPWM16L = DEFAULT_OUTPUT_PIN_PWM16_L;
        }
        pinMode(settings.pinOutputPWM16L, OUTPUT);
      }
      break;
    case 3:
      {
        if (!checkDigitalPin(settings.pinOutput)) {
          settings.pinOutput = DEFAULT_OUTPUT_PIN;
        }
        pinMode(settings.pinOutput, OUTPUT);
      }
      break;
  }

  if (!checkAnalogPin(settings.pinSetpoint)) {
    settings.pinSetpoint = DEFAULT_SETPOINT_PIN;
  }
  pinMode(settings.pinSetpoint, INPUT);

  if (settings.inputAsSetpoint > 1) {
    settings.inputAsSetpoint = 0;
  }

  if ((settings.windowSize < 0) || (settings.windowSize > 32767)) {
    settings.windowSize = DEFAULT_WINDOW_SIZE;
  }

  if ((settings.sampleTime < 0) || (settings.sampleTime > 32767)) {
    settings.sampleTime = DEFAULT_SAMPLE_TIME;
  }

  if ((settings.idleTime < 10) || (settings.idleTime > 32767)) {
    settings.idleTime = DEFAULT_IDLE_TIME;
  }

  switch (settings.pidMode) {
    case 0:
      {
        outMin = 0; outMax = 255;            
        ScalingFactor = DEFAULT_SCALING_FACTOR;
      }
      break;
    case 1:
    case 2:
      {
        outMin = 0; outMax = 65535;            
        ScalingFactor = 65535.0 / 1023.0;
      }
      break;
    case 3:
      {
        outMin = 0; outMax = settings.windowSize;            
        ScalingFactor = (double)settings.windowSize / 1023.0;
      }
      break;
  }

  if (settings.relayHigh > 1) {
    settings.relayHigh = 1;
  }

  if (isnan(settings.Kp)) {
    settings.Kp = DEFAULT_P;
  }

  if (isnan(settings.Ki)) {
    settings.Ki = DEFAULT_I;
  }

  if (isnan(settings.Kd)) {
    settings.Kd = DEFAULT_D;
  }

  if (settings.pOnE > 1) {
    settings.pOnE = 1;
  }

  if (settings.controllerDirection > 1) {
    settings.controllerDirection = 0;
  }

  if ((settings.pinClip < 0) || (settings.pinClip > DISABLED_LED_PIN)) {
    settings.pinClip = DISABLED_LED_PIN;
  }

  if (settings.percentClip < 0) {
    settings.percentClip = 0;
  }

  if ((settings.pinTooLow < 0) || (settings.pinTooLow > DISABLED_LED_PIN)) {
    settings.pinTooLow = DISABLED_LED_PIN;
  }

  if ((settings.percentTooLow < 0) || (settings.percentTooLow > 100)) {
    settings.percentTooLow = 100;
  }

  if ((settings.pinSSRActive < 0) || (settings.pinSSRActive > DISABLED_LED_PIN)) {
    settings.pinSSRActive = DISABLED_LED_PIN;
  }

  if ((settings.pinClip != DISABLED_LED_PIN) && (checkDigitalPin(settings.pinClip))) {
    pinMode(settings.pinClip, OUTPUT);
  }
  if ((settings.pinTooLow != DISABLED_LED_PIN) && (checkDigitalPin(settings.pinTooLow))) {
    pinMode(settings.pinTooLow, OUTPUT);
  }
  if ((settings.pinSSRActive != DISABLED_LED_PIN) && (checkDigitalPin(settings.pinSSRActive))) {
    pinMode(settings.pinSSRActive, OUTPUT);
  }
  
  scalePidParams();
}

void scalePidParams()
{
   Kp = settings.Kp;
   Ki = settings.Ki;
   Kd = settings.Kd;

   double ratio  = (double)settings.sampleTime
                      / (double)DEFAULT_SAMPLE_TIME;
   Ki *= ratio;
   Kd /= ratio;

   double SampleTimeInSec = ((double)settings.sampleTime)/1000;
   Ki = Ki * SampleTimeInSec;
   Kd = Kd / SampleTimeInSec;

  if(settings.controllerDirection == 1)
   {
      Kp = (0 - Kp);
      Ki = (0 - Ki);
      Kd = (0 - Kd);
   }
}

void pidCompute()
{
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   byte pOnE = settings.pOnE;

   if(timeChange>=settings.sampleTime)
   {
      /*Compute all the working error variables*/
      double error = Setpoint - Input;
      double dInput = (Input - lastInput);
      outputSum += (Ki * error);
      
      /*Add Proportional on Measurement, if P_ON_M is specified*/
      if(!pOnE) outputSum-= Kp * dInput;

      if(outputSum > outMax) outputSum= outMax;
      else if(outputSum < outMin) outputSum= outMin;

      /*Add Proportional on Error, if P_ON_E is specified*/
      if(settings.pOnE) Output = Kp * error;
      else Output = 0;

      /*Compute Rest of PID Output*/
      Output += outputSum - Kd * dInput;

      if(Output > outMax){
          outputSum -= Output - outMax; // backcalculate integral to feasability
          Output = outMax;
      }
      else if(Output < outMin) {
          outputSum += outMin - Output; // backcalculate integral to feasability
          Output = outMin;
      }

      /*Remember some variables for next time*/
      lastInput = Input;      
      lastTime = now;
   }

}
void doOutput() {
  switch (settings.pidMode) {
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
        unsigned int outInt = (unsigned int)Output;
        unsigned int outH = (outInt >> 8) & 0xFF;
        unsigned int outL = outInt & 0xFF;
        analogWrite(settings.pinOutputPWM16H, outH);
        analogWrite(settings.pinOutputPWM16L, outL);
      }
      break;
    case 3:
      {
        if (millis() - windowStartTime > settings.windowSize) {  //time to shift the Relay Window
          windowStartTime += settings.windowSize;
        }
        if (Output > millis() - windowStartTime) {
          digitalWrite(settings.pinOutput, settings.relayHigh);
          if (settings.pinSSRActive != DISABLED_LED_PIN) digitalWrite(settings.pinSSRActive, HIGH);
          swOn = 1;
        } else {
          digitalWrite(settings.pinOutput, (1 - settings.relayHigh));
          if (settings.pinSSRActive != DISABLED_LED_PIN) digitalWrite(settings.pinSSRActive, LOW);
          swOn = 0;
        }
      }
      break;
  }
}

void doWarningLEDS() {
  if (settings.pinClip != DISABLED_LED_PIN) {
    double percent = (double)settings.percentClip;
    if ((ValueSetpoint > 0) && ((100.0 * ValueInput / ValueSetpoint) > percent)) {
      digitalWrite(settings.pinClip, HIGH);
    } else {
      digitalWrite(settings.pinClip, LOW);
    }
  }

  if (settings.pinTooLow != DISABLED_LED_PIN) {
    double percent = (double)settings.percentTooLow;
    if ((ValueSetpoint > 0) && ((100.0 * ValueInput / ValueSetpoint) < percent)) {
      digitalWrite(settings.pinTooLow, HIGH);
    } else {
      digitalWrite(settings.pinTooLow, LOW);
    }
  }
}

void printFilledStr(String s, int row) {
  lcd.setCursor(0, row);
  lcd.print(rightFilledStr(s, 16));
}

void printError(String caption, String message, byte* code) {
  printFilledStr(caption, 0);
  printFilledStr(message, 1);
  dispCounter += 1;
  if (dispCounter >= DISPLAY_TIME) {
    *code = 0;
    dispCounter = 0;
  }
}

void printEntering() {
  printFilledStr("ENTER MMI CODE", 0);
  if (blnk == 0) {
    printFilledStr(shortcode, 1);
  } else {
    printFilledStr(shortcode + '_', 1);
  }
  blnk += 1;
  if (blnk == 2) blnk = 0;
}

void printInfo() {
  printFilledStr(displayTitle, 0);
  printFilledStr(displayStr, 1);
  dispCounter += 1;
  if (dispCounter >= (DISPLAY_TIME * 2)) {
    displayInfo = 0;
    dispCounter = 0;
  }
}

void printCustomChar(byte ch, byte col, byte row) {
  lcd.setCursor(col, row);
  lcd.write(ch);
}

void printValues() {
  String s;
  s = "SV:" + leftFilledStr(String((unsigned int)ValueSetpoint), 4);
  s += " PV:" + leftFilledStr(String((unsigned int)ValueInput), 4);
  printFilledStr(s, 0);
  byte len = settings.pidMode == 0 ? 4 : 5;
  s = "OV:" + leftFilledStr(String((unsigned int)Output), len);
  printFilledStr(s, 1);

  if (pidEnabled == 1) {
    printCustomChar(4, 15, 1);

    if (ValueInput > ValueLastInput) {
      printCustomChar(0, 15, 0);
    } else if (ValueInput < ValueLastInput) {
      printCustomChar(1, 15, 0);
    }

    if (settings.pidMode == 3)  //SSR
    {
      if (swOn == 0) {
        printCustomChar(2, 9, 1);
      } else {
        printCustomChar(3, 9, 1);
      }
    }

    if (Input > Setpoint)
    {
        printCustomChar(5, 10, 1);
    }

  }
}
