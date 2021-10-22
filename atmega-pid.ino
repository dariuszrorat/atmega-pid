
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
      18*XXXX#       set PID sample time ms or tuner interval
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
      34*XX#         set tuner target input value 0..1023, default 511
      35*XX#         set tuner output range, 0..255, default 255
      36*X#          set tuning mode, 0 = basic, 1 = less overshoot, 2 = no overshoot, default 0
      37*XX#         set tuning cycles, default 10
      41*#           enable PID regulator
      42*#           disable PID regulator
      43*#           start auto tuner, press any key to abort
      51*#           save settings to EEPROM
      52*#           load settings from EEPROM

*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include <pidautotuner.h>

#include "strings.h"
#include "pins.h"

#define BUZZ_PIN 6
#define NUM_COMMANDS 30
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

const char* MMI_COMMANDS[NUM_COMMANDS]
{
  "*00*",
  "*01*",
  "*10*",
  "*11*",
  "*12*",
  "*13*",
  "*14*",
  "*15*",
  "*16*",
  "*17*",
  "*18*",
  "*20*",
  "*21*",
  "*22*",
  "*23*",
  "*24*",
  "*25*",
  "*26*",
  "*31*",
  "*32*",
  "*33*",
  "*34*",
  "*35*",
  "*36*",
  "*37*",
  "*41*",
  "*42*",
  "*43*",
  "*51*",
  "*52*"
};

byte rowPins[ROWS] = {13, 12, 11, 8};
byte colPins[COLS] = {7, 5, 4, 3};

Keypad keypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

byte lock = 0;
byte blnk = 0;
byte idle = 0;
byte entering = 0;
byte displayInput = 0;
byte invalidMMI = 0;
byte invalidPin = 0;
byte dispCounter = 0;
byte idleCounter = 0;
unsigned int counter = 0;

byte pidEnabled = 0;
unsigned long windowStartTime;
int setpointValue = 0;

byte tunerRunning = 0;
int targetInputValue = 511;
unsigned int outputRange = 255;
PIDAutotuner::ZNMode znMode = PIDAutotuner::ZNModeBasicPID;
long loopInterval;
int tuningCycles = 10;

double Setpoint = 0, Input = 0, Output = 0;
double Kp = DEFAULT_P, Ki = DEFAULT_I, Kd = DEFAULT_D;

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
PIDAutotuner tuner = PIDAutotuner();

String MMI = "";
String lastMMI = "";
String displayTitle = "";
String displayStr = "";

void setup()
{
  loadDefaults();

  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(settings.pinInput, INPUT);
  pinMode(settings.pinSetpoint, INPUT);
  pinMode(settings.pinOutput, OUTPUT);
  digitalWrite(settings.pinOutput, LOW);

  Input = analogRead(settings.pinInput);
  Setpoint = 0;
  pid.SetTunings(settings.Kp, settings.Ki, settings.Kd, settings.pOnE);
  pid.SetControllerDirection(settings.controllerDirection);
  pid.SetSampleTime(settings.sampleTime);
  pid.SetMode(AUTOMATIC);

  tuner.setTargetInputValue(targetInputValue);
  tuner.setLoopInterval(settings.sampleTime * 1000);
  tuner.setOutputRange(0, 255);
  tuner.setZNMode(PIDAutotuner::ZNModeBasicPID);

  windowStartTime = millis();

  lcd.init();
  lcd.backlight();

  printFilledStr("WELCOME TO MMI", 0);
  printFilledStr("PID CONTROLLER", 1);
  delay(2000);
}

/* Main Program */
void loop()
{
  if (settings.inputAsSetpoint == 1)
  {
    setpointValue = analogRead(settings.pinSetpoint);
    Setpoint = (double) setpointValue;
  }

  Input = analogRead(settings.pinInput);

  if (pidEnabled == 1)
  {
    pid.Compute();
    doOutput();
    doWarningLEDS();
  }

  //auto tuner
  if ((tunerRunning == 1) && (pidEnabled == 0))
  {
    doTuning();
  }

  ledUpdate();
  settingsUpdate();
  delay(1);

  counter += 1;
  if (counter > 1000)
  {
    counter = 0;
  }
}

void settingsUpdate()
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
    displayInput = 0;
    idleCounter = 0;

    switch (key)
    {
      case 'A':
        {
          MMI = "";
          entering = 0;
        }
        break;
      case 'B':
        {
          MMI = MMI.substring(0, MMI.length() - 1);
        }
        break;
      case 'C': MMI = ""; break;
      case 'D':
        {
          scanMMI();
          lastMMI = MMI;
          MMI = "";
        }
        break;
      default:
        {
          if (MMI.length() < 16)
          {
            MMI += key;
          }
        }
        break;
    }

    tone(BUZZ_PIN, 1000, 50);
  }
}

void ledUpdate()
{
  if ((counter % 500) != 0)
  {
    return;
  }

  idleCounter += 1;
  if (idleCounter == (2 * IDLE_TIME))
  {
    lcd.setBacklight(0);
    idleCounter = 0;
    idle = 1;
  }

  if (invalidMMI == 1)
  {
    printFilledStr("ERROR", 0);
    printFilledStr("INVALID CODE", 1);
    dispCounter += 1;
    if (dispCounter >= DISPLAY_TIME)
    {
      invalidMMI = 0;
      dispCounter = 0;
    }
    return;
  }

  if (invalidPin == 1)
  {
    printFilledStr("ERROR", 0);
    printFilledStr("INVALID PIN", 1);
    dispCounter += 1;
    if (dispCounter >= DISPLAY_TIME)
    {
      invalidPin = 0;
      dispCounter = 0;
    }
    return;
  }

  if (entering == 1)
  {
    printFilledStr("ENTER MMI CODE", 0);
    if (blnk == 0)
    {
      printFilledStr(MMI, 1);
    }
    else
    {
      if (MMI.length() < 16)
      {
        printFilledStr(MMI + '_', 1);
      }
      else
      {
        printFilledStr(MMI + '_', 1);
      }
    }
    blnk += 1;
    if (blnk == 2) blnk = 0;
    return;
  }

  if (displayInput == 1)
  {
    printFilledStr(displayTitle, 0);
    printFilledStr(displayStr, 1);
    dispCounter += 1;
    if (dispCounter >= (DISPLAY_TIME * 2))
    {
      displayInput = 0;
      dispCounter = 0;
    }
    return;
  }

  String s;
  s = "SETPOINT: " + String((int) Setpoint);
  printFilledStr(s, 0);
  s = "INPUT   : " + String((int) Input);
  printFilledStr(s, 1);
}

void scanMMI()
{
  int i;
  invalidMMI = 1;

  entering = 1;
  String cmd = MMI.substring(0, 4);
  String params = MMI.substring(4, MMI.length() - 1);
  String existingCmd;

  int len = MMI.length();

  for (i = 0; i < NUM_COMMANDS; i++)
  {
    existingCmd = String(MMI_COMMANDS[i]);
    if (cmd == existingCmd)
    {
      invalidMMI = 0;
      entering = 0;
      break;
    }
  }

  if ((len > 0) && (MMI[len - 1] != '#'))
  {
    invalidMMI = 1;
    entering = 1;
  }

  if (invalidMMI == 0)
  {
    execMMI(cmd, params);
  }
}


void execMMI(String cmd, String params)
{
  String cm = cmd.substring(1, 3);
  int command = cm.toInt();
  String svalue0 = trimAll(getParam(params, '*', 0));
  String svalue1 = trimAll(getParam(params, '*', 1));

  switch (command)
  {
    case 00: //repeat last MMI command
      {
        if (lastMMI != "")
        {
          MMI = lastMMI;
          scanMMI();
        }
      }
      break;
    case 01:
      {
        const char* modes[4] = {"PWM", "PWM16", "DUALPWM", "SSR"};
        String s = String(modes[settings.pidMode]);
        displayTitle = "P=" + String(settings.Kp) + " I=" + String(settings.Ki);
        displayStr = "D=" + String(settings.Kd) + " " + s;
        displayInput = 1;
      }
      break;

    case 10:
      {
        int value = limitValue(svalue0.toInt(), 0, 32767);
        settings.Kp = (double) value / 100.0;
        pid.SetTunings(settings.Kp, settings.Ki, settings.Kd, settings.pOnE);
      }
      break;
    case 11:
      {
        int value = limitValue(svalue0.toInt(), 0, 32767);
        settings.Ki = (double) value / 100.0;
        pid.SetTunings(settings.Kp, settings.Ki, settings.Kd, settings.pOnE);
      }
      break;
    case 12:
      {
        int value = limitValue(svalue0.toInt(), 0, 32767);
        settings.Kd = (double) value / 100.0;
        pid.SetTunings(settings.Kp, settings.Ki, settings.Kd, settings.pOnE);
      }
      break;
    case 13:
      {
        int value = limitValue(svalue0.toInt(), 0, 1);
        settings.pOnE = value;
        pid.SetTunings(settings.Kp, settings.Ki, settings.Kd, settings.pOnE);
      }
      break;
    case 14:
      {
        int value = limitValue(svalue0.toInt(), 0, 1);
        settings.controllerDirection = value;
        pid.SetControllerDirection(settings.controllerDirection);
      }
      break;
    case 15:
      {
        int value = limitValue(svalue0.toInt(), 0, 3);
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
      break;
    case 16:
      {
        int value = limitValue(svalue0.toInt(), 0, 1);
        settings.relayHigh = value;
      }
      break;
    case 17:
      {
        int value = limitValue(svalue0.toInt(), 0, 32767);
        settings.windowSize = value;
        if (settings.pidMode == 0)
        {
          pid.SetOutputLimits(0, 255);
        }
        else
        {
          pid.SetOutputLimits(0, settings.windowSize);
        }
      }
      break;
    case 18:
      {
        int value = limitValue(svalue0.toInt(), 50, 32767);
        settings.sampleTime = value;
        pid.SetSampleTime(settings.sampleTime);
        tuner.setLoopInterval(settings.sampleTime * 1000);
      }
      break;


    case 20:
      {
        int pin = svalue0.toInt();
        invalidPin = checkAnalogPin(pin) ? 0 : 1;
        if (invalidPin == 0)
        {
          settings.pinInput = pin;
          pinMode(pin, INPUT);
        }
      }
      break;
    case 21:
      {
        int pin = svalue0.toInt();
        invalidPin = checkAnalogPin(pin) ? 0 : 1;
        if (invalidPin == 0)
        {
          settings.pinSetpoint = pin;
          pinMode(pin, INPUT);
        }
      }
      break;
    case 22:
      {
        int pin = svalue0.toInt();
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
      break;
    case 23:
      {
        int value = limitValue(svalue0.toInt(), 0, 1);
        settings.inputAsSetpoint = value;
      }
      break;
    case 24:
      {
        int pin = svalue0.toInt();
        invalidPin = checkDigitalPin(pin) ? 0 : 1;
        if (invalidPin == 0)
        {
          int value = limitValue(svalue1.toInt(), 0, 1);
          pinMode(pin, OUTPUT);
          digitalWrite(pin, value);
        }
      }
      break;
    case 25:
      {
        int value = limitValue(svalue0.toInt(), 0, 1023);
        Setpoint = (double) value;
      }
      break;
    case 26:
      {
        int pinH = svalue0.toInt();
        int pinL = svalue1.toInt();
        invalidPin = (checkPWMPin(pinH) && checkPWMPin(pinL)) ? 0 : 1;
        if (invalidPin == 0)
        {
          settings.pinOutputPWM16H = pinH;
          settings.pinOutputPWM16L = pinL;
          pinMode(pinH, OUTPUT);
          pinMode(pinL, OUTPUT);
        }
      }
      break;

    case 31:
      {
        int pin = svalue0.toInt();
        int percent = svalue1.toInt();
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
      break;
    case 32:
      {
        int pin = svalue0.toInt();
        int percent = svalue1.toInt();
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
      break;
    case 33:
      {
        int pin = svalue0.toInt();
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
      break;
    case 34:
      {
        int value = limitValue(svalue0.toInt(), 0, 1023);
        targetInputValue = value;
        tuner.setTargetInputValue(targetInputValue);
      }
      break;
    case 35:
      {
        unsigned int value = limitUintValue(svalue0.toInt(), 0, 65535);
        outputRange = value;
        tuner.setOutputRange(0, outputRange);
      }
      break;
    case 36:
      {
        int value = limitValue(svalue0.toInt(), 0, 2);
        znMode = (PIDAutotuner::ZNMode) value;
        tuner.setZNMode(znMode);
      }
      break;
    case 37:
      {
        int value = limitValue(svalue0.toInt(), 3, 255);
        tuningCycles = value;
        tuner.setTuningCycles(value);
      }
      break;

    case 41:
      {
        pidEnabled = 1;
        displayTitle = "INFO";
        displayStr = "PID ENABLED";
        displayInput = 1;
      }
      break;
    case 42:
      {
        pidEnabled = 0;
        displayTitle = "INFO";
        displayStr = "PID DISABLED";
        displayInput = 1;
        Output = 0;
        disableAllOutputs();
      }
      break;
    case 43:
      {
        if (pidEnabled == 0)
        {
          tunerRunning = 1;
          printFilledStr("PID TUNING", 0);
          printFilledStr("PLEASE WAIT...", 1);
        }
        else
        {
          displayTitle = "ERROR";
          displayStr = "PID IS ENABLED";
          displayInput = 1;
        }
      }
      break;

    case 51:
      {
        EEPROM.put(0, settings);
        displayTitle = "SAVE SETTINGS";
        displayStr = "OK";
        displayInput = 1;
      }
      break;
    case 52:
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
        displayInput = 1;
      }
      break;
  }
}

void disableAllOutputs()
{
  int valoff = (settings.pidMode == 0) ? 0 : (1 - settings.relayHigh);
  digitalWrite(settings.pinOutput, valoff);
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
  if (settings.pidMode > 2)
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
    case3:
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
  tuner.setLoopInterval(settings.sampleTime * 1000);

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
        int outInt = (int) Output;
        int outH = (outInt >> 8) & 0xFF;
        int outL = outInt & 0xFF;
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
        }
        else
        {
          digitalWrite(settings.pinOutput, settings.relayHigh);
          if (settings.pinSSRActive != DISABLED_LED_PIN) digitalWrite(settings.pinSSRActive, HIGH);
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

void doTuning()
{
  loopInterval = settings.sampleTime;
  unsigned long milliseconds = millis();
  char key = keypad.getKey();

  tuner.startTuningLoop(micros());

  while (!tuner.isFinished() && !key)
  {
    unsigned long microseconds = micros();
    milliseconds = millis();
    Input = analogRead(settings.pinInput);
    Output = tuner.tunePID(Input, microseconds);
    switch (settings.pidMode)
    {
      case 0: analogWrite(settings.pinOutput, Output); break;
      case 1: analogWrite16(settings.pinOutput, Output); break;
    }

    while (millis() - milliseconds < loopInterval) delay(1);
    key = keypad.getKey();
  }

  switch (settings.pidMode)
  {
    case 0: analogWrite(settings.pinOutput, 0); break;
    case 1: analogWrite16(settings.pinOutput, 0); break;
  }

  Kp = tuner.getKp();
  Ki = tuner.getKi();
  Kd = tuner.getKd();
  settings.Kp = Kp;
  settings.Ki = Ki;
  settings.Kd = Kd;
  pid.SetTunings(settings.Kp, settings.Ki, settings.Kd, settings.pOnE);
  tunerRunning = 0;

  displayTitle = "PID TUNING";
  displayStr = "FINISHED";
  displayInput = 1;

}

void printFilledStr(String s, int row)
{
  lcd.setCursor(0, row);
  lcd.print(filledStr(s, 16));
}

int limitValue(int value, int lo, int hi)
{
  return (value > hi) ? hi : ((value < lo) ? lo : value);
}

unsigned int limitUintValue(unsigned value, unsigned int lo, unsigned int hi)
{
  return (value > hi) ? hi : ((value < lo) ? lo : value);
}

void setupPWM16()
{
  TCCR1A = (TCCR1A & B00111100) | B10000010;
  TCCR1B = (TCCR1B & B11100000) | B00010001;
  ICR1 = 0xFFFF;
}

void analogWrite16(int pin, uint16_t value)
{
  switch (pin)
  {
    case 9: OCR1A = value; break;
    case 10: OCR1B = value; break;
  }
}
