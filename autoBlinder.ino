#include <EEPROM.h>
#include <avr/sleep.h>

// Connected stepper motor count
#define MOTORS 2

// Pin config
byte stepperClockPins[4] = {2, 3, 4, 5};
byte steppers[MOTORS] = {6, 7};
byte buttonPin = 3; //analog
byte stepperSelPin = 1; //analog CURRENTLY NOT USED
byte stepperSelPins[MOTORS] = {9, 8};
byte calPin = 12; 
byte indicatorLED = 11;
byte powerLED = 10;

// Other config
byte stepsAtOnce = 10;
float analogResolution = 0.1; //volts
float analogStepperRes = 0.1; //volts
unsigned short calibrationStartDelay = 3000; //milliseconds
unsigned short stepperClockDelay = 2; //ms; TEMPORARY: SET BACK TO 2 WHEN USING FULLY POWERED MOTORS
unsigned short ledVoltage = 102; // 255 = 5V

// =!= Do not change the variables below this line =!=
int curPos[MOTORS], maxPos[MOTORS], target[MOTORS];
bool sel[MOTORS], cal[MOTORS]; //currently selected (alt "bitRead(PORTD, <pin>)"); currently calibrating
float analogModifier = 1 / (1024 / 5 * analogResolution);
float analogStepperMod = 1 / (1024 / 5 * analogStepperRes);
byte MEMORYOFFSET = 32;
unsigned long lastAction = 0;


void load() { // 6 bytes per motor
  //current: 0: +/-; 1: 255c 2: c
  //max    : 3: +/-; 4: 255m 5: m   NOTE: is always positive
  if (EEPROM.read(MEMORYOFFSET) != 127) { // memory initialization
    for (int i = 0; i < EEPROM.length(); i++) {
      EEPROM.write(i, 0);
    }
    EEPROM.write(MEMORYOFFSET, 127);
  }
  for (int i = 0; i < MOTORS; i++) {
    curPos[i] = EEPROM.read(MEMORYOFFSET + i*6 + 2) * 255 + EEPROM.read(MEMORYOFFSET + i*6 + 3);
    if (EEPROM.read(MEMORYOFFSET + i*6 + 1) == 1) {
      curPos[i] = -curPos[i];
    }
    maxPos[i] = EEPROM.read(MEMORYOFFSET + i*6 + 5) * 255 + EEPROM.read(MEMORYOFFSET + i*6 + 6);
    if (EEPROM.read(MEMORYOFFSET + i*6 + 4) == 1) {
      maxPos[i] = -maxPos[i];
    }
  }
}

void _writePos(int address, int value) {
  if (value < 0) {
    value = -value;
    EEPROM.write(address, 1);
  } else {
    EEPROM.write(address, 0);
  }
  byte factor = value/255;
  EEPROM.write(address + 1, factor);
  EEPROM.write(address + 2, value - factor*255);
}

void save(bool saveMax = false) {
  for (int i = 0; i < MOTORS; i++) {
    if (cal[i]) { //skip currently calibrating motors - possible source of corrupt data if a power cut happens during calibration
      continue;
    }
    _writePos(MEMORYOFFSET + i*6 + 1, curPos[i]);
    if (saveMax) {
      _writePos(MEMORYOFFSET + i*6 + 3 + 1, maxPos[i]);
    }
  }
}

void setup() {  
  Serial.begin(9600);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
  sleep_enable();
  analogReference(DEFAULT);

  // calibration pin is in reverse mode
  pinMode(calPin, INPUT);
  digitalWrite(calPin, HIGH);
  
  // pin initialization
  for (char i = 0; i < sizeof(stepperClockPins); i++) {
    pinMode(stepperClockPins[i], OUTPUT);
  }
  for (char i = 0; i < sizeof(steppers); i++) {
    pinMode(steppers[i], OUTPUT);
  }
  pinMode(indicatorLED, OUTPUT);
  pinMode(powerLED, OUTPUT);
  analogWrite(powerLED, 1);

  load();
  
  for (int i = 0; i < MOTORS; i++) { //set all "cal" values to false and set all targets to current position
    cal[i] = false;
    target[i] = curPos[i];
  }
}

int readAnalog(int pin) {
  if (pin == buttonPin) {
    return round(analogRead(pin) * analogModifier);
  } else if (pin == stepperSelPin) {
    return round(analogRead(pin) * analogStepperMod);
  } else {
    return analogRead(pin);
  }
}

/*void _setSelected(){ // Determines which motors are selected and sets values in "sel" array
  int selVal = readAnalog(stepperSelPin);
  if (selVal == 2) {
    sel[0] = 0;
    sel[1] = 1;
  } else if (selVal == 3) {
    sel[0] = 1;
    sel[1] = 0;
  } else {
    sel[0] = 1;
    sel[1] = 1;
  }
}*/

void _setSelected() { 
  if (digitalRead(stepperSelPins[0]) == 1 && digitalRead(stepperSelPins[1]) == 1) {
    sel[0] = 1;
    sel[1] = 1;
  } else {
    for (int i = 0; i < MOTORS; i++) {
      if (digitalRead(stepperSelPins[i]) == 0) {
        sel[i] = 1;
      } else {
        sel[i] = 0;
      }
    }
  }
}

void _setTarget(int steps = 0, byte percent = 0) { // Sets motor targets from arguments; if steps is 0, percent is used
  for (int i = 0; i < MOTORS; i++) {
    if (sel[i]) {
      if (steps != 0) { //steps are set
        if (cal[i]) { //ignore restrictions
          target[i] = curPos[i] + steps;
        } else {
          target[i] = max(0, min(maxPos[i], curPos[i] + steps));
        }
      } else { //using percentage
        if (!cal[i]) { //this would otherwise ruin the calibration
          target[i] = (long)maxPos[i] * percent / 100;
        }
      }
    }
  }
}

int _setPinsAndGetMinSteps() { // Determines which motors to activate and how many steps to take based on their targets
  int minSteps = 0;
  for (byte i = 0; i < MOTORS; i++) {
    if (sel[i]) {
      int steps = target[i] - curPos[i];
      if (minSteps == 0) { // the rotation direction is based on which way the first motor needs to turn
        minSteps = steps;
      }
      if (steps > 0) {
        if (minSteps > 0) {
          minSteps = min(minSteps, steps);
          digitalWrite(steppers[i], 1);
          continue;
        }
      } else if (steps < 0) {
        if (minSteps < 0) {
          minSteps = max(minSteps, steps);
          digitalWrite(steppers[i], 1);
          continue;
        }
      }
      digitalWrite(steppers[i], 0);
    } else {
      digitalWrite(steppers[i], 0);
    }
  }
  return minSteps;
}

void runSteppers() {
  int steps = _setPinsAndGetMinSteps();
  if (steps != 0) {
    analogWrite(indicatorLED, 255);
  }
  bool reverse = false;
  if (steps < 0) {
    reverse = true;
    steps = -steps;
  }
  for (int i = 0; i < steps; i++) { // run clock
    for (byte p = 0; p < 4; p++) {
      byte pin = p;
      if (reverse) {
        pin = 3-p;
      }
      digitalWrite(stepperClockPins[pin], 1);
      delay(stepperClockDelay);
      digitalWrite(stepperClockPins[pin], 0);
      //a delay could be added here if necessary
    }
  }
  for (byte i = 0; i < MOTORS; i++) { // save changes
    if (bitRead(PORTD, steppers[i])) { // for motors that are activated
      if (reverse) {
        curPos[i] -= steps;
      } else {
        curPos[i] += steps;
      }
    }
  }
  //save(); why? WHY? WHY EVERY TIME??? eeprom ruined
  if (steps != 0) {
    lastAction = millis();
  }
  analogWrite(indicatorLED, 0);
}

void loop() {  
  _setSelected(); // determine which motors are currently selected
  
  bool calibrationMode = false;
  for (int i = 0; i < MOTORS; i++) { //check if any of the steppers is already in calibration mode
    /*Serial.print(i);
    Serial.print(": ");
    Serial.print(curPos[i]);
    Serial.print(", ");
    Serial.print(target[i]);
    Serial.print(", ");
    Serial.print(maxPos[i]);
    Serial.print("; ");*/
    if (cal[i]) {
      calibrationMode = true;
      //break; TEMPORARY, FOR LOGGING
    }
  }
  //Serial.println();

  if (!digitalRead(calPin)) { // parse calibration button input
    if (!calibrationMode) { //try to initiate calibration mode
      unsigned long startTime = millis();
      while (!digitalRead(calPin)) { //calibration button must be held for the specified time
        delay(10);
        if (millis() - startTime >= calibrationStartDelay) { //notify user that calibration mode is (actually: will be) activated
          //Serial.println("CALIBRATION MODE READY\n");
          analogWrite(powerLED, 128);
        }
      }
      if (millis() - startTime >= calibrationStartDelay) { //initiate calibration mode
        //Serial.println("STARTED CALIBRATION");
        for (int i = 0; i < MOTORS; i++) {
          if (sel[i]) {
            cal[i] = true;
            curPos[i] = 0;
            target[i] = 0;
          }
        }
        calibrationMode = true;
        analogWrite(powerLED, 128); //just to be sure
      }
    } else { //end calibration
      for (int i = 0; i < MOTORS; i++) {
        if (cal[i] == true) {
          cal[i] = false;
          if (curPos[i] < 0) { //ensure maxPos is always positive
            maxPos[i] = -curPos[i];
            curPos[i] = 0;
            target[i] = 0;
          } else {
            maxPos[i] = curPos[i];
          }
        }
      }
      save(true);
      calibrationMode = false;
      analogWrite(powerLED, 1);
    }
  }

  // set motor rotation targets from input
  unsigned short buttVal = readAnalog(buttonPin);
  /*Serial.print(digitalRead(8));
  Serial.print(digitalRead(9));
  Serial.print("Button pressed: ");
  Serial.print(buttVal);
  Serial.print("; Motors selected: ");
  if (sel[0]) {
    Serial.print(0);
  }
  if (sel[1]) {
    Serial.print(1);
  }
  Serial.println();
  for (int i = 0; i < 50; i++) {
    Serial.print(EEPROM.read(i));
    Serial.print("; ");
  }
  Serial.println();*/
  if (buttVal == 2) { // forward
    _setTarget(stepsAtOnce, 0);
  } else if (buttVal == 1) { // backward
    _setTarget(-stepsAtOnce, 0);
  } else if (!calibrationMode) { // predefined percentages 
    if (buttVal == 10) { 
      _setTarget(0, 25); 
    } else if (buttVal == 11) { 
      _setTarget(0, 50);
    } else if (buttVal == 12) { 
      _setTarget(0, 75);
    } else if (buttVal == 13) { 
      _setTarget(0, 0);
    } else if (buttVal == 14) { 
      _setTarget(0, 100);
    }
  }
  
  runSteppers(); 
  if (lastAction != 0 && millis() - lastAction > 100) {
    save();
    lastAction = 0;
  }
  /*Serial.println();  
  if (bitRead(PORTD, 6)) {
    Serial.print(0);
  }
  if (bitRead(PORTD, 7)){
    Serial.print(1);
  }
  Serial.println();*/
}

