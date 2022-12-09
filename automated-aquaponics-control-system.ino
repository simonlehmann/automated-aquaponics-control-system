/**
 * Automated Aquaponics Control System
 * Version 0.1.0
 *
 * This Sketch controlls the timing for an automated aquaponics system.
 * Copyright (C) 2015 Simon Lehmann
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Incude Libraries for Timer
#include <EventFuse.h> // Install with .zip from https://github.com/davidknaack/EventFuse
#include <MsTimer2.h> // Install from Arduino IDE Library Manager

// Include libary for Isolated Storage
#include <EEPROM.h>

// Timer #define statements
#define OUTPUT_COUNT 4
#define OFF_TIME 0
#define ON_TIME 1
#define OUTPUT_PIN 2

String ver      = "0.1.0";
String repo_url = "https://github.com/simonlehmann/automated-aquaponics-control-system";

/*=================*/
/*      TIMER      */
/*=================*/

// The outputs array defines how long each output will
// be turned off, on, and what pin to use for that output.
// The off and on values are in units of 'ticks'. The length
// of a tick is controlled by the setup of MsTimer2.
                              // off  on pin
byte outputs[OUTPUT_COUNT][3] = {{ 1,  1,  5},   // Output A - valve timer for grow bed ON/OFF cycle
                                 { 0,  0,  0},   // Output B - vacant
                                 { 0,  0,  0},   // Output C - vacant
                                 { 0,  0,  0}};  // Output D - vacant
                               
                                // at start-up of sketch relay is in OFF mode for 
                                // period of time of the OFF time for Output A (valve)

void OutputHandler(FuseID fuseID, int& outputID){
  // look up the pin associated with this output
  byte pin = outputs[outputID][OUTPUT_PIN];

  // get and invert the current pin state and write
  // it back to the port to invert the current pin state.
  int state = 1&~digitalRead(pin);
  digitalWrite(pin, state);

  // Reset the fuse length with a new interval. The current state
  // of the pin is used to determine which interval should be used.
  EventFuse::fuses[fuseID].fuseLen = outputs[outputID][state];
}

void timerTick(){
  EventFuse::burn();
}

/*==================*/
/*      SWITCH      */
/*==================*/

// Input pins for switching running mode
int inPinModeAuto     = 12;
int inPinModeDisabled = 11;

// Output pins for running mode LEDs
int outPinModeAuto     = 3;
int outPinModeDisabled = 2;

// Putput pins for relay control
int outPinPumpRelay = 5; // TODO: Only partially implemented

int state    = HIGH;   // the current state of the output pin
int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin

// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time     = 0;     // the last time the output pin was toggled
long debounce = 200;   // the debounce time (miliseconds), increase if the output flickers

/*==================*/
/*      SET-UP      */
/*==================*/

void setup() {

  Serial.begin(9600);
  while (!Serial);
  Serial.println("Automated Aquaponics Control System");
  Serial.println("Version: " + ver);
  Serial.println("Repo URL: " + repo_url);
  Serial.println();
  Serial.println("Starting...");

  // Extinguish onboard "L" LED
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // Set up and init all outputs to off
  for(byte i = 0; i < OUTPUT_COUNT; i++){
    //pinMode(outputs[i][OUTPUT_PIN], OUTPUT);
    digitalWrite(outputs[i][OUTPUT_PIN], LOW);

    // Set up an event fuse for this output.
    EventFuse::newFuse(i, outputs[i][OFF_TIME], INF_REPEAT, OutputHandler);
  }

  // Set pin modes
  pinMode(inPinModeAuto,      INPUT);
  pinMode(inPinModeDisabled,  INPUT);
  pinMode(outPinModeAuto,     OUTPUT);
  pinMode(outPinModeDisabled, OUTPUT);

  // Run lamp test to test all LEDs on controll panel
  Serial.println("Lamp test starting...");
  digitalWrite(outPinModeAuto,     HIGH);
  digitalWrite(outPinModeDisabled, HIGH);
  delay(2000);
  digitalWrite(outPinModeAuto,     LOW);
  digitalWrite(outPinModeDisabled, LOW);
  delay(500);
  Serial.println("Lamp test complete.");

  // Set MsTimer2 for one second per tick (in milliseconds,
  MsTimer2::set(1000, timerTick);

  Serial.println("System Ready.");
  Serial.println();

  // Read last running mode from EEPROM (if one exists)
  if (EEPROM.read(0) == 2){
    Serial.println("Resuming last run mode: AUTO.");
    modeAuto();
  } else if (EEPROM.read(0) == 2){
    Serial.println("Resuming last run mode: DISABLED.");
    modeDisabled();
  } else {
    // If no previous running mode found, remain stopped
    Serial.println("Unknown last run mode. Defaulting to run mode: DISABLED.");
    modeDisabled();
  }
}

/*================*/
/*      MAIN      */
/*================*/

void loop(){
  // Running modes
  // 1 = Auto
  // 2 = Disabled

  // Check if any running mode control buttons are pressed,
  // if so, change 'reading' to respective running mode int
  if (digitalRead(inPinModeAuto) == HIGH){
    reading = 1;
  } else if (digitalRead(inPinModeDisabled) == HIGH){
    reading = 2;
  } else {
    // if no button pressed, set 'reading' to 0
    reading = 0;
  }

  // if the input just went from LOW and HIGH and we've waited long enough
  // to ignore any noise on the circuit, toggle the output pin and remember
  // the time

  // Check for button state changed
  if (reading == 1 && previous != 1 && millis() - time > debounce) {
    modeAuto();

    time = millis();  
  }
  if (reading == 2 && previous != 2 && millis() - time > debounce) {
    modeDisabled();

    time = millis();  
  }
}

/*=========================*/
/*      RUNNING MODES      */
/*=========================*/

// AUTO mode method
void modeAuto(){
  // Update control panel LEDs to confirm running mode change
  digitalWrite(outPinModeAuto,     HIGH);
  digitalWrite(outPinModeDisabled, LOW);

  // Write running mode change to EEPROM so it can be recovered on next boot
  EEPROM.update(0, 1);
  previous = 1;

  // Start MsTimer
  MsTimer2::start();

  Serial.println("Running mode changed to AUTO.");
}

// DISABLED mode method
void modeDisabled(){
  // Update control panel LEDs to confirm running mode change
  digitalWrite(outPinModeAuto,     LOW);
  digitalWrite(outPinModeDisabled, HIGH);

  // Write running mode change to EEPROM so it can be recovered on next boot
  EEPROM.update(0, 2);
  previous = 2;

  // Stop MsTimer
  MsTimer2::stop();

  Serial.println("Running mode changed to DISABLED.");
}

/*=========================*/
/*      PUMP CONTROLS      */
/*=========================*/

void pumpRun(){
  Serial.println("Pump starting...");
  digitalWrite(outPinPumpRelay, HIGH);
}

void pumpStop(){
  Serial.println("Pump stopping...");
  digitalWrite(outPinPumpRelay, LOW);
}
