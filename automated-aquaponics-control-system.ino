/*  Automated Aquaponics Control System
 *  Version 0.1.0
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

/*----- TIMER -----*/

// The outputs array defines how long each output will
// be turned off, on, and what pin to use for that output.
// The off and on values are in units of 'ticks'. The length
// of a tick is controlled by the setup of MsTimer2.
                              // off  on pin
byte outputs[OUTPUT_COUNT][3] = {{ 1,  1,  6},   // Output A - valve timer for grow bed ON/OFF cycle
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

/*----- END TIMER -----*/

/*----- SWITCH -----*/

// Input pins for switching running mode
int inPinRun  = 12; 
int inPinAuto = 11;
int inPinStop = 10;
// Output pins for running mode LEDs
int outPinRun  = 4;
int outPinAuto = 3;
int outPinStop = 2;

// Putput pins for relay control
int outPinPumpRelay = 6; // TODO: Not used yet

int state    = HIGH;   // the current state of the output pin
int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin

// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time     = 0;     // the last time the output pin was toggled
long debounce = 200;   // the debounce time (miliseconds), increase if the output flickers

/*----- END SWITCH -----*/

void setup() {

  Serial.begin(9600);
  while (!Serial);
  Serial.println("Automated Aquaponics Control System");
  Serial.println("Version: " + ver);
  Serial.println("Repo URL: " + repo_url);
  Serial.println();
  Serial.println("Starting...");

  // Disable onboard L LED
  pinMode(13, OUTPUT);
  digitalWrite(outPinStop, LOW);

  // Set up and init all outputs to off
  for(byte i = 0; i < OUTPUT_COUNT; i++){
    //pinMode(outputs[i][OUTPUT_PIN], OUTPUT);
    digitalWrite(outputs[i][OUTPUT_PIN], LOW);

    // Set up an event fuse for this output.
    EventFuse::newFuse(i, outputs[i][OFF_TIME], INF_REPEAT, OutputHandler);
  }

  // Set pin modes
  pinMode(inPinRun, INPUT);
  pinMode(inPinAuto, INPUT);
  pinMode(inPinStop, INPUT);

  pinMode(outPinRun, OUTPUT);
  pinMode(outPinAuto, OUTPUT);
  pinMode(outPinStop, OUTPUT);

  // Run lamp test to test all LEDs on controll panel
  Serial.println("Lamp test...");
  digitalWrite(outPinStop, HIGH);
  digitalWrite(outPinAuto, HIGH);
  digitalWrite(outPinRun, HIGH);
  delay(2000);
  digitalWrite(outPinStop, LOW);
  digitalWrite(outPinAuto, LOW);
  digitalWrite(outPinRun, LOW);
  delay(500);
  Serial.println("Lamp test complete.");

  // Set MsTimer2 for one second per tick (in milliseconds,
  MsTimer2::set(1000, timerTick);

  Serial.println("System Ready.");
  Serial.println();

  // Read last running mode from EEPROM (if one exists)
  if (EEPROM.read(0) == 1){
    Serial.println("Resuming last run mode: Run.");
    pumpRun();
  } else if (EEPROM.read(0) == 2){
    Serial.println("Resuming last run mode: Auto.");
    pumpAuto();
  } else if (EEPROM.read(0) == 3){
    Serial.println("Resuming last run mode: Stop.");
    pumpStop();
  } else {
    // If no previous running mode found, remain stopped
    Serial.println("Unknown last run mode. Defaulting to run mode: Stop.");
    pumpStop();
  }
}

void loop(){
  // Running modes
  // 1 = Run
  // 2 = Automatic
  // 3 = Stopped

  // Check if any running mode control buttons are pressed,
  // if so, change 'reading' to respective running mode int
  if (digitalRead(inPinRun) == HIGH){
    reading = 1;
  } else if (digitalRead(inPinAuto) == HIGH){
    reading = 2;
  } else if (digitalRead(inPinStop) == HIGH){
    reading = 3;
  } else {
    // if no button pressed, set 'reading' to 0
    reading = 0;
  }

  //Serial.println(reading);

  // if the input just went from LOW and HIGH and we've waited long enough
  // to ignore any noise on the circuit, toggle the output pin and remember
  // the time

  // Check for button state changed
  if (reading == 1 && previous != 1 && millis() - time > debounce) {
    pumpRun();

    time = millis();  
  }
  if (reading == 2 && previous != 2 && millis() - time > debounce) {
    pumpAuto();

    time = millis();  
  }
  if (reading == 3 && previous != 3 && millis() - time > debounce) {
    pumpStop();

    time = millis();  
  }
}

// RUN mode method
void pumpRun(){
  // Update control panel LEDs to confirm running mode change
  digitalWrite(outPinRun, HIGH);
  digitalWrite(outPinAuto, LOW);
  digitalWrite(outPinStop, LOW);

  // Write running mode change to EEPROM so it can be recovered on next boot
  EEPROM.update(0, 1);
  previous = 1;

  // Stop MsTimer
  MsTimer2::stop();
  Serial.println("Pump run mode changed to RUN!");
}

// AUTO mode method
void pumpAuto(){
  // Update control panel LEDs to confirm running mode change
  digitalWrite(outPinAuto, HIGH);
  digitalWrite(outPinRun, LOW);
  digitalWrite(outPinStop, LOW);

  // Write running mode change to EEPROM so it can be recovered on next boot
  EEPROM.update(0, 2);
  previous = 2;

  // Start MsTimer
  MsTimer2::start();
  Serial.println("Pump run mode changed to AUTO!");
}

// STOP mode method
void pumpStop(){
  // Update control panel LEDs to confirm running mode change
  digitalWrite(outPinStop, HIGH);
  digitalWrite(outPinAuto, LOW);
  digitalWrite(outPinRun, LOW);

  // Write running mode change to EEPROM so it can be recovered on next boot
  EEPROM.update(0, 3);
  previous = 3;

  // Stop MsTimer
  MsTimer2::stop();
  Serial.println("Pump run mode changed to STOP!");
}
