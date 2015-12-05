/*Balcony_Timer_Ver1.0.0
 *  05.30.2015 -  versions made by rik kretzinger -- AGponic, INC.
 *
 * Description:
 * This Sketch turns the growbed drain valve on and off at specific time intervals. 
 * Multiple independent switched events can be configured. Each
 * output can be configured with independent
 * on and off durations with a minimum of 1 second (1000)
 * and a maximum of about 18 hours (2^16 seconds).
 * 1 second(1000)-1 minute (60000)- 1 hour (3,600,000)
 *
 */
// Timer function the requires libaries -- Balcony_Timer_V1.00
#include <EventFuse.h>
#include <MsTimer2.h>

#include <EEPROM.h>

// Timer #define statements from --  Balcony_Timer_V1.00 by rjk
#define OutputCount 4
 
#define OffTime 0
#define OnTime 1
#define OutputPin 2

String ver = "0.1.5";

/*----- TIMER -----*/

// document when added and from where here  (This section comes from Balcony_Timer_V1.00 by rjk)
// The outputs array defines how long each output - A B C D will
// be turned off, on, and what pin to use for which outputs.
// The off and on values are in units of 'ticks'. The length
// of a tick is controlled by the setup of MsTimer2.  1 second(1000) - 1 minute (60000) - 1 hour (3,600,000)
// changes are made to this line of code  MsTimer2::set(60000, timerTick ); below
                             // off   on  pin
byte outputs[OutputCount][3] = {{ 1,  1,  12},   // valve timer for grow bed ON/OFF cycle -- ON provides power to Valve
                                { 0,  0,  0},   // Output B - turned OFF until needed
                                { 0,  0,  0},   // Output C - turned OFF until needed
                                { 0,  0,  0},}; // Output D - turned OFF until needed
                               
                               // at start-up of sketch relay is in OFF mode for 
                               // period of time of the OFF time for Output A (valve)

// Taken from Balcony_Timer_V1.0.0                               
void OutputHandler(FuseID fuseID, int outputID){
  // look up the pin associated with this output
  byte pin = outputs[outputID][OutputPin];

  // get and invert the current pin state and write
  // it back to the port to invert the current pin state.
  int state = 1&~digitalRead(pin);
  digitalWrite( pin, state );

  // Reset the fuse length with a new interval. The current state
  // of the pin is used to determine which interval should be used.
  eventFuse[fuseID].fuseLen = outputs[outputID][state];
}

// Added from Balcony_Timer_V1.0.0
void timerTick(){
  eventFuse.burn(1);
}

/*----- END TIMER -----*/

/*----- SWITCH -----*/

int inPinRun = 4; 
int inPinAuto = 3;
int inPinStop = 2;         // the number of the input pin
int outPinRun = 13;
int outPinAuto = 12;
int outPinStop = 11;       // the number of the output pin

int state = HIGH;      // the current state of the output pin
int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin

// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounce = 200;   // the debounce time (miliseconds), increase if the output flickers

/*----- END SWITCH -----*/

void setup() {

  Serial.begin(9600);
  while (!Serial);
  Serial.println("Automated Aquaponics Control System");
  Serial.println("Version: " + ver);
  Serial.println();
  Serial.println("Starting...");
  
  
  // Taken from Balcony_Timer_V1.0.0
  // Set up and init all outputs to off
  for(byte i = 0; i<OutputCount; i++){
    //pinMode( outputs[i][OutputPin], OUTPUT);
    digitalWrite( outputs[i][OutputPin], LOW );  // Set to HIGH and valve will be ON at time of Sketch start
    
    // Set up an event fuse for this output.
    eventFuse.newFuse( i, outputs[i][OffTime], INF_REPEAT, OutputHandler );
  }
  pinMode(inPinRun, INPUT);
  pinMode(inPinAuto, INPUT);
  pinMode(inPinStop, INPUT);
  
  pinMode(outPinRun, OUTPUT);
  pinMode(outPinAuto, OUTPUT);
  pinMode(outPinStop, OUTPUT);
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
  // Set MsTimer2 for one mintue per tick.
  MsTimer2::set(1000, timerTick );  // 1 minute time frame per Tick
  //MsTimer2::start();
  Serial.println("Started.");
  Serial.println();
  //pumpAuto();

  if (EEPROM.read(0) == 1){
    Serial.println("Resuming last run mode: Run.");
    pumpRun();
  }
  else if (EEPROM.read(0) == 2){
    Serial.println("Resuming last run mode: Auto.");
    pumpAuto();
  }
  else if (EEPROM.read(0) == 3){
    Serial.println("Resuming last run mode: Stop.");
    pumpStop();
  }
  else {
    Serial.println("Last run mode: Unknown!");
  }

}

void loop(){
  //1 = Run
  //2 = Automatic
  //3 = Stopped
  
  //reading = digitalRead(inPinStop);
  if (digitalRead(inPinRun) == HIGH){
    reading = 1;
  } 
  else if (digitalRead(inPinAuto) == HIGH){
    reading = 2;
  }
  else if (digitalRead(inPinStop) == HIGH){
    reading = 3;
  }
  else {
    reading = 0;
  }
  
  //Serial.println(reading);
  // if the input just went from LOW and HIGH and we've waited long enough
  // to ignore any noise on the circuit, toggle the output pin and remember
  // the time
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
  
  
  //digitalWrite(outPinRun, state);

  //previous = reading;
  //delay(20);
}

void pumpRun(){
  digitalWrite(outPinRun, HIGH);
  digitalWrite(outPinAuto, LOW);
  digitalWrite(outPinStop, LOW);

  EEPROM.update(0, 1);
  previous = 1;

  MsTimer2::stop();
  Serial.println("Pump run mode changed to RUN!");
}

void pumpAuto(){
  digitalWrite(outPinAuto, HIGH);
  digitalWrite(outPinRun, LOW);
  digitalWrite(outPinStop, LOW);

  EEPROM.update(0, 2);
  previous = 2;

  MsTimer2::start();
  Serial.println("Pump run mode changed to AUTO!");
}

void pumpStop(){
  digitalWrite(outPinStop, HIGH);
  digitalWrite(outPinAuto, LOW);
  digitalWrite(outPinRun, LOW);

  EEPROM.update(0, 3);
  previous = 3;

  MsTimer2::stop();
  Serial.println("Pump run mode changed to STOP!");
}

