// 
// MME 4487 Pickup and Sorting Mechanism
// 
//  Language: Arduino (C++)
//  Target:   ESP32
//  Author:   Wesley Guo
//  Date:     2023 11 25
//

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"

// Function declarations
void doHeartbeat();
long degreesToDutyCycle(int deg);
void ARDUINO_ISR_ATTR buttonISR(void* arg);

// Button structure
struct Button {
  const int pin;                                      // GPIO pin for button
  unsigned int numberPresses;                         // counter for number of button presses
  unsigned int lastPressTime;                         // time of last button press in ms
  bool pressed;                                       // flag for button press event
  bool state;                                         // current state of button; 0 = pressed; 1 = unpressed
  bool lastState;                                     // last state of button
};

// Constants
const int ci_HeartbeatLED = 2;                       // GPIO pin of built-in LED for heartbeat
const int ci_HeartbeatInterval = 500;                // heartbeat blink interval, in milliseconds
const int ci_ServoPin = 18;                          // GPIO pin for servo motor ARM
const int ci_Servo1Pin = 16;                         // GPIO pin for servo motor GRIP
const int ci_ServoArm = 5;                           // PWM channel used for the RC servo motor
const int ci_ServoGrip = 4;                          // PWM channel used for the RC servo motor
const int cTCSLED = 23;                              // GPIO pin for LED on TCS34725
const long cDebounceDelay = 20;                      // button debounce delay in milliseconds
const long ul_ArmDelay = 7;                         // servo arm motor ms change limiter for non-instant transition
const long ul_GripDelay = 3;                         // grip motor delay

// Variables
boolean b_Heartbeat = true;                          // state of heartbeat LED
unsigned long ul_LastHeartbeat = 0;                  // time of last heartbeat state change
unsigned long ul_CurMillis = 0;                      // current time, in milliseconds
unsigned long ul_PrevMillis = 0;                     // start time for delay cycle, in milliseconds
unsigned int ui_State = 0;                           // current operating state
Button buttonScan = {13, 0, 0, false, true, true};   // NO pushbutton on GPIO 13, low state when pressed. Used to inititate sorting
bool pickup = false;                                 // boolean for whether pickup is in progress
bool badobj = false;                                 // bad object flag
bool goodobj = false;                                // good object flag
int totalAmb = 0;                                    // ambient light from colour sensor

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                     // TCS34725 flag: 1 = connected; 0 = not found

// Servo motor desired degree positions. Adjust as observed if necessary
int i_ServoArmStart = 180;
int i_ServoArmFinish = 0;
int i_ServoGripStart = 75;
int i_ServoGripFinish = 170;
int i_ServoArmPos = i_ServoArmStart;                                     
int i_ServoGripPos = i_ServoGripStart;

void setup() {

  Serial.begin(115200);                                                 // Standard baud rate for ESP32 serial monitor
  pinMode(ci_HeartbeatLED, OUTPUT);                                     // configure built-in LED for heartbeat
  ledcAttachPin(ci_ServoPin, ci_ServoArm);                              // assign servo pin to servo channel
  ledcSetup(ci_ServoArm, 50, 16);                                       // setup for channel for 50 Hz, 16-bit resolution
  ledcAttachPin(ci_Servo1Pin, ci_ServoGrip);                            // assign servo pin to servo channel
  ledcSetup(ci_ServoGrip, 50, 16);                                      // setup for channel for 50 Hz, 16-bit resolution
  pinMode(cTCSLED, OUTPUT);                                             // configure GPIO for control of LED on TCS34725
  pinMode(buttonScan.pin, INPUT_PULLUP);                                // configure GPIO for button for scan control
  attachInterruptArg(buttonScan.pin, buttonISR, &buttonScan, CHANGE);   // attach ISR to buttonScan

  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
    digitalWrite(cTCSLED, 1);                         // turn on onboard LED 
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }
}

void loop() {
  uint16_t r, g, b, c;                                // RGBC values from TCS34725
  // Calibration process. Determines the ambient light when the colour detector is not viewing any object. Only happens once.
  if (totalAmb == 0) {                                                      // only on the first iteration because totalAmb is initialized as 0
    ledcWrite(ci_ServoGrip, degreesToDutyCycle(i_ServoGripFinish));         // close the gripper
    i_ServoGripPos = i_ServoGripFinish;                                     // update position
    delay(1000);                                                            // allow some time for the gripper to settle
    tcs.getRawData(&r, &g, &b, &c);                                         // read R,G,B,C
    totalAmb = r + g + b;                                                   // totalAmb is summed from r,g,b
    ledcWrite(ci_ServoGrip, degreesToDutyCycle(i_ServoGripStart));          // open the gripper
    i_ServoGripPos = i_ServoGripStart;                                      // update position
  }

  // Sorting initialization process. Determines what the object is if there is one and returns information accordingly.
  if (!buttonScan.state && !pickup) {                                       // when button is pressed and it is not in a pickup process
    ledcWrite(ci_ServoGrip, degreesToDutyCycle(i_ServoGripFinish));         // close the gripper
    i_ServoGripPos = i_ServoGripFinish;                                     // update position
    delay(1000);                                                            // allow some time for the gripper to settle
    tcs.getRawData(&r, &g, &b, &c);                                         // read R,G,B,C
    int scanned = r + g + b;                                                // scanned is the summation of the read values
    if (abs(scanned - totalAmb) > 1) {                                     // if the difference between scanned and totalAmb is greater than 10, an object is considered to be present
      pickup = true;                                                        // set pickup flag to true
      if (scanned > 11) {                                                   // if scanned object summation is larger than 50 (the white rock), consider it as a bad object
        badobj = true;                                                      
      } else {                                                              // otherwise it is likely the green object as it has a low summation value
        goodobj = true;
      }
    }
    else {                                                                  // if little to no difference, open the gripper again
      ledcWrite(ci_ServoGrip, degreesToDutyCycle(i_ServoGripStart));
      i_ServoGripPos = i_ServoGripStart;
    }
    Serial.printf("Ambient: %d, Scanned: %d\n", totalAmb, scanned);         // Outputs information to serial monitor regarding ambient and scanned values
  }

  // Pickup process for a good object (the clear green rock)
  if (pickup && goodobj){                                                   // if pickup and goodobj are both true                  
    switch(ui_State) {                                                      // begin switch statemen
      case 0: {                                                             // case 0: close the gripper
        ul_CurMillis = millis();
        if (ul_CurMillis - ul_PrevMillis > ul_GripDelay){                   // uses ul_GripDelay to limit the servo rotation
          ul_PrevMillis = ul_CurMillis;
          if (i_ServoGripPos < i_ServoGripFinish){                          // if the position is less than the finishing point
            ledcWrite(ci_ServoGrip, degreesToDutyCycle(i_ServoGripPos));    // set the position of the motor
            i_ServoGripPos++;                                               // increment by 1 to allow for smooth transition
          }
          if (i_ServoGripPos >= i_ServoGripFinish){                         // when grip position is at the finish point (closed) switch to next state
            ui_State++;
          }
        }
        break;
      }
      case 1: {                                                             // case 1: rotate the arm
        ul_CurMillis = millis();
        if (ul_CurMillis - ul_PrevMillis > ul_ArmDelay){                    // uses ul_ArmDelay to limit the servo rotation
          ul_PrevMillis = ul_CurMillis;
          if (i_ServoArmPos > i_ServoArmFinish){                            // if arm position is greater than finishing position (starting position is larger)
            ledcWrite(ci_ServoArm, degreesToDutyCycle(i_ServoArmPos));      // set servo motor position
            i_ServoArmPos--;                                                // decrement by 1 to allow for smooth transition
          }
          if (i_ServoArmPos <= i_ServoArmFinish){                           // when arm position is at the finsh point switch to next state
            ui_State++;
          }
        }
        break;
      }
      case 2: {                                                             // case 2: open the gripper, drop the object
        ul_CurMillis = millis();
        if (ul_CurMillis - ul_PrevMillis > ul_GripDelay){                   // uses ul_GripDelay to limit the servo rotation
          ul_PrevMillis = ul_CurMillis;
          if (i_ServoGripPos > i_ServoGripStart){                           // if the position is greater than the finishing point
            ledcWrite(ci_ServoGrip, degreesToDutyCycle(i_ServoGripPos));    // set servo motor position
            i_ServoGripPos--;                                               // decrement by 1 to allow for smooth transition
          }
          if (i_ServoGripPos <= i_ServoGripStart){                          // when grip position is back at the start point (opened) switch to next state
            ui_State++;
          }
        }
        break;
      }
      case 3: {                                                             // case 3: rotate the arm back to starting position
        ul_CurMillis = millis();
        if (ul_CurMillis - ul_PrevMillis > ul_ArmDelay){                    // uses ul_ArmDelay to limit the servo rotation
          ul_PrevMillis = ul_CurMillis;
          if (i_ServoArmPos < i_ServoArmStart){                             // if the position is less than the finishing point
            ledcWrite(ci_ServoArm, degreesToDutyCycle(i_ServoArmPos));      // set servo motor position
            i_ServoArmPos++;                                                // increment by 1 to allow for smooth transition
          }
          if (i_ServoArmPos >= i_ServoArmStart){                            // when the arm is back at the starting position
            ui_State = 0;                                                   // reset ui_State and all flags
            pickup = false;
            badobj = false;
            goodobj = false;
          }
        }
        break;
      }

    }
  }
  // Pickup process for a bad object (white rock)
  if (pickup && badobj){
    switch(ui_State) {
      case 0: {                                                             // case 0: close the gripper, same as good object process
        ul_CurMillis = millis();
        if (ul_CurMillis - ul_PrevMillis > ul_GripDelay){
          ul_PrevMillis = ul_CurMillis;
          if (i_ServoGripPos < i_ServoGripFinish){
            ledcWrite(ci_ServoGrip, degreesToDutyCycle(i_ServoGripPos));
            i_ServoGripPos++;
          }
          if (i_ServoGripPos >= i_ServoGripFinish){
            ui_State++;
          }
        }
        break;
      }
      case 1: {                                                             // case 1: rotate the arm, this time it will not rotate all the way to the finish position
        ul_CurMillis = millis();
        if (ul_CurMillis - ul_PrevMillis > ul_ArmDelay){
          ul_PrevMillis = ul_CurMillis;
          if (i_ServoArmPos > 100){                                         // Does not exceed 150 degrees to differentiate from goodobj process
            ledcWrite(ci_ServoArm, degreesToDutyCycle(i_ServoArmPos));
            i_ServoArmPos--;
          }
          if (i_ServoArmPos <= 100){                                        // When desired position is reached, go to next state
            ui_State++;
          }
        }
        break;
      }
      case 2: {                                                             // case 3: return to arm starting position, same as goodobj process
        ul_CurMillis = millis();
        if (ul_CurMillis - ul_PrevMillis > ul_ArmDelay){
          ul_PrevMillis = ul_CurMillis;
          if (i_ServoArmPos < i_ServoArmStart){
            ledcWrite(ci_ServoArm, degreesToDutyCycle(i_ServoArmPos));
            i_ServoArmPos++;
          }
          if (i_ServoArmPos >= i_ServoArmStart){                            // reset state and flags when finished
            ui_State++;
          }
        }
        break;
      }
      case 3: {                                                             // case 2: open the gripper, drop the object, same as goodobj process
        ul_CurMillis = millis();
        if (ul_CurMillis - ul_PrevMillis > ul_GripDelay){
          ul_PrevMillis = ul_CurMillis;
          if (i_ServoGripPos > i_ServoGripStart){
            ledcWrite(ci_ServoGrip, degreesToDutyCycle(i_ServoGripPos));
            i_ServoGripPos--;
          }
          if (i_ServoGripPos <= i_ServoGripStart){
            ui_State = 0;
            pickup = false;
            badobj = false;
            goodobj = false;
          }
        }
        break;
      }
    }
  }

  doHeartbeat();                                     // update heartbeat LED
}

// blink heartbeat LED
void doHeartbeat() {
  ul_CurMillis = millis();                           // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((ul_CurMillis - ul_LastHeartbeat) > ci_HeartbeatInterval) {
    ul_LastHeartbeat = ul_CurMillis;                 // update the heartbeat toggle time for the next cycle
    b_Heartbeat = !b_Heartbeat;                      // toggle state of LED
    digitalWrite(ci_HeartbeatLED, b_Heartbeat);      // update LED
  }
}

// Converts servo position in degrees into the required duty cycle for an RC servo motor control signal 
// assuming 16-bit resolution (i.e., value represented as fraction of 65535). 
// Note that the constants for minimum and maximum duty cycle may need to be adjusted for a specific motor
long degreesToDutyCycle(int deg) {
  const long cl_MinDutyCycle = 1650;                 // duty cycle for 0 degrees
  const long cl_MaxDutyCycle = 7800;                 // duty cycle for 180 degrees

  long l_DutyCycle = map(deg, 0, 180, cl_MinDutyCycle, cl_MaxDutyCycle);  // convert to duty cycle
  return l_DutyCycle;
}

void ARDUINO_ISR_ATTR buttonISR(void* arg) {
  Button* s = static_cast<Button*>(arg);              // cast pointer to static structure

  unsigned int pressTime = millis();                  // capture current time
  s->state = digitalRead(s->pin);                     // capture state of button
  // if button has been pressed and sufficient time has elapsed
  if ((!s->state && s->lastState == 1) && (pressTime - s->lastPressTime > cDebounceDelay)) {
    s->numberPresses += 1;                            // increment button press counter
    s->pressed = true;                                // set flag for "valid" button press
  }
  s->lastPressTime = pressTime;                       // update time of last state change
  s->lastState = s->state;                            // save last state
}
