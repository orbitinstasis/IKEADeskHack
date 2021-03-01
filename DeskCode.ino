/*

*/

// no sleep, 2.1W no usb, 2.0w with sleeps

#include <NewPing.h>
#include <Bounce2.h>
#include <AccelStepper.h>
#include "LowPower.h"
#include <elapsedMillis.h>

#define TRIGGER_PIN   A0//12 // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PIN      A1 //13 // Arduino pin tied to echo pin on ping sensor.

#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 150// Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define ITERATIONS     5 // Number of iterations.

#define GND_BUTTONS 9
#define BUT_SIT 8
#define BUT_STAND 7
#define BUT_UP 6
#define BUT_DOWN 5
#define NUM_BUTTONS 4

#define DRIVER_PUL 2
#define DRIVER_DIR 3
#define DRIVER_EN 4

#define STEPS_PER_REV 1600
#define STEPS_PER_CM STEPS_PER_REV*2
#define MAX_STEPS_PER_SECOND 4000
#define MAX_RPM 800

#define UP 0
#define DOWN 1
#define NOT_HOLDING (!holdButs[DOWN] && !holdButs[UP])
#define AWAKE_DURATION 50

#define LOW_LIMIT_CM 61
#define HIGH_LIMIT_CM 87 // good standing 

#define STANDING_HEIGHT 87
#define SITTING_HEIGHT 62

#define SPEED_UP 3200 //holding
#define SPEED_DOWN 3800

#define DEFINED_SPEED_UP 3000
#define DEFINED_ACC_UP 2500

#define DEFINED_ACC_DOWN 2000
#define DEFINED_SPEED_DOWN 10000

boolean isFirstDistanceRead = true;
boolean finishedHold = true;
boolean holdButs[2] = {false, false};
boolean setButPressed[2] = {false, false};
boolean isShutdown = true;
boolean firstTimestamp = true;
unsigned long timeSinceHold;
const uint8_t buttons[NUM_BUTTONS] = {BUT_SIT, BUT_STAND, BUT_UP, BUT_DOWN};
uint8_t currentIteration = 0;        // Keeps track of iteration step.
unsigned long pingTimer[ITERATIONS]; // Holds the times when the next ping should happen for each iteration.
unsigned long time_now = 0;
int xVal, yVal = 0;
int pd = 500;       // Pulse Delay period
unsigned int cm[ITERATIONS];         // Where the ping distances are stored.
int currentHeight = 0;
uint8_t previousHeight = 0;
elapsedMillis timeRunning;
AccelStepper stepper(1, DRIVER_PUL, DRIVER_DIR);
Bounce * butBounce = new Bounce[NUM_BUTTONS];
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
  //    Serial.begin(115200);
  pinMode(DRIVER_EN, OUTPUT);
  digitalWrite(DRIVER_EN, false);
  delayMicroseconds(10);
  pinMode(GND_BUTTONS, OUTPUT);
  digitalWrite(GND_BUTTONS, LOW);

  for (int i = 0; i < NUM_BUTTONS; i++) {
    butBounce[i].attach( buttons[i] , INPUT_PULLUP  );       //setup the bounce instance for the current button
    butBounce[i].interval(25);              // interval in ms
  }
  pingTimer[0] = millis() + 75;            // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < ITERATIONS; i++) // Set the starting time for each iteration.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
#define NORMAL_SPEED 13000
  stepper.setMaxSpeed(NORMAL_SPEED);
  stepper.setAcceleration(500);
  digitalWrite(DRIVER_EN, LOW);
  delayMicroseconds(10);
  stepper.disableOutputs();
  Serial.println("Current position: " + String(stepper.currentPosition()));
  delay(100);
}


void loop() {
  /**
     Fetch input values at start of loop (buttons)
  */
  for (uint8_t i = 2; i < 4; i++) {
    butBounce[i].update();
    holdButs[i - 2] = !butBounce[i].read();
  }

  /**
    button handles for sit and stand predefined heights, ignore if manually moving
  */
  for (int i = 0; i < 2 && NOT_HOLDING; i++)  {
    // Update the Bounce instance :
    butBounce[i].update();
    if ( butBounce[i].fell() )
      setButPressed[i] = true;
  }

  if (currentHeight != previousHeight) {
    previousHeight = currentHeight;
    Serial.println("Current height: " + String(currentHeight) + "\nCurrent position: " + String(stepper.currentPosition()));
  }

  /**
     Handle sleep state
  */
  //  if (currentHeight > 0 && NOT_HOLDING && !setButPressed[0] && !setButPressed[1] && timeRunning >= AWAKE_DURATION && finishedHold) {
  //    Serial.println("Sleeping");
  //    LowPower.powerDown(SLEEP_500MS , ADC_OFF, BOD_OFF);
  //    timeRunning = 0;
  //    return;
  //  }

  /**
     process 1 distance before sleeping then sleep until moving
  */
  for (uint8_t i = 0; i < ITERATIONS; i++) { // Loop through all the iterations.
    if (millis() >= pingTimer[i]) {          // Is it this iteration's time to ping?
      pingTimer[i] += PING_INTERVAL * ITERATIONS; // Set next time this sensor will be pinged.
      if (i == 0 && currentIteration == ITERATIONS - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar.timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentIteration = i;        // Sensor being accessed.
      cm[currentIteration] = 0;    // Make distance zero in case there's no ping echo for this iteration.
      sonar.ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }

  /**
     button handlers for hold to move up and down
  */

  if ( holdButs[DOWN] && holdButs[UP]) {    // PRESSING BOTH BUTTONS VOIDSO BOTH
    stepper.setSpeed(0);
    stepper.stop();
    stepper.disableOutputs();
    digitalWrite(DRIVER_EN, LOW);
    delayMicroseconds(10);
  }

  if (holdButs[DOWN] && !holdButs[UP]) {        // UP
    if (currentHeight < HIGH_LIMIT_CM) {      // DON'T MOVE IF REACHED SOFT LIMIT
      if (firstTimestamp) {                     // GRAB TIME SINCE HOLD START
        timeSinceHold = millis();
        firstTimestamp = false;
      }
      stepper.enableOutputs();
      digitalWrite(DRIVER_EN, HIGH);
      delayMicroseconds(10);
      stepper.setSpeed(SPEED_UP);
      if (finishedHold)                         // only 'stop' moving once
        finishedHold = false;
    } else {                                    // reached limit
      Serial.println("limit");
      finishedHold = true;
      stepper.setSpeed(0);
      stepper.stop();
      stepper.disableOutputs();
      digitalWrite(DRIVER_EN, LOW);
      delayMicroseconds(10);
    }
  }

  if (holdButs[UP] && !holdButs[DOWN]) {        // DOWN
    if (currentHeight > LOW_LIMIT_CM) {
      if (firstTimestamp) {  //first hold
        timeSinceHold = millis();
        firstTimestamp = false;
      }
      stepper.enableOutputs();
      digitalWrite(DRIVER_EN, HIGH);
      delayMicroseconds(10);
      stepper.setSpeed(-SPEED_DOWN);
      if (finishedHold)
        finishedHold = false;
    } else {
      Serial.println("limit");
      finishedHold = true;
      stepper.setSpeed(0);
      stepper.stop();
      stepper.disableOutputs();
      digitalWrite(DRIVER_EN, LOW);
      delayMicroseconds(10);
    }
  }

  if (!holdButs[UP] && !holdButs[DOWN] && !finishedHold) {    // STOPPED HOLDING
    firstTimestamp = true;
    finishedHold = true;
    Serial.println("\tStopped moving");
    stepper.setSpeed(0);
    stepper.stop();
    stepper.disableOutputs();
    digitalWrite(DRIVER_EN, LOW);
    delayMicroseconds(10);
  }

  /**
     Process outputs
  */
  stepper.runSpeed();

  if (setButPressed[0] && setButPressed[1]) {
    Serial.println("E stopped!");
    stepper.setSpeed(0);
    stepper.stop();
    stepper.disableOutputs();
    digitalWrite(DRIVER_EN, LOW);
    delayMicroseconds(10);
    setButPressed[0] = false;
    setButPressed[1] = false;
  }

  for (uint8_t i = 0; i < 2; i++) {
    if (setButPressed[i]) {
      setButPressed[i] = false;
      long distanceToMove = 0;
      if (i == 0) {                                            // SIT
        distanceToMove = ((long)SITTING_HEIGHT - (long)currentHeight) * (long)STEPS_PER_CM; // GOING DOWN
      } else {                                        // SIT
        distanceToMove = ((long)STANDING_HEIGHT - (long)currentHeight) * (long)STEPS_PER_CM; // GOING UP
      }
      if (distanceToMove < 0) {
        stepper.setMaxSpeed(DEFINED_SPEED_DOWN);
        stepper.setAcceleration(DEFINED_ACC_DOWN);
      } else {
        stepper.setMaxSpeed(DEFINED_SPEED_UP);
        stepper.setAcceleration(DEFINED_ACC_UP);
      }
      stepper.move(distanceToMove);
      stepper.enableOutputs();
      digitalWrite(DRIVER_EN, HIGH);
      delayMicroseconds(10);
      stepper.runToPosition();
      stepper.disableOutputs();
      digitalWrite(DRIVER_EN, LOW);
      delayMicroseconds(10);
    }
  }
}
void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar.check_timer())
    cm[currentIteration] = sonar.ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // All iterations complete, calculate the median.
  unsigned int uS[ITERATIONS];
  uint8_t j, it = ITERATIONS;
  uS[0] = NO_ECHO;
  for (uint8_t i = 0; i < it; i++) { // Loop through iteration results.
    if (cm[i] != NO_ECHO) { // Ping in range, include as part of median.
      if (i > 0) {          // Don't start sort till second ping.
        for (j = i; j > 0 && uS[j - 1] < cm[i]; j--) // Insertion sort loop.
          uS[j] = uS[j - 1];                         // Shift ping array to correct position for sort insertion.
      } else j = 0;         // First ping is sort starting point.
      uS[j] = cm[i];        // Add last ping to array in sorted position.
    } else it--;            // Ping out of range, skip and don't include as part of median.
  }

  currentHeight = (uint8_t)uS[it >> 1];
  if (abs(currentHeight - previousHeight) > 2 && !isFirstDistanceRead) {
    currentHeight = previousHeight;
    isFirstDistanceRead = false;
  }
}
