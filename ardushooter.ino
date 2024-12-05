#include <SoftwareSerial.h>

#define EN        8      // Stepper Motor Enable, Active Low Level 
#define R_DIR     6      // Rail Stepper Motor Direction Control 
#define S_DIR     7      // Shoot Stepper Motor Direction Control 
#define R_STP     3     // Rail Stepper Control 
#define S_STP     4      // Shoot Stepper Control

#define R_PIN     12      // Arduino pin connected to Right Limit
#define L_PIN     13     // Arduino pin connected to Left Limit

float stepsPerDegree = 3.8;  // Steps per degree, calculated during homing
float currentAngle = 0;    // Current angle position of the rail

SoftwareSerial stmSerial(2, 3); // RX, TX

// microstepping
// full = 0 0 0
// half = 1 0 0
// quarter = 0 1 0
// eigth = 1 1 0
// sixteenth = 0 0 1
// thirtysecond = 1 0 1

void stepRail(boolean dir, int steps)
{
  digitalWrite(R_DIR, dir);
  for (int i = 0; i < steps; i++) {
    digitalWrite(R_STP, HIGH);
    delayMicroseconds(6000);
    digitalWrite(R_STP, LOW);
    delayMicroseconds(6000);
  }
  delayMicroseconds(100);
}

void stepShoot(boolean dir, int steps)
{
  digitalWrite(S_DIR, dir);
  for (int i = 0; i < steps; i++) {
    digitalWrite(S_STP, HIGH);
    delayMicroseconds(4000);
    digitalWrite(S_STP, LOW);
    delayMicroseconds(4000);
  }
  delayMicroseconds(100);
}

void homeAndCenterStepper() {
  digitalWrite(EN, LOW);  
  // uint16_t stepCount = 0;

  // while (digitalRead(L_PIN) == HIGH) {  
  //   stepRail(false, 1);  
  //   stepCount += 1;
  // }

  uint16_t totalSteps = 0;
  while (digitalRead(R_PIN) == HIGH) {  
    stepRail(true, 1); 
    totalSteps += 1;
  }

  // uint16_t stepsToCenter = totalSteps / 2;
  uint16_t stepsToCenter = 171;
  stepRail(false, stepsToCenter); 
  currentAngle = 0;
  digitalWrite(EN, HIGH);  
}

void moveToAngle(float angle) {
  float stepsNeeded = (angle - currentAngle) * stepsPerDegree;
  int steps = abs(stepsNeeded);
  boolean dir = stepsNeeded < 0 ? false : true;  
  digitalWrite(EN, LOW);  
  stepRail(dir, steps);
  digitalWrite(EN, HIGH);  
  currentAngle = angle;
}

void clearSerialBuffer() {
    while (stmSerial.available() > 0) {
        stmSerial.read(); // Discard any incoming data
    }
}


void setup() {
  pinMode(R_DIR, OUTPUT);
  pinMode(R_STP, OUTPUT);
  pinMode(S_DIR, OUTPUT); 
  pinMode(S_STP, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);
  pinMode(R_PIN, INPUT_PULLUP);  
  pinMode(L_PIN, INPUT_PULLUP);
  Serial.begin(9600);
  stmSerial.begin(115200);
}

void loop() {
  if (stmSerial.available() > 0) {
    String input = stmSerial.readStringUntil('\n');
    input.trim(); 

    if (input.equalsIgnoreCase("H")) {
      stmSerial.flush();
      homeAndCenterStepper();
    }
    else if (input.equalsIgnoreCase("S")) {
      stmSerial.flush();
      digitalWrite(EN, LOW);  
      stepShoot(false, 150); 
      digitalWrite(EN, HIGH);  
    }
    else {
      stmSerial.flush();
      float angle = input.toFloat();
      if (angle != 0 || input == "0") {
        moveToAngle(angle);
      }
    }
  }
}