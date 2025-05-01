#include <Arduino.h>
#include <math.h>

// === SHARP IR SENSOR PINS ===
#define IR_LEFT_PIN    2
#define IR_FORWARD_PIN 20
#define IR_RIGHT_PIN   19
float IR_LEFT_DISTANCE = 0;
float IR_FORWARD_DISTANCE = 0;
float IR_RIGHT_DISTANCE = 0;

// === Sumostart Module ===
#define SM_PIN 38
bool robotEnabled = false;

// === Line Detection IR ===
#define LD_FR_PIN 7
#define LD_FL_PIN 1
#define LD_BR_PIN 4
#define LD_BL_PIN 6

 int frontRightValue;
  int frontLeftValue;
  int backRightValue;
  int backLeftValue;

// === MOTOR PINS ===
// [Back Left]
#define FBL 18
#define BBL 17
#define PWM_BL 5

// [Back Right]
#define FBR 3
#define BBR 8
#define PWM_BR 9

// [Front Right]
#define FFR 13
#define BFR 14
#define PWM_FR 10

// [Front Left]
#define FFL 11
#define BFL 12
#define PWM_FL 16

// STBY pins
#define STBY1 47
#define STBY2 21

void irLineDetectSetup(){
  pinMode(LD_FR_PIN,INPUT);
  pinMode(LD_FL_PIN,INPUT);
  pinMode(LD_BR_PIN,INPUT);
  pinMode(LD_BL_PIN,INPUT);
}

void irSetup(){
  pinMode(IR_FORWARD_PIN, INPUT);
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
}

void motorSetup() {
  pinMode(FBL, OUTPUT); pinMode(BBL, OUTPUT); pinMode(PWM_BL, OUTPUT);
  pinMode(FBR, OUTPUT); pinMode(BBR, OUTPUT); pinMode(PWM_BR, OUTPUT);
  pinMode(FFR, OUTPUT); pinMode(BFR, OUTPUT); pinMode(PWM_FR, OUTPUT);
  pinMode(FFL, OUTPUT); pinMode(BFL, OUTPUT); pinMode(PWM_FL, OUTPUT);
  pinMode(STBY1, OUTPUT); digitalWrite(STBY1, HIGH);
  pinMode(STBY2, OUTPUT); digitalWrite(STBY2, HIGH);

  ledcAttach(PWM_BL, 1000, 8);
  ledcAttach(PWM_BR, 1000, 8);
  ledcAttach(PWM_FR, 1000, 8);
  ledcAttach(PWM_FL, 1000, 8);
}

void microStartSetup() {
  pinMode(SM_PIN, INPUT);  
  robotEnabled = false;    
}

void setSpeed(int speed) {
  ledcWrite(PWM_BL, speed);
  ledcWrite(PWM_BR, speed);
  ledcWrite(PWM_FR, speed);
  ledcWrite(PWM_FL, speed);
}

void stopMotors() {
  digitalWrite(FBL, LOW); digitalWrite(BBL, LOW);
  digitalWrite(FBR, LOW); digitalWrite(BBR, LOW);
  digitalWrite(FFR, LOW); digitalWrite(BFR, LOW);
  digitalWrite(FFL, LOW); digitalWrite(BFL, LOW);
  setSpeed(0);
}

void spinLeft() {
  setSpeed(255);
  digitalWrite(FBL, LOW); digitalWrite(BBL, HIGH);
  digitalWrite(FFL, HIGH); digitalWrite(BFL, LOW);
  digitalWrite(FBR, HIGH); digitalWrite(BBR, LOW);
  digitalWrite(FFR, LOW); digitalWrite(BFR, HIGH);
  delay(10);
  stopMotors();
}

void spinRight() {
  setSpeed(255);
  digitalWrite(FBL, HIGH); digitalWrite(BBL, LOW);
  digitalWrite(FFL, LOW); digitalWrite(BFL, HIGH);
  digitalWrite(FBR, LOW); digitalWrite(BBR, HIGH);
  digitalWrite(FFR, HIGH); digitalWrite(BFR, LOW);
  delay(10);  // brief turn
  stopMotors();
}

void moveForward() {
  setSpeed(255);
  digitalWrite(FBL, HIGH); digitalWrite(BBL, LOW);   // Back Left forward
  digitalWrite(FFL, HIGH); digitalWrite(BFL, LOW);   // Front Left forward
  digitalWrite(FBR, HIGH); digitalWrite(BBR, LOW);   // Back Right forward
  digitalWrite(FFR, HIGH); digitalWrite(BFR, LOW);   // Front Right forward
}

void moveBackwards() {
  setSpeed(255);
  digitalWrite(FBL, LOW); digitalWrite(BBL, HIGH);   // Back Left backward
  digitalWrite(FFL, LOW); digitalWrite(BFL, HIGH);   // Front Left backward
  digitalWrite(FBR, LOW); digitalWrite(BBR, HIGH);   // Back Right backward
  digitalWrite(FFR, LOW); digitalWrite(BFR, HIGH);   // Front Right backward
}

float readDistance(int pin) {
  float sum = 0;
  int adc = analogRead(pin);
  float voltage = adc * (3.3 / 4095.0);
  sum = 27.86 / pow(voltage, 1.15);
  adc=analogRead(pin);
  voltage = adc * (3.3 / 4095.0);
  sum+=27.86 / pow(voltage, 1.15);
  float distance = sum/2;
  if (distance > 100) distance = 100;
  return distance;
}

bool detectLine() {
  frontRightValue = digitalRead(LD_FR_PIN);
  frontLeftValue = digitalRead(LD_FL_PIN);
  backRightValue = digitalRead(LD_BR_PIN);
  backLeftValue = digitalRead(LD_BL_PIN);
  
  /*
  Serial.print("FR: "); Serial.print(frontRightValue);
  Serial.print(" FL: "); Serial.print(frontLeftValue);
  Serial.print(" BR: "); Serial.print(backRightValue);
  Serial.print(" BL: "); Serial.println(backLeftValue);
  */
  
  if (frontRightValue == 0 || frontLeftValue == 0) {
    if (frontRightValue == 0 && frontLeftValue == 0) {
      moveBackwards();
      delay(500);
    } 
    else if (frontRightValue == 0) {
      moveBackwards();
      delay(300);
      stopMotors();
      spinLeft();
      delay(200);
    } 
    else {
      moveBackwards();
      delay(300);
      stopMotors();
      spinRight();
      delay(200); 
    }
    return true; 
  }
  
  if (backRightValue == 0 || backLeftValue == 0) {
    if (backRightValue == 0 && backLeftValue == 0) {
      moveForward();
      delay(500);
    } 
    else if (backRightValue == 0) {
      moveForward();
      delay(300);
      stopMotors();
      spinLeft();
      delay(200); 
    } 
    else {
      moveForward();
      delay(300);
      stopMotors();
      spinRight();
      delay(200);  
    }
    return true;  
  }
  
  return false;  // No line detected
}


void checkMicroStartSignal() {
  int signalValue = digitalRead(SM_PIN);
  
  if (signalValue == HIGH && !robotEnabled) {
    robotEnabled = true; 
    Serial.println("Robot STARTED by MicroStart module!");
  }
  
  if (signalValue == LOW && robotEnabled) {
    robotEnabled = false;
    stopMotors();
    Serial.println("Robot STOPPED by MicroStart module!");
  }
}

void setup() {
  Serial.begin(9600);
  irSetup();
  motorSetup();
  irLineDetectSetup();  
  microStartSetup();
}

void loop() {
 // checkMicroStartSignal();
 // if(robotEnabled){
    IR_LEFT_DISTANCE = readDistance(IR_LEFT_PIN);
    IR_FORWARD_DISTANCE = readDistance(IR_FORWARD_PIN);
    IR_RIGHT_DISTANCE = readDistance(IR_RIGHT_PIN);
    
    // First priority: Check for ring edge (white line)
    if (detectLine()) {
     return; 
    }

    // Second priority: Find and attack opponent
    minDist = min(IR_LEFT_DISTANCE, min(IR_FORWARD_DISTANCE, IR_FORWARD_RIGHT));

  if (minDist > 70) {
    stopMotors();
    return;
  }

  if (abs(distF - minDist) < 2.0) {
    moveForward();
    delay(10);
  } else if (abs(distL - minDist) < 2.0) {
    spinLeft();
  } else {
    spinRight();
  }
  delay(10);
}
//}
