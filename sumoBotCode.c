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

// === Line Detection IR ===
#define LD_FR_PIN 7
#define LD_FL_PIN 1
#define LD_BR_PIN 4
#define LD_BL_PIN 6

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
  setSpeed(200);
  digitalWrite(FBL, LOW); digitalWrite(BBL, HIGH);
  digitalWrite(FFL, HIGH); digitalWrite(BFL, LOW);
  digitalWrite(FBR, HIGH); digitalWrite(BBR, LOW);
  digitalWrite(FFR, LOW); digitalWrite(BFR, HIGH);
  delay(10);
  stopMotors();
}

void spinRight() {
  setSpeed(200);
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

void setup() {
// Serial communication at a baudrate of 9600
  Serial.begin(9600);
  irSetup();
}

void loop() {
  // Get a distance measurement and store it as distance_cm
  IR_LEFT_DISTANCE = readDistance(IR_LEFT_PIN);
  IR_FORWARD_DISTANCE = readDistance(IR_FORWARD_PIN);
  IR_RIGHT_DISTANCE = readDistance(IR_RIGHT_PIN);
/*
// Print the measured distance to the serial monitor
 Serial.print("Left distance: ");
  Serial.print(IR_LEFT_DISTANCE);
  Serial.println(" cm");
 Serial.print("Forward distance: ");
  Serial.print(IR_FORWARD_DISTANCE);
  Serial.println(" cm");
 Serial.print("Right distance: ");
  Serial.print(IR_RIGHT_DISTANCE);
  Serial.println(" cm");
 delay(100);
*/
}
