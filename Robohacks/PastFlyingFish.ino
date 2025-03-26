#define LEFT_SENSOR 12
#define RIGHT_SENSOR 3

// Function to initialize the setup
void setupSensors() {
    Serial.begin(9600);
    pinMode(LEFT_SENSOR, INPUT);
    pinMode(RIGHT_SENSOR, INPUT);
}

// Function to read the left sensor value
int readLeftSensor() {
    int leftValue = digitalRead(LEFT_SENSOR);
    Serial.println(leftValue);
    delay(10);
    return leftValue;
}

// Function to read the right sensor value
int readRightSensor() {
    int rightValue = digitalRead(RIGHT_SENSOR);
    Serial.println(rightValue);
    delay(10);
    return rightValue;
}
