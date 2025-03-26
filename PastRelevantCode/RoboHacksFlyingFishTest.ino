#define LEFT_SENSOR 7

void setup(){
Serial.begin(9600);
pinMode(LEFT_SENSOR,INPUT);
}

void loop(){
  int leftValue = digitalRead(LEFT_SENSOR);
  Serial.println(leftValue);
  delay(10);
}
