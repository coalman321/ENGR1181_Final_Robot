#include <Servo.h>
Servo left;
Servo right;

const int ledPin = 6;
const int pingPin = 7;
long duration, inches;

void ping() {
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  inches = duration/74/2;
  Serial.println(inches);
}

void drive() {
  left.write(170);
  right.write(0);
}

void park() {
  left.write(90);
  right.write(90);
}

void turn() {
  left.write(0);
  right.write(0);
}

void blink() {
  digitalWrite(ledPin, HIGH);
  delay(50);
  digitalWrite(ledPin, LOW);
}

void setup() {
left.attach(10);
right.attach(9);

pinMode(ledPin, OUTPUT);

Serial.begin(9600);
}

void loop() {
  ping();
  
  if (inches<4){
    tone(3, 330, 1000);
    park();
    delay(50);
    blink();
    turn();
    delay(500);
  }
  else{
    drive();
  }
  
  delay(200);
}
