#include <Servo.h>

Servo steeringServo;

const int ENA = 5;
const int IN1 = 13;
const int IN2 = A3;
const int trigPin = 2;
const int echoPins[6] = {5, 6, 7, 8, 12, 11};
const int startSwitchPin = 4;
const int servoPin = 9;

long distances[6];
int irSensors[3];
int motorSpeed = 0;
int steeringAngle = 90;

void setup() {
  Serial.begin(115200);
  steeringServo.attach(servoPin);
  pinMode(trigPin, OUTPUT);
  for (int i = 0; i < 6; i++) pinMode(echoPins[i], INPUT);
  pinMode(startSwitchPin, INPUT_PULLUP);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  motorSpeed = 0;
  steeringAngle = 90;
  steeringServo.write(steeringAngle);
  stopMotor();
}

void loop() {
  if (digitalRead(startSwitchPin) == LOW) {
    performAction();
  }
  delay(100);
}

void performAction() {
  readUltrasonicSensors();
  readIRSensors();
  Serial.print("SENSORS:");
  for (int i = 0; i < 6; i++) {
    Serial.print(distances[i]);
    Serial.print(",");
  }
  for (int i = 0; i < 3; i++) {
    Serial.print(irSensors[i]);
    if (i < 2) Serial.print(",");
  }
  Serial.println();

  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    parseCommand(cmd);
  }
  delay(20);
}

long readUltrasonicDistance(int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration / 29 / 2;
}

void readUltrasonicSensors() {
  for (int i = 0; i < 6; i++) {
    distances[i] = readUltrasonicDistance(echoPins[i]);
  }
}

void readIRSensors() {
  irSensors[0] = analogRead(A0);
  irSensors[1] = analogRead(A1);
  irSensors[2] = analogRead(A2);
}

void parseCommand(String cmd) {
  cmd.trim();
  if (cmd.startsWith("MOTOR:")) {
    motorSpeed = cmd.substring(6).toInt();
    setMotorSpeed(motorSpeed);
  } else if (cmd.startsWith("STEER:")) {
    steeringAngle = cmd.substring(6).toInt();
    setSteering(steeringAngle);
  }
}

void setMotorSpeed(int speed) {
  speed = constrain(speed, 0, 255);
  if (speed == 0) {
    stopMotor();
    return;
  }
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

void setSteering(int angle) {
  angle = constrain(angle, 0, 180);
  steeringServo.write(angle);
}
