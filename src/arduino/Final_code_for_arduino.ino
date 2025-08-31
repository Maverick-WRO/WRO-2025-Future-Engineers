#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

Servo steering;               // SG90 servo
MPU6050 imu;

int ENA = 5;                  // PWM for motor speed
int IN1 = 13;                 
int IN2 = 10;

int SERVO_PIN = 9;

int trigPin = 2;
int echoPinFL = 3;
int echoPinFR = 6;
int echoPinSL = 7;
int echoPinSR = 8;
int echoPinRR = 12;
int echoPinRL = 11;

int IR1 = A0;  // front-right
int IR2 = A1;  // front-left
int IR3 = A2;  // rear-center

int startButton = 4;   // changed from D4 → D3 (avoid conflicts with trigPin)

int motorSpeed = 0;
int steeringAngle = 90;

bool started = false;

void setup() {
  Serial.begin(115200);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  steering.attach(SERVO_PIN);
  steering.write(90);

  pinMode(startButton, INPUT_PULLUP);

  Wire.begin();
  imu.initialize();

  Serial.println("Arduino ready");
}

void loop() {
  // --- Start button ---
  if (digitalRead(startButton) == LOW) {
    started = true;
    Serial.println("Run Started");
    delay(500);
  }

  if (!started) return;

  // --- Parse serial from Pi ---
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('>');
    if (cmd.startsWith("<SPD:")) {
      int sIndex = cmd.indexOf(':') + 1;
      int aIndex = cmd.indexOf(";ANG:");
      int speed = cmd.substring(sIndex, aIndex).toInt();
      int angle = cmd.substring(aIndex + 5).toInt();

      motorSpeed = speed;
      steeringAngle = angle;

      setMotor(motorSpeed);
      steering.write(steeringAngle);
    }
  }

  // --- Debug IMU yaw ---
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // (You can compute angles if needed later)
}

void setMotor(int speed) {
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    // Map 0–100% from Pi into usable torque range (120–255 PWM)
    analogWrite(ENA, map(speed, 0, 100, 120, 255));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

long getDistance(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  pinMode(echo, INPUT);
  long duration = pulseIn(echo, HIGH, 20000);
  return duration * 0.034 / 2;
}
