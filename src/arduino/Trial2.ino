#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

// Pin Definitions (Updated IN2_PIN to 10)
#define ENA_PIN           5    // Motor PWM speed
#define IN1_PIN           13   // Motor direction pin 1
#define IN2_PIN           10   // Motor direction pin 2 (changed from A3 to D10)
#define SERVO_PIN         9    // Steering servo pin
#define START_SWITCH_PIN  4    // Start switch with internal pull-up

// Ultrasonic sensors
#define TRIG_PIN          2    // All ultrasonic TRIG pins connected
#define ECHO_FRONT_LEFT   5
#define ECHO_FRONT_RIGHT  6
#define ECHO_SIDE_LEFT    7
#define ECHO_SIDE_RIGHT   8
#define ECHO_REAR_RIGHT  12
#define ECHO_REAR_LEFT   11

// IR Sensors
#define IR_FRONT_RIGHT A0
#define IR_FRONT_LEFT  A1
#define IR_REAR_CENTER A2

// Constants
#define INITIAL_ANGLE     35
#define CALIBRATION_SAMPLES 1000

MPU6050 mpu;
Servo steeringServo;

// Global Variables
int16_t gz;
float gyro_z_offset = 0;
float yaw_angle = 0;

int currentAngle = INITIAL_ANGLE;
int targetYawAngle = 0;

unsigned long previous_time;
float integral = 0;
float previousError = 0;

bool started = false; // Flag to check start switch
int fixing_time = 0;  // Used in reverse maneuver

// PID gains (tune as needed)
float Kp = 0.2;  // Reduced for less aggressive correction
float Ki = 0.0;
float Kd = 0.0;
float ANGLE_CORRECTION_FACTOR = 1.0;

// Servo bias offset to fix left tilt (adjust experimentally)
int servo_bias_offset = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Motor pins
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  // Servo setup
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(INITIAL_ANGLE + servo_bias_offset);

  // Ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_FRONT_LEFT, INPUT);
  pinMode(ECHO_FRONT_RIGHT, INPUT);
  pinMode(ECHO_SIDE_LEFT, INPUT);
  pinMode(ECHO_SIDE_RIGHT, INPUT);
  pinMode(ECHO_REAR_RIGHT, INPUT);
  pinMode(ECHO_REAR_LEFT, INPUT);

  // Start switch input with internal pull-up
  pinMode(START_SWITCH_PIN, INPUT_PULLUP);

  // MPU6050 initialization
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connected" : "MPU6050 connection failed");

  // Gyro calibration
  Serial.println("Calibrating gyro offset...");
  calculateGyroDrift();
  Serial.print("Gyro Z offset: ");
  Serial.println(gyro_z_offset);

  previous_time = millis();

  // Initially stop motor
  stopMotor();

  Serial.println("Setup complete, waiting for start switch...");
}

void loop() {
  // Wait for start switch press (active low)
  if (!started) {
    if (digitalRead(START_SWITCH_PIN) == LOW) {
      started = true;
      Serial.println("Start switch pressed, starting robot.");
      delay(500); // Debounce delay
    } else {
      // Wait here until start switch pressed
      stopMotor();
      return;
    }
  }

  // Read and send IR sensor values for monitoring or use in logic
  sendIRSensors();

  // Drive forward with gyro stabilization
  MoveFWwithGyro();

  // Avoid collisions and reverse if too close
  stopfromcrashing_andreturn();

  // Your additional state machine, turns, counters can be added here
}

// --- IR Sensor reading and sending function ---
void sendIRSensors() {
  int irFrontRight = analogRead(IR_FRONT_RIGHT);
  int irFrontLeft = analogRead(IR_FRONT_LEFT);
  int irRearCenter = analogRead(IR_REAR_CENTER);

  Serial.print("IR0:");
  Serial.println(irFrontRight);
  Serial.print("IR1:");
  Serial.println(irFrontLeft);
  Serial.print("IR2:");
  Serial.println(irRearCenter);

  // Optionally, use these values in control logic:
  // e.g., slow down or avoid obstacles if IR reading below threshold
}

// --- Other functions unchanged from your previous code ---

void calculateGyroDrift() {
  long sum = 0;
  int validReadings = 0;
  for (int i=0; i<CALIBRATION_SAMPLES; i++) {
    int16_t raw = mpu.getRotationZ();
    if (raw > -32760 && raw < 32760) {
      sum += raw;
      validReadings++;
    }
    delay(3);
  }
  gyro_z_offset = (validReadings > 0) ? (float)sum/validReadings : 0;
}

void MoveFWwithGyro() {
  unsigned long current_time = millis();
  float dt = (current_time - previous_time)/1000.0;
  previous_time = current_time;

  gz = mpu.getRotationZ() - gyro_z_offset;
  float gyro_z = ((float)gz)/131.0;

  yaw_angle += gyro_z * dt;

  float error = targetYawAngle - yaw_angle;
  integral += error * dt;
  float derivative = (error - previousError) / dt;
  float output = Kp*error + Ki*integral + Kd*derivative;
  previousError = error;

  currentAngle += output * ANGLE_CORRECTION_FACTOR;
  currentAngle = constrain(currentAngle, 0, 180);

  int servoAngleToWrite = constrain(currentAngle + servo_bias_offset, 0, 180);
  steeringServo.write(servoAngleToWrite);

  Serial.print("Yaw Ang: "); Serial.print(yaw_angle);
  Serial.print(" | Target: "); Serial.print(targetYawAngle);
  Serial.print(" | PID Out: "); Serial.print(output);
  Serial.print(" | Servo angle: "); Serial.println(servoAngleToWrite);

  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  analogWrite(ENA_PIN, 255);
}

void stopfromcrashing_andreturn() {
  if (usForward() < 15) {
    stopMotor();
    delay(100);

    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    analogWrite(ENA_PIN, 255);

    fixing_time = 0;
    while (fixing_time < 100) {
      MoveFWwithGyro_Reverse();
      fixing_time++;
    }

    stopMotor();
    delay(100);

    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, 255);
  }
}

void MoveFWwithGyro_Reverse() {
  unsigned long current_time = millis();
  float dt = (current_time - previous_time)/1000.0;
  previous_time = current_time;

  gz = mpu.getRotationZ() - gyro_z_offset;
  float gyro_z = ((float)gz)/131.0;

  yaw_angle += gyro_z * dt;

  float error = targetYawAngle - yaw_angle;
  integral += error * dt;
  float derivative = (error - previousError) / dt;
  float output = Kp*error + Ki*integral + Kd*derivative;
  previousError = error;

  float servo_angle = constrain(INITIAL_ANGLE + output + servo_bias_offset, 60, 140);
  steeringServo.write((int)servo_angle);

  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  analogWrite(ENA_PIN, 200);
}

int usForward() {
  return ultrasonicDistance(TRIG_PIN, ECHO_FRONT_LEFT);
}

int ultrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0)
    return 200;
  return duration * 0.034 / 2;
}

void stopMotor() {
  analogWrite(ENA_PIN, 0);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
}
