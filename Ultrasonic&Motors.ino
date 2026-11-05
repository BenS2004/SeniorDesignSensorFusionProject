#include <Arduino.h>
#include <ESP32Servo.h>

// =====================================================
// ULTRASONIC SENSOR STRUCTURE
// =====================================================

struct UltrasonicSensor {
  uint8_t trigPin;
  uint8_t echoPin;
};

// Current ultrasonic wiring
UltrasonicSensor sensorFront = {23, 22};
UltrasonicSensor sensorLeft  = {18, 19};
UltrasonicSensor sensorRight = {17, 5};
UltrasonicSensor sensorBack  = {4, 16};

const float SPEED_OF_SOUND = 0.0343;

// =====================================================
// MOTOR DRIVER SIGNAL PINS
// HB-25 W signal connections
// =====================================================

#define MOTOR_LEFT_SIGNAL   33
#define MOTOR_RIGHT_SIGNAL  32

Servo motorLeft;
Servo motorRight;

// HB-25 pulse widths
const int MOTOR_STOP_US    = 1500;
const int MOTOR_FORWARD_US = 1350;
const int MOTOR_REVERSE_US = 1650;

// =====================================================
// ENCODER PINS
// =====================================================
// RIGHT [ 2, 15] ,  LEFT[27, 14]
// Right encoder
#define ENC_RIGHT_A 27
#define ENC_RIGHT_B 14

// Left encoder
#define ENC_LEFT_A 2
#define ENC_LEFT_B 15

#define ENCODER_PULSES_PER_REV 36f
#define WHEEL_DIAMETER_CM 15.24f
// =====================================================
// ENCODER VARIABLES
// =====================================================

volatile long rightEncoderCount = 0;
volatile long leftEncoderCount  = 0;

volatile int rightDirection = 1;
volatile int leftDirection  = 1;

volatile float rightWheelDistanceCM = 0.0;
volatile float leftWheelDistanceCM  = 0.0;

volatile int prevRightA = LOW;
volatile int prevRightB = LOW;

volatile int prevLeftA = LOW;
volatile int prevLeftB = LOW;

// =====================================================
// OBSTACLE SETTINGS
// =====================================================

const float FRONT_OBSTACLE_DISTANCE = 40.0;
const float SIDE_OBSTACLE_DISTANCE  = 25.0;

// =====================================================
// ENCODER INTERRUPTS
// =====================================================

void IRAM_ATTR rightEncoderISR() {

  int A = digitalRead(ENC_RIGHT_A);
  int B = digitalRead(ENC_RIGHT_B);

  // Detect change on channel A
  if (A != prevRightA) {

    rightDirection = (A == B) ? -1 : 1;
    rightEncoderCount += rightDirection;
  }

  rightWheelDistanceCM = (rightEncoderCount / ENCODER_PULSES_PER_REV) * (PI * WHEEL_DIAMETER_CM);

  prevRightA = A;
  prevRightB = B;
}


void IRAM_ATTR leftEncoderISR() {

  int A = digitalRead(ENC_LEFT_A);
  int B = digitalRead(ENC_LEFT_B);

  // Detect change on channel A
  if (A != prevLeftA) {

    leftDirection = (A == B) ? -1 : 1;
    leftEncoderCount += leftDirection;
  }

  leftWheelDistanceCM = (leftEncoderCount / ENCODER_PULSES_PER_REV) * (PI * WHEEL_DIAMETER_CM);
  prevLeftA = A;
  prevLeftB = B;
}

// =====================================================
// ULTRASONIC FUNCTION
// =====================================================

float measureDistance(UltrasonicSensor sensor) {

  digitalWrite(sensor.trigPin, LOW);
  delayMicroseconds(5);

  digitalWrite(sensor.trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(sensor.trigPin, LOW);

  unsigned long duration =
      pulseIn(sensor.echoPin, HIGH, 30000);

  // No echo detected
  if (duration == 0) {
    return 400.0;
  }

  return (duration * SPEED_OF_SOUND) / 2.0;
}

// =====================================================
// MOTOR CONTROL
// =====================================================

void stopMotors() {

  motorLeft.writeMicroseconds(MOTOR_STOP_US);
  motorRight.writeMicroseconds(MOTOR_STOP_US);

  Serial.println("ACTION: STOP");
}


void moveForward() {

  motorLeft.writeMicroseconds(MOTOR_FORWARD_US);
  motorRight.writeMicroseconds(MOTOR_FORWARD_US);

  Serial.println("ACTION: FORWARD");
}


void moveBackward() {

  motorLeft.writeMicroseconds(MOTOR_REVERSE_US);
  motorRight.writeMicroseconds(MOTOR_REVERSE_US);

  Serial.println("ACTION: BACKWARD");
}


void pivotLeft() {

  motorLeft.writeMicroseconds(MOTOR_REVERSE_US);
  motorRight.writeMicroseconds(MOTOR_FORWARD_US);

  Serial.println("ACTION: PIVOT LEFT");
}


void pivotRight() {

  motorLeft.writeMicroseconds(MOTOR_FORWARD_US);
  motorRight.writeMicroseconds(MOTOR_REVERSE_US);

  Serial.println("ACTION: PIVOT RIGHT");
}


void slightLeft() {

  motorLeft.writeMicroseconds(MOTOR_STOP_US);
  motorRight.writeMicroseconds(MOTOR_FORWARD_US);

  Serial.println("ACTION: SLIGHT LEFT");
}


void slightRight() {

  motorLeft.writeMicroseconds(MOTOR_FORWARD_US);
  motorRight.writeMicroseconds(MOTOR_STOP_US);

  Serial.println("ACTION: SLIGHT RIGHT");
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  // -------------------------------------------------
  // Ultrasonic sensors
  // -------------------------------------------------

  pinMode(sensorFront.trigPin, OUTPUT);
  pinMode(sensorFront.echoPin, INPUT);

  pinMode(sensorLeft.trigPin, OUTPUT);
  pinMode(sensorLeft.echoPin, INPUT);

  pinMode(sensorRight.trigPin, OUTPUT);
  pinMode(sensorRight.echoPin, INPUT);

  pinMode(sensorBack.trigPin, OUTPUT);
  pinMode(sensorBack.echoPin, INPUT);

  // Start all trigger pins LOW
  digitalWrite(sensorFront.trigPin, LOW);
  digitalWrite(sensorLeft.trigPin, LOW);
  digitalWrite(sensorRight.trigPin, LOW);
  digitalWrite(sensorBack.trigPin, LOW);

  // -------------------------------------------------
  // Encoders
  // -------------------------------------------------

  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);

  // Store initial encoder states
  prevRightA = digitalRead(ENC_RIGHT_A);
  prevRightB = digitalRead(ENC_RIGHT_B);

  prevLeftA = digitalRead(ENC_LEFT_A);
  prevLeftB = digitalRead(ENC_LEFT_B);

  attachInterrupt(
    digitalPinToInterrupt(ENC_RIGHT_A),
    rightEncoderISR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(ENC_LEFT_A),
    leftEncoderISR,
    CHANGE
  );

  // -------------------------------------------------
  // HB-25 motor drivers
  // -------------------------------------------------

  motorLeft.attach(
    MOTOR_LEFT_SIGNAL,
    1000,
    2000
  );

  motorRight.attach(
    MOTOR_RIGHT_SIGNAL,
    1000,
    2000
  );

  // Always begin stopped
  stopMotors();

  delay(1000);

  Serial.println();
  Serial.println("==============================");
  Serial.println("ESP32 AUTO CONTROL READY");
  Serial.println("==============================");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

  // -------------------------------------------------
  // Read all four ultrasonic sensors
  // -------------------------------------------------

  float distanceFront =
      measureDistance(sensorFront);

  delay(30);

  float distanceLeft =
      measureDistance(sensorLeft);

  delay(30);

  float distanceRight =
      measureDistance(sensorRight);

  delay(30);

  float distanceBack =
      measureDistance(sensorBack);

  delay(30);

  // -------------------------------------------------
  // Print sensor data
  // -------------------------------------------------

  Serial.printf(
    "Front: %.1f cm | Left: %.1f cm | "
    "Right: %.1f cm | Back: %.1f cm\n",
    distanceFront,
    distanceLeft,
    distanceRight,
    distanceBack
  );

  // -------------------------------------------------
  // Print encoder counts
  // -------------------------------------------------

  Serial.printf(
    "Encoder Right: %ld | Encoder Left: %ld\n",
    rightEncoderCount,
    leftEncoderCount
  );

  // =================================================
  // AUTONOMOUS OBSTACLE AVOIDANCE
  // =================================================

  // Obstacle directly in front
  // =================================================
// AUTONOMOUS OBSTACLE AVOIDANCE
// =================================================

// Front + Left + Right blocked
  if (
      distanceFront < FRONT_OBSTACLE_DISTANCE &&
      distanceLeft  < SIDE_OBSTACLE_DISTANCE &&
      distanceRight < SIDE_OBSTACLE_DISTANCE
  ) {

    // Back is clear, so reverse
    if (distanceBack > FRONT_OBSTACLE_DISTANCE) {

      Serial.println(
        "Front/Left/Right blocked -> moving BACKWARD"
      );

      moveBackward();

      delay(500);

      stopMotors();

      delay(100);
    }

    // Back is also blocked, so do not move
    else {

      Serial.println(
        "All directions blocked -> STOP"
      );

      stopMotors();

      delay(100);
    }
  }

  // Front obstacle
  else if (distanceFront < FRONT_OBSTACLE_DISTANCE) {

    stopMotors();

    delay(150);

    // Compare left and right distances
    if (distanceLeft > distanceRight) {

      Serial.println(
        "Front obstacle -> turning LEFT"
      );

      pivotLeft();

    } else {

      Serial.println(
        "Front obstacle -> turning RIGHT"
      );

      pivotRight();
    }

    // Perform a short pivot
    delay(400);

    stopMotors();

    delay(100);
  }

  // Obstacle on left side
  else if (distanceLeft < SIDE_OBSTACLE_DISTANCE) {

    Serial.println(
      "Left obstacle -> steering RIGHT"
    );

    slightRight();
  }

  // Obstacle on right side
  else if (distanceRight < SIDE_OBSTACLE_DISTANCE) {

    Serial.println(
      "Right obstacle -> steering LEFT"
    );

    slightLeft();
  }

  // Clear path
  else {

    Serial.println(
      "Path clear -> moving FORWARD"
    );

    moveForward();
  }

  delay(50);

}
