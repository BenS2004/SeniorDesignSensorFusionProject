#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

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

// =====================================================
// ENCODER VARIABLES
// =====================================================

volatile long rightEncoderCount = 0;
volatile long leftEncoderCount  = 0;

volatile int rightDirection = 1;
volatile int leftDirection  = 1;

volatile int prevRightA = LOW;
volatile int prevRightB = LOW;

volatile int prevLeftA = LOW;
volatile int prevLeftB = LOW;

// =====================================================
// ODEMETRY VARIABLES
// =====================================================

#define ENCODER_PULSES_PER_REV 36.0f
#define WHEEL_DIAMETER_CM 15.5f
#define WHEELBASE_CM 39.5f
#define PI 3.14159f
#define Right_Left_Sensor_Angle PI/6

volatile float rightWheelDistanceCM = 0.0;
volatile float leftWheelDistanceCM  = 0.0;

volatile float positionXCM = 0.0;
volatile float positionYCM = 0.0;

volatile float vectorAngle = 0.0f;
const float wheelCircumCM = PI*WHEELBASE_CM;

// =====================================================
// OBSTACLE SETTINGS
// =====================================================

const float FRONT_OBSTACLE_DISTANCE = 40.0;
const float SIDE_OBSTACLE_DISTANCE  = 25.0;

// =====================================================
//Robot State
// =====================================================

typedef enum RobotState {
  Free_State = 0,
  Wide_Turn_Left,
  Wide_Turn_Right,
  Tight_Turn_Left,
  Tight_Turn_Right,
  Reverse,
  Stop
} RobotState_t;

RobotState_t robotState = Free_State;
RobotState_t prevRobotState = Free_State;

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
  float rightwheelDistanceTraveled = (rightEncoderCount / ENCODER_PULSES_PER_REV) * (PI * WHEEL_DIAMETER_CM);
  positionXCM += rightwheelDistanceTraveled * cos(vectorAngle);
  positionYCM += rightwheelDistanceTraveled * sin(vectorAngle);
  rightWheelDistanceCM += rightwheelDistanceTraveled;

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
// Driving Commands
// =====================================================

void rotateLeft(int angle){
  float leftStartDistCM = leftWheelDistanceCM;
  float rightStartDistCM = rightWheelDistanceCM;

  float positionXStart = positionXCM;
  float positionYStart = positionYCM;

  float leftEndDistCM = -1*wheelCircumCM * angle / 360;
  leftEndDistCM = leftEndDistCM + leftWheelDistanceCM;
  float rightEndDistCM = wheelCircumCM * angle / 360;
  rightEndDistCM = rightEndDistCM + rightWheelDistanceCM;

  while(leftWheelDistanceCM > leftEndDistCM && rightWheelDistanceCM < rightEndDistCM){
    pivotLeft();
  };

  stopMotors();

  leftWheelDistanceCM = leftStartDistCM;
  rightWheelDistanceCM = rightStartDistCM;

  positionXCM = positionXStart;
  positionYCM = positionYStart;

  vectorAngle += (PI*angle/180);
  Serial.printf("Rotated left %d degrees\n", angle);
}

void rotateRight(int angle){
  float leftStartDistCM = leftWheelDistanceCM;
  float rightStartDistCM = rightWheelDistanceCM;

  float positionXStart = positionXCM;
  float positionYStart = positionYCM;

  float leftEndDistCM = wheelCircumCM * angle / 360;
  leftEndDistCM = leftEndDistCM + leftWheelDistanceCM;
  float rightEndDistCM = -1*wheelCircumCM * angle / 360;
  rightEndDistCM = rightEndDistCM + rightWheelDistanceCM;

  while(leftWheelDistanceCM < leftEndDistCM && rightWheelDistanceCM > rightEndDistCM){
    pivotRight();
  };

  stopMotors();

  leftWheelDistanceCM = leftStartDistCM;
  rightWheelDistanceCM = rightStartDistCM;
  positionXCM = positionXStart;
  positionYCM = positionYStart;

  vectorAngle -= (PI*angle/180);
  Serial.printf("Rotated right %d degrees\n", angle);
}

void collectDist(uint8_t* blockDir){
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
    "Encoder Right: %ld | Encoder Left: %ld\nLeft Wheel Distance: %.2f cm | Right Wheel Distance: %.2f cm\n",
    rightEncoderCount,
    leftEncoderCount,
    leftWheelDistanceCM,
    rightWheelDistanceCM
  );

  // =================================================
  // AUTONOMOUS OBSTACLE AVOIDANCE
  // =================================================

  // Obstacle directly in front
  // =================================================
  // AUTONOMOUS OBSTACLE AVOIDANCE
  // =================================================

  uint8_t frontBlocked = uint8_t(distanceFront < FRONT_OBSTACLE_DISTANCE);
  uint8_t leftBlocked  = uint8_t(distanceLeft  < SIDE_OBSTACLE_DISTANCE);
  uint8_t rightBlocked = uint8_t(distanceRight < SIDE_OBSTACLE_DISTANCE);
  uint8_t backBlocked  = uint8_t(distanceBack  < FRONT_OBSTACLE_DISTANCE);

  *blockDir = (frontBlocked << 3) | (leftBlocked << 2) | (rightBlocked << 1) | backBlocked;

  return;

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
    "Encoder Right: %ld | Encoder Left: %ld\nLeft Wheel Distance: %.2f cm | Right Wheel Distance: %.2f cm\n",
    rightEncoderCount,
    leftEncoderCount,
    leftWheelDistanceCM,
    rightWheelDistanceCM
  );

  // =================================================
  // AUTONOMOUS OBSTACLE AVOIDANCE
  // =================================================

  // Obstacle directly in front
  // =================================================
// AUTONOMOUS OBSTACLE AVOIDANCE
// =================================================

uint8_t frontBlocked = uint8_t(distanceFront < FRONT_OBSTACLE_DISTANCE);
uint8_t leftBlocked  = uint8_t(distanceLeft  < SIDE_OBSTACLE_DISTANCE);
uint8_t rightBlocked = uint8_t(distanceRight < SIDE_OBSTACLE_DISTANCE);
uint8_t backBlocked  = uint8_t(distanceBack  < FRONT_OBSTACLE_DISTANCE);
float temp_DistanceCM = 0;

uint8_t blockDir = (frontBlocked << 3) | (leftBlocked << 2) | (rightBlocked << 1) | backBlocked;

switch(robotState){
  case Free_State:
    switch(blockDir){
      case 0000:
      case 0001:
        robotState = Free_State;
        moveForward();
      break;

      case 0110:
      case 0111:
        temp_DistanceCM = (distanceRight < distanceLeft ? distanceRight : distanceLeft);
        temp_DistanceCM = 2 * cos(Right_Left_Sensor_Angle * PI / 180) * temp_DistanceCM;
        if(temp_DistanceCM > WHEELBASE_CM){
          moveForward();
        } else if (distanceRight < distanceLeft){
          prevRobotState = robotState;
          robotState = Wide_Turn_Left;
          rotateLeft(45);
          moveForward();
        } else {
          prevRobotState = robotState;
          robotState = Wide_Turn_Right;
          rotateRight(45);
          moveForward();
        }

      case 0011:
      case 0010:
        prevRobotState = robotState;
        robotState = Wide_Turn_Left;
        rotateLeft(45);
        temp_DistanceCM = (leftWheelDistanceCM + rightWheelDistanceCM)/2;
        moveForward();

      case 0101:
      case 0100:
        prevRobotState = robotState;
        robotState = Wide_Turn_Right; 
        rotateRight(45);
        temp_DistanceCM = (leftWheelDistanceCM + rightWheelDistanceCM)/2;
        moveForward();
      break;

      case 1011:
      case 1010:
        prevRobotState = robotState;
        robotState = Tight_Turn_Left;
        rotateLeft(90);
        temp_DistanceCM = (leftWheelDistanceCM + rightWheelDistanceCM)/2;
        moveForward();
      break;

      case 1101:
      case 1100:
        prevRobotState = robotState;
        robotState = Tight_Turn_Right;
        rotateRight(90);
        temp_DistanceCM = (leftWheelDistanceCM + rightWheelDistanceCM)/2;
        moveForward();
      break;

      case 1110:
        prevRobotState = robotState;
        robotState = Reverse;
        moveBackward();
      break;

      case 1111:
        prevRobotState = robotState;
        robotState = Stop;
        stopMotors();
      break;
    }
  break;

  case Wide_Turn_Left:
    if(((leftWheelDistanceCM + rightWheelDistanceCM)/2) >= (temp_DistanceCM + 300)) {
      stopMotors();
      rotateRight(45);
      collectDist(&blockDir);
      switch(blockDir){
        case 0000:
        case 0001:
        case 0100:
        case 0101:
          prevRobotState = robotState;
          robotState = Free_State;
          moveForward();
        break;

        case 1010:
        case 1011:
          prevRobotState = robotState;
          robotState = Tight_Turn_Left;
          temp_DistanceCM = (leftWheelDistanceCM + rightWheelDistanceCM)/2;
          rotateLeft(90);
          moveForward();
        break;

        default:
          temp_DistanceCM = (leftWheelDistanceCM + rightWheelDistanceCM)/2;
          rotateLeft(45);
          moveForward();
        break;
      }
    } else {
      switch(blockDir){
        case 1111:
          prevRobotState = robotState;
          robotState = Stop;
          stopMotors();
        break;

        case 1110:
          prevRobotState = robotState;
          robotState = Reverse;
          moveBackward();
        break;

        case 1100:
        case 1101:
          prevRobotState = robotState;
          robotState = Free_State;
          rotateRight(45);
          moveForward;
        break;

        case 1011:
        case 1010:
          prevRobotState = robotState;
          robotState = Tight_Turn_Left;
          rotateLeft(45);
          moveForward();
        break;

        default:
          moveForward();
        break;
      }
    }
  break;

  case Wide_Turn_Right:
    if(((leftWheelDistanceCM + rightWheelDistanceCM)/2) >= (temp_DistanceCM + 300)) {
      stopMotors();
      rotateLeft(45);
      collectDist(&blockDir);
      switch(blockDir){
        case 0000:
        case 0001:
        case 0010:
        case 0011:
          prevRobotState = robotState;
          robotState = Free_State;
          moveForward();
        break;

        case 1100:
        case 1101:
          prevRobotState = robotState;
          robotState = Tight_Turn_Right;
          leftWheelDistanceCM = (leftWheelDistanceCM + rightWheelDistanceCM)/2;
          rotateRight(90);
          moveForward();
        break;

        default:
          temp_DistanceCM = (leftWheelDistanceCM + rightWheelDistanceCM)/2;
          rotateRight(45);
          moveForward();
        break;
      }
    } else {
      switch(blockDir){
        case 1111:
          prevRobotState = robotState;
          robotState = Stop;
          stopMotors();
        break;

        case 1110:
          prevRobotState = robotState;
          robotState = Reverse;
          moveBackward();
        break;

        case 1010:
        case 1011:
          prevRobotState = robotState;
          robotState = Free_State;
          rotateLeft(45);
          moveForward();
        break;

        case 1101:
        case 1100:
          prevRobotState = robotState;
          robotState = Tight_Turn_Right;
          rotateRight(45);
          moveForward();
        break;

        default:
          moveForward();
        break;
      }
    }
  break;

  case Tight_Turn_Left:
    if(((leftWheelDistanceCM + rightWheelDistanceCM)/2) >= (temp_DistanceCM + 300)){
      stopMotors();
      rotateRight(90);
      collectDist(&blockDir);
      switch(blockDir){
        case 0000:
        case 0001:
        case 0100:
        case 0101:
          prevRobotState = robotState;
          robotState = Free_State;
          moveForward();
        break;

        default:
          temp_DistanceCM = (leftWheelDistanceCM + rightWheelDistanceCM)/2;
          rotateLeft(90);
          moveForward();
        break;
      }
    } else {
      switch(blockDir){
        case 1111:
          prevRobotState = robotState;
          robotState = Stop;
          stopMotors();
        break;

        case 1110:
          prevRobotState = robotState;
          robotState = Reverse;
          moveBackward();
        break;

        case 1100:
        case 1101:
          prevRobotState = robotState;
          robotState = Wide_Turn_Left;
          rotateRight(45);
          moveForward();
        break;
        
        case 1010:
        case 1011:
          prevRobotState = robotState;
          robotState = Tight_Turn_Right;
          rotateRight(180);
          moveForward();
        break;
        
        default:
          moveForward();
        break;
      }
    }
  break;

  case Tight_Turn_Right:
    if(((leftWheelDistanceCM + rightWheelDistanceCM)/2) >= (temp_DistanceCM + 300)){
      stopMotors();
      rotateLeft(90);
      collectDist(&blockDir);
      switch(blockDir){
        case 0000:
        case 0001:
        case 0010:
        case 0011:
          prevRobotState = robotState;
          robotState = Free_State;
          moveForward();
        break;

        default:
          temp_DistanceCM = (leftWheelDistanceCM + rightWheelDistanceCM)/2;
          rotateRight(90);
          moveForward();
        break;
      }
    } else {
      case 1111:
        prevRobotState = robotState;
        robotState = Stop;
        stopMotors();
      break;

      case 1110:
        prevRobotState = robotState;
        robotState = Reverse;
        moveBackward();
      break;

      case 1010:
      case 1011:
        prevRobotState = robotState;
        robotState = Wide_Turn_Right;
        rotateLeft(45);
        moveForward();
      break;
      
      case 1100:
      case 1101:
        prevRobotState = robotState;
        robotState = Tight_Turn_Left;
        rotateLeft(180);
        moveForward();
      break;

      default:
        moveForward();
      break;
    }
  break;

  case Reverse:
    switch(blockDir){
      case 1111:
        prevRobotState = robotState;
        robotState = Stop;
        stopMotors();
      break;

      case 1110:
        moveBackward();
      break;

      default:
        switch(prevRobotState){
          case Free_State:
            if (distanceRight < distanceLeft){
              prevRobotState = robotState;
              robotState = Wide_Turn_Left;
              rotateLeft(45);
              moveForward();
            }else{
              prevRobotState = robotState;
              robotState = Wide_Turn_Right;
              rotateRight(45);
              moveForward();
            }
          break;

          case Wide_Turn_Left:
            prevRobotState = robotState;
            robotState = Tight_Turn_Left;
            rotateLeft(45);
            moveForward();
          break;

          case Wide_Turn_Right:
            prevRobotState = robotState;
            robotState = Tight_Turn_Right;
            rotateRight(45);
            moveForward();
          break;

          default:
            prevRobotState = robotState;
            robotState = Free_State;
            moveForward();
        }
    }
  break;

  case Stop:
    switch(blockDir){
      case 1111:
      break;

      case 1110:
      case 1100:
      case 1010:
        prevRobotState = robotState;
        robotState = Reverse;
        moveBackward();
      break;
      
      case 1001:
      case 1101:
        prevRobotState = robotState;
        robotState = Tight_Turn_Right;
        rotateRight(90);
        moveForward();
      break;

      case 1011:
        prevRobotState = robotState;
        robotState = Tight_Turn_Left;
        rotateLeft(90);
        moveForward();

      default:
        prevRobotState = robotState;
        robotState = Free_State;
        moveForward();
      break;
    }

  break;
}

delay(50);

}
