
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <esp32-hal-ledc.h>
// ===== I2CLCD Pins =====
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip

// ===== Quadrature Encoder Pins =====
#define ENC_AR 34  // Quadrature encoder A pin
#define ENC_BR 35  // Quadrature encoder B pin
#define ENC_AL 21  // Quadrature encoder A pin
#define ENC_BL 22  // Quadrature encoder B pin

// ===== Motor Control Pins =====
#define LED_PIN 13
Servo motorA;
Servo motorB;
#define MAX_SPEED 180
#define MID_SPEED 130
#define MIN_SPEED 50
// Right Motor (Motor A)  
#define MotorA_pin 25 
#define R_ENa 23
#define L_ENa 27

// Left Motor (Motor B)
#define MotorB_pin 26
#define R_ENb 31 
#define L_ENb 32  

// ===== Ultrasonic Sensor Pins =====
struct UltrasonicSensor {
  uint8_t trigPin;
  uint8_t echoPin;
};

UltrasonicSensor sensorFront = {19, 18};  // Front sensor
UltrasonicSensor sensorLeft  = {23, 5};   // Left sensor
UltrasonicSensor sensorRight = {15, 17};  // Right sensor
UltrasonicSensor sensorBack = {2, 4};     // Back sensor 

// LCD geometry
const int LCD_COLS = 16;
const int LCD_ROWS = 2;

// ===== Constants =====
const int OBSTACLE_THRESHOLD = 40; // Reduced for faster reaction
const int OBSTACLE_THRESHOLD_SIDE = 20;
const int WARNING_DISTANCE = 60;   // Reduced for faster reaction
const int SAFE_DISTANCE = 30;      // Backup threshold
const int MOTOR_SPEED = 150;       // Speed level (0-255)
const int TURN_SPEED_SLIGHT = 100; // Speed for slight turns
const int TURN_SPEED_LARGE = 200;  // Speed for larger turns
const int TURN_SPEED_FAST = 250;   // Speed for pivot turns
const float SPEED_OF_SOUND = 0.0343; // Speed of sound in cm/Î¼s

//===== parse variables =====
String commandBuffer = "";
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT_MS = 3000;
enum State {
  STOP, FORWARD, BACKWARD, SLIGHT_TURN, LARGE_TURN
};
int motorSpeed = 180;
State currentState = STOP;
bool turndir;
bool fordir;

// ===== GLOBAL Variables =====
int flagFront = 0;
int flagLeft = 0;
int flagRight = 0;

// ===== Quadrature Encoder Variables =====
volatile long positionR = 0;
volatile int directionR = 1;
volatile long positionL = 0;
volatile int directionL = 1;

volatile unsigned long lastTimeR = 0;
volatile unsigned long lastTimeL = 0;

volatile int AnumCycles = 0;
volatile int BnumCycles = 0;

volatile float totDistanceL = 0.0;
volatile float totDistanceR = 0.0;
volatile float speedR = 0.0;
volatile float speedL = 0.0;
unsigned long pulseCountR = 0;
unsigned long pulseCountL = 0;
const float WHEEL_CIRCUMFERENCE = 18.5; // In inches
const int ENCODER_RESOLUTION = 36; 
const float timeInterval = 100.0;

volatile int prevEncAR = 0;
volatile int prevEncBR = 0;

volatile int prevEncAL = 0;
volatile int prevEncBL = 0;
int i = 7;

//----------------NEW MOTOR VARS-----------------------
const int REVERSE_VAL = 1300;
const int STOP_VAL = 1500;
const int FORWARD_VAL = 1700;
const int FASTFORWARD_VAL = 1900;
const int TURN_VAL = 140;
const int PWM_FREQ = 50;
const int PWM_RESOLUTION = 16;
const int MOTOR1_CHANNEL = 0;
const int MOTOR2_CHANNEL = 1;

void IRAM_ATTR encoderISR_R() {
    int encA = digitalRead(ENC_AR);
    int encB = digitalRead(ENC_BR);

    if ((prevEncAR == LOW) && (encA == HIGH)) {  // Rising edge on ENC_A
        directionR = (encB == LOW) ? 1 : -1;  // CW if B is LOW, CCW if B is HIGH
    } 
    else if ((prevEncBR == LOW) && (encB == HIGH)) { // Rising edge on ENC_B
        directionR = (encA == HIGH) ? 1 : -1;
    }

    positionR += directionR;
    pulseCountR++;
    prevEncAR = encA;
    prevEncBR = encB;
}

void IRAM_ATTR encoderISR_L() {
    int encAL = digitalRead(ENC_AL);
    int encBL = digitalRead(ENC_BL);

    if ((prevEncAL == LOW) && (encAL == HIGH)) {  // Rising edge on ENC_A
        directionL = (encBL == LOW) ? 1 : -1;  // CW if B is LOW, CCW if B is HIGH
    } 
    else if ((prevEncBL == LOW) && (encBL == HIGH)) { // Rising edge on ENC_B
        directionL = (encAL == HIGH) ? 1 : -1;
    }

    positionL += directionL;
    pulseCountL++;
    prevEncAL = encAL;
    prevEncBL = encBL;
}

float calculateSpeedR(volatile unsigned long &pulseCount) {
    unsigned long currentTime = millis();
    unsigned long timeElapsed = currentTime - lastTimeR;

    if (timeElapsed < timeInterval) {
        return -1;  // Avoid calculating speed too frequently
    }

    if (pulseCount == 0) {
        lastTimeR = currentTime;
        return 0;  // No movement detected
    }

    float distanceR = (pulseCount * (WHEEL_CIRCUMFERENCE / ENCODER_RESOLUTION));  // Distance in cm
    float speed = (pulseCount * (WHEEL_CIRCUMFERENCE / ENCODER_RESOLUTION)) / (timeElapsed / 1000.0);  // Convert ms to s for cm/s

    totDistanceR += distanceR;
    // Reset pulse count and update lastTime
    pulseCount = 0;
    lastTimeR = currentTime;

    return speed;
}

float calculateSpeedL(volatile unsigned long &pulseCount) {
    unsigned long currentTime = millis();
    unsigned long timeElapsed = currentTime - lastTimeL;

    if (timeElapsed < timeInterval) {
        return -1;  // Avoid calculating speed too frequently
    }

    if (pulseCount == 0) {
        lastTimeL = currentTime;
        return 0;  // No movement detected
    }

    float distanceL = (pulseCount * (WHEEL_CIRCUMFERENCE / ENCODER_RESOLUTION));  // Distance in cm
    float speed = (pulseCount * (WHEEL_CIRCUMFERENCE / ENCODER_RESOLUTION)) / (timeElapsed / 1000.0);  // Convert ms to s for cm/s
    
    totDistanceL += distanceL;
    // Reset pulse count and update lastTime
    pulseCount = 0;
    lastTimeL = currentTime;
    return speed;
}
//------------------------NEW MOTOR FUNCTIONS------------------------------------------
// ===== Function to Set Motor Speed =====
void setMotorSpeed(int pinout, int pulseWidth) {
  int duty = calculateDuty(pulseWidth);
  ledcWrite(pinout, duty);
}

int calculateDuty(int pulseWidth){
  int maxDuty = (1 << PWM_RESOLUTION) -1;
  return (pulseWidth * maxDuty) / 20000;
}
//---------------------------parse command new functions--------------------------
// ===== Parse Command =====
void parseCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  int commaIndex = cmd.indexOf(',');
  String dir = (commaIndex > 0) ? cmd.substring(0, commaIndex) : cmd;
  String speedStr = (commaIndex > 0) ? cmd.substring(commaIndex + 1) : "";

  if (speedStr.length() > 0) {
    int newSpeed = speedStr.toInt();
    if (newSpeed >= 0 && newSpeed <= 255) {
      motorSpeed = newSpeed;
      Serial.printf("Updated speed: %d\n", motorSpeed);
    } else {
      Serial.println("Invalid speed");
    }
  }

  if (dir == "F") {currentState = FORWARD; fordir = true;}
  else if (dir == "S") currentState = STOP;
  else if (dir == "SL" || dir == "SR") currentState = SLIGHT_TURN;
  else if (dir == "SL" || dir == "HL") turndir = true;
  else if (dir == "HL" || dir == "HR") currentState = LARGE_TURN;
  else if (dir == "SR" || dir == "HR") turndir = false;
  else if (dir == "B") {currentState = BACKWARD; fordir = false;}
  else {
    Serial.println("Unknown command");
    return;
  }

  // Acknowledge
  Serial.print("ACK:");
  Serial.print(dir);
  if (speedStr.length() > 0) {
    Serial.print(",");
    Serial.println(motorSpeed);
  } else {
    Serial.println();
  }

  lastCommandTime = millis();
}

// ===== Read Serial Command =====
void readCommandFromPi() {
  while (Serial.available()) {
    char ch = Serial.read();

    if (ch == '\n' || ch == '\r') {
      if (commandBuffer.length() > 0) {
        parseCommand(commandBuffer);
        commandBuffer = "";
      }
    } else if (isPrintable(ch)) {
      commandBuffer += ch;
    }
  }
}

// ===== Print State =====
void printState() {
  switch (currentState) {
    case STOP: Serial.println("STATE: STOP"); break;
    case FORWARD: Serial.println("STATE: FORWARD"); break;
    case BACKWARD: Serial.println("STATE: BACKWARD"); break;
    case SLIGHT_TURN: Serial.println("STATE: SLIGHT_TURN"); break;
    case LARGE_TURN: Serial.println("STATE: LARGE_TURN"); break;
  }
}

//===== FSM Update =====
void updateState() {
  printState();
  switch (currentState) {
    case STOP:
      gradualStop(fordir);
      break;
    case FORWARD:
      moveForward();
      break;
    case BACKWARD:
      moveBackward();
      break;
    case SLIGHT_TURN:
      slightTurn(turndir);
      delay(300);
      currentState = FORWARD;
      break;
    case LARGE_TURN:
      largeTurn(turndir);
      delay(300);
      currentState = FORWARD;
      break;
  }
}
//--------------------------------------------------------------------------------


// ===== Move Forward =====
void moveForward() {
    Serial.println("Moving Forward...");
    motorA.write(MAX_SPEED);
    motorB.write(MAX_SPEED);
}

// ===== Move Backward =====
void moveBackward() {
    Serial.println("Moving Backward...");
    motorA.write(MID_SPEED);
    motorB.write(MID_SPEED);
}

// ===== Slight Turn =====
void slightTurn(bool turnLeft) {
    if (turnLeft) {
        Serial.println("Slight Turn Left...");
        motorA.write(MAX_SPEED);
        motorB.write(MID_SPEED);
    } else {
        Serial.println("Slight Turn Right...");
        motorA.write(MID_SPEED);
        motorB.write(MAX_SPEED);
    }
}

// ===== Large Turn =====
void largeTurn(bool turnLeft) {
    if (turnLeft) {
        Serial.println("Large Turn Left...");
        motorA.write(MAX_SPEED);
        motorB.write(MIN_SPEED);
    } else {
        Serial.println("Large Turn Right...");
        motorA.write(MIN_SPEED);
        motorB.write(MAX_SPEED);
    }
}

// ===== Pivot Turn =====
void pivotTurn(bool turnLeft) {
  Serial.println(turnLeft ? "Pivot Turn Left..." : "Pivot Turn Right...");
  unsigned long startTime = millis(); // Record the start time of the pivot turn

  while (true) {
    // Measure the front distance during the pivot turn
    float distanceFront = measureDistance(sensorFront);
    float distanceLeft = measureDistance(sensorLeft);
    float distanceRight = measureDistance(sensorRight);
    // Check if the front sensor no longer detects an obstacle
    if (!isObstacleTooClose(distanceFront) && !isSideObstacleTooClose(distanceLeft) && !isSideObstacleTooClose(distanceRight)) {
      Serial.println("Front obstacle cleared. Ending pivot turn.");
      stopMotor(); // Stop the motors
      break; // Exit the pivot turn loop
    }

    // Perform the pivot turn logic
    if (turnLeft) {
      motorA.write(MAX_SPEED);
      motorB.write(0);
    } else {
      motorA.write(0);
      motorB.write(MAX_SPEED);
    }

    // Small delay to avoid excessive CPU usage
    delay(10);
  }
}

// ===== Stop Motors =====
void stopMotor() {
    Serial.println("Stopping...");
    motorA.write(0);
    motorB.write(0);
}

// ===== Measure Distance Using Ultrasonic Sensor =====
float measureDistance(UltrasonicSensor sensor) {
    pinMode(sensor.trigPin, OUTPUT);
    pinMode(sensor.echoPin, INPUT);
  
    digitalWrite(sensor.trigPin, LOW);
    delayMicroseconds(5);
  
    digitalWrite(sensor.trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensor.trigPin, LOW);
  
    long duration = pulseIn(sensor.echoPin, HIGH, 30000);
    return duration == 0 ? 400 : (duration * SPEED_OF_SOUND) / 2.0;
}

// ===== Check for Obstacles =====
bool isObstacleTooClose(float distance) {
    return distance < OBSTACLE_THRESHOLD;
}

bool isSideObstacleTooClose(float distance) {
    return distance < OBSTACLE_THRESHOLD_SIDE;
}

// ===== Gradual Stop with Faster Decay =====
void gradualStop(bool forward) {
    Serial.println(forward ? "Gradually Stopping..." : "Gradually Stopping Backwards...");
    unsigned long startTime = millis();
    float currentSpeed = FORWARD_VAL;

    while (currentSpeed > 0) {
        float elapsedTime = (millis() - startTime) / 1000.0; // Time in seconds
        currentSpeed = FORWARD_VAL * exp(-elapsedTime * 4); // Faster exponential decay (multiplier = 5)

        if (forward) {
            for(int posDegrees = 180; posDegrees >= 0; posDegrees--) {
              motorA.write(posDegrees);
              motorB.write(posDegrees);
              delay(20);
            }
        } else {
            for(int posDegrees = 180; posDegrees >= 0; posDegrees--) {
              motorA.write(posDegrees);
              motorB.write(posDegrees);
              delay(20);
            }
        }
        currentSpeed = round(currentSpeed * 100) / 100;
        delay(10); // Small delay for smooth deceleration
    }
    stopMotor(); // Ensure motors are fully stopped
}

// ===== Update Turning Flags with Earlier Turning =====
void updateFlags(float distanceFront, float distanceLeft, float distanceRight) {
    // Initialize flags with 0 (no obstacle)
    flagFront = 0;
    flagLeft = 0;
    flagRight = 0;

    // Set front flag
    if (distanceFront < OBSTACLE_THRESHOLD) {
        flagFront = 2;
    } else if (distanceFront < WARNING_DISTANCE) {
        flagFront = 1;
    }

    // Set left flag
    if (distanceLeft < OBSTACLE_THRESHOLD_SIDE) {
        flagLeft = 2;
    } else if (distanceLeft < WARNING_DISTANCE) {
        flagLeft = 1;
    }

    // Set right flag
    if (distanceRight < OBSTACLE_THRESHOLD_SIDE) {
        flagRight = 2;
    } else if (distanceRight < WARNING_DISTANCE) {
        flagRight = 1;
    }

    // Display flags on the LCD
    lcd.setCursor(0, 1);  // Move cursor to the second row
    lcd.print(flagLeft);
    lcd.print(flagFront);
    lcd.print(flagRight);
}

// ===== Setup Function =====
void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);

    // LCD Start-up Code
    lcd.begin(LCD_COLS, LCD_ROWS);
    lcd.clear();
    lcd.print("LF, FF, RF");

    // Encoder Setup
    pinMode(ENC_AR, INPUT_PULLUP);
    pinMode(ENC_BR, INPUT_PULLUP);
    pinMode(ENC_AL, INPUT_PULLUP);
    pinMode(ENC_BL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_AR), encoderISR_R, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_BR), encoderISR_R, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_AL), encoderISR_L, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_BL), encoderISR_L, CHANGE);

    // Motor Control Setup
    //ledcSetup(MOTOR1_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    //ledcSetup(MOTOR2_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    //ledcAttach(MotorA_pin, PWM_FREQ, PWM_RESOLUTION);
    //ledcAttach(MotorB_pin, PWM_FREQ, PWM_RESOLUTION);
    motorA.attach(MotorA_pin);
    motorB.attach(MotorB_pin);

    pinMode(R_ENa, OUTPUT);
    pinMode(L_ENa, OUTPUT);
    pinMode(R_ENb, OUTPUT);
    pinMode(L_ENb, OUTPUT);

    // pinMode(RPWMa, OUTPUT);
    // pinMode(LPWMa, OUTPUT);
    // pinMode(RPWMb, OUTPUT);
    // pinMode(LPWMb, OUTPUT);

    digitalWrite(R_ENa, HIGH);
    digitalWrite(L_ENa, HIGH);
    digitalWrite(R_ENb, HIGH);
    digitalWrite(L_ENb, HIGH);

    stopMotor();
}

// ===== Obstacle Avoidance Function =====
void obstacleAvoidance(float front, float left, float right) {
    if (flagFront > 0) {
        (flagRight < flagLeft) ? largeTurn(false) : largeTurn(true);
    } else if (left < WARNING_DISTANCE) {
        slightTurn(false);
    } else if (right < WARNING_DISTANCE) {
        slightTurn(true);
    } else {
        moveForward();
    }
    return;
}

// ===== Loop Function =====
void loop() {
  static unsigned long lastSensorRead = 0;
  static unsigned long lastStateChange = 0;
  //static enum State { FORWARD, AVOIDING, STOP, REVERSE, SPIN } currentState = FORWARD;

  // Measure Distances Every 100ms
  if (millis() - lastSensorRead >= 100) {
    float distanceFront = measureDistance(sensorFront);
    float distanceLeft  = measureDistance(sensorLeft);
    float distanceRight = measureDistance(sensorRight);
    float distanceBack = measureDistance(sensorBack);

    // Print Sensor Data
    Serial.printf("Front: %.1f cm | Left: %.1f cm | Right: %.1f cm | Back: %.1f cm\n", 
                  distanceFront, distanceLeft, distanceRight, distanceBack);

    // Update Turning Flags
    updateFlags(distanceFront, distanceLeft, distanceRight);

    readCommandFromPi();
    updateState();
    lastSensorRead = millis();
  }
  Serial.printf("Direction_R: %s\n", directionR == 1 ? "CW" : "CCW");     
  Serial.printf("Direction_L: %s\n", directionL == 1 ? "CW" : "CCW");

  float speedR = calculateSpeedR(pulseCountR);  // Right wheel speed
  float speedL = calculateSpeedL(pulseCountL);  // Left wheel speed

  // Output the speeds
  if (speedR >= 0 || speedL >= 0){
    Serial.printf("Right Wheel Speed: %.2f cm/s\n", speedR);
    Serial.printf("Left Wheel Speed: %.2f cm/s\n", speedL);
  }

  Serial.printf("Total Distance Traveled R: %.2f cm\n", totDistanceR);
  Serial.printf("Total Distance Traveled L: %.2f cm\n", totDistanceL);
  
  if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS && currentState != STOP) {
    Serial.println("Timeout reached, stopping motors");
    currentState = STOP;
    lastCommandTime = millis();
  }
}