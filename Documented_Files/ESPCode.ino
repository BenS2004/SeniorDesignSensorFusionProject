#include <Arduino.h>
#include <Wire.h>
#include "Motor.h"
#include "encoders.h"
#include "Odometry.h"
#include "DFRobot_MatrixLidar.h"

static const float WHEELBASE_M = 0.30f;
static const float V_MAX_MPS = 0.60f;
static const int PWM_MAX = 1023;
static const int PWM_DEADBAND = 120;
static const uint32_t CMD_TIMEOUT_MS = 500;
static const int SLEW_PER_TICK = 35;
static const uint32_t CTRL_HZ = 50;

// Motors
Motor leftMotor(M1_PWM, M1_INA, M1INB.
M1_EN, 0);
Motor rightMotor(M2_PWM, M2_INA, M2_INB,
M2_EN,1);

// Encoders
Encoder leftEnc(L_ENCA, L_ENCB, flase);
Encoder rightENC(R_ENCA, R_ENCB, true);
Odometry odom(&leftENC, &rightENC);

//ToF / I2C
#define SDA_PIN 1
#define SCL_PIN 2
DFRobot_MatrixLidar_I2C tofFront(0x30, &Wire);

//Ultrasonics
struct UltrasonicSensor {
    uint8_t trigPin;
    uint8_t echoPin;
};

UltrasonicSensor usFront = {14, 15};
UltrasonicSensor usLeft = {6, 3};
UltrasonicSensor usRight = {38, 39};
UltrasonicSensor usBack = {8, 9};

//ToF publish rate
static const uint32_t TOF_HZ = 10;
static const uint32_t ULTRA_HZ = 10;

// Row bands
static const int FAR_ROW_START = 0;
static const int FAR_ROW_END = 1;
static const int NEAR_ROW_START = 2;
static const int NEAR_ROW_END = 4;

// Valud ranges
static const float VALID_MIN_CM = 10.0f;
static const float FAR_VALID_MAX_CM = 300.0f;
static const float NEAR_VALID_MAX_CM = 180.0f;
static const float ULTRA_VALID_MAX_CM = 250.0f;
static const float ULTRA_NO_ECHO_CM = 400.0f;

//Filter tuning
static const float ALPHA_CENTER = 0.35f;
static const float ALPHA_NEAR_AVG = 0.25f;
static const float ALPHA_FAR_AVG = 0.25f;
static const float ALPHA_ULTRA = 0.35f;

static String line;
static uint32_t last_cmd_ms = 0;

static float v_cmd = 0.0f;
static float w_cmd = 0.0f;

static int dutyL = 0;
static int dutyR = 0;
struct TofSummary{
    bool valid;
    float centerCM;
    float nearMinCM;
    float nearAvgCM;
    float farAvgCM;
};

struct FilterState {
    bool initialized;
    float centerCM;
    float nearMinCM;
    float nearAvgCM;
    float farAvgCM;
    float nearMinHist[3];
    int nearMinIdx;
    int nearMinCount;
};

struct UltrasonicFilterState{
    bool initialized;
    float frontCM;
    float leftCM;
    float rightCM;
    float backCM;
};

static FilterState frontFilter = 
{false, 0, 0, 0, 0, {0, 0, 0}, 0, 0};

static UltrasonicFilterState ultraFilter = 
{false, ULTRA_NO_ECHO_CM, ULTRA_NO_ECHO_CM, ULTRA_NO_ECHO_CM, ULTRA_NO_ECHO_CM};

static uint32_t last_tof_pub = 0;
static uint32_t last_ultra_pub = 0;

static uint32_t slewTo(int current, int target){
    int delta = target - current;
    if(delta > SLEW_PER_TICK) delta = SLEW_PER_TICK;
    if(delta < -SLEW_PER_TICK) delta = -SLEW_PER_TICK;
    
    return current + delta;
}

static int wheelSpeedToDuty(float v_wheel_mps){
    float x = fabsf(v_wheel_mps) / V_MAX_MPS;
    int duty = (int)(x * PWM_MAX);
    if(duty > 0 && duty < PWM_DEADBAND) duty = PWM_DEADBAND;
    if(duty > PWM_MAX) duty = PWM_MAX;
    return duty;
}

static void applyMotor(Motor& m, float v_wheel_mps, int duty){
    if(duty <= 0 || fabsf(v_wheel_mps) < 1e-3f){
        m.stop();
        return;
    }
    if(v_wheel_mps >= 0.0f) m.forward(duty);
    else m.reverse(duty);
}

static void stopRobot(){
    v_cmd = 0.0f;
    w_cmd = 0.0f;
    dutyL = 0;
    leftMotor.stop();
    rightMotor.stop();
}

static void handleLine(const String& s){
    digitalWrite(2, !digitalRead(2));

    float v, w;
    if(sscanf(s.c_str(), "CMD v %f w %f", &v, &w) == 2){
        v_cmd = v;
        w_cmd = w;
        last_cmd_ms = millis();
    }
}

static float median3(float a, float b, float c){
    if(a > b) {float t = a; a = b; b = t;}
    if(b > c) {float t = b; b = c; c = t;}
    if(a > b) {float t = a; a = b; b = t;}
}

static float ema(float prev, float x, float alpha){
    return alpa * x + (1.0f - alpha) * prev;
}

static float getCenterAverageCM(const uint16_t* frame){
    const int idx[4] = {27, 28, 35, 36};
    float sum = 0.0f;
    int count = 0;

    for(int i = 0; i < 4; i++){
        uint16_t v = frame[idx[i]];
        if(v > 0){
            sum += v / 10.0f;
            count++;
        }
    }

    if(count == 0 ) return 400.0f;
    return sum / count;
}

static bool readFrame(DFRobot_MatrixLidar_I2C &tof, 
uint16_t frame[64]){
    return (tof.getALLData(frame) == 0);
}

static TofSummmary summarizeFrame(const uint16_t frame[64], 
bool validRead){
    TofSummary s{};

    if(!valid Read){
        s.valid = false;
        s.centerCM = 400.0f;
        s.nearMinCM = 300.0f;
        s.nearAvgCM = 300.0f;
        s.farAvgCM = 300.0f;
        return s;
    }

    s.valid = true;
    s.centerCM = getCenterAverageCM(frame);
    s.nearMinCM = 300.0f;
    s.nearAvgCM = 300.0f;
    s.farAvgCM = 300.0f;

    float farSum = 0.0f;
    int farCount = 0;
    float nearSum = 0.0;
    int nearCount = 0;

    for(int i = 0; i < 64; i++){
        uint16_t raw = frame[i];
        if(raw == 0) continue;
        float cm = raw /10.0f;
        int row = i/8;

        if(row >= FAR_ROW_START 
        && row <= FAR_ROW_END){
            if(cm >= VALID_MIN_CM 
            cm <= FAR_VALID_MAX_CM){
                farSum += cm;
                farCount++;
            }
        }

        if(row >= NEAR_ROW_START 
        && row <= NEAR_ROW_END){
            if(cm >= VALID_MIN_CM 
            cm <= NEAR_VALID_MAX_CM){
                nearSum += cm;
                nearCount++;
                if(cm < s.nearMinCM) {
                    s.nearMinCM = cm;
                }
            }
        }
    }
    if (farCount > 0) s.farAvgCM = farSum / farCount;
    if(nearCount > 0) s.nearAvgCM = nearSum / nearCount;
    else s.nearMinCM = 300.0f;
    return s;
}

static TofSummary filterSummary(const TofSummary &raw, FilterState $fs){
    TofSummary out = raw;

    if(!raw.valid){
        return out;
    }
    if(!fs.initialized){
        fs.initialized = true;
        fs.centerCM = raw.centerCM;
        fs.nearMinCM = raw.nearMinCM;
        fs.nearAvgCM = raw.nearAvgCM;
        fs.farAvgCM = raw.farAvgCM;

        fs.nearMinHist[0] = raw.nearMinCM;
        fs.nearMinHist[1] = raw.nearMinCM;
        fs.nearMinHist[2] = raw.nearMinCM;
        fs.nearMinIdx = 1;
        fs.nearMinCount = 1;
    } else {
        fs.centerCM = ema(fs.centerCM, raw.centerCM, ALPHA_CENTER);
        fs.nearAvgCM = ema(fs.nearAvgCM, raw.nearAvgCM, ALPHA_NEAR_AVG);
        fs.farAvgCM = ema(fs.farAvgCM, raw.farAvgCM, ALPHA_FAR_AVG);
    
        fs.nearMinHist[fs.nearMinIdx] = raw.nearMinCM;
        fs.nearMinIdx = (fs.nearMinIdx + 1) % 3;
        if(fs.nearMinCount < 3) fs.nearMinCount++;
    }

    if(fs.nearMinCount == 1){
        fs.nearMinCM = median3(fs.nearMinHist[0], 
            fs.nearMinHist[1], fs.nearMinHist[2]);
    }

        out.centerCM = fs.centerCM;
        out.nearMinCM = fs.nearMinCM;
        out.nearAvgCM = fs.nearAvgCM;
        out.farAvgCM = fs.farAvgCM;

        return out;
}

// Ultrasonic helper functions
static bool valifUltrasonic(float cm){
    return cm >= VALID_MIN_CM && cm <= ULTRA_VALID_MAX_CM;
}

static float sanitizeUltrasonic(float cm){
    if(!validUltrasonic(cm)){
        return UTRA_NO_ECHO_CM;
    }
    return cm;
}

static float measureDistanceCM(const UltrasonicSensor &s){
    digitalWrite(s.trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(s.trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(s.trigPin, LOW);

    unsigned long duration = pulseIn(s.echoPin, HIGH, 30000);
    if(duration == 0) return ULTRA_NO_ECHO_CM;
    float cm = duration * 0.0343f / 2.0f;
    return sanitizeUltrasonic(cm);
}

static UltrasonicSummary readUltrasonics(){
    UltrasonicSummary s{};

    s.frontCM = measureDistanceCM(usFront);
    s.leftCM = measureDistanceCM(usLeft);
    s.rightCM = measureDistanceCM(usRight);
    s.backCM = measureDistanceCM(usBack);

    s.validFront = validUltrasonic(s.frontCM);
    s.validLeft = validUltrasonic(s.leftCM);
    s.validRight = validUltrasonic(s.rightCM);
    s.validBack = valifUltrasonic(s.backCM);

    return s;
}

static UltrasonicSummary filterUltrasonics(const UltrasonicSummary &raw, UltrasonicFilterState &fs){
    UltrasonicSummary out = raw;

    if(!fs.initialized){
        fs.initialized = true;
        fs.frontCM = raw.frontCM;
        fs.leftCM = raw.leftCM;
        fs.rightCM = raw.rightCM;
        fs.backCM = raw.backCM;
    } else {
        fs.frontCM = ema(fs.frontCM, raw.frontCM, ALPHA_ULTRA);
        fs.leftCM = ema(fs.leftCM, raw.leftCM, ALPHA_ULTRA);
        fs.rightCM = ema(fs.rightCM, raw.rightCM, ALPHA_ULTRA);
        fs.backCM = ema(fs.backCM, raw.backCM, ALPHA_ULTRA);
    }

    out.frontCM = fs.frontCM;
    out.leftCM = fs.leftCM;
    out.rightCM = fs.rightCM;
    out.backCM = fs.backCM;

    out.validFront = validUltrasonic(out.frontCM);
    out.validLeft = validUltrasonic(out.leftCM);
    out.validRight = validUltrasonic(out.rightCM);
    out.validBack = validUltrasonic(out.backCM);

    return out;
}

static void publishTofSummary(const TofSummary &s){
    Serial.printf("TOF|ID:FRONT|T:%lu|VALID:%d|CENTER:%.1f|NEAR_MIN:%.1f|NEAR_AVG:%.1f|FAR_AVG:%.1f\n",
    millis(), s.valid ? 1:0, s.centerCM, s.nearMinCM, s.nearAvgCM, s.farAvgCM);
}

static void publishUltrasonicSummary(const UltrasonicSummary &s){
    Serial.printf("ULTRA|ID:ALL|T:%lu|FRONT:%.1f|LEFT:%.1f|RIGHT:%.1f|BACK:%.1f
        |VALID_FRONT:%d|VALID_LEFT:%d|VALID_RIGHT:%d|VALID_BACK:%d\n",
    millis(), s.frontCM, s.leftCM, s.rightCM, s.backCM, s.validFront ? 1:0, s.validLeft ? 1:0, 
    s.validRight ? 1:0, s.validBack ? 1:0);
}

void setup(){
    Serial.begin(115200);
    delay(200);
    pinMode(2, OUTPUT);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
    tofFront.begin();

    pinMode(usFront.trigPin, OUTPUT);
    pinMode(usFront.echoPin, INPUT);
    pinMode(usLeft.trigPin, OUTPUT);
    pinMode(usLeft.echoPin, INPUT);
    pinMode(usRight.trigPin, OUTPUT);
    pinMode(usRight.echoPin, INPUT);
    pinMode(usBack.trigPin, OUTPUT);
    pinMode(usBack.echoPin, INPUT);

    leftMotor.begin();
    rightMotor.begin();
    leftEnc.begin();
    rightEnc.begin();

    stopRobot();
    last_cmd_ms = millis();
}

void loop(){
    // Read serial lines
    while(Serial.available()){
        char c = Serial.read();
        if(c == '\n'){
            line.trim();
            if(line.length() > 0){
                handleLine(line);
            }
        } else {
            line += c;
        }
    }
    static uint32_t last_ctrl = 0;
    uint32_t now = millis();
    if(now - last_ctrl >= 1000 / CTRL_HZ){
        last_ctrl = now;
    }
    if(now - last_cmd_ms > CMD_TIMEOUT_MS){
        stpRobot();
        return;
    }

    float vL = v_cmd + (w_cmd * WHEELBASE_M * 0.5f);
    float vR = v_cmd - (w_cmd * WHEELBASE_M * 0.5f);

    int targetL = wheelSpeedToDuty(vL);
    int targetR = wheelSpeedToDuty(vR);

    dutyL = slewTo(dutyL, targetL);
    dutyR = slewTo(dutyR, targetR);

    applyMotor(leftMotor, vL, dutyL);
    applyMotor(rightMotor, vR, dutyR);

    //Odom update
    odom.update();

    //Odom publish
    static uint32_t last_odom_pub = 0;
    const uint32_t ODOM_HZ = 20;
    uint32_t now_ms = millis();

    if(now_ms - last_odom_pub >= 1000 / ODOM_HZ){
        last_odom_pub = now_ms;
        float x_m  = odom.getX() / 100.0f;
        float y_m = odom.getY() / 100.0f;
        float th = odom.getTheta();
        float v_m = odom.getSpeedCms() / 100.0f;
        float w_r = odom.getOmegaRads();
        Serial.printf("ODOM x %.4f y %.4f th %.4f v %.4f w %.4f\n", x_m, y_m, th, v_m, w_r);    
    }
    //ToF publish
    if(now_ms - last_tof_pub >= 1000 / TOF_HZ){
        last_tof_pub = now_ms;

        uint16_t frontFrame[64] = {0};
        bool frontOK = readFrame(tofFront, frontFrame);
        TofSummary rawSummary = summarizeFrame(frontFrame, frontOK);
        TofSummary filteredSummary = filterSummary(rawSummary, frontFilter);
        publishTofSummary(filteredSummary);
    }

    //Ultrasonic publish
    if(now_ms - last_ultra_pub >= 1000 / ULTRA_HZ){
        last_ultra_pub = now_ms;
        UltrasonicSummary rawUltra = readUltrasonics();
        UltrasonicSummary filteredUltra = filterUltrasonics(rawUltra, ultraFilter);
        publishUltrasonicSummary(filteredUltra);
    }
}
