#ifndef QUADRUPED_ROBOT_H
#define QUADRUPED_ROBOT_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <math.h>
#include "NeuralNetwork.h"
#include "DataCollector.h"
#include "MpuData.h"

enum Direction {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

class QuadrupedRobot {
public:
    QuadrupedRobot();
    ~QuadrupedRobot();
    
    void init();
    void update();
    void walk(int cycles, float velocity, Direction direction = FORWARD);
    void stand();
    void balance();
    void stabilize();
    void learn();
    void saveModel(const char* filename);
    void loadModel(const char* filename);
    
private:
    static const int NUM_LEGS = 4;
    static const int SERVOS_PER_LEG = 2;
    static const int TOTAL_SERVOS = NUM_LEGS * SERVOS_PER_LEG;
    
    Adafruit_PWMServoDriver pwm;
    Adafruit_MPU6050 mpu;
    NeuralNetwork* nn;
    DataCollector dataCollector;
    
    float currentLegAngles[TOTAL_SERVOS];
    float defaultStandAngles[TOTAL_SERVOS];
    
    // Servo configuration
    struct ServoConfig {
        int pin;
        int minPulse;
        int maxPulse;
        float minAngle;
        float maxAngle;
    };
    ServoConfig servoConfigs[TOTAL_SERVOS];
    
    // MPU data and processing
    MpuData mpuData;
    void updateMpuData();
    MpuData getMpuData() const { return mpuData; }
    
    // Neural network functions
    void initNeuralNetwork();
    void collectTrainingData();
    void trainModel();
    void applyLearnedBehavior();
    
    // Servo control functions
    void initServoConfigs();
    int getServoIndex(int legIndex, bool isHam) const;
    void setServoAngle(int servoIndex, float angle);
    void moveLegs(const float* targetAngles, float speed);
    
    // Gait generation
    void generateWalkingGait(float* angles, int cycle, float velocity, Direction direction);
    void calculateLegTrajectory(float* x, float* y, float phase, float velocity);
};

#endif // QUADRUPED_ROBOT_H