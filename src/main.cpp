#include "QuadrupedRobot.h"

QuadrupedRobot robot;
bool isLearning = true;

void setup() {
    robot.init();
    delay(1000);
}

void loop() {
    if (isLearning) {
        // Learning phase
        float velocities[] = {0.3, 0.4, 0.5};
        for (float velocity : velocities) {
            robot.walk(5, velocity, FORWARD);
            robot.learn();  // Collect data and train during walking
            delay(10);
        }
        
        // Save learned model periodically
        static unsigned long lastSave = 0;
        if (millis() - lastSave > 300000) { // Save every 5 minutes
            robot.saveModel("/model.dat");
            lastSave = millis();
        }
    } else {
        // Normal operation with learned behavior
        float velocities[] = {0.3, 0.4, 0.5};
        for (float velocity : velocities) {
            robot.walk(5, velocity, FORWARD);
            delay(10);
        }
    }
    
    robot.balance();
    robot.stabilize();
}