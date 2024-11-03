#include "QuadrupedRobot.h"

// ... [previous code remains unchanged until calculateLegTrajectory]

void QuadrupedRobot::calculateLegTrajectory(float* x, float* z, float phase, float velocity, float turnRadius = 0.0f) {
    const float stepHeight = 30.0f * velocity;  // Z axis (up/down)
    const float stepLength = 50.0f * velocity;  // X axis (forward/backward)
    
    if (phase < 0.5f) {
        float swingPhase = phase * 2.0f;
        *x = stepLength * (swingPhase - 0.5f);  // X axis movement
        
        // Add turning offset based on turn radius (Y axis rotation)
        if (turnRadius != 0.0f) {
            *x += turnRadius * sin(swingPhase * M_PI);
        }
        
        *z = stepHeight * sin(swingPhase * M_PI);  // Z axis movement (leg lifting)
    } else {
        float stancePhase = (phase - 0.5f) * 2.0f;
        *x = stepLength * (0.5f - stancePhase);
        
        if (turnRadius != 0.0f) {
            *x += turnRadius * sin((stancePhase + 0.5f) * M_PI);
        }
        
        *z = 0;  // Ground contact phase
    }
}

void QuadrupedRobot::generateWalkingGait(float* angles, int cycle, float velocity, Direction direction) {
    const float phaseOffset[4] = {0.0f, 0.5f, 0.5f, 0.0f};  // Trot gait
    float currentPhase = (cycle % 100) / 100.0f;
    
    float turnRadius = 0.0f;
    if (direction == LEFT) turnRadius = -30.0f;
    else if (direction == RIGHT) turnRadius = 30.0f;
    
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        float phase = fmod(currentPhase + phaseOffset[leg], 1.0f);
        float x, z;  // x: forward/backward, z: up/down
        
        float legTurnRadius = turnRadius;
        if (direction == LEFT || direction == RIGHT) {
            if (leg % 2 == 0) { // Left legs
                legTurnRadius = -turnRadius;
            }
        }
        
        calculateLegTrajectory(&x, &z, phase, velocity, legTurnRadius);
        
        float hamAngle, shankAngle;
        if (direction == FORWARD || direction == BACKWARD) {
            if (direction == BACKWARD) x = -x;
            
            const float L1 = 50.0f;  // Upper leg length
            const float L2 = 50.0f;  // Lower leg length
            float targetX = x;
            float targetZ = -100.0f + z;  // Base height + lift height
            
            // Inverse kinematics calculation
            float D = sqrt(targetX * targetX + targetZ * targetZ);
            float alpha = atan2(targetZ, targetX);  // Base angle from horizontal
            float beta = acos((L1 * L1 + D * D - L2 * L2) / (2 * L1 * D));  // Angle from base to upper leg
            float gamma = acos((L1 * L1 + L2 * L2 - D * D) / (2 * L1 * L2));  // Angle between upper and lower leg
            
            hamAngle = (alpha + beta) * 180.0f / M_PI;
            shankAngle = gamma * 180.0f / M_PI;
        } else {
            // Turning motion
            float turnAngleOffset = 0.0f;
            if (direction == LEFT || direction == RIGHT) {
                turnAngleOffset = (leg % 2 == 0) ? -x : x;
                if (direction == LEFT) turnAngleOffset = -turnAngleOffset;
            }
            
            hamAngle = 90.0f + turnAngleOffset;
            shankAngle = 90.0f + (turnAngleOffset * 0.5f);
        }
        
        angles[leg * 2] = hamAngle;
        angles[leg * 2 + 1] = shankAngle;
    }
}

void QuadrupedRobot::stabilize() {
    updateMpuData();
    
    float pitchError = mpuData.Sta_Pitch;  // Rotation around Y axis
    float rollError = mpuData.Sta_Roll;    // Rotation around X axis
    
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        float x = 0;           // Forward/backward position
        float z = -100.0f;     // Base height (negative because Z points up)
        
        // Adjust leg heights based on pitch and roll
        if (leg < 2) z += pitchError * 2;  // Front legs
        else z -= pitchError * 2;          // Back legs
        
        if (leg % 2) z += rollError * 2;   // Right legs
        else z -= rollError * 2;           // Left legs
        
        // Inverse kinematics calculation
        const float L1 = 50.0f;
        const float L2 = 50.0f;
        float D = sqrt(x * x + z * z);
        float alpha = atan2(z, x);
        float beta = acos((L1 * L1 + D * D - L2 * L2) / (2 * L1 * D));
        float gamma = acos((L1 * L1 + L2 * L2 - D * D) / (2 * L1 * L2));
        
        float hamAngle = (alpha + beta) * 180.0f / M_PI;
        float shankAngle = gamma * 180.0f / M_PI;
        
        setServoAngle(leg * 2, hamAngle);
        setServoAngle(leg * 2 + 1, shankAngle);
    }
}

// ... [rest of the file remains unchanged]