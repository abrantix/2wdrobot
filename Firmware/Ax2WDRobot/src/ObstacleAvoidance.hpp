#ifndef __OBSTACLE_AVOIDANCE_H
#define __OBSTACLE_AVOIDANCE_H

#include "MotorDriver.hpp"
#include "Ultrasonic.h" // Assuming you have an Ultrasonic class for sensor handling
#include <Arduino.h>

#define OBSTACLE_THRESHOLD_CM 35            // Distance threshold for obstacle detection
#define OBSTACLE_THRESHOLD_TOO_CLOSE_CM 10  // Distance threshold for obstacle detection which triggers a reverse action
#define ROTATE_DURATION_MS 500              // How long to rotate in case of an obstacle
#define MOVE_FORWARD_DURATION_MS 450        // Time to move forward in milliseconds
#define MOVE_BACKWARD_DURATION_MS 400        // Time to move backwards in milliseconds

class ObstacleAvoidance
{
private:
    unsigned long nextCheckTime;

public:
    ObstacleAvoidance();
    ~ObstacleAvoidance();

    void Init();
    void Process();
    bool DetectObstacle();  // Returns true if an obstacle is detected
    bool enabled;
};

extern ObstacleAvoidance ObstacleAvoidanceInstance;

#endif
