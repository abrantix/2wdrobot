#include "ObstacleAvoidance.hpp"
#include "MotorDriver.hpp"
#include "Ultrasonic.h"
#include "Trace.h"

ObstacleAvoidance ObstacleAvoidanceInstance;

ObstacleAvoidance::ObstacleAvoidance()
    : nextCheckTime(0)
{
}

ObstacleAvoidance::~ObstacleAvoidance()
{
}

void ObstacleAvoidance::Init()
{
}

bool ObstacleAvoidance::DetectObstacle()
{
    // Fetch the distance from the ultrasonic sensor
    float distance = UltrasonicInstance.GetMeasurement();
    TRACEPRINTF("Dst:%f cm\n", roundf(distance));
    return (distance <= OBSTACLE_THRESHOLD_CM);
}

void ObstacleAvoidance::Process()
{
    if(enabled)
    {
        unsigned long currentTime = millis();

        // Only check at defined intervals
        if (currentTime - nextCheckTime > 0)
        {
            // Detect an obstacle
            if (DetectObstacle())
            {
                nextCheckTime = currentTime + 100;
                TRACELN("Obstacle detected!");
                if(!MotorDriverInstance.IsMoving())
                {
                    TRACELN("ObstacleAvoidance: Enqueue rotation movement");
                    
                    if(UltrasonicInstance.GetMeasurement() < OBSTACLE_THRESHOLD_TOO_CLOSE_CM)
                    {
                        //Too close -> Enqueue reverse movement
                        MotorDriverInstance.EnqueueMovement(MovementType_Backward, SPEED_DEFAULT, MOVE_BACKWARD_DURATION_MS);     
                    }

                    //MotorDriverInstance.EnqueueMovement(MovementType_Stop, SPEED_DEFAULT, 0); // Stop the robot
                    MotorDriverInstance.EnqueueMovement(MovementType_RotateClockwise, SPEED_DEFAULT, ROTATE_DURATION_MS); 
                }
            }
            else
            {
                // Move forward if no obstacle is detected
                nextCheckTime = currentTime + 500;
                if(!MotorDriverInstance.IsMoving())
                {
                    TRACELN("ObstacleAvoidance: Enqueue forward movement");
                    MotorDriverInstance.EnqueueMovement(MovementType_Forward, SPEED_DEFAULT, MOVE_FORWARD_DURATION_MS);
                }
            }
        }
    }
}
