#ifndef __MOTOR_H
#define __MOTOR_H

#pragma once

#include <Arduino.h>
#include "Pinout.h"
#include <queue>

#define PWM_Resolution  8
#define PWM_Frequency   5000

#define PWM_L_Ch        2
#define PWM_R_Ch        3

#define SPEED_MIN         130
#define SPEED_DEFAULT     200
#define SPEED_MAX         210

enum MotorState
{
    MotorState_Stop,
    MotorState_Forward,
    MotorState_Backward,
};

enum MovementType
{
    MovementType_Forward,
    MovementType_Backward,
    MovementType_Left,
    MovementType_Right,
    MovementType_RotateClockwise,
    MovementType_RotateCounterClockwise,
    MovementType_Stop
};

struct Movement
{
    MovementType type;
    uint8_t speed;
    unsigned long duration;  // in milliseconds
};

class Motor
{ 
private:
    uint8_t speed;
    void SetupPwm(uint8_t pin);

public:
    MotorState state;
    uint8_t pinA;
    uint8_t pinB;
    uint8_t pwmChannel;

    Motor();
    ~Motor();
    void SetSpeed(uint8_t speed);
    void UpdatePwm(bool MotorStateChanged);
};

class MotorDriver
{
private:
    Motor motorR;
    Motor motorL;
    
    std::queue<Movement> movementQueue;    // Queue for movements
    Movement currentMovement;              // Store the currently executing movement
    unsigned long lastMoveTime;            // Timestamp of when the last movement started
    bool isMoving;                         // Flag indicating if a movement is in progress

    void UpdatePwm(bool MotorStateChanged);
    void ExecuteMovement(const Movement& move);

public:
    MotorDriver();
    ~MotorDriver();
    void Init();
    void Process();
    void Stop();
    void SetSpeed(uint8_t speed);
    void Forward();
    void Backward();
    void Left();
    void Right();
    void RotateClockwise();
    void RotateCounterClockwise();
    void EnqueueMovement(MovementType type, uint8_t speed, unsigned long duration);  // Queue a movement with a duration
    bool IsMoving();
    void ExecuteQueue();
    void JoyStickControl(float x, float y);
};

extern MotorDriver MotorDriverInstance;

#endif
