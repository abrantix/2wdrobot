#ifndef __MOTOR_H
#define __MOTOR_H

#pragma once

#include <Arduino.h>
#include "Pinout.h"
#include <queue>

#define PWM_Resolution  8
#define PWM_Frequency   20000

#define PWM_L_Ch        2
#define PWM_R_Ch        3

#define SPEED_MIN         130
#define SPEED_DEFAULT     200
#define SPEED_MAX         240

// Time (ms) for a full 0 -> 255 PWM sweep
#define MOTOR_SLEW_FULL_SWEEP_MS 200

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
    uint8_t currentSpeed;     // actually applied PWM value
    uint8_t targetSpeed;      // requested PWM value to ramp toward
    unsigned long lastSlewUpdate; // last millis() timestamp for slew calculations
    bool requestedStop;           // indicates we should enter Stop state when speed reaches 0
    void SetupPwm(uint8_t pin);
    bool pwmConfigured;           // true once ledcSetup executed for this motor's channel

public:
    MotorState state;
    uint8_t pinA;
    uint8_t pinB;
    uint8_t pwmChannel;

    Motor();
    ~Motor();
    void SetSpeed(uint8_t speed);     // request new target speed (will slew)
    void UpdatePwm(bool MotorStateChanged);
    bool Process();                   // advance slew rate; return true if PWM changed
    void ForceStop();                 // immediately stop (no slew) used for emergency Stop
    void SmoothStop();                // request target speed 0 and stop when reached
    bool IsIdle() const { return state == MotorState_Stop && currentSpeed == 0 && targetSpeed == 0; }
    uint8_t GetTargetSpeed() const { return targetSpeed; }
    void InitPwmChannel();            // ensure channel configured once before writes
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

    // Pending direction change handling
    bool pendingDirChange = false;
    MotorState pendingLeftState = MotorState_Stop;
    MotorState pendingRightState = MotorState_Stop;
    uint8_t pendingSpeed = 0;              // target speed after direction change
    uint8_t pendingLeftSpeed = 0;          // stored desired left speed for after reversal
    uint8_t pendingRightSpeed = 0;         // stored desired right speed for after reversal

    void UpdatePwm(bool MotorStateChanged);
    void ExecuteMovement(const Movement& move);

public:
    MotorDriver();
    ~MotorDriver();
    void Init();
    void Process();
    void Stop();               // smooth stop
    void EmergencyStop();      // immediate stop bypassing slew
    void SetSpeed(uint8_t speed);
    void Forward();
    void Backward();
    void Left();
    void Right();
    void RotateClockwise();
    void RotateCounterClockwise();
    void RequestDirection(MotorState left, MotorState right, uint8_t speed); // smooth zero-cross direction change
    void EnqueueMovement(MovementType type, uint8_t speed, unsigned long duration);  // Queue a movement with a duration
    bool IsMoving();
    void ExecuteQueue();
    void JoyStickControl(float x, float y);
};

extern MotorDriver MotorDriverInstance;

#endif
