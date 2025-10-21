#include "MotorDriver.hpp"
#include "Trace.h"
#include "ObstacleAvoidance.hpp"

MotorDriver MotorDriverInstance;

Motor::Motor() : currentSpeed(0), targetSpeed(0), lastSlewUpdate(0), requestedStop(false), pwmConfigured(false) {
    state = MotorState_Stop; // explicit init
}
Motor::~Motor() {}

void Motor::InitPwmChannel() {
    if (!pwmConfigured) {
        ledcSetup(pwmChannel, PWM_Frequency, PWM_Resolution); // configure frequency & resolution once
        pwmConfigured = true;
    }
}

void Motor::SetupPwm(uint8_t pin) {
    InitPwmChannel();
    ledcAttachPin(pin, pwmChannel); // (re)attach the active direction pin
}

void Motor::SetSpeed(uint8_t speed) {
    if (speed > 255) speed = 255; // guard
    targetSpeed = speed;
}

void Motor::ForceStop() {
    targetSpeed = 0;
    currentSpeed = 0;
    if (pwmConfigured) {
        ledcWrite(pwmChannel, 0);
    }
    requestedStop = false;
}

void Motor::SmoothStop() {
    requestedStop = true;
    targetSpeed = 0;
}

void Motor::UpdatePwm(bool MotorStateChanged) {
    if (MotorStateChanged) {
        switch (state) {
            case MotorState_Forward:
                SetupPwm(pinA);
                ledcWrite(pwmChannel, currentSpeed);
                pinMode(pinB, OUTPUT);
                digitalWrite(pinB, 0);
                break;
            case MotorState_Backward:
                SetupPwm(pinB);
                ledcWrite(pwmChannel, currentSpeed);
                pinMode(pinA, OUTPUT);
                digitalWrite(pinA, 0);
                break;
            case MotorState_Stop:
                pinMode(pinA, OUTPUT);
                digitalWrite(pinA, 0);
                pinMode(pinB, OUTPUT);
                digitalWrite(pinB, 0);
                break;
            default:
                break;
        }
    } else {
        if (state != MotorState_Stop) {
            ledcWrite(pwmChannel, currentSpeed);
        }
    }
}

bool Motor::Process() {
    // If already in Stop state just ensure PWM zero then exit
    if (state == MotorState_Stop) {
        if (currentSpeed != 0) {
            currentSpeed = 0;
            if (pwmConfigured) ledcWrite(pwmChannel, 0);
            return true;
        }
        return false;
    }

    if (currentSpeed == targetSpeed) {
        // If we were requesting a stop and have reached zero, now set the state to Stop and deassert pins
        if (requestedStop && targetSpeed == 0 && currentSpeed == 0) {
            state = MotorState_Stop;
            // ensure both pins low
            pinMode(pinA, OUTPUT); digitalWrite(pinA, 0);
            pinMode(pinB, OUTPUT); digitalWrite(pinB, 0);
            requestedStop = false;
            return true;
        }
        return false; // nothing else changed
    }

    unsigned long now = millis();
    if (lastSlewUpdate == 0) lastSlewUpdate = now;
    unsigned long dt = now - lastSlewUpdate;
    if (dt == 0) return false;

    // compute max delta per ms so that 0->255 takes MOTOR_SLEW_FULL_SWEEP_MS
    // delta per ms = 255 / MOTOR_SLEW_FULL_SWEEP_MS
    const float deltaPerMs = 255.0f / (float)MOTOR_SLEW_FULL_SWEEP_MS;
    int maxStep = (int)roundf(deltaPerMs * dt);
    if (maxStep < 1) maxStep = 1; // ensure progress

    if (currentSpeed < targetSpeed) {
        int newSpeed = currentSpeed + maxStep;
        if (newSpeed > targetSpeed) newSpeed = targetSpeed;
        currentSpeed = (uint8_t)newSpeed;
    } else {
        int newSpeed = currentSpeed - maxStep;
        if (newSpeed < targetSpeed) newSpeed = targetSpeed;
        currentSpeed = (uint8_t)newSpeed;
    }
    lastSlewUpdate = now;

    // apply updated PWM value if motor is active
    if (state != MotorState_Stop && pwmConfigured) {
        ledcWrite(pwmChannel, currentSpeed);
    }

    // If we have reached target and requestedStop was set, convert to Stop (handled next loop or now if reached zero)
    if (requestedStop && currentSpeed == 0) {
        state = MotorState_Stop;
        pinMode(pinA, OUTPUT); digitalWrite(pinA, 0);
        pinMode(pinB, OUTPUT); digitalWrite(pinB, 0);
        requestedStop = false;
    }
    return true;
}

MotorDriver::MotorDriver() : isMoving(false), lastMoveTime(0) {
    motorL.pwmChannel = PWM_L_Ch;
    motorR.pwmChannel = PWM_R_Ch;
    motorL.pinA = MOTOR_L1_PIN;
    motorL.pinB = MOTOR_L2_PIN;
    motorR.pinA = MOTOR_R1_PIN;
    motorR.pinB = MOTOR_R2_PIN;
}

MotorDriver::~MotorDriver() {}

void MotorDriver::Init() {
    // Pre-configure LEDC channels before any writes to avoid ledc_get_duty() errors
    motorL.InitPwmChannel();
    motorR.InitPwmChannel();
    Stop();
    SetSpeed(SPEED_DEFAULT);
}

void MotorDriver::Stop() {
    // Request a smooth ramp down; keep current state (direction) until motors reach 0 then they change to Stop internally
    motorL.SmoothStop();
    motorR.SmoothStop();
}

void MotorDriver::EmergencyStop() {
    motorR.state = MotorState_Stop;
    motorL.state = MotorState_Stop;
    motorL.ForceStop();
    motorR.ForceStop();
    motorR.UpdatePwm(true); // ensures direction pins are low
    motorL.UpdatePwm(true);
}

void MotorDriver::SetSpeed(uint8_t speed) {
    motorR.SetSpeed(speed);
    motorL.SetSpeed(speed);
}

void MotorDriver::Forward() {
    RequestDirection(MotorState_Forward, MotorState_Forward, 0);
}

void MotorDriver::Backward() {
    RequestDirection(MotorState_Backward, MotorState_Backward, 0);
}

void MotorDriver::Left() {
    RequestDirection(MotorState_Forward, MotorState_Stop, 0);
}

void MotorDriver::Right() {
    RequestDirection(MotorState_Stop, MotorState_Forward, 0);
}

void MotorDriver::RotateClockwise() {
    RequestDirection(MotorState_Backward, MotorState_Forward, 0);
}

void MotorDriver::RotateCounterClockwise() {
    RequestDirection(MotorState_Forward, MotorState_Backward, 0);
}

void MotorDriver::RequestDirection(MotorState left, MotorState right, uint8_t speed) {
    // speed parameter currently unused; using existing motor target speeds
    (void)speed;

    // If no change needed just ensure PWM update
    if (motorL.state == left && motorR.state == right) {
        return; // ongoing slew covers speed adjustments
    }

    // If either motor is moving (not idle) we request a smooth stop first
    bool reversalNeeded = (motorL.state != left) || (motorR.state != right);
    if (reversalNeeded) {
        if (!pendingDirChange) {
            pendingDirChange = true;
            pendingLeftState = left;
            pendingRightState = right;
            // store the speeds we want after transition (current targets at request time)
            pendingLeftSpeed = motorL.GetTargetSpeed();
            pendingRightSpeed = motorR.GetTargetSpeed();
        } else {
            // overwrite with the latest requested direction; last command wins
            pendingLeftState = left;
            pendingRightState = right;
            pendingLeftSpeed = motorL.GetTargetSpeed();
            pendingRightSpeed = motorR.GetTargetSpeed();
        }
        // Initiate smooth stop (it will ramp to zero)
        motorL.SmoothStop();
        motorR.SmoothStop();
    }
}

void MotorDriver::EnqueueMovement(MovementType type, uint8_t speed, unsigned long duration) {
    Movement move = { type, speed, duration };
    movementQueue.push(move);
}


bool MotorDriver::IsMoving() {
    return !movementQueue.empty() || isMoving;
}

void MotorDriver::ExecuteMovement(const Movement& move) {
    motorL.SetSpeed(move.speed);
    motorR.SetSpeed(move.speed);
    switch (move.type) {
        case MovementType_Forward:
            Forward();
            break;
        case MovementType_Backward:
            Backward();
            break;
        case MovementType_Left:
            Left();
            break;
        case MovementType_Right:
            Right();
            break;
        case MovementType_RotateClockwise:
            RotateClockwise();
            break;
        case MovementType_RotateCounterClockwise:
            RotateCounterClockwise();
            break;
        case MovementType_Stop:
            Stop();
            break;
    }
}

void MotorDriver::ExecuteQueue() {
    if (movementQueue.empty() || isMoving) {
        return;
    }

    // Start a new movement
    currentMovement = movementQueue.front();  // Store current movement
    ExecuteMovement(currentMovement);         // Execute the movement
    lastMoveTime = millis();                  // Record start time
    isMoving = true;                          // Mark as moving
}

void MotorDriver::Process() {
    // advance slew for each motor first
    bool changedL = motorL.Process();
    bool changedR = motorR.Process();
    (void)changedL; (void)changedR; // reserved for future diagnostics

    // If we have a pending direction change and both motors are idle at zero speed, apply it
    if (pendingDirChange && motorL.IsIdle() && motorR.IsIdle()) {
        motorL.state = pendingLeftState;
        motorR.state = pendingRightState;
        motorL.UpdatePwm(true);
        motorR.UpdatePwm(true);
        // restore speeds (will ramp up after zero-cross)
        motorL.SetSpeed(pendingLeftSpeed);
        motorR.SetSpeed(pendingRightSpeed);
        pendingDirChange = false;
    }

    if (isMoving) {
        // Check if the current movement's duration has passed
        if (millis() - lastMoveTime >= currentMovement.duration) {
            movementQueue.pop();              // Now pop the completed movement
            isMoving = false;                 // Reset movement state
        }
    }

    // Start the next movement if not moving
    if (!isMoving && !movementQueue.empty()) {
        ExecuteQueue();
    }

    // Yield to feed watchdog in case Process is called in a tight loop with heavy async activity
    delay(1);
}

void MotorDriver::JoyStickControl(float x, float y)
{

    float overallSpeed = sqrt(x*x + y*y);
    //80...255
    uint8_t pwmOverallSpeed = map(overallSpeed * 255, 0, 255, SPEED_MIN, SPEED_MAX);
    uint8_t pwmXSpeed = map(abs(x) * 255 * overallSpeed, 0, 255, SPEED_MIN, SPEED_MAX);
    uint8_t pwmYSpeed = map(abs(y) * 255 * overallSpeed, 0, 255, SPEED_MIN, SPEED_MAX);

    TRACEPRINTF("overall:%d x:%d y:%d\n", pwmOverallSpeed, pwmXSpeed, pwmYSpeed);

    //lookup quadrant
    if(y==0 && y==0)
    {
        Stop();
    }
    //0...90°
    else if(x<0 && y<0)
    {
        ObstacleAvoidanceInstance.enabled = false;
        motorL.SetSpeed(pwmYSpeed);
        motorR.SetSpeed(pwmOverallSpeed);
        if(motorR.state != MotorState_Forward || motorL.state != MotorState_Forward) {
            RequestDirection(MotorState_Forward, MotorState_Forward, 0);
        }
    }
    //90...180
    else if(x>0 && y<0)
    {
        ObstacleAvoidanceInstance.enabled = false;
        motorL.SetSpeed(pwmOverallSpeed);
        motorR.SetSpeed(pwmYSpeed);
        if(motorR.state != MotorState_Forward || motorL.state != MotorState_Forward) {
            RequestDirection(MotorState_Forward, MotorState_Forward, 0);
        }
    }
    //180...270
    else if(x>0 && y>0)
    {
        ObstacleAvoidanceInstance.enabled = false;
        motorL.SetSpeed(pwmOverallSpeed);
        motorR.SetSpeed(pwmYSpeed);
        if(motorR.state != MotorState_Backward || motorL.state != MotorState_Backward) {
            RequestDirection(MotorState_Backward, MotorState_Backward, 0);
        }
    }
    //270..360
    else if(x<0 && y>0)
    {
        ObstacleAvoidanceInstance.enabled = false;
        motorL.SetSpeed(pwmYSpeed);
        motorR.SetSpeed(pwmOverallSpeed);
        if(motorR.state != MotorState_Backward || motorL.state != MotorState_Backward) {
            RequestDirection(MotorState_Backward, MotorState_Backward, 0);
       }
    }
    else
    {
        //TODO transients...
    }
}
