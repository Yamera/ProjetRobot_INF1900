#pragma once

#include "timer/motor_timer_impl.h"

class MotorControl {
private:
    enum Direction : unsigned char {
        FORWARD,
        BACKWARD
    };
public:

    static MotorControl& getInstance() {
        static MotorControl instance;
        return instance;
    }
    
    void rotateLeft(uint8_t speed);
    void rotateRight(uint8_t speed);

    void moveForward(uint8_t speed);
    void driveBackward(uint8_t speed);

    void halt();
   
    void setLeftMotorDirection(Direction direction);
    void setRightMotorDirection(Direction direction);

    void controlLeftMotor(uint8_t speed);
    void controlRightMotor(uint8_t speed);

private:
    void enable();
    void disable();
    
    void initializeDirectionPins();
    void resetDirectionPins();

    void setMotorDirection(Direction direction, uint8_t motorPin);
    
private:
    MotorTimerImpl waveTimer_; 

    MotorControl();
    ~MotorControl();
    MotorControl(const MotorControl&) = delete;
    MotorControl& operator=(const MotorControl&) = delete;
};

