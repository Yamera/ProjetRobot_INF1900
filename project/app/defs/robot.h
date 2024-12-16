#pragma once

#include "button/button_intern.h"
#include "sensor/line_sensor.h"
#include "led.h"
#include "motor_control.h"
#include "sound_control.h"
#include "uart.h"
#include "button/button_intern.h"
#include "memory.h"


namespace navigation {
    const uint8_t forwardSpeed = 87;
    const uint8_t rightMotorSpeed = 87;
    const uint8_t leftMotorSpeed = 87;
    const uint8_t speed = 87;
}

class Robot {
public:
    static Robot& getInstance() {
        static Robot instance;
        return instance;
    }

    void turnLedRed();
    void turnLedGreen();
    void turnLedAmber();
    void turnLedOff();

    void controlLeftMotorSpeed(uint8_t speed);
    void controlRightMotorSpeed(uint8_t speed);
    void rotateLeft(uint8_t speed);
    void rotateRight(uint8_t speed);
    void moveForward(uint8_t speed);
    void driveBackward(uint8_t speed);
    void haltMotor();

    void playNote(uint16_t frequency);
    void turnOffSound();

    void transmitData(const char* data);
    void transmitData(const char* data, uint8_t length);
    uint8_t receiveData();

    bool isSensorActive(SensorType sensorType);
    bool allSensorsActive();
    bool noSensorsActive();

    void configureInternalButton(
        TriggerCondition triggerCondition, 
        ButtonEventCallback callback
    );
    void callInternalButtonEvent();

    uint8_t readMemory(uint16_t address, uint8_t* data);
    uint8_t readMemory(uint16_t address, uint8_t* data, uint8_t length);
    void writeMemory(uint16_t address, uint8_t data);
    void writeMemory(uint16_t address, uint8_t* data, uint8_t length);

    bool isLessThanActiveSensors(uint8_t number);

private:
    Robot() = default;
    ~Robot() = default;

    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;
};