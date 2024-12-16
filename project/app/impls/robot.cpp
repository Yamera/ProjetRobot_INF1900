#include "../defs/robot.h"

#include "debug/debug_uart.h"

void Robot::configureInternalButton(
    TriggerCondition triggerCondition, 
    ButtonEventCallback callback
) {
    ButtonIntern::getInstance().configureButton(triggerCondition, callback);
}

void Robot::callInternalButtonEvent() {
    ButtonIntern::getInstance().callEventFunction();
}

void Robot::turnLedRed() {
    Led::getInstance().turnRed();
}

void Robot::turnLedGreen() {
    Led::getInstance().turnGreen();
}

void Robot::turnLedAmber() {
    Led::getInstance().turnAmber();
}

void Robot::turnLedOff() {
    Led::getInstance().turnOff();
}

void Robot::controlLeftMotorSpeed(uint8_t speed) {
    MotorControl::getInstance().controlLeftMotor(speed);
}

void Robot::controlRightMotorSpeed(uint8_t speed) {
    MotorControl::getInstance().controlRightMotor(speed);
}

void Robot::rotateLeft(uint8_t speed) {
    MotorControl::getInstance().rotateLeft(speed);
}

void Robot::rotateRight(uint8_t speed) {
    MotorControl::getInstance().rotateRight(speed);
}

void Robot::moveForward(uint8_t speed) {
    MotorControl::getInstance().moveForward(speed);
}

void Robot::driveBackward(uint8_t speed) {
    MotorControl::getInstance().driveBackward(speed);
}

void Robot::haltMotor() {
    MotorControl::getInstance().halt();
}

void Robot::playNote(uint16_t frequency) {
    SoundControl::getInstance().playNote(frequency);
}

void Robot::turnOffSound() {
    SoundControl::getInstance().turnOffSound();
}

void Robot::transmitData(const char* data) {
    Uart::getInstance().transmit(data);
}

void Robot::transmitData(const char* data, uint8_t length) {
    Uart::getInstance().transmit(data, length);
}

uint8_t Robot::receiveData() {
    return Uart::getInstance().receive();
}

bool Robot::isSensorActive(SensorType sensorType){
    return LineSensor::getInstance().isSensorActive(sensorType);
}

bool Robot::isLessThanActiveSensors(uint8_t number) {
    return LineSensor::getInstance().isLessThanActiveSensors(number);
}

bool Robot::allSensorsActive(){
    return LineSensor::getInstance().allSensorsActive();
}

bool Robot::noSensorsActive(){
    return LineSensor::getInstance().noSensorsActive();
}

uint8_t Robot::readMemory(uint16_t address, uint8_t* data) {
    return Memory::getInstance().read(address, data); 
}

uint8_t Robot::readMemory(uint16_t address, uint8_t* data, uint8_t length) {
    return Memory::getInstance().read(address, data, length);  
}

void Robot::writeMemory(uint16_t address, uint8_t data) {
    Memory::getInstance().write(address, data); 
}

void Robot::writeMemory(uint16_t address, uint8_t* data, uint8_t length) {
    Memory::getInstance().write(address, data, length); 
}