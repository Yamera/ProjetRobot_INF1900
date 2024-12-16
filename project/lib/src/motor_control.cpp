#include "motor_control.h"

MotorControl::MotorControl() {
    cli();
    enable(); 
    sei();
}

MotorControl::~MotorControl() {
    cli();
    disable();  
    sei();
}

void MotorControl::rotateLeft(uint8_t speed) {
    setLeftMotorDirection(Direction::BACKWARD);
    setRightMotorDirection(Direction::FORWARD);
    controlLeftMotor(speed);
    controlRightMotor(speed);
}

void MotorControl::rotateRight(uint8_t speed) {
    setLeftMotorDirection(Direction::FORWARD);
    setRightMotorDirection(Direction::BACKWARD);
    controlLeftMotor(speed);
    controlRightMotor(speed);
}

void MotorControl::moveForward(uint8_t speed) {
    setLeftMotorDirection(Direction::FORWARD);
    setRightMotorDirection(Direction::FORWARD);
    controlLeftMotor(speed);
    controlRightMotor(speed);  
}

void MotorControl::driveBackward(uint8_t speed) {
    setLeftMotorDirection(Direction::BACKWARD);
    setRightMotorDirection(Direction::BACKWARD);
    controlLeftMotor(speed);
    controlRightMotor(speed);
}

void MotorControl::halt() {
    controlLeftMotor(0);
    controlRightMotor(0);
}

void MotorControl::controlRightMotor(uint8_t speed) {
    waveTimer_.setPWMCyclesA(speed);
}

void MotorControl::controlLeftMotor(uint8_t speed) {
    waveTimer_.setPWMCyclesB(speed);
}

void MotorControl::setLeftMotorDirection(Direction direction) {
    setMotorDirection(direction, MOTOR_PIN_LEFT);
}

void MotorControl::setRightMotorDirection(Direction direction) {
    setMotorDirection(direction, MOTOR_PIN_RIGHT);
}

void MotorControl::setMotorDirection(Direction direction,
     uint8_t motorPin) {
    if (direction == Direction::BACKWARD) {
        MOTOR_PORT |= (1 << motorPin); 
    } else {
        MOTOR_PORT &= ~(1 << motorPin);
    }
}

void MotorControl::initializeDirectionPins(){
    MOTOR_REGISTER  |= (1 << MOTOR_PIN_LEFT) 
                          | (1 << MOTOR_PIN_RIGHT);
}

void MotorControl::resetDirectionPins() {
    MOTOR_REGISTER &= ~((1 << MOTOR_PIN_LEFT) 
                            | (1 << MOTOR_PIN_RIGHT));
}

void MotorControl::enable(){
    initializeDirectionPins(); 
    waveTimer_.enableWaveOutput();   
}

void MotorControl::disable(){
    waveTimer_.disableWaveOutput();
    resetDirectionPins();
}

