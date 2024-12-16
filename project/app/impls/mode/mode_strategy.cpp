#include "../defs/mode/mode_strategy.h"

#include "debug/debug_uart.h"

const char* ModeStrategy::toString(Orientation orientation) {
    switch (orientation) {
        case Orientation::NORTH_EAST: return "NE";
        case Orientation::NORTH_WEST: return "NW";
        case Orientation::SOUTH_EAST: return "SE";
        case Orientation::SOUTH_WEST: return "SW";
        default: return "Unknown";
    }
}

void ModeStrategy::playConfirmationSound() {
    // Premier bip
    robot_->playNote(81);     // Fréquence de 2000 Hz
    _delay_ms(750);             // Durée de 750 ms
    robot_->turnOffSound();     // Arrêter le son
    _delay_ms(50);              // Silence de 50 ms

    // Deuxième bip
    robot_->playNote(81);     // Fréquence de 2000 Hz
    _delay_ms(750);             // Durée de 750 ms
    robot_->turnOffSound();     // Arrêter le son
}

/*
 * The followLine function enables the robot to follow a line using sensor data.
 * Depending on the sensor input, it adjusts the speed of the left and right motors:
 * - If the left sensor detects the line, the robot steers slightly right.
 * - If the right sensor detects the line, the robot steers slightly left.
 * - If the middle sensor detects the line, the robot adjusts to move straight.
 * By default, the robot moves forward at a set speed.
 */
void ModeStrategy::followLine() {
    robot_->moveForward(navigation::forwardSpeed);

    if (robot_->isSensorActive(SensorType::SHORT_LEFT)) {
        robot_->controlLeftMotorSpeed(navigation::forwardSpeed - 15);
        robot_->controlRightMotorSpeed(navigation::forwardSpeed + 15);
    }
    else if (robot_->isSensorActive(SensorType::SHORT_RIGHT)) {
        robot_->controlRightMotorSpeed(navigation::forwardSpeed - 10);
    }
    else if (robot_->isSensorActive(SensorType::MID)) {
        robot_->controlRightMotorSpeed(navigation::forwardSpeed + 10);
        robot_->controlLeftMotorSpeed(navigation::forwardSpeed);
    }
}

