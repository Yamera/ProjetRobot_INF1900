#include "line_sensor.h"

LineSensor::LineSensor() {
    cli();
    configureSensorPins();
    sei();
}

bool LineSensor::sensorFarLeft(){
    return (SENSOR_PIN & (1 << SENSOR_PIN_0));
}

bool LineSensor::sensorShortLeft(){
    return (SENSOR_PIN & (1 << SENSOR_PIN_1));
}

bool LineSensor::sensorMid(){
    return (SENSOR_PIN & (1 << SENSOR_PIN_2));
}

bool LineSensor::sensorShortRight(){
    return (SENSOR_PIN & (1 << SENSOR_PIN_3));
}

bool LineSensor::sensorFarRight(){
    return (SENSOR_PIN & (1 << SENSOR_PIN_4));
}

bool LineSensor::allSensorsActive(){
    return sensorFarLeft() && sensorShortLeft() && 
          sensorMid() && sensorShortRight() && sensorFarRight();
}

bool LineSensor::noSensorsActive() {
    return !sensorFarLeft() && !sensorShortLeft() && 
          !sensorMid() && !sensorShortRight() && !sensorFarRight();
}

bool LineSensor::isLessThanActiveSensors(uint8_t number) {
    uint8_t activeCount = 0;

    if (sensorFarLeft()) {
        ++activeCount;
    }
    if (sensorShortLeft()) {
        ++activeCount;
    }
    if (sensorMid()) {
        ++activeCount;
    }
    if (sensorShortRight()) {
        ++activeCount;
    }
    if (sensorFarRight()) {
        ++activeCount;
    }

    return (activeCount > number);
}

bool LineSensor::isSensorActive(SensorType sensorType) {
    switch (sensorType) {
        case SensorType::SHORT_LEFT:
            return sensorShortLeft();
        case SensorType::FAR_LEFT:
            return sensorFarLeft();
        case SensorType::MID:
            return sensorMid();
        case SensorType::SHORT_RIGHT:
            return sensorShortRight();
        case SensorType::FAR_RIGHT:
            return sensorFarRight();
        case SensorType::ALL:
            return allSensorsActive();
        case SensorType::NONE:
            return noSensorsActive();
        default:
            return false;
    }
}

void LineSensor::configureSensorPins() {
    SENSOR_REGISTER &= ~(1 << SENSOR_PIN_0);
    SENSOR_REGISTER &= ~(1 << SENSOR_PIN_1);
    SENSOR_REGISTER &= ~(1 << SENSOR_PIN_2);
    SENSOR_REGISTER &= ~(1 << SENSOR_PIN_3);
    SENSOR_REGISTER &= ~(1 << SENSOR_PIN_4);
}
