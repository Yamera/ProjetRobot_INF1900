#pragma once

/**
 * @class LineSensor
 * @brief Singleton class that manages the configuration and reading of multiple line sensors.
 *
 * The `LineSensor` class provides a centralized interface to interact with different line sensors
 * on a robot or other hardware setup. This class follows the singleton pattern to ensure only one
 * instance manages the sensor configuration and reads. Each sensor can be individually checked for
 * activity, or all sensors can be checked simultaneously.
 *
 * @enum SensorType
 * Defines the different line sensors managed by the `LineSensor` class:
 * - `SHORT_LEFT`: Short-range sensor on the left side.
 * - `FAR_LEFT`: Long-range sensor on the left side.
 * - `MID`: Center sensor.
 * - `SHORT_RIGHT`: Short-range sensor on the right side.
 * - `FAR_RIGHT`: Long-range sensor on the right side.
 * - `ALL`: Represents all sensors collectively.
 *
 * @authors
 * - Ahmed Sami Benabbou
 * - Maroua Lassakeur
 * - Mohamed-Borheneddine Mokaddem
 * - Yasmine Meraoubi
 */

class LineSensor {
public:
    enum SensorType {
        SHORT_LEFT,
        FAR_LEFT,
        MID,
        SHORT_RIGHT,
        FAR_RIGHT,
        ALL,
        NONE,
        INVALID,
    };

    static LineSensor& getInstance() {
        static LineSensor instance;
        return instance;
    }

    bool isSensorActive(SensorType sensorType);
    bool allSensorsActive();
    bool noSensorsActive();
    bool isLessThanActiveSensors(uint8_t number);

private:
    LineSensor();
    ~LineSensor() = default;
    LineSensor(const LineSensor&) = delete;
    LineSensor& operator=(const LineSensor&) = delete;

    bool sensorShortLeft();
    bool sensorFarLeft();
    bool sensorMid();
    bool sensorShortRight();
    bool sensorFarRight();
    
    
    void configureSensorPins();
};

using SensorType = LineSensor::SensorType;
