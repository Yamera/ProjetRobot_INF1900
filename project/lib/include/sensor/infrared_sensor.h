#pragma once

#include "../analog_converter.h"
#include <math.h>

struct Interval {
    uint8_t min;
    uint8_t max;

    static constexpr uint8_t INF = 255;

    bool contains(uint8_t value) const {
        return (value >= min && value <= max);
    }
};

namespace PoleDetectionIntersectionConfig {
    //constexpr Interval POLE_DETECTION = {50, Interval::INF};
     constexpr Interval POLE_DETECTION = {50, Interval::INF};
}

namespace ProximityStateConfig {
    constexpr uint8_t PROXIMITY_REFERENCE = 123; // ~10cm

    // constexpr Interval PROXIMITY_INTERVAL_REFERENCE = {PROXIMITY_REFERENCE, 142};
    // constexpr Interval PROXIMITY_INTERVAL_TOO_CLOSE = {PROXIMITY_REFERENCE, 142}; // [[10, 2]] cm
    // constexpr Interval PROXIMITY_INTERVAL_TOO_FAR = {0, PROXIMITY_REFERENCE}; // [[15cm, 10]] cm
    constexpr Interval PROXIMITY_INTERVAL_REFERENCE = {95-3, 95+5}; // ~10cm
}

class InfraredSensor {
public:
    static InfraredSensor& getInstance() {
        static InfraredSensor instance;
        return instance;
    }

    enum class ProximityState {
        TOO_CLOSE,
        WITHIN_RANGE,
        TOO_FAR,
    };

    bool determineProximityState();
    bool isPoleDetectedFromIntersection();

private:
    uint8_t readRawSensorValue();
    uint8_t calculateMedianDistance();
    uint8_t calculateDistinctDistance();
    void configureSensorPin();

private:
    InfraredSensor();
    InfraredSensor(const InfraredSensor&) = delete;
    InfraredSensor& operator=(const InfraredSensor&) = delete;
};

using ProximityState = InfraredSensor::ProximityState;