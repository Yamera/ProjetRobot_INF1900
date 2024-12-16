#include "infrared_sensor.h"

#include "led.h"
InfraredSensor::InfraredSensor() {
    cli();
    configureSensorPin();
    sei();
}

bool InfraredSensor::isPoleDetectedFromIntersection() {
    uint8_t medianeDistance = calculateDistinctDistance();
    if (PoleDetectionIntersectionConfig::POLE_DETECTION.contains(medianeDistance)) { 
        return true; 
    }  
    return false;
}

bool InfraredSensor::determineProximityState() {
    uint8_t distinctDistance = calculateDistinctDistance();

    if (ProximityStateConfig::PROXIMITY_INTERVAL_REFERENCE.contains(distinctDistance)) {
        return true;
    }

    return false;
}

void InfraredSensor::configureSensorPin() {
    INFRARED_REGISTER &= ~(1 << INFRARED_PIN);
}

uint8_t InfraredSensor::readRawSensorValue() {
    return (AnalogConverter::getInstance().read(INFRARED_PIN)) >> 2;
}

uint8_t InfraredSensor::calculateMedianDistance() {
    constexpr uint8_t numValues = 100;
    uint8_t values[numValues];

    for (uint8_t i = 0; i < numValues; i++) {
        values[i] = readRawSensorValue();
    }

    auto partition = [](uint8_t* arr, uint8_t low, uint8_t high) -> uint8_t {
        uint8_t pivot = arr[high];
        uint8_t i = low;
        for (uint8_t j = low; j < high; j++) {
            if (arr[j] < pivot) {
                uint8_t temp = arr[i];
                arr[i] = arr[j];
                arr[j] = temp;
                i++;
            }
        }
        uint8_t temp = arr[i];
        arr[i] = arr[high];
        arr[high] = temp;
        return i;
    };

    auto quickSelect = [&](uint8_t* arr, uint8_t low, uint8_t high, uint8_t k) -> uint8_t {
        while (low <= high) {
            uint8_t pivotIndex = partition(arr, low, high);
            if (pivotIndex == k) {
                return arr[pivotIndex];
            } else if (pivotIndex < k) {
                low = pivotIndex + 1;
            } else {
                high = pivotIndex - 1;
            }
        }
        return 0;
    };

    if (numValues % 2 == 1) {
        return quickSelect(values, 0, numValues - 1, numValues / 2);
    } else {
        uint8_t mid1 = quickSelect(values, 0, numValues - 1, numValues / 2 - 1);
        uint8_t mid2 = quickSelect(values, 0, numValues - 1, numValues / 2);
        return (mid1 + mid2) / 2;
    }
}


uint8_t InfraredSensor::calculateDistinctDistance() {
    constexpr uint8_t numValues = 100;
    uint8_t values[numValues];

    for (uint8_t i = 0; i < numValues; i++) {
        values[i] = readRawSensorValue();
    }

    uint16_t sum = 0;
    for (uint8_t i = 0; i < numValues; i++) {
        sum += values[i];
    }
    uint8_t mean = sum / numValues;

    uint8_t mostDistinctValue = values[0];
    uint8_t maxDeviation = 0;

    for (uint8_t i = 0; i < numValues; i++) {
        uint8_t deviation = (values[i] > mean) ? (values[i] - mean) : (mean - values[i]);
        if (deviation > maxDeviation) {
            maxDeviation = deviation;
            mostDistinctValue = values[i];
        }
    }

    return mostDistinctValue;
}

