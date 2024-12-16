#include "avr/io.h"
#include <util/delay.h>
#include "avr/interrupt.h"

#include "debug/debug_uart.h"

#include "sensor/infrared_sensor.h"
#include "robot.h"

namespace navigation_control {
    static constexpr uint16_t POST_INTERSECTION_DELAY_MS = 3200;

    static constexpr uint8_t rotationSpeed = 120;
    static constexpr uint8_t quickAdvanceSpeed = 120;
    static constexpr uint8_t forwardSpeed = 120;

    static constexpr uint16_t STABILIZATION_DELAY_MS = 1000; 
}

enum class StartingPlaceFirstCourse { 
    C = 'C',    
    D = 'D',  
    UNKNOWN = 0    
}; 

enum class PoleLocationFirstCourse {
    ONE = 1,
    TWO = 2,        
    UNKNOWN = 0,         
};

enum class StartingPlaceSecondCourse { 
    ONE = 1,   
    TWO = 2,  
    UNKNOWN = 0    
}; 

enum class PoleLocationSecondCourse  {
    THREE = 3,
    FOUR = 4,        
    UNKNOWN = 0,         
};

enum class StartingPlaceThirdCourse { 
    THREE = 3,    
    FOUR = 4,  
    UNKNOWN = 0    
}; 

enum class PoleLocationThirdCourse  {
    FIVE = 5,
    SIX = 6,        
    UNKNOWN = 0,         
};

template <typename StartingPlaceEnum, typename PoleLocationEnum>
struct InferredData {
    PoleLocationEnum poleLocation;
    StartingPlaceEnum startingPosition;
};

using InferredDataFirstCourse  = 
    InferredData<StartingPlaceFirstCourse, PoleLocationFirstCourse>;

using InferredDataSecondCourse = 
    InferredData<StartingPlaceSecondCourse, PoleLocationSecondCourse>;

using InferredDataThirdCourse  = 
    InferredData<StartingPlaceThirdCourse, PoleLocationThirdCourse>;

enum class RotationState {
    NONE,  
    LEFT,   
    RIGHT   
};

enum class RotationDirection {
    LEFT,
    RIGHT
};

enum class RotationStageLeft {
    DETECTED_IN_LEFT_ADJUSTMENT,      
    DETECTED_IN_LEFT_ROTATION,        
    DETECTED_IN_LEFT_EMPTY_SPACE,     
    DETECTED_IN_MIDDLE_RETURN,       
    DETECTED_IN_RIGHT_EMPTY_SPACE,   
    DETECTED_IN_RIGHT_ROTATION,       
    NOT_DETECTED
};

enum class RotationStageRight {
    DETECTED_IN_RIGHT_ADJUSTMENT,      
    DETECTED_IN_RIGHT_ROTATION,        
    DETECTED_IN_RIGHT_EMPTY_SPACE,     
    DETECTED_IN_MIDDLE_RETURN,       
    DETECTED_IN_LEFT_EMPTY_SPACE,   
    DETECTED_IN_LEFT_ROTATION,       
    NOT_DETECTED
};

enum class RotationStageMiddle {
    DETECTED_IN_LEFT_EMPTY_SPACE,   
    DETECTED_IN_MIDDLE_SLIGHT_LEFT, 
    DETECTED_IN_RIGHT_EMPTY_SPACE,
    DETECTED_IN_MIDDLE_SLIGHT_RIGHT,         
};

struct RotationStepMiddle {
    RotationDirection direction;
    SensorType sensorType;
    RotationStageMiddle stage;
};

struct RotationStepLeft {
    RotationDirection direction;
    SensorType sensorType;
    RotationStageLeft stage;
};

struct RotationStepRight {
    RotationDirection direction;
    SensorType sensorType;
    RotationStageRight stage;
};

constexpr RotationStepMiddle stepsMid_[] = {
    {RotationDirection::LEFT, SensorType::NONE, RotationStageMiddle::DETECTED_IN_LEFT_EMPTY_SPACE},
    {RotationDirection::RIGHT, SensorType::SHORT_RIGHT, RotationStageMiddle::DETECTED_IN_MIDDLE_SLIGHT_LEFT},
    {RotationDirection::RIGHT, SensorType::NONE, RotationStageMiddle::DETECTED_IN_RIGHT_EMPTY_SPACE},
    {RotationDirection::LEFT, SensorType::SHORT_LEFT, RotationStageMiddle::DETECTED_IN_MIDDLE_SLIGHT_RIGHT},  
};

constexpr RotationStepLeft stepsLeft_[] = {
    {RotationDirection::LEFT, SensorType::NONE, RotationStageLeft::DETECTED_IN_LEFT_ADJUSTMENT},
    {RotationDirection::LEFT, SensorType::FAR_LEFT, RotationStageLeft::DETECTED_IN_LEFT_ROTATION},
    {RotationDirection::LEFT, SensorType::NONE, RotationStageLeft::DETECTED_IN_LEFT_EMPTY_SPACE},
    {RotationDirection::RIGHT, SensorType::MID, RotationStageLeft::DETECTED_IN_MIDDLE_RETURN},
    {RotationDirection::RIGHT, SensorType::NONE, RotationStageLeft::DETECTED_IN_RIGHT_EMPTY_SPACE},
    {RotationDirection::RIGHT, SensorType::FAR_RIGHT, RotationStageLeft::DETECTED_IN_RIGHT_ROTATION}
};

constexpr RotationStepRight stepsRight_[] = {
    {RotationDirection::RIGHT, SensorType::NONE, RotationStageRight::DETECTED_IN_RIGHT_ADJUSTMENT},
    {RotationDirection::RIGHT, SensorType::FAR_RIGHT, RotationStageRight::DETECTED_IN_RIGHT_ROTATION},
    {RotationDirection::RIGHT, SensorType::NONE, RotationStageRight::DETECTED_IN_RIGHT_EMPTY_SPACE},
    {RotationDirection::LEFT, SensorType::MID, RotationStageRight::DETECTED_IN_MIDDLE_RETURN},
    {RotationDirection::LEFT, SensorType::NONE, RotationStageRight::DETECTED_IN_LEFT_EMPTY_SPACE},
    {RotationDirection::LEFT, SensorType::FAR_LEFT, RotationStageRight::DETECTED_IN_LEFT_ROTATION}
};

namespace configRapport {
    constexpr uint16_t departureNodeFirstCourseAdress = 0x0004;
    constexpr uint16_t poleLocationFirstCourseAdress= 0x0005;
    constexpr uint16_t poleLocationSecondCourseAdress = 0x0006;
    constexpr uint16_t poleLocationThirdCourseAdress = 0x0007;  
}

Robot& robot_ = Robot::getInstance();
InfraredSensor& infraredSensor_ = InfraredSensor::getInstance();

static InferredDataFirstCourse locationDataFirstCourse_;
static InferredDataSecondCourse locationDataSecondCourse_;
static InferredDataThirdCourse locationDataThirdCourse_;

static SensorType triggeredSensor_;
static RotationState lastRotation_;

static bool isPoleFoundOnLeft_;
static bool isPoleFoundOnRight_;
static bool isPoleDetectedOppositeDirection_;

void initialize() {
    cli();
    locationDataFirstCourse_ = { 
        PoleLocationFirstCourse::UNKNOWN, 
        StartingPlaceFirstCourse::UNKNOWN 
    };

    locationDataSecondCourse_ = { 
        PoleLocationSecondCourse::UNKNOWN, 
        StartingPlaceSecondCourse::UNKNOWN 
    };

    locationDataThirdCourse_ = { 
        PoleLocationThirdCourse::UNKNOWN, 
        StartingPlaceThirdCourse::UNKNOWN 
    };

    triggeredSensor_ = SensorType::NONE;

    lastRotation_ = RotationState::NONE;

    isPoleFoundOnLeft_ = false;
    isPoleFoundOnRight_ = false;
    isPoleDetectedOppositeDirection_ = false;
    sei();
}

void stopAndStabilize() {
    robot_.haltMotor();
    _delay_ms(navigation_control::STABILIZATION_DELAY_MS);
}

void followLineDefault() {
    robot_.moveForward(navigation_control::forwardSpeed);

    if (robot_.isSensorActive(SensorType::SHORT_LEFT)) {
        robot_.controlLeftMotorSpeed(navigation_control::forwardSpeed - 15);
        robot_.controlRightMotorSpeed(navigation_control::forwardSpeed + 5);
    } else if (robot_.isSensorActive(SensorType::SHORT_RIGHT)) {
        robot_.controlRightMotorSpeed(navigation_control::forwardSpeed - 10);
    } else if (robot_.isSensorActive(SensorType::MID)) {
        robot_.controlRightMotorSpeed(navigation_control::forwardSpeed + 10);
        robot_.controlLeftMotorSpeed(navigation_control::forwardSpeed);
    }
}

void followLineForDuration(uint16_t durationMs) {
    constexpr uint16_t delayPerIteration = 2;
    uint16_t iterations = durationMs / delayPerIteration;

    for (uint16_t i = 0; i < iterations; i++) {
        followLineDefault();
        _delay_ms(delayPerIteration); 
    }
}

bool followLineUntil(SensorType (*shouldExit)()) {
    followLineDefault();
    triggeredSensor_ = shouldExit();
    return (triggeredSensor_ == SensorType::NONE);
}

bool followLineUntil(bool (*shouldExit)()) {
    followLineDefault();
    return !shouldExit();
}

bool detectPoleWrapper() {
    return infraredSensor_.determineProximityState();
}

bool isSensorActiveWrapper(SensorType sensorType) {
    return robot_.isSensorActive(sensorType);
}

SensorType detectIntersection() {
    if (robot_.isSensorActive(SensorType::FAR_LEFT)) {
        return SensorType::FAR_LEFT;
    } else if (robot_.isSensorActive(SensorType::FAR_RIGHT)) {
        return SensorType::FAR_RIGHT;
    }
    return SensorType::NONE;
}

bool rotateUntil( bool (*shouldExit)(SensorType), SensorType sensorType, 
                    uint8_t speed, 
                    RotationDirection direction) {   

    if (direction == RotationDirection::LEFT) {
        robot_.rotateLeft(speed);
    } 
    else {
        robot_.rotateRight(speed);
    }

    return !shouldExit(sensorType);
}

void processInferredDataFirstCourse() {
    if (isPoleDetectedOppositeDirection_) {
        if (triggeredSensor_ == SensorType::FAR_RIGHT) {
            locationDataFirstCourse_ = { 
                PoleLocationFirstCourse::ONE, 
                StartingPlaceFirstCourse::D 
            };
        } else if (triggeredSensor_ == SensorType::FAR_LEFT) {
            locationDataFirstCourse_ = { 
                PoleLocationFirstCourse::TWO, 
                StartingPlaceFirstCourse::C 
            };
        }
    } else {
        if (isPoleFoundOnRight_) {
            locationDataFirstCourse_ = { 
                PoleLocationFirstCourse::TWO, 
                StartingPlaceFirstCourse::D 
            };
        } else if (isPoleFoundOnLeft_) {
            locationDataFirstCourse_ = { 
                PoleLocationFirstCourse::ONE, 
                StartingPlaceFirstCourse::C 
            };
        }
    }
}

void processInferredDataSecondCourse() {
    InferredDataFirstCourse locationDataFirstCourseTemp = locationDataFirstCourse_;

    if (isPoleDetectedOppositeDirection_) {
        if (locationDataFirstCourseTemp.poleLocation == PoleLocationFirstCourse::ONE) {
            locationDataSecondCourse_ = { 
                PoleLocationSecondCourse::FOUR, 
                StartingPlaceSecondCourse::ONE 
            };
        } else if (locationDataFirstCourseTemp.poleLocation == PoleLocationFirstCourse::TWO) {
            locationDataSecondCourse_ = { 
                PoleLocationSecondCourse::THREE, 
                StartingPlaceSecondCourse::TWO 
            };
        }
    } else {
        if (isPoleFoundOnRight_) {
            locationDataSecondCourse_ = { 
                PoleLocationSecondCourse::FOUR, 
                StartingPlaceSecondCourse::TWO 
            };
        } else if (isPoleFoundOnLeft_) {
            locationDataSecondCourse_ = { 
                PoleLocationSecondCourse::THREE, 
                StartingPlaceSecondCourse::ONE 
            };
        }
    }
}

void processInferredDataThirdCourse() {
    InferredDataSecondCourse locationDataSecondCourseTemp = locationDataSecondCourse_;

    if (isPoleDetectedOppositeDirection_) {
        if (locationDataSecondCourseTemp.poleLocation == PoleLocationSecondCourse::THREE) {
            locationDataThirdCourse_ = { 
                PoleLocationThirdCourse::SIX, 
                StartingPlaceThirdCourse::THREE 
            };
        } else if (locationDataSecondCourseTemp.poleLocation == PoleLocationSecondCourse::FOUR) {
            locationDataThirdCourse_ = { 
                PoleLocationThirdCourse::FIVE, 
                StartingPlaceThirdCourse::FOUR 
            };
        }
    } else {
        if (isPoleFoundOnRight_) {
            locationDataThirdCourse_ = { 
                PoleLocationThirdCourse::SIX, 
                StartingPlaceThirdCourse::FOUR 
            };
        } else if (isPoleFoundOnLeft_) {
            locationDataThirdCourse_ = { 
                PoleLocationThirdCourse::FIVE, 
                StartingPlaceThirdCourse::THREE 
            };
        }
    }
}

bool isPoleDetectedBeforeIntersection() {   
    if (infraredSensor_.isPoleDetectedFromExtremity()) {
        isPoleDetectedOppositeDirection_  = true;
        return true;
    }
    return false;
}

bool isPoleDetectedFromIntersection(RotationDirection direction) {
    if (infraredSensor_.isPoleDetectedFromIntersection()) {
        if (direction == RotationDirection::LEFT) {
            isPoleFoundOnLeft_ = true;
        } else {
            isPoleFoundOnRight_ = true;
        }
        return true;
    }
    return false;
}

bool isPoleDetectedRotatingFromIntersection(RotationDirection direction,
         bool (*isSensorActive)(SensorType),
         SensorType sensorType, uint8_t speed) {

    bool isPoleDetected = false;

    while (rotateUntil(isSensorActive, sensorType, speed, direction)) {
        if(!isPoleDetected){
            isPoleDetected = isPoleDetectedFromIntersection(direction); 
        }
    }

    return isPoleDetected;
}

template<typename RotationStepType>
bool processRotationAndDetectPole(const RotationStepType steps[], size_t stepsSize, bool isMiddleStage = false) {
    static_assert(sizeof(RotationStepType) == 0, "Unsupported RotationStepType!");
    return false;  
}

template<>
bool processRotationAndDetectPole<RotationStepLeft>(const RotationStepLeft steps[], size_t stepsSize, bool isMiddleStage) {
    bool isPoleDetected = false; 
    bool isProcessComplete = false;

    for (uint8_t currentStepIndex = 0; currentStepIndex < stepsSize; ++currentStepIndex) {
        const auto& currentStep = steps[currentStepIndex];

        isPoleDetected = isPoleDetectedRotatingFromIntersection(
                currentStep.direction,
                isSensorActiveWrapper,
                currentStep.sensorType,
                navigation_control::rotationSpeed);
        stopAndStabilize();

        if (isMiddleStage) {
            continue;
        }

        if (isPoleDetected) {
            uint8_t stepIndex = currentStepIndex + 1;
            bool resetFlag = false;

            while (!resetFlag) {
                const auto& step = steps[stepIndex];

                while (rotateUntil(isSensorActiveWrapper, 
                            step.sensorType,
                            navigation_control::rotationSpeed,
                            step.direction));

                if (step.stage == RotationStageLeft::DETECTED_IN_MIDDLE_RETURN) {
                    isProcessComplete = true;
                    resetFlag = true;
                }

                stepIndex++;
                if (stepIndex >= stepsSize) {
                    stepIndex = 0;
                }
            }
        }

        if (isProcessComplete) {
            break;
        }
    }
    return isPoleDetected;
}

template<>
bool processRotationAndDetectPole<RotationStepRight>(const RotationStepRight steps[], size_t stepsSize, bool isMiddleStage) {
    bool isPoleDetected = false; 
    bool isProcessComplete = false;

    for (uint8_t currentStepIndex = 0; currentStepIndex < stepsSize; ++currentStepIndex) {
        const auto& currentStep = steps[currentStepIndex];

        isPoleDetected = isPoleDetectedRotatingFromIntersection(
                currentStep.direction,
                isSensorActiveWrapper,
                currentStep.sensorType,
                navigation_control::rotationSpeed);
        stopAndStabilize();

        if (isMiddleStage) {
            continue;
        }

        if (isPoleDetected) {
            uint8_t stepIndex = currentStepIndex + 1;
            bool resetFlag = false;

            while (!resetFlag) {
                const auto& step = steps[stepIndex];

                while (rotateUntil(isSensorActiveWrapper, 
                            step.sensorType,
                            navigation_control::rotationSpeed,
                            step.direction));

                if (step.stage == RotationStageRight::DETECTED_IN_MIDDLE_RETURN) {
                    isProcessComplete = true;
                    resetFlag = true;
                }

                stepIndex++;
                if (stepIndex >= stepsSize) {
                    stepIndex = 0;
                }
            }
        }

        if (isProcessComplete) {
            break;
        }
    }
    return isPoleDetected;
}

template<>
bool processRotationAndDetectPole<RotationStepMiddle>(const RotationStepMiddle steps[], size_t stepsSize, bool isMiddleStage) {
    bool isPoleDetected = false; 

    for (uint8_t currentStepIndex = 0; currentStepIndex < stepsSize; ++currentStepIndex) {
        const auto& currentStep = steps[currentStepIndex];

        isPoleDetected = isPoleDetectedRotatingFromIntersection(
                currentStep.direction,
                isSensorActiveWrapper,
                currentStep.sensorType,
                navigation_control::rotationSpeed);
        stopAndStabilize();
    }
    return isPoleDetected;
}

bool processRotationAndDetectPoleLeft() {
    return processRotationAndDetectPole(stepsLeft_,
                     sizeof(stepsLeft_) / sizeof(stepsLeft_[0]));
}

bool processRotationAndDetectPoleRight() {
    return processRotationAndDetectPole(stepsRight_,
                     sizeof(stepsRight_) / sizeof(stepsRight_[0]));
}

bool processRotationAndDetectPoleMiddle() {
    return processRotationAndDetectPole(stepsMid_,
                     sizeof(stepsMid_) / sizeof(stepsMid_[0]), true);
}

bool isPoleDetectedWhileChecking() {
    if (processRotationAndDetectPoleLeft()) {
        isPoleFoundOnLeft_ = true;
        return true;
    }

    if(processRotationAndDetectPoleMiddle()) {
        isPoleDetectedOppositeDirection_ = true;
        return true;
    }
    
    if (processRotationAndDetectPoleRight()) {
        isPoleFoundOnRight_ = true;
        return true;
    }
    
    return false;
}

void writeInferredDataToMemory() {
    auto writeLocationToMemory = [](uint8_t value, uint8_t address) {
        if (value != static_cast<uint8_t>(0)) { 
            robot_.writeMemory(address, value);
            _delay_ms(100);
        }
    };

    writeLocationToMemory(static_cast<uint8_t>(locationDataFirstCourse_.startingPosition), configRapport::departureNodeFirstCourseAdress);
    writeLocationToMemory(static_cast<uint8_t>(locationDataFirstCourse_.poleLocation), configRapport::poleLocationFirstCourseAdress);

    writeLocationToMemory(static_cast<uint8_t>(locationDataSecondCourse_.poleLocation), configRapport::poleLocationSecondCourseAdress);

    writeLocationToMemory(static_cast<uint8_t>(locationDataThirdCourse_.poleLocation), configRapport::poleLocationThirdCourseAdress);
}

void resetStateData() {
    triggeredSensor_ = SensorType::NONE;
    lastRotation_ = RotationState::NONE;

    isPoleFoundOnLeft_ = false;
    isPoleFoundOnRight_ = false;
    isPoleDetectedOppositeDirection_ = false;
}

void executeModeDefault() {
    bool isPoleDetected = false;

    while (followLineUntil(detectIntersection)) { 
        if (!isPoleDetected) {
            isPoleDetected = isPoleDetectedBeforeIntersection(); 
        }
    }
    stopAndStabilize(); 

    followLineForDuration(navigation_control::POST_INTERSECTION_DELAY_MS);
    stopAndStabilize();

    while (!isPoleDetected) {
        isPoleDetected = isPoleDetectedWhileChecking();
    }
    stopAndStabilize();  
   
    while (followLineUntil(detectPoleWrapper)) {};
    stopAndStabilize();
}

void executeFirstCourse() {   
    executeModeDefault();
    processInferredDataFirstCourse();
    resetStateData();
}

void executeSecondCourse() {
    executeModeDefault();
    processInferredDataSecondCourse();
    resetStateData();
}

void executeThirdCourse() {
    executeModeDefault();
    processInferredDataThirdCourse();
    resetStateData();
}

void executeFinalCourse() {
    // add logic to reach E
}

int main() {
    initialize();

    executeFirstCourse();   
    // add logic here to turn
    executeSecondCourse();
    // add logic here to turn
    executeThirdCourse();
    // add logic here to turn
    executeFinalCourse();

    
    writeInferredDataToMemory();
    return 0;
}

// int main() {

//     executeFirstCourse();
//     return 0;
// }