#include "avr/io.h"
#include "util/delay.h"
#include "avr/interrupt.h"

#include "debug/debug_uart.h"
#include "sensor/infrared_sensor.h"
#include "robot.h"

namespace navigation_control {
    static constexpr uint16_t POST_INTERSECTION_FAR_RIGHT_DELAY_MS = 3200;
    static constexpr uint16_t POST_INTERSECTION_FAR_LEFT_DELAY_MS = 3000;
    static constexpr uint16_t ADVANCE_DELAY_MS = 700;
    static constexpr uint8_t forwardSpeed  = 80;
    static constexpr uint8_t rotationSpeed = 100;
    
   // static constexpr uint8_t durationRotationSpeed = 80;
    static constexpr uint16_t STABILIZATION_DELAY_MS = 1000;
}

namespace memory {
    static constexpr uint16_t MEMORY_DELAY_MS = 100;
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
    UNKNOWN = 0,
};

enum class PoleLocationSecondCourse {
    THREE = 3,
    FOUR = 4,
    UNKNOWN = 0,
};

enum class StartingPlaceThirdCourse {
    THREE = 3,
    FOUR = 4,
    UNKNOWN = 0,
};

enum class PoleLocationThirdCourse {
    FIVE = 5,
    SIX = 6,
    UNKNOWN = 0,
};

template <typename StartingPlaceEnum, typename PoleLocationEnum>
struct InferredData {
    StartingPlaceEnum startingPosition = StartingPlaceEnum::UNKNOWN;
    PoleLocationEnum poleLocation = PoleLocationEnum::UNKNOWN;

    InferredData() = default;
};

using InferredDataFirstCourse = InferredData<StartingPlaceFirstCourse, PoleLocationFirstCourse>;
using InferredDataSecondCourse = InferredData<StartingPlaceSecondCourse, PoleLocationSecondCourse>;
using InferredDataThirdCourse = InferredData<StartingPlaceThirdCourse, PoleLocationThirdCourse>;

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
    DETECTED_IN_MIDDLE_SLIGHT_RIGHT,
    DETECTED_IN_RIGHT_EMPTY_SPACE
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
    {RotationDirection::LEFT, SensorType::SHORT_LEFT, RotationStageMiddle::DETECTED_IN_MIDDLE_SLIGHT_RIGHT}
};

constexpr RotationStepLeft stepsLeft_[] = {
    {RotationDirection::LEFT, SensorType::NONE, RotationStageLeft::DETECTED_IN_LEFT_ADJUSTMENT},
    {RotationDirection::LEFT, SensorType::FAR_LEFT, RotationStageLeft::DETECTED_IN_LEFT_ROTATION},
    {RotationDirection::LEFT, SensorType::NONE, RotationStageLeft::DETECTED_IN_LEFT_EMPTY_SPACE},
    {RotationDirection::RIGHT, SensorType::MID, RotationStageLeft::DETECTED_IN_MIDDLE_RETURN},
    {RotationDirection::RIGHT, SensorType::NONE, RotationStageLeft::DETECTED_IN_RIGHT_EMPTY_SPACE},
    {RotationDirection::RIGHT, SensorType::FAR_RIGHT, RotationStageLeft::DETECTED_IN_RIGHT_ROTATION},
};

constexpr RotationStepRight stepsRight_[] = {
    {RotationDirection::RIGHT, SensorType::NONE, RotationStageRight::DETECTED_IN_RIGHT_ADJUSTMENT},
    {RotationDirection::RIGHT, SensorType::FAR_RIGHT, RotationStageRight::DETECTED_IN_RIGHT_ROTATION},
    {RotationDirection::RIGHT, SensorType::NONE, RotationStageRight::DETECTED_IN_RIGHT_EMPTY_SPACE},
    {RotationDirection::LEFT, SensorType::MID, RotationStageRight::DETECTED_IN_MIDDLE_RETURN},
    {RotationDirection::LEFT, SensorType::NONE, RotationStageRight::DETECTED_IN_LEFT_EMPTY_SPACE},
    {RotationDirection::LEFT, SensorType::FAR_LEFT, RotationStageRight::DETECTED_IN_LEFT_ROTATION},
};

namespace configRapport {
    constexpr uint16_t DEPARTURE_NODE_FIRST_COURSE_ADDRESS = 0x0004;
    constexpr uint16_t POLE_LOCATION_FIRST_COURSE_ADDRESS = 0x0005;
    constexpr uint16_t POLE_LOCATION_SECOND_COURSE_ADDRESS = 0x0006;
    constexpr uint16_t POLE_LOCATION_THIRD_COURSE_ADDRESS = 0x0007;
}

namespace configSound {
    constexpr uint16_t SOUND_FREQUENCY = 220;
    constexpr uint16_t SOUND_DURATION = 750;
    constexpr uint16_t SOUND_OFF_DELAY = 50;
}

Robot& robot_ = Robot::getInstance();
InfraredSensor& infraredSensor_ = InfraredSensor::getInstance();

static InferredDataFirstCourse locationDataFirstCourse_;
static InferredDataSecondCourse locationDataSecondCourse_;
static InferredDataThirdCourse locationDataThirdCourse_;

struct CourseIntersectionStatus {
    bool partialIntersection = false;
    bool fullIntersection = false;

    CourseIntersectionStatus() = default;
};

struct CourseStatus {
    bool isPoleFoundOnLeft = false;
    bool isPoleFoundOnRight = false;
    bool isPoleDetectedOppositeDirection = false;

    CourseStatus() = default;
};

enum class CurrentCourse {
    FIRST = 0,
    SECOND = 1,
    THIRD = 2,
};

CurrentCourse currentCourse_;
static CourseStatus courseStatuses_[3];
static CourseIntersectionStatus firstCourse_;
static CourseIntersectionStatus thirdCourse_;
static SensorType triggeredSensor_;

void initialize() {
    cli();
    currentCourse_ = CurrentCourse::FIRST;
    triggeredSensor_ = SensorType::INVALID;
    sei();
}

void stopAndStabilize() {
    robot_.haltMotor();
    _delay_ms(navigation_control::STABILIZATION_DELAY_MS);
}

void playSound() {
    robot_.playNote(configSound::SOUND_FREQUENCY);
    _delay_ms(configSound::SOUND_DURATION);
    robot_.turnOffSound();
    _delay_ms(configSound::SOUND_OFF_DELAY);

    robot_.playNote(configSound::SOUND_FREQUENCY);
    _delay_ms(configSound::SOUND_DURATION);
    robot_.turnOffSound();
}

void followLineDefault() {
    robot_.moveForward(navigation_control::forwardSpeed);
    if (robot_.isSensorActive(SensorType::SHORT_LEFT)) {
        robot_.controlLeftMotorSpeed(navigation_control::forwardSpeed - 15);
        robot_.controlRightMotorSpeed(navigation_control::forwardSpeed + 5);
    } else if (robot_.isSensorActive(SensorType::SHORT_RIGHT)) {
        robot_.controlRightMotorSpeed(navigation_control::forwardSpeed - 10);
    } else if (robot_.isSensorActive(SensorType::MID)){
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

// void rotateForDuration(uint16_t durationMs, RotationDirection direction) {
//     constexpr uint16_t delayPerIteration = 2;  
//     uint16_t iterations = durationMs / delayPerIteration;

//     for (uint16_t i = 0; i < iterations; i++) {
//         if (direction == RotationDirection::LEFT) {
//             robot_.rotateLeft(navigation_control::durationRotationSpeed);  
//         } else if (direction == RotationDirection::RIGHT) {
//             robot_.rotateRight(navigation_control::durationRotationSpeed); 
//         }
//         _delay_ms(delayPerIteration);
//     }
// }

void quickAdvance(uint16_t durationMs) {
    constexpr uint16_t delayPerIteration = 2;
    uint16_t iterations = durationMs / delayPerIteration;

    for (uint16_t i = 0; i < iterations; i++) {
        robot_.controlLeftMotorSpeed(navigation_control::forwardSpeed + 5);
        robot_.controlRightMotorSpeed(navigation_control::forwardSpeed);
        _delay_ms(delayPerIteration);
    }
    stopAndStabilize();
}

bool followLineUntil(bool (*shouldExit)()) {
    followLineDefault();
    return !shouldExit();    
}

bool followLineUntil(SensorType (*shouldExit)(SensorType), SensorType sensorType) {
    followLineDefault();
    return !shouldExit(sensorType);
}
 
bool detectPoleWrapper() {
    return infraredSensor_.determineProximityState();
}

bool isSensorActiveWrapper(SensorType sensorType) {
    return robot_.isSensorActive(sensorType);
}

template <typename SensorFunction>
bool followLineUntil(SensorFunction shouldExit, SensorType& triggeredSensor) {
    followLineDefault();
    SensorType sensorType = shouldExit(triggeredSensor);  
    if (sensorType != SensorType::INVALID) {
        triggeredSensor = sensorType;  
        return false;
    }
    return true;
}

SensorType detectIntersection(SensorType& triggeredSensor) {
    if (robot_.isSensorActive(SensorType::FAR_LEFT)) {
        triggeredSensor = SensorType::FAR_LEFT;
        return SensorType::FAR_LEFT;
    } else if (robot_.isSensorActive(SensorType::FAR_RIGHT)) {
        triggeredSensor = SensorType::FAR_RIGHT;
        return SensorType::FAR_RIGHT;
    }
    return SensorType::INVALID;
}

bool rotateUntil(bool (*shouldExit)(SensorType), SensorType sensorType, uint8_t speed, RotationDirection direction) {
    if (direction == RotationDirection::LEFT) {
        robot_.rotateLeft(speed);
    } else {
        robot_.rotateRight(speed);
    }

    return !shouldExit(sensorType);
}

// enum DetectionResult {
//     LINE,
//     INTERSECTION
// };

// bool areTwoSensorsActiveNextToEachOther() {
//     if (robot_.isSensorActive(SensorType::FAR_LEFT) && robot_.isSensorActive(SensorType::SHORT_LEFT) &&
//         !robot_.isSensorActive(SensorType::MID) && !robot_.isSensorActive(SensorType::SHORT_RIGHT) && !robot_.isSensorActive(SensorType::FAR_RIGHT)) {
//         return true;
//     }

//     if (robot_.isSensorActive(SensorType::SHORT_LEFT) && robot_.isSensorActive(SensorType::MID) &&
//         !robot_.isSensorActive(SensorType::FAR_LEFT) && !robot_.isSensorActive(SensorType::SHORT_RIGHT) && !robot_.isSensorActive(SensorType::FAR_RIGHT)) {
//         return true;
//     }

//     if (robot_.isSensorActive(SensorType::MID) && robot_.isSensorActive(SensorType::SHORT_RIGHT) &&
//         !robot_.isSensorActive(SensorType::FAR_LEFT) && !robot_.isSensorActive(SensorType::SHORT_LEFT) && !robot_.isSensorActive(SensorType::FAR_RIGHT)) {
//         return true;
//     }

//     if (robot_.isSensorActive(SensorType::SHORT_RIGHT) && robot_.isSensorActive(SensorType::FAR_RIGHT) &&
//         !robot_.isSensorActive(SensorType::FAR_LEFT) && !robot_.isSensorActive(SensorType::SHORT_LEFT) && !robot_.isSensorActive(SensorType::MID)) {
//         return true;
//     }

//     return false;
// }

// DetectionResult adjustAndVerifie(RotationDirection direction) {
//     rotateForDuration(430, direction);

//     if (areTwoSensorsActiveNextToEachOther()) {
//         return DetectionResult::LINE;
//     } else {
//         return DetectionResult::INTERSECTION;
//     }
// }

void processInferredDataFirstCourse() {
    CourseStatus& firstCourseStatus = courseStatuses_[static_cast<int>(CurrentCourse::FIRST)];

    if (firstCourseStatus.isPoleDetectedOppositeDirection) {
        if (firstCourse_.partialIntersection) {
            locationDataFirstCourse_ = {StartingPlaceFirstCourse::D, PoleLocationFirstCourse::ONE};
        } else if (firstCourse_.fullIntersection) {
            locationDataFirstCourse_ = {StartingPlaceFirstCourse::C, PoleLocationFirstCourse::ONE};
        }
    } else {
        if (firstCourseStatus.isPoleFoundOnRight) {
            locationDataFirstCourse_ = {StartingPlaceFirstCourse::D, PoleLocationFirstCourse::TWO};
        } else if (firstCourseStatus.isPoleFoundOnLeft) {
            locationDataFirstCourse_ = {StartingPlaceFirstCourse::C, PoleLocationFirstCourse::TWO};
        }
    }
}

void processInferredDataSecondCourse() {
    CourseStatus& secondCourseStatus = courseStatuses_[static_cast<int>(CurrentCourse::SECOND)];

    if (secondCourseStatus.isPoleDetectedOppositeDirection) {
        if (firstCourse_.partialIntersection) {
            locationDataSecondCourse_ = {StartingPlaceSecondCourse::ONE, PoleLocationSecondCourse::FOUR};
        } else if (firstCourse_.fullIntersection) {
            locationDataSecondCourse_ = {StartingPlaceSecondCourse::TWO, PoleLocationSecondCourse::THREE};
        }
    } else {
        if (secondCourseStatus.isPoleFoundOnRight) {
            locationDataSecondCourse_ = {StartingPlaceSecondCourse::TWO, PoleLocationSecondCourse::FOUR};
        } else if (secondCourseStatus.isPoleFoundOnLeft) {
            locationDataSecondCourse_ = {StartingPlaceSecondCourse::ONE, PoleLocationSecondCourse::THREE};
        }
    }
}

void processInferredDataThirdCourse() {
    CourseStatus& thirdCourseStatus = courseStatuses_[static_cast<int>(CurrentCourse :: THIRD)];

    if(thirdCourseStatus.isPoleDetectedOppositeDirection){
        if (thirdCourse_. partialIntersection) {
            locationDataThirdCourse_ = {StartingPlaceThirdCourse :: FOUR, PoleLocationThirdCourse :: FIVE};
        } else if (thirdCourse_.fullIntersection) {
            locationDataThirdCourse_ = {StartingPlaceThirdCourse :: THREE,PoleLocationThirdCourse :: SIX};
        }
    } else {
        if (thirdCourseStatus.isPoleFoundOnRight) {
            locationDataThirdCourse_ = {StartingPlaceThirdCourse :: FOUR, PoleLocationThirdCourse :: SIX};
        } else if (thirdCourseStatus.isPoleFoundOnLeft) {
            locationDataThirdCourse_ = {StartingPlaceThirdCourse :: THREE, PoleLocationThirdCourse :: FIVE};
        }       
    }
}


bool isPoleDetectedFromExtremity() {
    CourseStatus& currentCourseStatus = courseStatuses_[static_cast<int>(currentCourse_)];
    if (infraredSensor_.isPoleDetectedFromExtremity()) {
        currentCourseStatus.isPoleDetectedOppositeDirection = true;
        return true;
    }
    return false;
}

bool isPoleDetectedFromIntersection(RotationDirection direction) {
    CourseStatus& currentCourseStatus = courseStatuses_[static_cast<int>(currentCourse_)];
    if (infraredSensor_.isPoleDetectedFromIntersection()) {
        if (direction == RotationDirection::LEFT) {
            currentCourseStatus.isPoleFoundOnLeft = true;
        } else {
            currentCourseStatus.isPoleFoundOnRight = true;
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
        if (!isPoleDetected) {
            isPoleDetected = isPoleDetectedFromIntersection(direction);
        }
    }

    return isPoleDetected;
}

enum class RotationStepType {
    Left,
    Right,
    Middle
};

bool processRotationAndDetectPole(const void* steps, size_t stepsSize, RotationStepType type) {
    bool isPoleDetected = false;
    bool isProcessComplete = false;

    for (uint8_t currentStepIndex = 0; currentStepIndex < stepsSize; ++currentStepIndex) {
        switch (type) {
            case RotationStepType::Left: {
                const auto& currentStep = reinterpret_cast<const RotationStepLeft*>(steps)[currentStepIndex];

                if (currentStep.stage == RotationStageLeft::DETECTED_IN_LEFT_ADJUSTMENT) {
                    while (rotateUntil(isSensorActiveWrapper, currentStep.sensorType, navigation_control::rotationSpeed, currentStep.direction));
                    stopAndStabilize();
                } else if (currentStep.stage == RotationStageLeft::DETECTED_IN_RIGHT_ROTATION) {
                    while (rotateUntil(isSensorActiveWrapper, currentStep.sensorType, navigation_control::rotationSpeed, currentStep.direction));
                    stopAndStabilize();
                } else {
                    isPoleDetected = isPoleDetectedRotatingFromIntersection(
                        currentStep.direction,
                        isSensorActiveWrapper,
                        currentStep.sensorType,
                        navigation_control::rotationSpeed);
                }
                break;
            }
            case RotationStepType::Right: {
                const auto& currentStep = reinterpret_cast<const RotationStepRight*>(steps)[currentStepIndex];

                if (currentStep.stage == RotationStageRight::DETECTED_IN_RIGHT_ADJUSTMENT) {
                    while (rotateUntil(isSensorActiveWrapper, currentStep.sensorType, navigation_control::rotationSpeed, currentStep.direction));
                    stopAndStabilize();
                } else if (currentStep.stage == RotationStageRight::DETECTED_IN_LEFT_ROTATION) {
                    while (rotateUntil(isSensorActiveWrapper, currentStep.sensorType, navigation_control::rotationSpeed, currentStep.direction));
                    stopAndStabilize();
                } else {
                    isPoleDetected = isPoleDetectedRotatingFromIntersection(
                        currentStep.direction,
                        isSensorActiveWrapper,
                        currentStep.sensorType,
                        navigation_control::rotationSpeed);
                }
                break;
            }
            case RotationStepType::Middle: {
                const auto& currentStep = reinterpret_cast<const RotationStepMiddle*>(steps)[currentStepIndex];
                isPoleDetected = isPoleDetectedRotatingFromIntersection(
                    currentStep.direction,
                    isSensorActiveWrapper,
                    currentStep.sensorType,
                    navigation_control::rotationSpeed);
                break;
            }
        }

        stopAndStabilize();

        if (isPoleDetected) {
            uint8_t stepIndex = currentStepIndex + 1;
            bool resetFlag = false;

            while (!resetFlag) {
                switch (type) {
                    case RotationStepType::Left: {
                        const auto& step = reinterpret_cast<const RotationStepLeft*>(steps)[stepIndex];
                        while (rotateUntil(isSensorActiveWrapper, step.sensorType, navigation_control::rotationSpeed, step.direction));
                        stopAndStabilize();
                        if (step.stage == RotationStageLeft::DETECTED_IN_MIDDLE_RETURN) {
                            isProcessComplete = true;
                            resetFlag = true;
                        }
                        break;
                    }
                    case RotationStepType::Right: {
                        const auto& step = reinterpret_cast<const RotationStepRight*>(steps)[stepIndex];
                        while (rotateUntil(isSensorActiveWrapper, step.sensorType, navigation_control::rotationSpeed, step.direction));
                        stopAndStabilize();
                        if (step.stage == RotationStageRight::DETECTED_IN_MIDDLE_RETURN) {
                            isProcessComplete = true;
                            resetFlag = true;
                        }
                        break;
                    }
                    case RotationStepType::Middle: {
                        const auto& step = reinterpret_cast<const RotationStepMiddle*>(steps)[stepIndex];
                        while (rotateUntil(isSensorActiveWrapper, step.sensorType, navigation_control::rotationSpeed, step.direction));
                        stopAndStabilize();
                        if (step.stage == RotationStageMiddle::DETECTED_IN_MIDDLE_SLIGHT_LEFT ||
                            step.stage == RotationStageMiddle::DETECTED_IN_MIDDLE_SLIGHT_RIGHT) {
                            isProcessComplete = true;
                            resetFlag = true;
                        }
                        break;
                    }
                    default:
                        break;
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

bool processRotationAndDetectPoleLeft() {
    return processRotationAndDetectPole(stepsLeft_, sizeof(stepsLeft_) / sizeof(stepsLeft_[0]), RotationStepType::Left);
}

bool processRotationAndDetectPoleRight() {
    return processRotationAndDetectPole(stepsRight_, sizeof(stepsRight_) / sizeof(stepsRight_[0]), RotationStepType::Right);
}

bool processRotationAndDetectPoleMiddle() {
    return processRotationAndDetectPole(stepsMid_, sizeof(stepsMid_) / sizeof(stepsMid_[0]), RotationStepType::Middle);
}

bool isPoleDetectedWhileChecking() {
    static bool isFirstIteration = true;
    CourseStatus& currentCourseStatus = courseStatuses_[static_cast<int>(currentCourse_)];

    if (processRotationAndDetectPoleMiddle()) {
        currentCourseStatus.isPoleDetectedOppositeDirection = true;
        return true;
    }

    if (isFirstIteration) {
        if (triggeredSensor_ == SensorType::FAR_LEFT && processRotationAndDetectPoleLeft()) {
            currentCourseStatus.isPoleFoundOnLeft = true;
            isFirstIteration = false;
            return true;
        }

        if (triggeredSensor_ == SensorType::FAR_RIGHT && processRotationAndDetectPoleRight()) {
            currentCourseStatus.isPoleFoundOnRight = true;
            isFirstIteration = false;
            return true;
        }
    }

    if (processRotationAndDetectPoleLeft()) {
        currentCourseStatus.isPoleFoundOnLeft = true;
        return true;
    }

    if (processRotationAndDetectPoleRight()) {
        currentCourseStatus.isPoleFoundOnRight = true;
        return true;
    }

    return false;
}

void writeInferredDataToMemory() {
    auto writeLocationToMemory = [](uint8_t value, uint8_t address) {
        if (value != static_cast<uint8_t>(0)) {
            robot_.writeMemory(address, value);
            _delay_ms(memory::MEMORY_DELAY_MS);
        }
    };

    writeLocationToMemory(static_cast<uint8_t>(locationDataFirstCourse_.startingPosition), configRapport::DEPARTURE_NODE_FIRST_COURSE_ADDRESS);
    writeLocationToMemory(static_cast<uint8_t>(locationDataFirstCourse_.poleLocation), configRapport::POLE_LOCATION_FIRST_COURSE_ADDRESS);
    writeLocationToMemory(static_cast<uint8_t>(locationDataSecondCourse_.poleLocation), configRapport::POLE_LOCATION_SECOND_COURSE_ADDRESS);
    writeLocationToMemory(static_cast<uint8_t>(locationDataThirdCourse_.poleLocation), configRapport::POLE_LOCATION_THIRD_COURSE_ADDRESS);
}

void followLineUntilIntersection() {
    while (followLineUntil(detectIntersection, triggeredSensor_));
    stopAndStabilize();
}


void handlePostIntersectionAdjustment() {
    if (triggeredSensor_ == SensorType::FAR_LEFT) {
        followLineForDuration(navigation_control::POST_INTERSECTION_FAR_LEFT_DELAY_MS);
    } else if (triggeredSensor_ == SensorType::FAR_RIGHT) {
        followLineForDuration(navigation_control::POST_INTERSECTION_FAR_RIGHT_DELAY_MS);
    }
    stopAndStabilize();
}

void waitForPoleDetection() {
    bool isPoleDetected = false;
    while (!isPoleDetected) {
        isPoleDetected = isPoleDetectedWhileChecking();
    }
    stopAndStabilize();
}

void executeModeDefault() {
    bool isPoleDetected = false;

    isPoleDetected = isPoleDetectedFromExtremity();
    followLineUntilIntersection();
    // bool keepFollowingLine = true;
    // while (keepFollowingLine) {
    //     DetectionResult detectionResult;

    //     followLineUntilIntersection();

    //     if (triggeredSensor_ == SensorType::FAR_LEFT) {
    //         detectionResult = adjustAndVerifie(RotationDirection::LEFT);
    //         if (detectionResult == DetectionResult::LINE) {
    //             triggeredSensor_ = SensorType::INVALID;
    //         } else {
    //             keepFollowingLine = false;
    //         }
    //     }
    //     else if (triggeredSensor_ == SensorType::FAR_RIGHT) {
    //         detectionResult = adjustAndVerifie(RotationDirection::RIGHT);
    //         if (detectionResult == DetectionResult::LINE) {
    //             triggeredSensor_ = SensorType::INVALID;
    //         } else {
    //             keepFollowingLine = false;
    //         }
    //     }
    // }
    quickAdvance(navigation_control::ADVANCE_DELAY_MS);
    
    if (!isPoleDetected) {       
        handlePostIntersectionAdjustment();
        waitForPoleDetection();       
    }

    while(followLineUntil(detectPoleWrapper));
    stopAndStabilize();  
    
    playSound();
}

void executeFirstCourse() {
    currentCourse_ = CurrentCourse::FIRST;
    executeModeDefault();
    processInferredDataFirstCourse();
}

void executeSecondCourse() {
    currentCourse_ = CurrentCourse::SECOND;
    executeModeDefault();
    processInferredDataSecondCourse();
}

void executeThirdCourse() {
    currentCourse_ = CurrentCourse::THIRD;
    executeModeDefault();
    processInferredDataThirdCourse();
}

void executeFinalCourse() {
    // add logic to reach E
}

// mode_1 code
// void partialIntersection(RotationDirection direction) {
//     _delay_ms(500);
//     robot_.moveForward(navigation::forwardSpeed);

//     if (direction == RotationDirection::RIGHT) {
//         robot_.controlLeftMotorSpeed(navigation::forwardSpeed + 15);
//     } else {
//         robot_.controlRightMotorSpeed(navigation::forwardSpeed + 25);
//     }

//     while (robot_.isSensorActive(SensorType::FAR_LEFT) || 
//            robot_.isSensorActive(SensorType::FAR_RIGHT));

//     stopAndStabilize();

//     auto shouldExitFollow = []() {
//         return robot_.noSensorsActive();
//     };

//     while (!followLineUntil(shouldExitFollow));

//     stopAndStabilize();

//     if (direction == RotationDirection::RIGHT) {
//         while (rotateUntil(isSensorActiveWrapper, SensorType::MID, navigation::forwardSpeed, RotationDirection::RIGHT));
//     } else {
//         while (rotateUntil(isSensorActiveWrapper, SensorType::MID, navigation::forwardSpeed, RotationDirection::LEFT));
//     }
//     stopAndStabilize();
// }

// enum class IntersectionType {
//     LINE,
//     PARTIAL,
//     FULL
// };

// constexpr uint8_t ROTATION_SPEED = 70;
// constexpr uint16_t MOVEMENT_DELAY = 1500;
// constexpr uint8_t STOP_DELAY = 100;

// SensorType getFarSensor(RotationDirection direction) {
//     return (direction == RotationDirection::LEFT) ? SensorType::FAR_LEFT : SensorType::FAR_RIGHT;
// }

// SensorType getOppositeFarSensor(RotationDirection direction) {
//     return (direction == RotationDirection::LEFT) ? SensorType::FAR_RIGHT : SensorType::FAR_LEFT;
// }

// IntersectionType adjustAndVerify(RotationDirection direction) {
//     const SensorType farSensor = getFarSensor(direction);
//     const SensorType oppositeFarSensor = getOppositeFarSensor(direction);

//     rotateUntil(isSensorActiveWrapper, SensorType::MID, ROTATION_SPEED, direction);
//     stopAndStabilize();

//     if (robot_.isSensorActive(farSensor)) {
//         robot_.moveForward(navigation::forwardSpeed);

//         while (robot_.isSensorActive(farSensor) && !robot_.isSensorActive(oppositeFarSensor));

//         if (robot_.isSensorActive(oppositeFarSensor)) {
//             return IntersectionType::FULL;
//         } else {
//             return IntersectionType::PARTIAL;
//         }
//     }

//     return IntersectionType::LINE;
// }

// void fullIntersection(RotationDirection direction) {
//     robot_.moveForward(navigation::forwardSpeed);
//     _delay_ms(MOVEMENT_DELAY);
//     stopAndStabilize();
//     rotateUntil(isSensorActiveWrapper, SensorType::MID, navigation_control::rotationSpeed, direction);
// }

// void handleNoSensorsActive() {
//     stopAndStabilize();
//     while (rotateUntil(isSensorActiveWrapper, SensorType::NONE, navigation::rightMotorSpeed, RotationDirection::RIGHT));
// }

// void updateCourseStatus(CourseIntersectionStatus& courseStatus, bool isPartial, bool isFull) {
//     if (isPartial) {
//         courseStatus.partialIntersection = true;
//     }
//     if (isFull) {
//         courseStatus.fullIntersection = true;
//     }
// }

// bool handleSensorActive(RotationDirection direction, CourseIntersectionStatus& courseStatus) {
//     stopAndStabilize();
//     IntersectionType intersectionType = adjustAndVerify(direction);

//     if (intersectionType == IntersectionType::PARTIAL) {
//         partialIntersection(direction);
//         updateCourseStatus(courseStatus, true, false);
//         return true;
//     }

//     if (intersectionType == IntersectionType::FULL) {
//         robot_.moveForward(navigation::forwardSpeed);
//         while (robot_.isSensorActive(LineSensor::FAR_LEFT) || 
//                robot_.isSensorActive(LineSensor::FAR_RIGHT));
//         _delay_ms(STOP_DELAY);

//         if (robot_.noSensorsActive()) {
//             fullIntersection(direction);
//             updateCourseStatus(courseStatus, false, true);
//             return true;
//         }
//     }

//     return false;
// }

// void transitionFromCourseToAnother() {
//     bool isTransitionComplete = false;

//     while (!isTransitionComplete) {
//         if (robot_.noSensorsActive()) {
//             handleNoSensorsActive();
//         } else if (robot_.isSensorActive(SensorType::FAR_RIGHT)) {
//             CourseIntersectionStatus courseStatus = (currentCourse_ == CurrentCourse::FIRST) ? firstCourse_ : thirdCourse_;
//             isTransitionComplete = handleSensorActive(RotationDirection::RIGHT, courseStatus);
//         } else if (robot_.isSensorActive(SensorType::FAR_LEFT)) {
//             CourseIntersectionStatus courseStatus = (currentCourse_ == CurrentCourse::FIRST) ? firstCourse_ : thirdCourse_;
//             isTransitionComplete = handleSensorActive(RotationDirection::LEFT, courseStatus);
//         } else {
//             followLineDefault();
//         }
//     }

//     followLineForDuration(700);
// }

int main() {
    initialize();

    executeFirstCourse();
    //transitionFromCourseToAnother();
    //executeSecondCourse();
    //transitionFromCourseToAnother();
    //executeThirdCourse();
    //transitionFromCourseToAnother();
    //executeFinalCourse();

    writeInferredDataToMemory();
    return 0;
}

// while(true){
//     if (robot_.isSensorActive(SensorType::FAR_RIGHT)) {
//         stopAndStabilize();

//         IntersectionType typeOfIntersection = adjustAndVerify(RotationDirection::RIGHT);
//         if (typeOfIntersection == IntersectionType::PARTIAL) {
//             partialIntersection(RotationDirection::RIGHT);
//             break;
//         } else if (typeOfIntersection == IntersectionType::FULL) {
//             robot_.moveForward(navigation::forwardSpeed);
//             while (robot_.isSensorActive(LineSensor::FAR_LEFT) || robot_.isSensorActive(LineSensor::FAR_RIGHT));
//             _delay_ms(100);
//             if (robot_.noSensorsActive()) {
//                 fullIntersection(RotationDirection::RIGHT);
//                 break;
//             }
//         }
//     } else if (robot_.isSensorActive(SensorType::FAR_LEFT)) {
//         stopAndStabilize();

//         IntersectionType typeOfIntersection = adjustAndVerify(RotationDirection::LEFT);
//         if (typeOfIntersection == IntersectionType::PARTIAL) {
//             partialIntersection(RotationDirection::LEFT);
//             break;
//         } else if (typeOfIntersection == IntersectionType::FULL) {
//             robot_.moveForward(navigation::forwardSpeed);
//             while (robot_.isSensorActive(LineSensor::FAR_LEFT) || robot_.isSensorActive(LineSensor::FAR_RIGHT));
//             _delay_ms(100);
//             if (robot_.noSensorsActive()) {
//                 fullIntersection(RotationDirection::LEFT);
//                 break;
//             }
//         }
//     } else if (robot_.noSensorsActive()) {
//         robot_.haltMotor();
//         robot_.rotateRight(navigation::rightMotorSpeed);
//         while (robot_.noSensorsActive());
//     } else {
//         followLineDefault();
//     }
// }