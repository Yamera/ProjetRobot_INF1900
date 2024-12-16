#include "../defs/mode/cross_course.h"
#include "debug/debug_uart.h"
#include "definitions/gpio_macros.h"

#include "avr/io.h"
#include "util/delay.h"
#include "avr/interrupt.h"

CrossCourseMode *CrossCourseMode::instance_ = nullptr;

RotationStepMiddle CrossCourseMode::stepsMid_[] = {
    {RotationDirection::LEFT, SensorType::NONE, RotationStageMiddle::DETECTED_IN_LEFT_EMPTY_SPACE},
    {RotationDirection::RIGHT, SensorType::SHORT_RIGHT, RotationStageMiddle::DETECTED_IN_MIDDLE_SLIGHT_LEFT},
    {RotationDirection::RIGHT, SensorType::NONE, RotationStageMiddle::DETECTED_IN_RIGHT_EMPTY_SPACE},
    {RotationDirection::LEFT, SensorType::SHORT_LEFT, RotationStageMiddle::DETECTED_IN_MIDDLE_SLIGHT_RIGHT}};

RotationStepLeft CrossCourseMode::stepsLeft_[] = {
    {RotationDirection::LEFT, SensorType::NONE, RotationStageLeft::DETECTED_IN_LEFT_ADJUSTMENT},
    {RotationDirection::LEFT, SensorType::FAR_LEFT, RotationStageLeft::DETECTED_IN_LEFT_ROTATION},
    {RotationDirection::LEFT, SensorType::NONE, RotationStageLeft::DETECTED_IN_LEFT_EMPTY_SPACE},
    {RotationDirection::RIGHT, SensorType::MID, RotationStageLeft::DETECTED_IN_MIDDLE_RETURN}, 
    {RotationDirection::RIGHT, SensorType::NONE, RotationStageLeft::DETECTED_IN_RIGHT_EMPTY_SPACE},
    {RotationDirection::RIGHT, SensorType::FAR_RIGHT, RotationStageLeft::DETECTED_IN_RIGHT_ROTATION},
};

RotationStepRight CrossCourseMode::stepsRight_[] = {
    {RotationDirection::RIGHT, SensorType::NONE, RotationStageRight::DETECTED_IN_RIGHT_ADJUSTMENT},
    {RotationDirection::RIGHT, SensorType::FAR_RIGHT, RotationStageRight::DETECTED_IN_RIGHT_ROTATION},
    {RotationDirection::RIGHT, SensorType::NONE, RotationStageRight::DETECTED_IN_RIGHT_EMPTY_SPACE},
    {RotationDirection::LEFT, SensorType::MID, RotationStageRight::DETECTED_IN_MIDDLE_RETURN},
    {RotationDirection::LEFT, SensorType::NONE, RotationStageRight::DETECTED_IN_LEFT_EMPTY_SPACE},
    {RotationDirection::LEFT, SensorType::FAR_LEFT, RotationStageRight::DETECTED_IN_LEFT_ROTATION},
};

CrossCourseMode::CrossCourseMode(Robot *robot) : ModeStrategy(robot)
{
    cli();
    if (!instance_)
    {
        instance_ = this;
        initialize();
    }
    sei();
}

void CrossCourseMode::updateNextState()
{
    switch (currentCourse_)
    {
    case CurrentCourse::INIT:
        currentCourse_ = CurrentCourse::FIRST;
        break;
    case CurrentCourse::FIRST:
        currentCourse_ = CurrentCourse::SECOND;
        break;
    case CurrentCourse::SECOND:
        currentCourse_ = CurrentCourse::THIRD;
        break;
    case CurrentCourse::THIRD:
        currentCourse_ = CurrentCourse::FINAL;
        break;
    default:
        currentCourse_ = CurrentCourse::INIT;
        break;
    }
}

void CrossCourseMode::execute()
{
    clignotement();

    while(!waitForRelease());
    _delay_ms(configTiming::INITIALIZATION_DELAY_MS);

    navigate();
}

void CrossCourseMode::internalButtonCallbackFunction()
{
    isButtonInternPressed_ = true;
}

void CrossCourseMode::internalButtonCallbackStatic()
{
    if (instance_)
    {
        instance_->internalButtonCallbackFunction();
    }
}

void CrossCourseMode::clignotement()
{
    for (uint8_t i = 0; i < configClignotement::totalBlinks; i++)
    {
        robot_->turnLedRed();
        _delay_ms(configClignotement::blinkIntervalMs);
        robot_->turnLedOff();
        _delay_ms(configClignotement::blinkIntervalMs);
    }
}

void CrossCourseMode::waitForButtonPress()
{
    while (!isButtonInternPressed_) {

    }
    
    isButtonInternPressed_ = false;
}

bool CrossCourseMode::isPressed() const {
    return BUTTON_PIN_REGISTER & _BV(BUTTON_INT_0_PIN);
}

void CrossCourseMode::initialize()
{
    cli();
    currentCourse_ = CurrentCourse::INIT;
    sei();
}

void CrossCourseMode::stopAndStabilize()
{
    robot_->haltMotor();
    _delay_ms(navigation_control::STABILIZATION_DELAY_MS);
}

void CrossCourseMode::playSoundCourseFin()
{
    for (int i = 0; i < 8; ++i)
    {
        robot_->playNote(configSoundCourse::SOUND_NOTE);
        _delay_ms(configSoundCourse::SOUND_DURATION);
        robot_->turnOffSound();
        _delay_ms(configSoundCourse::SOUND_OFF_DELAY);
    }
}

void CrossCourseMode::playSoundModeFin()
{
    for (int i = 0; i < 2; ++i)
    {
        robot_->playNote(configSoundMode::SOUND_NOTE);
        _delay_ms(configSoundMode::SOUND_DURATION);
        robot_->turnOffSound();
        _delay_ms(configSoundMode::SOUND_OFF_DELAY);
    }
}

void CrossCourseMode::followLineDefault()
{
    robot_->moveForward(navigation_control::FORWARD_SPEED);
    if (robot_->isSensorActive(SensorType::SHORT_LEFT))
    {
        robot_->controlLeftMotorSpeed(navigation_control::FORWARD_SPEED - 15);
        robot_->controlRightMotorSpeed(navigation_control::FORWARD_SPEED + 5);
    }
    else if (robot_->isSensorActive(SensorType::SHORT_RIGHT))
    {
        robot_->controlRightMotorSpeed(navigation_control::FORWARD_SPEED - 10);
    }
    else if (robot_->isSensorActive(SensorType::MID))
    {
        robot_->controlRightMotorSpeed(navigation_control::FORWARD_SPEED + 10);
        robot_->controlLeftMotorSpeed(navigation_control::FORWARD_SPEED);
    }
}

void CrossCourseMode::followLineSmart(uint8_t &nbStops)
{
    robot_->moveForward(100);
    if (robot_->isSensorActive(SensorType::SHORT_LEFT))
    {
        robot_->controlLeftMotorSpeed(navigation_control::FORWARD_SPEED - 15);
        robot_->controlRightMotorSpeed(navigation_control::FORWARD_SPEED + 5);
    }
    else if (robot_->isSensorActive(SensorType::SHORT_RIGHT))
    {
        robot_->controlRightMotorSpeed(navigation_control::FORWARD_SPEED - 10);
    }
    else if (robot_->isSensorActive(SensorType::MID))
    {
        robot_->controlRightMotorSpeed(navigation_control::FORWARD_SPEED + 10);
        robot_->controlLeftMotorSpeed(navigation_control::FORWARD_SPEED);
    }
    else if (robot_->isSensorActive(SensorType::FAR_LEFT))
    {
        robot_->rotateRight(70);
        while (!robot_->isSensorActive(LineSensor::MID))
        {
        };
        pause();
        nbStops++;
    }
    else if (robot_->isSensorActive(SensorType::FAR_RIGHT))
    {
        robot_->rotateLeft(70);
        while (!robot_->isSensorActive(LineSensor::MID))
        {
        };
        pause();
        nbStops++;
    }
}

void CrossCourseMode::followLineSmartForDuration(uint16_t durationMs)
{
    constexpr uint16_t delayPerIteration = 2;
    uint16_t iterations = durationMs / delayPerIteration;
    uint8_t nbsOfStops = 0;
    uint16_t totalStopDuration = 0;

    for (uint16_t i = 0; i < iterations; i++)
    {
        followLineSmart(nbsOfStops);

        if (nbsOfStops > 0)
        {
            totalStopDuration += nbsOfStops * 500;
            nbsOfStops = 0;
        }

        _delay_ms(delayPerIteration);
    }

    uint16_t remainingDuration = totalStopDuration;
    while (remainingDuration > 0)
    {
        followLineSmart(nbsOfStops);

        if (nbsOfStops > 0)
        {
            totalStopDuration += nbsOfStops * 500;
            nbsOfStops = 0;
        }

        _delay_ms(delayPerIteration);
        remainingDuration -= delayPerIteration;
    }
}

bool CrossCourseMode::followLineUntilDetectPole()
{
    uint8_t nbStops = -1;
    followLineSmart(nbStops);
    return !infraredSensor_.determineProximityState();
}

bool CrossCourseMode::followLineUntilIntersection()
{
    followLineDefault();
    SensorType detectedPrioritySensor = detectIntersectionForPriority();
    SensorType detectedSensor = detectIntersection();
    if (detectedSensor != SensorType::INVALID)
    {
        return false;
    }
    return true;
}

SensorType CrossCourseMode::detectIntersectionForPriority()
{
    if (robot_->isSensorActive(SensorType::FAR_LEFT))
    {
        return SensorType::FAR_LEFT;
    }
    else if (robot_->isSensorActive(SensorType::FAR_RIGHT))
    {
        return SensorType::FAR_RIGHT;
    }
    return SensorType::INVALID;
}

SensorType CrossCourseMode::detectIntersection()
{
    if (robot_->allSensorsActive())
    {
        return SensorType::NONE;
    }
    return SensorType::INVALID;
}

bool CrossCourseMode::rotateUntil(SensorType sensorType, uint8_t speed, RotationDirection direction)
{
    if (direction == RotationDirection::LEFT)
    {
        robot_->rotateLeft(speed);
    }
    else
    {
        robot_->rotateRight(speed);
    }

    return !robot_->isSensorActive(sensorType);
}

void CrossCourseMode::processInferredDataFirstCourse()
{
    CourseStatus &firstCourseStatus = courseStatuses_[static_cast<int>(CurrentCourse::FIRST)];

    if (firstCourseStatus.isPoleDetectedOppositeDirection)
    {
        if (firstCourse_.partialIntersection)
        {
            locationDataFirstCourse_ = {StartingPlaceFirstCourse::D, PoleLocationFirstCourse::ONE};
        }
        else if (firstCourse_.fullIntersection)
        {
            locationDataFirstCourse_ = {StartingPlaceFirstCourse::C, PoleLocationFirstCourse::TWO};
        }
    }
    else
    {
        if (firstCourseStatus.isPoleFoundOnRight)
        {
            locationDataFirstCourse_ = {StartingPlaceFirstCourse::D, PoleLocationFirstCourse::ONE};
        }
        else if (firstCourseStatus.isPoleFoundOnLeft)
        {
            locationDataFirstCourse_ = {StartingPlaceFirstCourse::C, PoleLocationFirstCourse::TWO};
        }
    }
}

void CrossCourseMode::processInferredDataSecondCourse()
{
    CourseStatus &secondCourseStatus = courseStatuses_[static_cast<int>(CurrentCourse::SECOND)];

    if (secondCourseStatus.isPoleDetectedOppositeDirection)
    {
        if (firstCourse_.partialIntersection)
        {
            locationDataSecondCourse_ = {StartingPlaceSecondCourse::ONE, PoleLocationSecondCourse::FOUR};
        }
        else if (firstCourse_.fullIntersection)
        {
            locationDataSecondCourse_ = {StartingPlaceSecondCourse::TWO, PoleLocationSecondCourse::THREE};
        }
    }
    else
    {
        if (secondCourseStatus.isPoleFoundOnRight)
        {
            locationDataSecondCourse_ = {StartingPlaceSecondCourse::TWO, PoleLocationSecondCourse::FOUR};
        }
        else if (secondCourseStatus.isPoleFoundOnLeft)
        {
            locationDataSecondCourse_ = {StartingPlaceSecondCourse::ONE, PoleLocationSecondCourse::THREE};
        }
    }
}

void CrossCourseMode::processInferredDataThirdCourse()
{
    CourseStatus &thirdCourseStatus = courseStatuses_[static_cast<int>(CurrentCourse ::THIRD)];

    if (thirdCourseStatus.isPoleDetectedOppositeDirection)
    {
        if (thirdCourse_.partialIntersection)
        {
            locationDataThirdCourse_ = {StartingPlaceThirdCourse ::THREE, PoleLocationThirdCourse ::SIX};
        }
        else if (thirdCourse_.fullIntersection)
        {
            locationDataThirdCourse_ = {StartingPlaceThirdCourse ::FOUR, PoleLocationThirdCourse ::FIVE};
        }
    }
    else
    {
        if (thirdCourseStatus.isPoleFoundOnRight)
        {
            locationDataThirdCourse_ = {StartingPlaceThirdCourse ::FOUR, PoleLocationThirdCourse ::SIX};
        }
        else if (thirdCourseStatus.isPoleFoundOnLeft)
        {
            locationDataThirdCourse_ = {StartingPlaceThirdCourse ::THREE, PoleLocationThirdCourse ::FIVE};
        }
    }
}

void CrossCourseMode::boost(RotationDirection direction)
{
    if (direction == RotationDirection::RIGHT)
    {
        robot_->rotateRight(250);
    }
    else
    {
        robot_->rotateLeft(250);
    }
    _delay_ms(60);
}

bool CrossCourseMode::isPoleDetectedRotatingFromIntersection(RotationDirection direction,
                                                             SensorType sensorType, uint8_t speed)
{
    bool isPoleDetected = false;

    boost(direction);
    while (rotateUntil(sensorType, speed, direction))
    {
        if (!isPoleDetected)
        {
            isPoleDetected = infraredSensor_.isPoleDetectedFromIntersection();
        }
    }

    return isPoleDetected;
}

bool CrossCourseMode::processRotationAndDetectPole(const void *steps, size_t stepsSize, RotationStepType type)
{
    bool isPoleDetected = false;
    bool isProcessComplete = false;

    for (uint8_t currentStepIndex = 0; currentStepIndex < stepsSize; ++currentStepIndex)
    {
        isPoleDetected = processRotationStep(steps, stepsSize, type, isPoleDetected, currentStepIndex);

        stopAndStabilize();

        if (isPoleDetected)
        {
            uint8_t stepIndex = currentStepIndex + 1;
            isProcessComplete = processPoleDetectionReset(steps, stepsSize, type, stepIndex);
        }

        if (isProcessComplete)
        {
            break;
        }
    }

    return isPoleDetected;
}

bool CrossCourseMode::processRotationStep(const void *steps, size_t stepsSize, RotationStepType type, bool &isPoleDetected, uint8_t &currentStepIndex)
{
    switch (type)
    {
    case RotationStepType::Left:
        return processLeftRotationStep(steps, stepsSize, currentStepIndex, isPoleDetected);
    case RotationStepType::Right:
        return processRightRotationStep(steps, stepsSize, currentStepIndex, isPoleDetected);
    case RotationStepType::Middle:
        return processMiddleRotationStep(steps, stepsSize, currentStepIndex, isPoleDetected);
    default:
        return false;
    }
}

bool CrossCourseMode::processLeftRotationStep(const void *steps, size_t stepsSize, uint8_t &currentStepIndex, bool &isPoleDetected)
{
    const auto &currentStep = reinterpret_cast<const RotationStepLeft *>(steps)[currentStepIndex];

    if (currentStep.stage == RotationStageLeft::DETECTED_IN_LEFT_ADJUSTMENT ||
        currentStep.stage == RotationStageLeft::DETECTED_IN_RIGHT_ROTATION)
    {
        boost(currentStep.direction);
        while (rotateUntil(currentStep.sensorType, navigation_control::ROTATION_SPEED, currentStep.direction))
            ;
        stopAndStabilize();
    }
    else
    {
        isPoleDetected = isPoleDetectedRotatingFromIntersection(
            currentStep.direction,
            currentStep.sensorType,
            navigation_control::ROTATION_SPEED);
    }

    return isPoleDetected;
}

bool CrossCourseMode::processRightRotationStep(const void *steps, size_t stepsSize, uint8_t &currentStepIndex, bool &isPoleDetected)
{
    const auto &currentStep = reinterpret_cast<const RotationStepRight *>(steps)[currentStepIndex];

    if (currentStep.stage == RotationStageRight::DETECTED_IN_RIGHT_ADJUSTMENT ||
        currentStep.stage == RotationStageRight::DETECTED_IN_LEFT_ROTATION)
    {
        boost(currentStep.direction);
        while (rotateUntil(currentStep.sensorType, navigation_control::ROTATION_SPEED, currentStep.direction))
            ;
        stopAndStabilize();
    }
    else
    {
        isPoleDetected = isPoleDetectedRotatingFromIntersection(
            currentStep.direction,
            currentStep.sensorType,
            navigation_control::ROTATION_SPEED);
    }

    return isPoleDetected;
}

bool CrossCourseMode::processMiddleRotationStep(const void *steps, size_t stepsSize, uint8_t &currentStepIndex, bool &isPoleDetected)
{
    const auto &currentStep = reinterpret_cast<const RotationStepMiddle *>(steps)[currentStepIndex];
    isPoleDetected = isPoleDetectedRotatingFromIntersection(
        currentStep.direction,
        currentStep.sensorType,
        navigation_control::ROTATION_SPEED);

    return isPoleDetected;
}

bool CrossCourseMode::processPoleDetectionReset(const void *steps, size_t stepsSize, RotationStepType type, uint8_t &stepIndex)
{
    bool resetFlag = false;

    while (!resetFlag)
    {
        switch (type)
        {
        case RotationStepType::Left:
            resetFlag = processLeftRotationReset(steps, stepsSize, stepIndex);
            break;
        case RotationStepType::Right:
            resetFlag = processRightRotationReset(steps, stepsSize, stepIndex);
            break;
        case RotationStepType::Middle:
            resetFlag = processMiddleRotationReset(steps, stepsSize, stepIndex);
            break;
        default:
            break;
        }

        stepIndex++;
        if (stepIndex >= stepsSize)
        {
            stepIndex = 0;
        }
    }

    return resetFlag;
}

bool CrossCourseMode::processLeftRotationReset(const void *steps, size_t stepsSize, uint8_t &stepIndex)
{
    const auto &step = reinterpret_cast<const RotationStepLeft *>(steps)[stepIndex];
    boost(step.direction);
    while (rotateUntil(step.sensorType, navigation_control::ROTATION_SPEED, step.direction))
        ;
    stopAndStabilize();

    return step.stage == RotationStageLeft::DETECTED_IN_MIDDLE_RETURN;
}

bool CrossCourseMode::processRightRotationReset(const void *steps, size_t stepsSize, uint8_t &stepIndex)
{
    const auto &step = reinterpret_cast<const RotationStepRight *>(steps)[stepIndex];
    boost(step.direction);
    while (rotateUntil(step.sensorType, navigation_control::ROTATION_SPEED, step.direction))
        ;
    stopAndStabilize();

    return step.stage == RotationStageRight::DETECTED_IN_MIDDLE_RETURN;
}

bool CrossCourseMode::processMiddleRotationReset(const void *steps, size_t stepsSize, uint8_t &stepIndex)
{
    const auto &step = reinterpret_cast<const RotationStepMiddle *>(steps)[stepIndex];
    boost(step.direction);
    while (rotateUntil(step.sensorType, navigation_control::ROTATION_SPEED, step.direction))
        ;
    stopAndStabilize();

    return step.stage == RotationStageMiddle::DETECTED_IN_MIDDLE_SLIGHT_LEFT ||
           step.stage == RotationStageMiddle::DETECTED_IN_MIDDLE_SLIGHT_RIGHT;
}

bool CrossCourseMode::processRotationAndDetectPoleLeft()
{
    return processRotationAndDetectPole(stepsLeft_, stepsLeftSize_, RotationStepType::Left);
}

bool CrossCourseMode::processRotationAndDetectPoleRight()
{
    return processRotationAndDetectPole(stepsRight_, stepsRightSize_, RotationStepType::Right);
}

bool CrossCourseMode::processRotationAndDetectPoleMiddle()
{
    return processRotationAndDetectPole(stepsMid_, stepsMidSize_, RotationStepType::Middle);
}

bool CrossCourseMode::isPoleDetectedWhileChecking(SensorType triggeredSensor)
{
    CourseStatus &currentCourseStatus = courseStatuses_[static_cast<int>(currentCourse_)];

    if (processRotationAndDetectPoleMiddle())
    {
        currentCourseStatus.isPoleDetectedOppositeDirection = true;
        return true;
    }

    if (isFirstIteration_)
    {
        if (triggeredSensor == SensorType::FAR_LEFT)
        {
            bool isPoleFound = processRotationAndDetectPoleLeft();
            currentCourseStatus.isPoleFoundOnLeft = true;
            isFirstIteration_ = false;
            return isPoleFound;
        }

        if (triggeredSensor == SensorType::FAR_RIGHT)
        {
            bool isPoleFound = processRotationAndDetectPoleRight();
            currentCourseStatus.isPoleFoundOnRight = true;
            isFirstIteration_ = false;
            return isPoleFound;
        }
    }

    if (processRotationAndDetectPoleLeft())
    {
        currentCourseStatus.isPoleFoundOnLeft = true;
        return true;
    }

    if (processRotationAndDetectPoleRight())
    {
        currentCourseStatus.isPoleFoundOnRight = true;
        return true;
    }

    return false;
}

void CrossCourseMode::writeInferredDataToMemory()
{
    auto writeLocationToMemory = [this](uint8_t value, uint8_t address)
    {
        if (value != static_cast<uint8_t>(0))
        {
            robot_->writeMemory(address, value);
            _delay_ms(memory::MEMORY_DELAY_MS);
        }
    };

    writeLocationToMemory(static_cast<uint8_t>(locationDataFirstCourse_.startingPosition), configRapport::DEPARTURE_NODE_FIRST_COURSE_ADDRESS);
    writeLocationToMemory(static_cast<uint8_t>(locationDataFirstCourse_.poleLocation), configRapport::POLE_LOCATION_FIRST_COURSE_ADDRESS);
    writeLocationToMemory(static_cast<uint8_t>(locationDataSecondCourse_.poleLocation), configRapport::POLE_LOCATION_SECOND_COURSE_ADDRESS);
    writeLocationToMemory(static_cast<uint8_t>(locationDataThirdCourse_.poleLocation), configRapport::POLE_LOCATION_THIRD_COURSE_ADDRESS);
}

void CrossCourseMode::handlePostIntersectionAdjustment(SensorType triggeredSensor)
{
    if (triggeredSensor == SensorType::FAR_LEFT)
    {   
        robot_->moveForward(100);
        _delay_ms(400);
        followLineSmartForDuration(navigation_control::POST_INTERSECTION_FAR_RIGHT_DELAY_MS);
        

    }
    else if (triggeredSensor == SensorType::FAR_RIGHT)
    {   
         robot_->moveForward(100);
        _delay_ms(400);
        followLineSmartForDuration(navigation_control::POST_INTERSECTION_FAR_RIGHT_DELAY_MS);      
    }
    stopAndStabilize();
}

void CrossCourseMode::waitForPoleDetection(SensorType triggeredSensor)
{
    isFirstIteration_ = true;
    bool isPoleDetected = false;
    while (!isPoleDetected)
    {
        isPoleDetected = isPoleDetectedWhileChecking(triggeredSensor);
    }
    stopAndStabilize();
}

ModeStrategy::IntersectionType CrossCourseMode::ajustVerifieRight()
{
    const uint8_t turningSpeed = 70;
    robot_->rotateRight(250);
    _delay_ms(50);
    robot_->rotateRight(turningSpeed);

    while (!robot_->isSensorActive(LineSensor::MID))
    {
    };

    robot_->haltMotor();
    _delay_ms(500);

    if (robot_->isSensorActive(LineSensor::FAR_RIGHT))
    {
        robot_->moveForward(250);
        _delay_ms(50);
        robot_->moveForward(navigation::forwardSpeed);

        while (robot_->isSensorActive(LineSensor::FAR_RIGHT) && !robot_->isSensorActive(LineSensor::FAR_LEFT))
        {
        }

        if (robot_->isSensorActive(LineSensor::FAR_LEFT))
        {
            return IntersectionType::FULL;
        }
        else if (!robot_->isSensorActive(LineSensor::FAR_LEFT))
        {
            return IntersectionType::PARTIAL;
        }
    }
    return IntersectionType::LINE;
}

ModeStrategy::IntersectionType CrossCourseMode::ajustVerifieRightIntersection()
{

    constexpr uint8_t turningSpeed = 70;
    robot_->rotateRight(turningSpeed);

    while (!robot_->isSensorActive(LineSensor::MID))
    {
    };

    pause();

    if (robot_->isSensorActive(LineSensor::FAR_RIGHT))
    {
        return IntersectionType::INTERSECTION;
    }
    return IntersectionType::LINE;
}



ModeStrategy::IntersectionType CrossCourseMode::ajustVerifieLeftIntersection()
{

    const uint8_t turningSpeed = 70;
    robot_->rotateLeft(turningSpeed);

    while (!robot_->isSensorActive(LineSensor::MID))
    {
    };

    pause();

    if (robot_->isSensorActive(LineSensor::FAR_LEFT))
    {
        return IntersectionType::INTERSECTION;
    }
    return IntersectionType::LINE;
}

ModeStrategy::IntersectionType CrossCourseMode::ajustFirstCourse(){
    constexpr uint8_t turningSpeed = 70;

    boost(RotationDirection::RIGHT);
    robot_->rotateRight(turningSpeed);

    while (!robot_->isSensorActive(LineSensor::MID));

    pause();

    robot_->moveForward(100);
    _delay_ms(200);
    stopAndStabilize();


    if (robot_->noSensorsActive()){
        return IntersectionType::FULL;
    } else {
        return IntersectionType::PARTIAL;
    }
}


ModeStrategy::IntersectionType CrossCourseMode::ajustSecondCourse(Direction direction) {
    constexpr uint8_t turningSpeed = 70;

    if (direction == Direction::RIGHT) {
        robot_->rotateRight(turningSpeed);
    } else if (direction == Direction::LEFT) {
        robot_->rotateLeft(turningSpeed);
    }

    while (!robot_->isSensorActive(LineSensor::MID));

    pause();

 
    if ((direction == Direction::RIGHT && robot_->isSensorActive(LineSensor::FAR_LEFT)) ||
        (direction == Direction::LEFT && robot_->isSensorActive(LineSensor::FAR_RIGHT)))
    {
        return IntersectionType::INTERSECTION;
    }

    return IntersectionType::LINE;
}


ModeStrategy::IntersectionType CrossCourseMode::ajustThirdCourse(){
    constexpr uint8_t turningSpeed = 70;

    boost(RotationDirection::LEFT);
    robot_->rotateLeft(turningSpeed);

    while (!robot_->isSensorActive(LineSensor::MID));

    pause();

    robot_->moveForward(100);
    _delay_ms(200);
    stopAndStabilize();

    if (robot_->noSensorsActive()){
        return IntersectionType::FULL;
    } else {
        return IntersectionType::PARTIAL;
    }
}

ModeStrategy::IntersectionType CrossCourseMode::ajustVerifieLeft()
{
    const uint8_t turningSpeed = 70;
    robot_->rotateLeft(250);
    _delay_ms(50);
    robot_->rotateLeft(turningSpeed);

    while (!robot_->isSensorActive(LineSensor::MID))
    {
    };

    robot_->haltMotor();
    _delay_ms(500);

    if (robot_->isSensorActive(LineSensor::FAR_LEFT))
    {
        robot_->moveForward(250);
        _delay_ms(50);
        robot_->moveForward(navigation::forwardSpeed);

        robot_->controlRightMotorSpeed(navigation::forwardSpeed + 7);

        while (robot_->isSensorActive(LineSensor::FAR_LEFT) && !robot_->isSensorActive(LineSensor::FAR_RIGHT))
        {
        }

        if (robot_->isSensorActive(LineSensor::FAR_RIGHT))
        {
            return IntersectionType::FULL;
        }
        else
        {
            return IntersectionType::PARTIAL;
        }
    }
    return IntersectionType::LINE;
}

void CrossCourseMode::fullIntersection()
{   

    incrementIntersections();
    
    prepareForIntersection();

    if (currentCourse_ == CurrentCourse::FIRST)
    {
        handleIntersectionLeft();
        return;
    }

    if (currentCourse_ == CurrentCourse::SECOND)
    {
        if (firstCourse_.partialIntersection == true)
        { // 1
            if (courseStatuses_[static_cast<int>(CurrentCourse::SECOND)].isPoleFoundOnLeft == true)
            { // 3
                handleIntersectionRight();
                return;
            }
            if (courseStatuses_[static_cast<int>(CurrentCourse::SECOND)].isPoleDetectedOppositeDirection == true)
            { // 4
                handleIntersectionLeft();
                return;
            }
        }
        if (firstCourse_.fullIntersection == true)
        { // 2
            if (courseStatuses_[static_cast<int>(CurrentCourse::SECOND)].isPoleDetectedOppositeDirection == true)
            { // 3
                handleIntersectionRight();
                return;
            }
            if (courseStatuses_[static_cast<int>(CurrentCourse::SECOND)].isPoleFoundOnRight == true)
            { // 4
                handleIntersectionLeft();
                return;
            }
        }
    }

    if (currentCourse_ == CurrentCourse::THIRD)
    {
        handleIntersectionRight();
        return;
    }
}

void CrossCourseMode::prepareForIntersection()
{
    robot_->moveForward(navigation::forwardSpeed);
    _delay_ms(1500);
    stopAndStabilize();
}

void CrossCourseMode::handleIntersectionRight()
{
    rotateRightToDetectLine();
    markFullIntersection(Direction::RIGHT);
}

void CrossCourseMode::handleIntersectionLeft()
{
    rotateLeftToDetectLine();
    markFullIntersection(Direction::LEFT);
}

void CrossCourseMode::finalizeIntersection()
{
    stopAndStabilize();
    robot_->driveBackward(87);
    while (robot_->noSensorsActive())
        ;
    stopAndStabilize();

    processCourseData();
    writeInferredDataToMemory();
    playSoundModeFin();
    while (true)
        ;
}

void CrossCourseMode::rotateRightToDetectLine()
{
    boost(RotationDirection::RIGHT);
    while(rotateUntil(SensorType::FAR_RIGHT, navigation_control::ROTATION_SPEED, RotationDirection::RIGHT));
    stopAndStabilize();
}

void CrossCourseMode::rotateLeftToDetectLine()
{  
    boost(RotationDirection::LEFT);
    while(rotateUntil(SensorType::FAR_LEFT, navigation_control::ROTATION_SPEED, RotationDirection::LEFT));
    stopAndStabilize();
}

void CrossCourseMode::markFullIntersection(Direction direction)
{
    if (currentCourse_ == CurrentCourse::FIRST)
    {
        firstCourse_.fullIntersection = true;
    }
    else if (currentCourse_ == CurrentCourse::SECOND)
    {
        secondCourse_.fullIntersection = true;
        secondCourse_.direction = direction;
    }
    else if (currentCourse_ == CurrentCourse::THIRD)
    {
        thirdCourse_.fullIntersection = true;
    }
}


void CrossCourseMode::processCourseData()
{
    processInferredDataFirstCourse();
    processInferredDataSecondCourse();
    processInferredDataThirdCourse();
    writeInferredDataToMemory();
}

void CrossCourseMode::intersectionX(SensorType triggeredSensor)
{

    incrementIntersections();

    if(currentCourse_ == CurrentCourse::THIRD){
        handleFinalCourse();
    }

    if(currentCourse_ == CurrentCourse::FINAL){
        handleFinalCourse();
    }

    updateNextState();

   
    handlePostIntersectionAdjustment(triggeredSensor);

    stopAndStabilize();

    waitForPoleDetection(triggeredSensor); 
    stopAndStabilize();

    while (followLineUntilDetectPole());
    stopAndStabilize();
    stopAndStabilize();

    playSoundCourseFin();

    while(!waitForRelease());
    _delay_ms(configTiming::INITIALIZATION_DELAY_MS);
}

void CrossCourseMode::partialIntersection(bool toRight)
{   

    incrementIntersections();
    _delay_ms(500);

    robot_->moveForward(navigation::forwardSpeed);

    if (toRight)
    {
        robot_->controlLeftMotorSpeed(navigation::forwardSpeed + 45);
    }
    else
    {
        robot_->controlRightMotorSpeed(navigation::forwardSpeed + 60);
    }

    while (robot_->isSensorActive(SensorType::FAR_LEFT) || robot_->isSensorActive(SensorType::FAR_RIGHT))
    {
    };

    pause();

    while (!robot_->noSensorsActive())
    {
        followLine();
        if (robot_->isSensorActive(SensorType::FAR_RIGHT))
        {
            while (robot_->isSensorActive(SensorType::FAR_RIGHT))
            {
                robot_->controlLeftMotorSpeed(navigation::forwardSpeed + 40);
            }
        }
        else if (robot_->isSensorActive(SensorType::FAR_LEFT))
        {
            while (robot_->isSensorActive(SensorType::FAR_LEFT))
            {
                robot_->controlRightMotorSpeed(navigation::forwardSpeed + 40);
            }
        }
    }

    pause();

    if (toRight)
    {
        if (currentCourse_ == CurrentCourse::FIRST)
        {
            firstCourse_.partialIntersection = true;
        }
        else if (currentCourse_ == CurrentCourse::THIRD)
        {
            thirdCourse_.partialIntersection = true;
        }
        do
        {
            robot_->rotateRight(navigation::speed);
        } while (!robot_->isSensorActive(SensorType::MID));
        robot_->haltMotor();
    }
    else
    {
        if (currentCourse_ == CurrentCourse::FIRST)
        {
            firstCourse_.partialIntersection = true;
        }
        else if (currentCourse_ == CurrentCourse::THIRD)
        {
            thirdCourse_.partialIntersection = true;
        }
        do
        {
            robot_->rotateLeft(navigation::speed);
        } while (!robot_->isSensorActive(SensorType::MID));
        robot_->haltMotor();
    }
}

void CrossCourseMode::navigate() {
    while (true) {  
        if (isPair()) {
            handlePairMode();
        }
         else {
            switch (currentCourse_) {
                robot_->turnLedRed();
                case CurrentCourse::FIRST:
                    handleFirstCourse();
                    break;
                case CurrentCourse::SECOND:
                    handleSecondCourse();
                    break;
                case CurrentCourse::THIRD:
                    handleThirdCourse();
                    break;
                case CurrentCourse::FINAL:
                    handleFinalCourse();
                    break;
                default:
                    followLine();
                    break;
            }
        }
    }
}

void CrossCourseMode::handlePairMode() {
    if (robot_->isSensorActive(SensorType::FAR_RIGHT)) {
        pause();

        IntersectionType intersectionType = ajustVerifieRightIntersection();
        if (intersectionType == IntersectionType::INTERSECTION) {
            intersectionX(SensorType::FAR_RIGHT);
            return;
        }
    } else if (robot_->isSensorActive(SensorType::FAR_LEFT)) {
        pause();

        IntersectionType intersectionType = ajustVerifieLeftIntersection();
        if (intersectionType == IntersectionType::INTERSECTION) {
            intersectionX(SensorType::FAR_LEFT);
            return;
        }
    } else if (robot_->noSensorsActive()) {
        handleLostLine();
    } else {
        followLine();
    }
}

void CrossCourseMode::handleFirstCourse() {
    if (robot_->isSensorActive(SensorType::FAR_RIGHT)) {
        pause();

        IntersectionType intersectionType = ajustVerifieRight();
        if (intersectionType == IntersectionType::PARTIAL) {
            partialIntersection(true); 
        }
    } else if (robot_->isSensorActive(SensorType::FAR_LEFT)) {
        pause();

        IntersectionType intersectionType = ajustVerifieLeft(); 
        if (intersectionType == IntersectionType::FULL) {
            fullIntersection();
        }
    } else if (robot_->noSensorsActive()) {
        handleLostLine();

        
    } else {
        followLine();
    }
}

void CrossCourseMode::handleSecondCourse() {
    if (robot_->isSensorActive(SensorType::FAR_RIGHT)) {
        pause();

        IntersectionType intersectionType = ajustVerifieRight(); 
        if (intersectionType == IntersectionType::FULL) {
            fullIntersection();
        }
    } else if (robot_->isSensorActive(SensorType::FAR_LEFT)) {
        pause();
    
        IntersectionType intersectionType = ajustVerifieLeft(); 
        if (intersectionType == IntersectionType::FULL) {
            fullIntersection();
        }
    } else if (robot_->noSensorsActive()) {
        handleLostLine();
    } else {
        followLine();
    }
}

void CrossCourseMode::pause() {
    robot_->haltMotor();
    _delay_ms(500);
}

void CrossCourseMode::handleThirdCourse() {
    if (robot_->isSensorActive(SensorType::FAR_RIGHT)) {
        pause();

        IntersectionType intersectionType = ajustVerifieRight(); 
        if (intersectionType == IntersectionType::FULL) {
            fullIntersection();
        }
    } else if (robot_->isSensorActive(SensorType::FAR_LEFT)) {
        pause();

        IntersectionType intersectionType = ajustVerifieLeft();
        if (intersectionType == IntersectionType::PARTIAL) {
            partialIntersection(false); 
        }
    } else if (robot_->noSensorsActive()) {
        handleLostLine();
    } else {
        followLine();
    }
}

void CrossCourseMode::handleFinalCourse() {
    if (thirdCourse_.fullIntersection == true) {
        while (true) {
            if (robot_->isSensorActive(SensorType::FAR_RIGHT)) {
                IntersectionType inter = ajustVerifieRightIntersection();
                if(inter == IntersectionType::INTERSECTION){
                    robot_->moveForward(100);
                    _delay_ms(350);
                }
                
            } 
            else if (robot_->isSensorActive(SensorType::FAR_LEFT)) {
                IntersectionType inter = ajustVerifieLeftIntersection();
                if(inter == IntersectionType::INTERSECTION){
                    robot_->moveForward(100);
                    _delay_ms(350);
                }
            } 
            else if (robot_->noSensorsActive()) {
                finalizeIntersection();
            } 
            else {
                followLine();
            }
        }
    }

    if (thirdCourse_.partialIntersection == true) {
        while (true) {
            if (robot_->isSensorActive(SensorType::FAR_RIGHT)) {
                IntersectionType inter = ajustVerifieRightIntersection();
                if(inter == IntersectionType::INTERSECTION){
                    robot_->moveForward(100);
                    _delay_ms(350);
                }
            } else if (robot_->isSensorActive(SensorType::FAR_LEFT)) {
                IntersectionType inter = ajustVerifieLeftIntersection();
                if(inter == IntersectionType::INTERSECTION){
                    robot_->moveForward(100);
                    _delay_ms(350);
                }
            } else if (robot_->noSensorsActive()) {
                if (pastFinalFalseIntersection_ == true) {
                    finalizeIntersection();
                }
                handleLostLine();
            } else {
                followLine();
            }
        }
    }
}

void CrossCourseMode::handleLostLine() {
     pastFinalFalseIntersection_ = true;
    robot_->haltMotor();
    robot_->rotateRight(250);
    _delay_ms(50);
    robot_->rotateRight(navigation::rightMotorSpeed);
    while (robot_->noSensorsActive());
}


bool CrossCourseMode::isPressingButton() const {
        if (isPressed()) { 
            _delay_ms(10);
            return isPressed();
        }
        return false;
    }

bool CrossCourseMode::waitForRelease() const {
    if (isPressingButton()){
        while (true){
            if(isPressingButton() == false){
                return true;
            }
        }
    }
    return false;
}