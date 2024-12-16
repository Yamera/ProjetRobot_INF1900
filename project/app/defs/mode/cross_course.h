#pragma once

// Team: 6170
// Mohamed Borhaneddine Mokaddem 2198125
// Sami Ahmed Benabbou 2122457
// Maroua Lassakeur 2267763
// Yasmine Meraoubi 2292276
//
// CrossCourseMode Class
// Description: Navigation mode for crossing intersections on a course,
//               handling necessary adjustments, and detecting poles during rotation.
//               Uses infrared sensors, motors, and sound for guiding the robot.


#include "mode_strategy.h"
#include "sensor/infrared_sensor.h"

#include <stddef.h>

namespace navigation_control
{
    static constexpr uint16_t POST_INTERSECTION_FAR_RIGHT_DELAY_MS = 2400;
    static constexpr uint16_t POST_INTERSECTION_FAR_LEFT_DELAY_MS = 2100;
    static constexpr uint8_t FORWARD_SPEED = 80;
    static constexpr uint8_t ROTATION_SPEED = 80;
}

namespace navigation_control
{
    static constexpr uint16_t STABILIZATION_DELAY_MS = 1000;
}

namespace configTiming
{
    static constexpr uint16_t INITIALIZATION_DELAY_MS = 2000;
}

namespace memory
{
    static constexpr uint16_t MEMORY_DELAY_MS = 100;
}

namespace configRapport
{
    constexpr uint16_t DEPARTURE_NODE_FIRST_COURSE_ADDRESS = 0x0004;
    constexpr uint16_t POLE_LOCATION_FIRST_COURSE_ADDRESS = 0x0005;
    constexpr uint16_t POLE_LOCATION_SECOND_COURSE_ADDRESS = 0x0006;
    constexpr uint16_t POLE_LOCATION_THIRD_COURSE_ADDRESS = 0x0007;
}

namespace configSoundCourse
{
    constexpr uint16_t SOUND_NOTE = 81;
    constexpr uint16_t SOUND_DURATION = 150;
    constexpr uint16_t SOUND_OFF_DELAY = 50;
}

namespace configSoundMode
{
    constexpr uint16_t SOUND_NOTE = 81;
    constexpr uint16_t SOUND_DURATION = 750;
    constexpr uint16_t SOUND_OFF_DELAY = 50;
}

enum class Direction
{
    LEFT,
    RIGHT,
    INVALID,
};

struct CourseIntersectionStatus
{
    bool partialIntersection = false;
    bool fullIntersection = false;
    Direction direction = Direction::INVALID;

    CourseIntersectionStatus() = default;
};

struct CourseStatus
{
    bool isPoleFoundOnLeft = false;
    bool isPoleFoundOnRight = false;
    bool isPoleDetectedOppositeDirection = false;

    CourseStatus() = default;
};

enum class RotationDirection
{
    LEFT,
    RIGHT
};

enum class CurrentCourse
{
    INIT = -1,
    FIRST = 0,
    SECOND = 1,
    THIRD = 2,
    FINAL = 3,
};

enum class StartingPlaceFirstCourse
{
    C = 'C',
    D = 'D',
    UNKNOWN = 0
};

enum class PoleLocationFirstCourse : char
{
    ONE = '1',
    TWO = '2',
    UNKNOWN = 0,
};

enum class StartingPlaceSecondCourse : char
{
    ONE = '1',
    TWO = '2',
    UNKNOWN = 0,
};

enum class PoleLocationSecondCourse : char
{
    THREE = '3',
    FOUR = '4',
    UNKNOWN = 0,
};

enum class StartingPlaceThirdCourse : char
{
    THREE = '3',
    FOUR = '4',
    UNKNOWN = 0,
};

enum class PoleLocationThirdCourse : char
{
    FIVE = '5',
    SIX = '6',
    UNKNOWN = 0,
};

enum class RotationStageLeft
{
    DETECTED_IN_LEFT_ADJUSTMENT,
    DETECTED_IN_LEFT_ROTATION,
    DETECTED_IN_LEFT_EMPTY_SPACE,
    DETECTED_IN_MIDDLE_RETURN,
    DETECTED_IN_RIGHT_EMPTY_SPACE,
    DETECTED_IN_RIGHT_ROTATION,
    NOT_DETECTED
};

enum class RotationStageRight
{
    DETECTED_IN_RIGHT_ADJUSTMENT,
    DETECTED_IN_RIGHT_ROTATION,
    DETECTED_IN_RIGHT_EMPTY_SPACE,
    DETECTED_IN_MIDDLE_RETURN,
    DETECTED_IN_LEFT_EMPTY_SPACE,
    DETECTED_IN_LEFT_ROTATION,
    NOT_DETECTED
};

enum class RotationStageMiddle
{
    DETECTED_IN_LEFT_EMPTY_SPACE,
    DETECTED_IN_MIDDLE_SLIGHT_LEFT,
    DETECTED_IN_MIDDLE_SLIGHT_RIGHT,
    DETECTED_IN_RIGHT_EMPTY_SPACE
};

enum class RotationStepType
{
    Left,
    Right,
    Middle
};

template <typename StartingPlaceEnum, typename PoleLocationEnum>
struct InferredData
{
    StartingPlaceEnum startingPosition = StartingPlaceEnum::UNKNOWN;
    PoleLocationEnum poleLocation = PoleLocationEnum::UNKNOWN;

    InferredData() = default;
};

using InferredDataFirstCourse = InferredData<StartingPlaceFirstCourse, PoleLocationFirstCourse>;
using InferredDataSecondCourse = InferredData<StartingPlaceSecondCourse, PoleLocationSecondCourse>;
using InferredDataThirdCourse = InferredData<StartingPlaceThirdCourse, PoleLocationThirdCourse>;

struct RotationStepMiddle
{
    RotationDirection direction;
    SensorType sensorType;
    RotationStageMiddle stage;
};

struct RotationStepLeft
{
    RotationDirection direction;
    SensorType sensorType;
    RotationStageLeft stage;
};

struct RotationStepRight
{
    RotationDirection direction;
    SensorType sensorType;
    RotationStageRight stage;
};

class CrossCourseMode : public ModeStrategy
{
public:
    explicit CrossCourseMode(Robot *robot);
    void execute() override;

private:
    void updateNextState();

    void navigate();
    IntersectionType ajustVerifieRight();
    IntersectionType ajustVerifieLeft();
    void fullIntersection();
    void intersectionX(SensorType triggeredSensor);
    void partialIntersection(bool toRight);
    void prepareForIntersection();
    void handleIntersectionRight();
    void handleIntersectionLeft();
    void finalizeIntersection();
    void rotateRightToDetectLine();
    void rotateLeftToDetectLine();
    void markFullIntersection(Direction direction);
    void processCourseData();

    void followLineSmart(uint8_t &nbsStops);

    void boost(RotationDirection direction);

    void clignotement();
    inline void internalButtonCallbackFunction();
    static void internalButtonCallbackStatic();
    void waitForButtonPress();

    void initialize();
    void stopAndStabilize();
    void playSoundCourseFin();
    void playSoundModeFin();

    void processInferredDataFirstCourse();
    void processInferredDataSecondCourse();
    void processInferredDataThirdCourse();

    void followLineDefault();
    void followLineSmartForDuration(uint16_t durationMs);

    bool followLineUntilIntersection();
    SensorType detectIntersectionForPriority();
    SensorType detectIntersection();

    bool followLineUntilDetectPole();
    bool rotateUntil(SensorType sensorType, uint8_t speed, RotationDirection direction);

    bool isPoleDetectedRotatingFromIntersection(RotationDirection direction, SensorType sensorType, uint8_t speed);
    bool isPoleDetectedWhileChecking(SensorType triggeredSensor);
    void waitForPoleDetection(SensorType triggeredSensor);

    bool processRotationAndDetectPole(const void *steps, size_t stepsSize, RotationStepType type);
    bool processRotationAndDetectPoleLeft();
    bool processRotationAndDetectPoleRight();
    bool processRotationAndDetectPoleMiddle();

    void writeInferredDataToMemory();

    void handlePostIntersectionAdjustment(SensorType triggeredSensor);

    bool processRotationStep(const void *steps, size_t stepsSize, RotationStepType type, bool &isPoleDetected, uint8_t &currentStepIndex);
    bool processLeftRotationStep(const void *steps, size_t stepsSize, uint8_t &currentStepIndex, bool &isPoleDetected);
    bool processRightRotationStep(const void *steps, size_t stepsSize, uint8_t &currentStepIndex, bool &isPoleDetected);
    bool processMiddleRotationStep(const void *steps, size_t stepsSize, uint8_t &currentStepIndex, bool &isPoleDetected);
    bool processPoleDetectionReset(const void *steps, size_t stepsSize, RotationStepType type, uint8_t &stepIndex);
    bool processLeftRotationReset(const void *steps, size_t stepsSize, uint8_t &stepIndex);
    bool processRightRotationReset(const void *steps, size_t stepsSize, uint8_t &stepIndex);
    bool processMiddleRotationReset(const void *steps, size_t stepsSize, uint8_t &stepIndex);

    inline bool isPair() const { return (nbsOfintersections_ % 2 == 0); }
    inline void incrementIntersections() { ++nbsOfintersections_; }

    IntersectionType ajustVerifieRightIntersection();
    IntersectionType ajustVerifieLeftIntersection();

    void handlePairMode();
    void handleLostLine();
    void handleFirstCourse();
    void handleSecondCourse();
    void handleThirdCourse();
    void handleFinalCourse();

    IntersectionType ajustFirstCourse();
    IntersectionType ajustSecondCourse(Direction direction);
    IntersectionType ajustThirdCourse();

    void pause();
    
    bool isPressed() const;
    bool isPressingButton() const;
    bool waitForRelease() const;

private:
    static CrossCourseMode *instance_;
    volatile bool isButtonInternPressed_ = false;

private:
    static constexpr size_t stepsMidSize_ = 4;
    static constexpr size_t stepsLeftSize_ = 6;
    static constexpr size_t stepsRightSize_ = 6;

    static RotationStepMiddle stepsMid_[];
    static RotationStepLeft stepsLeft_[];
    static RotationStepRight stepsRight_[];

private:
    InfraredSensor &infraredSensor_ = InfraredSensor::getInstance();

    InferredDataFirstCourse locationDataFirstCourse_;
    InferredDataSecondCourse locationDataSecondCourse_;
    InferredDataThirdCourse locationDataThirdCourse_;

    CurrentCourse currentCourse_;

    CourseStatus courseStatuses_[3];

    CourseIntersectionStatus firstCourse_;
    CourseIntersectionStatus secondCourse_;
    CourseIntersectionStatus thirdCourse_;

    uint8_t nbsOfintersections_ = 0;

    bool pastFinalFalseIntersection_ = false;
    bool isFirstIteration_ = true;
};