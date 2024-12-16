#pragma once
#include "mode_strategy.h"


enum class LastDirection {
    LEFT,
    RIGHT,
    UNDEFINED,
};

class SearchEndMode : public ModeStrategy {
public:
    enum class RobotState {
        A_TO_D = 1,         // Transition from A to D
        A_TO_C = 2,         // Transition from A to C
        A_TO_E_VIA_3 = 3,   // Transition from A to E via path 3
        A_TO_E_VIA_4 = 4,   // Transition from A to E via path 4
        B_TO_C = 5,         // Transition from B to C
        B_TO_D = 6,         // Transition from B to D
        B_TO_E_VIA_5 = 7,   // Transition from B to E via path 5
        B_TO_E_VIA_6 = 8    // Transition from B to E via path 6
    };
    
public:
    explicit SearchEndMode(Robot* robot) : ModeStrategy(robot) {}
    void execute() override;
    
private:
    void clignotement();
    void writeCoordInMemory();
    void turnLedBaseOnExtremity();

    IntersectionType ajustVerifieRight();
    IntersectionType ajustVerifieLeft();
    void partialIntersection(bool toRight);
    void fullIntersection();
    void navigate();
    void processInferredData();

    inline void incrementIntersections() { ++nbsOfintersections_; }


    void handleFirst();
    void handleSecond();
    void handleThird();

    void rotateAndCheckSensors(bool rotateRight, uint16_t timeoutDurationMs);
    bool followLineForTimeout(uint16_t timeoutDurationMs);

private:
    bool intersectionCrossed = false;

    bool fullIntersectionLeft_ = false;
    bool partialIntersectionRight_ = false;
    bool partialIntersectionLeft_ = false;
    bool fullIntersectionRight_ = false;

    uint8_t nbsOfintersections_ = 0;
    LastDirection lastDirection_ = LastDirection::UNDEFINED;

protected:
    RobotState currentState_;
};