#pragma once

#include "../robot.h"

namespace configClignotement {
    constexpr uint8_t blinkFrequency = 4;  
    constexpr uint16_t durationMs = 2000;   
    constexpr uint8_t blinkIntervalMs = 1000 / (blinkFrequency * 2);  
    constexpr uint8_t totalBlinks = (durationMs / 1000) * blinkFrequency; 
}

namespace configRapport {
    constexpr uint16_t startAddress = 0x0001;
    constexpr uint8_t dataSize = 7;
    constexpr uint16_t reportSize = 340;
    constexpr uint16_t writeMemoryDelay = 300;

    constexpr uint16_t departureNodeAdress = 0x0001;
    constexpr uint16_t orientationAdress = 0x0002;
    constexpr uint16_t extremityNodeAdress = 0x0003;
}

class ModeStrategy {
protected:
    enum class Orientation : uint8_t {
        NORTH_EAST = 0x00,
        NORTH_WEST = 0x0F,
        SOUTH_EAST = 0xF0,
        SOUTH_WEST = 0xFF
    };

    enum class ExtremityNode : char {
        C = 'C',
        D = 'D',
        E = 'E'
    };

    enum class DepartureNode : char {
        A = 'A',
        B = 'B'
    };

    enum class Node : char {
        ONE = '1',
        TWO = '2',
        THREE = '3',
        FOUR = '4',
        FIVE = '5',
        SIX = '6'
    };

    enum class IntersectionType {LINE, PARTIAL, FULL, INTERSECTION};

    void followLine();   
    void playConfirmationSound();
   
public:
    explicit ModeStrategy(Robot* robot) : robot_(robot) {}
    virtual ~ModeStrategy() = default;
 
protected:   
    virtual void clignotement() = 0;
    virtual void execute() = 0;

    static const char* toString(Orientation orientation);  

    void operator delete(void* ptr) {}

protected:
    Robot* robot_; 
};
