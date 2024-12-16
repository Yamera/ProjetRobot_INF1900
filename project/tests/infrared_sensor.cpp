#include "avr/io.h"
#include "util/delay.h"
#include "avr/interrupt.h"
#include "led.h"
#include "sensor/infrared_sensor.h"
#include "sound_control.h"
#include "debug/debug_uart.h"
#include "robot.h"

Robot& robot_ = Robot::getInstance();

namespace navigation_control {
    static constexpr uint8_t rotationSpeed = 80;
    static constexpr uint8_t quickAdvanceSpeed = 80;
    static constexpr uint8_t forwardSpeed = 80;
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

//test intersection
// int main() {
//     InfraredSensor&  infraredSensor = InfraredSensor::getInstance();

//     while (true) {
//         followLineDefault();
//         uint8_t distanceValue = infraredSensor.calculateDistinctDistance();
//         DEBUG_PRINT_INT((distanceValue));
//         bool isPoleDetected = infraredSensor.isPoleDetectedFromIntersection();
//     }

//     while(true);
//     return 0;
// }

//test intersection
int main() {
    InfraredSensor&  infraredSensor = InfraredSensor::getInstance();
    Led& led = Led::getInstance();

    while (true) {
        uint8_t distanceValue = infraredSensor.calculateDistinctDistance();
        DEBUG_PRINT_INT((distanceValue));

        bool isPoleDetected = infraredSensor.isPoleDetectedFromExtremity();
    }

    while(true);
    return 0;
}

//test pole
// int main() {
//     InfraredSensor&  infraredSensor = InfraredSensor::getInstance();
//     Led& led = Led::getInstance();
//     SoundControl& soundControl = SoundControl::getInstance();

//     while (true) {
//         uint8_t distanceValue = infraredSensor.calculateMedianDistance();        
//         DEBUG_PRINT_INT((distanceValue));

//         ProximityState proximityState = infraredSensor.determineProximityState();
//         if(proximityState == ProximityState::WITHIN_RANGE){
//             soundControl.playSound(750);
//         }
//         else if(proximityState == ProximityState::TOO_CLOSE) {
//             soundControl.turnOffSound();
//             led.turnRed();
//         }
//         else if(proximityState == ProximityState::TOO_FAR) {
//             soundControl.turnOffSound();
//             led.turnGreen();

//         }

//         _delay_ms(10);
//     }
// }
