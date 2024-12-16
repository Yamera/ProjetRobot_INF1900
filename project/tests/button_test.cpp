#include "avr/io.h"
#include "util/delay.h"
#include "avr/interrupt.h"

#include "definitions/interrupt_vectors.h"

#include "led.h"
#include "button/button_intern.h"
#include "motor_control.h"
#include "debug/debug_uart.h"

// ISR(BUTTON_INTERN_EVENT){
//     Led::getInstance().turnRed();   
//     ButtonIntern::getInstance().callEventFunction();
//     DEBUG_PRINT_STR(("ABCD"));
// }

// void ledFunction() {
//     Led::getInstance().turnRed();
// }

// int main() {
//     ButtonIntern& buttonIntern = ButtonIntern::getInstance();
//     buttonIntern.configureButton(Button::RISING_EDGE, ledFunction);
//     while(true);
//     return 0;
// }


int main() {
    MotorControl& motorControl = MotorControl::getInstance();
    motorControl.rotateRight(200);

    while(true);
}