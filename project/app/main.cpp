#include "avr/interrupt.h"

#include "defs/mode/selector.h"
#include "defs/robot.h"
#include "definitions/interrupt_vectors.h"
#include "button/button_intern.h"

#include "debug/debug_uart.h"

ISR(BUTTON_INTERN_EVENT){
    ButtonIntern::getInstance().callEventFunction();
}

int main() {
    Robot& robot = Robot::getInstance();
    //robot.writeMemory(configSelection::MODE_ADDRESS, ModeSelector::Mode::SELECTION);
    ModeSelector modeSelector = ModeSelector(&robot);
    modeSelector.selectAndExecuteMode();
    
    while(true);
    return 0;
}
