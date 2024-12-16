#include "button_intern.h"


void ButtonIntern::configureButton(TriggerCondition triggerCondition, ButtonEventCallback callback) {
    cli();
    buttonImpl_.setupIO(source_);
    buttonImpl_.configureSource(source_);
    buttonImpl_.setConditionMask(source_, triggerCondition);
    buttonImpl_.setPressButtonCallback(callback);
    sei();
}

void ButtonIntern::callEventFunction() {
    buttonImpl_.callEventFunction(source_);
}
