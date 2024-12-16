#include "button_extern.h"

void ButtonExtern::configureButton(TriggerCondition triggerCondition, ButtonEventCallback callback, Source source) {
    cli();
    source_ = source;
    buttonImpl_.setupIO(source_);
    buttonImpl_.configureSource(source_);
    buttonImpl_.setConditionMask(source_, triggerCondition);
    buttonImpl_.setPressButtonCallback(callback);
    sei();
}

void ButtonExtern::callEventFunction() {
    buttonImpl_.callEventFunction(source_);
}
