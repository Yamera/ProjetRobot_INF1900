#include "motor_timer_impl.h"

void MotorTimerImpl::enableWaveOutput() { 
    TCCR1C = 0; 
    setWaveformMode();
    initializePWMOutputPins();
    setPrescaler(defaultPrescaler_);
    setPWMCycles(0,0);  
}

void MotorTimerImpl::disableWaveOutput() {
    setPWMCycles(0,0);
    setPrescaler(Prescaler::NO_PRESCALER);  
    disablePWMOutputPins();
    clearWaveformMode();
}

void MotorTimerImpl::setPWMCycles(uint8_t cyclesA, uint8_t cyclesB) {
    setPWMCyclesA(cyclesA); 
    setPWMCyclesB(cyclesB);
}

void MotorTimerImpl::setPWMCyclesA(uint8_t cycles) {
    OCR2A = cycles;
}

void MotorTimerImpl::setPWMCyclesB(uint8_t cycles) {
    OCR2B = cycles; 
}

void MotorTimerImpl::setWaveformMode() {
    TCCR2A |= (1 << WGM20) | (1 << COM2A1) | (1 << COM2B1); 
}

void MotorTimerImpl::clearWaveformMode() {
    TCCR2A &= ~((1 << WGM20) | (1 << COM2A1) | (1 << COM2B1)); 
}

void MotorTimerImpl::setPrescaler(Prescaler prescaler) {
    resetPrescaler(); 

    switch (prescaler) {
        case Prescaler::NO_PRESCALER:
            TCCR2B |= (1 << CS20);
            break;
        case Prescaler::SCALE8:
            TCCR2B |= (1 << CS21);
            break;
        case Prescaler::SCALE64:
            TCCR2B|=(1 << CS20) | (1 << CS21);
            break;
        case Prescaler::SCALE256:
            TCCR2B |= (1 << CS22);
            break;
        case Prescaler::SCALE1024:
            TCCR2B |= (1 << CS22) | (1 << CS20);
            break;
    }
}

void MotorTimerImpl::resetPrescaler() {
    TCCR2B &= ~((1 << CS20) | (1 << CS21) | (1 << CS22)); 
}

void MotorTimerImpl::initializePWMOutputPins() {
    MOTOR_PWM_REGISTER |= (1 << MOTOR_PWM_PIN_LEFT) | (1 << MOTOR_PWM_PIN_RIGHT); 
}

void MotorTimerImpl::disablePWMOutputPins() {
    MOTOR_PWM_REGISTER  &= ~((1 << MOTOR_PWM_PIN_LEFT) | (1 << MOTOR_PWM_PIN_RIGHT)); 
}