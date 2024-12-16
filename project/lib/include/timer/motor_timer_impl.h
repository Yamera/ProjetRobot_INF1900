#pragma once

#include "definitions/timer_definitions.h"

class MotorTimerImpl {
private:
    MotorTimerImpl& getInstance() {
        static MotorTimerImpl instance;
        return instance;
    }   

private:
    void enableWaveOutput();
    void disableWaveOutput();

    void setPWMCycles(uint8_t cyclesA, uint8_t cyclesB);
    void setPWMCyclesA(uint8_t cycles);
    void setPWMCyclesB(uint8_t cycles);
   
    void setPrescaler(Prescaler prescaler);
    void resetPrescaler();

    void setWaveformMode();
    void clearWaveformMode();
    
private:
    void initializePWMOutputPins();
    void disablePWMOutputPins();

private:
    MotorTimerImpl() = default;
    ~MotorTimerImpl() = default;

    MotorTimerImpl(const MotorTimerImpl&) = delete;
    MotorTimerImpl& operator=(const MotorTimerImpl&) = delete;

    static constexpr Prescaler defaultPrescaler_ = Prescaler::SCALE1024;  

    friend class MotorControl; 
};
