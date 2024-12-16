#include "sound_control.h"

SoundControl::SoundControl() {
    cli();
    initializeSound();
    sei();
}

void SoundControl::initializeSound() {
    SOUND_REGISTER |= (1 << SOUND_PIN_GROUND) | (1 << SOUND_PWM_PIN);
    PORTB &= ~(1 << SOUND_PIN_GROUND);

    TCCR0A &= ~((1 << WGM02) | (1 << WGM00) | (1 << COM0A1));
    TCCR0A |= (1 << WGM01) | (1 << COM0A0);
    TCNT0 = 0;
}

void SoundControl::playFrequency(double frequency) {
    OCR0A = ((F_CPU / (multiplier * prescaler * frequency)) - 1);
}

void SoundControl::playNote(uint8_t noteId) {
    startWaveGeneration();
    double myFrequency = 0.0;

    for (uint8_t i = 0; i < sizeof(notes) / sizeof(notes[0]); i++) {
        if (notes[i] == noteId) {
            myFrequency = frequencies[i];
            break; 
        }
    }
    playFrequency(myFrequency);
}

void SoundControl::startWaveGeneration() {
    TCCR0A &= ~((1 << WGM02) | (1 << WGM00) | (1 << COM0A1));
    TCCR0A |= (1 << WGM01) | (1 << COM0A0);
    TCCR0B &= ~((1 << CS01) | (1 << CS00));
    TCCR0B |= (1 << CS02);
}

void SoundControl::turnOffSound() {
    OCR0A = 0;
    TCCR0B = 0;
}


