#pragma once

class SoundControl {
public:
    static SoundControl& getInstance() {
        static SoundControl instance;
        return instance;
    }

    void playFrequency(double frequency);
    void playNote(uint8_t noteId);
    void turnOffSound();

private:
    void startWaveGeneration();
    void initializeSound();

private:
    SoundControl();
    
    SoundControl(const SoundControl&) = delete;
    SoundControl& operator=(const SoundControl&) = delete;

    const uint8_t notes[37] = {
        45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62,
        63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81
    };

    const double frequencies[37] = {
        110, 116.54, 123.47, 130.81, 138.59, 146.83, 155.56, 164.81,
        174.61, 185.00, 196.00, 207.65, 220, 233.08, 246.94, 261.63,
        277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392.00, 415.30,
        440, 466.16, 493.88, 523.25, 554.37, 587.33, 622.25, 659.26,
        698.46, 739.99, 783.99, 830.61, 880
    };

    const uint16_t prescaler = 256;
    const uint8_t multiplier = 2;
};
