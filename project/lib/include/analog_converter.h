#pragma once

class AnalogConverter {
public:
    static AnalogConverter& getInstance() {
        static AnalogConverter instance;
        return instance;
    }

    uint16_t read(uint8_t position);

    ~AnalogConverter();

private:
    AnalogConverter();

    AnalogConverter(const AnalogConverter&) = delete;
    AnalogConverter& operator=(const AnalogConverter&) = delete;
};
