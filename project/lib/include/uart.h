/**
 * @class Uart
 * @brief Singleton class that manages UART communication, providing
 *        methods for transmitting and receiving data over a serial interface.
 *
 * @authors
 * - Ahmed Sami Benabbou
 * - Maroua Lassakeur
 * - Mohamed-Borheneddine Mokaddem
 * - Yasmine Meraoubi
 */

#pragma once

class Uart {
public:
    static Uart& getInstance() {
        static Uart instance; 
        return instance;
    }

    void transmit(const char* data);
    void transmit(const char* data, uint8_t length);
    void transmitChar(const char data);

    uint8_t receive();

private:
    //use this variables
    static constexpr uint16_t BAUD_RATE = 9600;     
    static constexpr uint16_t UBRR_VALUE = F_CPU / 16 / BAUD_RATE - 1;
    
    Uart();
    ~Uart();
    
};

