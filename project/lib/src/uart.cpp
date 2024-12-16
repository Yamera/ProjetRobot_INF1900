#include "uart.h"

Uart::Uart() {
    cli();

    UBRR0H = 0;
    UBRR0L = 0xCF;

    UCSR0A = (1<<UDRE0) ;
    UCSR0B = (1<<TXEN0) | (1<<RXEN0);
    UCSR0C = (1<<UCSZ00) | (1<<UCSZ01) | (1<<UCSZ02);

    sei();
}

void Uart::transmitChar(const char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void Uart::transmit(const char* data, uint8_t length) {
    for (uint8_t i = 0; i < length; ++i)
        transmitChar(data[i]);
}

void Uart::transmit(const char* data) {
    while (*data != '\0') {
        transmitChar(*data);
        data++;
    }
}

uint8_t Uart::receive() {
    while (!(UCSR0A & (1 << RXC0)));;
    return UDR0;
}

Uart::~Uart() {
    cli();

    UBRR0H = 0;      
    UBRR0L = 0;           

    UCSR0A &= ~(1 << UDRE0);  
    UCSR0B &= ~((1 << TXEN0) | (1 << RXEN0)); 
    UCSR0C &= ~((1 << UCSZ00) | (1 << UCSZ01) | (1 << UCSZ02)); 

    sei();
}

