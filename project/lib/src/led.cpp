#include "led.h"

Led::Led() {
    cli();                
    configureIO();       
    sei();               
} 

void Led::configureIO() {
    LED_REGISTER |= (1 << LED_PIN_0) | (1 << LED_PIN_1);
}

void Led::turnRed() {
    configureIO();
    LED_PORT |= (1 << LED_PIN_0);
    LED_PORT &= ~(1 << LED_PIN_1);
}

void Led::turnGreen() {
    configureIO();
    LED_PORT |= (1 << LED_PIN_1);
    LED_PORT &= ~(1 << LED_PIN_0);
}

void Led::turnAmber() {
    turnRed();                          
    _delay_ms(amberDelayMsGreen_);         
    turnGreen();      
    _delay_ms(amberDelayMsRed_);                           
}

void Led::turnOff() {
    LED_REGISTER &= ~((1 << LED_PIN_0) | (1 << LED_PIN_1));
}
