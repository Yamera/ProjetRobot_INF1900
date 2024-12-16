#include "analog_converter.h"

AnalogConverter::AnalogConverter() {
   ADMUX  = ( 0 << REFS1 ) | ( 0 << REFS0 ) | ( 0 << ADLAR ) | ( 0 << MUX4 ) |
            ( 0 << MUX3 ) | ( 0 << MUX2 ) | ( 0 << MUX1) | ( 0 << MUX0 );

   ADCSRA = ( 1 << ADEN ) | ( 0 << ADSC )  | ( 0 << ADATE ) | ( 0 << ADIF ) |
            ( 0 << ADIE ) | ( 1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 0 << ADPS0 );
}

uint16_t AnalogConverter::read(uint8_t position) {
   uint16_t adcVal;

   ADMUX  &=  ~(( 1 << MUX4 ) | ( 1 << MUX3 ) | 
                ( 1 << MUX2 ) | ( 1 << MUX1)  | ( 1 << MUX0 ));

   ADMUX |= ((position & 0x07) << MUX0);

   ADCSRA |= ( 1 << ADSC );

   while( ! (ADCSRA & ( 1 << ADIF )) );

   ADCSRA |= (1 << ADIF);

   adcVal =   ADCL ;
   adcVal +=  ADCH << 8;

   return adcVal;
}

AnalogConverter::~AnalogConverter() {
   ADCSRA = 0 << ADEN ;
}