#pragma once

#define LED_REGISTER                DDRC
#define LED_PORT                    PORTC
#define LED_PIN_0                   PIND6
#define LED_PIN_1                   PIND7

#define SENSOR_REGISTER             DDRA
#define SENSOR_PIN                  PINA
#define SENSOR_PORT                 PORTA 
#define SENSOR_PIN_0                PA0
#define SENSOR_PIN_1                PA2
#define SENSOR_PIN_2                PA3
#define SENSOR_PIN_3                PA4
#define SENSOR_PIN_4                PA5


#define INFRARED_REGISTER           DDRA
#define INFRARED_PIN                PA1


#define BUTTON_REGISTER_INT_0_1     DDRD
#define BUTTON_PIN_REGISTER         PIND
#define BUTTON_REGISTER_INT_2       DDRB
#define BUTTON_INT_0_PIN            PD2
#define BUTTON_INT_1_PIN            PD3
#define BUTTON_INT_2_PIN            PB2


#define MOTOR_PWM_REGISTER          DDRD
#define MOTOR_PWM_PIN_LEFT          PD6
#define MOTOR_PWM_PIN_RIGHT         PD7

#define MOTOR_REGISTER              DDRD
#define MOTOR_PORT                  PORTD  
#define MOTOR_PIN_LEFT              PD4       
#define MOTOR_PIN_RIGHT             PD5  



#define SOUND_REGISTER              DDRB
#define SOUND_PORT                  PORTB
#define SOUND_PWM_PIN               PB3 
#define SOUND_PIN_GROUND            PB1
  



