#include "avr/io.h"
#include "avr/delay.h"
#include "avr/interrupt.h"
#include "motor_control.h"
#include "led.h"
#include "sensor/line_sensor.h"

enum class IntersectionType {LINE, PARTIAL, FULL};

MotorControl &robot = MotorControl::getInstance();
LineSensor &lineSensor = LineSensor::getInstance();
Led &led = Led::getInstance();
const uint8_t speedF = 80;
const uint8_t speedR = 80;
const uint8_t speedL = 80;
const uint8_t speed = 80;

bool allSensorsActive()
{
    return lineSensor.isSensorActive(SensorType::MID) &&
           lineSensor.isSensorActive(SensorType::SHORT_LEFT) &&
           lineSensor.isSensorActive(SensorType::SHORT_RIGHT) &&
           lineSensor.isSensorActive(SensorType::FAR_LEFT) &&
           lineSensor.isSensorActive(SensorType::FAR_RIGHT);
}

bool noSensorsActive()
{
    return !lineSensor.isSensorActive(SensorType::MID) &&
           !lineSensor.isSensorActive(SensorType::SHORT_LEFT) &&
           !lineSensor.isSensorActive(SensorType::SHORT_RIGHT) &&
           !lineSensor.isSensorActive(SensorType::FAR_LEFT) &&
           !lineSensor.isSensorActive(SensorType::FAR_RIGHT);
}

void followLine()
{
    robot.moveForward(speedF);
    if (lineSensor.isSensorActive(SensorType::SHORT_LEFT))
    {
        robot.controlLeftMotor(speedF - 15);
        robot.controlRightMotor(speedF + 5);
    }
    else if (lineSensor.isSensorActive(SensorType::SHORT_RIGHT))
    {
        robot.controlRightMotor(speedF - 10);
    }
    else if (lineSensor.isSensorActive(SensorType::MID))
    {
        robot.controlRightMotor(speedF + 10);
        robot.controlLeftMotor(speedF);
    }
}

IntersectionType ajustVerifieRight(MotorControl &robot, LineSensor &lineSensor)
{
    const uint8_t speed = 70;
    robot.rotateRight(speed);
    while (!lineSensor.isSensorActive(LineSensor::MID)){};
    robot.halt();
    _delay_ms(500);
    
    if (lineSensor.isSensorActive(LineSensor::FAR_RIGHT)){
        robot.moveForward(speedF);
        while(lineSensor.isSensorActive(LineSensor::FAR_RIGHT) && !lineSensor.isSensorActive(LineSensor::FAR_LEFT)){}
        if (lineSensor.isSensorActive(LineSensor::FAR_LEFT)){
            return IntersectionType::FULL;
        }
        else if(!lineSensor.isSensorActive(LineSensor::FAR_LEFT)){
            return IntersectionType::PARTIAL;
        }
    }
    return IntersectionType::LINE;
}

IntersectionType ajustVerifieLeft(MotorControl &robot, LineSensor &lineSensor)
{
    const uint8_t speed = 70;
    robot.rotateLeft(speed);
    while (!lineSensor.isSensorActive(LineSensor::MID))
    {
    };
    robot.halt();
    _delay_ms(500);
    if (lineSensor.isSensorActive(LineSensor::FAR_LEFT)){
        robot.moveForward(speedF);
        robot.controlRightMotor(speedF + 7);
        while(lineSensor.isSensorActive(LineSensor::FAR_LEFT) && !lineSensor.isSensorActive(LineSensor::FAR_RIGHT)){}
        if (lineSensor.isSensorActive(LineSensor::FAR_RIGHT)){
            return IntersectionType::FULL;
        }
        else{
            return IntersectionType::PARTIAL;
        }
    }
    return IntersectionType::LINE;
}

void partialIntersection(bool toRight) {
    bool turnRight = false;
    bool turnLeft = false;
    _delay_ms(500);
    robot.moveForward(speedF);
    if (toRight)
    {
        turnRight = true;
        turnLeft = false;
        robot.controlLeftMotor(speedF + 15);
    }
    else {
        turnRight = false;
        turnLeft = true;
        robot.controlRightMotor(speedF + 25);
    }

    while (lineSensor.isSensorActive(SensorType::FAR_LEFT) || lineSensor.isSensorActive(SensorType::FAR_RIGHT))
    {
    }
    robot.halt();
    _delay_ms(500);
    while (!noSensorsActive())
    {
        followLine();
    }

    robot.halt();
    _delay_ms(500);

    if (turnRight)
    {
        do
        {
            robot.rotateRight(speed);
        } while (!lineSensor.isSensorActive(SensorType::MID));
        robot.halt();
    }
    else
    {
        do
        {
            robot.rotateLeft(speed);
        }

        while (!lineSensor.isSensorActive(SensorType::MID));
        robot.halt();
    }
}

void fullIntersection() {
    bool RightLineFind = true;
    bool LeftLineFind = true;

    robot.moveForward(speed);
    _delay_ms(1500);
    robot.halt();
    _delay_ms(1000);

    
    uint16_t elapsed_time = 0;
    const uint16_t timeout_ms = 1500;
    const uint16_t step_delay = 10;

    // roue alignees
    robot.rotateRight(250);
    _delay_ms(50);
    robot.rotateRight(speed);
    while (elapsed_time <= timeout_ms && noSensorsActive()){
        _delay_ms(step_delay);
        elapsed_time += step_delay;        
    }
    robot.halt();
    _delay_ms(500); 
    elapsed_time = 0;
    // robot a tourne a droite
    if (!noSensorsActive()){ // sur une bande
        while (elapsed_time <= 500 && !noSensorsActive()){
            followLine();
            _delay_ms(step_delay);
            elapsed_time += step_delay;
        }
        if (!noSensorsActive()){
            return;
        }  
    }
    elapsed_time = 0;

    robot.rotateLeft(250);
    _delay_ms(50);
    robot.rotateLeft(speed);
    while(elapsed_time <= (timeout_ms * 2 - 200) && noSensorsActive()){
        _delay_ms(step_delay);
        elapsed_time += step_delay;
    }
    robot.halt();
    _delay_ms(500); 
    elapsed_time = 0;
    // robot a tourne a gauche
    if (!noSensorsActive()){ // sur une bande
        while (elapsed_time <= 500 && !noSensorsActive()){
            followLine();
            _delay_ms(step_delay);
            elapsed_time += step_delay;
        }
        if (!noSensorsActive()){
            return;
        }  
    }
    
    elapsed_time = 0;
    robot.rotateRight(250);
    _delay_ms(50);
    robot.rotateRight(speed);
    while (elapsed_time <= timeout_ms){
        _delay_ms(step_delay);
        elapsed_time += step_delay;        
    }
    robot.halt();

    while (true){}
}

void navigate() {
    while (true)
    {
        if (lineSensor.isSensorActive(SensorType::FAR_RIGHT))
        {
            robot.halt();
            _delay_ms(500);
            
            IntersectionType typeOfIntersection = ajustVerifieRight(robot, lineSensor);
            if (typeOfIntersection == IntersectionType::PARTIAL)
            {
                partialIntersection(true);
            }
            else if (typeOfIntersection == IntersectionType::FULL){
                robot.moveForward(speedF);
                while (lineSensor.isSensorActive(LineSensor::FAR_LEFT) || lineSensor.isSensorActive(LineSensor::FAR_RIGHT)){}
                _delay_ms(50);
                if (noSensorsActive()){
                    led.turnGreen();
                    fullIntersection();

                }
            }
        }

        else if (lineSensor.isSensorActive(SensorType::FAR_LEFT))
        {
            led.turnRed();
            robot.halt();
            _delay_ms(500);
            IntersectionType typeOfIntersection = ajustVerifieLeft(robot, lineSensor);
            if (typeOfIntersection == IntersectionType::PARTIAL)
            {
                partialIntersection(false);
            }
            else if (typeOfIntersection == IntersectionType::FULL){
                robot.moveForward(speedF);
                while (lineSensor.isSensorActive(LineSensor::FAR_LEFT) || lineSensor.isSensorActive(LineSensor::FAR_RIGHT)){}
                _delay_ms(50);
                if (noSensorsActive()){
                    led.turnRed();
                    fullIntersection();

                }
            }
        }

        else if (noSensorsActive()){
            robot.halt();
            robot.rotateRight(speedR);
            
            while (noSensorsActive()){}
            
        }
        
        else
        {
            followLine();
        }
    }
}

int main()
{
    DDRA &= ~(1 << PA1);
    navigate();
    return 0;
}