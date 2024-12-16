#include "avr/io.h"
#include "avr/delay.h"
#include "avr/interrupt.h"
#include "motor_control.h"
#include "led.h"
#include "sensor/line_sensor.h"
#include "sound_control.h"
#include "debug/debug_uart.h"

bool fullIntersectionRight = false;
bool fullIntersectionLeft = false;
bool partialIntersectionRight = false;
bool partialIntersectionLeft = false;
char destination;
enum RobotState
{
    CASE_AtoD = 1,    // A -> D
    CASE_AtoC = 2,    // A -> C
    CASE_AtoEby3 = 3, // A -> E
    CASE_AtoEby4 = 4, // A -> E
    CASE_BtoC = 5,    // B -> C
    CASE_BtoD = 6,    // B -> D
    CASE_BtoEby5 = 7, // B -> E
    CASE_BtoEby6 = 8  // B -> E
};

RobotState currentState;

enum class IntersectionType
{
    Line,
    Partial,
    Full
};

MotorControl &robot = MotorControl::getInstance();
LineSensor &lineSensor = LineSensor::getInstance();
Led &led = Led::getInstance();
SoundControl &sound = SoundControl::getInstance();

const uint8_t speedF = 80;
const uint8_t speedR = 80;
const uint8_t speedL = 80;
const uint8_t speed = 80;

void findEnd()
{
    // navigate();
    // Debugging: afficher les valeurs des intersections

    if (fullIntersectionLeft && !fullIntersectionRight && !partialIntersectionLeft && !partialIntersectionRight)
    {
        currentState = CASE_AtoD; // A -> D
        return;
    }
    else if (!fullIntersectionLeft && !fullIntersectionRight && !partialIntersectionLeft && partialIntersectionRight)
    {
        currentState = CASE_AtoC; // A -> C
        return;
    }
    else if (fullIntersectionRight && partialIntersectionLeft && !fullIntersectionLeft && partialIntersectionRight)
    {
        currentState = CASE_AtoEby3; // A -> E via 3
        return;
    }
    else if (fullIntersectionLeft && fullIntersectionRight && !partialIntersectionLeft && !partialIntersectionRight)
    {
        currentState = CASE_AtoEby4; // A -> E via 4
        return;
    }
    else if (partialIntersectionLeft && partialIntersectionRight && !fullIntersectionLeft && !fullIntersectionRight)
    {
        currentState = CASE_BtoC; // B -> C
        return;
    }
    else if (!partialIntersectionLeft && partialIntersectionRight && fullIntersectionLeft && !fullIntersectionRight)
    {
        currentState = CASE_BtoD; // B -> D
        return;
    }
    else if (!partialIntersectionLeft && !partialIntersectionRight && fullIntersectionRight && !fullIntersectionLeft)
    {
        currentState = CASE_BtoEby5; // B -> E via 5
        return;
    }
    else if (partialIntersectionLeft && !partialIntersectionRight && !fullIntersectionLeft && !fullIntersectionRight)
    {
        currentState = CASE_BtoEby6; // B -> E via 6
        return;
    }
    return;
}
void playSound()
{
    // Premier bip
    sound.playNote(220); // Fréquence de 2000 Hz
    _delay_ms(750);       // Durée de 750 ms
    sound.turnOffSound(); // Arrêter le son
    _delay_ms(50);        // Silence de 50 ms

    // Deuxième bip
    sound.playNote(220); // Fréquence de 2000 Hz
    _delay_ms(750);       // Durée de 750 ms
    sound.turnOffSound(); // Arrêter le son
    return;
}

void handleState()
{
    switch (currentState)
    {
    case CASE_AtoD: // A -> D
        led.turnRed();
        destination = 'D';
        playSound();
        return;

    case CASE_AtoC: // A -> C
        led.turnGreen();
        destination = 'C';
        playSound();
        return;

    case CASE_AtoEby3: // A -> E via 3
    case CASE_AtoEby4: // A -> E via 4
        destination = 'E';
        playSound();
        while(true){
            led.turnAmber();
        }
        return;

    case CASE_BtoC: // B -> C
        led.turnGreen();
        destination = 'C';
        playSound();
        return;

    case CASE_BtoD: // B -> D
        led.turnRed();
        destination = 'D';
        playSound();
        return;

    case CASE_BtoEby5: // B -> E via 5
    case CASE_BtoEby6: // B -> E via 6     
        destination = 'E';
        playSound();
        while(true){
            led.turnAmber();
        }
        return;

    default:
        led.turnOff();
        return;
    }
}

bool allSensorsActive(LineSensor &lineSensor)
{
    return lineSensor.isSensorActive(SensorType::MID) &&
           lineSensor.isSensorActive(SensorType::SHORT_LEFT) &&
           lineSensor.isSensorActive(SensorType::SHORT_RIGHT) &&
           lineSensor.isSensorActive(SensorType::FAR_LEFT) &&
           lineSensor.isSensorActive(SensorType::FAR_RIGHT);
}

bool noSensorsActive(LineSensor &lineSensor)
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
    while (!lineSensor.isSensorActive(LineSensor::MID))
    {
    };
    robot.halt();
    _delay_ms(500);

    if (lineSensor.isSensorActive(LineSensor::FAR_RIGHT))
    {
        robot.moveForward(speedF);
        while (lineSensor.isSensorActive(LineSensor::FAR_RIGHT) && !lineSensor.isSensorActive(LineSensor::FAR_LEFT))
        {
        }
        if (lineSensor.isSensorActive(LineSensor::FAR_LEFT))
        {
            return IntersectionType::Full;
        }
        else if (!lineSensor.isSensorActive(LineSensor::FAR_LEFT))
        {
            return IntersectionType::Partial;
        }
    }
    return IntersectionType::Line;
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
    if (lineSensor.isSensorActive(LineSensor::FAR_LEFT))
    {
        robot.moveForward(speedF);
        robot.controlRightMotor(speedF + 7);
        while (lineSensor.isSensorActive(LineSensor::FAR_LEFT) && !lineSensor.isSensorActive(LineSensor::FAR_RIGHT))
        {
        }
        if (lineSensor.isSensorActive(LineSensor::FAR_RIGHT))
        {
            return IntersectionType::Full;
        }
        else
        {
            return IntersectionType::Partial;
        }
    }
    return IntersectionType::Line;
}

void partialIntersection(bool toRight)
{
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
    else
    {
        turnRight = false;
        turnLeft = true;
        robot.controlRightMotor(speedF + 25);
    }

    while (lineSensor.isSensorActive(SensorType::FAR_LEFT) || lineSensor.isSensorActive(SensorType::FAR_RIGHT))
    {
    }
    robot.halt();
    _delay_ms(500);
    while (!noSensorsActive(lineSensor))
    {
        followLine();
    }

    robot.halt();
    _delay_ms(500);

    if (turnRight)
    {
        partialIntersectionRight = true;

        do
        {
            robot.rotateRight(speed);
        } while (!lineSensor.isSensorActive(SensorType::MID));
        robot.halt();
    }
    else
    {
        partialIntersectionLeft = true;
        do
        {
            robot.rotateLeft(speed);
        }

        while (!lineSensor.isSensorActive(SensorType::MID));
        robot.halt();
    }
}

void fullIntersection()
{
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
    while (elapsed_time <= timeout_ms && noSensorsActive(lineSensor))
    {
        _delay_ms(step_delay);
        elapsed_time += step_delay;
    }
    robot.halt();
    _delay_ms(500);
    elapsed_time = 0;
    // robot a tourne a droite
    if (!noSensorsActive(lineSensor))
    { // sur une bande
        while (elapsed_time <= 500 && !noSensorsActive(lineSensor))
        {
            followLine();
            _delay_ms(step_delay);
            elapsed_time += step_delay;
        }
        if (!noSensorsActive(lineSensor))
        {
            fullIntersectionRight = true;
            return;
        }
    }
    elapsed_time = 0;

    robot.rotateLeft(250);
    _delay_ms(50);
    robot.rotateLeft(speed);
    while (elapsed_time <= (timeout_ms * 2 - 200) && noSensorsActive(lineSensor))
    {
        _delay_ms(step_delay);
        elapsed_time += step_delay;
    }
    robot.halt();
    _delay_ms(500);
    elapsed_time = 0;
    // robot a tourne a gauche
    if (!noSensorsActive(lineSensor))
    { // sur une bande
        while (elapsed_time <= 500 && !noSensorsActive(lineSensor))
        {
            followLine();
            _delay_ms(step_delay);
            elapsed_time += step_delay;
        }
        if (!noSensorsActive(lineSensor))
        {
            fullIntersectionLeft = true;
            return;
        }
    }

    elapsed_time = 0;
    robot.rotateRight(250);
    _delay_ms(50);
    robot.rotateRight(speed);
    while (elapsed_time <= timeout_ms)
    {
        _delay_ms(step_delay);
        elapsed_time += step_delay;
    }

    robot.halt();

    findEnd();
    handleState();

    while (true)
    {
    }
}

void navigate()
{
    while (true)
    {
        if (lineSensor.isSensorActive(SensorType::FAR_RIGHT))
        {
            robot.halt();
            _delay_ms(500);

            IntersectionType typeOfIntersection = ajustVerifieRight(robot, lineSensor);
            if (typeOfIntersection == IntersectionType::Partial)
            {
                partialIntersection(true);
            }
            else if (typeOfIntersection == IntersectionType::Full)
            {
                robot.moveForward(speedF);
                while (lineSensor.isSensorActive(LineSensor::FAR_LEFT) || lineSensor.isSensorActive(LineSensor::FAR_RIGHT))
                {
                }
                _delay_ms(100);
                if (noSensorsActive(lineSensor))
                {
                    fullIntersection();
                }
            }
        }

        else if (lineSensor.isSensorActive(SensorType::FAR_LEFT))
        {
            robot.halt();
            _delay_ms(500);
            IntersectionType typeOfIntersection = ajustVerifieLeft(robot, lineSensor);
            if (typeOfIntersection == IntersectionType::Partial)
            {
                partialIntersection(false);
            }
            else if (typeOfIntersection == IntersectionType::Full)
            {
                robot.moveForward(speedF);
                while (lineSensor.isSensorActive(LineSensor::FAR_LEFT) || lineSensor.isSensorActive(LineSensor::FAR_RIGHT))
                {
                }
                _delay_ms(100);
                if (noSensorsActive(lineSensor))
                {
                    fullIntersection();
                }
            }
        }

        else if (noSensorsActive(lineSensor))
        {
            robot.halt();
            robot.rotateRight(speedR);
            while (noSensorsActive(lineSensor))
            {
            }
        }

        else
        {
            followLine();
        }
    }
}

int main() {

    DDRA &= ~(1 << PA1);
    navigate();

    return 0;
}

// int main() {
//     for(uint8_t i = 0; i< 5000; i++){
//             led.turnAmber();
//         }

//     while(true);
// }