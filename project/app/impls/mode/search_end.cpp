#include "../defs/mode/search_end.h"
#include "debug/debug_uart.h"

void SearchEndMode::execute() {
    clignotement();
    navigate();
    
}

void SearchEndMode::clignotement() {
    for (int i = 0; i < configClignotement::totalBlinks; i++){
        robot_->turnLedGreen();  
        _delay_ms(configClignotement::blinkIntervalMs);
        robot_->turnLedOff();          
        _delay_ms(configClignotement::blinkIntervalMs);
    }
}

void SearchEndMode::writeCoordInMemory() {
    auto writeMemoryValues = [this](DepartureNode departure, Orientation orientation, ExtremityNode extremity) {
        robot_->writeMemory(configRapport::departureNodeAdress, static_cast<uint8_t>(departure));
        _delay_ms(configRapport::writeMemoryDelay);
        robot_->writeMemory(configRapport::orientationAdress, static_cast<uint8_t>(orientation));
        _delay_ms(configRapport::writeMemoryDelay);
        robot_->writeMemory(configRapport::extremityNodeAdress, static_cast<uint8_t>(extremity));
        _delay_ms(configRapport::writeMemoryDelay);
    };

    switch (currentState_) {
        case RobotState::A_TO_D:
            writeMemoryValues(DepartureNode::A, Orientation::NORTH_WEST, ExtremityNode::D);
            break;

        case RobotState::A_TO_C:
            writeMemoryValues(DepartureNode::A, Orientation::SOUTH_WEST, ExtremityNode::C);
            break;

        case RobotState::A_TO_E_VIA_3:
            writeMemoryValues(DepartureNode::A, Orientation::NORTH_EAST, ExtremityNode::E);
            break;

        case RobotState::A_TO_E_VIA_4:
            writeMemoryValues(DepartureNode::A, Orientation::SOUTH_EAST, ExtremityNode::E);
            break;

        case RobotState::B_TO_C:
            writeMemoryValues(DepartureNode::B, Orientation::NORTH_WEST, ExtremityNode::C);
            break;

        case RobotState::B_TO_D:
            writeMemoryValues(DepartureNode::B, Orientation::SOUTH_WEST, ExtremityNode::D);
            break;

        case RobotState::B_TO_E_VIA_5:
            writeMemoryValues(DepartureNode::B, Orientation::NORTH_EAST, ExtremityNode::E);
            break;

        case RobotState::B_TO_E_VIA_6:
            writeMemoryValues(DepartureNode::B, Orientation::SOUTH_EAST, ExtremityNode::E);
            break;

        default:
            break;
    }
}

void SearchEndMode::turnLedBaseOnExtremity() {
    switch (currentState_) {
        case RobotState::A_TO_D:
        case RobotState::B_TO_D:
            robot_->turnLedRed();
            while(true);
            break;

        case RobotState::A_TO_C:
        case RobotState::B_TO_C:
            robot_->turnLedGreen();
            while(true);
            break;

        case RobotState::A_TO_E_VIA_3:
        case RobotState::A_TO_E_VIA_4:
        case RobotState::B_TO_E_VIA_5:
        case RobotState::B_TO_E_VIA_6:
            while(true){
                robot_->turnLedAmber();
            }
            break;

        default:
            return; 
    }
}


void SearchEndMode::processInferredData() {
    if (fullIntersectionLeft_ && !fullIntersectionRight_ && !partialIntersectionLeft_ && !partialIntersectionRight_) {
        currentState_ = RobotState::A_TO_D; 
    } else if (!fullIntersectionLeft_ && !fullIntersectionRight_ && !partialIntersectionLeft_ && partialIntersectionRight_) {
        currentState_ = RobotState::A_TO_C; 
    } else if (fullIntersectionRight_ && partialIntersectionLeft_ && !fullIntersectionLeft_ && !partialIntersectionRight_) {
        currentState_ = RobotState::A_TO_E_VIA_3; 
    } else if (fullIntersectionLeft_ && fullIntersectionRight_ && !partialIntersectionLeft_ && !partialIntersectionRight_) {
        currentState_ = RobotState::A_TO_E_VIA_4; 
    } else if (partialIntersectionLeft_ && partialIntersectionRight_ && !fullIntersectionLeft_ && !fullIntersectionRight_) {
        currentState_ = RobotState::B_TO_C; 
    } else if (!partialIntersectionLeft_ && partialIntersectionRight_ && fullIntersectionLeft_ && !fullIntersectionRight_) {
        currentState_ = RobotState::B_TO_D; 
    } else if (!partialIntersectionLeft_ && !partialIntersectionRight_ && fullIntersectionRight_ && !fullIntersectionLeft_) {
        currentState_ = RobotState::B_TO_E_VIA_5;
    } else if (partialIntersectionLeft_ && !partialIntersectionRight_ && !fullIntersectionLeft_ && !fullIntersectionRight_) {
        currentState_ = RobotState::B_TO_E_VIA_6; 
    }
}

/*
 * The ajustVerifieRight function adjusts the robot's orientation and verifies the type of intersection
 * it encounters while turning to the right. The function performs the following steps:
 * 1. Initiates a right rotation at a specified speed until the middle sensor detects the line.
 * 2. Stops the motors briefly for stabilization.
 * 3. Checks the state of the far-right and far-left sensors:
 *    - If both sensors are active, it moves forward until one sensor is inactive, then determines:
 *      * FULL intersection if the far-left sensor remains active.
 *      * PARTIAL intersection if the far-left sensor is inactive.
 * 4. Returns LINE if no conditions for FULL or PARTIAL are met.
 */
ModeStrategy::IntersectionType SearchEndMode::ajustVerifieRight() {
    const uint8_t turningSpeed  = 78;
    robot_->rotateRight(turningSpeed);

    while (!robot_->isSensorActive(LineSensor::MID)){};

    robot_->haltMotor();
    _delay_ms(500);
    
    if (robot_->isSensorActive(LineSensor::FAR_RIGHT)){
        robot_->moveForward(navigation::forwardSpeed);

        while(robot_->isSensorActive(LineSensor::FAR_RIGHT) && !robot_->isSensorActive(LineSensor::FAR_LEFT)){}

        if (robot_->isSensorActive(LineSensor::FAR_LEFT)){
            return IntersectionType::FULL;
        }
        else if(!robot_->isSensorActive(LineSensor::FAR_LEFT)){
            return IntersectionType::PARTIAL;
        }
    }
    return IntersectionType::LINE;
}

ModeStrategy::IntersectionType SearchEndMode::ajustVerifieLeft() {
    const uint8_t turningSpeed  = 78;
    robot_->rotateLeft(turningSpeed);

    while (!robot_->isSensorActive(LineSensor::MID)){};

    robot_->haltMotor();
    _delay_ms(500);

    if (robot_->isSensorActive(LineSensor::FAR_LEFT)){
        robot_->moveForward(navigation::forwardSpeed);  
                    
        robot_->controlRightMotorSpeed(navigation::forwardSpeed + 7); // wierd

        while(robot_->isSensorActive(LineSensor::FAR_LEFT) && !robot_->isSensorActive(LineSensor::FAR_RIGHT)){}

        if (robot_->isSensorActive(LineSensor::FAR_RIGHT)){
            return IntersectionType::FULL;
        }
        else {
            return IntersectionType::PARTIAL;
        }
    }
    return IntersectionType::LINE;
}

/*
 * The partialIntersection function handles navigation through a partial intersection 
 * based on the specified direction (to the right or to the left). The function performs 
 * the following steps:
 * 
 * 1. Delays briefly to stabilize the robot before starting.
 * 2. Moves the robot forward at a set speed and adjusts motor speeds depending on 
 *    the desired turning direction:
 *    - If turning right, the left motor speed is slightly increased.
 *    - If turning left, the right motor speed is slightly increased.
 * 3. Waits until both far-left and far-right sensors are inactive.
 * 4. Stops the robot momentarily, then follows the line until no sensors are active.
 * 5. Stops the robot again and initiates a rotation:
 *    - Rotates right until the middle sensor detects the line if turning right.
 *    - Rotates left until the middle sensor detects the line if turning left.
 * 6. Halts the motors after successfully aligning with the middle sensor.
 */
void SearchEndMode::partialIntersection(bool toRight) {
    incrementIntersections();
    _delay_ms(500);

    robot_->moveForward(navigation::forwardSpeed);

    if (toRight) {
        robot_->controlLeftMotorSpeed(navigation::forwardSpeed + 45);
    }
    else {
        robot_->controlRightMotorSpeed(navigation::forwardSpeed + 60);
    }

    while (robot_->isSensorActive(SensorType::FAR_LEFT) || robot_->isSensorActive(SensorType::FAR_RIGHT)){};

    robot_->haltMotor();
    _delay_ms(500);

    while (!robot_->noSensorsActive()){
        followLine();
        if (robot_->isSensorActive(SensorType::FAR_RIGHT)){
            while (robot_->isSensorActive(SensorType::FAR_RIGHT))
            {
                robot_->controlLeftMotorSpeed(navigation::forwardSpeed+40);
            }
            
        }
        else if (robot_->isSensorActive(SensorType::FAR_LEFT)){
            while (robot_->isSensorActive(SensorType::FAR_LEFT))
            {
                robot_->controlRightMotorSpeed(navigation::forwardSpeed+40);
            }
            
        }
    }

    robot_->haltMotor();
    _delay_ms(500);

    if (toRight) {
        partialIntersectionRight_ = true;
        robot_->rotateRight(250);
        _delay_ms(50);
        robot_->rotateRight(navigation::speed);
        while (!robot_->isSensorActive(SensorType::MID));
        robot_->haltMotor();
    }
    else {
        partialIntersectionLeft_ = true;
        robot_->rotateLeft(250);
        _delay_ms(50);
        robot_->rotateLeft(navigation::speed);
        while (!robot_->isSensorActive(SensorType::MID));       
        robot_->haltMotor();
    }
}

/*
 * The fullIntersection function handles the robot's navigation at a full intersection. 
 * It systematically searches for a valid path by rotating and checking sensor inputs 
 * in multiple directions. The function performs the following steps:
 * 
 * 1. Moves forward briefly, then stops to stabilize.
 * 2. Initiates a right rotation and waits until one or more sensors detect a line:
 *    - If a line is detected during the right turn, the robot follows the line for 
 *      a brief period to confirm alignment. If alignment is successful, the function ends.
 * 3. If no line is found on the right, initiates a left rotation and repeats the process:
 *    - If a line is detected during the left turn, the robot follows the line for 
 *      a brief period to confirm alignment. If alignment is successful, the function ends.
 * 4. If no line is found on either side, returns to the original orientation by performing 
 *    another right rotation for a set duration.
 * 5. Halts the robot after all checks. If no valid path is found, the function enters an 
 *    infinite loop to prevent further action.
 * 
 * This function ensures that the robot explores all possible directions in a systematic 
 * manner to identify a valid path at a full intersection.
 */

void SearchEndMode::rotateAndCheckSensors(bool rotateRight, uint16_t timeoutDurationMs) {
    uint16_t elapsedTimeMs = 0;
    
    if (rotateRight) {
        robot_->rotateRight(250);
        _delay_ms(50);
        robot_->rotateRight(navigation::speed);
    } else {
        robot_->rotateLeft(250);
        _delay_ms(50);
        robot_->rotateLeft(navigation::speed);
    }

    while (elapsedTimeMs <= timeoutDurationMs && robot_->noSensorsActive()) {
        _delay_ms(10);
        elapsedTimeMs += 10;
    }
    robot_->haltMotor();
    _delay_ms(500);
}

bool SearchEndMode::followLineForTimeout(uint16_t timeoutDurationMs) {
    uint16_t elapsedTimeMs = 0;

    while (elapsedTimeMs <= timeoutDurationMs && !robot_->noSensorsActive()) {
        followLine();
        _delay_ms(10);
        elapsedTimeMs += 10;
    }
    return !robot_->noSensorsActive();
}

void SearchEndMode::fullIntersection() { /////////////////////////////////////////////

    incrementIntersections();

    if(nbsOfintersections_ == 3){
        robot_->driveBackward(80);
        while (robot_->noSensorsActive());
        robot_->haltMotor();

        processInferredData();
        writeCoordInMemory();
        playConfirmationSound();
        turnLedBaseOnExtremity();

        while(true); //block code
    }

    if(nbsOfintersections_ == 2) {
        robot_->moveForward(navigation::speed);
        _delay_ms(1750);
        robot_->haltMotor();
        _delay_ms(1000);

        rotateAndCheckSensors(true, 2100);
        if (followLineForTimeout(750)) {
            fullIntersectionRight_ = true;
            lastDirection_ = LastDirection::RIGHT;
            return; //sortir
        }

        rotateAndCheckSensors(false, 4200); 
        if (followLineForTimeout(750)) {
            fullIntersectionLeft_ = true;
            lastDirection_ = LastDirection::LEFT;
            return; //sortir
        }

        rotateAndCheckSensors(true, 2100);

        robot_->driveBackward(80);
        while (robot_->noSensorsActive());
        robot_->haltMotor();

        processInferredData();
        writeCoordInMemory();
        playConfirmationSound();
        turnLedBaseOnExtremity();

        while(true); //block the code
    }

    if(nbsOfintersections_ == 1){
        robot_->moveForward(navigation::speed);
        _delay_ms(1750);
        robot_->haltMotor();
        _delay_ms(1000);
        rotateAndCheckSensors(true, 2100);
        if (followLineForTimeout(750)) {
            fullIntersectionRight_ = true;
            lastDirection_ = LastDirection::RIGHT;
            return; //sortir
        }

        rotateAndCheckSensors(false, 4200); 
        if (followLineForTimeout(750)) {
            fullIntersectionLeft_ = true;
            lastDirection_ = LastDirection::LEFT;
            return; //sortir
        }
    }
}

/*
 * The navigate function is the primary control loop for the robot's navigation. 
 * It continuously monitors sensor inputs to determine the robot's next action, 
 * handling intersections, path-following, and course corrections. The function operates as follows:
 * 
 * 1. Continuously checks the sensors in an infinite loop to determine the robot's state:
 *    - If the far-right sensor is active:
 *        * The robot halts briefly to stabilize.
 *        * Calls ajustVerifieRight() to determine the intersection type.
 *        * Handles PARTIAL intersections by calling partialIntersection(true).
 *        * Handles FULL intersections by moving forward until the far-left and 
 *          far-right sensors are inactive, and then calls fullIntersection().
 *    - If the far-left sensor is active:
 *        * The robot halts briefly to stabilize.
 *        * Calls ajustVerifieLeft() to determine the intersection type.
 *        * Handles PARTIAL intersections by calling partialIntersection(false).
 *        * Handles FULL intersections similarly to the right sensor case, 
 *          moving forward and calling fullIntersection().
 *    - If no sensors are active:
 *        * The robot halts and initiates a right rotation until a sensor detects a line.
 *    - If none of the above conditions are met, the robot defaults to following the line.
 * 
 * 2. The function ensures the robot navigates the path effectively by handling all 
 *    scenarios dynamically based on sensor inputs, while continuously adjusting 
 *    for intersections and lost paths.
 */

void SearchEndMode::handleFirst(){
    if (robot_->isSensorActive(SensorType::FAR_RIGHT)) {
        robot_->haltMotor();
        _delay_ms(500);
        
        IntersectionType intersectionType = ajustVerifieRight();
        if (intersectionType == IntersectionType::PARTIAL){
            partialIntersection(true);
        }
        else if (intersectionType == IntersectionType::FULL) {
            robot_->moveForward(250);
            _delay_ms(50);  
            robot_->moveForward(navigation::forwardSpeed);

            while (robot_->isSensorActive(LineSensor::FAR_LEFT) || robot_->isSensorActive(LineSensor::FAR_RIGHT)){}

            _delay_ms(120);

            if (robot_->noSensorsActive()){
                fullIntersection();
            }
        }
    }
    else if (robot_->isSensorActive(SensorType::FAR_LEFT)) {
        robot_->haltMotor();
        _delay_ms(500);

        IntersectionType typeOfIntersection = ajustVerifieLeft();
        if (typeOfIntersection == IntersectionType::PARTIAL){
            partialIntersection(false);
        }
        else if (typeOfIntersection == IntersectionType::FULL){
            robot_->moveForward(250);
            _delay_ms(50);
            robot_->moveForward(navigation::forwardSpeed);

            while (robot_->isSensorActive(LineSensor::FAR_LEFT) || robot_->isSensorActive(LineSensor::FAR_RIGHT)){}

            _delay_ms(120);

            if (robot_->noSensorsActive()){
                fullIntersection();                 
            }
        }
    }
    else if (robot_->noSensorsActive()){
        robot_->haltMotor();
        robot_->rotateRight(navigation::rightMotorSpeed);

        while (robot_->noSensorsActive()){}            
    }       
    else {
        followLine();
    }
}

void SearchEndMode::handleSecond(){
    if (robot_->isSensorActive(SensorType::FAR_RIGHT)) {
        robot_->haltMotor();
        _delay_ms(500);
        
        IntersectionType intersectionType = ajustVerifieRight();
        if (intersectionType == IntersectionType::PARTIAL){
            partialIntersection(true);
        }
        else if (intersectionType == IntersectionType::FULL) {
            robot_->moveForward(250);
            _delay_ms(50);  
            robot_->moveForward(navigation::forwardSpeed);

            while (robot_->isSensorActive(LineSensor::FAR_LEFT) || robot_->isSensorActive(LineSensor::FAR_RIGHT)){}

            _delay_ms(120);

            if (robot_->noSensorsActive()){
                fullIntersection();
            }
        }
    }
    else if (robot_->isSensorActive(SensorType::FAR_LEFT)) {
        robot_->haltMotor();
        _delay_ms(500);

        IntersectionType typeOfIntersection = ajustVerifieLeft();
        if (typeOfIntersection == IntersectionType::PARTIAL){
            partialIntersection(false);
        }
        else if (typeOfIntersection == IntersectionType::FULL){
            robot_->moveForward(250);
            _delay_ms(50);
            robot_->moveForward(navigation::forwardSpeed);

            while (robot_->isSensorActive(LineSensor::FAR_LEFT) || robot_->isSensorActive(LineSensor::FAR_RIGHT)){}

            _delay_ms(120);

            if (robot_->noSensorsActive()){
                fullIntersection();                 
            }
        }
    }
    else if (robot_->noSensorsActive()){
        robot_->haltMotor();
        robot_->rotateRight(navigation::rightMotorSpeed);

        while (robot_->noSensorsActive()){}            
    }       
    else {
        followLine();
    }
}

void SearchEndMode::handleThird(){
    if (robot_->isSensorActive(SensorType::FAR_RIGHT)) {
        robot_->haltMotor();
        _delay_ms(500);
        
        IntersectionType intersectionType = ajustVerifieRight();
        if (intersectionType == IntersectionType::PARTIAL){
            partialIntersection(true);
        }
        else if (intersectionType == IntersectionType::FULL) {
            robot_->moveForward(250);
            _delay_ms(50);  
            robot_->moveForward(navigation::forwardSpeed);

            while (robot_->isSensorActive(LineSensor::FAR_LEFT) || robot_->isSensorActive(LineSensor::FAR_RIGHT)){}

            _delay_ms(120);

            if (robot_->noSensorsActive()){
                fullIntersection();
            }
        }
    }
    else if (robot_->isSensorActive(SensorType::FAR_LEFT)) {
        robot_->haltMotor();
        _delay_ms(500);

        IntersectionType typeOfIntersection = ajustVerifieLeft();
        if (typeOfIntersection == IntersectionType::PARTIAL){
            partialIntersection(false);
        }
        else if (typeOfIntersection == IntersectionType::FULL){
            robot_->moveForward(250);
            _delay_ms(50);
            robot_->moveForward(navigation::forwardSpeed);

            while (robot_->isSensorActive(LineSensor::FAR_LEFT) || robot_->isSensorActive(LineSensor::FAR_RIGHT)){}

            _delay_ms(120);

            if (robot_->noSensorsActive()){
                fullIntersection();                 
            }
        }
    }
    else if (robot_->noSensorsActive()){
        robot_->haltMotor();
        robot_->rotateRight(navigation::rightMotorSpeed);

        while (robot_->noSensorsActive()){}            
    }       
    else {
        followLine();
    }
}

void SearchEndMode::navigate() {
    _delay_ms(2000);
    while (true) {
        if(nbsOfintersections_ == 0){
            handleFirst();
        } else if(nbsOfintersections_ == 1){
            handleSecond();
        } else if(nbsOfintersections_ == 2){
            handleThird();
        }
    }
}