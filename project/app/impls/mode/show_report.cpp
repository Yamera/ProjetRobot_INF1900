#include <stdio.h>

#include "../defs/mode/show_report.h"

void ShowReportMode::execute() {
    clignotement();

    //test:
    // robot_->writeMemory(0x0001, static_cast<uint8_t>(DepartureNode::A));
    // _delay_ms(200);
    // robot_->writeMemory(0x0002, static_cast<uint8_t>(Orientation::NORTH_EAST));
    // _delay_ms(200);
    // robot_->writeMemory(0x0003, static_cast<uint8_t>(ExtremityNode::C));
    // _delay_ms(200);
    // robot_->writeMemory(0x0004, static_cast<uint8_t>(Node::ONE));
    // _delay_ms(200);
    // robot_->writeMemory(0x0005, static_cast<uint8_t>(Node::TWO));
    // _delay_ms(200);
    // robot_->writeMemory(0x0006, static_cast<uint8_t>(Node::THREE));
    // _delay_ms(200);
    // robot_->writeMemory(0x0007, static_cast<uint8_t>(Node::FOUR));
    // _delay_ms(200);
    // robot_->writeMemory(0x0008, static_cast<uint8_t>(Node::FIVE));
    // _delay_ms(200);
    
    uint8_t data[configRapport::dataSize];
    robot_->readMemory(configRapport::startAddress, data, sizeof(data));

    char rapport[configRapport::reportSize];
    generateReport(data, rapport, sizeof(rapport)); 
    
    robot_->transmitData(rapport);
}

void ShowReportMode::generateReport(uint8_t* data, char* reportBuffer, uint16_t bufferSize) const {
    snprintf(reportBuffer, bufferSize,
        "Identification de l’extrémité\n"
        "-----------------------------\n"
        "Point de départ : %c\n"
        "Orientation de départ : %s\n"
        "Extrémité trouvée : %c\n\n"
        "Traversée du parcours\n"
        "---------------------\n"
        "Point de départ : %c\n"
        "Point du poteau 1 : %c\n"
        "Point du poteau 2 : %c\n"
        "Point du poteau 3 : %c\n\n"
        "Numéro d’équipe 6170 - Wall-e",
        static_cast<char>(data[0]),                     // Starting point
        toString(static_cast<Orientation>(data[1])),    // Starting orientation
        static_cast<char>(data[2]),                     // Endpoint identified
        static_cast<char>(data[3]),                     // Path starting point
        static_cast<char>(data[4]),                     // Pole 1 position
        static_cast<char>(data[5]),                     // Pole 2 position
        static_cast<char>(data[6])                      // Pole 3 position
    );
}

void ShowReportMode::clignotement() {
    for (uint8_t i = 0; i < configClignotement::totalBlinks; i++) {
        for(uint8_t j = 0; j < 6; j++)
            robot_->turnLedAmber();
        robot_->turnLedOff();
        _delay_ms(configClignotement::blinkIntervalMs);
    }
}