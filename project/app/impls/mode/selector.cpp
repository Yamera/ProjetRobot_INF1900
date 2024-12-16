#include "../defs/mode/selector.h"
#include "debug/debug_uart.h"

ModeSelector::ModeSelector(Robot* robot) 
    : robot_(robot), searchEndMode_(robot), crossCourseMode_(robot), showReportMode_(robot) {}


void ModeSelector::selectAndExecuteMode() {
    Mode currentMode = readModeFromEEPROM();

    if (currentMode == Mode::SELECTION) {
        selectMode();
    } else {
        executeMode(currentMode);
    }
}

void ModeSelector::selectMode() {
    while (true) {
        selectSearchEndMode();
        selectCrossCourseMode();
        selectShowReportMode();
    }
}

void ModeSelector::selectSearchEndMode() {
    robot_->turnLedGreen();
    robot_->writeMemory(MODE_ADDRESS, Mode::SEARCH_END);
    DEBUG_PRINT_STR((ModeDebug::selection_search_end));
    _delay_ms(MODE_DELAY_MS);
}

void ModeSelector::selectCrossCourseMode() {
    robot_->turnLedRed();
    robot_->writeMemory(MODE_ADDRESS, Mode::CROSS_COURSE);
    DEBUG_PRINT_STR((ModeDebug::selection_cross_course));
    _delay_ms(MODE_DELAY_MS);
}

void ModeSelector::selectShowReportMode() {
    robot_->writeMemory(MODE_ADDRESS, Mode::SHOW_REPORT);
    DEBUG_PRINT_STR((ModeDebug::selection_show_report));
    for(uint8_t i = 0; i < 100; i++) {
        robot_->turnLedAmber();
    }
}

void ModeSelector::executeMode(Mode mode) {
    resetModeInMemoryToSelection();

    switch (mode) {
        case Mode::SEARCH_END:
            DEBUG_PRINT_STR((ModeDebug::search_end));
            searchEndMode_.execute();
            break;
        case Mode::CROSS_COURSE:
            DEBUG_PRINT_STR((ModeDebug::cross_course));
            crossCourseMode_.execute();
            break;
        case Mode::SHOW_REPORT:
            DEBUG_PRINT_STR((ModeDebug::show_report));
            showReportMode_.execute();
            break;
        default:
            DEBUG_PRINT_STR((ModeDebug::error));
            break;
    }
}

ModeSelector::Mode ModeSelector::readModeFromEEPROM() {
    uint8_t modeData;
    robot_->readMemory(MODE_ADDRESS, &modeData);

    switch (modeData) {
        case Mode::SEARCH_END:
            return Mode::SEARCH_END;
        case Mode::CROSS_COURSE:
            return Mode::CROSS_COURSE;
        case Mode::SHOW_REPORT:
            return Mode::SHOW_REPORT;
        default:
            DEBUG_PRINT_STR((ModeDebug::error));
            return Mode::SELECTION;
    }
}

void ModeSelector::resetModeInMemoryToSelection() {
    robot_->writeMemory(MODE_ADDRESS, Mode::SELECTION);
}

