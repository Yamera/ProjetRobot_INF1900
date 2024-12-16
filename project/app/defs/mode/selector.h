#pragma once

#include "mode_strategy.h"
#include "search_end.h"
#include "show_report.h"
#include "cross_course.h"

#include "../robot.h"

#include "debug/debug_uart.h"

namespace ModeDebug {
    constexpr char selection[] = "selection\n";
    constexpr char search_end[] = "search_end\n";
    constexpr char cross_course[] = "cross_course\n";
    constexpr char show_report[] = "show_report\n";    
    constexpr char error[] = "error\n";  

    constexpr char selection_search_end[] = "selection_search_end\n";
    constexpr char selection_cross_course[] = "selection_cross_course\n";
    constexpr char selection_show_report[] = "selection_show_report\n";       
};

class ModeSelector {
public:
    enum Mode: uint8_t {
        SELECTION = 0x00,
        SEARCH_END = 0x01,      
        CROSS_COURSE = 0x02,     
        SHOW_REPORT = 0x03              
    };
    
public:
    explicit ModeSelector(Robot* robot);
    void selectAndExecuteMode();
    
private:
    void selectMode();
    void executeMode(Mode mode);
    Mode readModeFromEEPROM();
    void resetModeInMemoryToSelection();
    void selectSearchEndMode();
    void selectCrossCourseMode();
    void selectShowReportMode();

private:
    static constexpr uint16_t MODE_ADDRESS = 0x0000;
    static constexpr uint16_t MODE_DELAY_MS = 2000;

private:
    Robot* robot_;

    SearchEndMode searchEndMode_;
    CrossCourseMode crossCourseMode_;
    ShowReportMode showReportMode_;
};

