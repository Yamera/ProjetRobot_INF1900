#pragma once

#include <stdint.h>

#include "mode_strategy.h"

class ShowReportMode : public ModeStrategy {
public:
    explicit ShowReportMode(Robot* robot) : ModeStrategy(robot) {}
    void execute() override;

private:
    void generateReport(uint8_t* data, char* buffer, uint16_t bufferSize) const;
    void clignotement();
};
