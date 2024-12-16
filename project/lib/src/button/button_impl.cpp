#include "button_impl.h"

const uint8_t ButtonImpl::iscBitMask[3][3] = {
    {ISC00, ISC01, ISC01 | ISC00},
    {ISC11, ISC10, ISC10 | ISC11},
    {ISC20, ISC21, ISC20 | ISC21}
};

void ButtonImpl::setConditionMask(Source source, TriggerCondition triggerCondition) {
    uint8_t sourceIndex = static_cast<uint8_t>(source);
    uint8_t conditionIndex = static_cast<uint8_t>(triggerCondition);
    EICRA |= (1 << iscBitMask[sourceIndex][conditionIndex]);
}

void ButtonImpl::configureSource(Source source) {
    switch (source) {
        case Source::INT_0:
            EIMSK |= (1 << INT0);
            break;
        case Source::INT_1:
            EIMSK |= (1 << INT1);
            break;
        case Source::INT_2:
            EIMSK |= (1 << INT2);
            break;
    }
}

void ButtonImpl::setupIO(Source source) {
    switch (source) {
        case Source::INT_0:
            BUTTON_REGISTER_INT_0_1 &= ~(1 << BUTTON_INT_0_PIN);
            break;
        case Source::INT_1:
            BUTTON_REGISTER_INT_0_1 &= ~(1 << BUTTON_INT_1_PIN);
            break;
        case Source::INT_2:
            BUTTON_REGISTER_INT_2 &= ~(1 << BUTTON_INT_2_PIN);
            break;
    }
}

void ButtonImpl::setPressButtonCallback(ButtonEventCallback callback) {
    pressButtonCallback_ = callback;
}

void ButtonImpl::resetButtonState(Source source) {
        switch (source) {
            case Source::INT_0:
                EIFR |= (1 << INTF0);
                break;
            case Source::INT_1:
                EIFR |= (1 << INTF1);
                break;
            case Source::INT_2:
                EIFR |= (1 << INTF2);
                break;
        }
}
