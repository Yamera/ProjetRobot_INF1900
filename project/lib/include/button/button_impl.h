/**
 * @class ButtonImpl
 * @brief Class that implements the button's functionality, 
 *        including configuration, triggering, and event handling.
 *
 * The `ButtonImpl` class provides methods for configuring an external 
 * button with different trigger conditions, handling button presses, 
 * and executing callback functions when events occur. It manages the 
 * setup of interrupt sources for the button, such as external interrupts 
 * (INT_0, INT_1, and INT_2), and provides debouncing functionality 
 * to ensure stable event triggering.
 *
 * @enum TriggerCondition
 * Defines the possible trigger conditions for the button press:
 * - `BOTH_EDGES`: Triggered on both rising and falling edges of the signal.
 * - `FALLING_EDGE`: Triggered when the signal transitions from high to low.
 * - `RISING_EDGE`: Triggered when the signal transitions from low to high.
 *
 * @enum Source
 * Defines the external interrupt sources for the button, each mapped to 
 * a specific microcontroller pin:
 * - `INT_0`: Corresponds to pin **PD2**.
 * - `INT_1`: Corresponds to pin **PD3**.
 * - `INT_2`: Corresponds to pin **PB2**.
 *
 * This implementation does not use any external components, aside from 
 * the microcontroller’s own input pins.
 *
 * @note Hardware limitations may restrict the use of certain interrupt 
 *       sources, depending on the microcontroller.
 *
 * @authors
 * - Ahmed Sami Benabbou
 * - Maroua Lassakeur
 * - Mohamed-Borheneddine Mokaddem
 * - Yasmine Meraoubi
 */

#pragma once

#include <util/delay.h>

class ButtonImpl {
public:
    enum TriggerCondition {
        BOTH_EDGES,
        FALLING_EDGE,
        RISING_EDGE
    };

    enum Source {
        INT_0,  
        INT_1, 
        INT_2   
    };

    using ButtonEventCallback = void (*)();

private:
    static void setConditionMask(Source source, TriggerCondition triggerCondition);
    static void configureSource(Source source);
    static void setupIO(Source source);

    void setPressButtonCallback(ButtonEventCallback callback);
    
    static inline void debounceDelay() { 
        _delay_ms(debounceMs); 
    }

    void resetButtonState(Source source);

    inline void callEventFunction(Source source) {
        debounceDelay();
        if (pressButtonCallback_) {
            pressButtonCallback_();
        }
        resetButtonState(source);
    } 



private:
    static const uint8_t iscBitMask[3][3];
    ButtonEventCallback pressButtonCallback_ = nullptr;
    static constexpr uint8_t debounceMs = 30;

    ButtonImpl() = default;
    ~ButtonImpl() = default; 

    friend class ButtonExtern;
    friend class ButtonIntern;
};

using Button = ButtonImpl;
using Source = ButtonImpl::Source;
using TriggerCondition = ButtonImpl::TriggerCondition;
using ButtonEventCallback = ButtonImpl::ButtonEventCallback;
