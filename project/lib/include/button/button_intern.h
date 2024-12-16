/**
 * @class ButtonIntern
 * @brief Singleton class that manages the configuration and event handling 
 *        of an internal button.
 *
 * The `ButtonIntern` class provides a centralized interface for configuring
 * and handling events related to an internal button. It follows the singleton 
 * pattern to ensure only one instance is created. This class manages the setup 
 * and interaction with a `ButtonImpl` object, which represents the actual 
 * implementation of the button’s functionality.
 *
 * @enum Source
 * Defines the external interrupt sources for the button, each mapped to a 
 * specific microcontroller pin:
 * - `INT_0`: Corresponds to pin **PD2** (the only available option due 
 * to hardware limitations).
 *
 * @details
 * - `configureButton`: Configures the button with a specified trigger condition
 *   and event callback.
 *   The button is fixed to use `Source::INT_0` due to hardware limitations.
 * - `callEventFunction`: Invokes the callback function associated with the 
 *   button event, allowing external actions to be triggered.
 * - This implementation uses the following component:
 *   - **Digi-Key: SW401-ND** (internal button switch).
 *
 * @authors
 * - Ahmed Sami Benabbou
 * - Maroua Lassakeur
 * - Mohamed-Borheneddine Mokaddem
 * - Yasmine Meraoubi
 */

#pragma once

#include "button_impl.h"

class ButtonIntern {
public:
    static ButtonIntern& getInstance() {
        static ButtonIntern instance;
        return instance;
    }

    void configureButton(TriggerCondition triggerCondition, ButtonEventCallback callback);
    void callEventFunction();

    ~ButtonIntern() = default;

private:
    Source source_{Source::INT_0}; 
    ButtonImpl buttonImpl_;
    ButtonEventCallback pressButtonCallback_ = nullptr; 

    ButtonIntern() = default; 
    
    ButtonIntern(const ButtonIntern&) = delete; 
    ButtonIntern& operator=(const ButtonIntern&) = delete;  
};
