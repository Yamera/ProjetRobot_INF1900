/**
 * @class ButtonExtern
 * @brief Singleton class that manages the configuration and event handling 
 *        of an external button.
 *
 * The `ButtonExtern` class provides a centralized interface for configuring 
 * and handling events related to an external button. It follows the singleton 
 * pattern to ensure only one instance is created. This class manages the setup 
 * and interaction with a `ButtonImpl` object, which represents the actual 
 * implementation of the button’s functionality.
 *
 * @enum Source
 * Defines the external interrupt sources for the button, each mapped to a 
 * specific microcontroller pin:
 * - `INT_0`: Corresponds to pin **PD2**
 * - `INT_1`: Corresponds to pin **PD3**
 * - `INT_2`: Corresponds to pin **PB2**
 *
 * @details
 * - `configureButton`: Configures the button with a specified trigger condition 
 *   and event callback. Optionally, it accepts a source for the button signal, 
 *   defaulting to `Source::INT_1`.
 * - `callEventFunction`: Invokes the callback function associated with the 
 *   button event, allowing external actions to be triggered.
 * - This implementation uses the following components:
 *   - A white push-button: **Digi-Key: EG1328-ND**
 *   - A 0.1 µF capacitor (gray or blue) for power regulation: 
 *     **Digi-Key: BC1621-ND**
 *   - A 100K resistor (brown-black-yellow): **Digi-Key: S100KQTR-ND**
 *
 * @authors
 * - Ahmed Sami Benabbou
 * - Maroua Lassakeur
 * - Mohamed-Borheneddine Mokaddem
 * - Yasmine Meraoubi
 */

#pragma once

#include "button_impl.h"

class ButtonExtern {
public:
    static ButtonExtern& getInstance() {
        static ButtonExtern instance;
        return instance;
    }

    void configureButton(
        TriggerCondition triggerCondition, 
        ButtonEventCallback callback, 
        Source source = Source::INT_1
    );

    void callEventFunction();

    ~ButtonExtern() = default;

private:
    Source source_;
    ButtonImpl buttonImpl_;

    ButtonExtern() = default;
    ButtonExtern(const ButtonExtern&) = delete; 
    ButtonExtern& operator=(const ButtonExtern&) = delete; 
};
