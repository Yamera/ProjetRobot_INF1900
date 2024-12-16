/**
 * @namespace debug
 * @brief Provides UART-based debugging functions.
 *
 * The `debug` namespace includes functions to send strings and integers 
 * over UART for debugging purposes. The functions are enabled or disabled 
 * based on the `DEBUG` macro.
 *
 * - `printDebugStr`: Sends a string over UART.
 * - `printDebugInt`: Sends an integer value over UART.
 *
 * If `DEBUG` is defined, the `DEBUG_PRINT_STR` and `DEBUG_PRINT_INT` 
 * macros call these functions. If `DEBUG` is not defined, the macros do nothing.
 * 
 * @authors
 * - Ahmed Sami Benabbou
 * - Maroua Lassakeur
 * - Mohamed-Borheneddine Mokaddem
 * - Yasmine Meraoubi
 */

#pragma once

#include <string.h>
#include <stdlib.h>

#include "uart.h"

//remove this after
#define DEBUG

namespace debug {
    inline void printDebugStr(const char* string, uint8_t length = 0) {
        Uart& uart = Uart::getInstance();
        uart.transmit(string, length ? length : strlen(string));
    }

    inline void printDebugInt(uint16_t value) {
        Uart& uart = Uart::getInstance();
        uart.transmitChar(value);
    }
}

#ifdef DEBUG

#define DEBUG_PRINT_STR(x)  debug::printDebugStr x
#define DEBUG_PRINT_INT(x)  debug::printDebugInt x
#else
#define DEBUG_PRINT_STR(x) do {} while (false) 
#define DEBUG_PRINT_INT(x) do {} while (false)

#endif
