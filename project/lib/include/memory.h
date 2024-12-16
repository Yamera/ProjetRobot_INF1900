#pragma once

#include <avr/io.h>
#include <util/twi.h>

class Memory {
public:
   static Memory& getInstance() {
       static Memory instance; 
       return instance;
   }

   uint8_t read(const uint16_t address, uint8_t *data);
   uint8_t read(const uint16_t address, uint8_t *data,
                const uint8_t length);

   uint8_t write(const uint16_t address, const uint8_t data);
   uint8_t write(const uint16_t address, uint8_t *data,
                 const uint8_t length);

private:
   Memory();
   ~Memory() = default;

   void init();
   static uint8_t selectBank(const uint8_t bank);

   Memory(const Memory&) = delete;
   Memory& operator=(const Memory&) = delete;

   uint8_t writePage(const uint16_t address, uint8_t *data,
                      const uint8_t length);

private:
   static uint8_t deviceAddress_;
   const uint8_t pageSize_;
};
