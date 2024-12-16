#include "memory.h"

uint8_t Memory::deviceAddress_ = 0xA0;

Memory::Memory()
:  pageSize_(128) {
   init();
}

void Memory::init() {
   selectBank(0);
   
   TWSR = 0;
   
   TWBR =  (F_CPU / 100000UL - 16) / 2;
}

uint8_t Memory::selectBank(const uint8_t bank) {
   uint8_t temp = bank & 0x03;
   uint8_t rv = 255;
   if(bank == temp) {
      Memory::deviceAddress_ = (0xA0 | ( bank << 1 ));
      rv = Memory::deviceAddress_;
   }
   return rv;
}

uint8_t Memory::read(const uint16_t adresse, uint8_t *data) {
            uint8_t rv = 0;


   for (;;) {
      TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);    
      while ((TWCR & _BV(TWINT)) == 0)   
         ;

      TWDR = deviceAddress_;    
      TWCR = _BV(TWINT) | _BV(TWEN);     
      while ((TWCR & _BV(TWINT)) == 0)   
         ;
      if (TWSR==0x18)         
         break;
   }

   TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);     
   while ((TWCR & _BV(TWINT)) == 0)      
      ;

   TWDR = deviceAddress_;       
   TWCR = _BV(TWINT) | _BV(TWEN);       
   while ((TWCR & _BV(TWINT)) == 0)     
      ;

   TWDR =  ( adresse >> 8 );            
   TWCR = _BV(TWINT) | _BV(TWEN);       
   while ((TWCR & _BV(TWINT)) == 0)     
      ;

   TWDR = adresse;                      
   TWCR = _BV(TWINT) | _BV(TWEN);       
   while ((TWCR & _BV(TWINT)) == 0)     
      ;


   while ((TWCR & _BV(TWINT)) == 0)      
      ;
   TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
   while ((TWCR & _BV(TWINT)) == 0)      
      ;

   TWDR =  deviceAddress_ + 1;   
   TWCR = _BV(TWINT) | _BV(TWEN);        
   while ((TWCR & _BV(TWINT)) == 0)      
      ;

   TWCR = _BV(TWINT) | _BV(TWEN);     
   while ((TWCR & _BV(TWINT)) == 0)   
      ;
   *data = TWDR;

   TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
   return rv;
}


uint8_t Memory::read(const uint16_t adresse, uint8_t *data,
                              uint8_t length) {
   uint8_t twcr;

   for (;;) {
      TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);    
      while ((TWCR & _BV(TWINT)) == 0) ;   

      TWDR = deviceAddress_;       
      TWCR = _BV(TWINT) | _BV(TWEN);       
      while ((TWCR & _BV(TWINT)) == 0)     
         ;
      if (TWSR==0x18)                      
         break;
   }

   TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);     
   while ((TWCR & _BV(TWINT)) == 0)    
      ;

   TWDR = deviceAddress_;        
   TWCR = _BV(TWINT) | _BV(TWEN);        
   while ((TWCR & _BV(TWINT)) == 0)      
      ;

   TWDR = adresse >> 8;                  
   TWCR = _BV(TWINT) | _BV(TWEN);        
   while ((TWCR & _BV(TWINT)) == 0)      
      ;

   TWDR = adresse;                       
   TWCR = _BV(TWINT) | _BV(TWEN);        
   while ((TWCR & _BV(TWINT)) == 0)      
      ;

   TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);    
   while ((TWCR & _BV(TWINT)) == 0)      
      ;

   TWDR =  deviceAddress_ + 1;  
   TWCR = _BV(TWINT) | _BV(TWEN);       
   while ((TWCR & _BV(TWINT)) == 0)     
      ;

   for (twcr = _BV(TWINT) | _BV(TWEN) | _BV(TWEA) ; length > 0; length--) {
         if (length == 1)
            twcr = _BV(TWINT) | _BV(TWEN);  
         TWCR = twcr;                       
         while ((TWCR & _BV(TWINT)) == 0) ; 
      *data++ = TWDR;               
   }

   TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);

   return 0;
}

uint8_t Memory::write(const uint16_t adresse, const uint8_t data) {

   for ( ; ; ) {
      TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);    
      while ((TWCR & _BV(TWINT)) == 0)      
         ;

      TWDR = deviceAddress_;       
      TWCR = _BV(TWINT) | _BV(TWEN);       
      while ((TWCR & _BV(TWINT)) == 0)     
         ;

      if (TWSR==0x18)
         break;               
   }


   TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);     
   while ((TWCR & _BV(TWINT)) == 0)    
      ;

   TWDR = deviceAddress_;        
   TWCR = _BV(TWINT) | _BV(TWEN);        
   while ((TWCR & _BV(TWINT)) == 0)      
      ;

   TWDR = adresse >> 8;                 
   TWCR = _BV(TWINT) | _BV(TWEN);        
   while ((TWCR & _BV(TWINT)) == 0)      
      ;

   TWDR = adresse;                      
   TWCR = _BV(TWINT) | _BV(TWEN);        
   while ((TWCR & _BV(TWINT)) == 0)      
      ;

   TWDR = data;
   TWCR = _BV(TWINT) | _BV(TWEN);        
   while ((TWCR & _BV(TWINT)) == 0)      
      ;

   TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);  

   return 0;
}

uint8_t Memory::write(const uint16_t adresse, uint8_t *donnee,
                              const uint8_t longueur) {
   uint8_t rv;
   uint16_t copieAdresse = adresse;
   uint8_t copieLongueur = longueur;
   do {
         rv = writePage(copieAdresse, donnee, copieLongueur);
         copieAdresse += rv;      
         copieLongueur -= rv;     
         donnee += rv;            
   }
   while (copieLongueur > 0);

   return 0;
}

uint8_t Memory::writePage(const uint16_t adresse, uint8_t *data,
                                 const uint8_t length) {
   uint16_t addr_fin;
   uint8_t rv = 0;
   uint8_t copieLongueur = length;

   if (adresse + length < (adresse | (pageSize_ - 1)))
      addr_fin = adresse + length;
   else
      addr_fin = (adresse | (pageSize_ - 1)) + 1;
   copieLongueur = addr_fin - adresse;

   for ( ; ; ) {
      TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);    
      while ((TWCR & _BV(TWINT)) == 0);   
      TWDR = deviceAddress_;       
      TWCR = _BV(TWINT) | _BV(TWEN);       
      while ((TWCR & _BV(TWINT)) == 0)     
      ;

      if (TWSR==0x18)
         break;               
   }

   TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);     
   while ((TWCR & _BV(TWINT)) == 0)       
      ;

   TWDR = deviceAddress_;        
   TWCR = _BV(TWINT) | _BV(TWEN);        
   while ((TWCR & _BV(TWINT)) == 0)      
   ;

   TWDR = adresse >> 8;                  
   TWCR = _BV(TWINT) | _BV(TWEN);        
   while ((TWCR & _BV(TWINT)) == 0)      
      ;

   TWDR = adresse;                       
   TWCR = _BV(TWINT) | _BV(TWEN);        
   while ((TWCR & _BV(TWINT)) == 0)      
      ;

   for ( ; copieLongueur > 0; copieLongueur--) {
      TWDR = *data++;
      TWCR = _BV(TWINT) | _BV(TWEN);     
      while ((TWCR & _BV(TWINT)) == 0)   
         ;
      rv++;                              
   }

   TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); 

   return rv;
}
