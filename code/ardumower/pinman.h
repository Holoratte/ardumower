// pin manager
// replacement for Arduino wiring, allowing us to change PWM frequency

#ifndef PINMAN_H
#define PINMAN_H

#include <Arduino.h>

class PinManager {
  public:  
    void begin();
	  void analogWrite25k( uint8_t uPin, uint8_t uValue ) ;  
};

extern PinManager PinMan;

#endif 

