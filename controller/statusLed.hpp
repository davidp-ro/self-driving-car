#ifndef STATUS_LED
#define STATUS_LED

#include "Arduino.h"
#define RED A3
#define GREEN A4
#define BLUE A5

void _allOff();
void _blink();
void setStatus(String status, uint16_t blinks);

#endif // STATUS_LED
