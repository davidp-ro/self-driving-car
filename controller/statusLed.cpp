#include "Arduino.h"
#include "statusLed.hpp"

void _allOff() {
    digitalWrite(RED, LOW);
    digitalWrite(GREEN, LOW);
    digitalWrite(BLUE, LOW);
}

void _blink() {
    digitalWrite(RED, HIGH);
    digitalWrite(GREEN, HIGH);
    delay(500);
    _allOff();
}

void setStatus(String status, uint16_t blinks) {
    _allOff();
    if (status == "init") {
        digitalWrite(BLUE, HIGH);
    } else if (status == "fail") {
        digitalWrite(RED, HIGH);
    } else if (status == "ok") {
        digitalWrite(GREEN, HIGH);
    } else {
        while (blinks) {
            _blink();
            blinks--;
        }
    }
}