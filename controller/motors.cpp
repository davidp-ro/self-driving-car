/**
 * @file motors.cpp
 * @author David Pescariu | https://github.com/davidp-ro
 * @brief Implementations for the prototipes in motors.hpp
 * @version 1.1
 * @date 2020-11-28
 *
 * @copyright GPL-3.0 License
 */

#include "motors.hpp"

#include <Arduino.h>

void initPins(Pins pins) {
    pinMode(pins.PWM_LEFT, OUTPUT);
    pinMode(pins.PWM_RIGHT, OUTPUT);
    pinMode(pins.LEFT_FWD, OUTPUT);
    pinMode(pins.LEFT_RWD, OUTPUT);
    pinMode(pins.RIGHT_FWD, OUTPUT);
    pinMode(pins.RIGHT_RWD, OUTPUT);
    stop(pins);
}

void goForward(Pins pins, uint8_t speed) {
    analogWrite(pins.PWM_LEFT, speed);
    analogWrite(pins.PWM_RIGHT, speed);
    digitalWrite(pins.LEFT_FWD, HIGH);
    digitalWrite(pins.LEFT_RWD, LOW);
    digitalWrite(pins.RIGHT_FWD, HIGH);
    digitalWrite(pins.RIGHT_RWD, LOW);
}

void stop(Pins pins) {
    analogWrite(pins.PWM_LEFT, 0);
    analogWrite(pins.PWM_RIGHT, 0);
    digitalWrite(pins.LEFT_FWD, LOW);
    digitalWrite(pins.LEFT_RWD, LOW);
    digitalWrite(pins.RIGHT_FWD, LOW);
    digitalWrite(pins.RIGHT_RWD, LOW);
}

void reverse(Pins pins) {
    analogWrite(pins.PWM_LEFT, SPEED_MED);
    analogWrite(pins.PWM_RIGHT, SPEED_MED);
    digitalWrite(pins.LEFT_FWD, LOW);
    digitalWrite(pins.LEFT_RWD, HIGH);
    digitalWrite(pins.RIGHT_FWD, LOW);
    digitalWrite(pins.RIGHT_RWD, HIGH);
}

void turnSlightlyLeft(Pins pins) {
    analogWrite(pins.PWM_LEFT, SPEED_SLOW);
    analogWrite(pins.PWM_RIGHT, SPEED_FAST);
    digitalWrite(pins.LEFT_FWD, HIGH);
    digitalWrite(pins.LEFT_RWD, LOW);
    digitalWrite(pins.RIGHT_FWD, HIGH);
    digitalWrite(pins.RIGHT_RWD, LOW);
}

void turnLeft(Pins pins) {
    analogWrite(pins.PWM_LEFT, SPEED_FAST);
    analogWrite(pins.PWM_RIGHT, SPEED_FAST);
    digitalWrite(pins.LEFT_FWD, LOW);
    digitalWrite(pins.LEFT_RWD, HIGH);
    digitalWrite(pins.RIGHT_FWD, HIGH);
    digitalWrite(pins.RIGHT_RWD, LOW);
}

void turnSlightlyRight(Pins pins) {
    analogWrite(pins.PWM_LEFT, SPEED_FAST);
    analogWrite(pins.PWM_RIGHT, SPEED_SLOW);
    digitalWrite(pins.LEFT_FWD, HIGH);
    digitalWrite(pins.LEFT_RWD, LOW);
    digitalWrite(pins.RIGHT_FWD, HIGH);
    digitalWrite(pins.RIGHT_RWD, LOW);
}

void turnRight(Pins pins) {
    analogWrite(pins.PWM_LEFT, SPEED_FAST);
    analogWrite(pins.PWM_RIGHT, SPEED_FAST);
    digitalWrite(pins.LEFT_FWD, HIGH);
    digitalWrite(pins.LEFT_RWD, LOW);
    digitalWrite(pins.RIGHT_FWD, LOW);
    digitalWrite(pins.RIGHT_RWD, HIGH);
}
