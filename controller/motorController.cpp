#include "Arduino.h"
#include "motorController.hpp"

MotorController::MotorController () {
    setStatus("init", NULL);
}

void MotorController::bindPins(uint8_t selectedPins[]) {
    pinMode(this->p.LEFT_PWM_PIN = selectedPins[0], OUTPUT);
    pinMode(this->p.RIGHT_PWM_PIN = selectedPins[1], OUTPUT);
    pinMode(this->p.LEFT_FOWARD_PIN = selectedPins[2], OUTPUT);
    pinMode(this->p.LEFT_BACKWARD_PIN = selectedPins[3], OUTPUT);
    pinMode(this->p.RIGHT_FOWARD_PIN = selectedPins[4], OUTPUT);
    pinMode(this->p.RIGHT_BACKWARD_PIN = selectedPins[5], OUTPUT);
}

void MotorController::move(Direction d) {
    switch (d) {
        case FOWARD:
            this->goFwd(this->p);
            break;
        case BACKWARD:
            this->goBkd(this->p);
            break;
        case LEFT:
            this->goLeft(this->p);
            break;
        case RIGHT:
            this->goRight(this->p);
            break;
        case STOP:
            this->stop(this->p);
            break;
    }
}

void MotorController::stop(pins p) {
    digitalWrite(p.LEFT_FOWARD_PIN, 0);
    digitalWrite(p.LEFT_BACKWARD_PIN, 0);
    digitalWrite(p.RIGHT_FOWARD_PIN, 0);
    digitalWrite(p.RIGHT_BACKWARD_PIN, 0);
}

void MotorController::goLeft(pins p) {}
void MotorController::goRight(pins p) {}
void MotorController::goFwd(pins p) {}
void MotorController::goBkd(pins p) {}