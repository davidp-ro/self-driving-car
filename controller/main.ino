#include "motorController.hpp"

MotorController motorController;
Direction direction;
uint8_t motorPins[] = {1, 2, 3, 4, 5, 6}; //FIXME: Placeholder pins

void setup() {
    motorController.bindPins(motorPins);
}

void loop() {
    direction = FOWARD;
    motorController.move(direction);
    delay(5000);
    direction = STOP;
    motorController.move(direction);
    delay(5000);
}