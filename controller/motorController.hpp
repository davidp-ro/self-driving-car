#ifndef MOTOR_CONTROLLER
#define MOTOR_CONTROLLER

#include "Arduino.h"
#include "statusLed.hpp"

typedef struct {
    byte LEFT_PWM_PIN;
    byte RIGHT_PWM_PIN;
    byte LEFT_FOWARD_PIN;
    byte LEFT_BACKWARD_PIN;
    byte RIGHT_FOWARD_PIN;
    byte RIGHT_BACKWARD_PIN;
} pins;

typedef enum {
    FOWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    STOP
} Direction;

class MotorController {
    public:
        MotorController();
        void bindPins(uint8_t selectedPins[]);
        void move(Direction d);
    private:
        pins p;
        void goLeft(pins p);
        void goRight(pins p);
        void goFwd(pins p);
        void goBkd(pins p);
        void stop(pins p);
};

#endif // MOTOR_CONTROLLER
