/**
 * @file main.ino
 * @author David Pescariu | https://github.com/davidp-ro
 * @brief Part of the self-driving-car project.
 * @version 1.0
 * @date 2020-11-28
 * 
 * @copyright GPL-3.0 License
 * 
 *  The uno will recieve via serial commands from the pi to control the motors.
 * It also acts a sort of "supervisor", as it will use a distance sensor (hc-sr04)
 * and if the distance reported by it is less than a threshold it will only accept
 * reverse commands.
 * 
 * Serial commands:
 *  TODO 
 */

#include "motors.hpp"

Pins motorPins;

void setup() {
    // Motor setup 
    motorPins.PWM_LEFT = 11;
    motorPins.PWM_RIGHT = 10;
    motorPins.LEFT_FWD = 13;
    motorPins.LEFT_RWD = 12;
    motorPins.RIGHT_FWD = 9;
    motorPins.RIGHT_RWD = 8;
    stop(motorPins);

    // Serial
    Serial.begin(9200);
    Serial.println("[INIT] Running version: 1.0");
}

void loop() {
    goForward(motorPins, SPEED_FAST);
    delay(1000);
    stop(motorPins);
    delay(500);
    reverse(motorPins);
    delay(500);
    stop(motorPins);
    delay(500);
}