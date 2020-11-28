/**
 * @file controller.ino
 * @author David Pescariu | https://github.com/davidp-ro
 * @brief Arduino UNO controller for the self-driving-car project.
 * @version 1.1
 * @date 2020-11-28
 *
 * @copyright GPL-3.0 License
 *
 *  The uno will recieve via serial commands from the pi to control the motors.
 * It also acts a sort of "supervisor", as it will use a distance sensor
 * (hc-sr04) and if the distance reported by it is less than a threshold it will
 * only accept reverse commands.
 *
 * Serial commands:
 *  NOTE: The commands should not contain spaces and be terminated with a \n
 * 
 *  set_speed=SPEED -> Sets the speed where SPEED is slow/med/fast, default=fast
 *  get_distance <-> Will print(\n) the distance from the sc-hr04
 *
 *  fwd -> Go forward
 *  stop -> Stop
 *  rev -> Go in reverse
 *  turn_sl -> Turn slightly left
 *  turn_l -> Turn left
 *  turn_sr -> Turn slightly right
 *  turn_r -> Turn right
 * 
 * TODO:
 *  - Add support for distance sensor
 *      -> If distance < threshold, only accept the rev command
 *      -> Implement the get_distance functionality
 * 
 *  - Add some light to the car for extra *wow* ¯\_(ツ)_/¯
 */

#include "commandParser.hpp"
#include "motors.hpp"

bool DEBUG_MODE = true;

Pins motorPins;
CommandParser commandParser(DEBUG_MODE);

void setup() {
    // Serial
    Serial.begin(9600);
    Serial.println("[INIT] Running version: 1.0");

    // Motor setup
    motorPins.PWM_LEFT = 11;
    motorPins.PWM_RIGHT = 10;
    motorPins.LEFT_FWD = 13;
    motorPins.LEFT_RWD = 12;
    motorPins.RIGHT_FWD = 9;
    motorPins.RIGHT_RWD = 8;
    pinMode(motorPins.PWM_LEFT, OUTPUT);
    pinMode(motorPins.PWM_RIGHT, OUTPUT);
    pinMode(motorPins.LEFT_FWD, OUTPUT);
    pinMode(motorPins.LEFT_RWD, OUTPUT);
    pinMode(motorPins.RIGHT_FWD, OUTPUT);
    pinMode(motorPins.RIGHT_RWD, OUTPUT);
    stop(motorPins);

    // Set parser
    commandParser.assignPins(motorPins);
}

void loop() {
    if (Serial.available() > 0) {
        String data = Serial.readStringUntil('\n');
        if (DEBUG_MODE) {
            Serial.print("[RX] Command::");
            Serial.println(data);
        }

        uint8_t res = commandParser.parse(data);
        if (res && DEBUG_MODE) {
            Serial.print("[FAIL::CommandParser] Returned ");
            Serial.println(res);
        }
    }
}
