/**
 * @file commandParser.cpp
 * @author David Pescariu | https://github.com/davidp-ro
 * @brief Implementations for the CommandParser class
 * @version 1.0
 * @date 2020-11-28
 *
 * @copyright GPL-3.0 License
 */

#include "commandParser.hpp"

#include <Arduino.h>

CommandParser::CommandParser(bool DEBUG_MODE) {
    this->isInDebugMode = DEBUG_MODE;
    this->motorSpeed = SPEED_FAST;
}

void CommandParser::setMotorSpeed(String recvSpeed) {
    if (recvSpeed == "slow") {
        this->motorSpeed = SPEED_SLOW;
    } else if (recvSpeed == "med") {
        this->motorSpeed = SPEED_MED;
    } else {
        this->motorSpeed = SPEED_FAST;
    }

    if (this->isInDebugMode) {
        Serial.print("[DEBUG::CommandParser] Set the motorSpeed to ");
        Serial.println(this->motorSpeed);
    }
}

void CommandParser::assignPins(Pins pins) { this->motorPins = pins; }

uint8_t CommandParser::parse(String cmd) {
    uint8_t status = 99;

    if (cmd.startsWith("set_speed")) {
        String spd = cmd.substring(10);
        this->setMotorSpeed(spd);
        status = 0;
    } else if (cmd == "get_distance") {
        // TODO: Implement
        status = 0;
    } else if (cmd == "fwd") {
        goForward(this->motorPins, this->motorSpeed);
        status = 0;
    } else if (cmd == "stop") {
        stop(this->motorPins);
        status = 0;
    } else if (cmd == "rev") {
        reverse(this->motorPins);
        status = 0;
    } else if (cmd == "turn_sl") {
        turnSlightlyLeft(this->motorPins);
        status = 0;
    } else if (cmd == "turn_l") {
        turnLeft(this->motorPins);
        status = 0;
    } else if (cmd == "turn_sr") {
        turnSlightlyRight(this->motorPins);
        status = 0;
    } else if (cmd == "turn_r") {
        turnRight(this->motorPins);
        status = 0;
    } else {
        status = 1;
    }

    return status;
}
