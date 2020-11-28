/**
 * @file commandParser.hpp
 * @author David Pescariu | https://github.com/davidp-ro
 * @brief Parser for the commands recieved via Serial
 * @version 1.1
 * @date 2020-11-28
 *
 * @copyright GPL-3.0 License
 */

#ifndef CMD_PARSER
#define CMD_PARSER

#include <Arduino.h>

#include "motors.hpp"

/**
 * @brief Parser for the commands recieved via Serial
 */
class CommandParser {
   private:
    Pins motorPins;
    uint8_t motorSpeed;  // Defaults to SPEED_FAST
    bool isInDebugMode;
    String lastCommand;

    /**
     * @brief Set the motorSpeed with the value recieved
     *
     * @param recvSpeed String - slow/med/fast, if unknown will be SPEED_FAST
     */
    void setMotorSpeed(String recvSpeed);

   public:
    /**
     * @brief Construct a new Command Parser object
     *
     * @param DEBUG_MODE bool - DEBUG_MODE var from controller.ino
     */
    CommandParser(bool DEBUG_MODE);

    /**
     * @brief Assign the pins for the CommandParser
     *
     * @param pins Pins - struct where the pins are defined
     */
    void assignPins(Pins pins);

    /**
     * @brief Parse a command recieved via Serial
     *
     * @param cmd String - The command
     * @return uint8_t - 0: Success | 1: Unknown command 99: Fail
     */
    uint8_t parse(String cmd);
};

#endif
