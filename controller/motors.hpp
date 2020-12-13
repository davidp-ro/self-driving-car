/**
 * @file motors.hpp
 * @author David Pescariu | https://github.com/davidp-ro
 * @brief Functions that control the motors and the pin struct
 * @version 1.2
 * @date 2020-12-13
 *
 * @copyright GPL-3.0 License
 */

#ifndef MOTORS
#define MOTORS

#include <Arduino.h>

const uint8_t SPEED_SLOW = 100;
const uint8_t SPEED_MED = 110;
const uint8_t SPEED_FAST = 255;

/**
 * @brief Struct for the pin selection. Used by all motor functions
 */
typedef struct {
    uint8_t PWM_LEFT;
    uint8_t PWM_RIGHT;
    uint8_t LEFT_FWD;
    uint8_t LEFT_RWD;
    uint8_t RIGHT_FWD;
    uint8_t RIGHT_RWD;
} Pins;

/**
 * @brief Initialize the pins (set pinMode)
 * 
 * @param pins Pins struct 
 */
void initPins(Pins pins);

/**
 * @brief Set the motors to go forward
 *
 * @param pins Pins struct
 * @param speed PWM value (0-255), use SPEED_* constants
 */
void goForward(Pins pins, uint8_t speed);

/**
 * @brief Stop the motors
 *
 * @param pins Pins struct
 */
void stop(Pins pins);

/**
 * @brief Set the motors to go in reverse
 *
 * @param pins Pins struct
 */
void reverse(Pins pins);

/**
 * @brief Turn a little bit to the left
 *
 * @param pins Pins struct
 */
void turnSlightlyLeft(Pins pins);

/**
 * @brief Hard turn left
 *
 * @param pins Pins struct
 */
void turnLeft(Pins pins);

/**
 * @brief Turn a little bit to the right
 *
 * @param pins Pins struct
 */
void turnSlightlyRight(Pins pins);

/**
 * @brief Hard turn right
 *
 * @param pins Pins struct
 */
void turnRight(Pins pins);

#endif
