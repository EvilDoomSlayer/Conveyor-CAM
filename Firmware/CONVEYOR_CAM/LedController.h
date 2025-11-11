/**
 * @file LedController.h
 * @author M. Alejandro Sáchez R.
 * @brief Public interface for a non-blocking LED controller using a dedicated FreeRTOS task.
 * @version 0.2
 * @date 2025-10-26
 *
 * @copyright Copyright (c) 2025
 *
 * This library abstracts away the complexity of blinking an LED without using `delay()`.
 * It creates a separate task that handles all timing and state changes, allowing
 * the rest of your code to run without interruption. You can change the LED's
 * pattern at any time by calling `setLedMode()`.
 */

#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <Arduino.h>

/**
 * @brief Defines the different blinking patterns the LED can perform.
 */
typedef enum {
    SOLID_ON,    // LED is constantly on.
    BLINK_RAPID, // LED blinks quickly (e.g., for pairing or error states).
    BLINK_SLOW,  // LED blinks slowly (e.g., for a "waiting" or "standby" state).
    SOLID_OFF    // LED is constantly off.
} BlinkType_t;

/**
 * @brief Initializes and starts the LED control task.
 *
 * This function creates the necessary FreeRTOS queue and the dedicated task that
 * will manage the LED. This should be called once from your main `setup()` function.
 *
 * @param ledPin The GPIO pin number the LED is connected to.
 * @param priority The priority for the new FreeRTOS task.
 * @param core The CPU core on which the task should run (0 or 1).
 */
void setupLedController(uint8_t ledPin, UBaseType_t priority, BaseType_t core);

/**
 * @brief Sends a command to change the LED's blinking pattern.
 *
 * This function is non-blocking. It sends a message to the LED control task's
 * queue, instructing it to switch to a new pattern. It is safe to call this
 * from any task in your application.
 *
 * @param mode The desired blinking pattern (e.g., SOLID_ON, BLINK_RAPID).
 */
void setLedMode(BlinkType_t mode);

#endif // LED_CONTROLLER_H