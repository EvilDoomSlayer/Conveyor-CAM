/**
 * @file LedController.cpp
 * @author M. Alejandro Sáchez R.
 * @brief Implementation of the non-blocking FreeRTOS LED controller.
 * @version 0.2
 * @date 2025-10-26
 *
 * @copyright Copyright (c) 2025
 *
 * This file contains the private task and queue management for the LED
 * controller. The `ledControlTask` is the heart of the library, running in an
 * infinite loop to manage the LED's state based on commands received via a
 * FreeRTOS queue.
 */

#include "LedController.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// --- Internal (Private) Variables ---
// The 'static' keyword limits the scope of these variables to this file only,
// providing encapsulation.
static QueueHandle_t ledQueue; // Queue to receive commands (BlinkType_t) from the main application.
static uint8_t _ledPin;        // Stores the pin number provided during setup.

/**
 * @brief The FreeRTOS task that handles all LED blinking patterns.
 * @param pvParameters Unused task parameters.
 *
 * This task is the core of the LED controller's logic. It runs in an infinite loop.
 * In each iteration, it briefly checks for a new command from the `ledQueue`.
 * If a new command exists, it updates its current blinking mode. Then, it executes
 * one cycle of the current mode (e.g., turning the LED on and off). Using `vTaskDelay`
 * ensures this task does not consume unnecessary CPU time and allows other tasks to run.
 */
static void ledControlTask(void *pvParameters) {
    pinMode(_ledPin, OUTPUT);

    BlinkType_t currentBlinkType = BLINK_RAPID; // Default starting state
    BlinkType_t newBlinkType;

    while (1) {
        // Check for a new message in the queue. Use a short, non-blocking timeout (10ms).
        // This allows the loop to continue executing the *current* blinking pattern
        // without getting stuck waiting for a new command.
        if (xQueueReceive(ledQueue, &newBlinkType, pdMS_TO_TICKS(10))) {
            currentBlinkType = newBlinkType;
        }

        // Execute one step of the current blinking pattern
        switch (currentBlinkType) {
            case SOLID_ON:
                digitalWrite(_ledPin, HIGH);
                // A small delay is still needed to yield CPU time to other tasks.
                vTaskDelay(pdMS_TO_TICKS(50));
                break;

            case BLINK_RAPID:
                digitalWrite(_ledPin, HIGH);
                vTaskDelay(pdMS_TO_TICKS(100));
                digitalWrite(_ledPin, LOW);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case BLINK_SLOW:
                digitalWrite(_ledPin, HIGH);
                vTaskDelay(pdMS_TO_TICKS(1000));
                digitalWrite(_ledPin, LOW);
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;

            case SOLID_OFF:
            default:
                digitalWrite(_ledPin, LOW);
                // Yield CPU time while waiting for a new command. The xQueueReceive
                // at the top of the loop will handle most of the waiting.
                vTaskDelay(pdMS_TO_TICKS(50));
                break;
        }
    }
}


// --- Public Function Definitions ---

/**
 * @brief Sets up the LED pin, creates the queue, and launches the control task.
 */
void setupLedController(uint8_t ledPin, UBaseType_t priority, BaseType_t core) {
    _ledPin = ledPin; // Store the pin for the task to use

    // Create the queue. It can hold up to 5 commands of type BlinkType_t.
    ledQueue = xQueueCreate(5, sizeof(BlinkType_t));

    if (ledQueue == NULL) {
        Serial.println("Error creating the LED queue");
        while (1); // Halt on critical error
    }

    // Create the dedicated FreeRTOS task for controlling the LED.
    xTaskCreatePinnedToCore(
        ledControlTask,     // Function to implement the task
        "LED Control Task", // A descriptive name for debugging
        2048,               // Stack size in bytes
        NULL,               // Task input parameter
        priority,           // Priority of the task
        NULL,               // Task handle
        core);              // Core where the task should run
}

/**
 * @brief Public function to send a new blinking mode to the LED task.
 */
void setLedMode(BlinkType_t mode) {
    // Safely send the new mode to the queue if it has been initialized.
    // We use a 0-tick wait time, meaning it will not block if the queue is full.
    // This is generally safe as it's unlikely to send 5+ commands before the
    // LED task can process them.
    if (ledQueue != NULL) {
        xQueueSend(ledQueue, &mode, (TickType_t)0);
    }
}