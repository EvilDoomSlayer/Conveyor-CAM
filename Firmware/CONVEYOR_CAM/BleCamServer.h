#ifndef BLE_CAM_SERVER_H
#define BLE_CAM_SERVER_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLECharacteristic.h>
#include <BLE2902.h>
#include "ble_definitions.h" // Include your shared definitions
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

class BleCamServer {
public:
    BleCamServer();
    
    /**
     * @brief Initializes the BLE server, services, characteristics,
     * and starts the BLE notification task.
     * @param commandQueue Handle to the queue for receiving state commands.
     * @param detectionQueue Handle to the queue for sending detection results.
     */
    void begin(QueueHandle_t commandQueue, QueueHandle_t detectionQueue);

    /**
     * @brief Checks if a BLE client is currently connected.
     * @return true if a client is connected, false otherwise.
     */
    bool isClientConnected();

private:
    // --- RTOS ---
    QueueHandle_t _commandQueue;     // Queue for incoming commands (from client)
    QueueHandle_t _detectionQueue;   // Queue for outgoing results (to client)
    TaskHandle_t _notifyTaskHandle;

    // --- BLE Objects ---
    BLEServer* _pServer;
    BLECharacteristic* _pStateCommandChar;
    BLECharacteristic* _pDetectionResultChar;

    // --- State ---
    // Volatile as it's modified in a callback (ISR context) and read in a task
    volatile bool _deviceConnected;

    // --- Task Methods ---
    static void notifyTaskWrapper(void* pvParameters);
    void runNotifyTask();

    // --- Nested Callback Classes ---
    // These are now private to the BleCamServer class

    // Server (Connect/Disconnect) Callbacks
    class ServerCallbacks : public BLEServerCallbacks {
    private:
        BleCamServer* _pInstance; // Pointer back to the parent BleCamServer
    public:
        ServerCallbacks(BleCamServer* pInstance) : _pInstance(pInstance) {}
        void onConnect(BLEServer* pServer);
        void onDisconnect(BLEServer* pServer);
    };

    // Characteristic (Write) Callbacks
    class StateCommandCallbacks : public BLECharacteristicCallbacks {
    private:
        BleCamServer* _pInstance; // Pointer back to the parent
    public:
        StateCommandCallbacks(BleCamServer* pInstance) : _pInstance(pInstance) {}
        void onWrite(BLECharacteristic *pCharacteristic);
    };
};

#endif // BLE_CAM_SERVER_H