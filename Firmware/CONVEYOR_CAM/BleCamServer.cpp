#include "BleCamServer.h"
#include "LedController.h"
#include <string> // For std::string in onWrite

// --- Constants ---
#define DEVICE_NAME "ESP32_CAM_SERVER"
#define CAM_SERVER_TAG "BLE_CAM_LIB" // Log Tag

// --- Constructor ---
BleCamServer::BleCamServer() :
    _commandQueue(NULL),
    _detectionQueue(NULL),
    _notifyTaskHandle(NULL),
    _pServer(NULL),
    _pStateCommandChar(NULL),
    _pDetectionResultChar(NULL),
    _deviceConnected(false) {
    // Constructor body is empty, initialization is done in begin()
}

// --- Public Methods ---

void BleCamServer::begin(QueueHandle_t commandQueue, QueueHandle_t detectionQueue) {
    _commandQueue = commandQueue;
    _detectionQueue = detectionQueue;

    // --- 1. Initialize BLE ---
    BLEDevice::init(DEVICE_NAME);
    
    _pServer = BLEDevice::createServer();
    // Pass 'this' (a pointer to this BleCamServer instance) to the callbacks
    _pServer->setCallbacks(new ServerCallbacks(this));

    // --- 2. Create Service ---
    BLEService *pService = _pServer->createService(SERVICE_UUID_CONVEYOR);

    // --- 3. Create State Command Characteristic (Client -> Server) ---
    _pStateCommandChar = pService->createCharacteristic(
        CHAR_UUID_STATE_COMMAND,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    _pStateCommandChar->setCallbacks(new StateCommandCallbacks(this));

    // --- 4. Create Detection Result Characteristic (Server -> Client) ---
    _pDetectionResultChar = pService->createCharacteristic(
        CHAR_UUID_DETECTION,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    _pDetectionResultChar->addDescriptor(new BLE2902()); // Add CCC Descriptor

    // --- 5. Start Service & Advertising ---
    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID_CONVEYOR);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();
    
    Serial.println("BLE Server started. Advertising.");

    // --- 6. Create BLE Notification Task on Core 0 ---
    xTaskCreatePinnedToCore(
        BleCamServer::notifyTaskWrapper, // Static wrapper function
        "ble_notify_task",
        4096,               // Stack size
        this,               // Pass 'this' instance as the parameter
        5,                  // Priority
        &_notifyTaskHandle, // Task handle
        0                   // Pin to Core 0 (Arduino default core)
    );
}

bool BleCamServer::isClientConnected() {
    return _deviceConnected;
}

// --- Private Task Methods ---

/**
 * @brief Static wrapper to launch the FreeRTOS task.
 */
void BleCamServer::notifyTaskWrapper(void* pvParameters) {
    // Cast the void pointer back to our class instance
    BleCamServer* pInstance = (BleCamServer*)pvParameters;
    // Call the non-static member function
    pInstance->runNotifyTask();
}

/**
 * @brief The main loop for the BLE notification task.
 * Runs on Core 0.
 */
void BleCamServer::runNotifyTask() {
    Serial.println("BLE Notify Task started on Core 0");
    detection_result_t detection_to_send;

    while (1) {
        // Wait for a new detection result from the inference task
        if (xQueueReceive(_detectionQueue, &detection_to_send, portMAX_DELAY) == pdPASS) {
            
            if (!_deviceConnected) {
                Serial.println("Want to send detection, but no device connected.");
                continue;
            }

            // Convert the enum to the string your client expects
            const char* notify_data_str;
            switch(detection_to_send) {
                case DETECTION_RES_LIGHT:
                    notify_data_str = "light";
                    break;
                case DETECTION_RES_HEAVY:
                    notify_data_str = "heavy";
                    break;
                case DETECTION_RES_MULTI_ERROR:
                    notify_data_str = "multi_error"; // Send a specific string for multi-error
                    break;
                case DETECTION_RES_NONE:
                default:
                    notify_data_str = "none";
                    break;
            }
            
            Serial.printf("Sending Notify! Detection: %s\n", notify_data_str);

            // Set the value of the characteristic as a string
            _pDetectionResultChar->setValue(notify_data_str);
            
            // Send the notification
            _pDetectionResultChar->notify();
            
            // Bluetooth stack will handle timing, but a small delay can prevent flooding
            vTaskDelay(pdMS_TO_TICKS(10)); 
        }
    }
}


// --- Callback Implementations ---

void BleCamServer::ServerCallbacks::onConnect(BLEServer* pServer) {
    Serial.println("Client Connected");
    _pInstance->_deviceConnected = true;
    setLedMode(SOLID_ON);
}

void BleCamServer::ServerCallbacks::onDisconnect(BLEServer* pServer) {
    Serial.println("Client Disconnected");
    _pInstance->_deviceConnected = false;
    setLedMode(BLINK_RAPID);
    // Restart advertising
    BLEDevice::startAdvertising();
}

void BleCamServer::StateCommandCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue().c_str();

    if (value.length() > 0) {
        state_command_t received_cmd = (state_command_t)value[0];
        Serial.printf("Received command: 0x%02X\n", received_cmd);
        
        // Send this command to the camera simulation task on Core 0
        // Use the queue handle stored in our parent instance
        xQueueSend(_pInstance->_commandQueue, &received_cmd, pdMS_TO_TICKS(10));
    }
}