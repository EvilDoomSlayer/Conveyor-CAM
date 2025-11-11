/* Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * ===================================================================
 * CAM (GATT Server / Peripheral) - Arduino Version
 * * Merged with Edge Impulse for real-time inference.
 *
 * This main file handles:
 * - Camera Initialization
 * - Edge Impulse Inference Task (Core 1)
 *
 * All BLE logic is handled by the BleCamServer class.
 * ===================================================================
 */

// --- Custom Library Includes ---
#include "ble_definitions.h"  // Shared definitions
#include "BleCamServer.h"     // Our new BLE library

// --- Edge Impulse & Camera Includes ---
#include <EvilDoomSlayer-project-1_inferencing.h> // Your Edge Impulse Model
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include "LedController.h"

// --- Camera Model Definition (Unchanged) ---
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#define FLASH_GPIO_NUM   4
#define LED_GPIO_NUM  33

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#else
#error "Camera model not selected or supported"
#endif

/* Constant defines */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS         320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS         240
#define EI_CAMERA_FRAME_BYTE_SIZE               3

/* Private variables */
static bool debug_nn = false; 
static bool is_initialised = false;
uint8_t *snapshot_buf; 

// --- FreeRTOS Queues for Cross-Core Communication ---
// These are the "glue" between the BLE lib and the Inference task
static QueueHandle_t command_queue = NULL;   // (BLE -> Inference)
static QueueHandle_t detection_queue = NULL; // (Inference -> BLE)

// --- BLE Global ---
// We now only have one object for our library
static BleCamServer gBleServer;

// --- Camera Configuration ---
static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,   //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,      //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};


// -------------------------------------------------------------------
// --- EI HELPER FUNCTIONS ---
// -------------------------------------------------------------------
void ei_printf(const char *format, ...) {
   static char loc_buf[64];
   char * temp = loc_buf;
   va_list arg;
   va_start(arg, format);
   int len = vsnprintf(NULL, 0, format, arg);
   va_end(arg);
   if (len > sizeof(loc_buf)) {
       temp = (char*)malloc(len + 1);
       if (temp == NULL) {
           return;
       }
   }
   va_start(arg, format);
   vsnprintf(temp, len + 1, format, arg);
   va_end(arg);
   Serial.print(temp);
   if (temp != loc_buf) {
       free(temp);
   }
}

bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

// -------------------------------------------------------------------
// --- CORE 1 TASK: Inference & Logic Task ---
// -------------------------------------------------------------------
void inference_and_logic_task(void *pvParameters) {
    Serial.println("Inference & Logic Task started on Core 1");
    state_command_t current_state = STATE_CMD_IDLE;
    detection_result_t last_detection_sent = DETECTION_RES_NONE;
    
    // --- Debouncing variables ---
    detection_result_t last_frame_detection = DETECTION_RES_NONE;
    int stable_frames_count = 0;
    const int FRAMES_FOR_STABLE = 2; // Must see same result for 3 frames

    while (1) {
        // --- 1. Check for new commands from Conveyor (non-blocking) ---
        state_command_t new_cmd;
        if (xQueueReceive(command_queue, &new_cmd, 0) == pdPASS) {
            if (new_cmd == STATE_CMD_RUN && current_state == STATE_CMD_IDLE) {
                Serial.println("Received [RUN] command. Starting detection.");
                current_state = STATE_CMD_RUN;
                last_detection_sent = DETECTION_RES_NONE; // Reset
                last_frame_detection = DETECTION_RES_NONE;
                stable_frames_count = 0;
            } else if (new_cmd == STATE_CMD_IDLE) {
                if (current_state == STATE_CMD_RUN) {
                   Serial.println("Received [IDLE] command. Stopping detection.");
                }
                current_state = STATE_CMD_IDLE;
            }
        }

        // --- 2. Run inference logic ---
        if (current_state == STATE_CMD_RUN) {
            
            // --- 2a. Allocate Buffer (from your EI sketch) ---
            snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
            if(snapshot_buf == nullptr) {
                Serial.println("ERR: Failed to allocate snapshot buffer!");
                vTaskDelay(pdMS_TO_TICKS(1000)); // Wait before retrying
                continue; 
            }

            // --- 2b. Prepare Signal (from your EI sketch) ---
            ei::signal_t signal;
            signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
            signal.get_data = &ei_camera_get_data;

            // --- 2c. Capture Image (from your EI sketch) ---
            if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
                Serial.println("Failed to capture image\r\n");
                free(snapshot_buf);
                snapshot_buf = NULL;
                vTaskDelay(pdMS_TO_TICKS(100)); // Wait a bit before retry
                continue; 
            }

            // --- 2d. Run Classifier (from your EI sketch) ---
            ei_impulse_result_t result = { 0 };
            EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
            
            if (err != EI_IMPULSE_OK) {
                Serial.printf("ERR: Failed to run classifier (%d)\n", err);
                free(snapshot_buf); 
                snapshot_buf = NULL;
                continue; 
            }

            // --- 2e. Process results and send to queue ---
            detection_result_t current_detection = DETECTION_RES_NONE;
            float best_value = 0.0;
            const char* best_label = "none";
            
            Serial.printf("Predictions (DSP: %d ms., Classification: %d ms.): \n",
                result.timing.dsp, result.timing.classification);

            #if EI_CLASSIFIER_OBJECT_DETECTION == 1
                // Logic for Object Detection
                ei_printf("Object detection bounding boxes:\r\n");
                for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
                    ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
                    if (bb.value == 0) {
                        continue;
                    }
                    ei_printf("   %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                            bb.label,
                            bb.value,
                            bb.x,
                            bb.y,
                            bb.width,
                            bb.height);
                    if (bb.value > best_value) {
                        best_value = bb.value;
                        best_label = bb.label;
                    }
                }
                
                if (result.bounding_boxes_count > 1) {
                        Serial.println("WARN: Multiple objects detected!");
                        current_detection = DETECTION_RES_MULTI_ERROR;
                } else if (result.bounding_boxes_count == 1) {
                    if (strcmp(best_label, "light") == 0) {
                        current_detection = DETECTION_RES_LIGHT;
                    } else if (strcmp(best_label, "heavy") == 0) {
                        current_detection = DETECTION_RES_HEAVY;
                    } else {
                        current_detection = DETECTION_RES_NONE; // Found something, but not light/heavy
                    }
                } else {
                    current_detection = DETECTION_RES_NONE; // No valid detections
                }
            
            #else // Logic for Classification
                for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                    Serial.printf("   %s: %.5f\n", ei_classifier_inferencing_categories[i], result.classification[i].value);
                    if (result.classification[i].value > best_value) {
                        best_value = result.classification[i].value;
                        best_label = ei_classifier_inferencing_categories[i];
                    }
                }

                // Map labels to protocol enums
                if (best_value < 0.8) { // Confidence threshold
                    current_detection = DETECTION_RES_NONE;
                } else if (strcmp(best_label, "light") == 0) {
                    current_detection = DETECTION_RES_LIGHT;
                } else if (strcmp(best_label, "heavy") == 0) {
                    current_detection = DETECTION_RES_HEAVY;
                } else { // "none" or other class
                    current_detection = DETECTION_RES_NONE;
                }
            #endif

            // --- 2f. Debounce the result ---
            if (current_detection == last_frame_detection) {
                stable_frames_count++;
            } else {
                // Result flickered, reset count
                stable_frames_count = 1; 
                last_frame_detection = current_detection;
            }

            detection_result_t final_detection_to_send;
            
            if (stable_frames_count >= FRAMES_FOR_STABLE) {
                // We have a stable result
                final_detection_to_send = current_detection;
            } else {
                // Not stable yet, report the last stable result
                final_detection_to_send = last_detection_sent;
            }

            // Only send an update if the *stable* detection changes
            if (final_detection_to_send != last_detection_sent) {
                Serial.printf("Stable detection changed to %d. Sending to BLE Task.\n", final_detection_to_send);
                // Send to the BLE task
                xQueueSend(detection_queue, &final_detection_to_send, portMAX_DELAY);
                last_detection_sent = final_detection_to_send;
            }
            
            free(snapshot_buf); // Free memory *after* use
            snapshot_buf = NULL;
            
            // Add a small delay so we're not running inference at 100% CPU
            vTaskDelay(pdMS_TO_TICKS(50));

        } else {
            // We are IDLE, just sleep
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    } // end while(1)
}


// -------------------------------------------------------------------
// --- Main Application Setup ---
// -------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    Serial.println("Starting CAM Server (Arduino + Edge Impulse)...");

    setupLedController(LED_GPIO_NUM, 1, 0);
    setLedMode(BLINK_RAPID);

    pinMode(FLASH_GPIO_NUM, OUTPUT);
    digitalWrite(FLASH_GPIO_NUM, HIGH); // Turn on LED

    // --- Initialize Camera ---
    if (ei_camera_init() == false) {
        Serial.println("Failed to initialize Camera! Halting.");
        while(1);
    }
    Serial.println("Camera initialized");

    // --- Create Queues ---
    command_queue = xQueueCreate(10, sizeof(state_command_t));
    detection_queue = xQueueCreate(10, sizeof(detection_result_t));

    // --- Initialize BLE ---
    // This is now just one function call!
    gBleServer.begin(command_queue, detection_queue);
    
    // --- Create FreeRTOS Tasks ---
    // Create Inference & Logic Task on Core 1
    xTaskCreatePinnedToCore(
        inference_and_logic_task,
        "inference_task",       // Task name
        10240,                  // Stack size (EI needs a lot)
        NULL,
        5,                      // Priority
        NULL,
        1                       // Pin to Core 1
    );

    // The BLE task is created *inside* gBleServer.begin()

    Serial.println("Initialization complete. Tasks started.");
}

void loop() {
    // Arduino loop is not used. All logic is in FreeRTOS tasks.
    vTaskDelay(pdMS_TO_TICKS(2000));
}


// -------------------------------------------------------------------
// --- EI CAMERA FUNCTIONS ---
// -------------------------------------------------------------------

bool ei_camera_init(void) {
    if (is_initialised) return true;

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
     Serial.printf("Camera init failed with error 0x%x\n", err);
     return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
     s->set_vflip(s, 1); // flip it back
     s->set_brightness(s, 1); // up the brightness just a bit
     s->set_saturation(s, 0); // lower the saturation
    }

    #if defined(CAMERA_MODEL_AI_THINKER)
     s->set_vflip(s, 1);
     s->set_hmirror(s, 1);
    #endif

    is_initialised = true;
    return true;
}

void ei_camera_deinit(void) {
    esp_err_t err = esp_camera_deinit();
    if (err != ESP_OK) {
        ei_printf("Camera deinit failed\n");
        return;
    }
    is_initialised = false;
    return;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

    // Use out_buf as the destination for the RGB data
    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, out_buf);
    esp_camera_fb_return(fb);
    if(!converted){
        ei_printf("Conversion failed\n");
        return false;
    }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf, // src
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf, // dst
        img_width,
        img_height);
    }
    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        // Swap BGR to RGB here
        // due to https://github.com/espressif/esp32-camera/issues/379
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
}