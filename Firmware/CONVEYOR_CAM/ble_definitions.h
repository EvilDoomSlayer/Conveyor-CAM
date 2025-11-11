#pragma once

/*

COMMON DEFINITIONS FOR CAM SERVER & CONVEYOR CLIENT (ARDUINO)
*/

// --- SERVICE & CHARACTERISTIC UUIDs ---
// Service UUID
#define SERVICE_UUID_CONVEYOR   "00000001-0000-1000-8000-00805F9B34FB"
// Characteristic: Conveyor writes state [IDLE/RUN] to this
#define CHAR_UUID_STATE_COMMAND "00000002-0000-1000-8000-00805F9B34FB"
// Characteristic: Cam notifies detection result [NONE/LIGHT/HEAVY/ERR]
#define CHAR_UUID_DETECTION     "00000003-0000-1000-8000-00805F9B34FB"

// --- APPLICATION PROTOCOL COMMANDS ---

// 1. State Commands (Conveyor -> Cam)
typedef enum {
STATE_CMD_IDLE = 0x00,
STATE_CMD_RUN  = 0x01
} state_command_t;

// 2. Detection Results (Cam -> Conveyor)
typedef enum {
DETECTION_RES_NONE        = 0x00,
DETECTION_RES_LIGHT       = 0x01,
DETECTION_RES_HEAVY       = 0x02,
DETECTION_RES_MULTI_ERROR = 0x99
} detection_result_t;