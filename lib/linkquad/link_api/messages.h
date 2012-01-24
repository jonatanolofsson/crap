#ifndef MESSAGES_H_
#define MESSAGES_H_


// Control commands with IDS below 100 are for internal use only
enum {
    MSG_CONTROL_COMMAND = 100,				// Control command
    MSG_CONTROL_COMMAND_INT = 101, 			// Control command, with integer
    MSG_CONTROL_COMMAND_TEXT = 102,			// Control command, with text

    MSG_PRINT_DEBUG = 103,					// Text message
    MSG_PRINT_DEBUG_I = 104,				// Text message including an integer

    MSG_CMCU_DATA_PART_REQUEST = 105,		// Control MCU data request
    MSG_CMCU_DATA_PART = 106,				// Requested control MCU data
    MSG_CMCU_DATA_PART_CONFIG_REQUEST = 107,// Current control MCU data request configuration

    MSG_SMCU_DATA_PART_REQUEST = 108,		// Sensor MCU data request
    MSG_SMCU_DATA_PART = 109,				// Requested sensor MCU data
    MSG_SMCU_DATA_PART_CONFIG_REQUEST = 110,// Current sensor MCU data request configuration

    MSG_USER_PARAMS_PART = 111,				// User parameters

    MSG_FLIGHT_TARGET_PARAMS = 112,			// Flight (e.g. hover) target parameters
    MSG_PAN_TILT_CONTROL_MODE = 113,			// Camera control mode
    MSG_HL_COMMAND_REQUEST = 114,			//
    MSG_HL_COMMAND_RESPONSE = 115			//
};

// Control sub-commands used with MSG_CONTROL_COMMAND
// Control sub-commands with IDS below 20 are for internal use only
enum {
    MSG_CONTROL_COMMAND_CMCU_START_MAIN_LOOP = 20, // Start the main loop of the control MCU
    MSG_CONTROL_COMMAND_CMCU_STOP_MAIN_LOOP = 21,  // Stop the main loop of the control MCU
    MSG_CONTROL_COMMAND_STOP_LOG = 22,			   // Stops logging
    MSG_CONTROL_COMMAND_SMCU_START_MAIN_LOOP = 23, // Start the main loop of the sensor MCU
    MSG_CONTROL_COMMAND_SMCU_STOP_MAIN_LOOP = 24,  // Stop the main loop of the sensor MCU
    MSG_CONTROL_COMMAND_ENABLE_MOTORS = 25,		   // Enables sending of the control signals to the engines
    MSG_CONTROL_COMMAND_DISABLE_MOTORS = 26		   // Disables sending of the control signals to the engines
};

// Control (with text) sub-commands used with MSG_CONTROL_COMMAND_TEXT
enum {
    MSG_CONTROL_COMMAND_START_LOG = 0		// Starts logging
};


#endif /* MESSAGES_H_ */
