#ifndef PACKET_TYPES_DEFINES_H_
#define PACKET_TYPES_DEFINES_H_

#define STRUCT_MARSHAL 0
#define STRUCT_UNMARSHAL 1

#define USER_PARAMS_FLOAT_FIRST 0
#define USER_PARAMS_FLOAT_COUNT 10

#define USER_PARAMS_INT8_FIRST (USER_PARAMS_FLOAT_FIRST + USER_PARAMS_FLOAT_COUNT)
#define USER_PARAMS_INT8_COUNT 10

#define USER_PARAMS_INT16_FIRST (USER_PARAMS_INT8_FIRST + USER_PARAMS_INT8_COUNT)
#define USER_PARAMS_INT16_COUNT 10

#define USER_PARAMS_INT32_FIRST (USER_PARAMS_INT16_FIRST + USER_PARAMS_INT16_COUNT)
#define USER_PARAMS_INT32_COUNT 10
// USER_PARAMS: number of variables which can be requested (all above + flags)
#define USER_PARAMS_REQUEST_IDS_COUNT (USER_PARAMS_FLOAT_COUNT + USER_PARAMS_INT8_COUNT + USER_PARAMS_INT16_COUNT + USER_PARAMS_INT32_COUNT + 1)

#define SMCU_DATA_IDS_COUNT 66
#define CMCU_DATA_IDS_COUNT 157

typedef enum {
	SYSTEM_STATUS_CMCU_INITIALIZING = 0,
	SYSTEM_STATUS_CMCU_INIT_OK = 1,
	SYSTEM_STATUS_CMCU_INIT_FAILED = 2,
	SYSTEM_STATUS_CONFIG_READING = 3,
	SYSTEM_STATUS_CONFIG_READ_OK = 4,
	SYSTEM_STATUS_CONFIG_READ_FAILED = 5,
	SYSTEM_STATUS_SYNC_WAIT = 6,
	SYSTEM_STATUS_SYNC_DONE = 7,
	SYSTEM_STATUS_CONFIGURING_SMCU = 8,
	SYSTEM_STATUS_CONFIGURING_SMCU_DONE = 9,
	SYSTEM_STATUS_SMCU_RUNNING = 10,
	SYSTEM_STATUS_SMCU_STOPPED = 11,
	SYSTEM_STATUS_SMCU_INITIALIZING = 12,
	SYSTEM_STATUS_SMCU_INIT_FAILED = 13,
	SYSTEM_STATUS_SMCU_STOPPED_CMCU_STOPPED = 14,
	SYSTEM_STATUS_SMCU_RUNNING_CMCU_STOPPED = 15,
	SYSTEM_STATUS_SMCU_STOPPED_CMCU_RUNNING = 16,
	SYSTEM_STATUS_READY = 17,
	SYSTEM_STATUS_ENGINES_FAILED = 18,
	SYSTEM_STATUS_ENGINES_ON = 19

} system_status_type_t;

typedef enum
{
LOG_OFF = 1,
LOG_ON = 2,
LOG_ERASE = 3,
LOG_DOWNLOAD = 4,
LOG_IMPOSSIBLE = 5,
LOG_STARTING = 6,
LOG_STOPPING = 7
} log_mode_t;

#define HL_COMMANDS_FARGS_SIZE 12
#define HL_COMMANDS_IARGS_SIZE 4

typedef enum
{
hl_request_flyq,
hl_flyq_reset,
hl_flyq_add_flyto_3d,
hl_flyq_add_flyto_3d_m,
hl_flyq_add_trajectory_segment_3d,
hl_flyq_add_trajectory_segment_3d_m,
hl_flyq_add_wait,
hl_flyq_run,
hl_get_origo
} HlCommandEnum;

#define HL_COMMANDS_RESPONSE_FARGS_SIZE 4
#define HL_COMMANDS_RESPONSE_IARGS_SIZE 4

typedef enum
{
hlcr_ok,
hlcr_queue_busy,
hlcr_invalid_token,
hlcr_progress,
hlcr_not_allowed,
hlcr_queue_run_finishes,
hlcr_not_available,
hlcr_no_response
} HlCommandResponseEnum;

// number conditions to check before switching on engines (how many variables to evaluate)
#define ENGINES_ON_VAR_COUNT 4


#endif
