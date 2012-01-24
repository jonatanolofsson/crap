// This file is generated automatically. Do not change it!
#ifdef __cplusplus
extern "C" {
#endif

#ifndef PACKET_TYPES_H_
#define PACKET_TYPES_H_

#include <stdint.h>

#include "packet_types_defines.h"

typedef struct
{
    uint32_t iReftime;
    int16_t gyro_data[3];
    int16_t gyro_raw[3];
    int16_t gyro_temp_raw[3];
    int16_t accel_raw[3];
    int16_t micromag[3];
    int16_t ahrs[3];
    int16_t compass_heading;
    int16_t rc_sticks[7];
    int16_t battery_voltage;
    int8_t cGPSValid;
    int32_t iLatDeg;
    int32_t iLonDeg;
    float fAlt;
    int32_t iGpsLatDeg;
    int32_t iGpsLonDeg;
    int32_t iGpsAlt;
    int16_t siGpsYawDeg;
    uint8_t ucGpsStatus;
    uint8_t ucGpsStatusFlags;
    uint16_t gDop;
    uint16_t pDop;
    uint16_t hDop;
    uint16_t vDop;
    uint8_t num_SV;
    int16_t iGpsVelN;
    int16_t iGpsVelE;
    int16_t iGpsVelD;
    uint16_t iGpsSAcc;
    uint16_t iGpsPAcc;
    uint16_t iGpsHAcc;
    uint16_t iGpsVAcc;
    uint8_t system_status;
    uint16_t usiError;
    uint8_t remote_controller_status;
    uint16_t adc[3];
    uint8_t ext_uart_crc_error_cnt;
    uint8_t ext_uart_tx_overflow_cnt;
    float fAltCF;
    float fVelU;
    int gps_longitude_origo;
    int gps_latitude_origo;
    int gps_altitude_origo;
    char gps_origo_taken;
    float gps_longitude_m;
    float gps_latitude_m;
    float fHeadCF;
} SSMCUData;

typedef struct
{
    uint16_t everyNth;
    uint8_t ids[66];
    uint8_t ids_cnt;
} SSMCUDataRequestPart;

typedef struct
{
    int8_t id;
    uint32_t time;
    uint32_t logtime;
    float phi;
    float theta;
    float psi;
    float compass_heading;
    float phi_dot_raw;
    float theta_dot_raw;
    float psi_dot_raw;
    float phi_dot;
    float theta_dot;
    float psi_dot;
    int16_t siPhiTemp;
    int16_t siThetaTemp;
    int16_t siPsiTemp;
    float accel_x_raw;
    float accel_y_raw;
    float accel_z_raw;
    float micromag_x;
    float micromag_y;
    float micromag_z;
    float hp;
    float fPressureAltitude;
    int8_t cGPSValid;
    uint8_t cGPSStatus;
    int32_t iGpsLatDeg;
    int32_t iGpsLonDeg;
    uint32_t iGpsAlt;
    uint8_t ucGpsStatusFlags;
    uint16_t gDop;
    uint16_t pDop;
    uint16_t hDop;
    uint16_t vDop;
    uint8_t num_SV;
    int16_t iGpsVelN;
    int16_t iGpsVelE;
    int16_t iGpsVelD;
    uint16_t iGpsSAcc;
    uint16_t iGpsPAcc;
    uint16_t iGpsHAcc;
    uint16_t iGpsVAcc;
    float GpsHeadingDeg;
    uint16_t rc_sticks_0;
    uint16_t rc_sticks_1;
    uint16_t rc_sticks_2;
    uint16_t rc_sticks_3;
    uint16_t rc_sticks_4;
    uint16_t rc_sticks_5;
    uint16_t rc_sticks_6;
    uint16_t servo_output_0;
    uint16_t servo_output_1;
    uint16_t servo_output_2;
    uint16_t servo_output_3;
    uint16_t servo_output_4;
    uint16_t servo_output_5;
    uint16_t servo_output_6;
    int16_t servo_mixer_input_0;
    int16_t servo_mixer_input_1;
    int16_t servo_mixer_input_2;
    int16_t servo_mixer_input_3;
    int16_t servo_mixer_input_4;
    int16_t servo_mixer_input_5;
    int16_t servo_mixer_input_6;
    uint8_t rc_aileron_at_ms;
    uint8_t rc_elevator_at_ms;
    uint8_t rc_rudder_at_ms;
    uint8_t rc_throttle_at_ms;
    uint8_t rc_camera_pan_at_ms;
    uint8_t rc_camera_tilt_at_ms;
    int32_t targetLonDeg;
    int32_t targetLatDeg;
    float targetAltM;
    float targetHeadingDeg;
    int16_t battery_voltage;
    int8_t spi_status;
    uint8_t spi_crc_error_cnt;
    uint8_t spi_tx_overflow_cnt;
    uint8_t serial_1_crc_error_cnt;
    uint8_t serial_1_tx_overflow_cnt;
    uint8_t serial_2_crc_error_cnt;
    uint8_t serial_2_tx_overflow_cnt;
    uint16_t accel_status;
    uint8_t remote_controller_status;
    uint8_t flight_mode;
    uint8_t camera_mode;
    uint8_t logging_mode_status;
    uint8_t logging_progress;
    uint8_t sdcard_status;
    uint8_t system_status;
    uint8_t motor_status;
    uint16_t adc_0;
    uint16_t adc_1;
    uint16_t adc_2;
    float UserParamFloat_0;
    float UserParamFloat_1;
    float UserParamFloat_2;
    float UserParamFloat_3;
    float UserParamFloat_4;
    float UserParamFloat_5;
    float UserParamFloat_6;
    float UserParamFloat_7;
    float UserParamFloat_8;
    float UserParamFloat_9;
    int8_t UserParamInt8_0;
    int8_t UserParamInt8_1;
    int8_t UserParamInt8_2;
    int8_t UserParamInt8_3;
    int8_t UserParamInt8_4;
    int8_t UserParamInt8_5;
    int8_t UserParamInt8_6;
    int8_t UserParamInt8_7;
    int8_t UserParamInt8_8;
    int8_t UserParamInt8_9;
    int16_t UserParamInt16_0;
    int16_t UserParamInt16_1;
    int16_t UserParamInt16_2;
    int16_t UserParamInt16_3;
    int16_t UserParamInt16_4;
    int16_t UserParamInt16_5;
    int16_t UserParamInt16_6;
    int16_t UserParamInt16_7;
    int16_t UserParamInt16_8;
    int16_t UserParamInt16_9;
    int32_t UserParamInt32_0;
    int32_t UserParamInt32_1;
    int32_t UserParamInt32_2;
    int32_t UserParamInt32_3;
    int32_t UserParamInt32_4;
    int32_t UserParamInt32_5;
    int32_t UserParamInt32_6;
    int32_t UserParamInt32_7;
    int32_t UserParamInt32_8;
    int32_t UserParamInt32_9;
    float fAltCF;
    float fVelU;
    int32_t gps_longitude_origo;
    int32_t gps_latitude_origo;
    int32_t gps_altitude_origo;
    float gps_longitude_m;
    float gps_latitude_m;
    char gps_origo_taken;
    float fHeadCF;
    int16_t mixer_input_0_at_fl_ms;
    int16_t mixer_input_1_at_fl_ms;
    int16_t mixer_input_2_at_fl_ms;
    int16_t mixer_input_3_at_fl_ms;
    int16_t mixer_input_4_at_fl_ms;
    int16_t mixer_input_5_at_fl_ms;
    int16_t mixer_input_6_at_fl_ms;
    float outer_loop_output_0;
    float outer_loop_output_1;
    float outer_loop_output_2;
    float outer_loop_output_3;
    float outer_loop_output_4;
    float outer_loop_output_5;
    float outer_loop_output_6;
    float outer_loop_output_7;
} SCMCUData;

typedef struct
{
    uint16_t everyNth;
    uint8_t ids[157];
    uint8_t ids_cnt;
} SCMCUDataRequestPart;

typedef struct
{
    uint64_t flags;
    float params_32f[10];
    int8_t params_8i[10];
    int16_t params_16i[10];
    int32_t params_32i[10];
} SUserParams;

typedef struct
{
    uint8_t flags;
    float longitude;
    float latitude;
    float altitude;
    float heading;
} SFlightTargetParams;

typedef struct
{
    uint16_t everyNth;
    uint8_t ids[(10 + 10 + 10 + 10 + 1)];
    uint8_t ids_cnt;
} SUserParamsRequestPart;

typedef struct
{
    uint8_t cId;
} SCmd;

typedef struct
{
    uint8_t cId;
    int32_t iParam1;
} SCmdParam1Int;

typedef struct
{
    uint8_t cId;
    uint8_t rgcText[64];
} SCmdParamText;

typedef struct
{
    uint8_t msg[16];
} SDebugPrint;

typedef struct
{
    uint8_t msg[16];
    int32_t iValue;
} SDebugPrintI;

typedef struct
{
    HlCommandEnum command;
    float fargs[12];
    int32_t iargs[4];
    int32_t uuid;
    uint8_t token;
} HlCommandRequest;

typedef struct
{
    int uuid;
    HlCommandResponseEnum status;
    uint8_t token;
    float fargs[4];
    int32_t iargs[4];
} HlCommandResponse;

// Variable info functions
void getVarInfoSSMCUData(SSMCUData *in_data, uint8_t n, void **p, uint8_t *out_size);
void getVarInfoSCMCUData(SCMCUData *in_data, uint8_t n, void **p, uint8_t *out_size);
void getVarInfoSUserParams(SUserParams *in_data, uint8_t n, void **p, uint8_t *out_size);

// Marshal partial functions
void marshalPartSSMCUData(uint8_t mode, SSMCUData *in_data, uint8_t *ids, uint8_t ids_cnt, uint8_t *out_data, uint8_t *offset);
void marshalPartSCMCUData(uint8_t mode, SCMCUData *in_data, uint8_t *ids, uint8_t ids_cnt, uint8_t *out_data, uint8_t *offset);
void marshalPartSUserParams(uint8_t mode, SUserParams *in_data, uint8_t *ids, uint8_t ids_cnt, uint8_t *out_data, uint8_t *offset);

// Marshal functions
void marshalSSMCUData(SSMCUData *in_data, uint8_t *out_data, uint8_t *offset);
void marshalSSMCUDataRequestPart(SSMCUDataRequestPart *in_data, uint8_t *out_data, uint8_t *offset);
void marshalSCMCUData(SCMCUData *in_data, uint8_t *out_data, uint8_t *offset);
void marshalSCMCUDataRequestPart(SCMCUDataRequestPart *in_data, uint8_t *out_data, uint8_t *offset);
void marshalSUserParams(SUserParams *in_data, uint8_t *out_data, uint8_t *offset);
void marshalSFlightTargetParams(SFlightTargetParams *in_data, uint8_t *out_data, uint8_t *offset);
void marshalSUserParamsRequestPart(SUserParamsRequestPart *in_data, uint8_t *out_data, uint8_t *offset);
void marshalSCmd(SCmd *in_data, uint8_t *out_data, uint8_t *offset);
void marshalSCmdParam1Int(SCmdParam1Int *in_data, uint8_t *out_data, uint8_t *offset);
void marshalSCmdParamText(SCmdParamText *in_data, uint8_t *out_data, uint8_t *offset);
void marshalSDebugPrint(SDebugPrint *in_data, uint8_t *out_data, uint8_t *offset);
void marshalSDebugPrintI(SDebugPrintI *in_data, uint8_t *out_data, uint8_t *offset);
void marshalHlCommandRequest(HlCommandRequest *in_data, uint8_t *out_data, uint8_t *offset);
void marshalHlCommandResponse(HlCommandResponse *in_data, uint8_t *out_data, uint8_t *offset);

// Unmarshal functions
void unmarshalSSMCUData(uint8_t *in_data, SSMCUData *out_data);
void unmarshalSSMCUDataRequestPart(uint8_t *in_data, SSMCUDataRequestPart *out_data);
void unmarshalSCMCUData(uint8_t *in_data, SCMCUData *out_data);
void unmarshalSCMCUDataRequestPart(uint8_t *in_data, SCMCUDataRequestPart *out_data);
void unmarshalSUserParams(uint8_t *in_data, SUserParams *out_data);
void unmarshalSFlightTargetParams(uint8_t *in_data, SFlightTargetParams *out_data);
void unmarshalSUserParamsRequestPart(uint8_t *in_data, SUserParamsRequestPart *out_data);
void unmarshalSCmd(uint8_t *in_data, SCmd *out_data);
void unmarshalSCmdParam1Int(uint8_t *in_data, SCmdParam1Int *out_data);
void unmarshalSCmdParamText(uint8_t *in_data, SCmdParamText *out_data);
void unmarshalSDebugPrint(uint8_t *in_data, SDebugPrint *out_data);
void unmarshalSDebugPrintI(uint8_t *in_data, SDebugPrintI *out_data);
void unmarshalHlCommandRequest(uint8_t *in_data, HlCommandRequest *out_data);
void unmarshalHlCommandResponse(uint8_t *in_data, HlCommandResponse *out_data);

// Size functions
uint16_t getSizeSSMCUData(void);
uint16_t getSizeSSMCUDataRequestPart(void);
uint16_t getSizeSCMCUData(void);
uint16_t getSizeSCMCUDataRequestPart(void);
uint16_t getSizeSUserParams(void);
uint16_t getSizeSFlightTargetParams(void);
uint16_t getSizeSUserParamsRequestPart(void);
uint16_t getSizeSCmd(void);
uint16_t getSizeSCmdParam1Int(void);
uint16_t getSizeSCmdParamText(void);
uint16_t getSizeSDebugPrint(void);
uint16_t getSizeSDebugPrintI(void);
uint16_t getSizeHlCommandRequest(void);
uint16_t getSizeHlCommandResponse(void);


#endif /* PACKET_TYPES_H_ */

#ifdef __cplusplus
}
#endif
