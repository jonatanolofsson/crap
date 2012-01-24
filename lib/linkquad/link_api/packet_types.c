// This file is generated automatically. Do not change it!
#include "packet_types.h"
#include <string.h> 

void getVarInfoSSMCUData(SSMCUData *in_data, uint8_t n, void **p, uint8_t *out_size)
{
    switch (n){
    case 0:
        *out_size = sizeof(in_data->iReftime);
        *p = (void*)&in_data->iReftime;
        break;
    case 1:
        *out_size = sizeof(in_data->gyro_data[0]);
        *p = (void*)&in_data->gyro_data[0];
        break;
    case 2:
        *out_size = sizeof(in_data->gyro_data[1]);
        *p = (void*)&in_data->gyro_data[1];
        break;
    case 3:
        *out_size = sizeof(in_data->gyro_data[2]);
        *p = (void*)&in_data->gyro_data[2];
        break;
    case 4:
        *out_size = sizeof(in_data->gyro_raw[0]);
        *p = (void*)&in_data->gyro_raw[0];
        break;
    case 5:
        *out_size = sizeof(in_data->gyro_raw[1]);
        *p = (void*)&in_data->gyro_raw[1];
        break;
    case 6:
        *out_size = sizeof(in_data->gyro_raw[2]);
        *p = (void*)&in_data->gyro_raw[2];
        break;
    case 7:
        *out_size = sizeof(in_data->gyro_temp_raw[0]);
        *p = (void*)&in_data->gyro_temp_raw[0];
        break;
    case 8:
        *out_size = sizeof(in_data->gyro_temp_raw[1]);
        *p = (void*)&in_data->gyro_temp_raw[1];
        break;
    case 9:
        *out_size = sizeof(in_data->gyro_temp_raw[2]);
        *p = (void*)&in_data->gyro_temp_raw[2];
        break;
    case 10:
        *out_size = sizeof(in_data->accel_raw[0]);
        *p = (void*)&in_data->accel_raw[0];
        break;
    case 11:
        *out_size = sizeof(in_data->accel_raw[1]);
        *p = (void*)&in_data->accel_raw[1];
        break;
    case 12:
        *out_size = sizeof(in_data->accel_raw[2]);
        *p = (void*)&in_data->accel_raw[2];
        break;
    case 13:
        *out_size = sizeof(in_data->micromag[0]);
        *p = (void*)&in_data->micromag[0];
        break;
    case 14:
        *out_size = sizeof(in_data->micromag[1]);
        *p = (void*)&in_data->micromag[1];
        break;
    case 15:
        *out_size = sizeof(in_data->micromag[2]);
        *p = (void*)&in_data->micromag[2];
        break;
    case 16:
        *out_size = sizeof(in_data->ahrs[0]);
        *p = (void*)&in_data->ahrs[0];
        break;
    case 17:
        *out_size = sizeof(in_data->ahrs[1]);
        *p = (void*)&in_data->ahrs[1];
        break;
    case 18:
        *out_size = sizeof(in_data->ahrs[2]);
        *p = (void*)&in_data->ahrs[2];
        break;
    case 19:
        *out_size = sizeof(in_data->compass_heading);
        *p = (void*)&in_data->compass_heading;
        break;
    case 20:
        *out_size = sizeof(in_data->rc_sticks[0]);
        *p = (void*)&in_data->rc_sticks[0];
        break;
    case 21:
        *out_size = sizeof(in_data->rc_sticks[1]);
        *p = (void*)&in_data->rc_sticks[1];
        break;
    case 22:
        *out_size = sizeof(in_data->rc_sticks[2]);
        *p = (void*)&in_data->rc_sticks[2];
        break;
    case 23:
        *out_size = sizeof(in_data->rc_sticks[3]);
        *p = (void*)&in_data->rc_sticks[3];
        break;
    case 24:
        *out_size = sizeof(in_data->rc_sticks[4]);
        *p = (void*)&in_data->rc_sticks[4];
        break;
    case 25:
        *out_size = sizeof(in_data->rc_sticks[5]);
        *p = (void*)&in_data->rc_sticks[5];
        break;
    case 26:
        *out_size = sizeof(in_data->rc_sticks[6]);
        *p = (void*)&in_data->rc_sticks[6];
        break;
    case 27:
        *out_size = sizeof(in_data->battery_voltage);
        *p = (void*)&in_data->battery_voltage;
        break;
    case 28:
        *out_size = sizeof(in_data->cGPSValid);
        *p = (void*)&in_data->cGPSValid;
        break;
    case 29:
        *out_size = sizeof(in_data->iLatDeg);
        *p = (void*)&in_data->iLatDeg;
        break;
    case 30:
        *out_size = sizeof(in_data->iLonDeg);
        *p = (void*)&in_data->iLonDeg;
        break;
    case 31:
        *out_size = sizeof(in_data->fAlt);
        *p = (void*)&in_data->fAlt;
        break;
    case 32:
        *out_size = sizeof(in_data->iGpsLatDeg);
        *p = (void*)&in_data->iGpsLatDeg;
        break;
    case 33:
        *out_size = sizeof(in_data->iGpsLonDeg);
        *p = (void*)&in_data->iGpsLonDeg;
        break;
    case 34:
        *out_size = sizeof(in_data->iGpsAlt);
        *p = (void*)&in_data->iGpsAlt;
        break;
    case 35:
        *out_size = sizeof(in_data->siGpsYawDeg);
        *p = (void*)&in_data->siGpsYawDeg;
        break;
    case 36:
        *out_size = sizeof(in_data->ucGpsStatus);
        *p = (void*)&in_data->ucGpsStatus;
        break;
    case 37:
        *out_size = sizeof(in_data->ucGpsStatusFlags);
        *p = (void*)&in_data->ucGpsStatusFlags;
        break;
    case 38:
        *out_size = sizeof(in_data->gDop);
        *p = (void*)&in_data->gDop;
        break;
    case 39:
        *out_size = sizeof(in_data->pDop);
        *p = (void*)&in_data->pDop;
        break;
    case 40:
        *out_size = sizeof(in_data->hDop);
        *p = (void*)&in_data->hDop;
        break;
    case 41:
        *out_size = sizeof(in_data->vDop);
        *p = (void*)&in_data->vDop;
        break;
    case 42:
        *out_size = sizeof(in_data->num_SV);
        *p = (void*)&in_data->num_SV;
        break;
    case 43:
        *out_size = sizeof(in_data->iGpsVelN);
        *p = (void*)&in_data->iGpsVelN;
        break;
    case 44:
        *out_size = sizeof(in_data->iGpsVelE);
        *p = (void*)&in_data->iGpsVelE;
        break;
    case 45:
        *out_size = sizeof(in_data->iGpsVelD);
        *p = (void*)&in_data->iGpsVelD;
        break;
    case 46:
        *out_size = sizeof(in_data->iGpsSAcc);
        *p = (void*)&in_data->iGpsSAcc;
        break;
    case 47:
        *out_size = sizeof(in_data->iGpsPAcc);
        *p = (void*)&in_data->iGpsPAcc;
        break;
    case 48:
        *out_size = sizeof(in_data->iGpsHAcc);
        *p = (void*)&in_data->iGpsHAcc;
        break;
    case 49:
        *out_size = sizeof(in_data->iGpsVAcc);
        *p = (void*)&in_data->iGpsVAcc;
        break;
    case 50:
        *out_size = sizeof(in_data->system_status);
        *p = (void*)&in_data->system_status;
        break;
    case 51:
        *out_size = sizeof(in_data->usiError);
        *p = (void*)&in_data->usiError;
        break;
    case 52:
        *out_size = sizeof(in_data->remote_controller_status);
        *p = (void*)&in_data->remote_controller_status;
        break;
    case 53:
        *out_size = sizeof(in_data->adc[0]);
        *p = (void*)&in_data->adc[0];
        break;
    case 54:
        *out_size = sizeof(in_data->adc[1]);
        *p = (void*)&in_data->adc[1];
        break;
    case 55:
        *out_size = sizeof(in_data->adc[2]);
        *p = (void*)&in_data->adc[2];
        break;
    case 56:
        *out_size = sizeof(in_data->ext_uart_crc_error_cnt);
        *p = (void*)&in_data->ext_uart_crc_error_cnt;
        break;
    case 57:
        *out_size = sizeof(in_data->ext_uart_tx_overflow_cnt);
        *p = (void*)&in_data->ext_uart_tx_overflow_cnt;
        break;
    case 58:
        *out_size = sizeof(in_data->fAltCF);
        *p = (void*)&in_data->fAltCF;
        break;
    case 59:
        *out_size = sizeof(in_data->fVelU);
        *p = (void*)&in_data->fVelU;
        break;
    case 60:
        *out_size = sizeof(in_data->gps_longitude_origo);
        *p = (void*)&in_data->gps_longitude_origo;
        break;
    case 61:
        *out_size = sizeof(in_data->gps_latitude_origo);
        *p = (void*)&in_data->gps_latitude_origo;
        break;
    case 62:
        *out_size = sizeof(in_data->gps_altitude_origo);
        *p = (void*)&in_data->gps_altitude_origo;
        break;
    case 63:
        *out_size = sizeof(in_data->gps_origo_taken);
        *p = (void*)&in_data->gps_origo_taken;
        break;
    case 64:
        *out_size = sizeof(in_data->gps_longitude_m);
        *p = (void*)&in_data->gps_longitude_m;
        break;
    case 65:
        *out_size = sizeof(in_data->gps_latitude_m);
        *p = (void*)&in_data->gps_latitude_m;
        break;
    case 66:
        *out_size = sizeof(in_data->fHeadCF);
        *p = (void*)&in_data->fHeadCF;
        break;

    }
}

void getVarInfoSCMCUData(SCMCUData *in_data, uint8_t n, void **p, uint8_t *out_size)
{
    switch (n){
    case 0:
        *out_size = sizeof(in_data->id);
        *p = (void*)&in_data->id;
        break;
    case 1:
        *out_size = sizeof(in_data->time);
        *p = (void*)&in_data->time;
        break;
    case 2:
        *out_size = sizeof(in_data->logtime);
        *p = (void*)&in_data->logtime;
        break;
    case 3:
        *out_size = sizeof(in_data->phi);
        *p = (void*)&in_data->phi;
        break;
    case 4:
        *out_size = sizeof(in_data->theta);
        *p = (void*)&in_data->theta;
        break;
    case 5:
        *out_size = sizeof(in_data->psi);
        *p = (void*)&in_data->psi;
        break;
    case 6:
        *out_size = sizeof(in_data->compass_heading);
        *p = (void*)&in_data->compass_heading;
        break;
    case 7:
        *out_size = sizeof(in_data->phi_dot_raw);
        *p = (void*)&in_data->phi_dot_raw;
        break;
    case 8:
        *out_size = sizeof(in_data->theta_dot_raw);
        *p = (void*)&in_data->theta_dot_raw;
        break;
    case 9:
        *out_size = sizeof(in_data->psi_dot_raw);
        *p = (void*)&in_data->psi_dot_raw;
        break;
    case 10:
        *out_size = sizeof(in_data->phi_dot);
        *p = (void*)&in_data->phi_dot;
        break;
    case 11:
        *out_size = sizeof(in_data->theta_dot);
        *p = (void*)&in_data->theta_dot;
        break;
    case 12:
        *out_size = sizeof(in_data->psi_dot);
        *p = (void*)&in_data->psi_dot;
        break;
    case 13:
        *out_size = sizeof(in_data->siPhiTemp);
        *p = (void*)&in_data->siPhiTemp;
        break;
    case 14:
        *out_size = sizeof(in_data->siThetaTemp);
        *p = (void*)&in_data->siThetaTemp;
        break;
    case 15:
        *out_size = sizeof(in_data->siPsiTemp);
        *p = (void*)&in_data->siPsiTemp;
        break;
    case 16:
        *out_size = sizeof(in_data->accel_x_raw);
        *p = (void*)&in_data->accel_x_raw;
        break;
    case 17:
        *out_size = sizeof(in_data->accel_y_raw);
        *p = (void*)&in_data->accel_y_raw;
        break;
    case 18:
        *out_size = sizeof(in_data->accel_z_raw);
        *p = (void*)&in_data->accel_z_raw;
        break;
    case 19:
        *out_size = sizeof(in_data->micromag_x);
        *p = (void*)&in_data->micromag_x;
        break;
    case 20:
        *out_size = sizeof(in_data->micromag_y);
        *p = (void*)&in_data->micromag_y;
        break;
    case 21:
        *out_size = sizeof(in_data->micromag_z);
        *p = (void*)&in_data->micromag_z;
        break;
    case 22:
        *out_size = sizeof(in_data->hp);
        *p = (void*)&in_data->hp;
        break;
    case 23:
        *out_size = sizeof(in_data->fPressureAltitude);
        *p = (void*)&in_data->fPressureAltitude;
        break;
    case 24:
        *out_size = sizeof(in_data->cGPSValid);
        *p = (void*)&in_data->cGPSValid;
        break;
    case 25:
        *out_size = sizeof(in_data->cGPSStatus);
        *p = (void*)&in_data->cGPSStatus;
        break;
    case 26:
        *out_size = sizeof(in_data->iGpsLatDeg);
        *p = (void*)&in_data->iGpsLatDeg;
        break;
    case 27:
        *out_size = sizeof(in_data->iGpsLonDeg);
        *p = (void*)&in_data->iGpsLonDeg;
        break;
    case 28:
        *out_size = sizeof(in_data->iGpsAlt);
        *p = (void*)&in_data->iGpsAlt;
        break;
    case 29:
        *out_size = sizeof(in_data->ucGpsStatusFlags);
        *p = (void*)&in_data->ucGpsStatusFlags;
        break;
    case 30:
        *out_size = sizeof(in_data->gDop);
        *p = (void*)&in_data->gDop;
        break;
    case 31:
        *out_size = sizeof(in_data->pDop);
        *p = (void*)&in_data->pDop;
        break;
    case 32:
        *out_size = sizeof(in_data->hDop);
        *p = (void*)&in_data->hDop;
        break;
    case 33:
        *out_size = sizeof(in_data->vDop);
        *p = (void*)&in_data->vDop;
        break;
    case 34:
        *out_size = sizeof(in_data->num_SV);
        *p = (void*)&in_data->num_SV;
        break;
    case 35:
        *out_size = sizeof(in_data->iGpsVelN);
        *p = (void*)&in_data->iGpsVelN;
        break;
    case 36:
        *out_size = sizeof(in_data->iGpsVelE);
        *p = (void*)&in_data->iGpsVelE;
        break;
    case 37:
        *out_size = sizeof(in_data->iGpsVelD);
        *p = (void*)&in_data->iGpsVelD;
        break;
    case 38:
        *out_size = sizeof(in_data->iGpsSAcc);
        *p = (void*)&in_data->iGpsSAcc;
        break;
    case 39:
        *out_size = sizeof(in_data->iGpsPAcc);
        *p = (void*)&in_data->iGpsPAcc;
        break;
    case 40:
        *out_size = sizeof(in_data->iGpsHAcc);
        *p = (void*)&in_data->iGpsHAcc;
        break;
    case 41:
        *out_size = sizeof(in_data->iGpsVAcc);
        *p = (void*)&in_data->iGpsVAcc;
        break;
    case 42:
        *out_size = sizeof(in_data->GpsHeadingDeg);
        *p = (void*)&in_data->GpsHeadingDeg;
        break;
    case 43:
        *out_size = sizeof(in_data->rc_sticks_0);
        *p = (void*)&in_data->rc_sticks_0;
        break;
    case 44:
        *out_size = sizeof(in_data->rc_sticks_1);
        *p = (void*)&in_data->rc_sticks_1;
        break;
    case 45:
        *out_size = sizeof(in_data->rc_sticks_2);
        *p = (void*)&in_data->rc_sticks_2;
        break;
    case 46:
        *out_size = sizeof(in_data->rc_sticks_3);
        *p = (void*)&in_data->rc_sticks_3;
        break;
    case 47:
        *out_size = sizeof(in_data->rc_sticks_4);
        *p = (void*)&in_data->rc_sticks_4;
        break;
    case 48:
        *out_size = sizeof(in_data->rc_sticks_5);
        *p = (void*)&in_data->rc_sticks_5;
        break;
    case 49:
        *out_size = sizeof(in_data->rc_sticks_6);
        *p = (void*)&in_data->rc_sticks_6;
        break;
    case 50:
        *out_size = sizeof(in_data->servo_output_0);
        *p = (void*)&in_data->servo_output_0;
        break;
    case 51:
        *out_size = sizeof(in_data->servo_output_1);
        *p = (void*)&in_data->servo_output_1;
        break;
    case 52:
        *out_size = sizeof(in_data->servo_output_2);
        *p = (void*)&in_data->servo_output_2;
        break;
    case 53:
        *out_size = sizeof(in_data->servo_output_3);
        *p = (void*)&in_data->servo_output_3;
        break;
    case 54:
        *out_size = sizeof(in_data->servo_output_4);
        *p = (void*)&in_data->servo_output_4;
        break;
    case 55:
        *out_size = sizeof(in_data->servo_output_5);
        *p = (void*)&in_data->servo_output_5;
        break;
    case 56:
        *out_size = sizeof(in_data->servo_output_6);
        *p = (void*)&in_data->servo_output_6;
        break;
    case 57:
        *out_size = sizeof(in_data->servo_mixer_input_0);
        *p = (void*)&in_data->servo_mixer_input_0;
        break;
    case 58:
        *out_size = sizeof(in_data->servo_mixer_input_1);
        *p = (void*)&in_data->servo_mixer_input_1;
        break;
    case 59:
        *out_size = sizeof(in_data->servo_mixer_input_2);
        *p = (void*)&in_data->servo_mixer_input_2;
        break;
    case 60:
        *out_size = sizeof(in_data->servo_mixer_input_3);
        *p = (void*)&in_data->servo_mixer_input_3;
        break;
    case 61:
        *out_size = sizeof(in_data->servo_mixer_input_4);
        *p = (void*)&in_data->servo_mixer_input_4;
        break;
    case 62:
        *out_size = sizeof(in_data->servo_mixer_input_5);
        *p = (void*)&in_data->servo_mixer_input_5;
        break;
    case 63:
        *out_size = sizeof(in_data->servo_mixer_input_6);
        *p = (void*)&in_data->servo_mixer_input_6;
        break;
    case 64:
        *out_size = sizeof(in_data->rc_aileron_at_ms);
        *p = (void*)&in_data->rc_aileron_at_ms;
        break;
    case 65:
        *out_size = sizeof(in_data->rc_elevator_at_ms);
        *p = (void*)&in_data->rc_elevator_at_ms;
        break;
    case 66:
        *out_size = sizeof(in_data->rc_rudder_at_ms);
        *p = (void*)&in_data->rc_rudder_at_ms;
        break;
    case 67:
        *out_size = sizeof(in_data->rc_throttle_at_ms);
        *p = (void*)&in_data->rc_throttle_at_ms;
        break;
    case 68:
        *out_size = sizeof(in_data->rc_camera_pan_at_ms);
        *p = (void*)&in_data->rc_camera_pan_at_ms;
        break;
    case 69:
        *out_size = sizeof(in_data->rc_camera_tilt_at_ms);
        *p = (void*)&in_data->rc_camera_tilt_at_ms;
        break;
    case 70:
        *out_size = sizeof(in_data->targetLonDeg);
        *p = (void*)&in_data->targetLonDeg;
        break;
    case 71:
        *out_size = sizeof(in_data->targetLatDeg);
        *p = (void*)&in_data->targetLatDeg;
        break;
    case 72:
        *out_size = sizeof(in_data->targetAltM);
        *p = (void*)&in_data->targetAltM;
        break;
    case 73:
        *out_size = sizeof(in_data->targetHeadingDeg);
        *p = (void*)&in_data->targetHeadingDeg;
        break;
    case 74:
        *out_size = sizeof(in_data->battery_voltage);
        *p = (void*)&in_data->battery_voltage;
        break;
    case 75:
        *out_size = sizeof(in_data->spi_status);
        *p = (void*)&in_data->spi_status;
        break;
    case 76:
        *out_size = sizeof(in_data->spi_crc_error_cnt);
        *p = (void*)&in_data->spi_crc_error_cnt;
        break;
    case 77:
        *out_size = sizeof(in_data->spi_tx_overflow_cnt);
        *p = (void*)&in_data->spi_tx_overflow_cnt;
        break;
    case 78:
        *out_size = sizeof(in_data->serial_1_crc_error_cnt);
        *p = (void*)&in_data->serial_1_crc_error_cnt;
        break;
    case 79:
        *out_size = sizeof(in_data->serial_1_tx_overflow_cnt);
        *p = (void*)&in_data->serial_1_tx_overflow_cnt;
        break;
    case 80:
        *out_size = sizeof(in_data->serial_2_crc_error_cnt);
        *p = (void*)&in_data->serial_2_crc_error_cnt;
        break;
    case 81:
        *out_size = sizeof(in_data->serial_2_tx_overflow_cnt);
        *p = (void*)&in_data->serial_2_tx_overflow_cnt;
        break;
    case 82:
        *out_size = sizeof(in_data->accel_status);
        *p = (void*)&in_data->accel_status;
        break;
    case 83:
        *out_size = sizeof(in_data->remote_controller_status);
        *p = (void*)&in_data->remote_controller_status;
        break;
    case 84:
        *out_size = sizeof(in_data->flight_mode);
        *p = (void*)&in_data->flight_mode;
        break;
    case 85:
        *out_size = sizeof(in_data->camera_mode);
        *p = (void*)&in_data->camera_mode;
        break;
    case 86:
        *out_size = sizeof(in_data->logging_mode_status);
        *p = (void*)&in_data->logging_mode_status;
        break;
    case 87:
        *out_size = sizeof(in_data->logging_progress);
        *p = (void*)&in_data->logging_progress;
        break;
    case 88:
        *out_size = sizeof(in_data->sdcard_status);
        *p = (void*)&in_data->sdcard_status;
        break;
    case 89:
        *out_size = sizeof(in_data->system_status);
        *p = (void*)&in_data->system_status;
        break;
    case 90:
        *out_size = sizeof(in_data->motor_status);
        *p = (void*)&in_data->motor_status;
        break;
    case 91:
        *out_size = sizeof(in_data->adc_0);
        *p = (void*)&in_data->adc_0;
        break;
    case 92:
        *out_size = sizeof(in_data->adc_1);
        *p = (void*)&in_data->adc_1;
        break;
    case 93:
        *out_size = sizeof(in_data->adc_2);
        *p = (void*)&in_data->adc_2;
        break;
    case 94:
        *out_size = sizeof(in_data->UserParamFloat_0);
        *p = (void*)&in_data->UserParamFloat_0;
        break;
    case 95:
        *out_size = sizeof(in_data->UserParamFloat_1);
        *p = (void*)&in_data->UserParamFloat_1;
        break;
    case 96:
        *out_size = sizeof(in_data->UserParamFloat_2);
        *p = (void*)&in_data->UserParamFloat_2;
        break;
    case 97:
        *out_size = sizeof(in_data->UserParamFloat_3);
        *p = (void*)&in_data->UserParamFloat_3;
        break;
    case 98:
        *out_size = sizeof(in_data->UserParamFloat_4);
        *p = (void*)&in_data->UserParamFloat_4;
        break;
    case 99:
        *out_size = sizeof(in_data->UserParamFloat_5);
        *p = (void*)&in_data->UserParamFloat_5;
        break;
    case 100:
        *out_size = sizeof(in_data->UserParamFloat_6);
        *p = (void*)&in_data->UserParamFloat_6;
        break;
    case 101:
        *out_size = sizeof(in_data->UserParamFloat_7);
        *p = (void*)&in_data->UserParamFloat_7;
        break;
    case 102:
        *out_size = sizeof(in_data->UserParamFloat_8);
        *p = (void*)&in_data->UserParamFloat_8;
        break;
    case 103:
        *out_size = sizeof(in_data->UserParamFloat_9);
        *p = (void*)&in_data->UserParamFloat_9;
        break;
    case 104:
        *out_size = sizeof(in_data->UserParamInt8_0);
        *p = (void*)&in_data->UserParamInt8_0;
        break;
    case 105:
        *out_size = sizeof(in_data->UserParamInt8_1);
        *p = (void*)&in_data->UserParamInt8_1;
        break;
    case 106:
        *out_size = sizeof(in_data->UserParamInt8_2);
        *p = (void*)&in_data->UserParamInt8_2;
        break;
    case 107:
        *out_size = sizeof(in_data->UserParamInt8_3);
        *p = (void*)&in_data->UserParamInt8_3;
        break;
    case 108:
        *out_size = sizeof(in_data->UserParamInt8_4);
        *p = (void*)&in_data->UserParamInt8_4;
        break;
    case 109:
        *out_size = sizeof(in_data->UserParamInt8_5);
        *p = (void*)&in_data->UserParamInt8_5;
        break;
    case 110:
        *out_size = sizeof(in_data->UserParamInt8_6);
        *p = (void*)&in_data->UserParamInt8_6;
        break;
    case 111:
        *out_size = sizeof(in_data->UserParamInt8_7);
        *p = (void*)&in_data->UserParamInt8_7;
        break;
    case 112:
        *out_size = sizeof(in_data->UserParamInt8_8);
        *p = (void*)&in_data->UserParamInt8_8;
        break;
    case 113:
        *out_size = sizeof(in_data->UserParamInt8_9);
        *p = (void*)&in_data->UserParamInt8_9;
        break;
    case 114:
        *out_size = sizeof(in_data->UserParamInt16_0);
        *p = (void*)&in_data->UserParamInt16_0;
        break;
    case 115:
        *out_size = sizeof(in_data->UserParamInt16_1);
        *p = (void*)&in_data->UserParamInt16_1;
        break;
    case 116:
        *out_size = sizeof(in_data->UserParamInt16_2);
        *p = (void*)&in_data->UserParamInt16_2;
        break;
    case 117:
        *out_size = sizeof(in_data->UserParamInt16_3);
        *p = (void*)&in_data->UserParamInt16_3;
        break;
    case 118:
        *out_size = sizeof(in_data->UserParamInt16_4);
        *p = (void*)&in_data->UserParamInt16_4;
        break;
    case 119:
        *out_size = sizeof(in_data->UserParamInt16_5);
        *p = (void*)&in_data->UserParamInt16_5;
        break;
    case 120:
        *out_size = sizeof(in_data->UserParamInt16_6);
        *p = (void*)&in_data->UserParamInt16_6;
        break;
    case 121:
        *out_size = sizeof(in_data->UserParamInt16_7);
        *p = (void*)&in_data->UserParamInt16_7;
        break;
    case 122:
        *out_size = sizeof(in_data->UserParamInt16_8);
        *p = (void*)&in_data->UserParamInt16_8;
        break;
    case 123:
        *out_size = sizeof(in_data->UserParamInt16_9);
        *p = (void*)&in_data->UserParamInt16_9;
        break;
    case 124:
        *out_size = sizeof(in_data->UserParamInt32_0);
        *p = (void*)&in_data->UserParamInt32_0;
        break;
    case 125:
        *out_size = sizeof(in_data->UserParamInt32_1);
        *p = (void*)&in_data->UserParamInt32_1;
        break;
    case 126:
        *out_size = sizeof(in_data->UserParamInt32_2);
        *p = (void*)&in_data->UserParamInt32_2;
        break;
    case 127:
        *out_size = sizeof(in_data->UserParamInt32_3);
        *p = (void*)&in_data->UserParamInt32_3;
        break;
    case 128:
        *out_size = sizeof(in_data->UserParamInt32_4);
        *p = (void*)&in_data->UserParamInt32_4;
        break;
    case 129:
        *out_size = sizeof(in_data->UserParamInt32_5);
        *p = (void*)&in_data->UserParamInt32_5;
        break;
    case 130:
        *out_size = sizeof(in_data->UserParamInt32_6);
        *p = (void*)&in_data->UserParamInt32_6;
        break;
    case 131:
        *out_size = sizeof(in_data->UserParamInt32_7);
        *p = (void*)&in_data->UserParamInt32_7;
        break;
    case 132:
        *out_size = sizeof(in_data->UserParamInt32_8);
        *p = (void*)&in_data->UserParamInt32_8;
        break;
    case 133:
        *out_size = sizeof(in_data->UserParamInt32_9);
        *p = (void*)&in_data->UserParamInt32_9;
        break;
    case 134:
        *out_size = sizeof(in_data->fAltCF);
        *p = (void*)&in_data->fAltCF;
        break;
    case 135:
        *out_size = sizeof(in_data->fVelU);
        *p = (void*)&in_data->fVelU;
        break;
    case 136:
        *out_size = sizeof(in_data->gps_longitude_origo);
        *p = (void*)&in_data->gps_longitude_origo;
        break;
    case 137:
        *out_size = sizeof(in_data->gps_latitude_origo);
        *p = (void*)&in_data->gps_latitude_origo;
        break;
    case 138:
        *out_size = sizeof(in_data->gps_altitude_origo);
        *p = (void*)&in_data->gps_altitude_origo;
        break;
    case 139:
        *out_size = sizeof(in_data->gps_longitude_m);
        *p = (void*)&in_data->gps_longitude_m;
        break;
    case 140:
        *out_size = sizeof(in_data->gps_latitude_m);
        *p = (void*)&in_data->gps_latitude_m;
        break;
    case 141:
        *out_size = sizeof(in_data->gps_origo_taken);
        *p = (void*)&in_data->gps_origo_taken;
        break;
    case 142:
        *out_size = sizeof(in_data->fHeadCF);
        *p = (void*)&in_data->fHeadCF;
        break;
    case 143:
        *out_size = sizeof(in_data->mixer_input_0_at_fl_ms);
        *p = (void*)&in_data->mixer_input_0_at_fl_ms;
        break;
    case 144:
        *out_size = sizeof(in_data->mixer_input_1_at_fl_ms);
        *p = (void*)&in_data->mixer_input_1_at_fl_ms;
        break;
    case 145:
        *out_size = sizeof(in_data->mixer_input_2_at_fl_ms);
        *p = (void*)&in_data->mixer_input_2_at_fl_ms;
        break;
    case 146:
        *out_size = sizeof(in_data->mixer_input_3_at_fl_ms);
        *p = (void*)&in_data->mixer_input_3_at_fl_ms;
        break;
    case 147:
        *out_size = sizeof(in_data->mixer_input_4_at_fl_ms);
        *p = (void*)&in_data->mixer_input_4_at_fl_ms;
        break;
    case 148:
        *out_size = sizeof(in_data->mixer_input_5_at_fl_ms);
        *p = (void*)&in_data->mixer_input_5_at_fl_ms;
        break;
    case 149:
        *out_size = sizeof(in_data->mixer_input_6_at_fl_ms);
        *p = (void*)&in_data->mixer_input_6_at_fl_ms;
        break;
    case 150:
        *out_size = sizeof(in_data->outer_loop_output_0);
        *p = (void*)&in_data->outer_loop_output_0;
        break;
    case 151:
        *out_size = sizeof(in_data->outer_loop_output_1);
        *p = (void*)&in_data->outer_loop_output_1;
        break;
    case 152:
        *out_size = sizeof(in_data->outer_loop_output_2);
        *p = (void*)&in_data->outer_loop_output_2;
        break;
    case 153:
        *out_size = sizeof(in_data->outer_loop_output_3);
        *p = (void*)&in_data->outer_loop_output_3;
        break;
    case 154:
        *out_size = sizeof(in_data->outer_loop_output_4);
        *p = (void*)&in_data->outer_loop_output_4;
        break;
    case 155:
        *out_size = sizeof(in_data->outer_loop_output_5);
        *p = (void*)&in_data->outer_loop_output_5;
        break;
    case 156:
        *out_size = sizeof(in_data->outer_loop_output_6);
        *p = (void*)&in_data->outer_loop_output_6;
        break;
    case 157:
        *out_size = sizeof(in_data->outer_loop_output_7);
        *p = (void*)&in_data->outer_loop_output_7;
        break;

    }
}

void getVarInfoSUserParams(SUserParams *in_data, uint8_t n, void **p, uint8_t *out_size)
{
    switch (n){
    case 0:
        *out_size = sizeof(in_data->flags);
        *p = (void*)&in_data->flags;
        break;
    case 1:
        *out_size = sizeof(in_data->params_32f[0]);
        *p = (void*)&in_data->params_32f[0];
        break;
    case 2:
        *out_size = sizeof(in_data->params_32f[1]);
        *p = (void*)&in_data->params_32f[1];
        break;
    case 3:
        *out_size = sizeof(in_data->params_32f[2]);
        *p = (void*)&in_data->params_32f[2];
        break;
    case 4:
        *out_size = sizeof(in_data->params_32f[3]);
        *p = (void*)&in_data->params_32f[3];
        break;
    case 5:
        *out_size = sizeof(in_data->params_32f[4]);
        *p = (void*)&in_data->params_32f[4];
        break;
    case 6:
        *out_size = sizeof(in_data->params_32f[5]);
        *p = (void*)&in_data->params_32f[5];
        break;
    case 7:
        *out_size = sizeof(in_data->params_32f[6]);
        *p = (void*)&in_data->params_32f[6];
        break;
    case 8:
        *out_size = sizeof(in_data->params_32f[7]);
        *p = (void*)&in_data->params_32f[7];
        break;
    case 9:
        *out_size = sizeof(in_data->params_32f[8]);
        *p = (void*)&in_data->params_32f[8];
        break;
    case 10:
        *out_size = sizeof(in_data->params_32f[9]);
        *p = (void*)&in_data->params_32f[9];
        break;
    case 11:
        *out_size = sizeof(in_data->params_8i[0]);
        *p = (void*)&in_data->params_8i[0];
        break;
    case 12:
        *out_size = sizeof(in_data->params_8i[1]);
        *p = (void*)&in_data->params_8i[1];
        break;
    case 13:
        *out_size = sizeof(in_data->params_8i[2]);
        *p = (void*)&in_data->params_8i[2];
        break;
    case 14:
        *out_size = sizeof(in_data->params_8i[3]);
        *p = (void*)&in_data->params_8i[3];
        break;
    case 15:
        *out_size = sizeof(in_data->params_8i[4]);
        *p = (void*)&in_data->params_8i[4];
        break;
    case 16:
        *out_size = sizeof(in_data->params_8i[5]);
        *p = (void*)&in_data->params_8i[5];
        break;
    case 17:
        *out_size = sizeof(in_data->params_8i[6]);
        *p = (void*)&in_data->params_8i[6];
        break;
    case 18:
        *out_size = sizeof(in_data->params_8i[7]);
        *p = (void*)&in_data->params_8i[7];
        break;
    case 19:
        *out_size = sizeof(in_data->params_8i[8]);
        *p = (void*)&in_data->params_8i[8];
        break;
    case 20:
        *out_size = sizeof(in_data->params_8i[9]);
        *p = (void*)&in_data->params_8i[9];
        break;
    case 21:
        *out_size = sizeof(in_data->params_16i[0]);
        *p = (void*)&in_data->params_16i[0];
        break;
    case 22:
        *out_size = sizeof(in_data->params_16i[1]);
        *p = (void*)&in_data->params_16i[1];
        break;
    case 23:
        *out_size = sizeof(in_data->params_16i[2]);
        *p = (void*)&in_data->params_16i[2];
        break;
    case 24:
        *out_size = sizeof(in_data->params_16i[3]);
        *p = (void*)&in_data->params_16i[3];
        break;
    case 25:
        *out_size = sizeof(in_data->params_16i[4]);
        *p = (void*)&in_data->params_16i[4];
        break;
    case 26:
        *out_size = sizeof(in_data->params_16i[5]);
        *p = (void*)&in_data->params_16i[5];
        break;
    case 27:
        *out_size = sizeof(in_data->params_16i[6]);
        *p = (void*)&in_data->params_16i[6];
        break;
    case 28:
        *out_size = sizeof(in_data->params_16i[7]);
        *p = (void*)&in_data->params_16i[7];
        break;
    case 29:
        *out_size = sizeof(in_data->params_16i[8]);
        *p = (void*)&in_data->params_16i[8];
        break;
    case 30:
        *out_size = sizeof(in_data->params_16i[9]);
        *p = (void*)&in_data->params_16i[9];
        break;
    case 31:
        *out_size = sizeof(in_data->params_32i[0]);
        *p = (void*)&in_data->params_32i[0];
        break;
    case 32:
        *out_size = sizeof(in_data->params_32i[1]);
        *p = (void*)&in_data->params_32i[1];
        break;
    case 33:
        *out_size = sizeof(in_data->params_32i[2]);
        *p = (void*)&in_data->params_32i[2];
        break;
    case 34:
        *out_size = sizeof(in_data->params_32i[3]);
        *p = (void*)&in_data->params_32i[3];
        break;
    case 35:
        *out_size = sizeof(in_data->params_32i[4]);
        *p = (void*)&in_data->params_32i[4];
        break;
    case 36:
        *out_size = sizeof(in_data->params_32i[5]);
        *p = (void*)&in_data->params_32i[5];
        break;
    case 37:
        *out_size = sizeof(in_data->params_32i[6]);
        *p = (void*)&in_data->params_32i[6];
        break;
    case 38:
        *out_size = sizeof(in_data->params_32i[7]);
        *p = (void*)&in_data->params_32i[7];
        break;
    case 39:
        *out_size = sizeof(in_data->params_32i[8]);
        *p = (void*)&in_data->params_32i[8];
        break;
    case 40:
        *out_size = sizeof(in_data->params_32i[9]);
        *p = (void*)&in_data->params_32i[9];
        break;

    }
}

void marshalPartSSMCUData(uint8_t mode, SSMCUData *in_data, uint8_t *ids, uint8_t ids_cnt, uint8_t *out_data, uint8_t *offset)
{
    int i=0;  void *p; uint8_t size;

    for (i=0;i<ids_cnt;i++)
    {
        getVarInfoSSMCUData(in_data, ids[i], &p, &size);
        if (mode==STRUCT_MARSHAL)
            memcpy((void*)&out_data[*offset],p,size);
        else if (mode==STRUCT_UNMARSHAL)
            memcpy(p,(void*)&out_data[*offset],size);

        (*offset)+=size;
    }
}

void marshalPartSCMCUData(uint8_t mode, SCMCUData *in_data, uint8_t *ids, uint8_t ids_cnt, uint8_t *out_data, uint8_t *offset)
{
    int i=0;  void *p; uint8_t size;

    for (i=0;i<ids_cnt;i++)
    {
        getVarInfoSCMCUData(in_data, ids[i], &p, &size);
        if (mode==STRUCT_MARSHAL)
            memcpy((void*)&out_data[*offset],p,size);
        else if (mode==STRUCT_UNMARSHAL)
            memcpy(p,(void*)&out_data[*offset],size);

        (*offset)+=size;
    }
}

void marshalPartSUserParams(uint8_t mode, SUserParams *in_data, uint8_t *ids, uint8_t ids_cnt, uint8_t *out_data, uint8_t *offset)
{
    int i=0;  void *p; uint8_t size;

    for (i=0;i<ids_cnt;i++)
    {
        getVarInfoSUserParams(in_data, ids[i], &p, &size);
        if (mode==STRUCT_MARSHAL)
            memcpy((void*)&out_data[*offset],p,size);
        else if (mode==STRUCT_UNMARSHAL)
            memcpy(p,(void*)&out_data[*offset],size);

        (*offset)+=size;
    }
}

void marshalSSMCUData(SSMCUData *in_data, uint8_t *out_data, uint8_t *offset)
{
    uint16_t size = 0, total_size = *offset;

    size = sizeof(in_data->iReftime);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iReftime, size);
    total_size += size;

    size = sizeof(in_data->gyro_data);
    memcpy((void*) &out_data[total_size], (void*) &in_data->gyro_data, size);
    total_size += size;

    size = sizeof(in_data->gyro_raw);
    memcpy((void*) &out_data[total_size], (void*) &in_data->gyro_raw, size);
    total_size += size;

    size = sizeof(in_data->gyro_temp_raw);
    memcpy((void*) &out_data[total_size], (void*) &in_data->gyro_temp_raw, size);
    total_size += size;

    size = sizeof(in_data->accel_raw);
    memcpy((void*) &out_data[total_size], (void*) &in_data->accel_raw, size);
    total_size += size;

    size = sizeof(in_data->micromag);
    memcpy((void*) &out_data[total_size], (void*) &in_data->micromag, size);
    total_size += size;

    size = sizeof(in_data->ahrs);
    memcpy((void*) &out_data[total_size], (void*) &in_data->ahrs, size);
    total_size += size;

    size = sizeof(in_data->compass_heading);
    memcpy((void*) &out_data[total_size], (void*) &in_data->compass_heading, size);
    total_size += size;

    size = sizeof(in_data->rc_sticks);
    memcpy((void*) &out_data[total_size], (void*) &in_data->rc_sticks, size);
    total_size += size;

    size = sizeof(in_data->battery_voltage);
    memcpy((void*) &out_data[total_size], (void*) &in_data->battery_voltage, size);
    total_size += size;

    size = sizeof(in_data->cGPSValid);
    memcpy((void*) &out_data[total_size], (void*) &in_data->cGPSValid, size);
    total_size += size;

    size = sizeof(in_data->iLatDeg);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iLatDeg, size);
    total_size += size;

    size = sizeof(in_data->iLonDeg);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iLonDeg, size);
    total_size += size;

    size = sizeof(in_data->fAlt);
    memcpy((void*) &out_data[total_size], (void*) &in_data->fAlt, size);
    total_size += size;

    size = sizeof(in_data->iGpsLatDeg);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsLatDeg, size);
    total_size += size;

    size = sizeof(in_data->iGpsLonDeg);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsLonDeg, size);
    total_size += size;

    size = sizeof(in_data->iGpsAlt);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsAlt, size);
    total_size += size;

    size = sizeof(in_data->siGpsYawDeg);
    memcpy((void*) &out_data[total_size], (void*) &in_data->siGpsYawDeg, size);
    total_size += size;

    size = sizeof(in_data->ucGpsStatus);
    memcpy((void*) &out_data[total_size], (void*) &in_data->ucGpsStatus, size);
    total_size += size;

    size = sizeof(in_data->ucGpsStatusFlags);
    memcpy((void*) &out_data[total_size], (void*) &in_data->ucGpsStatusFlags, size);
    total_size += size;

    size = sizeof(in_data->gDop);
    memcpy((void*) &out_data[total_size], (void*) &in_data->gDop, size);
    total_size += size;

    size = sizeof(in_data->pDop);
    memcpy((void*) &out_data[total_size], (void*) &in_data->pDop, size);
    total_size += size;

    size = sizeof(in_data->hDop);
    memcpy((void*) &out_data[total_size], (void*) &in_data->hDop, size);
    total_size += size;

    size = sizeof(in_data->vDop);
    memcpy((void*) &out_data[total_size], (void*) &in_data->vDop, size);
    total_size += size;

    size = sizeof(in_data->num_SV);
    memcpy((void*) &out_data[total_size], (void*) &in_data->num_SV, size);
    total_size += size;

    size = sizeof(in_data->iGpsVelN);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsVelN, size);
    total_size += size;

    size = sizeof(in_data->iGpsVelE);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsVelE, size);
    total_size += size;

    size = sizeof(in_data->iGpsVelD);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsVelD, size);
    total_size += size;

    size = sizeof(in_data->iGpsSAcc);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsSAcc, size);
    total_size += size;

    size = sizeof(in_data->iGpsPAcc);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsPAcc, size);
    total_size += size;

    size = sizeof(in_data->iGpsHAcc);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsHAcc, size);
    total_size += size;

    size = sizeof(in_data->iGpsVAcc);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsVAcc, size);
    total_size += size;

    size = sizeof(in_data->system_status);
    memcpy((void*) &out_data[total_size], (void*) &in_data->system_status, size);
    total_size += size;

    size = sizeof(in_data->usiError);
    memcpy((void*) &out_data[total_size], (void*) &in_data->usiError, size);
    total_size += size;

    size = sizeof(in_data->remote_controller_status);
    memcpy((void*) &out_data[total_size], (void*) &in_data->remote_controller_status, size);
    total_size += size;

    size = sizeof(in_data->adc);
    memcpy((void*) &out_data[total_size], (void*) &in_data->adc, size);
    total_size += size;

    size = sizeof(in_data->ext_uart_crc_error_cnt);
    memcpy((void*) &out_data[total_size], (void*) &in_data->ext_uart_crc_error_cnt, size);
    total_size += size;

    size = sizeof(in_data->ext_uart_tx_overflow_cnt);
    memcpy((void*) &out_data[total_size], (void*) &in_data->ext_uart_tx_overflow_cnt, size);
    total_size += size;

    size = sizeof(in_data->fAltCF);
    memcpy((void*) &out_data[total_size], (void*) &in_data->fAltCF, size);
    total_size += size;

    size = sizeof(in_data->fVelU);
    memcpy((void*) &out_data[total_size], (void*) &in_data->fVelU, size);
    total_size += size;

    size = sizeof(in_data->gps_longitude_origo);
    memcpy((void*) &out_data[total_size], (void*) &in_data->gps_longitude_origo, size);
    total_size += size;

    size = sizeof(in_data->gps_latitude_origo);
    memcpy((void*) &out_data[total_size], (void*) &in_data->gps_latitude_origo, size);
    total_size += size;

    size = sizeof(in_data->gps_altitude_origo);
    memcpy((void*) &out_data[total_size], (void*) &in_data->gps_altitude_origo, size);
    total_size += size;

    size = sizeof(in_data->gps_origo_taken);
    memcpy((void*) &out_data[total_size], (void*) &in_data->gps_origo_taken, size);
    total_size += size;

    size = sizeof(in_data->gps_longitude_m);
    memcpy((void*) &out_data[total_size], (void*) &in_data->gps_longitude_m, size);
    total_size += size;

    size = sizeof(in_data->gps_latitude_m);
    memcpy((void*) &out_data[total_size], (void*) &in_data->gps_latitude_m, size);
    total_size += size;

    size = sizeof(in_data->fHeadCF);
    memcpy((void*) &out_data[total_size], (void*) &in_data->fHeadCF, size);
    total_size += size;


    *offset = total_size;
}

void marshalSSMCUDataRequestPart(SSMCUDataRequestPart *in_data, uint8_t *out_data, uint8_t *offset)
{
    uint16_t size = 0, total_size = *offset;

    size = sizeof(in_data->everyNth);
    memcpy((void*) &out_data[total_size], (void*) &in_data->everyNth, size);
    total_size += size;

    size = sizeof(in_data->ids);
    memcpy((void*) &out_data[total_size], (void*) &in_data->ids, size);
    total_size += size;

    size = sizeof(in_data->ids_cnt);
    memcpy((void*) &out_data[total_size], (void*) &in_data->ids_cnt, size);
    total_size += size;


    *offset = total_size;
}

void marshalSCMCUData(SCMCUData *in_data, uint8_t *out_data, uint8_t *offset)
{
    uint16_t size = 0, total_size = *offset;

    size = sizeof(in_data->id);
    memcpy((void*) &out_data[total_size], (void*) &in_data->id, size);
    total_size += size;

    size = sizeof(in_data->time);
    memcpy((void*) &out_data[total_size], (void*) &in_data->time, size);
    total_size += size;

    size = sizeof(in_data->logtime);
    memcpy((void*) &out_data[total_size], (void*) &in_data->logtime, size);
    total_size += size;

    size = sizeof(in_data->phi);
    memcpy((void*) &out_data[total_size], (void*) &in_data->phi, size);
    total_size += size;

    size = sizeof(in_data->theta);
    memcpy((void*) &out_data[total_size], (void*) &in_data->theta, size);
    total_size += size;

    size = sizeof(in_data->psi);
    memcpy((void*) &out_data[total_size], (void*) &in_data->psi, size);
    total_size += size;

    size = sizeof(in_data->compass_heading);
    memcpy((void*) &out_data[total_size], (void*) &in_data->compass_heading, size);
    total_size += size;

    size = sizeof(in_data->phi_dot_raw);
    memcpy((void*) &out_data[total_size], (void*) &in_data->phi_dot_raw, size);
    total_size += size;

    size = sizeof(in_data->theta_dot_raw);
    memcpy((void*) &out_data[total_size], (void*) &in_data->theta_dot_raw, size);
    total_size += size;

    size = sizeof(in_data->psi_dot_raw);
    memcpy((void*) &out_data[total_size], (void*) &in_data->psi_dot_raw, size);
    total_size += size;

    size = sizeof(in_data->phi_dot);
    memcpy((void*) &out_data[total_size], (void*) &in_data->phi_dot, size);
    total_size += size;

    size = sizeof(in_data->theta_dot);
    memcpy((void*) &out_data[total_size], (void*) &in_data->theta_dot, size);
    total_size += size;

    size = sizeof(in_data->psi_dot);
    memcpy((void*) &out_data[total_size], (void*) &in_data->psi_dot, size);
    total_size += size;

    size = sizeof(in_data->siPhiTemp);
    memcpy((void*) &out_data[total_size], (void*) &in_data->siPhiTemp, size);
    total_size += size;

    size = sizeof(in_data->siThetaTemp);
    memcpy((void*) &out_data[total_size], (void*) &in_data->siThetaTemp, size);
    total_size += size;

    size = sizeof(in_data->siPsiTemp);
    memcpy((void*) &out_data[total_size], (void*) &in_data->siPsiTemp, size);
    total_size += size;

    size = sizeof(in_data->accel_x_raw);
    memcpy((void*) &out_data[total_size], (void*) &in_data->accel_x_raw, size);
    total_size += size;

    size = sizeof(in_data->accel_y_raw);
    memcpy((void*) &out_data[total_size], (void*) &in_data->accel_y_raw, size);
    total_size += size;

    size = sizeof(in_data->accel_z_raw);
    memcpy((void*) &out_data[total_size], (void*) &in_data->accel_z_raw, size);
    total_size += size;

    size = sizeof(in_data->micromag_x);
    memcpy((void*) &out_data[total_size], (void*) &in_data->micromag_x, size);
    total_size += size;

    size = sizeof(in_data->micromag_y);
    memcpy((void*) &out_data[total_size], (void*) &in_data->micromag_y, size);
    total_size += size;

    size = sizeof(in_data->micromag_z);
    memcpy((void*) &out_data[total_size], (void*) &in_data->micromag_z, size);
    total_size += size;

    size = sizeof(in_data->hp);
    memcpy((void*) &out_data[total_size], (void*) &in_data->hp, size);
    total_size += size;

    size = sizeof(in_data->fPressureAltitude);
    memcpy((void*) &out_data[total_size], (void*) &in_data->fPressureAltitude, size);
    total_size += size;

    size = sizeof(in_data->cGPSValid);
    memcpy((void*) &out_data[total_size], (void*) &in_data->cGPSValid, size);
    total_size += size;

    size = sizeof(in_data->cGPSStatus);
    memcpy((void*) &out_data[total_size], (void*) &in_data->cGPSStatus, size);
    total_size += size;

    size = sizeof(in_data->iGpsLatDeg);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsLatDeg, size);
    total_size += size;

    size = sizeof(in_data->iGpsLonDeg);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsLonDeg, size);
    total_size += size;

    size = sizeof(in_data->iGpsAlt);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsAlt, size);
    total_size += size;

    size = sizeof(in_data->ucGpsStatusFlags);
    memcpy((void*) &out_data[total_size], (void*) &in_data->ucGpsStatusFlags, size);
    total_size += size;

    size = sizeof(in_data->gDop);
    memcpy((void*) &out_data[total_size], (void*) &in_data->gDop, size);
    total_size += size;

    size = sizeof(in_data->pDop);
    memcpy((void*) &out_data[total_size], (void*) &in_data->pDop, size);
    total_size += size;

    size = sizeof(in_data->hDop);
    memcpy((void*) &out_data[total_size], (void*) &in_data->hDop, size);
    total_size += size;

    size = sizeof(in_data->vDop);
    memcpy((void*) &out_data[total_size], (void*) &in_data->vDop, size);
    total_size += size;

    size = sizeof(in_data->num_SV);
    memcpy((void*) &out_data[total_size], (void*) &in_data->num_SV, size);
    total_size += size;

    size = sizeof(in_data->iGpsVelN);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsVelN, size);
    total_size += size;

    size = sizeof(in_data->iGpsVelE);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsVelE, size);
    total_size += size;

    size = sizeof(in_data->iGpsVelD);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsVelD, size);
    total_size += size;

    size = sizeof(in_data->iGpsSAcc);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsSAcc, size);
    total_size += size;

    size = sizeof(in_data->iGpsPAcc);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsPAcc, size);
    total_size += size;

    size = sizeof(in_data->iGpsHAcc);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsHAcc, size);
    total_size += size;

    size = sizeof(in_data->iGpsVAcc);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iGpsVAcc, size);
    total_size += size;

    size = sizeof(in_data->GpsHeadingDeg);
    memcpy((void*) &out_data[total_size], (void*) &in_data->GpsHeadingDeg, size);
    total_size += size;

    size = sizeof(in_data->rc_sticks_0);
    memcpy((void*) &out_data[total_size], (void*) &in_data->rc_sticks_0, size);
    total_size += size;

    size = sizeof(in_data->rc_sticks_1);
    memcpy((void*) &out_data[total_size], (void*) &in_data->rc_sticks_1, size);
    total_size += size;

    size = sizeof(in_data->rc_sticks_2);
    memcpy((void*) &out_data[total_size], (void*) &in_data->rc_sticks_2, size);
    total_size += size;

    size = sizeof(in_data->rc_sticks_3);
    memcpy((void*) &out_data[total_size], (void*) &in_data->rc_sticks_3, size);
    total_size += size;

    size = sizeof(in_data->rc_sticks_4);
    memcpy((void*) &out_data[total_size], (void*) &in_data->rc_sticks_4, size);
    total_size += size;

    size = sizeof(in_data->rc_sticks_5);
    memcpy((void*) &out_data[total_size], (void*) &in_data->rc_sticks_5, size);
    total_size += size;

    size = sizeof(in_data->rc_sticks_6);
    memcpy((void*) &out_data[total_size], (void*) &in_data->rc_sticks_6, size);
    total_size += size;

    size = sizeof(in_data->servo_output_0);
    memcpy((void*) &out_data[total_size], (void*) &in_data->servo_output_0, size);
    total_size += size;

    size = sizeof(in_data->servo_output_1);
    memcpy((void*) &out_data[total_size], (void*) &in_data->servo_output_1, size);
    total_size += size;

    size = sizeof(in_data->servo_output_2);
    memcpy((void*) &out_data[total_size], (void*) &in_data->servo_output_2, size);
    total_size += size;

    size = sizeof(in_data->servo_output_3);
    memcpy((void*) &out_data[total_size], (void*) &in_data->servo_output_3, size);
    total_size += size;

    size = sizeof(in_data->servo_output_4);
    memcpy((void*) &out_data[total_size], (void*) &in_data->servo_output_4, size);
    total_size += size;

    size = sizeof(in_data->servo_output_5);
    memcpy((void*) &out_data[total_size], (void*) &in_data->servo_output_5, size);
    total_size += size;

    size = sizeof(in_data->servo_output_6);
    memcpy((void*) &out_data[total_size], (void*) &in_data->servo_output_6, size);
    total_size += size;

    size = sizeof(in_data->servo_mixer_input_0);
    memcpy((void*) &out_data[total_size], (void*) &in_data->servo_mixer_input_0, size);
    total_size += size;

    size = sizeof(in_data->servo_mixer_input_1);
    memcpy((void*) &out_data[total_size], (void*) &in_data->servo_mixer_input_1, size);
    total_size += size;

    size = sizeof(in_data->servo_mixer_input_2);
    memcpy((void*) &out_data[total_size], (void*) &in_data->servo_mixer_input_2, size);
    total_size += size;

    size = sizeof(in_data->servo_mixer_input_3);
    memcpy((void*) &out_data[total_size], (void*) &in_data->servo_mixer_input_3, size);
    total_size += size;

    size = sizeof(in_data->servo_mixer_input_4);
    memcpy((void*) &out_data[total_size], (void*) &in_data->servo_mixer_input_4, size);
    total_size += size;

    size = sizeof(in_data->servo_mixer_input_5);
    memcpy((void*) &out_data[total_size], (void*) &in_data->servo_mixer_input_5, size);
    total_size += size;

    size = sizeof(in_data->servo_mixer_input_6);
    memcpy((void*) &out_data[total_size], (void*) &in_data->servo_mixer_input_6, size);
    total_size += size;

    size = sizeof(in_data->rc_aileron_at_ms);
    memcpy((void*) &out_data[total_size], (void*) &in_data->rc_aileron_at_ms, size);
    total_size += size;

    size = sizeof(in_data->rc_elevator_at_ms);
    memcpy((void*) &out_data[total_size], (void*) &in_data->rc_elevator_at_ms, size);
    total_size += size;

    size = sizeof(in_data->rc_rudder_at_ms);
    memcpy((void*) &out_data[total_size], (void*) &in_data->rc_rudder_at_ms, size);
    total_size += size;

    size = sizeof(in_data->rc_throttle_at_ms);
    memcpy((void*) &out_data[total_size], (void*) &in_data->rc_throttle_at_ms, size);
    total_size += size;

    size = sizeof(in_data->rc_camera_pan_at_ms);
    memcpy((void*) &out_data[total_size], (void*) &in_data->rc_camera_pan_at_ms, size);
    total_size += size;

    size = sizeof(in_data->rc_camera_tilt_at_ms);
    memcpy((void*) &out_data[total_size], (void*) &in_data->rc_camera_tilt_at_ms, size);
    total_size += size;

    size = sizeof(in_data->targetLonDeg);
    memcpy((void*) &out_data[total_size], (void*) &in_data->targetLonDeg, size);
    total_size += size;

    size = sizeof(in_data->targetLatDeg);
    memcpy((void*) &out_data[total_size], (void*) &in_data->targetLatDeg, size);
    total_size += size;

    size = sizeof(in_data->targetAltM);
    memcpy((void*) &out_data[total_size], (void*) &in_data->targetAltM, size);
    total_size += size;

    size = sizeof(in_data->targetHeadingDeg);
    memcpy((void*) &out_data[total_size], (void*) &in_data->targetHeadingDeg, size);
    total_size += size;

    size = sizeof(in_data->battery_voltage);
    memcpy((void*) &out_data[total_size], (void*) &in_data->battery_voltage, size);
    total_size += size;

    size = sizeof(in_data->spi_status);
    memcpy((void*) &out_data[total_size], (void*) &in_data->spi_status, size);
    total_size += size;

    size = sizeof(in_data->spi_crc_error_cnt);
    memcpy((void*) &out_data[total_size], (void*) &in_data->spi_crc_error_cnt, size);
    total_size += size;

    size = sizeof(in_data->spi_tx_overflow_cnt);
    memcpy((void*) &out_data[total_size], (void*) &in_data->spi_tx_overflow_cnt, size);
    total_size += size;

    size = sizeof(in_data->serial_1_crc_error_cnt);
    memcpy((void*) &out_data[total_size], (void*) &in_data->serial_1_crc_error_cnt, size);
    total_size += size;

    size = sizeof(in_data->serial_1_tx_overflow_cnt);
    memcpy((void*) &out_data[total_size], (void*) &in_data->serial_1_tx_overflow_cnt, size);
    total_size += size;

    size = sizeof(in_data->serial_2_crc_error_cnt);
    memcpy((void*) &out_data[total_size], (void*) &in_data->serial_2_crc_error_cnt, size);
    total_size += size;

    size = sizeof(in_data->serial_2_tx_overflow_cnt);
    memcpy((void*) &out_data[total_size], (void*) &in_data->serial_2_tx_overflow_cnt, size);
    total_size += size;

    size = sizeof(in_data->accel_status);
    memcpy((void*) &out_data[total_size], (void*) &in_data->accel_status, size);
    total_size += size;

    size = sizeof(in_data->remote_controller_status);
    memcpy((void*) &out_data[total_size], (void*) &in_data->remote_controller_status, size);
    total_size += size;

    size = sizeof(in_data->flight_mode);
    memcpy((void*) &out_data[total_size], (void*) &in_data->flight_mode, size);
    total_size += size;

    size = sizeof(in_data->camera_mode);
    memcpy((void*) &out_data[total_size], (void*) &in_data->camera_mode, size);
    total_size += size;

    size = sizeof(in_data->logging_mode_status);
    memcpy((void*) &out_data[total_size], (void*) &in_data->logging_mode_status, size);
    total_size += size;

    size = sizeof(in_data->logging_progress);
    memcpy((void*) &out_data[total_size], (void*) &in_data->logging_progress, size);
    total_size += size;

    size = sizeof(in_data->sdcard_status);
    memcpy((void*) &out_data[total_size], (void*) &in_data->sdcard_status, size);
    total_size += size;

    size = sizeof(in_data->system_status);
    memcpy((void*) &out_data[total_size], (void*) &in_data->system_status, size);
    total_size += size;

    size = sizeof(in_data->motor_status);
    memcpy((void*) &out_data[total_size], (void*) &in_data->motor_status, size);
    total_size += size;

    size = sizeof(in_data->adc_0);
    memcpy((void*) &out_data[total_size], (void*) &in_data->adc_0, size);
    total_size += size;

    size = sizeof(in_data->adc_1);
    memcpy((void*) &out_data[total_size], (void*) &in_data->adc_1, size);
    total_size += size;

    size = sizeof(in_data->adc_2);
    memcpy((void*) &out_data[total_size], (void*) &in_data->adc_2, size);
    total_size += size;

    size = sizeof(in_data->UserParamFloat_0);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamFloat_0, size);
    total_size += size;

    size = sizeof(in_data->UserParamFloat_1);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamFloat_1, size);
    total_size += size;

    size = sizeof(in_data->UserParamFloat_2);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamFloat_2, size);
    total_size += size;

    size = sizeof(in_data->UserParamFloat_3);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamFloat_3, size);
    total_size += size;

    size = sizeof(in_data->UserParamFloat_4);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamFloat_4, size);
    total_size += size;

    size = sizeof(in_data->UserParamFloat_5);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamFloat_5, size);
    total_size += size;

    size = sizeof(in_data->UserParamFloat_6);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamFloat_6, size);
    total_size += size;

    size = sizeof(in_data->UserParamFloat_7);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamFloat_7, size);
    total_size += size;

    size = sizeof(in_data->UserParamFloat_8);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamFloat_8, size);
    total_size += size;

    size = sizeof(in_data->UserParamFloat_9);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamFloat_9, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt8_0);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt8_0, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt8_1);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt8_1, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt8_2);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt8_2, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt8_3);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt8_3, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt8_4);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt8_4, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt8_5);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt8_5, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt8_6);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt8_6, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt8_7);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt8_7, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt8_8);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt8_8, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt8_9);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt8_9, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt16_0);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt16_0, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt16_1);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt16_1, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt16_2);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt16_2, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt16_3);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt16_3, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt16_4);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt16_4, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt16_5);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt16_5, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt16_6);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt16_6, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt16_7);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt16_7, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt16_8);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt16_8, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt16_9);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt16_9, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt32_0);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt32_0, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt32_1);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt32_1, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt32_2);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt32_2, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt32_3);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt32_3, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt32_4);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt32_4, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt32_5);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt32_5, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt32_6);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt32_6, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt32_7);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt32_7, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt32_8);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt32_8, size);
    total_size += size;

    size = sizeof(in_data->UserParamInt32_9);
    memcpy((void*) &out_data[total_size], (void*) &in_data->UserParamInt32_9, size);
    total_size += size;

    size = sizeof(in_data->fAltCF);
    memcpy((void*) &out_data[total_size], (void*) &in_data->fAltCF, size);
    total_size += size;

    size = sizeof(in_data->fVelU);
    memcpy((void*) &out_data[total_size], (void*) &in_data->fVelU, size);
    total_size += size;

    size = sizeof(in_data->gps_longitude_origo);
    memcpy((void*) &out_data[total_size], (void*) &in_data->gps_longitude_origo, size);
    total_size += size;

    size = sizeof(in_data->gps_latitude_origo);
    memcpy((void*) &out_data[total_size], (void*) &in_data->gps_latitude_origo, size);
    total_size += size;

    size = sizeof(in_data->gps_altitude_origo);
    memcpy((void*) &out_data[total_size], (void*) &in_data->gps_altitude_origo, size);
    total_size += size;

    size = sizeof(in_data->gps_longitude_m);
    memcpy((void*) &out_data[total_size], (void*) &in_data->gps_longitude_m, size);
    total_size += size;

    size = sizeof(in_data->gps_latitude_m);
    memcpy((void*) &out_data[total_size], (void*) &in_data->gps_latitude_m, size);
    total_size += size;

    size = sizeof(in_data->gps_origo_taken);
    memcpy((void*) &out_data[total_size], (void*) &in_data->gps_origo_taken, size);
    total_size += size;

    size = sizeof(in_data->fHeadCF);
    memcpy((void*) &out_data[total_size], (void*) &in_data->fHeadCF, size);
    total_size += size;

    size = sizeof(in_data->mixer_input_0_at_fl_ms);
    memcpy((void*) &out_data[total_size], (void*) &in_data->mixer_input_0_at_fl_ms, size);
    total_size += size;

    size = sizeof(in_data->mixer_input_1_at_fl_ms);
    memcpy((void*) &out_data[total_size], (void*) &in_data->mixer_input_1_at_fl_ms, size);
    total_size += size;

    size = sizeof(in_data->mixer_input_2_at_fl_ms);
    memcpy((void*) &out_data[total_size], (void*) &in_data->mixer_input_2_at_fl_ms, size);
    total_size += size;

    size = sizeof(in_data->mixer_input_3_at_fl_ms);
    memcpy((void*) &out_data[total_size], (void*) &in_data->mixer_input_3_at_fl_ms, size);
    total_size += size;

    size = sizeof(in_data->mixer_input_4_at_fl_ms);
    memcpy((void*) &out_data[total_size], (void*) &in_data->mixer_input_4_at_fl_ms, size);
    total_size += size;

    size = sizeof(in_data->mixer_input_5_at_fl_ms);
    memcpy((void*) &out_data[total_size], (void*) &in_data->mixer_input_5_at_fl_ms, size);
    total_size += size;

    size = sizeof(in_data->mixer_input_6_at_fl_ms);
    memcpy((void*) &out_data[total_size], (void*) &in_data->mixer_input_6_at_fl_ms, size);
    total_size += size;

    size = sizeof(in_data->outer_loop_output_0);
    memcpy((void*) &out_data[total_size], (void*) &in_data->outer_loop_output_0, size);
    total_size += size;

    size = sizeof(in_data->outer_loop_output_1);
    memcpy((void*) &out_data[total_size], (void*) &in_data->outer_loop_output_1, size);
    total_size += size;

    size = sizeof(in_data->outer_loop_output_2);
    memcpy((void*) &out_data[total_size], (void*) &in_data->outer_loop_output_2, size);
    total_size += size;

    size = sizeof(in_data->outer_loop_output_3);
    memcpy((void*) &out_data[total_size], (void*) &in_data->outer_loop_output_3, size);
    total_size += size;

    size = sizeof(in_data->outer_loop_output_4);
    memcpy((void*) &out_data[total_size], (void*) &in_data->outer_loop_output_4, size);
    total_size += size;

    size = sizeof(in_data->outer_loop_output_5);
    memcpy((void*) &out_data[total_size], (void*) &in_data->outer_loop_output_5, size);
    total_size += size;

    size = sizeof(in_data->outer_loop_output_6);
    memcpy((void*) &out_data[total_size], (void*) &in_data->outer_loop_output_6, size);
    total_size += size;

    size = sizeof(in_data->outer_loop_output_7);
    memcpy((void*) &out_data[total_size], (void*) &in_data->outer_loop_output_7, size);
    total_size += size;


    *offset = total_size;
}

void marshalSCMCUDataRequestPart(SCMCUDataRequestPart *in_data, uint8_t *out_data, uint8_t *offset)
{
    uint16_t size = 0, total_size = *offset;

    size = sizeof(in_data->everyNth);
    memcpy((void*) &out_data[total_size], (void*) &in_data->everyNth, size);
    total_size += size;

    size = sizeof(in_data->ids);
    memcpy((void*) &out_data[total_size], (void*) &in_data->ids, size);
    total_size += size;

    size = sizeof(in_data->ids_cnt);
    memcpy((void*) &out_data[total_size], (void*) &in_data->ids_cnt, size);
    total_size += size;


    *offset = total_size;
}

void marshalSUserParams(SUserParams *in_data, uint8_t *out_data, uint8_t *offset)
{
    uint16_t size = 0, total_size = *offset;

    size = sizeof(in_data->flags);
    memcpy((void*) &out_data[total_size], (void*) &in_data->flags, size);
    total_size += size;

    size = sizeof(in_data->params_32f);
    memcpy((void*) &out_data[total_size], (void*) &in_data->params_32f, size);
    total_size += size;

    size = sizeof(in_data->params_8i);
    memcpy((void*) &out_data[total_size], (void*) &in_data->params_8i, size);
    total_size += size;

    size = sizeof(in_data->params_16i);
    memcpy((void*) &out_data[total_size], (void*) &in_data->params_16i, size);
    total_size += size;

    size = sizeof(in_data->params_32i);
    memcpy((void*) &out_data[total_size], (void*) &in_data->params_32i, size);
    total_size += size;


    *offset = total_size;
}

void marshalSFlightTargetParams(SFlightTargetParams *in_data, uint8_t *out_data, uint8_t *offset)
{
    uint16_t size = 0, total_size = *offset;

    size = sizeof(in_data->flags);
    memcpy((void*) &out_data[total_size], (void*) &in_data->flags, size);
    total_size += size;

    size = sizeof(in_data->longitude);
    memcpy((void*) &out_data[total_size], (void*) &in_data->longitude, size);
    total_size += size;

    size = sizeof(in_data->latitude);
    memcpy((void*) &out_data[total_size], (void*) &in_data->latitude, size);
    total_size += size;

    size = sizeof(in_data->altitude);
    memcpy((void*) &out_data[total_size], (void*) &in_data->altitude, size);
    total_size += size;

    size = sizeof(in_data->heading);
    memcpy((void*) &out_data[total_size], (void*) &in_data->heading, size);
    total_size += size;


    *offset = total_size;
}

void marshalSUserParamsRequestPart(SUserParamsRequestPart *in_data, uint8_t *out_data, uint8_t *offset)
{
    uint16_t size = 0, total_size = *offset;

    size = sizeof(in_data->everyNth);
    memcpy((void*) &out_data[total_size], (void*) &in_data->everyNth, size);
    total_size += size;

    size = sizeof(in_data->ids);
    memcpy((void*) &out_data[total_size], (void*) &in_data->ids, size);
    total_size += size;

    size = sizeof(in_data->ids_cnt);
    memcpy((void*) &out_data[total_size], (void*) &in_data->ids_cnt, size);
    total_size += size;


    *offset = total_size;
}

void marshalSCmd(SCmd *in_data, uint8_t *out_data, uint8_t *offset)
{
    uint16_t size = 0, total_size = *offset;

    size = sizeof(in_data->cId);
    memcpy((void*) &out_data[total_size], (void*) &in_data->cId, size);
    total_size += size;


    *offset = total_size;
}

void marshalSCmdParam1Int(SCmdParam1Int *in_data, uint8_t *out_data, uint8_t *offset)
{
    uint16_t size = 0, total_size = *offset;

    size = sizeof(in_data->cId);
    memcpy((void*) &out_data[total_size], (void*) &in_data->cId, size);
    total_size += size;

    size = sizeof(in_data->iParam1);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iParam1, size);
    total_size += size;


    *offset = total_size;
}

void marshalSCmdParamText(SCmdParamText *in_data, uint8_t *out_data, uint8_t *offset)
{
    uint16_t size = 0, total_size = *offset;

    size = sizeof(in_data->cId);
    memcpy((void*) &out_data[total_size], (void*) &in_data->cId, size);
    total_size += size;

    size = sizeof(in_data->rgcText);
    memcpy((void*) &out_data[total_size], (void*) &in_data->rgcText, size);
    total_size += size;


    *offset = total_size;
}

void marshalSDebugPrint(SDebugPrint *in_data, uint8_t *out_data, uint8_t *offset)
{
    uint16_t size = 0, total_size = *offset;

    size = sizeof(in_data->msg);
    memcpy((void*) &out_data[total_size], (void*) &in_data->msg, size);
    total_size += size;


    *offset = total_size;
}

void marshalSDebugPrintI(SDebugPrintI *in_data, uint8_t *out_data, uint8_t *offset)
{
    uint16_t size = 0, total_size = *offset;

    size = sizeof(in_data->msg);
    memcpy((void*) &out_data[total_size], (void*) &in_data->msg, size);
    total_size += size;

    size = sizeof(in_data->iValue);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iValue, size);
    total_size += size;


    *offset = total_size;
}

void marshalHlCommandRequest(HlCommandRequest *in_data, uint8_t *out_data, uint8_t *offset)
{
    uint16_t size = 0, total_size = *offset;

    size = sizeof(in_data->command);
    memcpy((void*) &out_data[total_size], (void*) &in_data->command, size);
    total_size += size;

    size = sizeof(in_data->fargs);
    memcpy((void*) &out_data[total_size], (void*) &in_data->fargs, size);
    total_size += size;

    size = sizeof(in_data->iargs);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iargs, size);
    total_size += size;

    size = sizeof(in_data->uuid);
    memcpy((void*) &out_data[total_size], (void*) &in_data->uuid, size);
    total_size += size;

    size = sizeof(in_data->token);
    memcpy((void*) &out_data[total_size], (void*) &in_data->token, size);
    total_size += size;


    *offset = total_size;
}

void marshalHlCommandResponse(HlCommandResponse *in_data, uint8_t *out_data, uint8_t *offset)
{
    uint16_t size = 0, total_size = *offset;

    size = sizeof(in_data->uuid);
    memcpy((void*) &out_data[total_size], (void*) &in_data->uuid, size);
    total_size += size;

    size = sizeof(in_data->status);
    memcpy((void*) &out_data[total_size], (void*) &in_data->status, size);
    total_size += size;

    size = sizeof(in_data->token);
    memcpy((void*) &out_data[total_size], (void*) &in_data->token, size);
    total_size += size;

    size = sizeof(in_data->fargs);
    memcpy((void*) &out_data[total_size], (void*) &in_data->fargs, size);
    total_size += size;

    size = sizeof(in_data->iargs);
    memcpy((void*) &out_data[total_size], (void*) &in_data->iargs, size);
    total_size += size;


    *offset = total_size;
}

void unmarshalSSMCUData(uint8_t *in_data, SSMCUData *out_data)
{
    uint16_t size = 0, total_size = 0;

    size = sizeof(out_data->iReftime);
    memcpy((void*) &out_data->iReftime, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->gyro_data);
    memcpy((void*) &out_data->gyro_data, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->gyro_raw);
    memcpy((void*) &out_data->gyro_raw, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->gyro_temp_raw);
    memcpy((void*) &out_data->gyro_temp_raw, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->accel_raw);
    memcpy((void*) &out_data->accel_raw, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->micromag);
    memcpy((void*) &out_data->micromag, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->ahrs);
    memcpy((void*) &out_data->ahrs, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->compass_heading);
    memcpy((void*) &out_data->compass_heading, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->rc_sticks);
    memcpy((void*) &out_data->rc_sticks, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->battery_voltage);
    memcpy((void*) &out_data->battery_voltage, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->cGPSValid);
    memcpy((void*) &out_data->cGPSValid, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iLatDeg);
    memcpy((void*) &out_data->iLatDeg, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iLonDeg);
    memcpy((void*) &out_data->iLonDeg, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->fAlt);
    memcpy((void*) &out_data->fAlt, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsLatDeg);
    memcpy((void*) &out_data->iGpsLatDeg, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsLonDeg);
    memcpy((void*) &out_data->iGpsLonDeg, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsAlt);
    memcpy((void*) &out_data->iGpsAlt, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->siGpsYawDeg);
    memcpy((void*) &out_data->siGpsYawDeg, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->ucGpsStatus);
    memcpy((void*) &out_data->ucGpsStatus, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->ucGpsStatusFlags);
    memcpy((void*) &out_data->ucGpsStatusFlags, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->gDop);
    memcpy((void*) &out_data->gDop, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->pDop);
    memcpy((void*) &out_data->pDop, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->hDop);
    memcpy((void*) &out_data->hDop, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->vDop);
    memcpy((void*) &out_data->vDop, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->num_SV);
    memcpy((void*) &out_data->num_SV, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsVelN);
    memcpy((void*) &out_data->iGpsVelN, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsVelE);
    memcpy((void*) &out_data->iGpsVelE, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsVelD);
    memcpy((void*) &out_data->iGpsVelD, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsSAcc);
    memcpy((void*) &out_data->iGpsSAcc, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsPAcc);
    memcpy((void*) &out_data->iGpsPAcc, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsHAcc);
    memcpy((void*) &out_data->iGpsHAcc, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsVAcc);
    memcpy((void*) &out_data->iGpsVAcc, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->system_status);
    memcpy((void*) &out_data->system_status, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->usiError);
    memcpy((void*) &out_data->usiError, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->remote_controller_status);
    memcpy((void*) &out_data->remote_controller_status, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->adc);
    memcpy((void*) &out_data->adc, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->ext_uart_crc_error_cnt);
    memcpy((void*) &out_data->ext_uart_crc_error_cnt, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->ext_uart_tx_overflow_cnt);
    memcpy((void*) &out_data->ext_uart_tx_overflow_cnt, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->fAltCF);
    memcpy((void*) &out_data->fAltCF, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->fVelU);
    memcpy((void*) &out_data->fVelU, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->gps_longitude_origo);
    memcpy((void*) &out_data->gps_longitude_origo, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->gps_latitude_origo);
    memcpy((void*) &out_data->gps_latitude_origo, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->gps_altitude_origo);
    memcpy((void*) &out_data->gps_altitude_origo, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->gps_origo_taken);
    memcpy((void*) &out_data->gps_origo_taken, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->gps_longitude_m);
    memcpy((void*) &out_data->gps_longitude_m, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->gps_latitude_m);
    memcpy((void*) &out_data->gps_latitude_m, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->fHeadCF);
    memcpy((void*) &out_data->fHeadCF, (void*) &in_data[total_size], size);
    total_size += size;

}

void unmarshalSSMCUDataRequestPart(uint8_t *in_data, SSMCUDataRequestPart *out_data)
{
    uint16_t size = 0, total_size = 0;

    size = sizeof(out_data->everyNth);
    memcpy((void*) &out_data->everyNth, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->ids);
    memcpy((void*) &out_data->ids, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->ids_cnt);
    memcpy((void*) &out_data->ids_cnt, (void*) &in_data[total_size], size);
    total_size += size;

}

void unmarshalSCMCUData(uint8_t *in_data, SCMCUData *out_data)
{
    uint16_t size = 0, total_size = 0;

    size = sizeof(out_data->id);
    memcpy((void*) &out_data->id, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->time);
    memcpy((void*) &out_data->time, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->logtime);
    memcpy((void*) &out_data->logtime, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->phi);
    memcpy((void*) &out_data->phi, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->theta);
    memcpy((void*) &out_data->theta, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->psi);
    memcpy((void*) &out_data->psi, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->compass_heading);
    memcpy((void*) &out_data->compass_heading, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->phi_dot_raw);
    memcpy((void*) &out_data->phi_dot_raw, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->theta_dot_raw);
    memcpy((void*) &out_data->theta_dot_raw, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->psi_dot_raw);
    memcpy((void*) &out_data->psi_dot_raw, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->phi_dot);
    memcpy((void*) &out_data->phi_dot, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->theta_dot);
    memcpy((void*) &out_data->theta_dot, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->psi_dot);
    memcpy((void*) &out_data->psi_dot, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->siPhiTemp);
    memcpy((void*) &out_data->siPhiTemp, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->siThetaTemp);
    memcpy((void*) &out_data->siThetaTemp, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->siPsiTemp);
    memcpy((void*) &out_data->siPsiTemp, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->accel_x_raw);
    memcpy((void*) &out_data->accel_x_raw, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->accel_y_raw);
    memcpy((void*) &out_data->accel_y_raw, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->accel_z_raw);
    memcpy((void*) &out_data->accel_z_raw, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->micromag_x);
    memcpy((void*) &out_data->micromag_x, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->micromag_y);
    memcpy((void*) &out_data->micromag_y, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->micromag_z);
    memcpy((void*) &out_data->micromag_z, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->hp);
    memcpy((void*) &out_data->hp, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->fPressureAltitude);
    memcpy((void*) &out_data->fPressureAltitude, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->cGPSValid);
    memcpy((void*) &out_data->cGPSValid, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->cGPSStatus);
    memcpy((void*) &out_data->cGPSStatus, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsLatDeg);
    memcpy((void*) &out_data->iGpsLatDeg, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsLonDeg);
    memcpy((void*) &out_data->iGpsLonDeg, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsAlt);
    memcpy((void*) &out_data->iGpsAlt, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->ucGpsStatusFlags);
    memcpy((void*) &out_data->ucGpsStatusFlags, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->gDop);
    memcpy((void*) &out_data->gDop, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->pDop);
    memcpy((void*) &out_data->pDop, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->hDop);
    memcpy((void*) &out_data->hDop, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->vDop);
    memcpy((void*) &out_data->vDop, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->num_SV);
    memcpy((void*) &out_data->num_SV, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsVelN);
    memcpy((void*) &out_data->iGpsVelN, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsVelE);
    memcpy((void*) &out_data->iGpsVelE, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsVelD);
    memcpy((void*) &out_data->iGpsVelD, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsSAcc);
    memcpy((void*) &out_data->iGpsSAcc, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsPAcc);
    memcpy((void*) &out_data->iGpsPAcc, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsHAcc);
    memcpy((void*) &out_data->iGpsHAcc, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iGpsVAcc);
    memcpy((void*) &out_data->iGpsVAcc, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->GpsHeadingDeg);
    memcpy((void*) &out_data->GpsHeadingDeg, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->rc_sticks_0);
    memcpy((void*) &out_data->rc_sticks_0, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->rc_sticks_1);
    memcpy((void*) &out_data->rc_sticks_1, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->rc_sticks_2);
    memcpy((void*) &out_data->rc_sticks_2, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->rc_sticks_3);
    memcpy((void*) &out_data->rc_sticks_3, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->rc_sticks_4);
    memcpy((void*) &out_data->rc_sticks_4, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->rc_sticks_5);
    memcpy((void*) &out_data->rc_sticks_5, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->rc_sticks_6);
    memcpy((void*) &out_data->rc_sticks_6, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->servo_output_0);
    memcpy((void*) &out_data->servo_output_0, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->servo_output_1);
    memcpy((void*) &out_data->servo_output_1, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->servo_output_2);
    memcpy((void*) &out_data->servo_output_2, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->servo_output_3);
    memcpy((void*) &out_data->servo_output_3, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->servo_output_4);
    memcpy((void*) &out_data->servo_output_4, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->servo_output_5);
    memcpy((void*) &out_data->servo_output_5, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->servo_output_6);
    memcpy((void*) &out_data->servo_output_6, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->servo_mixer_input_0);
    memcpy((void*) &out_data->servo_mixer_input_0, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->servo_mixer_input_1);
    memcpy((void*) &out_data->servo_mixer_input_1, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->servo_mixer_input_2);
    memcpy((void*) &out_data->servo_mixer_input_2, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->servo_mixer_input_3);
    memcpy((void*) &out_data->servo_mixer_input_3, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->servo_mixer_input_4);
    memcpy((void*) &out_data->servo_mixer_input_4, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->servo_mixer_input_5);
    memcpy((void*) &out_data->servo_mixer_input_5, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->servo_mixer_input_6);
    memcpy((void*) &out_data->servo_mixer_input_6, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->rc_aileron_at_ms);
    memcpy((void*) &out_data->rc_aileron_at_ms, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->rc_elevator_at_ms);
    memcpy((void*) &out_data->rc_elevator_at_ms, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->rc_rudder_at_ms);
    memcpy((void*) &out_data->rc_rudder_at_ms, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->rc_throttle_at_ms);
    memcpy((void*) &out_data->rc_throttle_at_ms, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->rc_camera_pan_at_ms);
    memcpy((void*) &out_data->rc_camera_pan_at_ms, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->rc_camera_tilt_at_ms);
    memcpy((void*) &out_data->rc_camera_tilt_at_ms, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->targetLonDeg);
    memcpy((void*) &out_data->targetLonDeg, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->targetLatDeg);
    memcpy((void*) &out_data->targetLatDeg, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->targetAltM);
    memcpy((void*) &out_data->targetAltM, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->targetHeadingDeg);
    memcpy((void*) &out_data->targetHeadingDeg, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->battery_voltage);
    memcpy((void*) &out_data->battery_voltage, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->spi_status);
    memcpy((void*) &out_data->spi_status, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->spi_crc_error_cnt);
    memcpy((void*) &out_data->spi_crc_error_cnt, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->spi_tx_overflow_cnt);
    memcpy((void*) &out_data->spi_tx_overflow_cnt, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->serial_1_crc_error_cnt);
    memcpy((void*) &out_data->serial_1_crc_error_cnt, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->serial_1_tx_overflow_cnt);
    memcpy((void*) &out_data->serial_1_tx_overflow_cnt, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->serial_2_crc_error_cnt);
    memcpy((void*) &out_data->serial_2_crc_error_cnt, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->serial_2_tx_overflow_cnt);
    memcpy((void*) &out_data->serial_2_tx_overflow_cnt, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->accel_status);
    memcpy((void*) &out_data->accel_status, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->remote_controller_status);
    memcpy((void*) &out_data->remote_controller_status, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->flight_mode);
    memcpy((void*) &out_data->flight_mode, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->camera_mode);
    memcpy((void*) &out_data->camera_mode, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->logging_mode_status);
    memcpy((void*) &out_data->logging_mode_status, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->logging_progress);
    memcpy((void*) &out_data->logging_progress, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->sdcard_status);
    memcpy((void*) &out_data->sdcard_status, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->system_status);
    memcpy((void*) &out_data->system_status, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->motor_status);
    memcpy((void*) &out_data->motor_status, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->adc_0);
    memcpy((void*) &out_data->adc_0, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->adc_1);
    memcpy((void*) &out_data->adc_1, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->adc_2);
    memcpy((void*) &out_data->adc_2, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamFloat_0);
    memcpy((void*) &out_data->UserParamFloat_0, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamFloat_1);
    memcpy((void*) &out_data->UserParamFloat_1, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamFloat_2);
    memcpy((void*) &out_data->UserParamFloat_2, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamFloat_3);
    memcpy((void*) &out_data->UserParamFloat_3, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamFloat_4);
    memcpy((void*) &out_data->UserParamFloat_4, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamFloat_5);
    memcpy((void*) &out_data->UserParamFloat_5, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamFloat_6);
    memcpy((void*) &out_data->UserParamFloat_6, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamFloat_7);
    memcpy((void*) &out_data->UserParamFloat_7, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamFloat_8);
    memcpy((void*) &out_data->UserParamFloat_8, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamFloat_9);
    memcpy((void*) &out_data->UserParamFloat_9, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt8_0);
    memcpy((void*) &out_data->UserParamInt8_0, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt8_1);
    memcpy((void*) &out_data->UserParamInt8_1, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt8_2);
    memcpy((void*) &out_data->UserParamInt8_2, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt8_3);
    memcpy((void*) &out_data->UserParamInt8_3, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt8_4);
    memcpy((void*) &out_data->UserParamInt8_4, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt8_5);
    memcpy((void*) &out_data->UserParamInt8_5, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt8_6);
    memcpy((void*) &out_data->UserParamInt8_6, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt8_7);
    memcpy((void*) &out_data->UserParamInt8_7, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt8_8);
    memcpy((void*) &out_data->UserParamInt8_8, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt8_9);
    memcpy((void*) &out_data->UserParamInt8_9, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt16_0);
    memcpy((void*) &out_data->UserParamInt16_0, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt16_1);
    memcpy((void*) &out_data->UserParamInt16_1, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt16_2);
    memcpy((void*) &out_data->UserParamInt16_2, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt16_3);
    memcpy((void*) &out_data->UserParamInt16_3, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt16_4);
    memcpy((void*) &out_data->UserParamInt16_4, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt16_5);
    memcpy((void*) &out_data->UserParamInt16_5, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt16_6);
    memcpy((void*) &out_data->UserParamInt16_6, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt16_7);
    memcpy((void*) &out_data->UserParamInt16_7, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt16_8);
    memcpy((void*) &out_data->UserParamInt16_8, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt16_9);
    memcpy((void*) &out_data->UserParamInt16_9, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt32_0);
    memcpy((void*) &out_data->UserParamInt32_0, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt32_1);
    memcpy((void*) &out_data->UserParamInt32_1, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt32_2);
    memcpy((void*) &out_data->UserParamInt32_2, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt32_3);
    memcpy((void*) &out_data->UserParamInt32_3, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt32_4);
    memcpy((void*) &out_data->UserParamInt32_4, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt32_5);
    memcpy((void*) &out_data->UserParamInt32_5, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt32_6);
    memcpy((void*) &out_data->UserParamInt32_6, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt32_7);
    memcpy((void*) &out_data->UserParamInt32_7, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt32_8);
    memcpy((void*) &out_data->UserParamInt32_8, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->UserParamInt32_9);
    memcpy((void*) &out_data->UserParamInt32_9, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->fAltCF);
    memcpy((void*) &out_data->fAltCF, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->fVelU);
    memcpy((void*) &out_data->fVelU, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->gps_longitude_origo);
    memcpy((void*) &out_data->gps_longitude_origo, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->gps_latitude_origo);
    memcpy((void*) &out_data->gps_latitude_origo, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->gps_altitude_origo);
    memcpy((void*) &out_data->gps_altitude_origo, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->gps_longitude_m);
    memcpy((void*) &out_data->gps_longitude_m, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->gps_latitude_m);
    memcpy((void*) &out_data->gps_latitude_m, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->gps_origo_taken);
    memcpy((void*) &out_data->gps_origo_taken, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->fHeadCF);
    memcpy((void*) &out_data->fHeadCF, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->mixer_input_0_at_fl_ms);
    memcpy((void*) &out_data->mixer_input_0_at_fl_ms, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->mixer_input_1_at_fl_ms);
    memcpy((void*) &out_data->mixer_input_1_at_fl_ms, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->mixer_input_2_at_fl_ms);
    memcpy((void*) &out_data->mixer_input_2_at_fl_ms, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->mixer_input_3_at_fl_ms);
    memcpy((void*) &out_data->mixer_input_3_at_fl_ms, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->mixer_input_4_at_fl_ms);
    memcpy((void*) &out_data->mixer_input_4_at_fl_ms, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->mixer_input_5_at_fl_ms);
    memcpy((void*) &out_data->mixer_input_5_at_fl_ms, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->mixer_input_6_at_fl_ms);
    memcpy((void*) &out_data->mixer_input_6_at_fl_ms, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->outer_loop_output_0);
    memcpy((void*) &out_data->outer_loop_output_0, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->outer_loop_output_1);
    memcpy((void*) &out_data->outer_loop_output_1, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->outer_loop_output_2);
    memcpy((void*) &out_data->outer_loop_output_2, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->outer_loop_output_3);
    memcpy((void*) &out_data->outer_loop_output_3, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->outer_loop_output_4);
    memcpy((void*) &out_data->outer_loop_output_4, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->outer_loop_output_5);
    memcpy((void*) &out_data->outer_loop_output_5, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->outer_loop_output_6);
    memcpy((void*) &out_data->outer_loop_output_6, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->outer_loop_output_7);
    memcpy((void*) &out_data->outer_loop_output_7, (void*) &in_data[total_size], size);
    total_size += size;

}

void unmarshalSCMCUDataRequestPart(uint8_t *in_data, SCMCUDataRequestPart *out_data)
{
    uint16_t size = 0, total_size = 0;

    size = sizeof(out_data->everyNth);
    memcpy((void*) &out_data->everyNth, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->ids);
    memcpy((void*) &out_data->ids, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->ids_cnt);
    memcpy((void*) &out_data->ids_cnt, (void*) &in_data[total_size], size);
    total_size += size;

}

void unmarshalSUserParams(uint8_t *in_data, SUserParams *out_data)
{
    uint16_t size = 0, total_size = 0;

    size = sizeof(out_data->flags);
    memcpy((void*) &out_data->flags, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->params_32f);
    memcpy((void*) &out_data->params_32f, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->params_8i);
    memcpy((void*) &out_data->params_8i, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->params_16i);
    memcpy((void*) &out_data->params_16i, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->params_32i);
    memcpy((void*) &out_data->params_32i, (void*) &in_data[total_size], size);
    total_size += size;

}

void unmarshalSFlightTargetParams(uint8_t *in_data, SFlightTargetParams *out_data)
{
    uint16_t size = 0, total_size = 0;

    size = sizeof(out_data->flags);
    memcpy((void*) &out_data->flags, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->longitude);
    memcpy((void*) &out_data->longitude, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->latitude);
    memcpy((void*) &out_data->latitude, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->altitude);
    memcpy((void*) &out_data->altitude, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->heading);
    memcpy((void*) &out_data->heading, (void*) &in_data[total_size], size);
    total_size += size;

}

void unmarshalSUserParamsRequestPart(uint8_t *in_data, SUserParamsRequestPart *out_data)
{
    uint16_t size = 0, total_size = 0;

    size = sizeof(out_data->everyNth);
    memcpy((void*) &out_data->everyNth, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->ids);
    memcpy((void*) &out_data->ids, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->ids_cnt);
    memcpy((void*) &out_data->ids_cnt, (void*) &in_data[total_size], size);
    total_size += size;

}

void unmarshalSCmd(uint8_t *in_data, SCmd *out_data)
{
    uint16_t size = 0, total_size = 0;

    size = sizeof(out_data->cId);
    memcpy((void*) &out_data->cId, (void*) &in_data[total_size], size);
    total_size += size;

}

void unmarshalSCmdParam1Int(uint8_t *in_data, SCmdParam1Int *out_data)
{
    uint16_t size = 0, total_size = 0;

    size = sizeof(out_data->cId);
    memcpy((void*) &out_data->cId, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iParam1);
    memcpy((void*) &out_data->iParam1, (void*) &in_data[total_size], size);
    total_size += size;

}

void unmarshalSCmdParamText(uint8_t *in_data, SCmdParamText *out_data)
{
    uint16_t size = 0, total_size = 0;

    size = sizeof(out_data->cId);
    memcpy((void*) &out_data->cId, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->rgcText);
    memcpy((void*) &out_data->rgcText, (void*) &in_data[total_size], size);
    total_size += size;

}

void unmarshalSDebugPrint(uint8_t *in_data, SDebugPrint *out_data)
{
    uint16_t size = 0, total_size = 0;

    size = sizeof(out_data->msg);
    memcpy((void*) &out_data->msg, (void*) &in_data[total_size], size);
    total_size += size;

}

void unmarshalSDebugPrintI(uint8_t *in_data, SDebugPrintI *out_data)
{
    uint16_t size = 0, total_size = 0;

    size = sizeof(out_data->msg);
    memcpy((void*) &out_data->msg, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iValue);
    memcpy((void*) &out_data->iValue, (void*) &in_data[total_size], size);
    total_size += size;

}

void unmarshalHlCommandRequest(uint8_t *in_data, HlCommandRequest *out_data)
{
    uint16_t size = 0, total_size = 0;

    size = sizeof(out_data->command);
    memcpy((void*) &out_data->command, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->fargs);
    memcpy((void*) &out_data->fargs, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iargs);
    memcpy((void*) &out_data->iargs, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->uuid);
    memcpy((void*) &out_data->uuid, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->token);
    memcpy((void*) &out_data->token, (void*) &in_data[total_size], size);
    total_size += size;

}

void unmarshalHlCommandResponse(uint8_t *in_data, HlCommandResponse *out_data)
{
    uint16_t size = 0, total_size = 0;

    size = sizeof(out_data->uuid);
    memcpy((void*) &out_data->uuid, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->status);
    memcpy((void*) &out_data->status, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->token);
    memcpy((void*) &out_data->token, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->fargs);
    memcpy((void*) &out_data->fargs, (void*) &in_data[total_size], size);
    total_size += size;

    size = sizeof(out_data->iargs);
    memcpy((void*) &out_data->iargs, (void*) &in_data[total_size], size);
    total_size += size;

}


uint16_t getSizeSSMCUData(void)
{
    SSMCUData *tmp = 0;

    return sizeof(tmp->iReftime)
            + sizeof(tmp->gyro_data)
            + sizeof(tmp->gyro_raw)
            + sizeof(tmp->gyro_temp_raw)
            + sizeof(tmp->accel_raw)
            + sizeof(tmp->micromag)
            + sizeof(tmp->ahrs)
            + sizeof(tmp->compass_heading)
            + sizeof(tmp->rc_sticks)
            + sizeof(tmp->battery_voltage)
            + sizeof(tmp->cGPSValid)
            + sizeof(tmp->iLatDeg)
            + sizeof(tmp->iLonDeg)
            + sizeof(tmp->fAlt)
            + sizeof(tmp->iGpsLatDeg)
            + sizeof(tmp->iGpsLonDeg)
            + sizeof(tmp->iGpsAlt)
            + sizeof(tmp->siGpsYawDeg)
            + sizeof(tmp->ucGpsStatus)
            + sizeof(tmp->ucGpsStatusFlags)
            + sizeof(tmp->gDop)
            + sizeof(tmp->pDop)
            + sizeof(tmp->hDop)
            + sizeof(tmp->vDop)
            + sizeof(tmp->num_SV)
            + sizeof(tmp->iGpsVelN)
            + sizeof(tmp->iGpsVelE)
            + sizeof(tmp->iGpsVelD)
            + sizeof(tmp->iGpsSAcc)
            + sizeof(tmp->iGpsPAcc)
            + sizeof(tmp->iGpsHAcc)
            + sizeof(tmp->iGpsVAcc)
            + sizeof(tmp->system_status)
            + sizeof(tmp->usiError)
            + sizeof(tmp->remote_controller_status)
            + sizeof(tmp->adc)
            + sizeof(tmp->ext_uart_crc_error_cnt)
            + sizeof(tmp->ext_uart_tx_overflow_cnt)
            + sizeof(tmp->fAltCF)
            + sizeof(tmp->fVelU)
            + sizeof(tmp->gps_longitude_origo)
            + sizeof(tmp->gps_latitude_origo)
            + sizeof(tmp->gps_altitude_origo)
            + sizeof(tmp->gps_origo_taken)
            + sizeof(tmp->gps_longitude_m)
            + sizeof(tmp->gps_latitude_m)
            + sizeof(tmp->fHeadCF);
}

uint16_t getSizeSSMCUDataRequestPart(void)
{
    SSMCUDataRequestPart *tmp = 0;

    return sizeof(tmp->everyNth)
            + sizeof(tmp->ids)
            + sizeof(tmp->ids_cnt);
}

uint16_t getSizeSCMCUData(void)
{
    SCMCUData *tmp = 0;

    return sizeof(tmp->id)
            + sizeof(tmp->time)
            + sizeof(tmp->logtime)
            + sizeof(tmp->phi)
            + sizeof(tmp->theta)
            + sizeof(tmp->psi)
            + sizeof(tmp->compass_heading)
            + sizeof(tmp->phi_dot_raw)
            + sizeof(tmp->theta_dot_raw)
            + sizeof(tmp->psi_dot_raw)
            + sizeof(tmp->phi_dot)
            + sizeof(tmp->theta_dot)
            + sizeof(tmp->psi_dot)
            + sizeof(tmp->siPhiTemp)
            + sizeof(tmp->siThetaTemp)
            + sizeof(tmp->siPsiTemp)
            + sizeof(tmp->accel_x_raw)
            + sizeof(tmp->accel_y_raw)
            + sizeof(tmp->accel_z_raw)
            + sizeof(tmp->micromag_x)
            + sizeof(tmp->micromag_y)
            + sizeof(tmp->micromag_z)
            + sizeof(tmp->hp)
            + sizeof(tmp->fPressureAltitude)
            + sizeof(tmp->cGPSValid)
            + sizeof(tmp->cGPSStatus)
            + sizeof(tmp->iGpsLatDeg)
            + sizeof(tmp->iGpsLonDeg)
            + sizeof(tmp->iGpsAlt)
            + sizeof(tmp->ucGpsStatusFlags)
            + sizeof(tmp->gDop)
            + sizeof(tmp->pDop)
            + sizeof(tmp->hDop)
            + sizeof(tmp->vDop)
            + sizeof(tmp->num_SV)
            + sizeof(tmp->iGpsVelN)
            + sizeof(tmp->iGpsVelE)
            + sizeof(tmp->iGpsVelD)
            + sizeof(tmp->iGpsSAcc)
            + sizeof(tmp->iGpsPAcc)
            + sizeof(tmp->iGpsHAcc)
            + sizeof(tmp->iGpsVAcc)
            + sizeof(tmp->GpsHeadingDeg)
            + sizeof(tmp->rc_sticks_0)
            + sizeof(tmp->rc_sticks_1)
            + sizeof(tmp->rc_sticks_2)
            + sizeof(tmp->rc_sticks_3)
            + sizeof(tmp->rc_sticks_4)
            + sizeof(tmp->rc_sticks_5)
            + sizeof(tmp->rc_sticks_6)
            + sizeof(tmp->servo_output_0)
            + sizeof(tmp->servo_output_1)
            + sizeof(tmp->servo_output_2)
            + sizeof(tmp->servo_output_3)
            + sizeof(tmp->servo_output_4)
            + sizeof(tmp->servo_output_5)
            + sizeof(tmp->servo_output_6)
            + sizeof(tmp->servo_mixer_input_0)
            + sizeof(tmp->servo_mixer_input_1)
            + sizeof(tmp->servo_mixer_input_2)
            + sizeof(tmp->servo_mixer_input_3)
            + sizeof(tmp->servo_mixer_input_4)
            + sizeof(tmp->servo_mixer_input_5)
            + sizeof(tmp->servo_mixer_input_6)
            + sizeof(tmp->rc_aileron_at_ms)
            + sizeof(tmp->rc_elevator_at_ms)
            + sizeof(tmp->rc_rudder_at_ms)
            + sizeof(tmp->rc_throttle_at_ms)
            + sizeof(tmp->rc_camera_pan_at_ms)
            + sizeof(tmp->rc_camera_tilt_at_ms)
            + sizeof(tmp->targetLonDeg)
            + sizeof(tmp->targetLatDeg)
            + sizeof(tmp->targetAltM)
            + sizeof(tmp->targetHeadingDeg)
            + sizeof(tmp->battery_voltage)
            + sizeof(tmp->spi_status)
            + sizeof(tmp->spi_crc_error_cnt)
            + sizeof(tmp->spi_tx_overflow_cnt)
            + sizeof(tmp->serial_1_crc_error_cnt)
            + sizeof(tmp->serial_1_tx_overflow_cnt)
            + sizeof(tmp->serial_2_crc_error_cnt)
            + sizeof(tmp->serial_2_tx_overflow_cnt)
            + sizeof(tmp->accel_status)
            + sizeof(tmp->remote_controller_status)
            + sizeof(tmp->flight_mode)
            + sizeof(tmp->camera_mode)
            + sizeof(tmp->logging_mode_status)
            + sizeof(tmp->logging_progress)
            + sizeof(tmp->sdcard_status)
            + sizeof(tmp->system_status)
            + sizeof(tmp->motor_status)
            + sizeof(tmp->adc_0)
            + sizeof(tmp->adc_1)
            + sizeof(tmp->adc_2)
            + sizeof(tmp->UserParamFloat_0)
            + sizeof(tmp->UserParamFloat_1)
            + sizeof(tmp->UserParamFloat_2)
            + sizeof(tmp->UserParamFloat_3)
            + sizeof(tmp->UserParamFloat_4)
            + sizeof(tmp->UserParamFloat_5)
            + sizeof(tmp->UserParamFloat_6)
            + sizeof(tmp->UserParamFloat_7)
            + sizeof(tmp->UserParamFloat_8)
            + sizeof(tmp->UserParamFloat_9)
            + sizeof(tmp->UserParamInt8_0)
            + sizeof(tmp->UserParamInt8_1)
            + sizeof(tmp->UserParamInt8_2)
            + sizeof(tmp->UserParamInt8_3)
            + sizeof(tmp->UserParamInt8_4)
            + sizeof(tmp->UserParamInt8_5)
            + sizeof(tmp->UserParamInt8_6)
            + sizeof(tmp->UserParamInt8_7)
            + sizeof(tmp->UserParamInt8_8)
            + sizeof(tmp->UserParamInt8_9)
            + sizeof(tmp->UserParamInt16_0)
            + sizeof(tmp->UserParamInt16_1)
            + sizeof(tmp->UserParamInt16_2)
            + sizeof(tmp->UserParamInt16_3)
            + sizeof(tmp->UserParamInt16_4)
            + sizeof(tmp->UserParamInt16_5)
            + sizeof(tmp->UserParamInt16_6)
            + sizeof(tmp->UserParamInt16_7)
            + sizeof(tmp->UserParamInt16_8)
            + sizeof(tmp->UserParamInt16_9)
            + sizeof(tmp->UserParamInt32_0)
            + sizeof(tmp->UserParamInt32_1)
            + sizeof(tmp->UserParamInt32_2)
            + sizeof(tmp->UserParamInt32_3)
            + sizeof(tmp->UserParamInt32_4)
            + sizeof(tmp->UserParamInt32_5)
            + sizeof(tmp->UserParamInt32_6)
            + sizeof(tmp->UserParamInt32_7)
            + sizeof(tmp->UserParamInt32_8)
            + sizeof(tmp->UserParamInt32_9)
            + sizeof(tmp->fAltCF)
            + sizeof(tmp->fVelU)
            + sizeof(tmp->gps_longitude_origo)
            + sizeof(tmp->gps_latitude_origo)
            + sizeof(tmp->gps_altitude_origo)
            + sizeof(tmp->gps_longitude_m)
            + sizeof(tmp->gps_latitude_m)
            + sizeof(tmp->gps_origo_taken)
            + sizeof(tmp->fHeadCF)
            + sizeof(tmp->mixer_input_0_at_fl_ms)
            + sizeof(tmp->mixer_input_1_at_fl_ms)
            + sizeof(tmp->mixer_input_2_at_fl_ms)
            + sizeof(tmp->mixer_input_3_at_fl_ms)
            + sizeof(tmp->mixer_input_4_at_fl_ms)
            + sizeof(tmp->mixer_input_5_at_fl_ms)
            + sizeof(tmp->mixer_input_6_at_fl_ms)
            + sizeof(tmp->outer_loop_output_0)
            + sizeof(tmp->outer_loop_output_1)
            + sizeof(tmp->outer_loop_output_2)
            + sizeof(tmp->outer_loop_output_3)
            + sizeof(tmp->outer_loop_output_4)
            + sizeof(tmp->outer_loop_output_5)
            + sizeof(tmp->outer_loop_output_6)
            + sizeof(tmp->outer_loop_output_7);
}

uint16_t getSizeSCMCUDataRequestPart(void)
{
    SCMCUDataRequestPart *tmp = 0;

    return sizeof(tmp->everyNth)
            + sizeof(tmp->ids)
            + sizeof(tmp->ids_cnt);
}

uint16_t getSizeSUserParams(void)
{
    SUserParams *tmp = 0;

    return sizeof(tmp->flags)
            + sizeof(tmp->params_32f)
            + sizeof(tmp->params_8i)
            + sizeof(tmp->params_16i)
            + sizeof(tmp->params_32i);
}

uint16_t getSizeSFlightTargetParams(void)
{
    SFlightTargetParams *tmp = 0;

    return sizeof(tmp->flags)
            + sizeof(tmp->longitude)
            + sizeof(tmp->latitude)
            + sizeof(tmp->altitude)
            + sizeof(tmp->heading);
}

uint16_t getSizeSUserParamsRequestPart(void)
{
    SUserParamsRequestPart *tmp = 0;

    return sizeof(tmp->everyNth)
            + sizeof(tmp->ids)
            + sizeof(tmp->ids_cnt);
}

uint16_t getSizeSCmd(void)
{
    SCmd *tmp = 0;

    return sizeof(tmp->cId);
}

uint16_t getSizeSCmdParam1Int(void)
{
    SCmdParam1Int *tmp = 0;

    return sizeof(tmp->cId)
            + sizeof(tmp->iParam1);
}

uint16_t getSizeSCmdParamText(void)
{
    SCmdParamText *tmp = 0;

    return sizeof(tmp->cId)
            + sizeof(tmp->rgcText);
}

uint16_t getSizeSDebugPrint(void)
{
    SDebugPrint *tmp = 0;

    return sizeof(tmp->msg);
}

uint16_t getSizeSDebugPrintI(void)
{
    SDebugPrintI *tmp = 0;

    return sizeof(tmp->msg)
            + sizeof(tmp->iValue);
}

uint16_t getSizeHlCommandRequest(void)
{
    HlCommandRequest *tmp = 0;

    return sizeof(tmp->command)
            + sizeof(tmp->fargs)
            + sizeof(tmp->iargs)
            + sizeof(tmp->uuid)
            + sizeof(tmp->token);
}

uint16_t getSizeHlCommandResponse(void)
{
    HlCommandResponse *tmp = 0;

    return sizeof(tmp->uuid)
            + sizeof(tmp->status)
            + sizeof(tmp->token)
            + sizeof(tmp->fargs)
            + sizeof(tmp->iargs);
}
