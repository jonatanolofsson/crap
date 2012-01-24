// ^([ ]+)([^ ]+) ([^\[;]+)(\[[0-9N]+\])?;([0-9]+)$
// \1struct \3_t { typedef \2 type\4; type \3; static const size_t size = sizeof(type); static const uint8_t c = 1; static const uint8_t n = \5; void set(const uint8_t*& v){memcpy(&\3, v, sizeof(type)); v += sizeof(type); } void get(char*& v){memcpy(v, &\3, sizeof(type)); v += sizeof(type); } }; typedef \3_t \3;

                namespace request_part {
                    uint16_t everyNth;0
                    template<int N>
                    uint8_t ids[N];1
                    uint8_t ids_cnt;2

                    template<int TYPE, int N>
                    struct DataRequestPart : public serial_data<request_part::everyNth_t, request_part::ids_t<N>, request_part::ids_cnt_t>
                    {
                        DataRequestPart(const uint8_t everyNth_, uint8_t* ids_, uint8_t ids_cnt_) {
                            assert(ids_cnt_ <= N);
                            everyNth = everyNth_;
                            memcpy(ids_t::ids, ids_, ids_cnt_);
                            ids_cnt = ids_cnt;
                            metadata.packet_type = TYPE;
                        }
                    };
                }

                namespace SSMCU {
                    typedef request_part::DataRequestPart<MSG_SMCU_DATA_PART_REQUEST, 66> Part;

                    uint32_t iReftime;0
                    int16_t gyro_data[3];1
                    int16_t gyro_raw[3];2
                    int16_t gyro_temp_raw[3];3
                    int16_t accel_raw[3];4
                    int16_t micromag[3];5
                    int16_t ahrs[3];6
                    int16_t compass_heading;7
                    int16_t rc_sticks[7];8
                    int16_t battery_voltage;9
                    int8_t cGPSValid;10
                    int32_t iLatDeg;11
                    int32_t iLonDeg;12
                    float fAlt;13
                    int32_t iGpsLatDeg;14
                    int32_t iGpsLonDeg;15
                    int32_t iGpsAlt;16
                    int16_t siGpsYawDeg;17
                    uint8_t ucGpsStatus;18
                    uint8_t ucGpsStatusFlags;19
                    uint16_t gDop;20
                    uint16_t pDop;21
                    uint16_t hDop;22
                    uint16_t vDop;23
                    uint8_t num_SV;24
                    int16_t iGpsVelN;25
                    int16_t iGpsVelE;26
                    int16_t iGpsVelD;27
                    uint16_t iGpsSAcc;28
                    uint16_t iGpsPAcc;29
                    uint16_t iGpsHAcc;30
                    uint16_t iGpsVAcc;31
                    uint8_t system_status;32
                    uint16_t usiError;33
                    uint8_t remote_controller_status;34
                    uint16_t adc[3];35
                    uint8_t ext_uart_crc_error_cnt;36
                    uint8_t ext_uart_tx_overflow_cnt;37
                    float fAltCF;38
                    float fVelU;39
                    int gps_longitude_origo;40
                    int gps_latitude_origo;41
                    int gps_altitude_origo;42
                    char gps_origo_taken;43
                    float gps_longitude_m;44
                    float gps_latitude_m;45
                    float fHeadCF;46
                }

                namespace SCMCU {
                    typedef request_part::DataRequestPart<MSG_CMCU_DATA_PART_REQUEST, 157> Part;

                    int8_t id;0
                    uint32_t time;1
                    uint32_t logtime;2
                    float phi;3
                    float theta;4
                    float psi;5
                    float compass_heading;6
                    float phi_dot_raw;7
                    float theta_dot_raw;8
                    float psi_dot_raw;9
                    float phi_dot;10
                    float theta_dot;11
                    float psi_dot;12
                    int16_t siPhiTemp;13
                    int16_t siThetaTemp;14
                    int16_t siPsiTemp;15
                    float accel_x_raw;16
                    float accel_y_raw;17
                    float accel_z_raw;18
                    float micromag_x;19
                    float micromag_y;20
                    float micromag_z;21
                    float hp;22
                    float fPressureAltitude;23
                    int8_t cGPSValid;24
                    uint8_t cGPSStatus;25
                    int32_t iGpsLatDeg;26
                    int32_t iGpsLonDeg;27
                    uint32_t iGpsAlt;28
                    uint8_t ucGpsStatusFlags;29
                    uint16_t gDop;30
                    uint16_t pDop;31
                    uint16_t hDop;32
                    uint16_t vDop;33
                    uint8_t num_SV;34
                    int16_t iGpsVelN;35
                    int16_t iGpsVelE;36
                    int16_t iGpsVelD;37
                    uint16_t iGpsSAcc;38
                    uint16_t iGpsPAcc;39
                    uint16_t iGpsHAcc;40
                    uint16_t iGpsVAcc;41
                    float GpsHeadingDeg;42
                    uint16_t rc_sticks_0;43
                    uint16_t rc_sticks_1;44
                    uint16_t rc_sticks_2;45
                    uint16_t rc_sticks_3;46
                    uint16_t rc_sticks_4;47
                    uint16_t rc_sticks_5;48
                    uint16_t rc_sticks_6;49
                    uint16_t servo_output_0;50
                    uint16_t servo_output_1;51
                    uint16_t servo_output_2;52
                    uint16_t servo_output_3;53
                    uint16_t servo_output_4;54
                    uint16_t servo_output_5;55
                    uint16_t servo_output_6;56
                    int16_t servo_mixer_input_0;57
                    int16_t servo_mixer_input_1;58
                    int16_t servo_mixer_input_2;59
                    int16_t servo_mixer_input_3;60
                    int16_t servo_mixer_input_4;61
                    int16_t servo_mixer_input_5;62
                    int16_t servo_mixer_input_6;63
                    uint8_t rc_aileron_at_ms;64
                    uint8_t rc_elevator_at_ms;65
                    uint8_t rc_rudder_at_ms;66
                    uint8_t rc_throttle_at_ms;67
                    uint8_t rc_camera_pan_at_ms;68
                    uint8_t rc_camera_tilt_at_ms;69
                    int32_t targetLonDeg;70
                    int32_t targetLatDeg;71
                    float targetAltM;72
                    float targetHeadingDeg;73
                    int16_t battery_voltage;74
                    int8_t spi_status;75
                    uint8_t spi_crc_error_cnt;76
                    uint8_t spi_tx_overflow_cnt;77
                    uint8_t serial_1_crc_error_cnt;78
                    uint8_t serial_1_tx_overflow_cnt;79
                    uint8_t serial_2_crc_error_cnt;80
                    uint8_t serial_2_tx_overflow_cnt;81
                    uint16_t accel_status;82
                    uint8_t remote_controller_status;83
                    uint8_t flight_mode;84
                    uint8_t camera_mode;85
                    uint8_t logging_mode_status;86
                    uint8_t logging_progress;87
                    uint8_t sdcard_status;88
                    uint8_t system_status;89
                    uint8_t motor_status;90
                    uint16_t adc_0;91
                    uint16_t adc_1;92
                    uint16_t adc_2;93
                    float UserParamFloat_0;94
                    float UserParamFloat_1;95
                    float UserParamFloat_2;96
                    float UserParamFloat_3;97
                    float UserParamFloat_4;98
                    float UserParamFloat_5;99
                    float UserParamFloat_6;100
                    float UserParamFloat_7;101
                    float UserParamFloat_8;102
                    float UserParamFloat_9;103
                    int8_t UserParamInt8_0;104
                    int8_t UserParamInt8_1;105
                    int8_t UserParamInt8_2;106
                    int8_t UserParamInt8_3;107
                    int8_t UserParamInt8_4;108
                    int8_t UserParamInt8_5;109
                    int8_t UserParamInt8_6;110
                    int8_t UserParamInt8_7;111
                    int8_t UserParamInt8_8;112
                    int8_t UserParamInt8_9;113
                    int16_t UserParamInt16_0;114
                    int16_t UserParamInt16_1;115
                    int16_t UserParamInt16_2;116
                    int16_t UserParamInt16_3;117
                    int16_t UserParamInt16_4;118
                    int16_t UserParamInt16_5;119
                    int16_t UserParamInt16_6;120
                    int16_t UserParamInt16_7;121
                    int16_t UserParamInt16_8;122
                    int16_t UserParamInt16_9;123
                    int32_t UserParamInt32_0;124
                    int32_t UserParamInt32_1;125
                    int32_t UserParamInt32_2;126
                    int32_t UserParamInt32_3;127
                    int32_t UserParamInt32_4;128
                    int32_t UserParamInt32_5;129
                    int32_t UserParamInt32_6;130
                    int32_t UserParamInt32_7;131
                    int32_t UserParamInt32_8;132
                    int32_t UserParamInt32_9;133
                    float fAltCF;134
                    float fVelU;135
                    int32_t gps_longitude_origo;136
                    int32_t gps_latitude_origo;137
                    int32_t gps_altitude_origo;138
                    float gps_longitude_m;139
                    float gps_latitude_m;140
                    char gps_origo_taken;141
                    float fHeadCF;142
                    int16_t mixer_input_0_at_fl_ms;143
                    int16_t mixer_input_1_at_fl_ms;144
                    int16_t mixer_input_2_at_fl_ms;145
                    int16_t mixer_input_3_at_fl_ms;146
                    int16_t mixer_input_4_at_fl_ms;147
                    int16_t mixer_input_5_at_fl_ms;148
                    int16_t mixer_input_6_at_fl_ms;149
                    float outer_loop_output_0;150
                    float outer_loop_output_1;151
                    float outer_loop_output_2;152
                    float outer_loop_output_3;153
                    float outer_loop_output_4;154
                    float outer_loop_output_5;155
                    float outer_loop_output_6;156
                    float outer_loop_output_7;157
                }

                namespace SUser {
                    uint64_t flags;0
                    float params_32f[10];1
                    int8_t params_8i[10];2
                    int16_t params_16i[10];3
                    int32_t params_32i[10];4
                }

                namespace SFlightTarget {
                    uint8_t flags;0
                    float longitude;1
                    float latitude;2
                    float altitude;3
                    float heading;4
                }

                namespace SCmd {
                    uint8_t cId;1
                }
