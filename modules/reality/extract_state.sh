#!/bin/bash

# 170       c_target_vel_north
# 171       c_target_vel_east
# 172       c_target_vel_up
# 167       c_state_north_m
# 168       c_state_east_m
# 169       c_state_altitude
# 174       c_state_pitch
# 175       c_state_roll
# 173       c_state_heading

# 26380 offset works
awk -F "\"*,\"*" 'NR>1{print $170, $171, -$172, $167, $168, -$169, $174, $175, $173}' $1 | sed -n '13190~16p' > real_states

# 17        s_accel_x_raw
# 18        s_accel_y_raw
# 19        s_accel_z_raw
# 11         s_phi_dot
# 12         s_theta_dot
# 13        s_psi_dot
# 23        s_pressure_hp
# 20        s_mag_x
# 21        s_mag_y
# 22        s_mag_z
awk -F "\"*,\"*" 'NR>1{print $17, $18, $19, $11, $12, $13, $23, $20, $21, $22}' $1 | sed -n '13190~16p' > real_imu_data


