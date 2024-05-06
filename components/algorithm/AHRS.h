#ifndef AHRS_H
#define AHRS_H

#include "AHRS_MiddleWare.h"

/**
  * @brief          According to the data of the accelerometer and magnetometer, the quaternion is initialized
  * @param[in]      quaternion that required initialization
  * @param[in]      quaternion initialization, (x,y,z) is not empty unit m/s2
  * @param[in]      magnetometer initialization, (x,y,z) is not empty unit uT
  * @retval         NULL
  */
extern void AHRS_init(fp32 quat[4], const fp32 accel[3], const fp32 mag[3]);

/**
  * @brief          Update the quaternion based on gyroscope, accelerometer, and magnetometer data
  * @param[in]      quaternion array to be updated
  * @param[in]      update timing time, called at a fixed rate, for example, 1000Hz, pass in 0.001f
  * @param[in]      gyroscope data for update, array order (x, y, z) in radians
  * @param[in]      accelerometer data for initialization, array order (x, y, z) in m/s2
  * @param[in]      magnetometer data for initialization, array order (x, y, z) in uT
  * @retval         1: update successful, 0: update failed
  */
extern bool_t AHRS_update(fp32 quat[4], const fp32 timing_time, const fp32 gyro[3], const fp32 accel[3], const fp32 mag[3]);

/**
  * @brief          Calculate the corresponding Euler angle yaw based on the magnitude of the quaternion
  * @param[in]      quaternion array, not NULL
  * @retval         The calculated yaw angle, in radians
  */
extern fp32 get_yaw(const fp32 quat[4]);

/**
  * @brief          Calculate the corresponding Euler angle pitch based on the magnitude of the quaternion
  * @param[in]      quaternion array, not NULL
  * @retval         The calculated pitch angle, in radians
  */
extern fp32 get_pitch(const fp32 quat[4]);
/**
  * @brief          Calculate the corresponding Euler angle roll based on the magnitude of the quaternion
  * @param[in]      quaternion array, not NULL
  * @retval         The calculated roll angle, in radians
  */
/**
  * @brief          Calculate the corresponding Euler angles yaw, pitch, and roll based on the magnitude of the quaternion
  * @param[in]      quaternion array, not NULL
  * @param[out]     yaw angle in radians
  * @param[out]     pitch angle in radians
  * @param[out]     roll angle in radians
  */
extern void get_angle(const fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);
extern void get_angle(const fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);

extern fp32 get_carrier_gravity(void);

#endif
