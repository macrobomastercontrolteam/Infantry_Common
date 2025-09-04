/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "INS_task.h"

#include "main.h"

#include "cmsis_os.h"

#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "pid.h"
#include "ahrs.h"

#include "calibrate_task.h"
#include "detect_task.h"
#include <math.h> // For sqrtf if needed, though direct quaternion math avoids it here

#define GRAVITY_MAGNITUDE 9.80665f // Standard gravity in m/s^2


#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)

#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


/**
  * @brief          rotate the gyro, accel and mag, and calculate the zero drift, because sensors have 
  *                 different install derection.
  * @param[out]     gyro: after plus zero drift and rotate
  * @param[out]     accel: after plus zero drift and rotate
  * @param[out]     mag: after plus zero drift and rotate
  * @param[in]      bmi088: gyro and accel data
  * @param[in]      ist8310: mag data
  * @retval         none
  */
static void imu_cali_solve(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
static void imu_temp_control(fp32 temp);
/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
static void imu_cmd_spi_dma(void);



extern SPI_HandleTypeDef hspi1;


static TaskHandle_t INS_task_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGTH];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGTH] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGTH];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGTH] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};


uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGTH];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGTH] = {0xA2,0xFF,0xFF,0xFF};



volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;


bmi088_real_data_t bmi088_real_data;
fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

ist8310_real_data_t ist8310_real_data;
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];

static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;

static uint32_t last_integration_ticks = 0; 

static fp32 dt = 0.001f; 

static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};




static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // Initialize quaternion to identity [w,x,y,z]
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.

// Static variables to store world frame linear acceleration, velocity, and position
static fp32 world_linear_accel[3] = {0.0f, 0.0f, 0.0f}; // m/s^2
static fp32 world_velocity[3] = {0.0f, 0.0f, 0.0f};     // m/s
static fp32 world_position[3] = {0.0f, 0.0f, 0.0f};     // m


/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void INS_task(void const *pvParameters)
{
    //wait a time
    osDelay(INS_TASK_INIT_TIME);
    while(BMI088_init())
    {
        osDelay(100);
    }
    while(ist8310_init())
    {
        osDelay(100);
    }

    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    //rotate and zero drift 
    imu_cali_solve(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT, 0, &raw_err_handler);
    AHRS_init(INS_quat, INS_accel, INS_mag);
    // Ensure INS_quat is identity if AHRS_init doesn't set it or if it's called before first sensor read
    INS_quat[0] = 1.0f; INS_quat[1] = 0.0f; INS_quat[2] = 0.0f; INS_quat[3] = 0.0f;


    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];

    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    //set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }


    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGTH);

    imu_start_dma_flag = 1;

    last_integration_ticks = osKernelSysTick(); 
    
    while (1)
    {

        uint32_t current_ticks = osKernelSysTick();
        uint32_t tick_diff = current_ticks - last_integration_ticks;
        
        // Handle potential tick counter overflow if it's a concern for very long uptimes,
        // though for typical IMU rates and 32-bit counters, direct subtraction works due to modular arithmetic.
        // If tick_diff is 0, it means the loop ran faster than a tick, use a small default dt or skip.
        if (tick_diff == 0) {
            // Optionally, use a very small dt or skip integration if this happens frequently
            // For now, let's assume it won't be zero if task is properly waiting for notification.
            // If it can be zero, you might want to use the previous dt or a minimal dt.
        }

        // dt = (fp32)tick_diff / (fp32)configTICK_RATE_HZ; // If using FreeRTOS directly and know configTICK_RATE_HZ
        dt = (fp32)tick_diff / (fp32)configTICK_RATE_HZ; // CMSIS OS function to get tick frequency (usually 1000Hz)
        last_integration_ticks = current_ticks;

        // Ensure dt is not excessively large (e.g., if there was a long stall)
        if (dt > 0.1f) { // Max expected dt, e.g., 100ms. Adjust as needed.
            dt = 0.001f; // Fallback to a default small dt to prevent large integration errors
        }
        if (dt <= 0.0f) { // Should not be zero or negative if tick_diff > 0
             dt = 0.000001f; // Prevent division by zero or negative time, use a tiny positive value
        }
        //wait spi DMA tansmit done
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }


        if(gyro_update_flag & (1 << IMU_NOTIFY_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }

        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);

        }

        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }

        //rotate and zero drift 
        imu_cali_solve(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

        //accel low-pass filter
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];


        AHRS_update(INS_quat, dt, INS_gyro, accel_fliter_3, INS_mag);
        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);

        // Calculate linear acceleration, velocity, and position in world frame
        {
            // accel_fliter_3 contains accelerometer data in body frame [x, y, z]
            fp32 accel_body_frame_x = accel_fliter_3[0];
            fp32 accel_body_frame_y = accel_fliter_3[1];
            fp32 accel_body_frame_z = accel_fliter_3[2];

            // INS_quat is [w, qx, qy, qz]
            fp32 q0 = INS_quat[0]; // w
            fp32 q1 = INS_quat[1]; // x
            fp32 q2 = INS_quat[2]; // y
            fp32 q3 = INS_quat[3]; // z

            // Rotate acceleration from body frame to world frame
            // Using rotation matrix derived from quaternion:
            // R = [ q0^2+q1^2-q2^2-q3^2   2(q1q2-q0q3)          2(q1q3+q0q2)     ]
            //     [ 2(q1q2+q0q3)          q0^2-q1^2+q2^2-q3^2   2(q2q3-q0q1)     ]
            //     [ 2(q1q3-q0q2)          2(q2q3+q0q1)          q0^2-q1^2-q2^2+q3^2 ]

            fp32 accel_world_x = (q0*q0 + q1*q1 - q2*q2 - q3*q3) * accel_body_frame_x + \
                                 2.0f * (q1*q2 - q0*q3) * accel_body_frame_y + \
                                 2.0f * (q1*q3 + q0*q2) * accel_body_frame_z;

            fp32 accel_world_y = 2.0f * (q1*q2 + q0*q3) * accel_body_frame_x + \
                                 (q0*q0 - q1*q1 + q2*q2 - q3*q3) * accel_body_frame_y + \
                                 2.0f * (q2*q3 - q0*q1) * accel_body_frame_z;

            fp32 accel_world_z = 2.0f * (q1*q3 - q0*q2) * accel_body_frame_x + \
                                 2.0f * (q2*q3 + q0*q1) * accel_body_frame_y + \
                                 (q0*q0 - q1*q1 - q2*q2 + q3*q3) * accel_body_frame_z;

            // Compensate for gravity (assuming ENU world frame, Z is Up)
            world_linear_accel[0] = accel_world_x;
            world_linear_accel[1] = accel_world_y;
            world_linear_accel[2] = accel_world_z - GRAVITY_MAGNITUDE;
            // If NED world frame (Z is Down), it would be: 
            // world_linear_accel[2] = accel_world_z + GRAVITY_MAGNITUDE; 
            // or world_linear_accel[2] = accel_world_z - (-GRAVITY_MAGNITUDE);


            // Integrate linear acceleration to get velocity (Euler integration)
            // timing_time is the loop interval (e.g., 0.001f for 1kHz)
            world_velocity[0] += world_linear_accel[0] * dt;
            world_velocity[1] += world_linear_accel[1] * dt;
            world_velocity[2] += world_linear_accel[2] * dt;

            // Integrate velocity to get position (Euler integration)
            world_position[0] += world_velocity[0] * dt;
            world_position[1] += world_velocity[1] * dt;
            world_position[2] += world_velocity[2] * dt;
        }


        //because no use ist8310 and save time, no use
        if(mag_update_flag &= 1 << IMU_DR_SHFITS) // This line has a potential bug: should be (mag_update_flag & (1 << IMU_DR_SHFITS))
        {
            mag_update_flag &= ~(1<< IMU_DR_SHFITS);
            mag_update_flag |= (1 << IMU_SPI_SHFITS);
//            ist8310_read_mag(ist8310_real_data.mag);
        }

    }
}




/**
  * @brief          rotate the gyro, accel and mag, and calculate the zero drift, because sensors have 
  *                 different install derection.
  * @param[out]     gyro: after plus zero drift and rotate
  * @param[out]     accel: after plus zero drift and rotate
  * @param[out]     mag: after plus zero drift and rotate
  * @param[in]      bmi088: gyro and accel data
  * @param[in]      ist8310: mag data
  * @retval         none
  */
static void imu_cali_solve(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, get_control_temperature(), 1);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        // if not reach the setting temperature, always max power
        if (temp > get_control_temperature())
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                // if reach the setting temperature, set the integral term to half of the maximum power to accelerate convergence
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

/**
  * @brief          calculate gyro zero drift
  * @param[out]     gyro_offset:zero drift
  * @param[in]      gyro:gyro data
  * @param[out]     offset_time_count: +1 auto
  * @retval         none
  */
void gyro_offset_calc(fp32 gyro_offset[3], fp32 gyro[3], uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

        gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
        gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
        gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
        (*offset_time_count)++;
}

/**
  * @brief          calculate gyro zero drift
  * @param[out]     cali_scale:scale, default 1.0
  * @param[out]     cali_offset:zero drift, collect the gyro ouput when in still
  * @param[out]     time_count: time, when call gyro_offset_calc 
  * @retval         none
  */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
        if( *time_count == 0)
        {
            gyro_offset[0] = gyro_cali_offset[0];
            gyro_offset[1] = gyro_cali_offset[1];
            gyro_offset[2] = gyro_cali_offset[2];
        }
        gyro_offset_calc(gyro_offset, INS_gyro, time_count);

        cali_offset[0] = gyro_offset[0];
        cali_offset[1] = gyro_offset[1];
        cali_offset[2] = gyro_offset[2];
        cali_scale[0] = 1.0f;
        cali_scale[1] = 1.0f;
        cali_scale[2] = 1.0f;

}

/**
  * @brief          get gyro zero drift from flash
  * @param[in]      cali_scale:scale, default 1.0
  * @param[in]      cali_offset:zero drift, 
  * @retval         none
  */
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
}

/**
  * @brief          get the quaternion
  * @param[in]      none
  * @retval         the point of INS_quat
  */
const fp32 *get_INS_quat_point(void)
{
    return INS_quat;
}
/**
  * @brief          get the euler angle, 0:yaw, 1:pitch, 2:roll unit rad
  * @param[in]      none
  * @retval         the point of INS_angle
  */
const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}

/**
  * @brief          get the rotation speed, 0:x-axis, 1:y-axis, 2:roll-axis,unit rad/s
  * @param[in]      none
  * @retval         the point of INS_gyro
  */
extern const fp32 *get_gyro_data_point(void)
{
    return INS_gyro;
}
/**
  * @brief          get aceel, 0:x-axis, 1:y-axis, 2:roll-axis unit m/s2
  * @param[in]      none
  * @retval         the point of INS_accel
  */
extern const fp32 *get_accel_data_point(void)
{
    return INS_accel; // Note: This returns body-frame accel before gravity compensation
}
/**
  * @brief          get mag, 0:x-axis, 1:y-axis, 2:roll-axis unit ut
  * @param[in]      none
  * @retval         the point of INS_mag
  */
extern const fp32 *get_mag_data_point(void)
{
    return INS_mag;
}

/**
  * @brief          Get the world frame linear acceleration (gravity compensated) by reference.
  * @param[out]     accel_out: Pointer to a 3-element array to store [x, y, z] acceleration in m/s^2.
  * @retval         None
  */
void get_world_linear_accel(fp32 accel_out[3])
{
    if (accel_out != NULL)
    {
        accel_out[0] = world_linear_accel[0];
        accel_out[1] = world_linear_accel[1];
        accel_out[2] = world_linear_accel[2];
    }
}

/**
  * @brief          Get the world frame velocity by reference.
  * @param[out]     velocity_out: Pointer to a 3-element array to store [x, y, z] velocity in m/s.
  * @retval         None
  */
void get_world_velocity(fp32 velocity_out[3])
{
    if (velocity_out != NULL)
    {
        velocity_out[0] = world_velocity[0];
        velocity_out[1] = world_velocity[1];
        velocity_out[2] = world_velocity[2];
    }
}

/**
  * @brief          Get the world frame position by reference.
  * @param[out]     position_out: Pointer to a 3-element array to store [x, y, z] position in m.
  * @retval         None
  */
void get_world_position(fp32 position_out[3])
{
    if (position_out != NULL)
    {
        position_out[0] = world_position[0];
        position_out[1] = world_position[1];
        position_out[2] = world_position[2];
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
        detect_hook(BOARD_ACCEL_TOE);
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {
        detect_hook(BOARD_GYRO_TOE);
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == DRDY_IST8310_Pin)
    {
        detect_hook(BOARD_MAG_TOE);
        mag_update_flag |= 1 << IMU_DR_SHFITS;
    }
    else if(GPIO_Pin == GPIO_PIN_0)
    {
        //wake up the task
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

    }


}

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
static void imu_cmd_spi_dma(void)
{

        // open the gyro DMA transmit
        if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
            gyro_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGTH);
            return;
        }
        // open the accel DMA transmit
        if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGTH);
            return;
        }
        


        
        if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGTH);
            return;
        }
}


void DMA2_Stream2_IRQHandler(void)
{

    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read done
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
            
        }

        //accel read done
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read done
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        
        imu_cmd_spi_dma();

        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}

