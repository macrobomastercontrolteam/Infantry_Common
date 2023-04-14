/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CHASSIS_CONTROL_TIME_MS 2
#define CHASSIS_STEER_MOTOR_CONTROL_TIME_MS (CHASSIS_CONTROL_TIME_MS / 2) // faster turning control

#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191
#define MOTOR_ECD_TO_RAD 0.000766990394f // (2*PI)/8192

#define M6020_MOTOR_ANGLE_PID_KP 100.0f
#define M6020_MOTOR_ANGLE_PID_KI 5.0f
#define M6020_MOTOR_ANGLE_PID_KD 5.0f
// chassis 6020 max motor control voltage
#define MAX_MOTOR_CAN_VOLTAGE 20000.0f
#define M6020_MOTOR_ANGLE_PID_MAX_OUT MAX_MOTOR_CAN_VOLTAGE
#define M6020_MOTOR_ANGLE_PID_MAX_IOUT 10000.0f

#define M6020_MOTOR_0_ANGLE_ECD_OFFSET 1662U
#define M6020_MOTOR_1_ANGLE_ECD_OFFSET 3742U
#define M6020_MOTOR_2_ANGLE_ECD_OFFSET 5720U
#define M6020_MOTOR_3_ANGLE_ECD_OFFSET 2827U
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern moto_info_t motor_info[STEER_MOTOR_COUNT];
pid_struct_t motor_pid[STEER_MOTOR_COUNT];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
float abs_angle_pid_calc(pid_struct_t *pid, float target_ecd, float feedback_ecd);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOH, POWER1_CTRL_Pin | POWER2_CTRL_Pin | POWER3_CTRL_Pin | POWER4_CTRL_Pin, GPIO_PIN_SET); // switch on 24v power
  can_user_init();                                                                                               // config can filter, start can
  const static uint16_t steer_motor_offset_ecd[4] = {M6020_MOTOR_0_ANGLE_ECD_OFFSET, M6020_MOTOR_1_ANGLE_ECD_OFFSET, M6020_MOTOR_2_ANGLE_ECD_OFFSET, M6020_MOTOR_3_ANGLE_ECD_OFFSET};
  for (uint8_t i = 0; i < STEER_MOTOR_COUNT; i++)
  {
    pid_init(&motor_pid[i], M6020_MOTOR_ANGLE_PID_KP, M6020_MOTOR_ANGLE_PID_KI, M6020_MOTOR_ANGLE_PID_KD, M6020_MOTOR_ANGLE_PID_MAX_IOUT, M6020_MOTOR_ANGLE_PID_MAX_OUT);
    motor_info[i].offset_ecd = steer_motor_offset_ecd[i];
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    for (uint8_t i = 0; i < STEER_MOTOR_COUNT; i++)
    {
      motor_info[i].set_voltage = abs_angle_pid_calc(&motor_pid[i], (float)motor_info[i].target_ecd, (float)motor_info[i].feedback_ecd - (float)motor_info[i].offset_ecd);
    }
    /* send motor control message through can bus*/
    CAN_cmd_steer_motors(0, motor_info[0].set_voltage, motor_info[1].set_voltage, motor_info[2].set_voltage, motor_info[3].set_voltage);

    HAL_Delay(CHASSIS_STEER_MOTOR_CONTROL_TIME_MS);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief Constrain ecd to range [-HALF_ECD_RANGE,HALF_ECD_RANGE]
 */
float loop_ecd_constrain(float Input)
{
  if (Input > HALF_ECD_RANGE)
  {
    while (Input > HALF_ECD_RANGE)
    {
      Input -= ECD_RANGE;
    }
  }
  else if (Input < -HALF_ECD_RANGE)
  {
    while (Input < -HALF_ECD_RANGE)
    {
      Input += ECD_RANGE;
    }
  }
  return Input;
}

/**
  * @brief  pid calculation
  * @param  pid struct
    @param  target_ecd encoder range [-HALF_ECD_RANGE, HALF_ECD_RANGE]
    @param  feedback_ecd encoder range [-HALF_ECD_RANGE, HALF_ECD_RANGE]
  * @retval calculation result
  */
float abs_angle_pid_calc(pid_struct_t *pid, float target_ecd, float feedback_ecd)
{
  pid->err[1] = pid->err[0];
  pid->err[0] = loop_ecd_constrain(target_ecd - feedback_ecd);

  pid->p_out = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);

  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
