/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include "Motor.h"
#include "Encoder.h"
#include "OLED.h"
#include "speed_pi.h"
#include "motor_control.h"
#include "Bsp_Can.h"
#include "can_protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */

volatile uint8_t flag_10ms = 0;

int16_t speed_now = 0;
int16_t pwm_now = 0;
uint8_t oled_count = 0;
int16_t encoder_delta=0;
int16_t encoder_total=0;
int32_t pos_now = 0;
float target_speed_now = 0.0f;
//  系统运行的10ms计数
volatile uint32_t tick_10ms = 0;

//  本次任务开始的tick
uint32_t run_start_tick = 0;

//  本次任务耗时tick
uint32_t run_time_tick = 0;

//  防止完成后反复记录时间
uint8_t time_recorded = 0;
uint8_t finish_screen_updated = 0;


//can 测试
Can_MotorCommand_t test_cmd;
Can_MotorStatus_t slave1_status;

uint32_t status_count = 0;

int32_t test_total_count = 0;
int16_t test_delta_count = 0;
uint8_t test_state = 0;
uint32_t last_send_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//	OLED_Init();

//OLED_ShowString(1, 1, "TGT:");
//OLED_ShowString(2, 1, "POS:");
//OLED_ShowString(3, 1, "SPD:");
//OLED_ShowString(4, 1, "TIME:");

//Motor_Init();
//Encoder_Init();

//MotorControl_Init(
//    0.02f,     // position Kp
//    80.0f,     // speed Kp
//    130.0f     // speed Ki
//);

//Encoder_Reset();
//HAL_TIM_Base_Start_IT(&htim4);

//// [新增]
//run_start_tick = tick_10ms;
//time_recorded = 0;
//MotorControl_SetTarget(28875);
//SpeedPI_Init(80.0f, 130.0f); //测试PI的
//Motor_Forward();
//Motor_SetPWM(1000);//直接定PWM

//上面的电机，下面是CAN测试

//CAN测试   
Can_Init();
//OLED_ShowString(1, 1, "RX:");

test_cmd.command = MOTOR_CMD_RUN;
test_cmd.direction = 1;
test_cmd.target_count = 10000;
test_cmd.target_speed = 50;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//这下面为CAN测试
			
			CanProtocol_Process();


/* 每500ms给Slave1发一次命令 */
if (HAL_GetTick() - last_send_tick >= 500)
{
    last_send_tick = HAL_GetTick();

    CanProtocol_SendMotorCommand(
        1,
        &test_cmd
    );
}


/* 看Slave1有没有回状态 */
if (CanProtocol_GetSlave1Status(&slave1_status))
{
    status_count++;

    test_total_count = slave1_status.total_count;
    test_delta_count = slave1_status.delta_count;
    test_state = slave1_status.state;
}
		
		
		
		
		
		
		
		//下面的为电机测试
//   if (flag_10ms)
//    {
//        flag_10ms = 0;

//        
//				target_speed_now =MotorControl_GetTargetSpeed();
//        pos_now = MotorControl_GetPosition();
//        speed_now = MotorControl_GetSpeed();
//        pwm_now = MotorControl_GetPWM();

//			
////				Encoder_Update();

////        pos_now = Encoder_GetTotalCount();
////        speed_now = Encoder_GetDeltaCount();
//				
//        oled_count++;

//			if (MotorControl_IsReached() &&!time_recorded)
//							{
//									run_time_tick =
//											tick_10ms - run_start_tick;

//									time_recorded = 1;
//							}
//				// [新增]
//				uint32_t show_time_tick;

//				if (time_recorded)
//				{
//						show_time_tick = run_time_tick;
//				}
//				else
//				{
//						show_time_tick =
//								tick_10ms - run_start_tick;
//				}
//			
//			
//        if (oled_count >= 20)
//				{
//				oled_count = 0;

//				/*
//				 * 运行中：
//				 * 正常5Hz刷新
//				 */
//				if (!MotorControl_IsReached())
//				{
//						OLED_ShowSignedNum(
//								1, 6,
//								(int32_t)target_speed_now,
//								3
//						);

//						OLED_ShowSignedNum(
//								2, 6,
//								pos_now,
//								6
//						);

//						OLED_ShowSignedNum(
//								3, 6,
//								speed_now,
//								3
//						);

//						show_time_tick =
//								tick_10ms - run_start_tick;

//						OLED_ShowNum(
//								4, 6,
//								show_time_tick / 100,
//								2
//						);

//						OLED_ShowChar(4, 8, '.');

//						OLED_ShowNum(
//								4, 9,
//								show_time_tick % 100,
//								2
//						);
//				}

//				/*
//				 * [新增]
//				 * 完成以后只刷新一次最终结果
//				 */
//				else if (!finish_screen_updated)
//				{
//						finish_screen_updated = 1;

//						OLED_ShowSignedNum(
//								1, 6,
//								0,
//								3
//						);

//						OLED_ShowSignedNum(
//								2, 6,
//								pos_now,
//								6
//						);

//						OLED_ShowSignedNum(
//								3, 6,
//								0,
//								3
//						);

//						OLED_ShowNum(
//								4, 6,
//								run_time_tick / 100,
//								2
//						);

//						OLED_ShowChar(4, 8, '.');

//						OLED_ShowNum(
//								4, 9,
//								run_time_tick % 100,
//								2
//						);
//				}
//}
//				


//				/*
//				 * run_time_tick单位就是10ms。
//				 *
//				 * 1050 tick
//				 * = 10秒 + 50百分之一秒
//				 */
//				OLED_ShowNum(
//						4,
//						6,
//						show_time_tick / 100,
//						2
//				);

//				OLED_ShowChar(
//						4,
//						8,
//						'.'
//				);

//				OLED_ShowNum(
//						4,
//						9,
//						show_time_tick % 100,
//						2
//				);
//											
//							
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
				tick_10ms++;
				MotorControl_Update();
        flag_10ms = 1;
    }
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
