/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "pid.h"
#include "stdio.h"
#include "IIC.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
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
pid_type_def pid_speed[8];
pid_type_def pid_omega[8];

struct RC_Ctrl_t RC_CtrlData;
uint8_t buffer[18]; 

extern osThreadId IMUHandle;
extern osThreadId GM6020id2Handle;
extern osThreadId remove0Handle;
extern osThreadId dr16Handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// //pid参数

//注意！！！摩擦轮和拨弹电机都是基于2PI/60计算的！
//拨弹电机
float Speed0_pid [3] = {88,0.0,0.06};
fp32 Omega0_pid [3] = {0.795,0.0,0.06};

//左摩擦轮
fp32 Speed1_pid [3] = {10.0,0.01,0.001};
fp32 Omega1_pid [3] = {17.5,0.0,0.00};

//右摩擦轮
fp32 Speed2_pid [3] = {10.0,0.01,0.001};
fp32 Omega2_pid [3] = {17.5,0.0,0.00};

//归零
fp32 Speed3_pid [3] = {30.0,0,0.00};
fp32 Omega3_pid [3] = {22,0,0.00};

//pitch
fp32 Speed5_pid [3] = {112.0,0,0.00};
fp32 Omega5_pid [3] = {8.0,0.135,0.001};

//yaw
fp32 Speed7_pid [3] = {100.0,0,0.00};
fp32 Omega7_pid [3] = {8.0,1.50,0.00};


void Get_DR16_Data(uint8_t *Buff)
	{
		RC_CtrlData.remote .ch0 =(Buff[0]|Buff[1]<<8)&0x07FF;
		RC_CtrlData.remote .ch0 -=1024;
	  RC_CtrlData.remote .ch1 =(Buff [1]>>3|Buff[2]<<5)&0x07FF;
		RC_CtrlData.remote .ch1 -=1024;
		RC_CtrlData.remote .ch2 =(Buff[2]>>6|Buff[3]<<2|Buff[4]<<10)&0x07FF;
		RC_CtrlData.remote .ch2 -=1024;
		RC_CtrlData.remote .ch3 =(Buff[4]>>1|Buff[5]<<7)&0x07FF;
		RC_CtrlData.remote .ch3 -=1024;
		RC_CtrlData.remote .s1 =(Buff[5]>>4&0x000C)>>2;
		RC_CtrlData.remote .s2 =(Buff[5]>>4)&0x003;
		RC_CtrlData.mouse .x =(Buff[6]|Buff[7]<<8);
		RC_CtrlData.mouse .y =(Buff[8]|Buff[9]<<8);
		RC_CtrlData.mouse .z =(Buff[10]|Buff[11]<<8);
		RC_CtrlData.mouse .press_left =(Buff[12]);
		RC_CtrlData.mouse .press_right =(Buff[13]);
	}//A5 4C 00 08 FC 45 00 08 00 00 6C E9 00 00 00 00 00 00
     //9D 4C 00 08 F4 45 00 08 00 00 00 04 00 00 00 00 00 00
     //F1 4B 00 08 2C 45 00 08 00 00 00 04 00 00 00 00 00 00
     //F1 4B 00 08 2C 45 00 08 00 00 94 06 00 00 00 00 00 00
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    //HAL_Delay(2000);
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
    can_filter_init();
    //归零初始化
    PID_init(&pid_omega[3],PID_POSITION,Omega3_pid,1000,15);
    PID_init(&pid_speed[3],PID_POSITION,Speed3_pid,30000,10000);

    //2006拨弹电机初始化
    PID_init(&pid_omega[0],PID_POSITION,Omega0_pid,16000,1000);
    PID_init(&pid_speed[0],PID_POSITION,Speed0_pid,10000,1000);

    //左摩擦轮初始化
    PID_init(&pid_omega[1],PID_POSITION,Omega1_pid,16000,2500);
    PID_init(&pid_speed[1],PID_POSITION,Speed1_pid,16000,2500);

    //右摩擦轮初始化
    PID_init(&pid_omega[2],PID_POSITION,Omega2_pid,16000,2500);
    PID_init(&pid_speed[2],PID_POSITION,Speed2_pid,16000,2500);

    //pitch电机初始化
    PID_init(&pid_omega[5],PID_POSITION,Omega5_pid,2000,15);
    PID_init(&pid_speed[5],PID_POSITION,Speed5_pid,30000,10000);

    //yaw电机初始化
    PID_init(&pid_omega[7],PID_POSITION,Omega7_pid,20000,3);
    PID_init(&pid_speed[7],PID_POSITION,Speed7_pid,20000,1000);
    __HAL_UART_ENABLE_IT(&huart2 ,UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart2,buffer,18);
    //__HAL_UART_ENABLE_IT(&huart4 ,UART_IT_IDLE);

   HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
   MPU_Init();     // MPU6050初始化
        mpu_dmp_init(); // dmp初始化
        while (mpu_dmp_init() != 0)
        {
        printf("MPU6050 INIT ERROR\r\n");
        } // 初始化失败大概率是接线不稳定

   /* USER CODE END 2 */

   /* Call init function for freertos objects (in freertos.c) */
   MX_FREERTOS_Init();

   /* Start scheduler */
   osKernelStart();
   
//   vTaskSuspend(GM6020id2Handle);
//   vTaskSuspend(IMUHandle);
//   vTaskSuspend(dr16Handle);

   /* We should never get here as control is now taken by the scheduler */
   /* Infinite loop */
   /* USER CODE BEGIN WHILE */

   while (1)
   {
     // CAN_cmd_gimbal(0, 10000, 0, 0);
     // HAL_Delay(2);

     /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
