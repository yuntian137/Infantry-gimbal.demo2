/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "stdio.h"
#include "pid.h"
#include <math.h>
#include "IIC.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#define RPM_TO_RADPS (2.0f * PI / 60.0f)
#define PI acos(-1)

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
/* USER CODE BEGIN Variables */
extern struct sqks_data WaitSolveData; 
extern motor_measure_t motor_chassis[9];
extern pid_type_def pid_speed[8];
extern pid_type_def pid_omega[8];
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart4_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern struct RC_Ctrl_t RC_CtrlData;
extern TIM_HandleTypeDef htim2;
extern uint8_t buffer[18]; 
int check = 1;//控制拨盘电机连发
int checksingle = 1;//控制拨盘电机单发
int Setrpm;//摩擦轮目标转速
int checksetpitch = 1;
int checksetyaw = 1;
int checkling = 1;
int checkdr =1;
int Rx_0Angle;
//DR16输入值
float drpitch;
float dryaw;
//常量
    uint16_t Encoder_Num_Per_Round = 8192;
    //uint16_t Output_Max = 30000;
    uint32_t Encoder_Offset = 0;
//变量  

//pitch
    //总编码值
    int32_t Total_Encoder = 0;
    //接收到的编码值, 0~8191
    uint16_t Rx_Encoder = 0;
    //上一次结束的编码值
    uint16_t Pre_Encoder = 0;
    //接收到的角度值
    float Rx_Angle = 0;
    //接收到的转速, rpm
    float Rx_Omega = 0;
    //接收圈数
    int32_t Total_Round = 0;

//yaw
    //总编码值
    int32_t YawTotal_Encoder = 0;
    //接收到的编码值, 0~8191
    uint16_t YawRx_Encoder = 0;
    //上一次结束的编码值
    uint16_t YawPre_Encoder = 0;
    //接收到的角度值
    float YawRx_Angle = 0;
    //接收到的转速, rpm
    float YawRx_Omega = 0;
    //接收圈数
    int32_t YawTotal_Round = 0;

//拨盘
    //总编码值
    int32_t bpTotal_Encoder = 0;
    //接收到的编码值, 0~8191
    uint16_t bpRx_Encoder = 0;
    //上一次结束的编码值
    uint16_t bpPre_Encoder = 0;
    //接收到的角度值
    float bpRx_Angle = 0;
    //接收到的转速, rpm
    float bpRx_Omega = 0;
    //接收圈数
    int32_t bpTotal_Round = 0;

void pidsolvepitch(int n,float Set_Angle);//pitch
void pidsolveyaw(int n,float Set_Angle);//yaw
void pidsolvebp(int n,float Set_bpAngle);//拨盘
void PrintC(int n,float Set_Angle);
float pitch,roll,yaw; 		    //欧拉角
short aacx,aacy,aacz;			//加速度传感器原始数据
short gyrox,gyroy,gyroz;		//陀螺仪原始数据
float temp;				    //温度
float SetbpAngle = 0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId GM6020id2Handle;
osThreadId jieGMHandle;
osThreadId IMUHandle;
osThreadId pidHandle;
osThreadId myTask06Handle;
osThreadId myTask07Handle;
osThreadId dr16Handle;
osThreadId remove0Handle;
osMessageQId USART_RXPORTHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Get_DR16_Data(uint8_t *Buff);
float Complementary_Filter_x(float angle_m, float gyro_m);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void gm2(void const * argument);
void jbgm(void const * argument);
void IMUwuhu(void const * argument);
void pidxy(void const * argument);
void StartTask06(void const * argument);
void StartTask07(void const * argument);
void dr16Rx(void const * argument);
void guiling(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of USART_RXPORT */
  osMessageQDef(USART_RXPORT, 16, uint16_t);
  USART_RXPORTHandle = osMessageCreate(osMessageQ(USART_RXPORT), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of GM6020id2 */
  osThreadDef(GM6020id2, gm2, osPriorityBelowNormal, 0, 128);
  GM6020id2Handle = osThreadCreate(osThread(GM6020id2), NULL);

  /* definition and creation of jieGM */
  osThreadDef(jieGM, jbgm, osPriorityAboveNormal, 0, 128);
  jieGMHandle = osThreadCreate(osThread(jieGM), NULL);

  /* definition and creation of IMU */
  osThreadDef(IMU, IMUwuhu, osPriorityNormal, 0, 256);
  IMUHandle = osThreadCreate(osThread(IMU), NULL);

  /* definition and creation of pid */
  osThreadDef(pid, pidxy, osPriorityNormal, 0, 128);
  pidHandle = osThreadCreate(osThread(pid), NULL);

  /* definition and creation of myTask06 */
  osThreadDef(myTask06, StartTask06, osPriorityBelowNormal, 0, 128);
  myTask06Handle = osThreadCreate(osThread(myTask06), NULL);

  /* definition and creation of myTask07 */
  osThreadDef(myTask07, StartTask07, osPriorityBelowNormal, 0, 128);
  myTask07Handle = osThreadCreate(osThread(myTask07), NULL);

  /* definition and creation of dr16 */
  osThreadDef(dr16, dr16Rx, osPriorityNormal, 0, 128);
  dr16Handle = osThreadCreate(osThread(dr16), NULL);

  /* definition and creation of remove0 */
  osThreadDef(remove0, guiling, osPriorityAboveNormal, 0, 128);
  remove0Handle = osThreadCreate(osThread(remove0), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_gm2 */
/**
* @brief Function implementing the GM6020id2 thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_gm2 */
void gm2(void const * argument)
{
    vTaskDelay(3000);//延时3s保证稳定抬头
  /* USER CODE BEGIN gm2 */
    TickType_t lasttick = xTaskGetTickCount();    
  /* Infinite loop */
  for(;;)
  {
    //  //pitch

    if (checksetpitch)
    {
      pidsolvepitch(5,drpitch);

      //pitch前馈
      // pid_speed[n].out+= -(120.82f*(-Rx_Angle)) - 3623.5f;
      // pid_speed[n].out += -115.94f*(-Rx_Angle) - 1379.2f;
      pid_speed[5].out += -96.61f * (-Rx_Angle) - 2570.5f;//这套最好
      // 限制加前馈后的最大输出，需要注意，can包发送函数传参最大值为32767，如果不限制，会爆范围导致骤降到-32767，导致堵转。
      if (pid_speed[5].out > 30000)
      {
        pid_speed[5].out = 30000;
      }
      else if (pid_speed[5].out < -30000)
      {
        pid_speed[5].out = -30000;
      }
      CAN1_cmd_gimbal(0, pid_speed[5].out, 0, 0);
    }
    else
    {
      CAN1_cmd_gimbal(0, 0, 0, 0);
    }

     //yaw
    if (checksetyaw)
    {
      pidsolveyaw(7, dryaw);
      CAN2_cmd_gimbal(0, 0, 0, pid_speed[7].out);
    }
    else
    {
      CAN2_cmd_gimbal(0, 0, 0, 0);
    }

    vTaskDelayUntil(&lasttick,pdMS_TO_TICKS(1));
  }
  /* USER CODE END gm2 */
}

/* USER CODE BEGIN Header_jbgm */
/**
* @brief Function implementing the jieGM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_jbgm */
void jbgm(void const * argument)
{
  /* USER CODE BEGIN jbgm */
  //处理DR16反馈数据
  //可以通过debug查看数据
   uint8_t BUFF[18];
    //float data[3];
  /* Infinite loop */
  for(;;)
  {
        Get_DR16_Data(buffer);
       if(xQueueReceive(USART_RXPORTHandle,&BUFF,20) != pdPASS) 
       {
        //掉电保护，要求yaw和pitch电机输出为0，摩擦轮目标转速为0.
        
        //yaw电机输出为零
       checksetyaw = 0;

        //pitch电机输出为零
       checksetpitch = 0;

        //摩擦轮目标转速置零
        Setrpm = 0;

        //波胆电机
        checksingle = 0;
        checkdr = 0;

        }else
        {
          //yaw上电恢复
          checksetyaw = 1;

          //pitch上电恢复
          checksetpitch = 1;

          checksingle = 1;
          checkdr = 1;
        }
  }
  /* USER CODE END jbgm */
}

/* USER CODE BEGIN Header_IMUwuhu */
/**
* @brief Function implementing the IMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMUwuhu */
void IMUwuhu(void const * argument)
{
  /* USER CODE BEGIN IMUwuhu */
  /* Infinite loop */
  for(;;)
  {
    osDelay(5);
        while(mpu_dmp_get_data(&pitch, &roll, &yaw));	//必须要用while等待，才能读取成功
        MPU_Get_Accelerometer(&aacx,&aacy, &aacz);		//得到加速度传感器数据
        MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);		//得到陀螺仪数据
    osDelay(1);
  }
  /* USER CODE END IMUwuhu */
}

/* USER CODE BEGIN Header_pidxy */
/**
* @brief Function implementing the pid thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pidxy */
void pidxy(void const * argument)
{
  /* USER CODE BEGIN pidxy */
  /* Infinite loop */
  for(;;)
  {
    //拨盘电机
    pidsolvebp(0,SetbpAngle);
    //左右摩擦轮转动方向相反
    PID_calc(&pid_speed[1],motor_chassis[1].speed_rpm,Setrpm);
    //右摩擦轮
    PID_calc(&pid_speed[2],motor_chassis[2].speed_rpm,-Setrpm);
    CAN_cmd_chassis(pid_speed[0].out,pid_speed[1].out,pid_speed[2].out,0);
    osDelay(1);
  }
  /* USER CODE END pidxy */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the myTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void const * argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
  for (;;)
  {
    // 控制弹仓盖开启和关闭
    //还有摩擦轮
    if (RC_CtrlData.remote.s2 == 1)
    {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 250);
        if(Setrpm != 0)
        Setrpm = 0;
    }
    else if (RC_CtrlData.remote.s2 == 3)
    {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 1170);
    }else if (RC_CtrlData.remote.s2 == 2)
    {
        //打开摩擦轮
        Setrpm = 6000;
    }

    //s1控制破盘
    if (RC_CtrlData.remote.s1 == 1) // 每五十毫秒递增一次target
    {
          // 连发
          check = 1;
          SetbpAngle -= 36 * 36 * check * checkdr;
          TickType_t lasttick2 = xTaskGetTickCount();
          vTaskDelayUntil(&lasttick2, pdMS_TO_TICKS(50));
    }
    else if (RC_CtrlData.remote.s1 == 3)
    {
          check = 0;
          checksingle = 1;
    }
    else if(RC_CtrlData.remote.s1 == 2)
    {
          // 单发
          SetbpAngle -= 36 * 36 * checksingle;
          checksingle = 0;
    }
    osDelay(2);
  }
  
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the myTask07 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void const * argument)
{
  /* USER CODE BEGIN StartTask07 */
  /* Infinite loop */
  for(;;)
  {
      //在这里存放调试曲线的打印函数
      //上位机为vofa+，文件末尾有示例
    }
  /* USER CODE END StartTask07 */
}

/* USER CODE BEGIN Header_dr16Rx */
/**
* @brief Function implementing the dr16 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dr16Rx */
void dr16Rx(void const * argument)
{
  /* USER CODE BEGIN dr16Rx */
  /* Infinite loop */
  for(;;)
  {
    //限制pitch最大角度
      drpitch += (RC_CtrlData.remote .ch1/660);
        if(drpitch < -40)
    {
      drpitch = -39.9;
    }else if (drpitch > 20)
    {
      drpitch = 19.9;
    }

    dryaw += (RC_CtrlData.remote .ch2/660);
  
    osDelay(10);
  }
  /* USER CODE END dr16Rx */
}

/* USER CODE BEGIN Header_guiling */
/**
* @brief Function implementing the remove0 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_guiling */
int Angle = 0;
void guiling(void const * argument)
{
  /* USER CODE BEGIN guiling */
  /* Infinite loop */
  for(;;)
  {
    // if (checksetpitch != 0)
    //  {
    //   int16_t delta_encoder;
    //   Pre_Encoder = Rx_Encoder;
    //   Rx_Encoder = motor_chassis[5].ecd;
    //   delta_encoder = Rx_Encoder - Pre_Encoder;
    //   Rx_Omega = motor_chassis[5].speed_rpm;
    //   if (delta_encoder < -4096)
    //   {
    //     Total_Round++;
    //   }
    //   else if (delta_encoder > 4096)
    //   {
    //     Total_Round--;
    //   }
    //   Total_Encoder = Total_Round * Encoder_Num_Per_Round + Rx_Encoder + 0;

    //   Rx_0Angle = ((float)Total_Encoder / (float)Encoder_Num_Per_Round) * 360;
    //   Rx_Omega = (float)Rx_Omega * (360 / 60);
    //   PID_calc(&pid_omega[3], Rx_0Angle, Angle);
    //   PID_calc(&pid_speed[3], Rx_Omega, pid_omega[3].out);
    //   CAN1_cmd_gimbal(0, pid_speed[3].out, 0, 0);
    
    //  }
    //  else if (checksetpitch == 0)
    //  {
    //   CAN1_cmd_gimbal(0, 0, 0, 0);
    //  }

    //   if(Rx_0Angle = 0)
    // {
    //     vTaskSuspend(remove0Handle);
    // }
        
    osDelay(1);
  }
  /* USER CODE END guiling */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void pidsolvepitch(int n,float Set_pitchAngle)
{

    int16_t delta_encoder;
     Pre_Encoder =  Rx_Encoder;
     Rx_Encoder = motor_chassis[n].ecd;
     delta_encoder = Rx_Encoder - Pre_Encoder;
     Rx_Omega = motor_chassis[n].speed_rpm;
     
      if (delta_encoder < -4096)
    {
        Total_Round++;
    }
    else if (delta_encoder > 4096)
    {
        Total_Round--;
    }
    Total_Encoder = Total_Round * Encoder_Num_Per_Round + Rx_Encoder + 0;
    //Rx_Angle = (float)Total_Encoder/(float)Encoder_Num_Per_Round * 360;
    //Now_Torque = motor_chassis[n].given_current;
    Rx_Omega = (float)Rx_Omega * (360/60);
    Rx_Angle = Complementary_Filter_x(pitch,gyroy/16.384f);//互补滤波算法
    PID_calc(&pid_omega[n],-Rx_Angle,Set_pitchAngle);
    PID_calc(&pid_speed[n],-gyroy/16.384,pid_omega[n].out);

}

void pidsolveyaw(int n,float Set_yawAngle)
{
    int16_t delta_encoder;
     YawPre_Encoder =  YawRx_Encoder;
     YawRx_Encoder = motor_chassis[n].ecd;
     delta_encoder = YawRx_Encoder - YawPre_Encoder;
     YawRx_Omega = motor_chassis[n].speed_rpm;
     
      if (delta_encoder < -4096)
    {
        YawTotal_Round++;
    }
    else if (delta_encoder > 4096)
    {
        YawTotal_Round--;
    }
    YawTotal_Encoder = YawTotal_Round * Encoder_Num_Per_Round + YawRx_Encoder + 0;
    YawRx_Angle = (float)YawTotal_Encoder/(float)Encoder_Num_Per_Round * 360;
    //Now_Torque = motor_chassis[n].given_current;
    YawRx_Omega = (float)YawRx_Omega * (360/60);
    PID_calc(&pid_omega[n],YawRx_Angle,Set_yawAngle);
    PID_calc(&pid_speed[n],YawRx_Omega,pid_omega[n].out);
}

void pidsolvebp(int n,float Set_bpAngle)
{
    int16_t delta_encoder;
     bpPre_Encoder =  bpRx_Encoder;
     bpRx_Encoder = motor_chassis[n].ecd;
     delta_encoder = bpRx_Encoder - bpPre_Encoder;
     bpRx_Omega = motor_chassis[n].speed_rpm;
     
      if (delta_encoder < -4096)
    {
        bpTotal_Round++;
    }
    else if (delta_encoder > 4096)
    {
        bpTotal_Round--;
    }
    bpTotal_Encoder = bpTotal_Round * Encoder_Num_Per_Round + bpRx_Encoder + 0;
    bpRx_Angle = (float)bpTotal_Encoder/(float)Encoder_Num_Per_Round * 360;
    bpRx_Omega = (float)bpRx_Omega * RPM_TO_RADPS;
    PID_calc(&pid_omega[n],bpRx_Angle,Set_bpAngle);
    PID_calc(&pid_speed[n],bpRx_Omega,pid_omega[n].out);
}

void PrintC(int n,float Set_Angle)
{

  float data[6];
    
    //打印非IMU
    //data[0] = Rx_Angle;
    //打印IMU(pitch)
    data[0] = -Rx_Angle;
    data[1] = Set_Angle;
    data[2] = pid_omega[n].Iout;
    data[3] = pid_omega[n].out;
    data[4] = pid_speed[n].Iout;
    data[5] = pid_speed[n].out;
    HAL_UART_Transmit(&huart4,(uint8_t*)data,sizeof(float)*6,0xFFFF);
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    HAL_UART_Transmit(&huart4,tail,4,0xFFFF);
}

/****************************** BEFIN ********************************
**
**@Name       : Complementary_Filter_x
**@Brief      : 一阶互补滤波   
**@Param angle_m: 加速度算出的角度 
**		gyro_m: 陀螺仪的角速度
**@Return     : None
**@Author     : @mayuxin
**@Data	      : 2022-06-04
******************************** END *********************************/
float dt=0.005;//每5ms进行一次滤波  
float Complementary_Filter_x(float angle_m, float gyro_m)
{
	 static float angle;
	 float K1 =0.6; 
   angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * dt);
	 return angle;
}

/* USER CODE END Application */
