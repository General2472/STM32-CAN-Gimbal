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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>          /* atan2：必须显式包含，否则隐式声明会按返回 int 处理，姿态角恒为 0 */
#include "OLED.h"
#include "mpu6500.h"
#include "steer.h"
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
  //用来存放加速度和陀螺仪的三轴值
  int16_t accel[3];
  int16_t gyro[3];
  //存放陀螺仪的三轴偏置值
  float gx_bias,gy_bias,gz_bias;
  //加速度和陀螺仪的最终应用值
  float ax,ay,az;
  float gx,gy,gz;

  //PID控制器实例（pitch和roll各一份，参数独立可调）
  PID_t pitch_pid;
  PID_t roll_pid;
  float pid_out;
  float pid_out_roll;

//按键检测变量
uint8_t key_count;
volatile uint8_t key_flag;
uint8_t oled_choice=0;

//互补滤波姿态角
float pitch = 0, roll = 0;
#define dt  0.01f
#define ALPHA 0.98f
#define Pitch_Base 4.0f
#define Roll_Base  0.0f    /* roll水平零点（如实际水平时有偏，标定后填入） */
#define Pitch_Bias 3.0f
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void OledChoice(void){
  if(key_flag==1){
    oled_choice = !oled_choice;
    OLED_Clear();
    OLED_ShowString(1, 1, "MPU6500");
    OLED_ShowString(2,1,"Ax/Gx:");
    OLED_ShowString(3,1,"Ay/Gy:");
    OLED_ShowString(4,1,"Az/Gz:");

  }
}

/* CAN发送：把pitch和roll角度发出去 */
void CAN_Send_Attitude(float pitch, float roll)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;

    /* 帧头配置 */
    tx_header.StdId = 0x123;        /* 本节点ID */
    tx_header.IDE = CAN_ID_STD;     /* 标准帧 */
    tx_header.RTR = CAN_RTR_DATA;   /* 数据帧 */
    tx_header.DLC = 2;              /* 4字节数据 */

    /* 角度×100转整数，保留两位小数 */
    int8_t pitch_i = (int8_t)(pitch);
    int8_t roll_i  = (int8_t)(roll);

    /* 拆成高低字节 */
    tx_data[0] = pitch_i & 0xFF;
    tx_data[1] = roll_i & 0xFF;

    HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox);
}
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  MPU_I2C_Init();
  //得到陀螺仪各轴的偏置
  Calibrate_Gyro(&gx_bias,&gy_bias,&gz_bias);
  //启动定时中断
  HAL_TIM_Base_Start_IT(&htim3);
  //开启PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  
  
  OLED_Clear();
  OLED_ShowString(1, 1,"MPU6500");
  OLED_ShowString(2,1,"pitch:");
  OLED_ShowString(3,1,"roll:");

  //初始化俩舵机位置
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Pitch_Init_Ccr);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Roll_Init_Ccr);

  //初始化PID参数（先只开P，Kp从1.0开始试调）
  pitch_pid.Kp = 1.3f;
  pitch_pid.Ki = 0.3f;
  pitch_pid.Kd = 0.15f;
  pitch_pid.integral = 0;
  pitch_pid.last_error = 0;

  //roll PID参数和pitch调成一致
  roll_pid.Kp = 1.0f;
  roll_pid.Ki = 0.1f;
  roll_pid.Kd = 0.1f;
  roll_pid.integral = 0;
  roll_pid.last_error = 0;

  //启动CAN
  HAL_CAN_Start(&hcan);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  OLED_ShowSignedNum(2,7,pitch-Pitch_Bias,6);
  OLED_ShowSignedNum(3,6,roll,6);
  CAN_Send_Attitude(pitch, roll);   /* 发CAN：ID=0x123，4字节 */
  HAL_Delay(100);                    /* 100ms发一帧 */
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

//TIM3中断函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        // 按键检测
        if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET) {
            key_count++;
            if (key_count >= 3) {
                key_count = 0;
                key_flag = 1;
                while(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET);
            }
        }

        // 1. 读MPU
        MPU_ReadAandG(accel, gyro);

        // 2. 减零偏，换算成°/s
        gx = (gyro[0] - gx_bias) / 131.0f;
        gy = (gyro[1] - gy_bias) / 131.0f;

        // 3. 加速度算静态角度
        float acc_pitch = atan2(accel[1], accel[2]) * 57.3f;
        float acc_roll  = atan2(-accel[0], accel[2]) * 57.3f;

        // 4. 互补滤波
        pitch = ALPHA * (pitch + gx * dt) + (1-ALPHA) * acc_pitch;
        roll  = ALPHA * (roll  + gy * dt) + (1-ALPHA) * acc_roll;

        // 5. PID控制：让 pitch 回到 Pitch_Base（水平零点）
        float error = Pitch_Base - pitch;              /* 误差 = 目标 - 当前 */
        pid_out = PID_Calc(&pitch_pid, error, dt);     /* 算出角度修正量 */
        if(fabs(error) < 2.0f) pid_out = 0;      // pitch
        steer_pitch_pid(pid_out);                      /* 驱动舵机 */

        // 6. PID控制：让 roll 回到 Roll_Base（水平零点）
        float error_roll = Roll_Base - roll;           /* 误差 = 目标 - 当前 */
        pid_out_roll = PID_Calc(&roll_pid, error_roll, dt);
        if(fabs(error_roll) < 3.0f) pid_out_roll = 0;  // roll
        steer_roll_pid(pid_out_roll);                  /* 驱动roll舵机 */

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
