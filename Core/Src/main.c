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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 电机信息
typedef struct
{
  CAN_HandleTypeDef *can_handle;
  CAN_TxHeaderTypeDef txconf;
  uint32_t tx_id;
  uint32_t tx_mailbox;
  uint8_t tx_buff[8];
  uint8_t rx_buff[8];
  uint32_t rx_id;
  uint8_t rx_len;
} CANInstance;

// DJI电机反馈信息
typedef struct
{
    // 原始数据
    uint16_t ecd;                   // 0-8191,刻度总共有8192格
    int16_t speed_rpm;              // 转速,单位为rpm
    int16_t real_current;           // 实际电流
    uint8_t temperature;            // 温度 Celsius

    // 处理后的数据
    int16_t real_current_handle;    // 处理后的电流
    float speed_aps;                // 角速度,单位为:度/秒
    uint16_t last_ecd;              // 上一次读取的编码器值
    float angle_single_round;       // 单圈角度
    float total_angle;              // 总角度,注意方向
    int32_t total_round;            // 总圈数,注意方向

} DJI_Motor_Measure_s;

#define PI                      3.14159265358979f
float t = 0.0f;                 // 用于正弦波计算的时间变量
CANInstance motor1;
DJI_Motor_Measure_s motor1_measure;
int16_t current;
float current_fitter = 0.9f;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
    static CAN_RxHeaderTypeDef rxconf; // 同上
    uint8_t can_rx_buff[8];
    while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff); // 从FIFO中获取数据
        if (_hcan == motor1.can_handle && rxconf.StdId == motor1.rx_id)
        {
            motor1.rx_len = rxconf.DLC;                         // 保存接收到的数据长度
            memcpy(motor1.rx_buff, can_rx_buff, rxconf.DLC);    // 消息拷贝到对应实例
            motor1_measure.last_ecd = motor1_measure.ecd;
            // 解析电机数据
            motor1_measure.ecd = (uint16_t)(motor1.rx_buff[0] << 8 | motor1.rx_buff[1]);
            motor1_measure.speed_rpm = (int16_t)(motor1.rx_buff[2] << 8 | motor1.rx_buff[3]);
            motor1_measure.real_current = (int16_t)(motor1.rx_buff[4] << 8 | motor1.rx_buff[5]);
            motor1_measure.temperature = (int8_t)motor1.rx_buff[6];
            // 处理数据
            motor1_measure.angle_single_round = (float)(360.0f/8192.0f) * (float)motor1_measure.ecd;
            
            motor1_measure.speed_aps = (1.0f - 0.85f) * motor1_measure.speed_aps + 0.85f * (360.0f / 60.0f) * (float)motor1_measure.speed_rpm;
            
            motor1_measure.real_current_handle = (1.0f - current_fitter) * motor1_measure.real_current_handle + current_fitter * (float)motor1_measure.real_current;

            // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
            if (motor1_measure.ecd - motor1_measure.last_ecd > 4096)
                motor1_measure.total_round--;
            else if (motor1_measure.ecd - motor1_measure.last_ecd < -4096)
                motor1_measure.total_round++;
            motor1_measure.total_angle = motor1_measure.total_round * 360 + motor1_measure.angle_single_round;
            return; 
        }
        
    }
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  // 初始化CAN服务
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  // 配置电机的基本参数
  motor1.txconf.StdId = 0x200;
  motor1.txconf.IDE = CAN_ID_STD;
  motor1.txconf.RTR = CAN_RTR_DATA;
  motor1.txconf.DLC = 8;
  motor1.can_handle = &hcan1;
  motor1.tx_id = 0x200;
  motor1.rx_id = 0x201;

  // 配置电机的过滤器
  /* 配置这个电机的过滤器 */
  CAN_FilterTypeDef can_filter_conf;
  static uint8_t can1_filter_idx = 0, can2_filter_idx = 14; // 0-13给can1用,14-27给can2用
  can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;
  can_filter_conf.FilterFIFOAssignment = (motor1.tx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
  can_filter_conf.SlaveStartFilterBank = 14;
  can_filter_conf.FilterIdLow = motor1.rx_id << 5;
  can_filter_conf.FilterBank = motor1.can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++);
  can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;
  HAL_CAN_ConfigFilter(motor1.can_handle, &can_filter_conf);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    current = 1000* sin(t*PI);
    t+= 0.02f;
    // current = 0; // 暂时不发送电流
    if(HAL_CAN_GetTxMailboxesFreeLevel(motor1.can_handle) == 0); // 等待邮箱空闲
    else
    {
      motor1.tx_buff[0] = (uint8_t)(current >> 8);       // 高八位
      motor1.tx_buff[1] = (uint8_t)(current & 0x00ff);    // 低八位
      motor1.tx_buff[2] = 0;
      motor1.tx_buff[3] = 0;
      motor1.tx_buff[4] = 0;
      motor1.tx_buff[5] = 0;
      motor1.tx_buff[6] = 0;
      motor1.tx_buff[7] = 0;
      HAL_CAN_AddTxMessage(motor1.can_handle, &motor1.txconf, motor1.tx_buff, &motor1.tx_mailbox);
    }
    HAL_Delay(10); // 10ms发送一次
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
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
