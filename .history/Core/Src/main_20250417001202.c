/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION < 6000000)
#error "不支持使用 Keil ARM Compiler V5编译器，请使用 Keil ARM Compiler V6编译"
#endif
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "motor/motor_runtime_param.h"
#include "motor/foc.h"
#include "global_def.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void set_pwm_duty(float d_u, float d_v, float d_w)
{
  // 限制占空比最高为90%，留出一个电流稳定的时段，有利于减少电机抖动以及给后续ADC采样提供稳定电流时段。原因是：占空比接近100%时，会出现mos关闭后瞬间开启的情况，可能会导致上下桥臂同时导通导致短路；没有足够时间进行电流采样（通常在 PWM 低电平期间采样，需预留 10% 的周期用于采样电路稳定）。
  d_u = min(d_u, 0.9); // 留出10%的电流采样时间
  d_v = min(d_v, 0.9);
  d_w = min(d_w, 0.9);
  __disable_irq();                                                         // 关闭全局中断，确保设置 PWM 比较值的过程为 原子操作，避免被中断打断导致三相占空比不同步。目标：通过三个通道的 PWM 信号驱动三相全桥电路的 6 个功率管（每相上下桥臂各 1 个），实现磁场定向控制（FOC）或六步换相。
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, d_u * htim1.Instance->ARR); // d_x * ARR：计算高电平持续时间（如 ARR=1000，d=0.5 时，CCR=500，占空比 50%）
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, d_v * htim1.Instance->ARR);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, d_w * htim1.Instance->ARR);
  __enable_irq(); // 开启全局中断
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  // PID系数，之后的调参也是调节这里的参数
  set_motor_pid(
      3.5, 0, 7,
      0.02, 0.001, 0,
      1.2, 0.02, 0,
      1.2, 0.02, 0);

  extern uint8_t mt6701_rx_data[3];
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_DMA(&hspi1, mt6701_rx_data, mt6701_rx_data, 3); // 将源地址（MT6701位置传感器，通过SPI进行通信）将读取到的数据通过DMA发送给目标地址（MCU中的内存缓冲区）
  HAL_Delay(10);                                                          // 延时一会，让角度变量被赋值，不然默认角度是0
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);                               // 启动了定时器1的通道1的PWM功能
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);                               // 启动了定时器1的通道2的PWM功能
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);                               // 启动了定时器1的通道3的PWM功能
  HAL_TIM_Base_Start_IT(&htim3);
  // ADC电流采样
  /*
  校准目的
    --消除偏移 / 增益误差：ADC 硬件存在固有偏移（Offset）和增益（Gain）误差，校准通过内部参考电压计算修正值，确保采样数据准确。
    --电流采集精度需求：电机控制中，相电流（i_u、i_v）是 FOC 算法的核心输入，误差会导致磁场定向错误、转矩波动甚至硬件损坏。
  */
  HAL_ADCEx_Calibration_Start(&hadc1); // 校准 ADC1，确保采样精度
  HAL_ADCEx_Calibration_Start(&hadc2); // 校准 ADC2
  /*
  启动注入通道转换
  注入通道（Injected Channel）特性
    高优先级采样：独立于常规通道，可打断常规转换，适用于实时性要求高的场景（如电机相电流采样）。
    触发方式：项目中配置为 软件触发（ADC_SOFTWARE_START），在 PWM 低电平期间手动触发，确保功率管关断时采样（避免开关噪声干扰）。

    减少中断开销：仅 hadc1 启用中断，hadc2 通过同一中断回调读取数据，避免双中断带来的同步问题和 CPU 负载。
    多 ADC 协同：通过双 ADC 同步采样，避免单 ADC 分时采样的相位差，提升三相电流测量的同步性和控制精度。
  */
  HAL_ADCEx_InjectedStart_IT(&hadc1); // 启动 ADC1 注入通道，使能中断
  HAL_ADCEx_InjectedStart(&hadc2);    // 启动 ADC2 注入通道，不使能中断（轮询或依赖 ADC1 中断）
  /*
  hadc1中断模式：
    转换完成后触发 HAL_ADCEx_InjectedConvCpltCallback 回调（见 adc.c 第 246 行），实时处理两路电流数据：
  hadc2中断模式：
    依赖 hadc1 中断回调中通过 HAL_ADCEx_InjectedGetValue(&hadc2, ...) 主动读取数据（见代码逻辑），而非独立中断，简化多 ADC 同步控制。
  */
  set_pwm_duty(0.5, 0, 0); // d轴强拖，形成SVPWM模型中的基础矢量1，即对应转子零度位置
  HAL_Delay(400);          // 保持一会
  rotor_zero_angle = encoder_angle;
  set_pwm_duty(0, 0, 0); // 松开电机
  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  motor_control_context.torque_norm_d = 0;
  motor_control_context.torque_norm_q = 0.4; // 百分比强度
  motor_control_context.type = control_type_torque;
  HAL_Delay(1000);
  // 速度模式
  /*
  本文选择的电机是云台电机，这里的位置环的PID系数只用到P系数和D系数，可以在主while(1)循环里打印电机实时角度辅助调节。
  调节P系数建议从2开始，0.1为步距往上调整，直到电机到位过程中出现轻微位置弹簧感
  然后设定D系数，建议从5开始，1为步距往上调，直到弹簧感消失，这样一个归位迅速而回弹轻微的位置环就调节好了
  */
  motor_control_context.position = deg2rad(90);
  motor_control_context.type = control_type_position;
  // 速度模式
  /*
  调参：
  速度环主要是使用P系数和I系数，速度波动比较剧烈，D系数干扰大。P系数要给的非常小，建议从0.01开始，步距0.001往上调节，I系数从0.001开始往上调节。
  经过简单的调节，选用的电机在空载时，在速度环模式下，速度大概能到每秒122.6弧度（每秒39转，每分钟1171转）左右。
  */
  // motor_control_context.speed = 30;       //每秒转30弧度
  // motor_control_context.type = control_type_speed;

  // 力矩环
  /*
  经过简单的调节，选用的电机在空载时，在力矩环模式下，空载速度大概能到每秒145.6弧度（每秒23转，每分钟1391转）左右。
  力矩环的PID参数调节方式通常单独调节d轴电流然后复制一份给q轴，或者在堵转情况下对q轴进行调节。
  */
  // motor_control_context.torque_norm_d = 0;
  // motor_control_context.torque_norm_q = 0.4;
  // motor_control_context.type = control_type_torque;
  while (1)
  {
    printf("%.3f\n", motor_logic_angle);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
