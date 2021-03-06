
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include <stdio.h>
#include "arm_etm.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId Task0Handle;
osThreadId Task1Handle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
CAN_FilterConfTypeDef  sFliterConfig;
CanTxMsgTypeDef TxMessage;
CanRxMsgTypeDef RxMessage;
int TickCounter = 0;

int sensor_flag1 = 0;
int sensor_flag2 = 0;
int sensor_flag3 = 0;


int flag;

int globalCounter = 0;

// int setupCounter = 0;

volatile int a = 0;
volatile int s = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
void Func_Task0(void const * argument);
void Func_Task1(void const * argument);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/*
void Setup_Icing(int Icing_value);    // 结冰只调PWM
void Setup_Visibility(int visibility_value);    // 能见度调PWM和LED
void Setup_Rainfall(int Rainfall_value);    // 降雨量只调PWM
void Setup_Light(int Light_value);    // 光照强度只调LED
void Setup_Flow_Speed(int Flow_value, int speed_value);    // 车流和车流速度只调PWM
*/

void PWM_100(void);
void PWM_70(void);
void PWM_40(void);
void PWM_10(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void configure_tracing()
{
    /* STM32 specific configuration to enable the TRACESWO IO pin */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    AFIO->MAPR |= (2 << 24); // Disable JTAG to release TRACESWO
    DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN; // Enable IO trace pins 
    
    uint32_t DBGMCU_val = DBGMCU->CR; 
    // DBGMCU->CR |= 0x00000020; 
   
    if (!(DBGMCU->CR & DBGMCU_CR_TRACE_IOEN))
    {
        // Some (all?) STM32s don't allow writes to DBGMCU register until
        // C_DEBUGEN in CoreDebug->DHCSR is set. This cannot be set by the
        // CPU itself, so in practice you need to connect to the CPU with
        // a debugger once before resetting it.
        return;
    }
    
    /* Configure Trace Port Interface Unit */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable access to registers
    TPI->ACPR = 8; // Trace clock = HCLK/(x+1) = 8MHz    这里HCLK是处理器时钟,这个值的初始值是0，0+1=1就是没有分频,还注释说是8MHz那是因为F105是8MHz
    TPI->SPPR = 2; // Pin protocol = NRZ/USART
    
    // TPI->FFCR = 0x102;
    TPI->FFCR = 0x100; //为什么设置102开启ETM后，ITM输出就不对了？？,在博客http://essentialscrap.com/tips/arm_trace/theory.html 中说到启用ETM会生成大量数据将使ITM追踪事件的数据丢失
    		       // TPIU packet framing enabled when bit 2 is set.
    		       // You can use 0x102 if you need both DWT/ITM and ETM.
                       // You can use 0x100 if you only need DWT/ITM and not ETM.
   
    /* Configure PC sampling and exception trace  */
    DWT->CTRL = (1 << DWT_CTRL_CYCTAP_Pos) // Prescaler for PC sampling
                                           // 0 = x32, 1 = x512  //
                    //bits[6]就是DWT_CYCCNT寄存器的0到5bit，那么节拍就是32记录一次PC,bits[10]就是0到9bit，那么节拍记录就是512记录一次PC
              | (0 << DWT_CTRL_POSTPRESET_Pos) // Postscaler for PC sampling
                                                // Divider = value + 1
              | (1 << DWT_CTRL_PCSAMPLENA_Pos) // Enable PC sampling
              | (2 << DWT_CTRL_SYNCTAP_Pos)    // Sync packet interval
                                               // 0 = Off, 1 = Every 2^23 cycles,
                                               // 2 = Every 2^25, 3 = Every 2^27
              | (1 << DWT_CTRL_EXCTRCENA_Pos)  // Enable exception trace
              | (1 << DWT_CTRL_CYCCNTENA_Pos); // Enable cycle counter
    
    /* Configure instrumentation trace macroblock */
    ITM->LAR = 0xC5ACCE55;
    ITM->TCR = (1 << ITM_TCR_TraceBusID_Pos)  // Trace bus ID for TPIU
             | (1 << ITM_TCR_DWTENA_Pos)      // Enable events from DWT
             | (1 << ITM_TCR_SYNCENA_Pos)     // Enable sync packets
             | (1 << ITM_TCR_ITMENA_Pos)      // Main enable for ITM
             | (1 << ITM_TCR_TSENA_Pos)       // 使能ITM时间戳
             | (0 << ITM_TCR_TSPrescale_Pos); // 先不设置时间戳分频系数,00:不分频，01:4分频，10:16分频，11:64分频
    ITM->TER = 0xFFFFFFFF; // Enable all stimulus ports


    //ETM_Lock_Access = 0xC5ACCE55;
    //ETM_Control = 0x00001D1E;
    //ETM_Trigger_Event = 0x0000406F;
    //ETM_Trace_Enable_Event = 0x0000006F;
    //ETM_Trace_Start_Stop = 0x00000001;


   /* Configure embedded trace macroblock */
    ETM->LAR = 0xC5ACCE55;
    ETM_SetupMode();
    ETM->CR = ETM_CR_ETMEN // Enable ETM output port
            | ETM_CR_STALL_PROCESSOR // Stall processor when fifo is full
            | ETM_CR_BRANCH_OUTPUT // Report all branches
            | (1 << 4);//port_size位21,6,5,4是0001表示8bit，这里目前认为复位值都是0,那么把第4位改为1就行
    ETM->TRACEIDR = 2; // Trace bus ID for TPIU
    ETM->TECR1 = ETM_TECR1_EXCLUDE; // Trace always enabled
    ETM->FFRR = ETM_FFRR_EXCLUDE; // Stalling always enabled
    ETM->FFLR = 24; // Stall when less than N bytes free in FIFO (range 1..24)
                    // Larger values mean less latency in trace, but more stalls.
    // Note: we do not enable ETM trace yet, only for specific parts of code.
    
    ETM->TRIGGER = 0x0000406F;
    ETM->TEEVR = 0x0000006F;
    //ETM->TSSCR = 0x00000001;

}

void configure_watchpoint()
{
    /* This is an example of how to configure DWT to monitor a watchpoint.
       The data value is reported when the watchpoint is hit. */
    
    /* Monitor all accesses to GPIOC (range length 32 bytes) */
    //DWT->COMP0 = (uint32_t)bubble_sort;                      //改为了比GPIOC
    DWT->COMP0 = (uint32_t)C_GPIO_GPIO_Port;                      //改为了比较GPIOC
    DWT->MASK0 = 8;							 //屏蔽掉数据地址的后5位，目前DWT->COMP0的值是GPIOA的地址:0x40010800
											//可能是出于加快比较速度的原因吧，那为什么不把MASK[3:0]
										        //设置为8,反正0x40010800最后八位都是0
										        
    DWT->FUNCTION0 = (2 << DWT_FUNCTION_FUNCTION_Pos)   // Report data and addr on watchpoint hit
                   | (1 << DWT_FUNCTION_EMITRANGE_Pos); // 这一位为1，前面那四位是0010表示在读和写操作时通过ITM发出数据和地址偏移量
                   
                 //    (4 << DWT_FUNCTION_FUNCTION_Pos)
                 //  | (1 << DWT_FUNCTION_CYCMATCH_Pos); // 这一位为1,前面那四位是0100,表示比较器0与PC采样计数器比较，匹配时打watchpoint
    
    /* Monitor all accesses to globalCounter (range length 4 bytes) */
    DWT->COMP1 = (uint32_t)&globalCounter;
    DWT->MASK1 = 2;
    DWT->FUNCTION1 = (3 << DWT_FUNCTION_FUNCTION_Pos); // Report data and PC on watchpoint hit
}

void ITM_Print(int port, const char *p)
{
    globalCounter = 0xFC;
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << port)))
    {
        while (*p)
        {
            while (ITM->PORT[port].u32 == 0);//时间片下多进程使用一个查询函数在这里始终过不去了，这个问题需要引起注意,原因是两个进程在抢这个发送u端口
            ITM->PORT[port].u8 = *p++;
        }   
    }
}

void ITM_SendValue (int port, uint32_t value)
{
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << port)))
    {
        while (ITM->PORT[port].u32 == 0);
        ITM->PORT[port].u32 = value;
    }
}

int _write(int fd, char *ptr, int len)  
{  
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 0xFFFF);
  return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();

  // 追踪初始化
  configure_tracing();
  configure_watchpoint();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of Task0 */
  osThreadDef(Task0, Func_Task0, osPriorityNormal, 0, 256);
  Task0Handle = osThreadCreate(osThread(Task0), NULL);

  /* definition and creation of Task1 */
  osThreadDef(Task1, Func_Task1, osPriorityNormal, 0, 256);
  Task1Handle = osThreadCreate(osThread(Task1), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_7TQ;
  hcan.Init.BS2 = CAN_BS2_8TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(C_GPIO_GPIO_Port, C_GPIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : C_GPIO_Pin */
  GPIO_InitStruct.Pin = C_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(C_GPIO_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*
void Setup_Icing(int Icing_value)
{
  setupCounter = 0xAA;
  uint16_t dutyCycle = 0;
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  if (Icing_value > 5)
  {
    dutyCycle = 0;
    while (dutyCycle < 1000)    // 逐渐降低动力输出，使低电平比例到950/1000
    {
      dutyCycle += 50;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dutyCycle);
      osDelay(5);
    }

    while (dutyCycle > 200)    // 逐渐升高动力输出，使低电平比例到150/1000
    {
      dutyCycle -= 50;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dutyCycle);
      osDelay(5);
    }
  }

  if (Icing_value <= 5 && Icing_value > 2)
  {
    dutyCycle = 0;
    while (dutyCycle < 700)    // 逐渐降低动力输出，使低电平比例到650/1000
    {
      dutyCycle += 50;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dutyCycle);
      osDelay(5);
    }

    while (dutyCycle > 200)    // 逐渐升高动力输出，使低电平比例到150/1000
    {
      dutyCycle -= 50;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dutyCycle);
      osDelay(5);
    }
  }
}

void Setup_Visibility(int visibility_value)
{
  setupCounter = 0xAB;
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    
  uint16_t dutyCycle = 0;
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  if (visibility_value < 5)    // 能见度小于5公里
  {
    dutyCycle = 0;
    while (dutyCycle < 600)    // 逐渐降低动力输出，使低电平比例到550/1000
    {
      dutyCycle += 50;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dutyCycle);
      osDelay(5);
    }

    while (dutyCycle > 400)    // 逐渐升高动力输出，使低电平比例到350/1000
    {
      dutyCycle -= 50;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dutyCycle);
      osDelay(5);
    }
  }

  if (visibility_value < 7)    // 能见度小于7公里
  {
    dutyCycle = 0;
    while (dutyCycle < 700)    // 逐渐降低动力输出，使低电平比例到650/1000
    {
      dutyCycle += 50;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dutyCycle);
      osDelay(5);
    }
  }
}

void Setup_Rainfall(int Rainfall_value)
{
  setupCounter = 0xAC;
  uint16_t dutyCycle = 0;
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  if (Rainfall_value > 200)     // 降雨量大于200
  {
    dutyCycle = 0;
    while (dutyCycle < 900)    // 逐渐降低动力输出，使低电平比例到850/1000
    {
      dutyCycle += 50;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dutyCycle);
      osDelay(5);
    }

    while (dutyCycle > 200)    // 逐渐升高动力输出，使低电平比例到150/1000
    {
      dutyCycle -= 50;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dutyCycle);
      osDelay(5);
    }
  }

  if (Rainfall_value <= 200 && Rainfall_value > 100)    // 降雨量在100-200间
  {
    dutyCycle = 0;
    while (dutyCycle < 700)    // 逐渐降低动力输出，使低电平比例到650/1000
    {
      dutyCycle += 50;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dutyCycle);
      osDelay(5);
    }

    while (dutyCycle > 200)    // 逐渐升高动力输出，使低电平比例到150/1000
    {
      dutyCycle -= 50;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dutyCycle);
      osDelay(5);
    }
  }
}

void Setup_Light(int Light_value)
{
  setupCounter = 0xAD;
  if (Light_value < 200)
  {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
  }
  if (Light_value < 300 && Light_value >= 200)
  {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
  }
  if (Light_value < 400 && Light_value >= 300)
  {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
  }
  if (Light_value >= 400)
  {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
  }
}

void Setup_Flow_Speed(int Flow_value, int speed_value)
{
  setupCounter = 0xAE;
  uint16_t dutyCycle = 0;
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  if (Flow_value > 30 && speed_value < 30)     // 车流量大于30，车流速度小于30
  {
    dutyCycle = 0;
    while (dutyCycle < 600)    // 逐渐降低动力输出，使低电平比例到550/1000
    {
      dutyCycle += 50;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dutyCycle);
      osDelay(5);
    }
  }

  if (Flow_value > 45 && speed_value < 20)    // 车流量大于45，车流速度小于20
  {
    dutyCycle = 0;
    while (dutyCycle < 800)    // 逐渐降低动力输出，使低电平比例到750/1000
    {
      dutyCycle += 50;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dutyCycle);
      osDelay(5);
    }
  }
}
*/

void PWM_100(void)
{
  globalCounter = 0xDA;
  uint16_t dutyCycle = 0;
  dutyCycle = 999;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dutyCycle);
  // printf("\r\nPWM_100");
  osDelay(250);
}

void PWM_70(void)
{
 globalCounter = 0xDB;
  uint16_t dutyCycle = 0;
  dutyCycle = 699;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dutyCycle);
  printf("\r\nPWM_70");
  osDelay(250);
}

void PWM_40(void)
{
  globalCounter = 0xDC;
  uint16_t dutyCycle = 0;
  dutyCycle = 399;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dutyCycle);
  printf("\r\nPWM_40");
  osDelay(250);
}

void PWM_10(void)
{
  globalCounter = 0xDD;
  uint16_t dutyCycle = 0;
  dutyCycle = 99;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dutyCycle);
  printf("\r\nPWM_10");
  osDelay(250);
}

/* USER CODE END 4 */

/* Func_Task0 function */
void Func_Task0(void const * argument)
{

  /* USER CODE BEGIN 5 */
    // hcan.pTxMsg = &TxMessage;    // hc.pTxMsg 按 TxMessage格式
    hcan.pRxMsg = &RxMessage;

    HAL_UART_Transmit(&huart2,"\r\nFunc_Task0\r\n",14,10);

    sFliterConfig.FilterNumber         = 0;
    sFliterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFliterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;            // 查看了下.h文件，最大滤波器位宽就是32位，8个字符
    sFliterConfig.FilterIdHigh         = 0x0000;
    sFliterConfig.FilterIdLow          = 0x0000;
    sFliterConfig.FilterMaskIdHigh     = 0x0000;
    sFliterConfig.FilterMaskIdLow      = 0x0000;
    sFliterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFliterConfig.FilterActivation     = ENABLE;
    // sFliterConfig.BankNumber           = 14;

    HAL_CAN_ConfigFilter(&hcan, &sFliterConfig);

    HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 999);

    // hcan.pTxMsg -> ExtId = 0x1314;
    // hcan.pTxMsg -> RTR = CAN_RTR_DATA;
    // hcan.pTxMsg -> IDE = CAN_ID_EXT;
    // hcan.pTxMsg -> DLC = 2;
    // hcan.pTxMsg -> Data[0] = 0xAB;
    // hcan.pTxMsg -> Data[1] = 0xCD;

    // HAL_CAN_Transmit(&hcan, 10);  // 注意第二个参数是timeout，与常规库函数不同
    /* Infinite loop */
    for(;;)
    {
        // osDelay(1000);
        // HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        // HAL_CAN_Transmit(&hcan, 10);

        osDelay(1);
        /*
        if (flag == 0)
        {
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 999);    // 在没有传感器数据到来时保证动力充足
        }
        */
 
        if (a == 0 && s == 0)
        {
          PWM_100();
        }
        else if (a == 0 && s == 1)
        {
          PWM_10();
        }
        else if (a == 1 && s == 1)
        {
          PWM_40();
        }
        else if (a == 1 && s == 0)
        {
          PWM_70();    
        }

        // HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        // HAL_CAN_Transmit(&hcan, 10);
    }
  /* USER CODE END 5 */ 
}

/* Func_Task1 function */
void Func_Task1(void const * argument)
{
  /* USER CODE BEGIN Func_Task1 */
    /* Infinite loop */
    flag = 0;

    char str1[8];
    char str2[8];

    char user_str[24];
    char uart_str[8];
    
    int temperature, humidity, Light, visibility, Rainfall, Flow, speed,Icing;

    HAL_UART_Transmit(&huart2,"CAN_RX", 6, 10);

    for(;;)
    {
      // osDelay(1);
      if (flag == 1)
	  {
		globalCounter = 0x0C;    // 报告0x0C意味CAN口接收到了1个P的数据
		flag = 0;
		HAL_UART_Transmit(&huart2, "\r\nOK\r\n", 6, 10);
		
		if ((RxMessage.Data[0] == 9 && RxMessage.Data[1] == 8) || (RxMessage.Data[0] == 1 && RxMessage.Data[1] == 2))
		{
		  sprintf(str1, "%d", RxMessage.Data[0]);
		  sprintf(str2, "%d", RxMessage.Data[1]);
		  // if (RxMessage.Data[0] = 0xAB)
		  //     HAL_UART_Transmit(&huart1, "AB", 2, 10);
		  HAL_UART_Transmit(&huart2, str1, 8, 10);
		  HAL_UART_Transmit(&huart2, str2, 8, 10);
		}
		else
		{
		  HAL_UART_Transmit(&huart2,"\r\nRX SensorOK\r\n", 15, 10);
		  for (int p = 0;p < 8;p++)
		  {
		    sprintf(str1, "%c", RxMessage.Data[p]);
		    HAL_UART_Transmit(&huart2, str1, 8, 10);
		  }

		  if (RxMessage.Data[2] == 'T')        // 18T30H05P
		  {
		    HAL_UART_Transmit(&huart2,"\r\n1P\r\n", 6, 10);
		    for (int k = 0;k < 8;k++)
		    {
		      user_str[k] = RxMessage.Data[k];
		    }
		    sensor_flag1 = 8;
		  }

		  if (RxMessage.Data[2] == 'L' && sensor_flag1 == 8)        // 00L15V00P
		  {
		    HAL_UART_Transmit(&huart2,"\r\n2P\r\n", 6, 10);
		    for (int k = 0;k < 8;k++)
		    {
		      user_str[k + 8] = RxMessage.Data[k];
		    }
		    sensor_flag2 = 16;
		  }

		  if (RxMessage.Data[1] == 'R' && sensor_flag2 == 16)        // 0R30F45SP
		  {
		    HAL_UART_Transmit(&huart2,"\r\n3P\r\n", 6, 10);
		    for (int k = 0;k < 8;k++)
		    {
		      user_str[k + 8 + 8] = RxMessage.Data[k];
		    }
		    sensor_flag3 = 24;

            temperature = (user_str[0] - 0x30) * 10 + (user_str[1] - 0x30);
            printf("\r\ntemperature:%d",temperature);
  		    

	    	if (user_str[5] == 'H')
            {
              humidity = (user_str[3] - 0x30) * 10 + (user_str[4] - 0x30);
              printf("\r\nhumidity:",humidity);
            }
            if (user_str[5] == 'I')
            {  
              Icing = (user_str[3] - 0x30) * 10 + (user_str[4] - 0x30);
              // Setup_Icing(Icing);
              printf("\r\nIcing:%d",Icing);
            }


	    	Light = (user_str[6] - 0x30) * 1000 + (user_str[7] - 0x30) * 100 + (user_str[8] - 0x30) * 10 + + (user_str[9] - 0x30);
		    // Setup_Light(Light);
            printf("\r\nLight:%d",Light);


	    	visibility = (user_str[11] - 0x30) * 10 + (user_str[12] - 0x30);
            // if (visibility < 10)
            // {  
              // HAL_UART_Transmit(&huart2,"\r\nFAT",5,10);    // 在计算没有错误的情况下，HAL_UART_Transmit出现显示错误,修改_write重定向printf则无措，不用管，反正那么多串口发送都得关闭
            //   printf("\r\nFAT");
            // }
		    // Setup_Visibility(visibility);

            printf("\r\nvisibility:%d",visibility);


		    Rainfall = (user_str[14] - 0x30) * 100 + (user_str[15] - 0x30) * 10 + + (user_str[16] - 0x30);
		    // Setup_Rainfall(Rainfall);    // 降雨量只调PWM
            printf("\r\nRainfall:%d",Rainfall);
	    	

	    	Flow = (user_str[18] - 0x30) * 10 + (user_str[19] - 0x30);
            printf("\r\nFlow:%d",Flow);
	    	

	    	speed = (user_str[21] - 0x30) * 10 + (user_str[22] - 0x30);
            printf("\r\nspeed:%d",speed);
		    // Setup_Flow_Speed(Flow, speed);
		  }

		}

        if (Rainfall > 150 || Icing > 5)
        {
          a = 0;
          s = 1;
        }
        else if (visibility < 10 && visibility > 0)
        {
          a = 1;
          s = 1;
        }
        else if (Flow > 45 && speed < 25 && speed > 0)
        {
          a = 1;
          s = 0;
        }
        else
        {
          a = 0;
          s = 0;
        }
	  }



        HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);    // 在USB_LP_CAN1_RX0_IRQHandler里用HAL_CAN_IRQHandler挂起hcan后，一定记得这里再注册一下，不然下次就进不去中断了


    }
  /* USER CODE END Func_Task1 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();

    ++TickCounter;
    if (TickCounter == 50)    // C_GPIO是PB14
      HAL_GPIO_WritePin(C_GPIO_GPIO_Port, C_GPIO_Pin, GPIO_PIN_RESET);  // set 0
    
    if (TickCounter == 100)
    {  
      HAL_GPIO_WritePin(C_GPIO_GPIO_Port, C_GPIO_Pin, GPIO_PIN_SET);    // set 1
      TickCounter = 0;
    }
  }

  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
