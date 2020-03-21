
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
#include "arm_etm.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;

osThreadId Task0Handle;
osThreadId Task1Handle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
CAN_FilterConfTypeDef  sFliterConfig;
CanTxMsgTypeDef TxMessage;
CanRxMsgTypeDef RxMessage;

uint8_t aRxBuffer1[1];
uint8_t aRxBuffer2[1];
uint8_t aRxBuffer3[1];

uint8_t aRxBuffer2_Data[4];
uint8_t aRxBuffer2_Data_flag = 0;
uint8_t aRxBuffer2_Data_Index = 0;

#define BUFFERSIZE 255                                
uint8_t ReceiveBuff[BUFFERSIZE];         //接收缓冲区
uint8_t recv_end_flag = 0,Rx_len;        //接收完成中断，接收到字符长度

int TickCounter = 0;

int globalCounter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void Func_Task0(void const * argument);
void Func_Task1(void const * argument);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void uartdamget(void);
/* USER CODE END PFP */
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
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  // 追踪初始化
  configure_tracing();
  configure_watchpoint();
  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart3,"DMA\r\n",5,10);
  /*        使能串口2 IDLE中断        */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
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

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART2)
  {
	HAL_UART_Transmit(&huart3,aRxBuffer2,1,100);	// 接收到数据马上使用串口3发送出去
    // aRxBuffer2_Data[aRxBuffer2_Data_Index] = aRxBuffer2[1];
    // aRxBuffer2_Data_Index++;
    // if (aRxBuffer2_Data_Index == 3)
    // { 
    //   aRxBuffer2_Data_flag = 1; 
    //   aRxBuffer2_Data_Index = 0;
    // }
	HAL_UART_Receive_IT(&huart2,aRxBuffer2,1);		// 重新使能串口2接收中断
  }
}
*/

void uartdamget(void)
{
  uint8_t temp[1];
  // uint8_t temp_before[1];
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  if(recv_end_flag ==1)
  {
        globalCounter = 0x0A;    // 只报告0x0A意味是ESP8266的返回数据

 	for(int i=0;i<Rx_len;i++)
	{
	  /*打印接收到的数据*/
	  sprintf(temp,"%c",ReceiveBuff[i]);
          HAL_UART_Transmit(&huart3,temp,1,10);

      // hcan.pTxMsg -> Data[0] = 0xEF;
	  // hcan.pTxMsg -> Data[1] = 0xFE;    // 替换CAN的发送数据，暂时用来测试
      // HAL_CAN_Transmit(&hcan, 10);      // 用CAN口把数据发出去
	}

    if (ReceiveBuff[Rx_len-1] == 'T' && ReceiveBuff[Rx_len - 2] == 'A' && ReceiveBuff[Rx_len - 3] == 'D' && ReceiveBuff[Rx_len - 4] == 'Y' && ReceiveBuff[Rx_len - 5] == 'M')   // 用最后一位向前数进行判断会好一些
    {
      globalCounter = 0x0B;    // 前面的0x0A报告了,这里的0x0B也报告,意味是客户端心跳包

      HAL_UART_Transmit(&huart3,"\r\nCorrect",9,10);
      hcan.pTxMsg -> ExtId = 0x1314;
      hcan.pTxMsg -> RTR = CAN_RTR_DATA;
      hcan.pTxMsg -> IDE = CAN_ID_EXT;
      hcan.pTxMsg -> DLC = 2;
      hcan.pTxMsg -> Data[0] = ReceiveBuff[Rx_len - 7] - 0x30;// ASC ii数字转制数字
      hcan.pTxMsg -> Data[1] = ReceiveBuff[Rx_len - 6] - 0x30;    // 替换CAN的发送数据，暂时用来测试
      HAL_CAN_Transmit(&hcan, 10);      // 用CAN口把数据发出去,还是得传两次那边才能收到
      // HAL_CAN_Transmit(&hcan, 10);
    }

    if (ReceiveBuff[Rx_len - 1] == 'P') // 9字符，8字符有效最后一字符为P,3个含有P的成一组，24字符有效数据
    {
      globalCounter = 0x0C;    // 前面的0x0A报告了,这里的0x0C也报告,意味是客户端数据包，以及CAN口开始发送1个P的数据
      HAL_UART_Transmit(&huart3,"\r\nSensorCorrect\r\n",17,10);
      hcan.pTxMsg -> ExtId = 0x1314;
      hcan.pTxMsg -> RTR = CAN_RTR_DATA;
      hcan.pTxMsg -> IDE = CAN_ID_EXT;
      hcan.pTxMsg -> DLC = 8;            // DLC是数据段位数0-8字节
      for (int k = 0;k < 8; k++)
      {
        hcan.pTxMsg -> Data[k] = ReceiveBuff[(Rx_len - 1) - (8 - k)];
      }
      HAL_CAN_Transmit(&hcan, 10);      // 用CAN口把数据发出去,还是得传两次那边才能收到
      // HAL_CAN_Transmit(&hcan, 10);
    }

	HAL_UART_Transmit(&huart3,"\r\n",2,10);         
	/*清空接收缓存区*/
    for(int i = 0; i < Rx_len ; i++)
	  ReceiveBuff[i]=0;
	/*接收数据长度清零*/
	Rx_len=0;
	recv_end_flag=0;
  }
  /*开启下一次接收*/
  HAL_UART_Receive_DMA(&huart2,(uint8_t*)ReceiveBuff,BUFFERSIZE);    // 注意这里一定要重新注册中断

  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
}
/* USER CODE END 4 */

/* Func_Task0 function */
void Func_Task0(void const * argument)
{

  /* USER CODE BEGIN 5 */
  hcan.pTxMsg = &TxMessage;    // hc.pTxMsg 按 TxMessage格式
  
  HAL_UART_Transmit(&huart1,"\r\nFunc_Task0\r\n",14,10);
  
  
  hcan.pTxMsg -> ExtId = 0x1314;
  hcan.pTxMsg -> RTR = CAN_RTR_DATA;
  hcan.pTxMsg -> IDE = CAN_ID_EXT;
  hcan.pTxMsg -> DLC = 2;
  hcan.pTxMsg -> Data[0] = 0xAB;
  hcan.pTxMsg -> Data[1] = 0xCD;

  // 为什么传2次那边CAN_rx才能收到
  HAL_CAN_Transmit(&hcan, 10);  // 注意第二个参数是timeout，与常规库函数不同
  // HAL_CAN_Transmit(&hcan, 10);
  /* Infinite loop */
  for(;;)
  {
    // osDelay(1000);
    // HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    // HAL_CAN_Transmit(&hcan, 10);

    // osDelay(1000);
    // HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    // HAL_CAN_Transmit(&hcan, 10);

    uartdamget();
  }
  /* USER CODE END 5 */ 
}

/* Func_Task1 function */
void Func_Task1(void const * argument)
{
  /* USER CODE BEGIN Func_Task1 */
  HAL_UART_Receive_IT(&huart2,aRxBuffer2,1);
  HAL_UART_Transmit(&huart3,"\r\nFunc_Task1\r\n",14,10);
  
  HAL_UART_Transmit(&huart2,"AT+CWMODE=2\r\n",13,10);
  osDelay(50);
  HAL_UART_Transmit(&huart2,"AT+RST\r\n",8,10);
  osDelay(3000);
  HAL_UART_Transmit(&huart2,"AT+CWSAP=\"PYY_8266\",\"12345678\",6,4\r\n",36,10);
  osDelay(50);
  HAL_UART_Transmit(&huart2,"AT+CIPMUX=1\r\n",13,10);
  osDelay(50);
  HAL_UART_Transmit(&huart2,"AT+CIPSERVER=1,333\r\n",20,10);
  osDelay(3000);
  HAL_UART_Transmit(&huart2,"AT+CIFSR\r\n",10,10);
  osDelay(50);
  
  HAL_UART_Transmit(&huart3,"\r\nFINISHED\r\n",12,10);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    // if (aRxBuffer2_Data_flag == 1)
    //   HAL_UART_Transmit(&huart3,aRxBuffer2_Data,4,10);
    // HAL_UART_Transmit(&huart3,"\r\nFunc_Task1_Loop\r\n",19,10);
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
