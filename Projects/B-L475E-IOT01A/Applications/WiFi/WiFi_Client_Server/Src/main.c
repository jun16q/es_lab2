/**
  ******************************************************************************
  * @file    Wifi/WiFi_Client_Server/src/main.c
  * @author  MCD Application Team
  * @brief   This file provides main program functions with motion detection
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l475e_iot01_accelero.h"

/* Private defines -----------------------------------------------------------*/
#define TERMINAL_USE

/* Update SSID and PASSWORD with own Access point settings */
#define SSID     "cch0610"
#define PASSWORD "86603863"

/* mobile phone */
//#define SSID     "jun"
//#define PASSWORD "00000000"

uint8_t RemoteIP[] = {192,168,0,140};
#define RemotePORT	5000

#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  10000
#define CONNECTION_TRIAL_MAX 10

/* Motion detection GPIO configuration - LSM6DSL INT1 pin */
#define MOTION_INT_PIN        GPIO_PIN_11
#define MOTION_INT_GPIO_PORT  GPIOD
#define MOTION_INT_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()
#define MOTION_INT_EXTI_IRQn  EXTI15_10_IRQn

/* Test with user button for debugging */
#define TEST_BUTTON_PIN       GPIO_PIN_13
#define TEST_BUTTON_PORT      GPIOC
#define TEST_BUTTON_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define TEST_BUTTON_EXTI_IRQn EXTI15_10_IRQn

#if defined (TERMINAL_USE)
#define TERMOUT(...)  printf(__VA_ARGS__)
#else
#define TERMOUT(...)
#endif

/* Private variables ---------------------------------------------------------*/
#if defined (TERMINAL_USE)
extern UART_HandleTypeDef hDiscoUart;
#endif /* TERMINAL_USE */
static uint8_t RxData [500];

/* Motion detection flag */
volatile uint8_t motion_detected = 0;
static uint32_t motion_count = 0;

/* Private function prototypes -----------------------------------------------*/
#if defined (TERMINAL_USE)
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#endif /* TERMINAL_USE */

static void SystemClock_Config(void);
static void Motion_INT_Init(void);

extern SPI_HandleTypeDef hspi;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize Motion Detection GPIO and EXTI (with test button)
  * @param  None
  * @retval None
  */
static void Motion_INT_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable GPIO clocks */
  MOTION_INT_GPIO_CLK_ENABLE();
  TEST_BUTTON_CLK_ENABLE();

  /* Configure GPIO pin for motion interrupt */
  GPIO_InitStruct.Pin = MOTION_INT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MOTION_INT_GPIO_PORT, &GPIO_InitStruct);

  /* Configure test button (User Button) for debugging */
  GPIO_InitStruct.Pin = TEST_BUTTON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // Button press (falling edge)
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TEST_BUTTON_PORT, &GPIO_InitStruct);

  /* Enable EXTI interrupts */
  HAL_NVIC_SetPriority(MOTION_INT_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MOTION_INT_EXTI_IRQn);
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  uint8_t  MAC_Addr[6] = {0};
  uint8_t  IP_Addr[4] = {0};
  static uint8_t TxData[200];
  int32_t Socket = -1;
  uint16_t Datalen;
  int32_t ret;
  int16_t Trials = CONNECTION_TRIAL_MAX;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure LED2 */
  BSP_LED_Init(LED2);

#if defined (TERMINAL_USE)
  /* Initialize UART for terminal output */
  hDiscoUart.Instance = DISCOVERY_COM1;
  hDiscoUart.Init.BaudRate = 9600;
  hDiscoUart.Init.WordLength = UART_WORDLENGTH_8B;
  hDiscoUart.Init.StopBits = UART_STOPBITS_1;
  hDiscoUart.Init.Parity = UART_PARITY_NONE;
  hDiscoUart.Init.Mode = UART_MODE_TX_RX;
  hDiscoUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hDiscoUart.Init.OverSampling = UART_OVERSAMPLING_16;
  hDiscoUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hDiscoUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  BSP_COM_Init(COM1, &hDiscoUart);
#endif /* TERMINAL_USE */

  TERMOUT("****** WIFI Module with Motion Detection Demo ****** \r\n\n");
  TERMOUT("Motion Detection Instructions :\r\n");
  TERMOUT("1- The LSM6DSL accelerometer will detect significant motion\r\n");
  TERMOUT("2- Motion events will be sent to the TCP server\r\n");
  TERMOUT("3- Regular accelerometer data is also transmitted\r\n\n");

  /* Initialize accelerometer */
  if(BSP_ACCELERO_Init() == ACCELERO_OK)
  {
    TERMOUT("> Accelerometer initialized successfully\r\n");

    /* Initialize motion detection GPIO */
    Motion_INT_Init();

    /* Enable significant motion detection interrupt */
    if(BSP_ACCELERO_Enable_Motion_Detection_IT() == ACCELERO_OK)
    {
      TERMOUT("> Motion detection enabled successfully\r\n");
    }
    else
    {
      TERMOUT("> ERROR: Failed to enable motion detection\r\n");
    }
  }
  else
  {
    TERMOUT("> ERROR: Failed to initialize accelerometer\r\n");
  }

  /* Initialize WIFI module */
  if(WIFI_Init() == WIFI_STATUS_OK)
  {
    TERMOUT("> WIFI Module Initialized.\r\n");
    if(WIFI_GetMAC_Address(MAC_Addr, sizeof(MAC_Addr)) == WIFI_STATUS_OK)
    {
      TERMOUT("> es-wifi module MAC Address : %X:%X:%X:%X:%X:%X\r\n",
               MAC_Addr[0], MAC_Addr[1], MAC_Addr[2],
               MAC_Addr[3], MAC_Addr[4], MAC_Addr[5]);
    }
    else
    {
      TERMOUT("> ERROR : CANNOT get MAC address\r\n");
      BSP_LED_On(LED2);
    }

    if(WIFI_Connect(SSID, PASSWORD, WIFI_ECN_WPA2_PSK) == WIFI_STATUS_OK)
    {
      TERMOUT("> es-wifi module connected \r\n");
      if(WIFI_GetIP_Address(IP_Addr, sizeof(IP_Addr)) == WIFI_STATUS_OK)
      {
        TERMOUT("> es-wifi module got IP Address : %d.%d.%d.%d\r\n",
               IP_Addr[0], IP_Addr[1], IP_Addr[2], IP_Addr[3]);

        TERMOUT("> Trying to connect to Server: %d.%d.%d.%d:%d ...\r\n",
               RemoteIP[0], RemoteIP[1], RemoteIP[2], RemoteIP[3], RemotePORT);

        while (Trials--)
        {
          if(WIFI_OpenClientConnection(0, WIFI_TCP_PROTOCOL, "TCP_CLIENT", RemoteIP, RemotePORT, 0) == WIFI_STATUS_OK)
          {
            TERMOUT("> TCP Connection opened successfully.\r\n");
            Socket = 0;
            break;
          }
        }
        if(Socket == -1)
        {
          TERMOUT("> ERROR : Cannot open Connection\r\n");
          BSP_LED_On(LED2);
        }
      }
      else
      {
        TERMOUT("> ERROR : es-wifi module CANNOT get IP address\r\n");
        BSP_LED_On(LED2);
      }
    }
    else
    {
      TERMOUT("> ERROR : es-wifi module NOT connected\r\n");
      BSP_LED_On(LED2);
    }
  }
  else
  {
    TERMOUT("> ERROR : WIFI Module cannot be initialized.\r\n");
    BSP_LED_On(LED2);
  }

  int16_t pDataXYZ[3] = {0};
  uint32_t last_motion_time = 0;

  while(1)
  {
    if(Socket != -1)
    {
      /* Check if motion was detected */
      if(motion_detected)
      {
        motion_detected = 0; /* Clear the flag */
        motion_count++;
        last_motion_time = HAL_GetTick();

        /* Send motion detection event */
        snprintf((char*)TxData, sizeof(TxData),
                 "MOTION_DETECTED,Count=%lu,Time=%lu\r\n",
                 motion_count, last_motion_time);

        TERMOUT(">> Significant Motion Detected! Count: %lu\r\n", motion_count);

        /* Blink LED to indicate motion detection */
        BSP_LED_On(LED2);
        HAL_Delay(100);
        BSP_LED_Off(LED2);

        ret = WIFI_SendData(Socket, TxData, strlen((char*)TxData), &Datalen, WIFI_WRITE_TIMEOUT);
        if (ret != WIFI_STATUS_OK)
        {
          TERMOUT("> ERROR : Failed to Send Motion Data, connection closed\r\n");
          break;
        }
      }

      /* Regular accelerometer reading and transmission */
      BSP_ACCELERO_AccGetXYZ(pDataXYZ);

      /* Send regular data with motion status */
      snprintf((char*)TxData, sizeof(TxData),
               "ACCEL,%d,%d,%d,MotionCount=%lu\r\n",
               pDataXYZ[0], pDataXYZ[1], pDataXYZ[2], motion_count);

      ret = WIFI_SendData(Socket, TxData, strlen((char*)TxData), &Datalen, WIFI_WRITE_TIMEOUT);
      if (ret != WIFI_STATUS_OK)
      {
        TERMOUT("> ERROR : Failed to Send Data, connection closed\r\n");
        break;
      }

      /* Print to terminal every 20 iterations (reduce spam) */
      static uint8_t print_counter = 0;
      if(++print_counter >= 20)
      {
        print_counter = 0;
        printf("Accelerometer: X=%d, Y=%d, Z=%d, Motion Count=%lu\r\n",
               pDataXYZ[0], pDataXYZ[1], pDataXYZ[2], motion_count);
      }

      HAL_Delay(100); /* Increased delay for better readability */
    }
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}

#if defined (TERMINAL_USE)
/**
  * @brief  Retargets the C library TERMOUT function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&hDiscoUart, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
#endif /* TERMINAL_USE */

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case (GPIO_PIN_1):
    {
      SPI_WIFI_ISR();
      break;
    }
    case (MOTION_INT_PIN): /* Motion detection interrupt */
    {
      motion_detected = 1; /* Set motion detection flag */
      TERMOUT("Motion interrupt triggered!\r\n");
      break;
    }
    case (TEST_BUTTON_PIN): /* Test button interrupt for debugging */
    {
      motion_detected = 1; /* Set motion detection flag */
      TERMOUT("Test button pressed - simulating motion!\r\n");
      break;
    }
    default:
    {
      break;
    }
  }
}

/**
  * @brief  EXTI15_10_IRQHandler for motion detection and test button
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
  /* Check if it's the motion detection pin interrupt */
  if(__HAL_GPIO_EXTI_GET_IT(MOTION_INT_PIN) != RESET)
  {
    HAL_GPIO_EXTI_IRQHandler(MOTION_INT_PIN);
  }

  /* Check if it's the test button interrupt */
  if(__HAL_GPIO_EXTI_GET_IT(TEST_BUTTON_PIN) != RESET)
  {
    HAL_GPIO_EXTI_IRQHandler(TEST_BUTTON_PIN);
  }
}

void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
}

#ifdef USE_FULL_ASSERT
/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: TERMOUT("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif
