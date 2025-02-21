/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "gpio.h"

#include <cmath>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define QUEUE_COUNT 10

osMessageQueueId_t setSpd_MsgQueue;

osThreadId UART_Thread0Handle;
osThreadId PWM_Thread1Handle;
osThreadId PWMRead_THread2Handle;

osMutexId_t mutex_ReadSpeed;
osMutexId_t mutex_pid;

double MeasuredSpeed = 0;

double pValue = 1;
double iValue = 0.950008;
double dValue = 0;

const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask0",
    .attr_bits = osThreadDetached,
    .cb_mem = NULL,
    .cb_size = 0,
    .stack_mem = NULL,
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
    .tz_module = 0,
    .reserved = 0};

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */




void InitServoPin()
{
//PWM output op pin PA6 wordt ingesteld met timer 3 en een overflow waarde van 10000 waardoor hij elke 100ms een signaal stuurt met de huidige prescale.
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  TIM3->PSC = 71; //define prescale

  TIM3->ARR = 20000-1; //define overflow value

  TIM3->CCMR1 = (TIM2->CCMR1 & ~TIM_CCMR1_OC1M) | (0b0110 << TIM_CCMR1_OC1M_Pos);

  TIM3->CCR1 = 1500; //pwm write value

  TIM3->CCER |= TIM_CCER_CC1E;

  TIM3->CR1 |= 1;

  GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER6) | (0b10 << GPIO_MODER_MODER6_Pos);

  GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFRL6) | (0b0010 << GPIO_AFRL_AFRL6_Pos);
}

void InitEchoPin()
{
//PWM output op pin PA0 wordt ingesteld met timer 2 hij kijkt op de microseconde of de pulse breedte is veranderd we gebruiken timer 2 omdat die ook accurater is.
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  //stap 1

  TIM2->PSC = 71; //stap 2

  TIM2->CCMR1 = (TIM2->CCMR1 & ~TIM_CCMR1_CC1S) | (0b01 << TIM_CCMR1_CC1S_Pos); //stap 3

  TIM2->CCER &= ~TIM_CCER_CC1NP;
  TIM2->CCER &= ~TIM_CCER_CC1P; //stap 4

  TIM2->CCER |= TIM_CCER_CC1E; //stap 5

  TIM2->CCMR1 |= (0b10 << TIM_CCMR1_CC2S_Pos); //stap 6

  TIM2->CCER &= ~TIM_CCER_CC2NP;
  TIM2->CCER |= TIM_CCER_CC2P; //stap 7

  TIM2->CCER |= TIM_CCER_CC2E; //stap 8

  TIM2->SMCR |= (0b0101 << TIM_SMCR_TS_Pos); //stap 9

  TIM2->SMCR |= 0b0100; //stap 10

  TIM2->CR1 |= 1; //stap 11

  GPIOA->MODER |= (0b10 << GPIO_MODER_MODER0_Pos);
  GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFRL0) | (0b0001 << GPIO_AFRL_AFRL0_Pos); //stap 12
}

void ChangeAndPrintDesiredSpeed(char* msgBuf, size_t MSGBUFSIZE, double receivedDouble)
{
  osMessageQueuePut(setSpd_MsgQueue, &receivedDouble, 0U, 0U);

  snprintf(msgBuf, MSGBUFSIZE, "%s", "\n Set rotation speed to: ");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%f", receivedDouble);
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%s", " rps\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);
}

void ShowHelpScreen(char* msgBuf, size_t MSGBUFSIZE)
{
  snprintf(msgBuf, MSGBUFSIZE, "%s", "\n--Help screen--\n\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%s", "- s[rotation speed] sets rotation speed in RPS.\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%s", "- h shows help screen.\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%s", "- g shows all current values in the system.\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%s", "- p[pid value] sets p value for PID.\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%s", "- i[pid value] sets i value for PID.\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%s", "- d[pid value] sets d value for PID.\n\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);
}

void ShowCurrentValues(char* msgBuf, size_t MSGBUFSIZE)
{
  snprintf(msgBuf, MSGBUFSIZE, "%s", "\nCurrentSpeed: ");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  osMutexAcquire(mutex_ReadSpeed, osWaitForever); // try to acquire mutex
  snprintf(msgBuf, MSGBUFSIZE, "%f", MeasuredSpeed);
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);
  osMutexRelease(mutex_ReadSpeed);

  snprintf(msgBuf, MSGBUFSIZE, "%s", " RPS\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%s", "P value: ");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  osMutexAcquire(mutex_pid, osWaitForever); // try to acquire mutex
  snprintf(msgBuf, MSGBUFSIZE, "%f", pValue);
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%s", " \n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%s", "I value: ");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%f", iValue);
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%s", " \n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%s", "D value: ");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%f", dValue);
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);
  osMutexRelease(mutex_pid);

  snprintf(msgBuf, MSGBUFSIZE, "%s", " \n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);
}

void ChangeAndPrintPValue(char* msgBuf, size_t MSGBUFSIZE, double receivedDouble)
{
  osMutexAcquire(mutex_pid, osWaitForever); // try to acquire mutex
  pValue = receivedDouble;
  osMutexRelease(mutex_pid);

  snprintf(msgBuf, MSGBUFSIZE, "%s", "\n Set PID P value to: ");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%f", receivedDouble);
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%s", "\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);
}

void ChangeAndPrintIValue(char* msgBuf, size_t MSGBUFSIZE, double receivedDouble)
{
  osMutexAcquire(mutex_pid, osWaitForever); // try to acquire mutex
  iValue = receivedDouble;
  osMutexRelease(mutex_pid);

  snprintf(msgBuf, MSGBUFSIZE, "%s", "\n Set PID I value to: ");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%f", receivedDouble);
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%s", "\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);
}

void ChangeAndPrintDValue(char* msgBuf, size_t MSGBUFSIZE, double receivedDouble)
{
  osMutexAcquire(mutex_pid, osWaitForever); // try to acquire mutex
  dValue = receivedDouble;
  osMutexRelease(mutex_pid);

  snprintf(msgBuf, MSGBUFSIZE, "%s", "\n Set PID D value to: ");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%f", receivedDouble);
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  snprintf(msgBuf, MSGBUFSIZE, "%s", "\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);
}


void UARTHandling(void* argument)
{
  InitServoPin();
  const size_t MSGBUFSIZE = 80;
  char msgBuf[MSGBUFSIZE];
  
  const int inputBufferSize = 10;
  uint8_t Rx_data[inputBufferSize];
  int inputTime = 4000;

  ShowHelpScreen(msgBuf, MSGBUFSIZE);

  while (1)
  {
    HAL_UART_Receive(&huart2, Rx_data, inputBufferSize, inputTime);

    double receivedDouble = 0;
    char* valuePointer = (char*)Rx_data + 1;
    receivedDouble = strtod(valuePointer, NULL);

    switch(Rx_data[0])
    {
      case 's':

        ChangeAndPrintDesiredSpeed(msgBuf, MSGBUFSIZE, receivedDouble);

        break;
      case 'h':

        ShowHelpScreen(msgBuf, MSGBUFSIZE);

        break;
      case 'g':

        ShowCurrentValues(msgBuf, MSGBUFSIZE);

        break;
      case 'p':

        ChangeAndPrintPValue(msgBuf, MSGBUFSIZE, receivedDouble);

        break;
      case 'i':

        ChangeAndPrintIValue(msgBuf, MSGBUFSIZE, receivedDouble);

        break;
      case 'd':

        ChangeAndPrintDValue(msgBuf, MSGBUFSIZE, receivedDouble);

        break;
      case 0:
        break;
    }

    for(int i = 0; i < inputBufferSize; i++)
    {
      Rx_data[i] = 0;
    }
    
  }
}

void SetServoRPS(double setSpeed)
{
  const double RPSToMillis = (94 + (2/7));
  const double servoStandStillTime = 1480;

  double newSpeed = -setSpeed;

  newSpeed *= RPSToMillis;
  newSpeed += servoStandStillTime;

  TIM3->CCR1 = (uint32_t)newSpeed;
}


void ControlServo(void* argument)
{
  uint32_t writeTimeMillis = 20;
  double servoSpeed = 0;
  double readSpeed = 0; 

  double error_prior = 0;
  double integral_prior = 0;
  double KP = 1;
  double KI = 0.950008;
  double KD = 0;

  double newOutput = 0;

  const double readDelayMicros = (((double)writeTimeMillis)/1000);

  while (1)
  {
    if (osMessageQueueGetCount(setSpd_MsgQueue) > 0)
    {
      osMessageQueueGet(setSpd_MsgQueue, &servoSpeed, NULL, 0U);   // wait for message
    }

    osMutexAcquire(mutex_ReadSpeed, osWaitForever); // try to acquire mutex
    readSpeed = MeasuredSpeed;
    osMutexRelease(mutex_ReadSpeed);

    osMutexAcquire(mutex_pid, osWaitForever); // try to acquire mutex
    KP = pValue;
    KI = iValue;
    KD = dValue;
    osMutexRelease(mutex_pid);
      
    double error = servoSpeed - readSpeed;
    double integral = integral_prior + error * readDelayMicros;
    double derivative = (error - error_prior) / readDelayMicros;

    newOutput = KP*error + KI*integral + KD*derivative;
    error_prior = error;
    integral_prior = integral;
    SetServoRPS(newOutput);

    
    HAL_Delay(writeTimeMillis);
  }
}


void ReadServoInput(void* argument)
{
  int32_t lastReadPWidth = 0;
  const int32_t maxPulseWidth = 1100;
  const double speedDivider = maxPulseWidth;
  const double readFrequency = 50;
  const uint32_t readTimeMillis = 20;

  while (1)
  {
    int32_t currentPWidth = TIM2->CCR2;
    int32_t	PWidthDifference = currentPWidth - lastReadPWidth;

    double currentSpeed = PWidthDifference;
    currentSpeed /= speedDivider;
    currentSpeed *= readFrequency;
    
    currentSpeed = std::fmod(currentSpeed, 4);

    osMutexAcquire(mutex_ReadSpeed, osWaitForever);
    MeasuredSpeed = currentSpeed;
    osMutexRelease(mutex_ReadSpeed);

    lastReadPWidth = currentPWidth;
    HAL_Delay(readTimeMillis);
  }
}

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  // ES Course Comments: Uncomment the three lines below to enable FreeRTOS.
  osKernelInitialize(); /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  InitServoPin();
  InitEchoPin();

  setSpd_MsgQueue = osMessageQueueNew(QUEUE_COUNT, sizeof(double), NULL);
  
  UART_Thread0Handle = osThreadNew(UARTHandling, nullptr, &defaultTask_attributes);

  PWMRead_THread2Handle = osThreadNew(ReadServoInput, nullptr, &defaultTask_attributes);

  PWM_Thread1Handle = osThreadNew(ControlServo, nullptr, &defaultTask_attributes);  
  
  osKernelStart(); /* Start scheduler */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17)
  {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
