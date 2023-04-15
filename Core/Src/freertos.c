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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//for debuging
extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim10;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile unsigned long ulHighFrequencyTimerTicks;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId task_AHandle;
osTimerId softwareTimer1Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask_A(void const * argument);
void softwareTimer1Callback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
	ulHighFrequencyTimerTicks = 0;
	HAL_TIM_Base_Start_IT(&htim10);
}

__weak unsigned long getRunTimeCounterValue(void)
{
//return 0;
	return ulHighFrequencyTimerTicks;
//return HAL_GetTick();
//return xTaskGetTickCount();
}
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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

  /* Create the timer(s) */
  /* definition and creation of softwareTimer1 */
  osTimerDef(softwareTimer1, softwareTimer1Callback);
  softwareTimer1Handle = osTimerCreate(osTimer(softwareTimer1), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of task_A */
  osThreadDef(task_A, StartTask_A, osPriorityIdle, 0, 256);
  task_AHandle = osThreadCreate(osThread(task_A), NULL);

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
	int arr[100];
    osDelay(1000);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//    printf("UART TEST: char: %c int: %d float: %f \r\n", 'a',46,4.543);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask_A */
/**
* @brief Function implementing the task_A thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_A */
void StartTask_A(void const * argument)
{
  /* USER CODE BEGIN StartTask_A */
	//define variables
	uint16_t raw_data;
	int ADCVolt_raw;
	int ADCTemp_raw;
	int ADCVoltRef_raw;

	// calculate TEMP const.
	float Vsense;
	float Temp;
	const float V25=0.76;	// V
	const float Avg_Slope=2.5; // mV/C
	const float VRefint=1.21; //V

  /* Infinite loop */
  for(;;)
  {
	//printf("task_A\r\n");
	HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	ADCTemp_raw = HAL_ADC_GetValue(&hadc1);	// RANK 1

	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	ADCVolt_raw = HAL_ADC_GetValue(&hadc1);	// RANK 2

	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	ADCVoltRef_raw = HAL_ADC_GetValue(&hadc1);	// RANK 3

	HAL_ADC_Stop(&hadc1);
	//printf("ADCVolt_raw: %d  ADCTemp_raw: %d  VoltRef: %d \r\n",ADCVolt_raw,ADCTemp_raw,ADCVoltRef_raw);

	// calculate TEMP

	Vsense=(ADCTemp_raw/4096.0)*VRefint;
	Temp=((Vsense-V25)/Avg_Slope)+25.0;

	printf("Calculated temp: %f \r\n",Temp);
    osDelay(1000);
  }
  /* USER CODE END StartTask_A */
}

/* softwareTimer1Callback function */
void softwareTimer1Callback(void const * argument)
{
  /* USER CODE BEGIN softwareTimer1Callback */
	//read adc
	//send data from temp sensor
//	float ADCVolt;
//	float ADCTemp;
//	HAL_ADC_Start(&hadc1);
//
//	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//	ADCVolt = HAL_ADC_GetValue(&hadc1);
//
//	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//	ADCTemp = HAL_ADC_GetValue(&hadc);
//
//	printf("ADCVolt: %f",ADCVolt);
//	HAL_ADC_Stop(&hadc1);
	//DMA
//	float ADCVolt=0;
//	HAL_ADC_Start(&hdma_adc1);
//	HAL_ADC_PollForConversion(&hdma_adc1, HAL_MAX_DELAY);
//	ADCVolt = HAL_ADC_GetValue(&hdma_adc1);
//	printf("DMA ADCVolt: %f",ADCVolt);
//	HAL_ADC_Stop(&hdma_adc1);
	//float temp_sensor=-300.0;
	//putchar(temp_sensor);


  /* USER CODE END softwareTimer1Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
