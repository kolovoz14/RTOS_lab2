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
#define QUEUE_DATA_LEN 16
#define ADC_BUFFER_LEN 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim10;
extern DAC_HandleTypeDef hdac;

volatile unsigned long ulHighFrequencyTimerTicks;
//volatile uint16_t ADC_results[2];
//const int ADC_channel_count=sizeof(ADC_results)/sizeof(ADC_results[0]);
//volatile ADC_conversion_completed=0;

volatile union ADC_reading ADC_readings_buffer[ADC_BUFFER_LEN];
union ADC_reading ADC_readings[QUEUE_DATA_LEN];
//struct ADC_reading ADC_reading_data;
union ADC_reading ADC_reading_data;

volatile int buffer_half_full;
volatile int buffer_full;

int ADCVolt_raw;
int ADCTemp_raw;

int USE_DMA=0;

// calculate TEMP const.
float Vsense;
float Temp;
const float V25=0.76;	// V
const float Avg_Slope=2.5; // mV/C
const float VRefint=1.21; //V

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId task_AHandle;
osThreadId genAnalogSigHandle;
osMessageQId ADC_queueHandle;
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
void StartGenAnalogSig(void const * argument);
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
	return ulHighFrequencyTimerTicks;
}

//HAL_ADC
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{
	buffer_full=1;
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc1)
{
	buffer_half_full=1;
}

void read_ADC_and_send_mess()
{
	HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	ADCTemp_raw = HAL_ADC_GetValue(&hadc1);	// RANK 1

	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	ADCVolt_raw = HAL_ADC_GetValue(&hadc1);	// RANK 2

	HAL_ADC_Stop(&hadc1);

	//send data to queue
	//printf("TIMER: ADCTemp_raw: %d  ADCVolt_raw: %d  \r\n",ADCTemp_raw,ADCVolt_raw);
	ADC_reading_data.word[0]=ADCTemp_raw;
	ADC_reading_data.word[1]=ADCVolt_raw;

	osMessagePut(ADC_queueHandle,ADC_reading_data.all_data, 5);
}

void get_ADC_data_from_queue(union ADC_reading* ADC_reading_queue)
{
	for(int i=0;i<QUEUE_DATA_LEN;i++)
		{
			osEvent mess_event=osMessageGet(ADC_queueHandle,100);
			//printf("message status %d",mess_event.status);
			if(mess_event.status==0x10)	// message
			{
				ADC_reading_queue[i].all_data=mess_event.value.v;
			}
			else
			{
			printf("msg error");
			ADC_reading_queue[i].all_data=0;
			}
		}
}


float calculate_average_temp(union ADC_reading* ADC_reading_queue)
{
	uint32_t temp_sum=0;
	for(int i=0;i<QUEUE_DATA_LEN;i++)
	{
			temp_sum=temp_sum+(uint32_t)ADC_reading_queue[i].word[0];
	}

	float mean_temp=(float)temp_sum/(float)QUEUE_DATA_LEN;
	//printf("temp sum: %d \r\n",temp_sum);
	Vsense=(mean_temp/4096.0)*VRefint;
	Temp=((Vsense-V25)/Avg_Slope)+25.0;

	return Temp;
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

	//start DMA
	if(USE_DMA) HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_readings_buffer, ADC_BUFFER_LEN);

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
  osTimerStart(softwareTimer1Handle, 10);	//runs every 10 ms
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of ADC_queue */
  osMessageQDef(ADC_queue, 16, uint32_t);
  ADC_queueHandle = osMessageCreate(osMessageQ(ADC_queue), NULL);

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

  /* definition and creation of genAnalogSig */
  osThreadDef(genAnalogSig, StartGenAnalogSig, osPriorityIdle, 0, 128);
  genAnalogSigHandle = osThreadCreate(osThread(genAnalogSig), NULL);

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

	//HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_readings_buffer, ADC_BUFFER_LEN); //start DMA
	//define variables
	// QUEUE

  /* Infinite loop */
  for(;;)
  {
	// QUEUE
	get_ADC_data_from_queue(ADC_readings);

	float average_temp=calculate_average_temp(ADC_readings);
	printf("Calculated temp: %f \r\n",average_temp);
    osDelay(1000);
  }
  /* USER CODE END StartTask_A */
}

/* USER CODE BEGIN Header_StartGenAnalogSig */
/**
* @brief Function implementing the genAnalogSig thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGenAnalogSig */
void StartGenAnalogSig(void const * argument)
{
  /* USER CODE BEGIN StartGenAnalogSig */
	float analogValue=0;
	int digitalValue=0;
	const float Vref=3.3;
	const float deltaTime=(2*3.1416)/10;
	float time=0;
	const int frequency=30;
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  /* Infinite loop */
  for(;;)
  {
	analogValue=0.3*sin(time)+0.5;	// Volts
	digitalValue=(analogValue*4096.0)/Vref;
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, digitalValue);
	time=time+deltaTime;
	if(time>2*3.1416) time=0;
    osDelay(100/frequency);

  }
  /* USER CODE END StartGenAnalogSig */
}

/* softwareTimer1Callback function */
void softwareTimer1Callback(void const * argument)
{
	//version 2 with DMA
  /* USER CODE BEGIN softwareTimer1Callback */

	// get data from channels
	//HAL_ADC_Start(&hadc1);
	//HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_readings_buffer, ADC_BUFFER_LEN);
//	while(buffer_full==0)
//	{
//		//wait until buffer is full
//	}
	//HAL_ADC_Stop(&hadc1);
	//HAL_ADC_Stop_DMA(&hadc1);
	//buffer_half_full=0;
	//buffer_full=0;


// version 1 with queue
	read_ADC_and_send_mess();

  /* USER CODE END softwareTimer1Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
