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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include <stdio.h>
#include "ism330dhcx.h"
#include "sensor_sample_buffer.h"
#include "sensor_analysis_window.h"
#include "sensor_acquisition.h"
#include "condition_monitor.h"

#include "cli.h"
//#include "cli_output.h"
#include "control_command.h"
//#include "uart_console.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    condition_state_t state;
    float severity_score;
    float max_magnitude_mps2;
    uint32_t consecutive_sensor_errors;
    uint32_t dropped_sensor_samples;
    uint32_t dropped_telemetry_reports;
} telemetry_report_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TELEMETRY_QUEUE_CAPACITY 4U

#define UART_RX_QUEUE_CAPACITY     64U
#define CONTROL_RESPONSE_TIMEOUT_TICKS 250U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */
#define LED_DELAY_MS					   500U



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* Definitions for acquisitionTask */
osThreadId_t acquisitionTaskHandle;
const osThreadAttr_t acquisitionTask_attributes = {
  .name = "acquisitionTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for interfaceTask */
osThreadId_t interfaceTaskHandle;
const osThreadAttr_t interfaceTask_attributes = {
  .name = "interfaceTask",
  .stack_size = 750 * 4,
  .priority = (osPriority_t) osPriorityLow,
};


/* USER CODE BEGIN PV */

static volatile uint32_t timer_event_count=0U;

static volatile bool button_pressed_event = false;
static bool blinking_enabled = true;
static uint32_t last_button_tick = 0U;
static const uint32_t debounce_time_ms = 40U;



//Motion sensor
static ism330dhcx_t motion_sensor;
static sensor_sample_t raw_sample;



//DMA variables
static sensor_physical_sample_t  dma_converted_sample;
static sensor_sample_buffer_t sensor_sample_buffer;
static sensor_window_t sensor_window;
static sensor_acquisition_t sensor_acquisition;
static uint32_t previous_timestamp_us = 0U;
static uint32_t latest_dt_us = 0U;

static uint32_t min_dt_us = UINT32_MAX;
static uint32_t max_dt_us = 0U;

static uint64_t sum_dt_us = 0U;
static uint32_t dt_sample_count = 0U;


static ism330dhcx_axes_t
    centered_accelerations[SENSOR_ANALYSIS_WINDOW_SIZE];
static sensor_acceleration_time_features_t acceleration_features;
static condition_monitor_t condition_monitor;



static osMessageQueueId_t telemetry_queue_handle;
static osMessageQueueId_t rx_queue_handle;
static uint8_t uart_rx_byte;
static volatile uint32_t uart_rx_dropped_byte_count = 0U;
static volatile uint32_t uart_rx_rearm_error_count = 0U;
static volatile bool uart_rx_restart_required = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void handle_control_command(
    const control_command_t *command,
    const telemetry_report_t *report,
    bool report_available);
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
  MX_TIM6_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
      Error_Handler();
  }
  //ISM330DHCX sensor
  ism330dhcx_status_t sensor_status;
  uint8_t device_id = 0U;

  const ism330dhcx_interface_config_t interface_config =
  {
		  .auto_increment = true,
		  .block_data_update = true
  };

  const ism330dhcx_sensor_config_t sensor_config =
  {
		  .accel_odr = ISM330DHCX_ACCEL_ODR_104_HZ,
		  .accel_range = ISM330DHCX_ACCEL_RANGE_4G,
		  .accel_mode  = ISM330DHCX_MODE_HIGH_PERFORMANCE,

		  .gyro_odr = ISM330DHCX_GYRO_ODR_104_HZ,
		  .gyro_range = ISM330DHCX_GYRO_500_DPS,
		  .gyro_mode   = ISM330DHCX_MODE_HIGH_PERFORMANCE
  };

  const ism330dhcx_interrupt1_output_config_t int1_config =
  {
	  .accelerometer_drdy = true,
	  .gyro_drdy = false,
	  .pulsed_drdy = true
  };

  static const condition_monitor_config_t condition_config =
  {
      .impact_reference_mps2 = 1.0f,
      .decay_factor = 0.8f,

      .warning_enter_score = 3.0f,
      .warning_exit_score = 1.0f,
      .alarm_enter_score = 10.0f,
	  .alarm_exit_score = 6.0f,
      .maximum_score = 100.0f,

      .sensor_fault_consecutive_error_limit = 3U
  };


  /* Step 1: Initialize the driver object. */
  sensor_status = ism330dhcx_init(
      &motion_sensor,
      &hi2c1,
	  ISM330DHCX_I2C_ADDRESS_HIGH_7BIT,
      100U);

  if (sensor_status != ISM330DHCX_OK)
  {
      Error_Handler();
  }

  /* Step 2: Confirm that the sensor responds. */
  sensor_status = ism330dhcx_read_device_id(
      &motion_sensor,
      &device_id);

  if (sensor_status != ISM330DHCX_OK)
  {
      Error_Handler();
  }

  /* Step 3: Reset the sensor and wait for completion. */
  sensor_status = ism330dhcx_reset(&motion_sensor);

  if (sensor_status != ISM330DHCX_OK)
  {
      Error_Handler();
  }
  /* Step 4: Confirm that the sensor still responds after reset. */
  device_id = 0U;

  sensor_status = ism330dhcx_read_device_id(
      &motion_sensor,
      &device_id);

  if (sensor_status != ISM330DHCX_OK)
  {
      Error_Handler();
  }

  /* Step 5: Apply the interface configuration. */
  sensor_status = ism330dhcx_configure_interface(
      &motion_sensor,
      &interface_config);

  if (sensor_status != ISM330DHCX_OK)
  {
      Error_Handler();
  }

  /* Step 6: Apply the sensor configuration. */

  sensor_status = ism330dhcx_configure_sensor(
      &motion_sensor,
      &sensor_config);

  if (sensor_status != ISM330DHCX_OK)
  {
      Error_Handler();
  }

  /* Step 7: Apply the INT1 DRDY configuration. */
  sensor_status = ism330dhcx_configure_int1(
      &motion_sensor,
      &int1_config);

  if (sensor_status != ISM330DHCX_OK)
  {
      Error_Handler();
  }
  /* Step 8: Apply the DMA configuration. */
  (void)sensor_sample_buffer_init(&sensor_sample_buffer);
  (void)sensor_window_init(&sensor_window);
  (void)sensor_acquisition_init(&sensor_acquisition, &motion_sensor);

  /* Step 9: Apply the condition monitoring configuration. */

  if (!condition_monitor_init(&condition_monitor,&condition_config))
  {
      Error_Handler();
  }

  if(HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
	  Error_Handler();
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  telemetry_queue_handle = osMessageQueueNew(
      TELEMETRY_QUEUE_CAPACITY,
      sizeof(telemetry_report_t),
      NULL);

  if (telemetry_queue_handle == NULL)
  {
      Error_Handler();
  }

  rx_queue_handle = osMessageQueueNew(
      UART_RX_QUEUE_CAPACITY,
      sizeof(uint8_t),
      NULL);
  if (rx_queue_handle == NULL)
  {
      Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of acquisitionTask */
  acquisitionTaskHandle = osThreadNew(StartDefaultTask, NULL, &acquisitionTask_attributes);

  /* creation of interfaceTask */
  interfaceTaskHandle = osThreadNew(StartTask02, NULL, &interfaceTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 4999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void handle_control_command(
    const control_command_t *command,
    const telemetry_report_t *report,
    bool report_available)
{
	if((command == NULL) || (report == NULL)){return;}

	switch(command->command_type){
	case CONTROL_COMMAND_HELP:
	    printf(
	        "Available commands:\r\n"
	        "  help                         - Show this command list\r\n"
	        "  get status                   - Show the latest monitoring report\r\n"
	        "\r\n"
	        "Planned commands (not implemented yet):\r\n"
	        "  get config                   - Show sensor and monitoring settings\r\n"
	        "  get rate                     - Show configured sampling rates\r\n"
	        "  set rate <hz>                - Request a supported sampling rate\r\n"
	        "  get impact-reference         - Show the severity calculation reference\r\n"
	        "  set impact-reference <mps2>   - Set reference acceleration in m/s^2\r\n"
	        "  get errors                   - Show sensor and UART error/drop counters\r\n"
	        "  get version                  - Show firmware version\r\n"
	        "  start                        - Start acquisition\r\n"
	        "  stop                         - Stop acquisition\r\n");
		break;

	case CONTROL_COMMAND_GET_STATUS:
		if(!report_available)
		{
			printf("No report available yet\r\n");
			break;
		}
	    printf(
	        "condition_state=%u\r\n"
	        "severity_score=%.2f\r\n"
	        "max_magnitude_mps2=%.3f\r\n"
	        "consecutive_sensor_errors=%lu\r\n"
	        "dropped_sensor_samples=%lu\r\n"
	        "dropped_telemetry_reports=%lu\r\n",
	        (unsigned int)report->state,
	        report->severity_score,
	        report->max_magnitude_mps2,
	        (unsigned long)report->consecutive_sensor_errors,
	        (unsigned long)report->dropped_sensor_samples,
	        (unsigned long)report->dropped_telemetry_reports);
		break;

	default:printf("Command not implemented yet \r\n");
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	//Pin interrupt with 40ms debounce, sets the event flag
	if(GPIO_Pin == GPIO_PIN_13)
	{
		const uint32_t now = HAL_GetTick();

		if(now- last_button_tick >= debounce_time_ms)
		{
			button_pressed_event = true;
			last_button_tick = now;
		}

	}
	//Pin interrupt with data-ready event
	if(GPIO_Pin == GPIO_PIN_10)
	{
		uint32_t time_us = __HAL_TIM_GET_COUNTER(&htim2);
		sensor_acquisition_on_drdy(&sensor_acquisition, time_us);

	}
}

//Printf
int _write(int file, char *data, int length)
{
    (void)file;

    if (HAL_UART_Transmit(&huart2,
                          (uint8_t *)data,
                          (uint16_t)length,
                          100U) == HAL_OK)
    {
        return length;
    }

    return -1;
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C1)
	{
		sensor_acquisition_on_dma_complete(
		    &sensor_acquisition);

	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    /* check which I2C instance failed */
	if (hi2c->Instance == I2C1)
	{
		/* notify sensor_acquisition */
		sensor_acquisition_on_error(&sensor_acquisition);
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* check which USART instance */
	if (huart->Instance == USART2)
	{

	    //Message put on the queue
	    const osStatus_t queue_status =
	        osMessageQueuePut(
	            rx_queue_handle,
	            &uart_rx_byte,
	            0U,  /* No message priority. */
	            0U);

	    if (queue_status == osErrorResource)
	    {
	        /* Queue was full. */
	        ++uart_rx_dropped_byte_count;
	    }

		if (HAL_UART_Receive_IT(
		        huart,
		        &uart_rx_byte,
		        sizeof(uart_rx_byte)) != HAL_OK)
		{
			++uart_rx_rearm_error_count;
			uart_rx_restart_required = true;
		    return;
		}

	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	   (void)argument;

	    condition_state_t last_reported_state =
	        CONDITION_STATE_COUNT;
	    uint32_t dropped_telemetry_reports = 0U;

  /* Infinite loop */
  for(;;)
  {


		bool has_new_window = false;
		//DMA
		const ism330dhcx_status_t status =
		    sensor_acquisition_process(&sensor_acquisition);

		if (status == ISM330DHCX_INVALID_ARGUMENT)
		{
		    /* Programming/configuration error. */
		    Error_Handler();
		}
		else if (status != ISM330DHCX_OK)
		{
		    /* Count recoverable DMA-start errors too. */
		    sensor_acquisition_on_error(&sensor_acquisition);
		}


		if (sensor_acquisition_get_sample(
		        &sensor_acquisition,
		        &raw_sample))
		{
		    (void)sensor_sample_buffer_push(
		        &sensor_sample_buffer,
		        &(raw_sample));
		}



		if (sensor_sample_buffer_pop(
		        &sensor_sample_buffer,
		        &raw_sample))
		{


		    if (previous_timestamp_us != 0U)
		    {
		        latest_dt_us =
		            raw_sample.timestamp_us - previous_timestamp_us;

		        if (latest_dt_us < min_dt_us)
		        {
		            min_dt_us = latest_dt_us;
		        }

		        if (latest_dt_us > max_dt_us)
		        {
		            max_dt_us = latest_dt_us;
		        }

		        sum_dt_us += latest_dt_us;
		        ++dt_sample_count;
		    }

		    previous_timestamp_us = raw_sample.timestamp_us;
		    /* convert raw_sample here */
		    const ism330dhcx_status_t dma_conversion_status =
		        ism330dhcx_convert_raw_sample(
		            &motion_sensor,
		            &raw_sample.data,
		            &dma_converted_sample.data);

		    if (dma_conversion_status != ISM330DHCX_OK)
		    {
		        Error_Handler();
		    }

		    dma_converted_sample.timestamp_us =
		        raw_sample.timestamp_us;

		    sensor_window_push(
		        &sensor_window,
		        &dma_converted_sample);

		}

		//Window consumption
		if (sensor_window_is_ready(&sensor_window))
		{

		    if (!sensor_window_calculate_acceleration_features(
		    	    &sensor_window,
		    	    &acceleration_features,
					centered_accelerations))
		    {
		        Error_Handler();
		    }
		    has_new_window = true;

		    sensor_window_release(&sensor_window);

		}

		const condition_monitor_input_t condition_input =
		{
		    .has_new_window = has_new_window,

		    .max_magnitude_mps2 =
		        has_new_window
		            ? acceleration_features.max_magnitude_mps2
		            : 0.0f,

		    .consecutive_sensor_errors =
		        sensor_acquisition_get_consecutive_error_count(
		            &sensor_acquisition)
		};

		if (!condition_monitor_update(
		        &condition_monitor,
		        &condition_input))
		{
		    Error_Handler();
		}

		const condition_state_t current_state =
		    condition_monitor_get_state(&condition_monitor);

		/*
		 * Prints a report of sensor acquisition window
		 */
		if (has_new_window ||
		    (current_state != last_reported_state))
		{

		    const telemetry_report_t report =
		    {
		    		.dropped_telemetry_reports = dropped_telemetry_reports,
					.state = current_state,
					.severity_score =
							condition_monitor_get_severity_score(&condition_monitor),
					.max_magnitude_mps2 =
							has_new_window ? acceleration_features.max_magnitude_mps2 : 0.0f,
					.consecutive_sensor_errors =
							condition_input.consecutive_sensor_errors,
					.dropped_sensor_samples =
							sensor_acquisition_get_dropped_sample_count(&sensor_acquisition)


		    };

		    //Message put on the queue
		    const osStatus_t queue_status =
		        osMessageQueuePut(
		            telemetry_queue_handle,
		            &report,
		            0U,  /* No message priority. */
		            0U);
		    if (queue_status == osOK)
		    {
		        last_reported_state = current_state;
		    }
		    else if (queue_status == osErrorResource)
		    {
		        /* Queue was full. */
		        ++dropped_telemetry_reports;
		    }
		    else
		    {
		        /* Invalid queue or another programming error. */
		        Error_Handler();
		    }
		}

    osDelay(1U);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the interfaceTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	(void)argument;
	uint32_t processed_timer_event_count = 0U;

	telemetry_report_t report = {0};
	bool report_available = false;

	if (HAL_UART_Receive_IT(
	        &huart2,
	        &uart_rx_byte,
	        sizeof(uart_rx_byte)) != HAL_OK)
	{
	    Error_Handler();
	}
	static command_assembler_state_t cli_assember_state;
	(void)command_assembler_init(&cli_assember_state);
  /* Infinite loop */
  for(;;)
  {
	  uint8_t received_byte;

	  const osStatus_t rx_status = osMessageQueueGet(
	      rx_queue_handle,
	      &received_byte,
	      NULL,
	      1U);

	  if(rx_status == osOK)
	  {
		  const command_assembler_status_t assembler_status =   command_byte_processing(&cli_assember_state, received_byte);


		  switch(assembler_status)
		  {
		  case COMMAND_NOT_COMPLETE:
			  //Nothing
			  break;

		  case COMMAND_IS_AVAILABLE:
			  printf("Received command: %s\r\n", cli_assember_state.command);
			  control_command_t control_command;
			  if (!command_parser(cli_assember_state.command, &control_command))
			  {
			      printf("Unknown command or invalid argument\r\n");
			  }
			  else
			  {
			      handle_control_command(
			          &control_command, &report, report_available);
			  }

			  (void)command_assembler_init(&cli_assember_state);
			  break;
		  case COMMAND_IS_DISCARDED:
			  printf("Command too long \r\n");
			  break;

		  case COMMAND_INVALID_ARGUMENT:
			  Error_Handler();
			  break;

		  default:Error_Handler();
		  break;
		  }

	  }
	  else if (rx_status != osErrorTimeout)
	  {
		  Error_Handler();
	  }
	  const osStatus_t queue_status =
	      osMessageQueueGet(
	          telemetry_queue_handle,
	          &report,
	          NULL,
	          0U);
	  if (queue_status == osOK)
	  {
		  report_available = true;
//	      printf(
//	          "Condition monitor State=%u Severity score=%.2f Max magnitude=%.2f Consecutive sensor errors=%lu Sensor dropped samples=%lu Telemetry dropped reports=%lu\r\n",
//	          (unsigned int)report.state,
//	          report.severity_score,
//	          report.max_magnitude_mps2,
//	          (unsigned long)report.consecutive_sensor_errors,
//	          (unsigned long)report.dropped_sensor_samples,
//	          (unsigned long)report.dropped_telemetry_reports);
	  }

	  else if (queue_status != osErrorResource)
	  {
	      Error_Handler();
	  }

		//BLINKER CODE BELOW

	  const uint32_t produced_timer_events = timer_event_count;

	    //Check the button, toggles the enabled flag
	    if(button_pressed_event)
	    {
	    	button_pressed_event = false;
	    	blinking_enabled = !blinking_enabled;

	    	if(!blinking_enabled)
	    	{
	    		HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);
	    	}
	    }

	    if (processed_timer_event_count != produced_timer_events)
	    {


	        ++processed_timer_event_count;

	        if (blinking_enabled)
	        {
	            HAL_GPIO_TogglePin(
	                LED2_GPIO_PORT,
	                LED2_PIN);


	        }
	    }

  }
  /* USER CODE END StartTask02 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

	//Timer that gives the toggle frequency
	if(htim->Instance == TIM6)
	{
		++timer_event_count;
	}

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
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
