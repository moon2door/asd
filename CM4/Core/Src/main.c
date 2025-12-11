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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "ipcc.h"
#include "openamp.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdlib.h>
// for openAMP
#include "rsc_table.h"
#include "virt_uart.h"
#include "stm32mp1xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MOVING_AVERAGE_WINDOW 100
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//openamp
VIRT_UART_HandleTypeDef huart0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// IRQ for tim1_channel3.
// this read I index of encoder, which means one rotation.
void TIM1_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim1);
}

int index_value;
volatile int encoder_origin = 0;
volatile int encoder_origin2 = 0;
bool is_origin_set = false;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        	// check origin encoder point
        	if(!is_origin_set){
        		encoder_origin = htim1.Instance->CNT;
        		if (encoder_origin == encoder_origin2){
        			is_origin_set = true;
        			return;
        		}
        		encoder_origin2 = encoder_origin;
        	}
            // Handle the captured value
        }
    }
}

// get encoder from start point.
#define ENOCDER_ROTATION_COUNT 40000
int calculate_relative_encoder_value(int origin_encoder, int current_encoder) {
    if (origin_encoder >= current_encoder) {
        return origin_encoder - current_encoder;
    } else {
        return ENOCDER_ROTATION_COUNT - (current_encoder - origin_encoder);
    }
}

// timestamp
uint32_t init_time_sec = 0;
uint32_t init_scaler = 0;
uint32_t cur_time_sec = 0;

void init_time(){
	init_time_sec = RTC->TR;
	init_scaler = (RTC->PRER) + 1;
}

uint32_t get_time_ms(){
    uint32_t cur_sec1, cur_sec2, cur_ms1, cur_ms2;
    uint32_t ret;

    // Read RTC_TR, RTC_SSR, and RTC_DR twice and compare for consistency
    do {
        cur_sec1 = RTC->TR;
        cur_ms1 = ((RTC->SSR * 1000) / init_scaler);
        (void)RTC->DR; // Read RTC_DR to unlock the registers
        cur_sec2 = RTC->TR;
        cur_ms2 = ((RTC->SSR * 1000) / init_scaler);
        (void)RTC->DR; // Read RTC_DR to unlock the registers
    } while (cur_sec1 != cur_sec2 || cur_ms1 != cur_ms2);

    // Calculate the elapsed time in milliseconds
    uint32_t cur_sec = cur_sec1 - init_time_sec;
    uint32_t cur_ms = cur_ms1;
    ret = cur_sec * 1000 + cur_ms;

    return ret;
}


// for speed control
//moving average
int encoder_counts[MOVING_AVERAGE_WINDOW] = {0};
int moving_average_index = 0;
int sum_counts = 0;

//rpm
float target_rpm = 24.0;
float target_ccr = 3000.0;
// for rpm check
int encoder_cur_count;
int encoder_prev_count;
int32_t count_diff;
float filtered_count_diff;
float speed;
float rpm;

//pid
float Kp = 3.3;
float Ki = 0.08;
float Kd = 0.05;
float integral_error = 0.0;
float MAX_INTEGRAL = 5.0; // Example limit, needs tuning
float previous_error = 0.0;

// checking run time in ms
uint32_t cur_time_ms = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // periodic task 1000hz. (1ms)
  if(htim->Instance == TIM2){
	  ++cur_time_ms;
	  // 1000Hz speed check.
	  encoder_cur_count = htim1.Instance->CNT;

	  if (encoder_cur_count <= encoder_prev_count) {
		  count_diff = encoder_prev_count - encoder_cur_count;
	  } else {
		  count_diff = (encoder_prev_count + 40000) - encoder_cur_count; // Handle counter rollover
	  }
	  encoder_prev_count = encoder_cur_count;

	  // Update the moving average filter
	  sum_counts -= encoder_counts[moving_average_index];
	  encoder_counts[moving_average_index] = count_diff;
	  sum_counts += count_diff;
	  moving_average_index = (moving_average_index + 1) % MOVING_AVERAGE_WINDOW;

	  filtered_count_diff = (float)sum_counts / MOVING_AVERAGE_WINDOW;

	  speed = filtered_count_diff * 1000.0;	// encoder_go/1sec
	  rpm = (speed / 40000.0) * 60.0;		// rpm ( rotations/min)

	  // I control
	  float error = target_rpm - rpm;
	  // 0.15 rpm == 1 encoder error.
	  if(fabs(error) > 0.15){
		  integral_error += error;
	  }

	  if (integral_error > MAX_INTEGRAL) {
	      integral_error = MAX_INTEGRAL;
	  } else if (integral_error < -MAX_INTEGRAL) {
	      integral_error = -MAX_INTEGRAL;
	  }

	  // D control
	  float derivative_error = error - previous_error;
	  previous_error = error;

	  // PID part
	  float control_action = Kp * error + (Ki * integral_error) + (Kd * derivative_error);


	  target_ccr = (int)(control_action);
	  uint32_t cur_ccr2 = htim8.Instance->CCR2+target_ccr;
	  if(cur_ccr2 > 300){
		  if(cur_ccr2 > 20887){
			  htim8.Instance->CCR2 = 20887;
		  } else {
			  htim8.Instance->CCR2 += (int)target_ccr;
		  }
	  } else {
		  htim8.Instance->CCR2 = 0;
          if(target_rpm == 0 ){
        	  // MOTOR_PWR Off
        	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, 1);
          } else {
        	  htim8.Instance->CCR2 = 300;
          }
	  }

  }
}


//openamp functions
//callback for openAMP(M4 Rx)
//#define RPMSG_PREFIX "RPMSG0 : "
//#define RPMSG_PREFIX_SIZE (sizeof(RPMSG_PREFIX) - 1)
//uint32_t cur_time;

int is_number(const char *str) {
    if (*str == '\0') return 0; // Empty string is not a number
    char *endptr;
    strtol(str, &endptr, 10);
    return *endptr == '\0'; // If endptr points to the null terminator, the string is a valid number
}

char VirtUart0ChannelBuffRx[RPMSG_BUFFER_SIZE];
uint16_t VirtUart0ChannelRxSize = 0;
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart)
{
	// receiving buffer part should be upgraded.
	// current version buffer managing is not good.
    VirtUart0ChannelRxSize = huart->RxXferSize < RPMSG_BUFFER_SIZE? huart->RxXferSize : RPMSG_BUFFER_SIZE-1;
    memcpy(VirtUart0ChannelBuffRx, huart->pRxBuffPtr, VirtUart0ChannelRxSize);
    if(VirtUart0ChannelBuffRx[VirtUart0ChannelRxSize-1] == '\n'){
    	VirtUart0ChannelBuffRx[VirtUart0ChannelRxSize-1] = '\0';
    } else {
    	VirtUart0ChannelBuffRx[VirtUart0ChannelRxSize] = '\0';
    }
    if (is_number(VirtUart0ChannelBuffRx)) {
            target_rpm = (float)atoi(VirtUart0ChannelBuffRx);
            if(target_rpm > 0){
            	//MOTOR_PWR on
            	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, 0);
            }
    }
//     Retrieve current time from RTC
//    cur_time = get_time_ms();
//
//    // Format the current time into a string (assuming a function itoc exists)
//    char time_str[20];  // Adjust size as needed
//    snprintf(time_str, sizeof(time_str), " %lu\n", cur_time);  // Convert time to string
//
//    // Copy received msg in a variable to send it back to master processor in main infinite loop
//    VirtUart0ChannelRxSize = huart->RxXferSize < (RPMSG_BUFFER_SIZE - RPMSG_PREFIX_SIZE - strlen(time_str))
//                             ? huart->RxXferSize - 1
//                             : (RPMSG_BUFFER_SIZE - RPMSG_PREFIX_SIZE - strlen(time_str) - 1) - 1;
//
//    // Use snprintf to add "rpmsg0 : " and current time to the VirtUart0ChannelBuffRx
//    snprintf(VirtUart0ChannelBuffRx, RPMSG_BUFFER_SIZE, "%s", RPMSG_PREFIX);
//
//    // Change memcpy regarding this
//    memcpy(VirtUart0ChannelBuffRx + RPMSG_PREFIX_SIZE, huart->pRxBuffPtr, VirtUart0ChannelRxSize);
//
//    // Append the current time to the message
//    memcpy(VirtUart0ChannelBuffRx + RPMSG_PREFIX_SIZE + VirtUart0ChannelRxSize, time_str, strlen(time_str));
//
//    // Post process (attach \0, change buffer size)
//    VirtUart0ChannelRxSize += RPMSG_PREFIX_SIZE + strlen(time_str);
//    if (VirtUart0ChannelRxSize < RPMSG_BUFFER_SIZE) {
//        VirtUart0ChannelBuffRx[VirtUart0ChannelRxSize] = '\0';
//    } else {
//        VirtUart0ChannelBuffRx[RPMSG_BUFFER_SIZE - 1] = '\0';
//    }
}


bool send_openAMP = true;

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

  if(IS_ENGINEERING_BOOT_MODE())
  {
    /* Configure the system clock */
    SystemClock_Config();
  }

  if(IS_ENGINEERING_BOOT_MODE())
  {
    /* Configure the peripherals common clocks */
    PeriphCommonClock_Config();
  }
  else
  {
    /* IPCC initialisation */
    MX_IPCC_Init();
    /* OpenAmp initialisation ---------------------------------*/
    MX_OPENAMP_Init(RPMSG_REMOTE, NULL);
  }

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  // OpenAmp setup
  if (VIRT_UART_Init(&huart0) != VIRT_UART_OK) {
    Error_Handler();
  }

  if(VIRT_UART_RegisterCallback(&huart0, VIRT_UART_RXCPLT_CB_ID, VIRT_UART0_RxCpltCallback) != VIRT_UART_OK)
  {
   Error_Handler();
  }

  // timer setup
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1|TIM_CHANNEL_2);	// encoder mode init (htim1_channel1,2 to read encoder A,B phase)						// inputcapture mode init (htim1_channel3, to read encoder I)
  if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3) != HAL_OK) {
	  Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK){
      Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2) != HAL_OK ){
	  Error_Handler();
  }

  //init time
//  init_time();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t prev_t = 0;
  while (1)
  {
	  //OpenAMP receive
	  OPENAMP_check_for_message();
	  // send encoder data to A7
		  // calculate encoder from origin, encoder speed/ms.
	  if(send_openAMP){
		  if(prev_t == cur_time_ms) continue;
		  int encoder_corrected = calculate_relative_encoder_value(encoder_origin, encoder_cur_count);
		  int encoder_speed_per_ms = round(filtered_count_diff);

		  // change format to fit the VIRT_UART_Transmit. "%05d%02lu," is the form.
		  char buffer_virtio0[RPMSG_BUFFER_SIZE];
		  snprintf(buffer_virtio0, RPMSG_BUFFER_SIZE, "%09ld%05d%05d%d\n", cur_time_ms, encoder_corrected, encoder_speed_per_ms, (int)is_origin_set);
//		  snprintf(buffer_virtio0, RPMSG_BUFFER_SIZE, "%05d%05d%d\n", encoder_corrected, encoder_speed_per_ms, (int)is_origin_set);
		  VIRT_UART_Transmit(&huart0, buffer_virtio0, strlen(buffer_virtio0));
		  prev_t = cur_time_ms;
	  }
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_DIG;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDivValue = RCC_HSI_DIV1;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL2.PLLSource = RCC_PLL12SOURCE_HSE;
  RCC_OscInitStruct.PLL2.PLLM = 3;
  RCC_OscInitStruct.PLL2.PLLN = 66;
  RCC_OscInitStruct.PLL2.PLLP = 2;
  RCC_OscInitStruct.PLL2.PLLQ = 1;
  RCC_OscInitStruct.PLL2.PLLR = 1;
  RCC_OscInitStruct.PLL2.PLLFRACV = 0x1400;
  RCC_OscInitStruct.PLL2.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL3.PLLSource = RCC_PLL3SOURCE_HSE;
  RCC_OscInitStruct.PLL3.PLLM = 2;
  RCC_OscInitStruct.PLL3.PLLN = 34;
  RCC_OscInitStruct.PLL3.PLLP = 2;
  RCC_OscInitStruct.PLL3.PLLQ = 17;
  RCC_OscInitStruct.PLL3.PLLR = 37;
  RCC_OscInitStruct.PLL3.PLLRGE = RCC_PLL3IFRANGE_1;
  RCC_OscInitStruct.PLL3.PLLFRACV = 6660;
  RCC_OscInitStruct.PLL3.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL4.PLLSource = RCC_PLL4SOURCE_HSE;
  RCC_OscInitStruct.PLL4.PLLM = 4;
  RCC_OscInitStruct.PLL4.PLLN = 99;
  RCC_OscInitStruct.PLL4.PLLP = 6;
  RCC_OscInitStruct.PLL4.PLLQ = 8;
  RCC_OscInitStruct.PLL4.PLLR = 8;
  RCC_OscInitStruct.PLL4.PLLRGE = RCC_PLL4IFRANGE_0;
  RCC_OscInitStruct.PLL4.PLLFRACV = 0;
  RCC_OscInitStruct.PLL4.PLLMODE = RCC_PLL_INTEGER;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** RCC Clock Config
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_ACLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3|RCC_CLOCKTYPE_PCLK4
                              |RCC_CLOCKTYPE_PCLK5;
  RCC_ClkInitStruct.AXISSInit.AXI_Clock = RCC_AXISSOURCE_PLL2;
  RCC_ClkInitStruct.AXISSInit.AXI_Div = RCC_AXI_DIV1;
  RCC_ClkInitStruct.MCUInit.MCU_Clock = RCC_MCUSSOURCE_PLL3;
  RCC_ClkInitStruct.MCUInit.MCU_Div = RCC_MCU_DIV1;
  RCC_ClkInitStruct.APB4_Div = RCC_APB4_DIV2;
  RCC_ClkInitStruct.APB5_Div = RCC_APB5_DIV4;
  RCC_ClkInitStruct.APB1_Div = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2_Div = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB3_Div = RCC_APB3_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Set the HSE division factor for RTC clock
  */
  __HAL_RCC_RTC_HSEDIV(24);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the common periph clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInit.CkperClockSelection = RCC_CKPERCLKSOURCE_HSE;
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
