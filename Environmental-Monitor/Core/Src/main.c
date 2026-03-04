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
#include "i2c.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
} Cal;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t R_Hum, R_Temp, R_Press;
float Hum, Temp, Press;
Cal c;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Calibration(void) {
	uint8_t H[9], T[6], P[18];

	HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0xA1, I2C_MEMADD_SIZE_8BIT, &H[0], 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0xE1, I2C_MEMADD_SIZE_8BIT, &H[1], 8, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0x88, I2C_MEMADD_SIZE_8BIT, T, 6, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0x8E, I2C_MEMADD_SIZE_8BIT, P, 18, HAL_MAX_DELAY);

	c.dig_H1 = H[0];
	c.dig_H2 = (H[2] << 8) | H[1];
	c.dig_H3 = H[3];
	c.dig_H4 = (int16_t)((H[4] << 4) | (H[5] & 0x0F));
	c.dig_H5 = (int16_t)((H[7] << 4) | (H[6] >> 4));
	c.dig_H6 = H[8];

	c.dig_T1 = (T[1] << 8) | T[0];
	c.dig_T2 = (T[3] << 8) | T[2];
	c.dig_T3 = (T[5] << 8) | T[4];

	c.dig_P1 = (P[1] << 8) | P[0];
	c.dig_P2 = (P[3] << 8) | P[2];
	c.dig_P3 = (P[5] << 8) | P[4];
	c.dig_P4 = (P[7] << 8) | P[6];
	c.dig_P5 = (P[9] << 8) | P[8];
	c.dig_P6 = (P[11] << 8) | P[10];
	c.dig_P7 = (P[13] << 8) | P[12];
	c.dig_P8 = (P[15] << 8) | P[14];
	c.dig_P9 = (P[17] << 8) | P[16];
}

void Raw_Data(void){
	uint8_t Buff[8];

	HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0xF7, I2C_MEMADD_SIZE_8BIT, Buff, 8, HAL_MAX_DELAY);

	R_Hum = (Buff[6] << 8) | Buff[7];
	R_Temp = (Buff[3] << 12) | (Buff[4] << 4) | (Buff[5] >> 4);
	R_Press = (Buff[0] << 12) | (Buff[1] << 4) | (Buff[2] >> 4);
}

//###########################################
//CODE FROM DATASHEET
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t t_fine;
int32_t BME280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2, T;
var1 = ((((adc_T>>3) - ((int32_t)c.dig_T1<<1))) * ((int32_t)c.dig_T2)) >> 11;
var2 = (((((adc_T>>4) - ((int32_t)c.dig_T1)) * ((adc_T>>4) - ((int32_t)c.dig_T1))) >> 12) *
((int32_t)c.dig_T3)) >> 14;
t_fine = var1 + var2;
T = (t_fine * 5 + 128) >> 8;
return T;
}
// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BME280_compensate_P_int64(int32_t adc_P)
{
int64_t var1, var2, p;
var1 = ((int64_t)t_fine) - 128000;
var2 = var1 * var1 * (int64_t)c.dig_P6;
var2 = var2 + ((var1*(int64_t)c.dig_P5)<<17);
var2 = var2 + (((int64_t)c.dig_P4)<<35);
var1 = ((var1 * var1 * (int64_t)c.dig_P3)>>8) + ((var1 * (int64_t)c.dig_P2)<<12);
var1 = (((((int64_t)1)<<47)+var1))*((int64_t)c.dig_P1)>>33;
if (var1 == 0)
{
return 0; // avoid exception caused by division by zero
}
p = 1048576-adc_P;
p = (((p<<31) - var2)*3125)/var1;
var1 = (((int64_t)c.dig_P9) * (p>>13) * (p>>13)) >> 25;
var2 = (((int64_t)c.dig_P8) * p) >> 19;
p = ((p + var1 + var2) >> 8) + (((int64_t)c.dig_P7)<<4);
return (uint32_t)p;
}
// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
uint32_t bme280_compensate_H_int32(int32_t adc_H)
{
int32_t v_x1_u32r;
v_x1_u32r = (t_fine - ((int32_t)76800));
v_x1_u32r = (((((adc_H << 14) - (((int32_t)c.dig_H4) << 20) - (((int32_t)c.dig_H5) * v_x1_u32r)) +
((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)c.dig_H6)) >> 10) * (((v_x1_u32r *
		((int32_t)c.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
		((int32_t)c.dig_H2) + 8192) >> 14));
		v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)c.dig_H1)) >> 4));
		v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
		v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
		return (uint32_t)(v_x1_u32r>>12);
		}
//END OF CODE FROM DATASHEET
//###########################################

void Compensate(void){
	Temp = BME280_compensate_T_int32(R_Temp) / 100.0f; //C
	Press = (BME280_compensate_P_int64(R_Press) / 256.0f) / 100.0f; //hPa
	Hum = bme280_compensate_H_int32(R_Hum) / 1024.0f; //%
}

void Configure(void){
	uint8_t data = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, 0x76 << 1, 0xF2, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	data = (0x02 << 5) | (0x05 << 2) | 0x03;
	HAL_I2C_Mem_Write(&hi2c1, 0x76 << 1, 0xF4, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	data = (0x00 << 5) | (0x04 << 2) | 0x00;
	HAL_I2C_Mem_Write(&hi2c1, 0x76 << 1, 0xF5, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

//#############################################
void BME280_Init(void){
	uint8_t data = 0xB6;
	HAL_I2C_Mem_Write(&hi2c1, 0x76 << 1, 0xE0, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY); //reset
	HAL_Delay(5);

	Calibration();
	Configure();
}
void BME280_Read(void){
	Raw_Data();
	Compensate();
}
void BME280_Display(void){

}
//#############################################
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  uint8_t id;
  if (HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0xD0, I2C_MEMADD_SIZE_8BIT, &id, 1, HAL_MAX_DELAY) != HAL_OK) return 1;
  if (id != 0x60) return 1;
  BME280_Init();
  uint32_t start = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(HAL_GetTick() - start >= 3000){
		  BME280_Read();
		  BME280_Display();
		  start = HAL_GetTick();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
