/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "i2c.h"
#include "quadspi.h"
#include "rtc.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "icm20948.h"       // Driver for ICM20948 IMU (Inertial Measurement Unit)
#include "time.h"           // Time related functions
#include "usbd_cdc_if.h"    // USB CDC (Communication Device Class) interface functions
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SRAM_BUFFER_SIZE 10
#define SAMPLES_PER_PAGE 11 //(MEMORY_PAGE_SIZE / sizeof(IMU_Dat))    //Calculating the number of samples per page
#define QSPI_START_ADDRESS 0x0 // Starting address for QSPI write
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
axises my_gyro;
axises my_accel;
axises my_mag;

typedef struct {
    time_data time_info;
    icm_20948_data sensor_data;
} combined_data;

RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};

int current_index = 0;
uint32_t lastErasedSector = 0x0;

NMEA_Result NMEA_result;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
bool is_SRAM_full(void);
void fill_SRAM_with_mock_data(void);
void batch_read_from_QSPI(IMU_Data* buffer, uint32_t startAddress, uint32_t numSamples);
void batch_write_to_QSPI(IMU_Data* buffer, uint32_t startAddress, uint32_t numSamples);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <string.h>
#define SECTORS_COUNT 100
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    //initialize organized data: time data+ IMU sensor data
	combined_data dataToSend;
	//get starttime in order to get elapsed time
	uint32_t startTime = HAL_GetTick();
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
  MX_QUADSPI_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //initialize ICM gyroscope, accelerometer and magnetometer peripherals and configuration
  icm20948_init();
  ak09916_init();


  CSP_QUADSPI_Init();

  if (CSP_QUADSPI_Init() != HAL_OK)
     {
      Error_Handler();
     }

  if (CSP_QSPI_Erase_Chip() != HAL_OK)
     {
      Error_Handler();
     }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	// This segment fetches sensor data, combines it with time information, 
    // formats it into a specific string format, and sends it over USB.
	  dataToSend.time_info = read_time(startTime);  // Assume you already have the read_time function
	  dataToSend.sensor_data = read_all_data();     // Assume you have modified the read_all_data function as previously indicated

	  char buffer[512]; // suppose 512 bytes is big enough
	 // Creating a formatted string from the combined time and sensor data
	  sprintf(buffer,
			  "#%u&" //unix timestamp
			  "%04d-%02d-%02d %02d:%02d:%02d&" // utc_timestamp
			  "%04d-%02d-%02d %02d:%02d:%02d&" //
			  "%02u:%02u&"//
			  "x_accel = %f/y_accel = %f/z_accel = %f&"
			  "x_gyro = %f/y_gyro = %f/z_gyro =  %f&"
			  "x_mag = %f/y_mag = %f/z_mag = %f&\r\n",
			  dataToSend.time_info.unix_timestamp,

			  dataToSend.time_info.utc_time.year,
			  dataToSend.time_info.utc_time.month,
			  dataToSend.time_info.utc_time.date,
			  dataToSend.time_info.utc_time.hour,
			  dataToSend.time_info.utc_time.min,
			  dataToSend.time_info.utc_time.sec,

			  dataToSend.time_info.uk_time.year,
			  dataToSend.time_info.uk_time.month,
			  dataToSend.time_info.uk_time.date,
			  dataToSend.time_info.uk_time.hour,
			  dataToSend.time_info.uk_time.min,
			  dataToSend.time_info.uk_time.sec,

			  dataToSend.time_info.elapsed_minutes, dataToSend.time_info.elapsed_seconds,

			  dataToSend.sensor_data.x_accel,
			  dataToSend.sensor_data.y_accel,
			  dataToSend.sensor_data.z_accel,

			  dataToSend.sensor_data.x_gyro,
			  dataToSend.sensor_data.y_gyro,
			  dataToSend.sensor_data.z_gyro,

			  dataToSend.sensor_data.x_magnet,
			  dataToSend.sensor_data.y_magnet,
			  dataToSend.sensor_data.z_magnet

	  );
     //transmit to the USB VCP
	  CDC_Transmit_FS(buffer, strlen(buffer));
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      // Check if the SRAM is full, if yes, transfer data to QSPI Flash,
      // otherwise continue to fill the SRAM with mock data.
	if (is_SRAM_full()) {
        	            transfer_data_to_QSPI_Flash();
        	 } else {
        	            fill_SRAM_with_mock_data(); 
        	 }
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}

/**
  * @brief Writes batches of IMU data to the QSPI flash memory.
  * @param buffer: Pointer to the buffer containing the IMU data to be written.
  * @param startAddress: The starting address in the QSPI flash memory to write the data.
  * @param numSamples: Number of IMU samples to be written.
  */
void batch_write_to_QSPI(IMU_Data* buffer, uint32_t startAddress, uint32_t numSamples) {
    uint32_t numPages = (numSamples + SAMPLES_PER_PAGE - 1) / SAMPLES_PER_PAGE;
    uint32_t endAddress = startAddress + numPages * MEMORY_PAGE_SIZE;

    for (uint32_t pageIndex = 0; pageIndex < numPages; pageIndex++) {
        uint32_t address = startAddress + pageIndex * MEMORY_PAGE_SIZE;
        uint32_t samplesToWrite = (pageIndex == numPages - 1) ? (numSamples % SAMPLES_PER_PAGE) : SAMPLES_PER_PAGE;

        // Check if a new sector is entered
        if ((address / MEMORY_SECTOR_SIZE) != lastErasedSector) {
            // Erase the new sector
            if (CSP_QSPI_EraseSector(address, address + MEMORY_SECTOR_SIZE) != HAL_OK) {
                Error_Handler();
            }
            // Update the last erased sector marker
            lastErasedSector = address / MEMORY_SECTOR_SIZE;
        }

        if (CSP_QSPI_WriteMemory((uint8_t*)&buffer[pageIndex * SAMPLES_PER_PAGE], address, samplesToWrite * sizeof(IMU_Data)) != HAL_OK) {
            Error_Handler();
        }

        HAL_Delay(10);  // Delay to ensure completion of write operation
    }
}

/**
  * @brief Reads batches of IMU data from the QSPI flash memory.
  * @param buffer: Pointer to the buffer where the IMU data will be stored after reading.
  * @param startAddress: The starting address in the QSPI flash memory from where the data will be read.
  * @param numSamples: Number of IMU samples to be read.
  */
void batch_read_from_QSPI(IMU_Data* buffer, uint32_t startAddress, uint32_t numSamples) {
    uint32_t numPages = (numSamples + SAMPLES_PER_PAGE - 1) / SAMPLES_PER_PAGE;

    for (uint32_t pageIndex = 0; pageIndex < numPages; pageIndex++) {
        uint32_t address = startAddress + pageIndex * MEMORY_PAGE_SIZE ;
        uint32_t samplesToRead = (pageIndex == numPages - 1) ? (numSamples % SAMPLES_PER_PAGE) : SAMPLES_PER_PAGE;

        if (CSP_QSPI_Read((uint8_t*)&buffer[pageIndex * SAMPLES_PER_PAGE], address, samplesToRead * sizeof(IMU_Data)) != HAL_OK) {
            Error_Handler();
        }
    }
}

/**
  * @brief Checks if the SRAM buffer is full.
  * @return Returns 'true' if the SRAM buffer is full, otherwise 'false'.
  */
bool is_SRAM_full() {
    return current_index >= SRAM_BUFFER_SIZE;
}

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
