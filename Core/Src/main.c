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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SPI_flash.h"
#include "lfs.h"
#include "lfs_config.h"
#include "definitions.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VoltageRange_1        ((uint8_t)0x00)  /*!< Device operating range: 1.8V to 2.1V */
#define VoltageRange_2        ((uint8_t)0x01)  /*!<Device operating range: 2.1V to 2.7V */
#define VoltageRange_3        ((uint8_t)0x02)  /*!<Device operating range: 2.7V to 3.6V */
#define VoltageRange_4        ((uint8_t)0x03)  /*!<Device operating range: 2.7V to 3.6V + External Vpp */
#define SECTOR_MASK           ((uint32_t)0xFFFFFF07)
#define FLASH_Sector_3			((uint16_t)0x0018) /*!< Sector Number 3   */
#define FLASH_Sector_4     		((uint16_t)0x0020) /*!< Sector Number 4   */
#define FLASH_Sector_5     		((uint16_t)0x0028) /*!< Sector Number 5   */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
lfs_t 							lfs;
lfs_file_t 						file;
struct lfs_config				cfg;
static struct lfs_file_config	fileConf;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t SPI_flash_read_byte(void){
	uint8_t byte;

	HAL_SPI_Receive(&hspi1, &byte, 1, 5000);
	return byte;
}


void SPI_flash_send_byte(uint8_t byte){

	HAL_SPI_Transmit(&hspi1, &byte, 1, 5000);

}

void SPI_flash_select(void){
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
}

void SPI_flash_deselect(void){
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
}

void jumpToApp(const uint32_t address)
{

	 typedef void(*pFunction)(void);//объявляем пользовательский тип
	 /* Устанавливаем адрес перехода на основную программу */
	 /* Переход производится выполнением функции, адрес которой указывается вручную */
	 /* +4 байта потому, что в самом начале расположен указатель на вектор прерывания */
	 uint32_t jumpAddress = *(__IO uint32_t*) (MAIN_PROGRAM_START_ADDRESS + 4);
	 pFunction Jump_To_Application = (pFunction) jumpAddress;

	 /*Сбрасываем всю периферию на APB1 */
	 RCC->APB1RSTR = 0xFFFFFFFF; RCC->APB1RSTR = 0x0;
	/*Сбрасываем всю периферию на APB2 */
	 RCC->APB2RSTR = 0xFFFFFFFF; RCC->APB2RSTR = 0x0;
	 RCC->APB1ENR = 0x0; /* Выключаем всю периферию на APB1 */
	 RCC->APB2ENR = 0x0; /* Выключаем всю периферию на APB2 */
	 //RCC->AHBENR = 0x0; /* Выключаем всю периферию на AHB */
	 /* Сбрасываем все источники тактования по умолчанию, переходим на HSI*/
	 //RCC_DeInit();
	 HAL_DeInit();

	 /* Выключаем прерывания */
	 __disable_irq();
	 /* Переносим адрес вектора прерываний */
	 //NVIC_SetVectorTable(NVIC_VectTab_FLASH, MAIN_PROGRAM_START_ADDRESS);
	 //NVIC_SetVector((uint32_t)0x08000000, MAIN_PROGRAM_START_ADDRESS);
	 SCB->VTOR = (uint32_t)0x08000000 | (MAIN_PROGRAM_START_ADDRESS & (uint32_t)0x1FFFFF80);
	 //0x08000000 VECT_TAB_OFFSET
	 /* Переносим адрес стэка */
	  __set_MSP(*(__IO uint32_t*) MAIN_PROGRAM_START_ADDRESS);
	  /* Переходим в основную программу */
	  Jump_To_Application();
	/*
	const JumpStruct* vector_p = (JumpStruct*)address;

	deinitEverything();

	// let's do The Jump!
    // Jump, used asm to avoid stack optimization
    asm("msr msp, %0; bx %1;" : : "r"(vector_p->stack_addr), "r"(vector_p->func_p));
   */
}

void delay(uint32_t ms){

	ms = ms * 8000;
	while(ms){
		ms--;
	}
}

int FLASH_EraseSector(uint32_t FLASH_Sector, uint8_t VoltageRange)
{
  uint32_t tmp_psize = 0x0;
  int status = 9; //FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FLASH_SECTOR(FLASH_Sector));
  assert_param(IS_VOLTAGERANGE(VoltageRange));

  if(VoltageRange == VoltageRange_1)
  {
     tmp_psize = FLASH_PSIZE_BYTE;
  }
  else if(VoltageRange == VoltageRange_2)
  {
    tmp_psize = FLASH_PSIZE_HALF_WORD;
  }
  else if(VoltageRange == VoltageRange_3)
  {
    tmp_psize = FLASH_PSIZE_WORD;
  }
  else
  {
    tmp_psize = FLASH_PSIZE_DOUBLE_WORD;
  }
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(HAL_MAX_DELAY);

  if(status == HAL_OK)
  {
    /* if the previous operation is completed, proceed to erase the sector */
    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= tmp_psize;
    FLASH->CR &= SECTOR_MASK;
    FLASH->CR |= FLASH_CR_SER | FLASH_Sector;
    FLASH->CR |= FLASH_CR_STRT;

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(HAL_MAX_DELAY);

    /* if the erase operation is completed, disable the SER Bit */
    FLASH->CR &= (~FLASH_CR_SER);
    FLASH->CR &= SECTOR_MASK;
  }
  /* Return the Erase Status */
  return status;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t 						lfsReadBuff[256];
	uint8_t 						lfsWriteBuff[256];
	uint8_t 						lfslookaheadBuff[256];
	uint8_t							lfs_file_buf[512];
	struct SPI_flash_info			flash_parameters;
	int 							err;
	uint32_t 						addr = 0;
	uint8_t 						res,data = 0;

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  SPI_flash_reg_cb(SPI_flash_select, SPI_flash_deselect, SPI_flash_read_byte, SPI_flash_send_byte);
  flash_parameters = sFLASH_GetInfo();
  if (flash_parameters.flash_id == 0x0) jumpToApp(APP_START); // jump to APP

  io_fs_init(lfsReadBuff, lfsWriteBuff, lfslookaheadBuff, 256, &cfg);
	memset(&fileConf, 0, sizeof(struct lfs_file_config));
	fileConf.buffer = lfs_file_buf;  						// use the static buffer
	fileConf.attr_count = 0;

	err = lfs_mount(&lfs, &cfg);

	if(err < 0){
	    jumpToApp(APP_START);
	}

	err = lfs_file_opencfg(&lfs, &file, FIRMWARE_FILE, LFS_O_RDONLY, &fileConf);

	if (err >= 0){
		HAL_FLASH_Unlock();
		FLASH_EraseSector(FLASH_Sector_5,VoltageRange_3);
		FLASH_EraseSector(FLASH_Sector_4,VoltageRange_3);
		FLASH_EraseSector(FLASH_Sector_3,VoltageRange_3);

		lfs_file_rewind(&lfs,&file);

		for ( addr = APP_START ; addr < APP_END ; addr = addr + sizeof(data) ){
			res = lfs_file_read(&lfs, &file, &data, sizeof(data));
			if (res <= 0 ) break; //из файла считалось 0 байт или ошибка, выходим
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr,data);
		}

	    lfs_file_close(&lfs, &file);

	    lfs_remove(&lfs, FIRMWARE_FILE);

	    HAL_FLASH_Lock();
	    jumpToApp(APP_START);

	}else{
		jumpToApp(APP_START);
	}
  /* USER CODE END 2 */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : FLASH_CS_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
