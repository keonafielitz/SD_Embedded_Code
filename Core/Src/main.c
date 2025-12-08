/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "fatfs.h"
#include "i2c.h"
#include "i2s.h"
#include "sdio.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    uint16_t format;
    uint16_t channels;
    uint32_t frequency;
    uint32_t bytes_per_sec;
    uint16_t bytes_per_block;
    uint16_t bits_per_sample;
} fmt_typedef;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SAMPLE_FREQ 96000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//uint8_t TxBuffer[] = "Hello World! From STM32 USB CDC Device To Virtual COM Port\r\n";
//uint8_t TxBufferLen = sizeof(TxBuffer);
//uint8_t ret;

//sample buffer
int16_t data_i2s[100];
volatile int16_t sample_i2s;

const char total_uptime_filename[] = "myfile.dat";

float amplifier = 0.5;

int8_t usart_rx_buffer[16];

int16_t *do_buffer;

uint32_t buffers_done;
uint32_t total_uptime;

uint8_t open_next_file = 1;

FRESULT res;
DIR dir;
FIL music_file;
FILINFO music_file_info;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_I2S2_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_I2S_Receive_DMA(&hi2s2, (uint16_t*) data_i2s, sizeof(data_i2s)/2);

  uint32_t wbytes, rbytes;


  FRESULT fr;
  UINT bytes;

  HAL_Delay(20);

  fr = f_mount(&SDFatFS, SDPath, 1);
  if (fr != FR_OK) {
      printf("Unable to mount disk: %d\n", fr);
      Error_Handler();
  }

  fr = f_open(&SDFile, total_uptime_filename, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
  if (fr != FR_OK) {
      printf("Unable to open/create file: %d\n", fr);
      Error_Handler();
  }

  // Read existing uptime if the file has data
  fr = f_read(&SDFile, &total_uptime, sizeof(total_uptime), &bytes);
  if (fr != FR_OK || bytes != sizeof(total_uptime)) {
      total_uptime = 0;  // File empty or read failed
  }

  // Move file pointer to beginning before writing
  f_lseek(&SDFile, 0);

  // Write uptime back
  fr = f_write(&SDFile, &total_uptime, sizeof(total_uptime), &bytes);
  if (fr != FR_OK || bytes != sizeof(total_uptime)) {
      printf("Unable to write: %d\n", fr);
      Error_Handler();
  }

  f_close(&SDFile);

//  printf("SD Card Information");
//
//  if (f_mount(&SDFatFS, (TCHAR const*) SDPath, 0) != FR_OK) {
//          printf("Unable to mount disk\n");
//          Error_Handler();
//  }
//
//  HAL_Delay(20);
//
//  if (f_open(&SDFile, total_uptime_filename, FA_OPEN_ALWAYS | FA_READ) == FR_OK) {
//      if (f_read(&SDFile, &total_uptime, sizeof(total_uptime), (void*) &rbytes) == FR_OK) {
//          printf("Total uptime = %lu\n", total_uptime);
//          f_close(&SDFile);
//      } else {
//          printf("Unable to read\n");
//          Error_Handler();
//      }
//  } else {
//      // File did not exist - let's create it
//      if (f_open(&SDFile, total_uptime_filename, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
//          if (f_write(&SDFile, &total_uptime, sizeof(total_uptime), (void*) &wbytes) == FR_OK) {
//              printf("File %s created\n", total_uptime_filename);
//              f_close(&SDFile);
//          } else {
//              printf("Unable to write\n");
//              Error_Handler();
//          }
//      } else {
//    	  FRESULT fr = f_open(&SDFile, total_uptime_filename, FA_CREATE_ALWAYS | FA_WRITE);
//          printf("Unable to create\n");
//          Error_Handler();
//      }
//  }

  uint32_t now = 0, next_blink = 500, next_tick = 1000, loop_cnt = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


      now = uwTick;
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);

      if (now >= next_blink) {

          HAL_GPIO_TogglePin(GPIOB, LED_Pin);

          next_blink = now + 500;
      }

      if (now >= next_tick) {

          //printf("Tick %lu (loop=%lu bd=%lu)\n", now / 1000, loop_cnt, buffers_done);

          if (((now / 1000) % 60) == 0) {

              ++total_uptime;

              printf("Updating total uptime to %lu minutes\n", total_uptime);

              // Update the total uptime file
              if (f_open(&SDFile, total_uptime_filename, FA_OPEN_EXISTING | FA_WRITE) == FR_OK) {
                  if (f_write(&SDFile, &total_uptime, sizeof(total_uptime), (void*) &wbytes) != FR_OK) {
                      printf("Unable to write\n");
                  }
                  f_close(&SDFile);
              } else {
                  printf("Unable to open file\n");
              }

          }

          open_next_file = 0;

          loop_cnt = 0;
          next_tick = now + 1000;

      }

      //this is a bunch of stuff from the guys example
//      if (open_next_file) {
//
//          f_close(&music_file);
//
//          // Experimental advance to next
//          res = f_findnext(&dir, &music_file_info);
//
//          if (res != FR_OK || music_file_info.fname[0] == '\0') { // If we're out of files start again
//              res = f_findfirst(&dir, &music_file_info, "", "*.wav");
//          }
//
//          printf("Next file: %s\n", music_file_info.fname);
//
//          if (f_open(&music_file, music_file_info.fname, FA_READ) != FR_OK) {
//              printf("Unable to open %s\n", music_file_info.fname);
//          }
//
//          fmt_typedef wav_format;
//
//          if (parse_wav_header(&music_file, &wav_format) != FR_OK) {
//              printf("Unable to parse header\n");
//              open_next_file = 1;
//          } else {
//
//              printf("Wav format: %d\n", wav_format.format);
//              printf("Wav channels: %d\n", wav_format.channels);
//              printf("Wav frequency: %lu\n", wav_format.frequency);
//              printf("Wav bytes per sec: %lu\n", wav_format.bytes_per_sec);
//              printf("Wav bytes per block: %d\n", wav_format.bytes_per_block);
//              printf("Wav bits per sample: %d\n", wav_format.bits_per_sample);
//
//              set_i2s_freq(wav_format.frequency);
//
//              open_next_file = 0;
//          }
//
//      }
      //i think this code is also music related
//
//      if (do_buffer) {
//
//          process_buffer(do_buffer);
//
//          ++buffers_done;
//          do_buffer = 0;
//      }

      ++loop_cnt;

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

int _write(int file, char *ptr, int len){
	int DataIdx;

	for(DataIdx = 0; DataIdx < len; DataIdx++){
		ITM_SendChar(*ptr++);
	}
	return len;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s){

	sample_i2s = data_i2s[1];
}



//
//void process_buffer(int16_t *target_buffer) {
//
//    int16_t buf[2 * I2S_DMA_BUFFER_SAMPLES] = { 0 };
//    unsigned int bytes_read = 0;
//
//    if (f_read(&music_file, &buf, sizeof(buf), &bytes_read) == FR_OK) {
//
//        for (int i = 0; i < 2 * I2S_DMA_BUFFER_SAMPLES; ++i) {
//            buf[i] = buf[i] * amplifier;
//        }
//
//        memcpy(target_buffer, buf, sizeof(buf));
//
//        if (bytes_read < sizeof(buf)) {
//
//            printf("File done!\n");
//
//            set_i2s_freq(0);
//
//            open_next_file = 1;
//        }
//
//    }
//
//}

void print_fresult(FRESULT fr) {
    switch (fr) {
    case FR_OK:            printf("FR_OK\n"); break;
    case FR_DISK_ERR:      printf("FR_DISK_ERR (1): Low-level read/write failure\n"); break;
    case FR_INT_ERR:       printf("FR_INT_ERR (2): Assertion/internal consistency error\n"); break;
    case FR_NOT_READY:     printf("FR_NOT_READY (3): Disk not ready\n"); break;
    case FR_NO_FILE:       printf("FR_NO_FILE (4)\n"); break;
    case FR_NO_PATH:       printf("FR_NO_PATH (5)\n"); break;
    case FR_INVALID_NAME:  printf("FR_INVALID_NAME (6)\n"); break;
    case FR_DENIED:        printf("FR_DENIED (7)\n"); break;
    case FR_EXIST:         printf("FR_EXIST (8)\n"); break;
    case FR_INVALID_OBJECT:printf("FR_INVALID_OBJECT (9)\n"); break;
    case FR_WRITE_PROTECTED: printf("FR_WRITE_PROTECTED (10)\n"); break;
    case FR_INVALID_DRIVE: printf("FR_INVALID_DRIVE (11)\n"); break;
    case FR_NOT_ENABLED:   printf("FR_NOT_ENABLED (12)\n"); break;
    case FR_NO_FILESYSTEM: printf("FR_NO_FILESYSTEM (13)\n"); break;
    case FR_MKFS_ABORTED:  printf("FR_MKFS_ABORTED (14)\n"); break;
    case FR_TIMEOUT:       printf("FR_TIMEOUT (15)\n"); break;
    case FR_LOCKED:        printf("FR_LOCKED (16)\n"); break;
    case FR_NOT_ENOUGH_CORE: printf("FR_NOT_ENOUGH_CORE (17)\n"); break;
    case FR_TOO_MANY_OPEN_FILES: printf("FR_TOO_MANY_OPEN_FILES (18)\n"); break;
    case FR_INVALID_PARAMETER: printf("FR_INVALID_PARAMETER (19)\n"); break;
    default: printf("Unknown error: %d\n", fr);
    }
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
