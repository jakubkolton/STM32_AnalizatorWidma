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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SSD1306_OLED.h"
#include "GFX_BW.h"
#include "fonts/fonts.h"
#include "my_face.h"
#include "bmp280.h"
#include "stdio.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FFT_SAMPLES 1024
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP280_ADDRESS 0x76
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
BMP280_t Bmp280; // struktura czujnika

uint32_t SoftTimerBmp;
uint32_t SoftTimerOled;

float Temp, Pressure; // temperatura, cisnienie

char Message[32]; // bufor na wiadomosc

uint16_t AdcMicrophone[FFT_SAMPLES];
float FFTInBuffer[FFT_SAMPLES]; // bufor we FFT
float FFTOutBuffer[FFT_SAMPLES]; // bufor wy FFT
arm_rfft_fast_instance_f32 FFTHandler; // struktura uchwytu do FFT
volatile uint8_t SamplesReady; // flaga - czy próbki są gotowe do liczenia FFT (np 1024 probki)
uint8_t OutFreqArray[10]; // tablica wyjsciowa na 10 prazkow FFT


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void CalculateFFT(void);
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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  BMP280_Init(&Bmp280, &hi2c1, BMP280_ADDRESS); // inicjacja struktury czujnika

  SSD1306_Init(&hi2c1);

  GFX_SetFont(font_8x5);

  SSD1306_Clear(BLACK);

  SSD1306_Display();

  HAL_TIM_Base_Start(&htim2); // startuje TIM2 (dla kanaLu Regular)
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)AdcMicrophone, FFT_SAMPLES); // start ADC w trybie Regular - zbierze 1024 probki

  arm_rfft_fast_init_f32(&FFTHandler, FFT_SAMPLES); // inicjacja FFT w strukturze

  SoftTimerBmp = HAL_GetTick();
  SoftTimerOled = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // FFT - samo obliczenie
	  if (SamplesReady == 1) // czy gotowy jest zestaw probek
	  {
		  SamplesReady = 0; // opuszczenie flagi

		  // przeniesienie probek z ADC do bufora wej FFT
		  for (uint32_t i = 0; i < FFT_SAMPLES; i++)
		  {
			  FFTInBuffer[i] = (float)AdcMicrophone[i];
		  }

		  CalculateFFT();// przeliczenie FFT
	  }


	  // obsluga czujnika
	  if (HAL_GetTick() - SoftTimerBmp > 10) // po 10 ms
	  {
		  SoftTimerBmp = HAL_GetTick(); // zmiana punktu odniesienia; przekarmienie timera

		  if (Bmp280.bmp_i2c->State == HAL_I2C_STATE_READY) // czy I2C nie jest zajete
		  {
			  BMP280_ReadPressureandTemperature(&Bmp280, &Pressure, &Temp); // odczyt pomiarow
		  }
	  }

	  // obsluga ekranu
	  if (HAL_GetTick() - SoftTimerOled > 100) // po 100 ms
	  {
		  SoftTimerOled = HAL_GetTick(); // zmiana punktu odniesienia; przekarmienie timera

		  SSD1306_Clear(BLACK);

		  // przekazanie cisnienia do bufora
		  sprintf(Message, "Press: %.2f hPa", Pressure);
		  GFX_DrawString(0,0, Message, WHITE, 0);

		  // przekazanie temperatury do bufora
		  sprintf(Message, "Temp: %.2f *C", Temp);
		  GFX_DrawString(0,10, Message, WHITE, 0);

		  // przekazanie FFT do bufora
		  for (uint8_t i = 0; i < 10; i++) // 10 prazkow FFT
		  {
			  // Wskaznik po ekranie idzie od lewego gornego rogu, stad obliczenia:
			  // slupki zaczynaja sie co 11 pikseli, na wysokosci 64-OutFreqArray[i],
			  // maja szerokosc 10, wysokosc OutFreqArray[i]
			  GFX_DrawFillRectangle(10+(i*11), 64-OutFreqArray[i], 10, OutFreqArray[i], WHITE);
		  }

		  SSD1306_Display(); // wyswietlenie
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* I2C1_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
}

/* USER CODE BEGIN 4 */

// callback przerwania z ADC
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		SamplesReady = 1; // ustawia flage
	}
}


// funckja liczaca modul liczby zespolonej
float complexABS(float real, float compl)
{
	return sqrtf(real*real+compl*compl);
}

// funkcja liczaca FFT
void CalculateFFT(void)
{
	// wykonuje FFT zwracajac liczby zespolone
	arm_rfft_fast_f32(&FFTHandler, FFTInBuffer, FFTOutBuffer, 0);

	int Freqs[FFT_SAMPLES]; // jest tyle przedzialow czestotliwosci co probek
	int FreqPoint = 0; // zmienna pomocnicza
	int Offset = 40; // offset do eliminacji (uciecia) domniemanych szumow podczas ciszy

	// liczenie modulu i przejscie na miare decybelowa
	for (int i = 0; i < FFT_SAMPLES; i = i+2)
	{
		// wyliczenie prazka i uciecie domniemanego szumu (-Offset)
		Freqs[FreqPoint] = (int)(20*log10f(complexABS(FFTOutBuffer[i], FFTOutBuffer[i+1]))) - Offset;

		if (Freqs[FreqPoint] < 0) // jesli slupek jest ujemny - domniemany szum wiekszy od sygnalu
		{
			Freqs[FreqPoint] = 0;
		}
		FreqPoint++;
	}

	// przerzucenie wybranych prazkow FFT do tablicy
	OutFreqArray[0] = (uint8_t)Freqs[1]; // 22 Hz
	OutFreqArray[1] = (uint8_t)Freqs[2]; // 63 Hz
	OutFreqArray[2] = (uint8_t)Freqs[3]; // 125 Hz
	OutFreqArray[3] = (uint8_t)Freqs[6]; // 250 Hz
	OutFreqArray[4] = (uint8_t)Freqs[12]; // 500 Hz
	OutFreqArray[5] = (uint8_t)Freqs[23]; // 1000 Hz
	OutFreqArray[6] = (uint8_t)Freqs[51]; // 2200 Hz
	OutFreqArray[7] = (uint8_t)Freqs[104]; // 4500 Hz
	OutFreqArray[8] = (uint8_t)Freqs[207]; // 9000 Hz
	OutFreqArray[9] = (uint8_t)Freqs[344]; // 15000 Hz

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
