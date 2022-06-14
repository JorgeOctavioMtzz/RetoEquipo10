/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R1_PORT GPIOA  // ROW 1 PORT
#define R1_PIN GPIO_PIN_7  // ROW 1 PIN

#define R2_PORT GPIOA  // ROW 2 PORT
#define R2_PIN GPIO_PIN_6  // ROW 2 PIN

#define R3_PORT GPIOA  // ROW 3 PORT
#define R3_PIN GPIO_PIN_5  // ROW 3 PIN

#define R4_PORT GPIOA  // ROW 4 PORT
#define R4_PIN GPIO_PIN_4  // ROW 4 PIN

#define C1_PORT GPIOA  // ROW 5 PORT
#define C1_PIN GPIO_PIN_3  // ROW 5 PIN

#define C2_PORT GPIOA  // ROW 6 PORT
#define C2_PIN GPIO_PIN_2  // ROW 6 PIN

#define C3_PORT GPIOA  // ROW 7 PORT
#define C3_PIN GPIO_PIN_1  // ROW 7 PIN

#define C4_PORT GPIOA  // ROW 8 PORT
#define C4_PIN GPIO_PIN_0  // ROW 8 PIN

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void USER_RCC_Init(void);
void USER_GPIO_Init(void);
void USER_USART_Init(void);
void USER_USART1_Transmit(uint8_t *pData, uint16_t size);
uint8_t USER_USART1_Receive(void);
char READ_KEYPAD(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t key;
uint8_t keyB;
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
  /* USER CODE BEGIN 2 */

  USER_RCC_Init(); // I/0 PORT C CLOCK ENABLE
  //PC13-User LED as output push pull max speed 10 MHz
  USER_GPIO_Init();
  LCD_Init( );//				inicializamos la libreria del LCD
  LCD_Cursor_ON( );//			cursor visible activo
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  key = '0';
	  while(key != '#'){
		 key = READ_KEYPAD();
		 USER_USART1_Transmit(key, sizeof(key)); // enviar dato del keypad a la terminal
		 HAL_Delay(10);  // delay
	  }
	  keyB = 'i';
	  while(keyB != 'e'){
		  LCD_Clear( );//borra la pantalla
		  LCD_Set_Cursor( 1, 0 );//posiciona cursor en la fila 1 columna 0
		  LCD_Put_Str( "KEYBOARD:" );//escribe un string
		  keyB = USER_USART1_Receive();
		  LCD_Put_Char(keyB);
		  HAL_Delay(10);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void USER_RCC_Init(void){
	  RCC->APB2ENR |= RCC_APB2ENR_USART1EN //Enable the clock for USART1
	  	  	  	   |  RCC_APB2ENR_IOPCEN // I/0 PORT C CLOCK ENABLE}
				   |  RCC_APB2ENR_IOPAEN; // I/0 PORT A CLOCK ENABLE}
}

void USER_GPIO_Init(void){
	//pin PA9 USART1_TX AS OUTPUT PUSH PULL MAX SPEED 10MHZ
	GPIOA->CRH &=  ~GPIO_CRH_CNF9_0 & ~GPIO_CRH_MODE9_1;
	GPIOA->CRH |=  GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_0;

	//pin PA10 USART1_RX AS INPUT FLOATING
	GPIOA->CRH &=  ~GPIO_CRH_CNF10_1 & ~GPIO_CRH_MODE10;
	GPIOA->CRH |=  GPIO_CRH_CNF10_0;

	GPIOC->BSRR = GPIO_BSRR_BS13; // led OFF
	//PIN PC13 AS OUTPUT PUSH PULL, MAX SPEED 10 MHZ
	GPIOC->CRH &= ~GPIO_CRH_CNF13 & ~GPIO_CRH_MODE13_1;
	GPIOC->CRH |= GPIO_CRH_MODE13_0;
}

void USER_USART_Init(void){
	USART1->BRR = 0X1D4C; //9600 BPS
	USART1->CR1 &= ~USART_CR1_M // 1 START BIT, 8 DATA BITS
				&  ~USART_CR1_WAKE// IDLE LINE
				&  ~USART_CR1_PCE//  PARITY CONTROL DISABLED
				&  ~USART_CR1_TXEIE//  INTERRUPT DISABLED (EMPTY TRANSMIT REG)
				&  ~USART_CR1_TCIE//  INTERRUPT DISABLED (TRANSMISSION COMPLETE)
				&  ~USART_CR1_RXNEIE//  INTERRUPT DISABLED (DATA RECEIVED)
				&  ~USART_CR1_IDLEIE// INTERRUPT DISABLED (IDLE LINE DETECTED)
				&  ~USART_CR1_RWU//  RECEIVER ACTIVE MODE
				&  ~USART_CR1_SBK;// NO BREAK CHARACTER TRANSMITTED
	USART1->CR1 |= USART_CR1_UE //USART ENABLED
				|  USART_CR1_TE// TRANSMITTER ENABLED
				|  USART_CR1_RE;//RECEIVER ENABLED
	USART1->CR2 &= ~USART_CR2_STOP; // 1 STOP BIT
}

void USER_USART1_Transmit(uint8_t *pData, uint16_t size){
	for(int i=0; i<size; i++){
		while((USART1->SR & USART_SR_TXE)==0){  // Wait until transmit reg is empty
			// Empty While
		}
		USART1->DR = *pData++; // Transmit Data
	}
}

uint8_t USER_USART1_Receive(void){
	while((USART1->SR & USART_SR_RXNE)==0){ // WAIT UNTIL DATA IS RECEIVED
		// EMPTY WHILE
	}
	return USART1->DR;// RETURN THE RECEIVED DATA
}

char READ_KEYPAD(void)
{
	HAL_GPIO_WritePin(R1_PORT, R1_PIN, GPIO_PIN_RESET); //R1 low
	HAL_GPIO_WritePin(R2_PORT, R2_PIN, GPIO_PIN_SET); //R2 high
	HAL_GPIO_WritePin(R3_PORT, R3_PIN, GPIO_PIN_SET); //R3 high
	HAL_GPIO_WritePin(R4_PORT, R4_PIN, GPIO_PIN_SET); //R4 high

	if (!(HAL_GPIO_ReadPin(C1_PORT, C1_PIN))){
		while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));
		return '1';
	}

	if (!(HAL_GPIO_ReadPin(C2_PORT, C2_PIN))){
		while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));
		return '2';
	}

	if (!(HAL_GPIO_ReadPin(C3_PORT, C3_PIN))){
		while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));
		return '3';
	}

	if (!(HAL_GPIO_ReadPin(C4_PORT, C4_PIN))){
		while (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)));
		return 'A';
	}

	HAL_GPIO_WritePin(R1_PORT, R1_PIN, GPIO_PIN_SET); //R1 high
	HAL_GPIO_WritePin(R2_PORT, R2_PIN, GPIO_PIN_RESET); //R2 low
	HAL_GPIO_WritePin(R3_PORT, R3_PIN, GPIO_PIN_SET); //R3 high
	HAL_GPIO_WritePin(R4_PORT, R4_PIN, GPIO_PIN_SET); //R4 high

	if (!(HAL_GPIO_ReadPin(C1_PORT, C1_PIN))){
			while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));
			return '4';
	}

	if (!(HAL_GPIO_ReadPin(C2_PORT, C2_PIN))){
		while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));
		return '5';
	}

	if (!(HAL_GPIO_ReadPin(C3_PORT, C3_PIN))){
		while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));
		return '6';
	}

	if (!(HAL_GPIO_ReadPin(C4_PORT, C4_PIN))){
		while (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)));
		return 'B';
	}

	HAL_GPIO_WritePin(R1_PORT, R1_PIN, GPIO_PIN_SET); //R1 high
	HAL_GPIO_WritePin(R2_PORT, R2_PIN, GPIO_PIN_SET); //R2 high
	HAL_GPIO_WritePin(R3_PORT, R3_PIN, GPIO_PIN_RESET); //R3 low
	HAL_GPIO_WritePin(R4_PORT, R4_PIN, GPIO_PIN_SET); //R4 high

	if (!(HAL_GPIO_ReadPin(C1_PORT, C1_PIN))){
		while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));
		return '7';
	}

	if (!(HAL_GPIO_ReadPin(C2_PORT, C2_PIN))){
		while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));
		return '8';
	}

	if (!(HAL_GPIO_ReadPin(C3_PORT, C3_PIN))){
		while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));
		return '9';
	}

	if (!(HAL_GPIO_ReadPin(C4_PORT, C4_PIN))){
		while (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)));
		return 'C';
	}

	HAL_GPIO_WritePin(R1_PORT, R1_PIN, GPIO_PIN_SET); //R1 high
	HAL_GPIO_WritePin(R2_PORT, R2_PIN, GPIO_PIN_SET); //R2 high
	HAL_GPIO_WritePin(R3_PORT, R3_PIN, GPIO_PIN_SET); //R3 high
	HAL_GPIO_WritePin(R4_PORT, R4_PIN, GPIO_PIN_RESET); //R4 low

	if (!(HAL_GPIO_ReadPin(C1_PORT, C1_PIN))){
		while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));
		return '\n';
	}

	if (!(HAL_GPIO_ReadPin(C2_PORT, C2_PIN))){
		while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));
		return '0';
	}

	if (!(HAL_GPIO_ReadPin(C3_PORT, C3_PIN))){
		while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));
		return '#';
	}

	if (!(HAL_GPIO_ReadPin(C4_PORT, C4_PIN))){
		while (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)));
		return 'D';
	}
	return '0';
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
