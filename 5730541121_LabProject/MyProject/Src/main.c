/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
// Accelerometer val
uint8_t address;
int acx,acy,acz;
int left = 0,right = 0,up = 0,down = 0;
//Mic Val
uint16_t buffer[20];
uint16_t pdm=0;
uint8_t  pcm=0;
int it,count = 0;
float pcm_buffer = 0.0;
float amp = 0.0;
float max_amp = 0.0;
//Speaker Val
uint8_t startval[2];
uint16_t Istr[1];
//Logic val
int curX,curY,stX = 0,stY = 0,edX,edY;
int isEnd = 0;
char grid[5][5];
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  // Init Accelero
  curX = 0; curY = 0;
  edX = rand()%3+2;
  edY = rand()%3+2;
  initAc();

  initSpeaker();
  updateScreen();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  isEnd = 0;
	  getXYZ();
	  //micUpdate();
	  if(left == 0 && right ==0 && up ==0 && down ==0){
		  HAL_Delay(200);
		  continue;
	  }
	  if(move() == 0) {
		  startval[0] = 0x1E;
		  startval[1] = 0x20;
		  HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);
		  // C
		  startval[0] = 0x1C;
		  startval[1] = 0x1F;
		  HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);

		  startval[0] = 0x1E;
		  startval[1] = 0xE0;
		  HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);
		  for (int k=0;k<1000;k++) {
		  	  HAL_I2S_Transmit (&hi2s3, Istr , 0x10, 10 );
		  }
	  }
	  else {
		  startval[0] = 0x1E;
		  startval[1] = 0x20;
		  HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);
		  // F
		  startval[0] = 0x1C;
		  startval[1] = 0x4F;
		  HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);

		  startval[0] = 0x1E;
		  startval[1] = 0xE0;
		  HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);
		  for (int k=0;k<1000;k++) {
			  //HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
		  	  HAL_I2S_Transmit (&hi2s3, Istr , 0x10, 10 );
		  }
		  updateScreen();
	  }
	  if(isReached() == 1){
		  char str[10] = "Arrived!!";
		  HAL_UART_Transmit(&huart2,"\033c",4,1000);
		  HAL_UART_Transmit(&huart2,&str,9,1000);
		  playMusic();
	  }
	  while(isReached() == 1){
		  count = 0;
		  micUpdate();
		  if(isSpeak() == 1) {
			  isEnd = 1;
			  break;
		  }
	  }
	  if(isEnd == 1){
		  //while(isSpeak() == 0) count = 0,micUpdate();
		  curX = 0;
		  curY = 0;
		  isEnd = 0;
		  edX = rand()%3+2;
		  edY = rand()%3+2;
		  updateScreen();
	  }

	  HAL_Delay(200);
  }
  /* USER CODE END 3 */

}
void initAc(){
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	address = 0x20;
	HAL_SPI_Transmit(&hspi1,&address,1,50);

	uint8_t data = 0x67;
	HAL_SPI_Transmit(&hspi1,&data,1,50);

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
}
void initSpeaker(){
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1);

    startval[0] = 0x47;
    startval[1] = 0x80;
    HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);

    startval[0] = 0x32;
    startval[1] = 0x80;
    HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);

    startval[0] = 0x32;
    startval[1] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);

    startval[0] = 0x1C;
    startval[1] = 0xAF;
    HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);

    startval[0] = 0x1E;
    startval[1] = 0xE0;
    HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);

    startval[0] = 0x02;
    startval[1] = 0x9E;
    HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);
}
void playMusic(){
	startval[0] = 0x1E;
	startval[1] = 0x20;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);
	// C
	startval[0] = 0x1C;
	startval[1] = 0x1F;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);

	startval[0] = 0x1E;
	startval[1] = 0xE0;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);
	for (int k=0;k<1000;k++) {
		  HAL_I2S_Transmit (&hi2s3, Istr , 0x10, 10 );
	}
	HAL_Delay(50);
	startval[0] = 0x1E;
	startval[1] = 0x20;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);
	// C
	startval[0] = 0x1C;
	startval[1] = 0x2F;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);

	startval[0] = 0x1E;
	startval[1] = 0xE0;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);
	for (int k=0;k<1000;k++) {
		  HAL_I2S_Transmit (&hi2s3, Istr , 0x10, 10 );
	}
	HAL_Delay(50);
	startval[0] = 0x1E;
	startval[1] = 0x20;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);
	// C
	startval[0] = 0x1C;
	startval[1] = 0x3F;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);

	startval[0] = 0x1E;
	startval[1] = 0xE0;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);
	for (int k=0;k<1000;k++) {
		  HAL_I2S_Transmit (&hi2s3, Istr , 0x10, 10 );
	}
	HAL_Delay(50);
	startval[0] = 0x1E;
	startval[1] = 0x20;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);
	// C
	startval[0] = 0x1C;
	startval[1] = 0x4F;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);

	startval[0] = 0x1E;
	startval[1] = 0xE0;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, startval, 2, 50);
	for (int k=0;k<1000;k++) {
		  HAL_I2S_Transmit (&hi2s3, Istr , 0x10, 10 );
	}
}
void updateScreen(){
	int i=0,j=0;
	for( i=0;i<5;i++ ){
		for( j=0;j<5;j++ ){
			grid[i][j] = '-';
		}
	}
	HAL_UART_Transmit(&huart2,"\033c",4,1000);
	grid[edY][edX] = 'T';
	grid[curY][curX] = 'P';
	HAL_UART_Transmit(&huart2,&grid[0],5,1000); HAL_UART_Transmit(&huart2,"\n\r",2,1000);
	HAL_UART_Transmit(&huart2,&grid[1],5,1000); HAL_UART_Transmit(&huart2,"\n\r",2,1000);
	HAL_UART_Transmit(&huart2,&grid[2],5,1000); HAL_UART_Transmit(&huart2,"\n\r",2,1000);
	HAL_UART_Transmit(&huart2,&grid[3],5,1000); HAL_UART_Transmit(&huart2,"\n\r",2,1000);
	HAL_UART_Transmit(&huart2,&grid[4],5,1000); HAL_UART_Transmit(&huart2,"\n\r",2,1000);
}
//Accelerometer FUnction
void getXYZ(){
	uint8_t temp;
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	address = 0x29+ 0x80;		//x
	HAL_SPI_Transmit(&hspi1,&address,1,50);
	HAL_SPI_Receive(&hspi1,&temp,1,50);
	acx=temp; // cast from uint -> int

	address = 0x2B+ 0x80;		//y
	HAL_SPI_Transmit(&hspi1,&address,1,50);
	HAL_SPI_Receive(&hspi1,&temp,1,50);
	acy=temp;

	address = 0x2D+ 0x80;		//z
	HAL_SPI_Transmit(&hspi1,&address,1,50);
	HAL_SPI_Receive(&hspi1,&temp,1,50);
	acz=temp;
	left = right = up = down = 0;
 	 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
 	 //up
 	 if((acy>=15&&acy<=60)&&(acx<15||acx>240))
 		 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET),up = 1;
 	 else
 		 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
 	 //down
 	 if((acy<=240&&acy>=200)&&(acx<15||acx>240))
 		 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET),down = 1;
 	 else
 		 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
 	 //right
 	 if((acx<=240&&acx>=200)&&(acy<15||acy>240))
 		 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET),right = 1;
 	 else
 		 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
 	 //left
 	 if((acx>=20&&acx<=60)&&(acy<15||acy>240))
 		 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET),left = 1;
 	 else
 		 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);

}
int isReached(){
	if(curX == edX && curY == edY) return 1;
	return 0;
}
int move(){
	if(right == 1){
		if(curX == 0) return 0;
		else curX--;
	}
	else if(down == 1){
		if(curY == 4) return 0;
		else curY++;
	}
	else if(up == 1){
		if(curY == 0) return 0;
		else curY--;
	}
	else if(left == 1){
		if(curX == 4) return 0;
		else curX++;
	}
	return 1;
}
// Microphone
int isSpeak(){
	if(amp>50000.0) return 1;
	return 0;
}
float absfunc(float x){
	if (x < 0) return -1*x;
	else return x;
}
void micUpdate(){
	  HAL_I2S_Receive(&hi2s2, buffer, 20, 1000);
	  amp = 0;
	  for(it=0; it<20; it++){
	        pcm = -8;
	        pdm = buffer[it];
	        // Count high bit
	        while ( pdm != 0 ) {
	          pcm ++;
	          pdm ^= pdm & -pdm;
	        }
	        pcm_buffer += pcm;
	        pcm_buffer *= 0.95;
	        amp += absfunc(pcm_buffer);
	        amp *= 0.95;
	  }
	      //count++;
//	      if(max_amp <amp) {
//	    	  max_amp = amp;
//	      }
//	      if(count == 2000){
//	      	max_amp = 0;
//	      	count = 0;
//	      }
}
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2S2 init function */
static void MX_I2S2_Init(void)
{

  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2S3 init function */
static void MX_I2S3_Init(void)
{

  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

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

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1680;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
