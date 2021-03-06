
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//  // Registers accselerometr.
uint8_t Device_ID=0xE5;
uint8_t Device_addres=0x53;
uint8_t DEVID=0x00;
uint8_t DATAX0=0x32;
uint8_t DATAX1=0x33;
uint8_t DATAY0=0x34;
uint8_t DATAY1=0x35;
uint8_t DATAZ0=0x36;
uint8_t DATAZ1=0x37;
uint8_t BW_RATE=0x2C;								//Data rate and power mode control 
uint8_t POWER_CTL=0x2D; 						//Power-saving features control 		
uint8_t DATA_FORMAT=0x31;						//Data format control
// Data from accselerometr
float X=0;
float Y=0;
float Z=0;
int16_t ACCEL_XANGLE=0;
int16_t ACCEL_YANGLE=0;
int16_t ACCEL_ZANGLE=0;

// Bufers for UART
//int i=0;
int len;
char buffer[100];
char Rx_indx, Rx_data[2],Rx_Buffer[100],Transfer_cplt;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */



void I2C_scaner(void);
void init_ADXL345(void);
void init_oled(void);
void STOP(void);
void FORWARD(void);
void BACK(void);
void LEFT(void); 
void RIGHT(void);
void transmit_data_in_comport(void);
void transmit_data_in_BLUETOOTH(void);
void drow_data_on_OLED(void);
void convert_accel_data_in_angle(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart2);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  init_ADXL345();    									   // Init  accselerometr
	init_oled();                           // Init OLED and draw lise and words on OLED
	
	
	//I2C_scaner();
	//HAL_Delay(1000);
	
	uint32_t lst = 0, cu;
	uint8_t transmit_test_data=0;

	MX_TIM3_Init();												 // Init Timer
	HAL_TIM_Base_Start(&htim3);    		     // Start Timer1
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_UART_Receive_IT(&huart2 ,(uint8_t*) Rx_data,1);    // Start UART
//	HAL_Delay(2000);
//	I2C_scaner();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {		 
		// Test UART--------------------------------------------------
		if(Transfer_cplt)   // If string hes Enter/#013 in the end transmiting paket.
		{
				  sprintf(buffer, "%s\r\n",Rx_Buffer);
			    len=strlen(buffer);
					HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 1000);  // Transmit data
			    Transfer_cplt=0;
			    
			    if((uint8_t)Rx_Buffer[0]==0x31)   // '1'
					{
						  sprintf(buffer, "1111111\r\n");
			        len=strlen(buffer);
					    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 1000);  // Transmit data
			        Transfer_cplt=0;
					}
					if((uint8_t)Rx_Buffer[0]==0x32)    // '2'
					{
						  sprintf(buffer, "22222222\r\n");
			        len=strlen(buffer);
					    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 1000);  // Transmit data
			        Transfer_cplt=0;
					}
			
			
			    HAL_Delay(2000);
		}
		//-----------------------------------------------------------
		      
				  transmit_data_in_comport();						// Transmit data in COMport
				  transmit_data_in_BLUETOOTH();					// Transmit data in bluetooth module

					HAL_TIM_Base_Stop(&htim3);									// Stop work timmer, for corect print data
		  
		      drow_data_on_OLED();   											// Print data acseleration on OLED
		
					HAL_TIM_Base_Start(&htim3);
    
	}		
  
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 35999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void I2C_scaner(void)
{
	
	/*Description function
	This function search devise connected to I2C in this case -hi2c1.
	After thet function print in console information about what to connect to I2C. 
	*/
	#define DEVICE_FOUND  0
	uint8_t addres_devise=0x00;;      //ADRESS_MPU6050=0x68; -> return 0  ,   DRESS_MS5611=0x77;-> return 0 		 
	uint8_t addr=0;
	uint16_t sizebuf=1;								// size how many data we receive from devise			
	uint8_t buff=0;										// data for receive
	uint32_t timeout=1000;						// timeout for receive
  uint16_t STATUS=0;								// Status connect to device (if STATUS==0 - device if found, if STATUS==1 - device if not found)
	uint8_t number_of_device=0;				// How many device controller is found
	
	uint8_t size=0;
	char str3[35]={0};
	uint8_t size_mas=sizeof(str3);
	uint8_t i=0;
	
	sprintf(str3,"SEARCH DEVISES... \r\n");      										// convert   in  str 
	size=sizeof(str3);
	HAL_UART_Transmit(&huart2 , (uint8_t *)str3, size, 0xFFFF);
	HAL_Delay(500);
	for(addres_devise=0x00;addres_devise<0x7F;addres_devise++)
	{
			HAL_Delay(10);
			STATUS=HAL_I2C_Mem_Read(&hi2c1, (uint16_t)addres_devise<<1,(uint16_t)addr, (uint16_t) sizebuf, &buff, (uint16_t) sizebuf,(uint32_t) timeout);
			if(STATUS==DEVICE_FOUND)																		// if devsice is found
			{	
				  for(i=0;i<size_mas;i++)    															// Delete data in str3[]
					{
						str3[i]=0;
					}
					sprintf(str3,"Device address-0x%x - found \r\n",addres_devise);      // convert   in  str 
					size=sizeof(str3);
					HAL_UART_Transmit(&huart2 , (uint8_t *)str3, size, 0xFFFF);
					number_of_device++;
			}
	}
	if(number_of_device==0)  																				// If devices nofound
	{	
			for(i=0;i<size_mas;i++)				 															// Delete data in str3[]
			{
				str3[i]=0;
			}
			sprintf(str3,"Devices no found!!!\r\n");      							// convert   in  str 
			size=sizeof(str3);
			HAL_UART_Transmit(&huart2 , (uint8_t *)str3, size, 0xFFFF);
	}
	HAL_Delay(500);
	for(i=0;i<size_mas;i++)																					// Delete data in str3[]
	{
		str3[i]=0;
	}
	sprintf(str3,"DONE\r\n");      																	// convert   in  str 
	size=sizeof(str3);
	HAL_UART_Transmit(&huart2 , (uint8_t *)str3, size, 0xFFFF);	
}

///////////////////////////////////////////////////////////////////////////////////////////////
void init_ADXL345(void)
{
		// Device ID	
	  uint8_t sizebuf=1;
		uint16_t timeout=1000;
		char str1[40]={0};	
		uint8_t size=0;
		uint8_t who_i_am_buffer=0x00;
		HAL_I2C_Mem_Read(&hi2c1, (uint16_t)Device_addres<<1,(uint16_t)DEVID, (uint16_t) sizebuf, &who_i_am_buffer, (uint16_t) sizebuf,(uint32_t) timeout);
	  if(who_i_am_buffer!=Device_ID)
		{
				sprintf(str1,"CONNECTION_TO_ADXL345_ERROR!!! \r\n");                		  					// convert   in  str 
				size=sizeof(str1);
				HAL_UART_Transmit(&huart2 , (uint8_t *)str1, size, 0xFFFF);     // send  new  line  in  com  port
				HAL_Delay(4000);	
		}
		else
		{
				sprintf(str1,"CONNECTION_TO_ADXL345_OK   \r\n");                		  						// convert  in  str 
				size=sizeof(str1);
				HAL_UART_Transmit(&huart2 , (uint8_t *)str1, size, 0xFFFF);     // send  new  line  in  com  port
				HAL_Delay(200);	
		}
		// 

    //Init registers
		uint8_t POWER_CTL_CONFIGURE=0x08;
		HAL_I2C_Mem_Write(&hi2c1, (uint16_t) Device_addres<<1, (uint16_t) POWER_CTL, (uint16_t) 1, &POWER_CTL_CONFIGURE, (uint16_t) 1, (uint32_t) 1000);
		HAL_Delay(20);
		uint8_t DATA_FORMAT_CONFIGURE=0x03;   // Measure 16 G
		HAL_I2C_Mem_Write(&hi2c1, (uint16_t) Device_addres<<1, (uint16_t) DATA_FORMAT, (uint16_t) 1, &DATA_FORMAT_CONFIGURE, (uint16_t) 1, (uint32_t) 1000);
		HAL_Delay(20);
    //
		
		// Test read configure registers.
		uint8_t sizebuf_1=1;
		uint8_t POWER_CTL_buffer=0;
		uint8_t BW_RATE_buffer=0;
		uint8_t DATA_FORMAT_buffer=0;
		uint16_t timeout_1=0x100;
		HAL_I2C_Mem_Read(&hi2c1, (uint16_t)Device_addres<<1,(uint16_t)POWER_CTL, (uint16_t) sizebuf_1, &POWER_CTL_buffer, (uint16_t) sizebuf,(uint32_t) timeout_1);
		HAL_I2C_Mem_Read(&hi2c1, (uint16_t)Device_addres<<1,(uint16_t)BW_RATE, (uint16_t) sizebuf_1, &BW_RATE_buffer, (uint16_t) sizebuf,(uint32_t) timeout_1);
		HAL_I2C_Mem_Read(&hi2c1, (uint16_t)Device_addres<<1,(uint16_t)DATA_FORMAT, (uint16_t) sizebuf_1, &DATA_FORMAT_buffer, (uint16_t) sizebuf,(uint32_t) timeout_1);

		char str6[70]={0};	
		sprintf(str6,"BUF:   POWER_CTL- %X, BW_RATE- %X, DATA_FORMAT- %X \r\n",POWER_CTL_buffer, BW_RATE_buffer, DATA_FORMAT_buffer);                		  	// convert   in  str 
		uint8_t size_1=0;
		size_1=sizeof(str6);
		HAL_UART_Transmit(&huart2 , (uint8_t *)str6, size_1, 0xFFFF);     // send  new  line  in  com  port
		//HAL_Delay(2000);
		//
}	


///////////////////////////////////////////////////////////////////////////////////////////////
void data_from_ADXL345(void)
{
	
	    int16_t X_acc_value=0;
	    int16_t Y_acc_value=0;
			int16_t Z_acc_value=0;
	    uint16_t status=0;
			char str1[40]={0};
			uint8_t sizebuf=1;
			uint8_t sizebuf_data=6;
			uint8_t buffer_XYZ[6]={0};
			uint32_t timeout_1=1000;
			
			// Read accekeration fron ADXL345
			status=HAL_I2C_Mem_Read(&hi2c1, (uint16_t)Device_addres<<1,(uint16_t)DATAX0, (uint16_t) sizebuf, buffer_XYZ, (uint16_t) sizebuf_data,(uint32_t) 0xff);
			if(status!=HAL_OK)  
			{
				// Error handler
				while(1){}
			}
			else
			{
				X_acc_value=(uint16_t)buffer_XYZ[0]<<8|buffer_XYZ[1];
				Y_acc_value=(uint16_t)buffer_XYZ[2]<<8|buffer_XYZ[3];
				Z_acc_value=(uint16_t)buffer_XYZ[4]<<8|buffer_XYZ[5];
			}
			//
			// Convert data in G/s
			X=(float)X_acc_value/8192.0;
			Y=(float)Y_acc_value/8192.0;
			Z=(float)Z_acc_value/8192.0;
			//

//			// Print data
//			sprintf(str1,"X:%.2f  Y:%.2f  Z:%.2f \r\n",X, Y, Z);                		  					// convert   in  str 
//			uint8_t size=sizeof(str1);
//			HAL_UART_Transmit(&huart2 , (uint8_t *)str1, size, 0xFF);     // send  new  line  in  com  port	
			
			//			
}
///////////////////////////////////////////////////////////////////////////////////////////////
void init_oled(void)
{				
			 // Test OLED 
			 uint8_t res = SSD1306_Init();	
			 char str8[10]={10}; 						// Variable for status OLED  
			 if(res==1)
			 {
						sprintf(str8,"OK");  
			 }
			 else
			 {
						sprintf(str8,"ERROR!!!!");
			 }
	     //
			 SSD1306_Fill(0);			//Erice  OLED
			 // Running line
			 int x2=0, y2=16;
			 for(x2=0;x2<128;x2++)								
			 {
						x2=x2+4;
						SSD1306_DrawLine(0, y2, x2, y2, 1);
						SSD1306_UpdateScreen();	
						if(x2>=127)
						{
								SSD1306_DrawLine(0, y2, x2, y2, 0);
								SSD1306_UpdateScreen();
						}
			 }
			 // Draw main figures in OLED
	     uint16_t  r=0;
			 int x1=0, y1=0;
			 x2=0,y2=0; 								  								// Print  text on OLED
			 SSD1306_GotoXY(x1,y1);
			 SSD1306_Puts("ACCELERATION g/sec", &Font_7x10, 1);
		
			 x1=0,y1=10,x2=130,y2=10;   									// Draw line
			 SSD1306_DrawLine(x1, y1, x2, y2, 1);
			 SSD1306_DrawRectangle(0, 16, 60, 34, 1);    	// Draw rectangle
			 x1=74,y1=38,	r=8;   													// Draw circle  back
			 SSD1306_DrawCircle(x1,y1, r, 1);
			 x1=95,y1=38, r=8;  													// Draw circle  // ser
			 SSD1306_DrawCircle(x1,y1, r, 1);
			 x1=116,y1=38,r=8;														// Draw circle  // First
			 SSD1306_DrawCircle(x1,y1, r, 1);
			 x1=95,y1=22, r=6;  													// Draw circle  // Left
			 SSD1306_DrawCircle(x1,y1, r, 1);
			 x1=95,y1=54, r=6;  													// Draw circle  // Right
			 SSD1306_DrawCircle(x1,y1, r, 1);
			 SSD1306_UpdateScreen(); 
			 //
}

///////////////////////////////////////////////////////////////////////////////////////////////
void STOP(void)
{
				uint16_t  r=0;
			  int x1=0, y1=0;							  
				// Draw circle
				x1=95,y1=38; r=7;
				SSD1306_DrawFilledCircle(x1,y1, r, 1);
				// Off  all another circle
				x1=116,y1=38;r=7;
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
				x1=74,y1=38; r=7;
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
				x1=95,y1=22;  r=5;
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
				x1=95,y1=54;r=5;
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
		 
				SSD1306_UpdateScreen();
}

///////////////////////////////////////////////////////////////////////////////////////////////
void FORWARD(void)
{
				uint16_t  r=0;
			  int x1=0, y1=0;	
				x1=116,y1=38;r=7;
				SSD1306_DrawFilledCircle(x1,y1, r, 1);
				// Off  all another circle
				x1=95,y1=38; r=7;
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
				x1=74,y1=38; r=7;
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
				x1=95,y1=22;  r=5;
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
				x1=95,y1=54;r=5;
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
		 
				SSD1306_UpdateScreen();
}

///////////////////////////////////////////////////////////////////////////////////////////////
void BACK(void)
{
        uint16_t  r=0;
			  int x1=0, y1=0;	
				x1=74,y1=38; r=7;
				SSD1306_DrawFilledCircle(x1,y1, r, 1);
				// Off  all another circle
				x1=116,y1=38; r=7;
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
				x1=95,y1=38; r=7;
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
				x1=95,y1=22;  r=5;
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
				x1=95,y1=54;r=5;
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
		 
				SSD1306_UpdateScreen();
}

///////////////////////////////////////////////////////////////////////////////////////////////
void LEFT(void)
{
				uint16_t  r=0;
			  int x1=0, y1=0;	
				x1=95,y1=22;  r=5;
				SSD1306_DrawFilledCircle(x1,y1, r, 1);
				x1=95,y1=38; r=7;
				// Off  all another circle
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
				x1=116,y1=38;r=7;
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
				x1=74,y1=38; r=7;
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
				x1=95,y1=54;r=5;
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
		 
				SSD1306_UpdateScreen();
}

///////////////////////////////////////////////////////////////////////////////////////////////
void RIGHT(void)
{ 
				uint16_t  r=0;
			  int x1=0, y1=0;	
				x1=95,y1=54;r=5;
				SSD1306_DrawFilledCircle(x1,y1, r, 1);
				// Off  all circle
				x1=116,y1=38;r=7;
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
				x1=95,y1=22;  r=5;
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
				x1=95,y1=38; r=7;
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
				x1=74,y1=38; r=7;
				SSD1306_DrawFilledCircle(x1,y1, r, 0);
		
				SSD1306_UpdateScreen();
}

void transmit_data_in_comport(void)
{
				uint16_t size=0;
				char buf2[40]={0};
				size=sizeof(buf2);
				sprintf(buf2, "X:%d Y:%d Z:%d \r\n",ACCEL_XANGLE ,ACCEL_YANGLE, ACCEL_ZANGLE);
				HAL_UART_Transmit(&huart2, (uint8_t*) buf2, size, 0xFF);					// Transmit data in COMPORT
}


void transmit_data_in_BLUETOOTH(void)
{
				uint16_t size=0;
				char buf2[40]={0};
				size=sizeof(buf2);
				sprintf(buf2, "X:%d Y:%d Z:%d \r\n",ACCEL_XANGLE,ACCEL_YANGLE, ACCEL_ZANGLE);
				HAL_UART_Transmit(&huart1, (uint8_t*) buf2, size, 0xFF);					// Transmit data in COMPORT
}

void drow_data_on_OLED(void)
{
	        convert_accel_data_in_angle();
	 
	
          // clear old data from OLED
	        char buf1[5]={0};
	
					sprintf(buf1, "X:%d",ACCEL_XANGLE);
					SSD1306_GotoXY(3,18);
					SSD1306_Puts(buf1, &Font_7x10, 1);
					sprintf(buf1, "Y:%d",ACCEL_YANGLE);
					SSD1306_GotoXY(3,30);
					SSD1306_Puts(buf1, &Font_7x10, 1);
					sprintf(buf1, "Z:%d",ACCEL_ZANGLE);
					SSD1306_GotoXY(3,40);
					SSD1306_Puts(buf1, &Font_7x10, 1);
					SSD1306_UpdateScreen();
					//
					 
					 // Drow circle and send data in 407
					 if(((X>=-0.2)&(X<=0.2))&((Y>=-0.2)&(Y<=0.2))&((Z>=0.8)&(Z<=1.2)))			// Cheack  data from acceleration  mudule.
					 {
								STOP();						// STOP function drow circle on OLED
								// Send action in 407		
								//transmit_test_data='0';			// '0' - stop command
								//HAL_UART_Transmit(&huart1, &transmit_test_data,1, 2);		  // Receive data from bluetooth
					 }
					 else if(X<-0.2)
					 {
								FORWARD();				// FORVARD function drow circle on OLED
								// Send action in 407		
								//transmit_test_data='1';			// '1' - stop command
								//HAL_UART_Transmit(&huart1, &transmit_test_data,1, 2);		  // Receive data from bluetooth
					 }
					 else if(X>0.2)
					 {
								BACK();  					// BACK function drow circle on OLED
								// Send action in 407		
								//transmit_test_data='2';			// '2' - stop command
								//HAL_UART_Transmit(&huart1, &transmit_test_data,1, 2);		  // Receive data from bluetooth
					 }	
					 else if(Y<-0.2)
					 {			
								LEFT();           // LEFT function drow circle on OLED
								// Send action in 407		
								//transmit_test_data='3';			// '3' - stop command
								//HAL_UART_Transmit(&huart1, &transmit_test_data,1, 2);		  // Receive data from bluetooth
					 }
					 else if(Y>0.2)
					 { 
								RIGHT();	        // RIGHT function drow circle on OLED
								// Send action in 407		
								//transmit_test_data='4';			// '4' - stop command
								//HAL_UART_Transmit(&huart1, &transmit_test_data,1, 2);		  // Receive data from bluetooth
					 }
}

void convert_accel_data_in_angle(void)
{
			//Convert accelerometr data in angle
			// FORMULA ||| X_ANHLE=
			ACCEL_XANGLE = 57.295*atan((float)-X/ sqrt(pow((float)Y,2)+pow((float)Z,2)));
			ACCEL_YANGLE = 57.295*atan((float)-Y/ sqrt(pow((float)X,2)+pow((float)Z,2)));
			ACCEL_ZANGLE = 57.295*atan((float)-Z/ sqrt(pow((float)X,2)+pow((float)Y,2)));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart2)  
{
	// It finctinon called when in buffer UART has one byte
	uint8_t i;
	if(Rx_indx==0)															
	{
			for(i=0;i<100;i++)																	// Clar buffer before receiving nev data.
			{
					Rx_Buffer[i]=0;
			}
		}
	if(Rx_data[0]!=13)																			// If check if it byte is not "Enter".
	{
			Rx_Buffer[Rx_indx++]=Rx_data[0];
	}
	else
	{
		Rx_indx=0;
		Transfer_cplt=1;												 							// Buffer complete, data ready to read.
	}
	HAL_UART_Receive_IT(huart2, (uint8_t*)Rx_data,1);				
}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
