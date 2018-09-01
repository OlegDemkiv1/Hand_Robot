/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t Device_ID=0xE5;
uint8_t Device_addres=0x53;
// Registers
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void I2C_scaner(void);
void init_ADXL345(void);
void data_from_ADXL345(void);
void init_oled(void);

void STOP(void);
void FORWARD(void);
void BACK(void);
void LEFT(void);
void RIGHT(void);
/* USER CODE END 0 */

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
  init_ADXL345();    									// Init  accselerometr
	HAL_TIM_Base_Start(&htim3);    		 //Start Timer1
	HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t lst = 0, cu;
	
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
			 uint8_t receive_data=0;
			 init_oled();                       //Draw lise and words on OLED

			 while(1)
			 {
					 // Print acceleration on OLED
					 //data_from_ADXL345(); 			// tesr read accel
				  
				 
				   // Transmit data in COMport
					uint16_t size=0;
					char buf2[40]={0};
					size=sizeof(buf2);
					sprintf(buf2, "X:%.3f Y:%.3f Z:%.3f \r\n",X ,Y, Z);
					HAL_UART_Transmit(&huart2, (uint8_t*) buf2, size, 0xFF);					// Transmit data in COMPORT
					//
	
					// Transmit data in bluetooth module
					HAL_UART_Transmit(&huart1, (uint8_t*) buf2, size, 0xFF);					// Transmit data in COMPORT
					//
					 
					 HAL_TIM_Base_Stop(&htim3);
				   // Print data acseleration on OLED
					 char buf1[8]={0};
					 sprintf(buf1, "X:%.3f",X);
					 SSD1306_GotoXY(3,18);
					 SSD1306_Puts(buf1, &Font_7x10, 1);
					 sprintf(buf1, "Y:%.3f",Y);
					 SSD1306_GotoXY(3,30);
					 SSD1306_Puts(buf1, &Font_7x10, 1);
					 sprintf(buf1, "Z:%.3f",Z);
					 SSD1306_GotoXY(3,40);
					 SSD1306_Puts(buf1, &Font_7x10, 1);
					 SSD1306_UpdateScreen();
					 //
					 
					 uint8_t transmit_test_data=0;
					 
					  
					 
					 // Drow circle and send data in 407
					 if(((X>=-0.2)&(X<=0.2))&((Y>=-0.2)&(Y<=0.2))&((Z>=0.8)&(Z<=1.2)))			// Cheack  data from acceleration  mudule.
					 {
								STOP();						// STOP function drow circle on OLED
								// Send action in 407		
								transmit_test_data='0';			// '0' - stop command
								HAL_UART_Transmit(&huart1, &transmit_test_data,1, 2);		  // Receive data from bluetooth
					 }
					 else if(X<-0.2)
					 {
								FORWARD();				// FORVARD function drow circle on OLED
								// Send action in 407		
								transmit_test_data='1';			// '1' - stop command
								HAL_UART_Transmit(&huart1, &transmit_test_data,1, 2);		  // Receive data from bluetooth
					 }
					 else if(X>0.2)
					 {
								BACK();  					// BACK function drow circle on OLED
								// Send action in 407		
								transmit_test_data='2';			// '2' - stop command
								HAL_UART_Transmit(&huart1, &transmit_test_data,1, 2);		  // Receive data from bluetooth
					 }	
					 else if(Y<-0.2)
					 {			
								LEFT();           // LEFT function drow circle on OLED
								// Send action in 407		
								transmit_test_data='3';			// '3' - stop command
								HAL_UART_Transmit(&huart1, &transmit_test_data,1, 2);		  // Receive data from bluetooth
					 }
					 else if(Y>0.2)
					 { 
								RIGHT();	        // RIGHT function drow circle on OLED
								// Send action in 407		
								transmit_test_data='4';			// '4' - stop command
								HAL_UART_Transmit(&huart1, &transmit_test_data,1, 2);		  // Receive data from bluetooth
					 }
					 HAL_TIM_Base_Start(&htim3);
					 
			 }		
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
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
		HAL_Delay(2000);
		//
}	

void data_from_ADXL345(void)
{
			// Read data X  register
			char str1[40]={0};
			uint8_t sizebuf=1;
			uint8_t sizebuf_data=6;
			uint8_t buffer_XYZ[6]={0};
			uint32_t timeout_1=1000;
			HAL_I2C_Mem_Read(&hi2c1, (uint16_t)Device_addres<<1,(uint16_t)DATAX0, (uint16_t) sizebuf, buffer_XYZ, (uint16_t) sizebuf_data,(uint32_t) timeout_1);
			int16_t X_value=(uint16_t)buffer_XYZ[0]<<8|buffer_XYZ[1];
			int16_t Y_value=(uint16_t)buffer_XYZ[2]<<8|buffer_XYZ[3];
			int16_t Z_value=(uint16_t)buffer_XYZ[4]<<8|buffer_XYZ[5];
			//
			
			// Convert data in G/s
			X=(float)X_value/8192.0;
			Y=(float)Y_value/8192.0;
			Z=(float)Z_value/8192.0;
			//
			
//			// Print data
//			sprintf(str1,"X:%.2f  Y:%.2f  Z:%.2f \r\n",X, Y, Z);                		  					// convert   in  str 
//			uint8_t size=sizeof(str1);
//			HAL_UART_Transmit(&huart2 , (uint8_t *)str1, size, 0xFF);     // send  new  line  in  com  port	
			
			//			
}
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





/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
