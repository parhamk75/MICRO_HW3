
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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "lm016.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


extern 		UART_HandleTypeDef huart2;
uint32_t 	speeds[] 		= {9600, 115200, 1000000};
uint8_t		msg[50];

typedef struct{
	GPIO_TypeDef* port;
	uint16_t 		 	pin;
}LED_t;

uint8_t 	receive_pckt[8];
uint8_t		pckt_rcvd_flg;
LED_t 		LEDs[4];

typedef enum{
	LED_set = 1,
	LCD = 2,
	ERR = 3
} Packet_Type;

Packet_Type pckt_t = ERR;

lcd_t lcd_0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
		
		/////////////// Q1 //////////////
/*
	for(uint8_t tmp_itrtr_1= 0; tmp_itrtr_1 < 3; tmp_itrtr_1++)
		{
			sprintf((char*)msg, "Speed is changing to %7d in 3 seconds\n\r", speeds[tmp_itrtr_1]);
			HAL_UART_Transmit(&huart2, msg, 43, 200);
			HAL_Delay(3000);
						
			huart2.Init.BaudRate = speeds[tmp_itrtr_1];
			HAL_UART_Init(&huart2);
			for(uint16_t tmp_itrtr_2= 0; tmp_itrtr_2 < 2000; tmp_itrtr_2++)
			{
				sprintf((char*)msg, "Hello UART %4d\n\r", tmp_itrtr_2+1);
				HAL_UART_Transmit(&huart2, msg, 18, 200);
			}					
		} 
	
*/
		

		
				///////////// Q2 ////////////			
		LEDs[0].port = GPIOA;
		LEDs[0].pin  = GPIO_PIN_0;
		LEDs[1].port = GPIOA;
		LEDs[1].pin	 = GPIO_PIN_1;
		LEDs[2].port = GPIOA;
		LEDs[2].pin	 = GPIO_PIN_4;
		LEDs[3].port = GPIOB;
		LEDs[3].pin	 = GPIO_PIN_0;
		
		HAL_UART_Receive_IT(&huart2, receive_pckt, 8);
		

		/////////// Q4 ////////////
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);	// LCD CONTRAST
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // LCD BACKLIGHT
	
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 10); // 0 => High Cont.  100 => Low Cont.
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 60);
	
	
	
	/* <= LCD Initialization BEGIN => */

	// Data Ports
	lcd_0.data_ports[0] = GPIOB;
	lcd_0.data_ports[1] = GPIOA;
	lcd_0.data_ports[2] = GPIOA;
	lcd_0.data_ports[3] = GPIOC;
	lcd_0.data_ports[4] = GPIOB;
	lcd_0.data_ports[5] = GPIOA;
	lcd_0.data_ports[6] = GPIOA;
	lcd_0.data_ports[7] = GPIOA;
	// Data Pins
	lcd_0.data_pins[0]  = GPIO_PIN_10;
	lcd_0.data_pins[1]  = GPIO_PIN_8;
	lcd_0.data_pins[2]  = GPIO_PIN_9;
	lcd_0.data_pins[3]  = GPIO_PIN_7;
	lcd_0.data_pins[4]  = GPIO_PIN_6;
	lcd_0.data_pins[5]  = GPIO_PIN_7;
	lcd_0.data_pins[6]  = GPIO_PIN_6;
	lcd_0.data_pins[7]  = GPIO_PIN_5;
	
	
	// Control Ports
	lcd_0.rs_port				= GPIOB;	
	lcd_0.en_port 			= GPIOB;
	
	// Control Pins
	lcd_0.rs_pin				= GPIO_PIN_3;
	lcd_0.en_pin 				= GPIO_PIN_4;
	
	
	// Mode
	lcd_0.mode					= _8_BIT;
	
	HAL_Delay(100);
	lcd_init(&lcd_0);
	/* <= LCD Initialization END => */
	
	lcd_puts(&lcd_0, "Hello! Welcome to the HW3!");
	HAL_Delay(1000);
	lcd_clear(&lcd_0);
		
  while (1)
  {

		if(pckt_rcvd_flg == 1)
		{
			/*
			lcd_puts(&lcd_0, "Checking the Packet Type");
			HAL_Delay(2000);
			lcd_clear(&lcd_0);
			*/
			
			// check the packet
			if(receive_pckt[0] == 255 && receive_pckt[1] == 255 && receive_pckt[6] == 200 && receive_pckt[7] == 200)
			{				
				if(receive_pckt[2] == 0 || receive_pckt[2] == 1)
				{
					pckt_t = LED_set;
					
					lcd_puts(&lcd_0, "Packet Type ==> LED");
					HAL_Delay(2000);
					lcd_clear(&lcd_0);
					
				}
				else
				{
					pckt_t = LCD;
					
					lcd_puts(&lcd_0, "Packet Type ==> LCD");
					HAL_Delay(2000);
					lcd_clear(&lcd_0);
					
				}					
			}
			else
			{
				pckt_t = ERR;
				/*
				lcd_puts(&lcd_0, "ERROR");
				HAL_Delay(2000);
				lcd_clear(&lcd_0);
				*/
				
			}
			
			// Perform the command!
			switch (pckt_t)
      {
      	case LED_set:
      		for(uint8_t tmp_itrtr_3= 0; tmp_itrtr_3 < 4; tmp_itrtr_3++)
					{
						HAL_GPIO_WritePin(LEDs[tmp_itrtr_3].port, LEDs[tmp_itrtr_3].pin, receive_pckt[tmp_itrtr_3 + 2]);
					}
					
      	case LCD:
      		switch (receive_pckt[2])
          {
          	case 2:
							lcd_puts(&lcd_0, "LCD --> 2");
							HAL_Delay(2000);
							lcd_clear(&lcd_0);
          		
							lcd_clear( &lcd_0 );
          	case 3:
							lcd_puts(&lcd_0, "LCD --> 3");
							HAL_Delay(2000);
							lcd_clear(&lcd_0);
						
          		lcd_set_curser(&lcd_0, receive_pckt[3], receive_pckt[4]);
						case 4:
							lcd_puts(&lcd_0, "LCD --> 4");
							HAL_Delay(2000);
							lcd_clear(&lcd_0);
						
							lcd_putchar( &lcd_0, receive_pckt[3] );
						case 5:
							lcd_puts(&lcd_0, "LCD --> 5");
							HAL_Delay(2000);
							lcd_clear(&lcd_0);
						
          		send_cmd(&lcd_0, receive_pckt[3]);
          	default:
          		HAL_UART_Transmit_IT(&huart2, (uint8_t*)"LCD command not valid!\n\r", 24);
          }
				
				default:
      		HAL_UART_Transmit_IT(&huart2, (uint8_t*)"Error in receiving the packet\n\r", 31);
      }
								
			pckt_rcvd_flg = 0;
			HAL_UART_Receive_IT(&huart2, receive_pckt, 8);
			
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
