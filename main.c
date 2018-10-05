
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
  *   18. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   2. Neither the name of STMicroelectronics nor the names of its contributors
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
#include "stm32f0xx_hal.h"
#include "arduinotoGreen.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

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
  int OUTPUT = 1;
  int INPUT = 0;
  int HIGH = 1;
  int LOW = 0;
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 18 */
  //Pinos ligados aos pinos 4, 5, 6 e 7 do teclado - Linhas
   pinMode(4, OUTPUT);
   pinMode(5, OUTPUT);
   pinMode(6, OUTPUT);
   pinMode(7, OUTPUT);

   //Pinos ligados aos pinos 8, 9, e 10 do teclado - Colunas
   pinMode(8, INPUT);
   //Ativacao resistor pull-up
   //digitalWrite(8, HIGH);
   pinMode(9, INPUT);
   //digitalWrite(9, HIGH);
   pinMode(19, INPUT);
   //digitalWrite(10, HIGH);
  /* USER CODE END 18 */

   //led
   pinMode(0,OUTPUT);
   digitalWrite(0,LOW);
   pinMode(1,OUTPUT);
   digitalWrite(1,LOW);
   pinMode(18,OUTPUT);
   digitalWrite(18,LOW);
   digitalWrite(2,LOW);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */
	  for (int porta = 4; porta<8; porta++)
	      {
	        //Alterna o estado dos pinos das linhas
	        digitalWrite(4, HIGH);
	        digitalWrite(5, HIGH);
	        digitalWrite(6, HIGH);
	        digitalWrite(7, HIGH);
	        digitalWrite(porta, LOW);
	        //Verifica se alguma tecla da coluna 1 foi pressionada
	        if (digitalRead(8) == LOW)
	        {
	        	digitalWrite(0,LOW);
	        		        		          	        	  digitalWrite(1,LOW);
	        		        		          	        	  digitalWrite(18,LOW);
	        		        		          	        	  digitalWrite(2,LOW);
	          if(porta == 4){
	        	  digitalWrite(0,HIGH);
	        	  digitalWrite(1,LOW);
	        	  digitalWrite(18,LOW);
	        	  digitalWrite(2,LOW);
	          }
	          if(porta == 5){
	          	        	  digitalWrite(0,LOW);
	          	        	  digitalWrite(1,LOW);
	          	        	  digitalWrite(18,HIGH);
	          	        	  digitalWrite(2,LOW);
	          	          }
	          if(porta == 6){
	          	        	  digitalWrite(0,HIGH);
	          	        	  digitalWrite(1,HIGH);
	          	        	  digitalWrite(18,HIGH);
	          	        	  digitalWrite(2,LOW);
	          	          }
	          if(porta == 7){
	          	        	  digitalWrite(0,HIGH);
	          	        	  digitalWrite(1,HIGH);
	          	        	  digitalWrite(18,HIGH);
	          	        	  digitalWrite(2,HIGH);
	          	          }
	          while(digitalRead(8) == LOW){}

	        }

	        //Verifica se alguma tecla da coluna 18 foi pressionada
	        if (digitalRead(9) == LOW)
	        {
	        	digitalWrite(0,LOW);
	        		        		          	        	  digitalWrite(1,LOW);
	        		        		          	        	  digitalWrite(18,LOW);
	        		        		          	        	  digitalWrite(2,LOW);
	        	if(porta == 4){
	        		        	  digitalWrite(0,LOW);
	        		        	  digitalWrite(1,HIGH);
	        		        	  digitalWrite(18,LOW);
	        		        	  digitalWrite(2,LOW);
	        		          }
	        		          if(porta == 5){
	        		          	        	  digitalWrite(0,LOW);
	        		          	        	  digitalWrite(1,HIGH);
	        		          	        	  digitalWrite(18,HIGH);
	        		          	        	  digitalWrite(2,LOW);
	        		          	          }
	        		          if(porta == 6){
	        		          	        	  digitalWrite(0,LOW);
	        		          	        	  digitalWrite(1,LOW);
	        		          	        	  digitalWrite(18,LOW);
	        		          	        	  digitalWrite(2,HIGH);
	        		          	          }
	        		          if(porta == 7){
	        		          	        	  digitalWrite(0,LOW);
	        		          	        	  digitalWrite(1,LOW);
	        		          	        	  digitalWrite(18,LOW);
	        		          	        	  digitalWrite(2,LOW);
	        		          	          }
	          while(digitalRead(9) == LOW){};

	        }

	        //Verifica se alguma tecla da coluna 2 foi pressionada
	        if (digitalRead(19) == LOW)
	        {
	        	digitalWrite(0,LOW);
	        		        		          	        	  digitalWrite(1,LOW);
	        		        		          	        	  digitalWrite(18,LOW);
	        		        		          	        	  digitalWrite(2,LOW);
	        	if(porta == 4){
	        		        	  digitalWrite(0,HIGH);
	        		        	  digitalWrite(1,HIGH);
	        		        	  digitalWrite(18,LOW);
	        		        	  digitalWrite(2,LOW);
	        		          }
	        		          if(porta == 5){
	        		          	        	  digitalWrite(0,LOW);
	        		          	        	  digitalWrite(1,HIGH);
	        		          	        	  digitalWrite(18,HIGH);
	        		          	        	  digitalWrite(2,LOW);
	        		          	          }
	        		          if(porta == 6){
	        		          	        	  digitalWrite(0,HIGH);
	        		          	        	  digitalWrite(1,LOW);
	        		          	        	  digitalWrite(18,LOW);
	        		          	        	  digitalWrite(2,HIGH);
	        		          	          }
	        		          if(porta == 7){
	        		          	        	  digitalWrite(0,LOW);
	        		          	        	  digitalWrite(1,HIGH);
	        		          	        	  digitalWrite(18,HIGH);
	        		          	        	  digitalWrite(2,HIGH);
	        		          	          }
	          while(digitalRead(19) == LOW){}

	        }
	      }
	     delay(10);
  /* USER CODE BEGIN 2 */

  }
  /* USER CODE END 2 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

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
