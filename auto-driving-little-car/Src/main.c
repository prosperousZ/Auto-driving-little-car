
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define LED1_Toggle									HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_8)
#define LED1_High										HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET)
#define LED1_Low										HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET)
#define LED2_Toggle									HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_9)
#define LED2_High										HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET)
#define LED2_Low										HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET)
#define c14_high                    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET)
#define c15_low                     HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET)
#define LED3_Toggle									HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6)
#define LED3_High										HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET)
#define LED3_Low										HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET)
#define LED4_Toggle									HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7)
#define LED4_High										HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET)
#define LED4_Low										HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET)
#define PWM_Toggle									HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_4)
#define PWM_High										HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET)
#define PWM_Low										HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET)
#define PWM1_Toggle									HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_10)
#define PWM1_High										HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET)
#define PWM1_Low										HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET)
#define PWM2_Toggle									HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_3)
#define PWM2_High										HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET)
#define PWM2_Low										HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET)
#define KEY1	 										HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
//static void MX_TIM1_Init(uint32_t arr,uint16_t psc);
static void MX_TIM2_Init(uint32_t arr,uint16_t psc);
//static void MX_TIM3_Init(uint32_t arr,uint16_t psc);
void delay_init(uint8_t SYSCLK);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);
void KeyProcess(void);
void Sensor4_Trig(void);
void Sensor2_Trig(void);
void Sensor3_Trig(void);
void Timer2_Length_Calc(uint8_t *state,uint32_t value,uint8_t channel);
//void Timer2_Length_Calc(uint8_t *state,uint32_t value);
//void Timer3_Length_Calc(uint8_t *state,uint16_t value);
void ChannelX_Start(uint8_t Channel);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint8_t  TIM2CH2_CAPTURE_STA=0,Channel2_ENABLE=0;		    				
uint32_t	TIM2CH2_CAPTURE_VAL;	
uint8_t  TIM2CH3_CAPTURE_STA=0,Channel3_ENABLE=0;		    				
uint32_t	TIM2CH3_CAPTURE_VAL;	
uint8_t  TIM2CH4_CAPTURE_STA=0,Channel4_ENABLE=0;		    				
uint32_t	TIM2CH4_CAPTURE_VAL;	
uint8_t FirstTrig=0;
int front = 0;
int right = 0; 
int left = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	long long temp;
//	float length;
//	uint8_t timeout;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	  GPIOA->MODER |= ((1 << 10) | (1 << 30) | (1 << 16) | (1 << 20));
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  delay_init(48);
  MX_GPIO_Init();
  //MX_USART1_UART_Init();
  //MX_TIM1_Init(0xFFFF,48-1); 
  MX_TIM2_Init(0XFFFFFFFF,48-1); 
  //MX_TIM3_Init(0xFFFF,48-1); 
  /* USER CODE BEGIN 2 */
  LED1_High;
  Channel2_ENABLE=1;
  printf("system start!\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */
	  //KeyProcess();
		
	  delay_ms(30);
		if(Channel2_ENABLE==1)
		{
			ChannelX_Start(2);
		}
		else if(Channel3_ENABLE==1)
		{
			ChannelX_Start(3);
		}
		else if(Channel4_ENABLE==1)
		{
			ChannelX_Start(4);
		}
		//delay_ms(300);
	  
		
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}


void motor_control (void)
{
	if (front == 0 && left == 0 && right == 0)
	{
			GPIOA->ODR |= (1 << 5);
			GPIOA->ODR &= ~(1 << 15);
			GPIOA->ODR |= (1 << 8);
			GPIOA->ODR &= ~(1 << 10);	
	}
		else if (right ==0)
	{
			GPIOA->ODR |= (1 << 5); // right wheel
			GPIOA->ODR &= ~(1 << 15);
			GPIOA->ODR &= ~(1 << 8); // left wheel
			GPIOA->ODR |= (1 << 10);	
			delay_ms(600);		
	}
	
	 if (left == 1)
	{
			GPIOA->ODR |= (1 << 5); // right wheel
			GPIOA->ODR &= ~(1 << 15);
			GPIOA->ODR &= ~(1 << 8); // left wheel
			GPIOA->ODR |= (1 << 10);		
			delay_ms(600);
		
	}
	 if (right == 1)
	{
		
			GPIOA->ODR &= ~(1 << 5); // right wheel
			GPIOA->ODR |= (1 << 15);
			GPIOA->ODR |= (1 << 8); // left wheel
			GPIOA->ODR &= ~(1 << 10);	
			delay_ms(600);		
		
	}

	
//	else if (right == 1)
//	{
//					GPIOA->ODR &= ~(1 << 5); // right wheel
//			GPIOA->ODR |= (1 << 15);
//			GPIOA->ODR |= (1 << 8); // left wheel
//			GPIOA->ODR &= ~(1 << 10);		
//		delay_ms(300);
//	}
//	
//	else if (left == 1)
//	{
//					GPIOA->ODR |= (1 << 5); // right wheel
//			GPIOA->ODR &= ~(1 << 15);
//			GPIOA->ODR &= ~(1 << 8); // left wheel
//			GPIOA->ODR |= (1 << 10);	
//delay_ms(300);		
//	}
	
	else
	{
			GPIOA->ODR |= (1 << 5);
			GPIOA->ODR &= ~(1 << 15);
			GPIOA->ODR |= (1 << 8);
			GPIOA->ODR &= ~(1 << 10);
	}
	
}

void Timer2_Length_Calc(uint8_t *state,uint32_t value,uint8_t channel)
{
	long long temp;
	float length;
	static uint8_t timeout=0;
	
	timeout++;
	
	if((*state)&0X80)        
	{
		//printf("3");
		timeout=0;
		temp=(*state)&0X3F; 
		temp*=0XFFFFFFFF;		 	    
		temp+=value;      
		printf("HIGH %lld us\r\n",temp);
		length = (float)temp*0.17;
		printf("Length = %f mm\r\n",length);
		
		if(length>450)
		{
			switch(channel)
			{
				case 2:
					LED2_High; // front
					front = 0;
					motor_control();
					break;
				case 3:
					LED3_High; //right
					right = 0;
					motor_control();
					break;
				case 4:
					LED4_High; // left
					left = 0;
					motor_control();
					break;
			}
			
		}
		else
		{
			switch(channel)
			{
				case 2:
					LED2_Low;
					front = 1;
					motor_control();
					break;
				case 3:
					LED3_Low;
					right = 1;
					motor_control();
					break;
				case 4:
					LED4_Low;
					left = 1;
					motor_control();
					break;
			}
			//LED2_Low;
		}
		
		*state=0;          
		
		switch(channel)
		{
			case 2:
				Channel2_ENABLE=0;
				Channel3_ENABLE=1;
				break;
			case 3:
				Channel3_ENABLE=0;
				Channel4_ENABLE=1;
				break;
			case 4:
				Channel4_ENABLE=0;
				Channel2_ENABLE=1;
				break;
		}
		
		FirstTrig=0;
		HAL_TIM_IC_Stop(&htim2,channel);
		delay_ms(10);
	}
	
	if(timeout==10)
	{
		timeout=0;
		printf("Timeout channel:%d\r\n",channel);
		switch(channel)
		{
			case 2:
				Channel2_ENABLE=0;
				Channel3_ENABLE=1;
				break;
			case 3:
				Channel3_ENABLE=0;
				Channel4_ENABLE=1;
				break;
			case 4:
				Channel4_ENABLE=0;
				Channel2_ENABLE=1;
				break;
		}
		
		FirstTrig=0;
		HAL_TIM_IC_Stop(&htim2,channel);
		delay_ms(10);
	}
}

void ChannelX_Start(uint8_t Channel)
{		
	if(FirstTrig==0)
	{
		switch(Channel)
		{
			case 2:
				HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
				break;
			case 3:
				HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_3);
				break;
			case 4:
				HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_4);
				break;
		}
		__HAL_TIM_ENABLE_IT(&htim2,TIM_IT_UPDATE);
		delay_ms(5);
		switch(Channel)
		{
			case 2:
				Sensor2_Trig();
				//printf("2");
				break;
			case 3:
				Sensor3_Trig();
				//printf("3");
				break;
			case 4:
				Sensor4_Trig();
				//printf("4");
				break;
		}
		FirstTrig=1;
	}
	
	switch(Channel)
	{
		case 2:
			Timer2_Length_Calc(&TIM2CH2_CAPTURE_STA,TIM2CH2_CAPTURE_VAL,Channel);
			break;
		case 3:
			Timer2_Length_Calc(&TIM2CH3_CAPTURE_STA,TIM2CH3_CAPTURE_VAL,Channel);
			break;
		case 4:
			Timer2_Length_Calc(&TIM2CH4_CAPTURE_STA,TIM2CH4_CAPTURE_VAL,Channel);
			break;
	}
//	Timer2_Length_Calc(&TIM2_CAPTURE_STA,TIM2_CAPTURE_VAL,Channel);
//	delay_ms(150);
//	Channel2_ENABLE=0;
//	HAL_TIM_IC_Stop(&htim2,TIM_CHANNEL_2);
//	delay_ms(100);
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim==(&htim2))
	{
		if(((TIM2CH2_CAPTURE_STA&0X80)==0)&&Channel2_ENABLE)
		{
			//printf("1\r\n");
			if(TIM2CH2_CAPTURE_STA&0X40)				
			{	  			
				TIM2CH2_CAPTURE_STA|=0X80;		
				 TIM2CH2_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2);
				 TIM_RESET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_2); 
				 TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_2,TIM_ICPOLARITY_RISING);
			}
			else  								
			{
				TIM2CH2_CAPTURE_STA=0;			
				TIM2CH2_CAPTURE_VAL=0;
				TIM2CH2_CAPTURE_STA|=0X40;		
				__HAL_TIM_DISABLE(&htim2);        
				__HAL_TIM_SET_COUNTER(&htim2,0);
				TIM_RESET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_2);   
				TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_2,TIM_ICPOLARITY_FALLING);
				__HAL_TIM_ENABLE(&htim2);
			}		    
		}
		else if(((TIM2CH3_CAPTURE_STA&0X80)==0)&&Channel3_ENABLE)
		{
			//printf("1\r\n");
			if(TIM2CH3_CAPTURE_STA&0X40)			
			{	  			
				TIM2CH3_CAPTURE_STA|=0X80;		
				 TIM2CH3_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3);
				 TIM_RESET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_3);   
				 TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_3,TIM_ICPOLARITY_RISING);
			}
			else  								
			{
				TIM2CH3_CAPTURE_STA=0;			
				TIM2CH3_CAPTURE_VAL=0;
				TIM2CH3_CAPTURE_STA|=0X40;		
				__HAL_TIM_DISABLE(&htim2);        
				__HAL_TIM_SET_COUNTER(&htim2,0);
				TIM_RESET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_3);   
				TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_3,TIM_ICPOLARITY_FALLING);
				__HAL_TIM_ENABLE(&htim2);
			}		    
		}	
		else if(((TIM2CH4_CAPTURE_STA&0X80)==0)&&Channel4_ENABLE)
		{
			//printf("1\r\n");
			if(TIM2CH4_CAPTURE_STA&0X40)				
			{	  			
				TIM2CH4_CAPTURE_STA|=0X80;		
				 TIM2CH4_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_4);
				 TIM_RESET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_4);   
				 TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_4,TIM_ICPOLARITY_RISING);
			}
			else  								
			{
				TIM2CH4_CAPTURE_STA=0;			
				TIM2CH4_CAPTURE_VAL=0;
				TIM2CH4_CAPTURE_STA|=0X40;		
				__HAL_TIM_DISABLE(&htim2);        
				__HAL_TIM_SET_COUNTER(&htim2,0);
				TIM_RESET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_4);   
				TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_4,TIM_ICPOLARITY_FALLING);
				__HAL_TIM_ENABLE(&htim2);
			}		    
		}	
	}
	
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==(&htim2))
	{
		if(((TIM2CH2_CAPTURE_STA&0X80)==0)&&Channel2_ENABLE)
		{
			//printf("3\r\n");
			if(TIM2CH2_CAPTURE_STA&0X40)
			{
				printf("2\r\n");
				if((TIM2CH2_CAPTURE_STA&0X3F)==0X3F)
				{
					printf("5\r\n");
					TIM2CH2_CAPTURE_STA|=0X80;		
					TIM2CH2_CAPTURE_VAL=0XFFFFFFFF;
				}else TIM2CH2_CAPTURE_STA++;
			}	 
		}
		else if(((TIM2CH3_CAPTURE_STA&0X80)==0)&&Channel3_ENABLE)
		{
			//printf("3\r\n");
			if(TIM2CH3_CAPTURE_STA&0X40)
			{
				printf("2\r\n");
				if((TIM2CH3_CAPTURE_STA&0X3F)==0X3F)
				{
					printf("5\r\n");
					TIM2CH3_CAPTURE_STA|=0X80;		
					TIM2CH3_CAPTURE_VAL=0XFFFFFFFF;
				}else TIM2CH3_CAPTURE_STA++;
			}	 
		}
		else if(((TIM2CH4_CAPTURE_STA&0X80)==0)&&Channel4_ENABLE)
		{
			//printf("3\r\n");
			if(TIM2CH4_CAPTURE_STA&0X40)
			{
				printf("2\r\n");
				if((TIM2CH4_CAPTURE_STA&0X3F)==0X3F)
				{
					printf("5\r\n");
					TIM2CH4_CAPTURE_STA|=0X80;		
					TIM2CH4_CAPTURE_VAL=0XFFFFFFFF;
				}else TIM2CH4_CAPTURE_STA++;
			}	 
		}
	}
	
}

/* TIM1 init function */
//static void MX_TIM1_Init(uint32_t arr,uint16_t psc)
//{

////  TIM_ClockConfigTypeDef sClockSourceConfig;
////  TIM_MasterConfigTypeDef sMasterConfig;
//  TIM_IC_InitTypeDef sConfigIC;

//  htim1.Instance = TIM1;
//  htim1.Init.Prescaler = psc;
//  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim1.Init.Period = arr;
//  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim1.Init.RepetitionCounter = 0;
//  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

////  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
////  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
////  {
////    _Error_Handler(__FILE__, __LINE__);
////  }

////  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
////  {
////    _Error_Handler(__FILE__, __LINE__);
////  }

////  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
////  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
////  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
////  {
////    _Error_Handler(__FILE__, __LINE__);
////  }

//  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
//  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
//  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
//  sConfigIC.ICFilter = 0;
//  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
//  
//  __HAL_TIM_CLEAR_FLAG(&htim1,TIM_IT_UPDATE);
//	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);   //开启TIM5的捕获通道1，并且开启捕获中断
//  __HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);   //使能更新中断

//}

/* TIM2 init function */
static void MX_TIM2_Init(uint32_t arr,uint16_t psc)
{

//  TIM_ClockConfigTypeDef sClockSourceConfig;
//  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = psc;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = arr;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

	__HAL_TIM_CLEAR_FLAG(&htim2,TIM_IT_UPDATE);

}

/* TIM3 init function */
//static void MX_TIM3_Init(uint32_t arr,uint16_t psc)
//{

////  TIM_ClockConfigTypeDef sClockSourceConfig;
////  TIM_MasterConfigTypeDef sMasterConfig;
//	TIM_IC_InitTypeDef sConfigIC;

//	htim3.Instance = TIM3;
//	htim3.Init.Prescaler = psc;
//	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
//	htim3.Init.Period = arr;
//	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
//	{
//		_Error_Handler(__FILE__, __LINE__);
//	}

//	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
//	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
//	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
//	sConfigIC.ICFilter = 0;
//	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
//	{
//		_Error_Handler(__FILE__, __LINE__);
//	}
//	__HAL_TIM_CLEAR_FLAG(&htim3,TIM_IT_UPDATE);

//}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
  /* GPIO Ports Clock Enable */
  //__HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_10|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_10|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  /*Configure GPIO pins : PA0*/
//  GPIO_InitStruct.Pin = GPIO_PIN_0;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  //HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;  
void _ttywrch(int y)
{
	y = y;
}
   
void _sys_exit(int x) 
{ 
	x = x; 
} 

int fputc(int ch, FILE *f)
{ 	
	while((USART1->ISR&0X40)==0); 
	USART1->TDR = (uint8_t) ch;      
	return ch;
}
#endif 

static uint32_t fac_us=0;

void delay_init(uint8_t SYSCLK)
{
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	fac_us=SYSCLK;						

}	

void delay_us(uint32_t nus)
{		
	uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload=SysTick->LOAD;				   	 
	ticks=nus*fac_us; 					
	told=SysTick->VAL;        				
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			
		}  
	};
}


void delay_ms(uint16_t nms)
{
	uint32_t i;
	for(i=0;i<nms;i++) delay_us(1000);
}


uint8_t KEY_Scan(uint8_t mode)
{	 
	static uint8_t key_up=1;
	if(mode)key_up=1;  	

	if(key_up&&(KEY1==1))
	{
		delay_ms(10);
		key_up=0;
		if(KEY1==1)return 1;
	}
	else if(KEY1==0)key_up=1; 	
	
 	return 0;
}

void KeyProcess(void)
{
	uint8_t KEY;
	KEY = KEY_Scan(0);
  if(KEY)
	{						   
		switch(KEY)
		{				 
			case 1:	
				//printf("1");
				LED1_Toggle;
//				PWM_High;
//				delay_ms(20);
//				PWM_Low;
				break;
			default :
				break;
		}
	}
}

void Sensor4_Trig(void)
{
	PWM1_High;
	delay_us(20);
	PWM1_Low;
}

void Sensor2_Trig(void)
{
	PWM_High;
	delay_us(20);
	PWM_Low;
}

void Sensor3_Trig(void)
{
	PWM2_High;
	delay_us(20);
	PWM2_Low;
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
