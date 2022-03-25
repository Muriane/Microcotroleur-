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
#include "usart.h"
#include "gpio.h"
#include "led.h"
#include "button.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
LED_TypeDef led;
LED_TypeDef led2;
BUTTON_TypeDef bouton;
BUTTON_TypeDef bouton2;
uint8_t last_button_state=1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Bouton_Led_Polling(uint8_t *last_button_state, LED_TypeDef *led, BUTTON_TypeDef *bouton){
	uint8_t etat_courant_bouton= (Button_State(bouton))? 1:0;
	if ((*last_button_state==0) && (etat_courant_bouton!=0)){
		Led_toggle(led);
		LL_mDelay(20); //attendre 20ms
	}
	//si le bouton vient d’être relâché faire une attente
	if ((*last_button_state!=0) && (etat_courant_bouton==0)){
		LL_mDelay(20); //attendre 20ms
	}
	*last_button_state= etat_courant_bouton;
}
void EXTI2_3_IRQHandler(){
//s’il y une interruption en attente
	if ( EXTI->PR&(1<<3)){
		GPIOC->ODR^=(1<<7);
//il ne faut pas oublier de réinitialiser le flag
		EXTI->PR|=(1<<12);
	}
}
void TIM_config(TIM_TypeDef * timer, uint32_t HCLKFrequency, uint8_t nb_s){
	if(timer==TIM22){
		RCC->APB2ENR |= RCC_APB2ENR_TIM22EN;
	}
	else if(timer==TIM21){
			RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;
	}
	else if(timer==TIM2){
			RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	}
	else if(timer==TIM6){
			RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	}
	timer->CR1|=TIM_CR1_DIR_Msk;
	timer->PSC=999;
	timer->DIER|=TIM_DIER_UIE_Msk;
	if(timer==TIM22){
			NVIC_EnableIRQ(TIM22_IRQn);
		}
		else if(timer==TIM21){
			NVIC_EnableIRQ(TIM21_IRQn);
		}
		else if(timer==TIM2){
			NVIC_EnableIRQ(TIM2_IRQn);
		}
		else if(timer==TIM6){
			NVIC_EnableIRQ(TIM6_IRQn);
		}

	timer->CR1|=TIM_CR1_CEN_Msk;
}
void TIM21_IRQHandler(LED_TypeDef *led){
	TIM21->SR = TIM21->SR & ~TIM_SR_UIF_Msk;
	Led_toggle(led);
}
void TIM2_IRQHandler(LED_TypeDef *led){
TIM2->SR = TIM2->SR & ~TIM_SR_UIF_Msk;
Led_toggle(led);
}

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

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Led_init(&led,GPIOA,5);
  Button_init(&bouton, GPIOC, 13, LL_GPIO_PULL_NO);
  Led_init(&led2,GPIOC,7);
  Button_init(&bouton2, GPIOB, 3, LL_GPIO_PULL_DOWN);
  Button_enableIRQ(&bouton2,LL_EXTI_TRIGGER_RISING);
  TIM_config(TIM21, 16000000, 1);
  TIM_config(TIM2, 16000000, 2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  /*if(Led_isOn(&led)){
		  Led_turnOff(&led);
		  LL_mDelay(1000);
	  }
	  else if(Led_isOff(&led)){
		  Led_turnOn(&led);
		  LL_mDelay(1000);
	  }*/
	  //last_button_state =Button_State(&bouton);
	  //Bouton_Led_Polling(&last_button_state,&led,&bouton);

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }

  LL_Init1msTick(16000000);

  LL_SetSystemCoreClock(16000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
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

