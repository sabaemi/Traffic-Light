/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#include "LiquidCrystal.h"
void numbertobcd(int i){
int x1=i&1;
int x2=i&2;
int x3=i&4;
int x4=i&8;
if(x1>0)
	x1=1;
if(x2>0)
	x2=1;
if(x3>0)
	x3=1;
if(x4>0)
	x4=1;
HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,x1);
HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,x2);
HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,x3);
HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,x4);

}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
	LiquidCrystal(GPIOD,GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6);
	setCursor(0,0);
	print("First is RED");
	setCursor(0,1);
	//print("");
	print("Second is GREEN");
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,0);
	
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1);
	HAL_TIM_Base_Stop(&htim4);
	
  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles EXTI line1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */
LiquidCrystal(GPIOD,GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6);
	setCursor(0,0);
	print("First is YELLOW");
	setCursor(0,1);
	//print("");
	print("Second is RED");
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,0);
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,1);
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1);
	HAL_TIM_Base_Stop(&htim4);
  /* USER CODE END EXTI1_IRQn 1 */
}

/**
* @brief This function handles EXTI line2 interrupt.
*/
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */
LiquidCrystal(GPIOD,GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6);
	setCursor(0,0);
	print("First is GREEN");
	setCursor(0,1);
	//print("");
	print("Second is RED");
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,0);
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,1);
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1);
	HAL_TIM_Base_Stop(&htim4);
  /* USER CODE END EXTI2_IRQn 1 */
}

/**
* @brief This function handles EXTI line3 interrupt.
*/
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */
LiquidCrystal(GPIOD,GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6);
	setCursor(0,0);
	//print("");
	print("First is GREEN");
	setCursor(0,1);
	print("Second is RED");
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,1);
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1);
	HAL_TIM_Base_Stop(&htim4);
  /* USER CODE END EXTI3_IRQn 1 */
}

/**
* @brief This function handles EXTI line4 interrupt.
*/
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
LiquidCrystal(GPIOD,GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6);
	setCursor(0,0);
	//print("");
	print("First is RED");
	setCursor(0,1);
	print("Second is YELLOW");
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,0);
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,1);
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
	HAL_TIM_Base_Stop(&htim4);
  /* USER CODE END EXTI4_IRQn 1 */
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
LiquidCrystal(GPIOD,GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6);
	setCursor(0,0);
	//print("");
	print("First is RED");
	setCursor(0,1);
	print("Second is GREEN");
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,1);
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1);
	HAL_TIM_Base_Stop(&htim4);
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
LiquidCrystal(GPIOD,GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6);
	if(counter==40){
		//zrd1 khamush
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
		counter1=bist;
		//ghermez2 khamush
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
		counter2=punza;
		counter=0;
		setCursor(0,0);
			print("First is RED");
		setCursor(0,1);
		print("Second is GREEN");
	}
	if(counter>35 && counter<40){
	setCursor(0,0);
			print("First is YELLOW");
		setCursor(0,1);
		print("Second is RED");
	}
	if(counter==35){
		//sabz1 khamush
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
		counter1=pan;
	setCursor(0,0);
			print("First is YELLOW");
		setCursor(0,1);
		print("Second is RED");
	}
	if(counter>20 && counter<35){
	setCursor(0,0);
			print("First is GREEN");
		setCursor(0,1);
		print("Second is RED");
	}
	if(counter==20){
		//ghermez1 khamush 
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2);
		counter1=punza;
		//zard2 khamush
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
		counter2=bist;
		setCursor(0,0);
			print("First is GREEN");
		setCursor(0,1);
		print("Second is RED");
	}
	if(counter>15 && counter<20){
	setCursor(0,0);
			print("First is RED");
		setCursor(0,1);
		print("Second is YELLOW");
	}
	if(counter==15){
		//sabz2 khamush 
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
		setCursor(0,0);
			print("First is RED");
		setCursor(0,1);
		print("Second is YELLOW");
		counter2=pan;
	}
	if(counter<15 && counter>0){
	setCursor(0,0);
			print("First is RED");
		setCursor(0,1);
		print("Second is GREEN");
	}
		
		counter++;
	
	numbertobcd(counter1/10);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,0);
	
	numbertobcd(counter1%10);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,0);
	
		counter1--;
				numbertobcd(counter2/10);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,0);
			
		numbertobcd(counter2%10);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,0);
	
		counter2--;
  /* USER CODE END TIM4_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	LiquidCrystal(GPIOD,GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6);

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1);
	HAL_TIM_Base_Stop(&htim4);
	
	
extern unsigned char data[1];
	//TODO
	switch(data[0]){
	
		case '0':
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
			setCursor(0,0);
	print("First is RED");
	setCursor(0,1);
	print("Second is GREEN");
		break;
		
		case '1':
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
			setCursor(0,0);
	print("First is YELLOW");
	setCursor(0,1);
	print("Second is RED");
		break;
		
		case '2':
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
			setCursor(0,0);
	print("First is GREEN");
	setCursor(0,1);
	print("Second is RED");
		break;
		
		case '3':
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
			setCursor(0,0);
	print("First is GREEN");
	setCursor(0,1);
	print("Second is RED");
		break;
		
		case '4':
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
			setCursor(0,0);
	print("First is RED");
	setCursor(0,1);
	print("Second is YELLOW");
		break;
		
		case '5':
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
			setCursor(0,0);
	print("First is RED");
	setCursor(0,1);
	print("Second is GREEN");
		break;
	}
	HAL_UART_Receive_IT(&huart1,data,sizeof(data));
  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
