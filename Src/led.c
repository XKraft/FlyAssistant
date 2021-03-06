#include "led.h"
#include "main.h"

#define LED1_ON   HAL_GPIO_WritePin(GPIOE, LED_R_Pin, GPIO_PIN_SET);
#define LED1_OFF  HAL_GPIO_WritePin(GPIOE, LED_R_Pin, GPIO_PIN_RESET);
#define LED2_ON   HAL_GPIO_WritePin(GPIOE, LED_G_Pin, GPIO_PIN_SET);
#define LED2_OFF  HAL_GPIO_WritePin(GPIOE, LED_G_Pin, GPIO_PIN_RESET);

void LED1_Flash(void)
{
  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
}

void LED2_Flash(void)
{
  HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
}

void LED3_Flash(void)
{
  HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
}


void LED1_On(void)
{
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
}


void LED2_On(void)
{
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
}

void LED3_On(void)
{
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
}


void LED1_Off(void)
{
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
}

void LED2_Off(void)
{
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
}

void LED3_Off(void)
{
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
}


void LED1_Slow_Flash(void)
{
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
}

void LED2_Slow_Flash(void)
{
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
}

void LED3_Slow_Flash(void)
{
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
}


//*****************顶层应用****************************
void LED_R_Flash(void)
{
  LED1_Flash();
}

void LED_G_Flash(void)
{
  LED2_Flash();
}

void LED_B_Flash(void)
{
  LED3_Flash();
}

//****打开********
void LED_R_On(void)
{
  LED1_On();
}

void LED_G_On(void)
{
  LED2_On();
}

void LED_B_On(void)
{
  LED3_On();
}

//*******关闭***********
void LED_R_Off(void)
{
	 LED1_Off();
}

void LED_G_Off(void)
{
	 LED2_Off();
}

void LED_B_Off(void)
{
	 LED3_Off();
}
