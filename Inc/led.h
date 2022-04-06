#ifndef  __LED_H__
#define  __LED_H__
#include "stm32f1xx_hal.h"

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Initialization
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void LED1_Flash(void);
void LED2_Flash(void); 
void LED3_Flash(void); 

void LED1_Slow_Flash(void);
void LED2_Slow_Flash(void); 
void LED3_Slow_Flash(void); 

void LED1_On(void);
void LED1_Off(void);

void LED2_On(void);
void LED2_Off(void);
void LED3_Off(void);

//*****ÉÁË¸********
void LED_R_Flash(void);
void LED_G_Flash(void);
void LED_B_Flash(void);

//****´ò¿ª********
void LED_R_On(void);
void LED_G_On(void);
void LED_B_On(void);

//*******¹Ø±Õ***********
void LED_R_Off(void);
void LED_G_Off(void);
void LED_B_Off(void);

#endif


