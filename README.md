# README

###### 修改时间 4.7 10:30 修改人:xbq

======================================

在项目中加上了源文件和头文件，稍微更改了下头文件的include后新增40+error

改动为

```C
//inc文件夹中的各头文件中
#include "stm32f1xx_hal.h" >>>>> #include "stm32f7xx_hal.h"
```

======================================

用cubemx把PE12作为KEY4，然后修改key.c源文件里的代码

修改部分如下:

```C
//修改前
#define S1  HAL_GPIO_ReadPin(KEY1_IN_GPIO_Port,KEY1_IN_Pin)
#define S2  HAL_GPIO_ReadPin(KEY2_IN_GPIO_Port,KEY2_IN_Pin)
#define S3  HAL_GPIO_ReadPin(KEY3_IN_GPIO_Port,KEY3_IN_Pin)
#define S4  HAL_GPIO_ReadPin(KEY4_IN_GPIO_Port,KEY4_IN_Pin)
//修改后
#define S1  HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)
#define S2  HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin)
#define S3  HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin)
#define S4  HAL_GPIO_ReadPin(KEY4_GPIO_Port,KEY4_Pin)
```

======================================

修改led.c中的函数,使得pin和port符合新设置的名字

```c
LED1_Pin >>>>> LED_R_Pin
LED2_Pin >>>>> LED_G_Pin
LED3_Pin >>>>> LED_B_Pin
LED1_GPIO_Port >>>>> LED_R_GPIO_Port
LED2_GPIO_Port >>>>> LED_G_GPIO_Port
LED3_GPIO_Port >>>>> LED_B_GPIO_Port
```

======================================

将v2版本里的main.c文件用户自己编写代码复制到新版本中对应位置后

截止到目前编译会报错 37error, 79warning,似乎出现了很多函数重定义问题

在main.c中添加htim1定义：
//修改前
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
//修改后
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

还剩两个error；