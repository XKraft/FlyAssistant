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

###### 修改时间：4/7 - 17：00      修改人：yy

概述：去掉了DSP库&在main.h中加上了ARM_MATH_CM7宏定义

###### 修改时间 4.7 19:50 修改人:xbq

======================================

修改main.c里面使能PWM部分，大概在280行左右，因为新版中，cubemx中配置tim2和tim3的8个通道输入pwm信号。

```C
//修改前
		HAL_TIM_PWM_Init(&htim2);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
		
		HAL_TIM_PWM_Init(&htim4);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
//修改后
		HAL_TIM_PWM_Init(&htim2);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
		
		HAL_TIM_PWM_Init(&htim3);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
```

修改main.c里面使能定时器输入捕获部分，大概在290行左右，因为新版中，cubemx中配置tim4和tim5的8个通道进行定时器输入捕获。

```c
//修改前
		HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
		HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);
		
		HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
		HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
		HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_3);
		HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_4);
//修改后
		HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);
		HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_2);
		HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_3);
		HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_4);
		
		HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_1);
		HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_2);
		HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_3);
		HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_4);
```

======================================

把v2版本中/* USER CODE BEGIN 4 */部分复制粘贴到新版本中，约从1262行开始

修改定时器终端服务程序HAL_TIM_PeriodElapsedCallback函数，将原本的tim1和time4改成现在的tim4和tim3

更改开头处函数的声明，将TIM1_Set和TIM4_Set改为TIM4_Set和TIM3_Set,查找所有调用这些函数的地方,都要改一下.

在定时器输入捕获函数HAL_TIM_IC_CaptureCallback中，将htim3全部换成htim4（中间发现几处漏打&的地方，不知道是本来就应该那样还是漏了，不过我改成带&的了，因为那个函数要的参数是指针类型）。将htim1全部换成htim5，并将缺少的通道补回来。

为此，还需要在main.c开头63行左右新增一些变量,在添加代码时要用

```c
//增加内容
int CHANNEL_7_RISE=0,CHANNEL_7_FALL=0,CHANNEL_7_PULSE_WIDE=0;
int CHANNEL_8_RISE=0,CHANNEL_8_FALL=0,CHANNEL_8_PULSE_WIDE=0;

int ICFLAG_1 = 1,ICFLAG_2 = 1,ICFLAG_3 = 1, ICFLAG_4 = 1, ICFLAG_5 = 1, ICFLAG_6 = 1, ICFLAG_7 = 1, ICFLAG_8 = 1;
```

======================================

发现新版的cubemx里tim2-5没有使能中断,应该要使能的吧,所以这里勾选上重新generate了一下.

![image-20220407193041758](C:\Users\26281\AppData\Roaming\Typora\typora-user-images\image-20220407193041758.png)