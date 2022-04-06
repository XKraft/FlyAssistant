#include "main.h"
#include "stdio.h"
#include "flyControl.h"
#include "coderDecoder.h"
#include "arm_math.h"


int PWM_Pitch_Max=6000;
int PWM_Pitch_mid=4500;
int PWM_Pitch_min=3000;

int PWM_Roll_Max=6000;
int PWM_Roll_mid=4500;
int PWM_Roll_min=3000;

int PWM_Thr_Max=6000;
int PWM_Thr_mid=4500;
int PWM_Thr_min=3000;

int PWM_yaw_Max=6000;
int PWM_yaw_mid=4500;
int PWM_yaw_min=3000;

int PWM_Mode_Max=6000;
int PWM_Mode_mid=4500;
int PWM_Mode_min=3000;

int DeadThreShold=2;

int PWM_Thr_min_LOCK =1101;

int          pwm_pitch_out=0;
int          pwm_roll_out=0;

unsigned char  SetHigh=80;
uint8_t InitedFlag;

PID         PID_Control_Pitch;
PID         PID_Control_Roll;
/*==================================================================================================== 
    PID Function 
     
    The PID (比例、积分、微分) function is used in mainly 
    control applications. PIDCalc performs one iteration of the PID 
    algorithm. 

    While the PID function works, main is just a dummy program showing 
    a typical usage. 
=====================================================================================================*/ 


//*********设备初始化********//
void Device_Init(void)
{
	for(int i = 0 ;i<2;i++)
	{
		 HAL_Delay(50);
	   LED1_Flash();
     LED2_Flash();
		 LED3_Flash();
		 BEEP_ON();
		 HAL_Delay(100);
		 BEEP_OFF();
		 InitedFlag =1;
		 LED_R_Off();
		 LED_G_Off();
		 LED_B_Off();
	}
	Set_PWM_Thr(PWM_Thr_min_LOCK);	
	Set_PWM_Pitch(PWM_Pitch_mid);
	Set_PWM_Roll(PWM_Roll_mid);
  Set_PWM_Yaw(PWM_yaw_mid);
  Set_PWM_Mode(PWM_Mode_mid);
}


///*==================================================================================================== 
//   PID计算部分 
//=====================================================================================================*/ 

//int PIDCalc( PID *pp, int CurrentPoint ,int SetPoint ) 
//{ 
//	  float out;  
//		float dError, Error; 
//		pp->SetPoint =SetPoint;

//		Error = (float)pp->SetPoint -  (float)CurrentPoint; //计算偏差
//		pp->SumError += Error;                              // 积分 
//		pp->LastError = Error; 		                          //更新状态
//		dError = pp->PreviousError - pp->LastError; // 当前微分 
//		out =   pp->Proportion * Error              // 比例项 
//				+   pp->Integral * pp->SumError         // 积分项 
//				+   (pp->Derivative * dError)* (-0.5*exp(-0.6*(Error*Error))+1);  // 微分项 
//		//计算输出,限幅 
//		out = range(out, -pp->Pid_Clc_limit, pp->Pid_Clc_limit);

//		pp->PreviousError = pp->LastError; 		     //更新状态
//		return (int)out;
//} 

/*==================================================================================================== 
   PID计算部分 
=====================================================================================================*/ 
int PIDCalc( PID *pp, int CurrentPoint ,int SetPoint ) 
{ 
	  float out;  
		float dError, Error; 
		pp->SetPoint =SetPoint;

		Error = (float)pp->SetPoint -  (float)CurrentPoint; //计算偏差
		pp->SumError += Error;                              // 积分 
		dError = pp->PreviousError - pp->LastError; // 当前微分 
		pp->LastError = Error; 		                  //更新状态
		pp->PreviousError = pp->LastError; 		      //更新状态
		out =   pp->Proportion * Error              // 比例项 
				+   pp->Integral * pp->SumError         // 积分项 
				+   pp->Derivative * dError;  // 微分项 
		//计算输出,限幅 
		out = range(out, -pp->Pid_Clc_limit, pp->Pid_Clc_limit);
		return (int)out;
} 

/*==================================================================================================== 
   Initialize PID Structure 
=====================================================================================================*/ 

void delay1ms(int time)
{
  HAL_Delay(time);
}


void PIDInit (PID *pp) 
{ 
	  PID_Control_Pitch.Pid_Clc_limit = 300;
    PID_Control_Pitch.Proportion    = 2.3;     
    PID_Control_Pitch.Integral      = 0; 
    PID_Control_Pitch.Derivative    = 0.5; 
    PID_Control_Pitch.SetPoint      = 120;     
	  PID_Control_Pitch.LastError     = 0;  
    PID_Control_Pitch.PreviousError = 0;    
    PID_Control_Pitch.SumError      = 0;      	
	
		PID_Control_Roll.Pid_Clc_limit = 300;
    PID_Control_Roll.Proportion    = 2.3;     
    PID_Control_Roll.Integral      = 0; 
    PID_Control_Roll.Derivative    = 0.5; 
    PID_Control_Roll.SetPoint      = 160;  
	  PID_Control_Roll.LastError     = 0;  
    PID_Control_Roll.PreviousError = 0;    
    PID_Control_Roll.SumError      = 0;     	
} 

/*==================================================================================================== 
   解锁 
=====================================================================================================*/ 
void Unlock(void)
{
	 //**********油门最低值***********//
	 Set_PWM_Thr(PWM_Thr_min_LOCK); //1071------>1101
	 HAL_Delay(2000);
	 for(int i=PWM_yaw_mid;i<=PWM_yaw_Max;i=i+86)
	 {
		LED1_Flash();
		Set_PWM_Yaw(i);   //解锁
	 }
	 delay1ms(3000);
	 for(int i =PWM_yaw_Max;i>=PWM_yaw_mid;i=i-86)
	 {
		LED1_Flash();
		Set_PWM_Yaw(i);   //回中
	 }
	 delay1ms(1000);
	 printf("*******************Unlock***************\r\n");
}



/*==================================================================================================== 
   加锁
=====================================================================================================*/ 
void Lock(void)
{
 BEEP_ON();
 HAL_Delay(3000);
 BEEP_OFF();
 HAL_Delay(1000);
 //**********油门最低值***********//
 Set_PWM_Thr(PWM_Thr_min_LOCK); 
 HAL_Delay(1000);
 //***********偏航向左*************//
 for(int i=PWM_yaw_mid;i>=PWM_yaw_min;i=i-128)
 {
	 LED1_Flash();
   Set_PWM_Yaw(i);   
 }
 //***********偏航回中*************//
 delay1ms(5000);
 for(int i=PWM_yaw_min;i<=PWM_yaw_mid;i=i+128)
 {
	 LED1_Flash();
   Set_PWM_Yaw(i);   //回中
 }
 delay1ms(5000);
 printf("*******************lock***************\r\n");
}

/*==================================================================================================== 
   起飞准备
=====================================================================================================*/ 
void Take_off_Preper(void)
{	
	 printf("*******************Take_off_Preper***************\r\n");
}


/*==================================================================================================== 
   起飞 Fly_Moder_AltHold
=====================================================================================================*/ 
void Take_off(float high)
{	 
	 AltAdj(high);
	 printf("*******************Take_off***************\r\n");
}


/*==================================================================================================== 
   降落
=====================================================================================================*/ 
void land(void)
{
	 printf("*******************Land***************\r\n");
}

/*==================================================================================================== 
   前进
=====================================================================================================*/ 
void Go_ahead(uint16_t pulse)
{
	Set_PWM_Pitch(PWM_Pitch_mid-pulse);      
}

/*==================================================================================================== 
   后退
=====================================================================================================*/ 
void Go_back(uint16_t pulse)
{
	Set_PWM_Pitch(PWM_Pitch_mid-pulse);      
}

/*==================================================================================================== 
   向左飞
=====================================================================================================*/ 
void Go_left(uint16_t pulse)
{
	Set_PWM_Roll(PWM_Roll_mid-pulse);      
}

/*==================================================================================================== 
   向右飞
=====================================================================================================*/ 
void Go_right(uint16_t pulse)
{
	Set_PWM_Roll(PWM_Roll_mid+pulse);      
}

/*==================================================================================================== 
   左转
=====================================================================================================*/ 
void Turn_left(uint16_t pulse)
{
	Set_PWM_Yaw(PWM_yaw_mid-pulse);      
}

/*==================================================================================================== 
   右转
=====================================================================================================*/ 
void Turn_right(uint16_t pulse)
{
	Set_PWM_Yaw(PWM_yaw_mid+pulse);      
}


/*==================================================================================================== 
   飞行模式
=====================================================================================================*/ 
void Fly_Moder(int16_t FlyMode)
 {
	 BEEP_ON();
   HAL_Delay(500);
   BEEP_OFF();
	 HAL_Delay(100);
	 if(FlyMode==Fly_Moder_Stabilize)
	 {
	  Set_PWM_Mode(PWM_Mode_min); 
		HAL_Delay(20);
		printf("*******************Fly_Moder_Stabilize***************\r\n");
	 }
   if(FlyMode==Fly_Moder_AltHold)
	 {
	  Set_PWM_Mode(PWM_Mode_mid); 
		HAL_Delay(20);
		printf("*******************Fly_Moder_AltHold***************\r\n");
	 }
	 if(FlyMode==Fly_Moder_Loiter)
	 {
	  Set_PWM_Mode(PWM_Mode_min); 
		HAL_Delay(20);
		printf("*******************Fly_Moder_Loiter***************\r\n");
	 }
	 
	 if(FlyMode==Fly_Moder_LAND)
	 {
	  Set_PWM_Mode(PWM_Mode_Max); 
		HAL_Delay(5000);
		land();
		printf("*******************Fly_Moder_LAND***************\r\n");
	 }
 }

 
/*==================================================================================================== 
   悬停
=====================================================================================================*/ 
void Loiter(int point_x,int point_y,int SetPoint_x,int SetPoint_y)
{  
		//printf("*******************Loiter***************\r\n");
	  int pwm_pitch_clc=0;
	  int pwm_roll_clc=0;
		int deadZoneX = 0;
	  int deadZoneY = 0;
	
		deadZoneX = point_x - SetPoint_x;
	  deadZoneY = point_y - SetPoint_y;
	
		if(myabs(deadZoneX)>=DeadThreShold)
		{
			pwm_roll_clc  = PIDCalc(&PID_Control_Roll  ,point_x,SetPoint_x);
			pwm_roll_out  = PWM_Roll_mid  - pwm_roll_clc;
		  Set_PWM_Roll (pwm_roll_out);
		}
		if(myabs(deadZoneX)<DeadThreShold)
		{
			Set_PWM_Roll(PWM_Roll_mid);
		}
		//pitch 方向
		if(myabs(deadZoneY)>=DeadThreShold)
		{
			pwm_pitch_clc = PIDCalc(&PID_Control_Pitch ,point_y,SetPoint_y);
			pwm_pitch_out = PWM_Pitch_mid - pwm_pitch_clc;
			Set_PWM_Pitch(pwm_pitch_out);
		}
		if(myabs(deadZoneY) < DeadThreShold)
		{
			Set_PWM_Pitch(PWM_Pitch_mid);
		}
		printf("pwm_pitch_out = %d, pwm_roll_out = %d\r\n ", pwm_pitch_out, pwm_roll_out);
}

/*==================================================================================================== 
   高度调整
=====================================================================================================*/ 
void AltAdj(float high)
{	 
	 Set_PWM_Thr(PWM_Thr_mid);
	 printf("******************高度调整**************\r\n");
}



/*==================================================================================================== 
   PWM底层控制
=====================================================================================================*/ 
//***********油门*********************************************//
void Set_PWM_Thr(uint16_t pulse)
{
  TIM2 ->CCR1 =pulse ;
}

//***********俯仰**********************************************//
void Set_PWM_Pitch(uint16_t pulse)
{
  TIM2 ->CCR2 =pulse ;
}


//***********横滚*********************************************//
void Set_PWM_Roll(uint16_t pulse)
{
  TIM2 ->CCR3 =pulse ;
}

//***********偏航**********************************************//
void Set_PWM_Yaw(uint16_t pulse)
{
  TIM2 ->CCR4 =pulse ;
}

//***********模式**********************************************//
void Set_PWM_Mode(uint16_t pulse)
{
  TIM4 ->CCR1 =pulse ;
}

//***********直通模式*********************************************//
void RC_bridge(void)
{
//	printf("1= %d 2= %d 3= %d 4= %d 5= %d 6= %d\r\n",CHANNEL_1_PULSE_WIDE, CHANNEL_2_PULSE_WIDE, CHANNEL_3_PULSE_WIDE, CHANNEL_4_PULSE_WIDE, CHANNEL_5_PULSE_WIDE, CHANNEL_6_PULSE_WIDE); 
//	printf("CHANNEL_1_PULSE_WIDE= %d \r\n",CHANNEL_1_PULSE_WIDE); 
//	printf("CHANNEL_2_PULSE_WIDE= %d \r\n",CHANNEL_2_PULSE_WIDE); 
//	printf("CHANNEL_3_PULSE_WIDE= %d \r\n",CHANNEL_3_PULSE_WIDE); 
//	printf("CHANNEL_4_PULSE_WIDE= %d \r\n",CHANNEL_4_PULSE_WIDE); 
//	printf("CHANNEL_5_PULSE_WIDE= %d \r\n",CHANNEL_5_PULSE_WIDE); 
//	printf("CHANNEL_6_PULSE_WIDE= %d \r\n",CHANNEL_6_PULSE_WIDE); 
  Set_PWM_Thr(CHANNEL_1_PULSE_WIDE);
	Set_PWM_Pitch(CHANNEL_2_PULSE_WIDE);
	Set_PWM_Roll(CHANNEL_3_PULSE_WIDE);
	Set_PWM_Yaw(CHANNEL_4_PULSE_WIDE);
	Set_PWM_Mode(CHANNEL_5_PULSE_WIDE);
}


void RC_bridge_Test(void)
{
	printf("1= %d 2= %d 3= %d 4= %d 5= %d 6= %d\r\n",CHANNEL_1_PULSE_WIDE, CHANNEL_2_PULSE_WIDE, CHANNEL_3_PULSE_WIDE, CHANNEL_4_PULSE_WIDE, CHANNEL_5_PULSE_WIDE, CHANNEL_6_PULSE_WIDE); 
	Set_PWM_Thr(CHANNEL_1_PULSE_WIDE);
	Set_PWM_Pitch(CHANNEL_2_PULSE_WIDE);
	Set_PWM_Roll(CHANNEL_3_PULSE_WIDE);
	Set_PWM_Yaw(CHANNEL_4_PULSE_WIDE);
	Set_PWM_Mode(CHANNEL_5_PULSE_WIDE);
}

