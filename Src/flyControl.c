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
     
    The PID (���������֡�΢��) function is used in mainly 
    control applications. PIDCalc performs one iteration of the PID 
    algorithm. 

    While the PID function works, main is just a dummy program showing 
    a typical usage. 
=====================================================================================================*/ 


//*********�豸��ʼ��********//
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
//   PID���㲿�� 
//=====================================================================================================*/ 

//int PIDCalc( PID *pp, int CurrentPoint ,int SetPoint ) 
//{ 
//	  float out;  
//		float dError, Error; 
//		pp->SetPoint =SetPoint;

//		Error = (float)pp->SetPoint -  (float)CurrentPoint; //����ƫ��
//		pp->SumError += Error;                              // ���� 
//		pp->LastError = Error; 		                          //����״̬
//		dError = pp->PreviousError - pp->LastError; // ��ǰ΢�� 
//		out =   pp->Proportion * Error              // ������ 
//				+   pp->Integral * pp->SumError         // ������ 
//				+   (pp->Derivative * dError)* (-0.5*exp(-0.6*(Error*Error))+1);  // ΢���� 
//		//�������,�޷� 
//		out = range(out, -pp->Pid_Clc_limit, pp->Pid_Clc_limit);

//		pp->PreviousError = pp->LastError; 		     //����״̬
//		return (int)out;
//} 

/*==================================================================================================== 
   PID���㲿�� 
=====================================================================================================*/ 
int PIDCalc( PID *pp, int CurrentPoint ,int SetPoint ) 
{ 
	  float out;  
		float dError, Error; 
		pp->SetPoint =SetPoint;

		Error = (float)pp->SetPoint -  (float)CurrentPoint; //����ƫ��
		pp->SumError += Error;                              // ���� 
		dError = pp->PreviousError - pp->LastError; // ��ǰ΢�� 
		pp->LastError = Error; 		                  //����״̬
		pp->PreviousError = pp->LastError; 		      //����״̬
		out =   pp->Proportion * Error              // ������ 
				+   pp->Integral * pp->SumError         // ������ 
				+   pp->Derivative * dError;  // ΢���� 
		//�������,�޷� 
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
   ���� 
=====================================================================================================*/ 
void Unlock(void)
{
	 //**********�������ֵ***********//
	 Set_PWM_Thr(PWM_Thr_min_LOCK); //1071------>1101
	 HAL_Delay(2000);
	 for(int i=PWM_yaw_mid;i<=PWM_yaw_Max;i=i+86)
	 {
		LED1_Flash();
		Set_PWM_Yaw(i);   //����
	 }
	 delay1ms(3000);
	 for(int i =PWM_yaw_Max;i>=PWM_yaw_mid;i=i-86)
	 {
		LED1_Flash();
		Set_PWM_Yaw(i);   //����
	 }
	 delay1ms(1000);
	 printf("*******************Unlock***************\r\n");
}



/*==================================================================================================== 
   ����
=====================================================================================================*/ 
void Lock(void)
{
 BEEP_ON();
 HAL_Delay(3000);
 BEEP_OFF();
 HAL_Delay(1000);
 //**********�������ֵ***********//
 Set_PWM_Thr(PWM_Thr_min_LOCK); 
 HAL_Delay(1000);
 //***********ƫ������*************//
 for(int i=PWM_yaw_mid;i>=PWM_yaw_min;i=i-128)
 {
	 LED1_Flash();
   Set_PWM_Yaw(i);   
 }
 //***********ƫ������*************//
 delay1ms(5000);
 for(int i=PWM_yaw_min;i<=PWM_yaw_mid;i=i+128)
 {
	 LED1_Flash();
   Set_PWM_Yaw(i);   //����
 }
 delay1ms(5000);
 printf("*******************lock***************\r\n");
}

/*==================================================================================================== 
   ���׼��
=====================================================================================================*/ 
void Take_off_Preper(void)
{	
	 printf("*******************Take_off_Preper***************\r\n");
}


/*==================================================================================================== 
   ��� Fly_Moder_AltHold
=====================================================================================================*/ 
void Take_off(float high)
{	 
	 AltAdj(high);
	 printf("*******************Take_off***************\r\n");
}


/*==================================================================================================== 
   ����
=====================================================================================================*/ 
void land(void)
{
	 printf("*******************Land***************\r\n");
}

/*==================================================================================================== 
   ǰ��
=====================================================================================================*/ 
void Go_ahead(uint16_t pulse)
{
	Set_PWM_Pitch(PWM_Pitch_mid-pulse);      
}

/*==================================================================================================== 
   ����
=====================================================================================================*/ 
void Go_back(uint16_t pulse)
{
	Set_PWM_Pitch(PWM_Pitch_mid-pulse);      
}

/*==================================================================================================== 
   �����
=====================================================================================================*/ 
void Go_left(uint16_t pulse)
{
	Set_PWM_Roll(PWM_Roll_mid-pulse);      
}

/*==================================================================================================== 
   ���ҷ�
=====================================================================================================*/ 
void Go_right(uint16_t pulse)
{
	Set_PWM_Roll(PWM_Roll_mid+pulse);      
}

/*==================================================================================================== 
   ��ת
=====================================================================================================*/ 
void Turn_left(uint16_t pulse)
{
	Set_PWM_Yaw(PWM_yaw_mid-pulse);      
}

/*==================================================================================================== 
   ��ת
=====================================================================================================*/ 
void Turn_right(uint16_t pulse)
{
	Set_PWM_Yaw(PWM_yaw_mid+pulse);      
}


/*==================================================================================================== 
   ����ģʽ
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
   ��ͣ
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
		//pitch ����
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
   �߶ȵ���
=====================================================================================================*/ 
void AltAdj(float high)
{	 
	 Set_PWM_Thr(PWM_Thr_mid);
	 printf("******************�߶ȵ���**************\r\n");
}



/*==================================================================================================== 
   PWM�ײ����
=====================================================================================================*/ 
//***********����*********************************************//
void Set_PWM_Thr(uint16_t pulse)
{
  TIM2 ->CCR1 =pulse ;
}

//***********����**********************************************//
void Set_PWM_Pitch(uint16_t pulse)
{
  TIM2 ->CCR2 =pulse ;
}


//***********���*********************************************//
void Set_PWM_Roll(uint16_t pulse)
{
  TIM2 ->CCR3 =pulse ;
}

//***********ƫ��**********************************************//
void Set_PWM_Yaw(uint16_t pulse)
{
  TIM2 ->CCR4 =pulse ;
}

//***********ģʽ**********************************************//
void Set_PWM_Mode(uint16_t pulse)
{
  TIM4 ->CCR1 =pulse ;
}

//***********ֱͨģʽ*********************************************//
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

