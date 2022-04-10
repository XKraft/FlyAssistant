#include "key.h"
#include "main.h"
#include "stdio.h"
#include "beep.h"

#define S1  HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)
#define S2  HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin)
#define S3  HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin)
#define S4  HAL_GPIO_ReadPin(KEY4_GPIO_Port,KEY4_Pin)


//����������
//���ذ���ֵ
//mode:0,��֧��������;1,֧��������;
//0��û���κΰ�������
//1��WKUP���� WK_UP
//ע��˺�������Ӧ���ȼ�,KEY0>KEY1>KEY2>WK_UP!!
uint8_t KeyScanning(uint8_t mode)
{
    static uint8_t key_up=1;     //�����ɿ���־
    if(mode==1)key_up=1;         //֧������
    if(key_up&&(S1==0||S2==0||S3==0||S4==1))
    {
			  //printf(" KEY_Scan \r\n");
        HAL_Delay(30);
        key_up=0;	//��ʾ�����Ѱ���
        if(S1==0)        
				{
					printf("KEY1_PRES \r\n");
					return  KEY1_PRES;
				}
        else if(S2==0)
				{
					printf("KEY2_PRES \r\n");
					return  KEY2_PRES;
				}
        else if(S3==0)   
				{
					printf("KEY3_PRES \r\n");
					return  KEY3_PRES;
				}					
        else if(S4==1)   
				{
					printf("KEY4_PRES \r\n");
				  return  KEY4_PRES;  
				}  
    }else if(S1==1&&S2==1&&S3==1&&S4==0)
		{
			key_up=1;
			//printf("key_up=1 \r\n");
		}
    return 0;   //�ް�������
}			



void GetKey_NUM(void)
{
	uint8_t key;
	key=KeyScanning(0);            //����ɨ��
	switch(key)
	{				 
		case  KEY1_PRES:	//����LED0,LED1�������
          LED2_Flash();
       	  BEEP_ON();
					break;
		case  KEY2_PRES:	//����LED0��ת
					LED2_Flash();
       	  BEEP_ON();
					break;
	}
}
	
