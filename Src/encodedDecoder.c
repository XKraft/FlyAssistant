#include "coderDecoder.h"
#include "main.h"
#include "stdio.h"
#include "uart_api.h"
#include "flyControl.h"

#define BUFFSIZE 11  

extern uint8_t encodeAnswer[BUFFSIZE];
ATTITUDE    Attitude; 

uint8_t encodeDecode_Analysis(uint8_t *inBuf,uint8_t *outBuf,uint16_t Buflen)
{
	//printf("\r\n");  
	if(Buflen>=2)
	{
		//**********************#*****��Ϣ��**λ��XH*****λ��XL***λ��YH****λ��YL*****Ŀ��λ��H***Ŀ��λ��L***β*******
    //********************* #ͷ***��Ϣ�ţ�����λ1L������λ1H,����λ2L,����λ2H,����λ3L,����λ3H,����λ4L,����λ4H*****֡β* *********
		if((inBuf[0]==0x23) && (inBuf[10]==0x2A))
		{
			
			//��ʼ
			if(inBuf[1]=='1')
			{
				//printf("*****��һ����Ϣ************ \r\n"); 
				Attitude.Position_x = inBuf[3]<<8|inBuf[2];
				Attitude.Position_y = inBuf[5]<<8|inBuf[4];
				Attitude.SetPoint_x = inBuf[7]<<8|inBuf[6];
				Attitude.SetPoint_y = inBuf[9]<<8|inBuf[8];
				outBuf[2]=inBuf[2]; //���յ�����Ч����1
				outBuf[3]=inBuf[3]; //���յ�����Ч����2
				outBuf[4]=inBuf[4]; //���յ�����Ч����3
				outBuf[5]=inBuf[5]; //���յ�����Ч����4
				outBuf[6]=inBuf[6]; //���յ�����Ч����5
				outBuf[7]=inBuf[7]; //���յ�����Ч����6
				outBuf[8]=inBuf[8]; //���յ�����Ч����7
				outBuf[9]=inBuf[9]; //���յ�����Ч����8
				//printf("Attitude Position x = %d, Attitude Position y = %d, SetPoint x = %d, SetPoint y = %d \r\n", Attitude.Position_x, Attitude.Position_y, Attitude.SetPoint_x, Attitude.SetPoint_y);
			  return 1;
			}
		}
	}
	else return 0;
}



