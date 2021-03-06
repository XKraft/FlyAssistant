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
		//**********************#*****消息号**位置XH*****位置XL***位置YH****位置YL*****目标位置H***目标位置L***尾*******
    //********************* #头***消息号，数据位1L，数据位1H,数据位2L,数据位2H,数据位3L,数据位3H,数据位4L,数据位4H*****帧尾* *********
		if((inBuf[0]==0x23) && (inBuf[10]==0x2A))
		{
			//开始
			if(inBuf[1]=='1')		//悬停到目标点
			{
				//printf("*****第一号消息************ \r\n"); 
				Attitude.Position_x = inBuf[3]<<8|inBuf[2];
				Attitude.Position_y = inBuf[5]<<8|inBuf[4];
				Attitude.SetPoint_x = inBuf[7]<<8|inBuf[6];
				Attitude.SetPoint_y = inBuf[9]<<8|inBuf[8];
				outBuf[2]=inBuf[2]; //接收到的有效数据1
				outBuf[3]=inBuf[3]; //接收到的有效数据2
				outBuf[4]=inBuf[4]; //接收到的有效数据3
				outBuf[5]=inBuf[5]; //接收到的有效数据4
				outBuf[6]=inBuf[6]; //接收到的有效数据5
				outBuf[7]=inBuf[7]; //接收到的有效数据6
				outBuf[8]=inBuf[8]; //接收到的有效数据7
				outBuf[9]=inBuf[9]; //接收到的有效数据8
				//printf("Attitude Position x = %d, Attitude Position y = %d, SetPoint x = %d, SetPoint y = %d \r\n", Attitude.Position_x, Attitude.Position_y, Attitude.SetPoint_x, Attitude.SetPoint_y);
			  return 1;
			}
			/* 以下指令可能会用于完成要求的飞行动作 */
			else if(inBuf[1] == '2')		//向右飞
			{
				return 2;
			}
			else if(inBuf[1] == '3')		//向左飞
			{
				return 3;
			}
			else if(inBuf[1] == '4')		//向右飞
			{
				return 4;
			}
			else if(inBuf[1] == '5')		//向右飞
			{
				return 5;
			}
			else return 0;
		}
		else return 0;
	}
	else return 0;
}




