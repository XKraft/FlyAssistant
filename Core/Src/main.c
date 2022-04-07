/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h" 
#include "arm_math.h"
#include <stdarg.h>
#include <string.h> 
#include "mavlink.h"
#include "mavlink_types.h"

#include "led.h"
#include "key.h"
#include "uart_api.h"
#include "beep.h"
#include "coderDecoder.h"
#include "flyControl.h"
#include "MAV_Altitude_Decode.h"

/* 定义例程名和例程发布日期 */
#define EXAMPLE_NAME	"V4梦创飞控室内自动飞行例程"
#define EXAMPLE_DATE	"2021-04-7 "
#define DEMO_VER		  "V-1.0"

  
#define Flag_SendToUsart  1 

#define CH_NUM	1	/* 8通道 */
#define N 3
#define  voltage_ratio   204  //2.04


#define USART1_MAX_RECV_LEN		256				//最大接收缓存字节数
#define USART2_MAX_RECV_LEN		256		
#define BUFFSIZE 5 

int CHANNEL_1_RISE=0,CHANNEL_1_FALL=0,CHANNEL_1_PULSE_WIDE=0;
int CHANNEL_2_RISE=0,CHANNEL_2_FALL=0,CHANNEL_2_PULSE_WIDE=0;
int CHANNEL_3_RISE=0,CHANNEL_3_FALL=0,CHANNEL_3_PULSE_WIDE=0;
int CHANNEL_4_RISE=0,CHANNEL_4_FALL=0,CHANNEL_4_PULSE_WIDE=0;
int CHANNEL_5_RISE=0,CHANNEL_5_FALL=0,CHANNEL_5_PULSE_WIDE=0;
int CHANNEL_6_RISE=0,CHANNEL_6_FALL=0,CHANNEL_6_PULSE_WIDE=0;
int CHANNEL_7_RISE=0,CHANNEL_7_FALL=0,CHANNEL_7_PULSE_WIDE=0;
int CHANNEL_8_RISE=0,CHANNEL_8_FALL=0,CHANNEL_8_PULSE_WIDE=0;

int ICFLAG_1 = 1,ICFLAG_2 = 1,ICFLAG_3 = 1, ICFLAG_4 = 1, ICFLAG_5 = 1, ICFLAG_6 = 1, ICFLAG_7 = 1, ICFLAG_8 = 1;

 int PWM_Mode_N1 =3000;
 int PWM_Mode_N2 =4000;
 int PWM_Mode_N3 =5000;
 int PWM_Mode_N4 =6000;
 

uint16_t    USART1_RX_STA=0; 
uint16_t    USART2_RX_STA=0; 
static int Recv_Cnt = 0;
static int UART1_Frame_Flag = 0;
static int UART2_Frame_Flag = 0;
static int heartbeat = 0;
static int Recv_Cnt_UART2 = 0;
static int MAVLink_message_length = 0;
static mavlink_distance_sensor_t packet;
static float height = 0;

uint8_t USART1_RX_BUF [USART1_MAX_RECV_LEN]; 
uint8_t USART2_RX_BUF [USART2_MAX_RECV_LEN]; 
uint8_t FreeBuffer_Encode [USART2_MAX_RECV_LEN];
uint8_t MAVLink_RECV_BUF[USART2_MAX_RECV_LEN];
uint8_t MAVLink_TX_BUF [MAVLINK_MAX_PACKET_LEN];
uint8_t MAVLink_RECV_BUF_FAKE [USART2_MAX_RECV_LEN] = {0};

//回应 头 消息号 数据1  数据2 数据3 数据4 尾
uint8_t encodeAnswer[11]    ={'#','1',0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,'*'};

#define A 60
float old_value;

static      int16_t s_dat[CH_NUM];
float       s_volt[CH_NUM];      

uint32_t    ADC_ReadBuffer[CH_NUM]; 
float  OutData_Test[4]; 
float  OutData[4]; 
char   menu=0;

PID         PID_Control_Att;           //  PID Control Structure 
float filter(float new_value);

void  OutPut_Data(void);
float ADC_CvtVolt(void);
float DataProcessing(float IN_Data);
float ADC_CvtVolt_and_filter(void);
float get_adc(char adc_id);
float filter_av(char filter_id);
void TIM3_Set(uint8_t sta);
void TIM5_Set(uint8_t sta);
void Data_to_VisualScope(void);
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);


//mavlink
void MANUAL_CONTROL_Send(int16_t xpoint,int16_t ypoint);
void RC_CHANNELS_OVERRIDE_Send(int16_t xpoint,int16_t ypoint);
void heartbeat_Mavlink(void);
int  RC_Read(void);
void Back_to_Center(void);
	

void end(void)
{
	printf("%c",0xff);
	printf("%c",0xff);
	printf("%c",0xff);
}

/* 仅允许本文件内调用的函数声明 */
static void PrintfLogo(void);
static void PrintfHardInfo(void);

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
////////////////////////////////不懂刚干啥用的
#ifdef __GNUC__  
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf 
	 set to 'Yes') calls __io_putchar() */  
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)  
#else  
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)  
#endif /* __GNUC__ */  

PUTCHAR_PROTOTYPE
{
    BSP_USART_SendData_LL( USART1, ch);
	  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t i1=0,i2=0;
	uint16_t rxlen_usart_1;
	uint16_t rxlen_usart_2;
	uint8_t cmd=0;
	uint8_t key;
	uint32_t RunTime=0;
	uint32_t decodetimer;
	uint32_t tickstart = HAL_GetTick();
	uint32_t ticklast = HAL_GetTick();
	mavlink_message_t msg1;
	mavlink_message_t msg2;
	mavlink_message_t msg_tmp;
	mavlink_message_t msg_altitude;
	mavlink_message_t msg_request_data_stream;
	uint32_t len = 0;
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	Device_Init();
	PrintfLogo();		  /* 打印例程Logo到串口1 */
	PrintfHardInfo();	/* 打印硬件接线信息 */
	
	/*
		由于ST固件库的启动文件已经执行了CPU系统时钟的初始化，所以不必再次重复配置系统时钟。
		启动文件配置了CPU主时钟频率、内部Flash访问速度和可选的外部SRAM FSMC初始化。
	*/

		//HAL_ADC_Init(&hadc1);
		//HAL_ADC_Start(&hadc1);		
		
		//开启PWM
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
		

		/// 使能定时器输入捕获。
		HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);
		HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_2);
		HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_3);
		HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_4);
		
		HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_1);
		HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_2);
		HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_3);
		HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_4);

		PIDInit(&PID_Control_Att); 
		LED1_Flash();
		BEEP_ON();
		HAL_Delay(1000);              
		BEEP_OFF();
		HAL_Delay(1000);   
		
		BSP_USART_StartIT_LL( USART2 );
	  BSP_USART_StartIT_LL( USART1 );
	
	  USART1_RX_STA=0;		//清零
	  USART2_RX_STA=0;		//清零
	
		TIM3_Set(0);			  
	  TIM5_Set(0);	
		
///*************************************打一个假包，用于获取消息实际长度*************/	
//		mavlink_msg_altitude_pack(54,0,&msg_tmp,0,1,1,1,1,1,1);                       //
//	  MAVLink_message_length=mavlink_msg_to_send_buffer(MAVLink_TX_BUF,&msg_tmp);   //
///**********************************************************************************/

		mavlink_msg_request_data_stream_pack(54,0,&msg_request_data_stream,0,0,12,5,1);
		len = mavlink_msg_to_send_buffer(MAVLink_TX_BUF, &msg_request_data_stream);
		BSP_USART_SendArray_LL(USART2, MAVLink_TX_BUF, len);
		BSP_USART_SendArray_LL(USART1, MAVLink_TX_BUF, len);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//		BSP_USART_SendArray_LL(USART1, MAVLink_TX_BUF, len);
		//LED_G_Flash(); 
		//mavlink读取飞控
		key=KeyScanning(0);            //按键扫描
		switch(key)
		{				 
			case  KEY1_PRES:	//控制LED0,LED1互斥点亮
						LED_B_Flash(); 
						BEEP_ON();
			      Unlock();
						break;
			case  KEY2_PRES:	//控制LED0翻转
						LED_G_Flash(); 
						BEEP_OFF();
						break;
		}
		if(InitedFlag)  
		{
//			decodetimer = HAL_GetTick();
//			if(decodetimer-ticklast>=20)
//			{
//				LL_USART_DisableIT_RXNE(USART2);
//				rxlen_usart_2 = USART2_RX_STA & 0x7FFF;
//				for(i2=0;i2<rxlen_usart_2;i2++)
//				{
//					MAVLink_RECV_BUF[i2]=USART2_RX_BUF[i2];	 //将串口2接收到的数据传输给自由缓冲区
//				}
//				if(Mav_Altitude_Decoder(rxlen_usart_2, msg_altitude, MAVLink_RECV_BUF, &height)==1) //分析字符串
//				{
//					printf("height = %f \r\n", height);
//					rxlen_usart_2=0;
//				  USART2_RX_STA=0;	 //启动下一次接收
//				}
//				ticklast = HAL_GetTick();
//				BSP_USART_StartIT_LL( USART2 );
//			}
			
			 //printf("USART1_RX_STA = %d\r\n", USART1_RX_STA);
			/********************************UART1接收并处理数据**********************************/
			 if(USART1_RX_STA & 0X8000)		  //接收到一次数据了
			 {
        //printf("USART1 INT =%d \r\n",USART1_RX_STA);				 
				rxlen_usart_1 = USART1_RX_STA & 0x7FFF;	//得到数据长度
				//printf("This is a USART1 test rxlen_usart_1 = %d USART1_RX_STA= %d\r\n" ,rxlen_usart_1 ,USART1_RX_STA);
				for(i1=0;i1<rxlen_usart_1;i1++)
				{
					FreeBuffer_Encode[i1]=USART1_RX_BUF[i1];	 //将串口2接收到的数据传输给自由缓冲区
					//BSP_USART_SendArray_LL( USART1,&FreeBuffer_Encode[i1],1);
				}
				cmd=encodeDecode_Analysis(FreeBuffer_Encode,encodeAnswer,rxlen_usart_1); //分析字符串
				BSP_USART_StartIT_LL( USART1 );
				rxlen_usart_1=0;
				USART1_RX_STA=0;	 //启动下一次接收
			}
			/********************************UART2接收并处理数据***********************************/
			if(USART2_RX_STA & 0X8000)		  //接收到一次数据，且超过了预设长度
			 {
//				mavlink_msg_request_data_stream_pack(54,0,&msg_request_data_stream,0,0,12,10,0);
//				len = mavlink_msg_to_send_buffer(MAVLink_TX_BUF, &msg_request_data_stream);
//				BSP_USART_SendArray_LL(USART2, MAVLink_TX_BUF, len);
//				BSP_USART_SendArray_LL(USART1, MAVLink_TX_BUF, len);
        //printf("USART1 INT =%d \r\n",USART1_RX_STA);				 
				rxlen_usart_2 = USART2_RX_STA & 0x7FFF;	//得到数据长度
				//printf("This is a USART1 test rxlen_usart_1 = %d USART1_RX_STA= %d\r\n" ,rxlen_usart_1 ,USART1_RX_STA);
				for(i2=0;i2<rxlen_usart_2;i2++)
				{
					MAVLink_RECV_BUF[i2]=USART2_RX_BUF[i2];						//将串口2接收到的数据传输给自由缓冲区
//					if (MAVLink_RECV_BUF[i2]==0xFE)
//						printf("%x", MAVLink_RECV_BUF[i2]);
					//BSP_USART_SendArray_LL( USART1,&FreeBuffer_Encode[i1],1);
				}
//				for(i2=0;i2<rxlen_usart_2;i2++)/
//				{
//				 if (MAVLink_RECV_BUF[i2]==0xFE)printf("\r\n");						//将串口2接收到的数据传输给自由缓冲区
					//printf("%x", MAVLink_RECV_BUF[i2]);
					//BSP_USART_SendData_LL( USART1,MAVLink_RECV_BUF[i2]);
//			}
				
//				Mav_Altitude_Decoder(rxlen_usart_2, msg_altitude, MAVLink_RECV_BUF, &height);
//				if(Mav_Altitude_Decoder(rxlen_usart_2, msg_altitude, MAVLink_RECV_BUF, &height)==1) //分析字符串
				{
					printf("\r\nheight = %f \r\n", height);
				}
				//BSP_USART_SendArray_LL(USART1, MAVLink_RECV_BUF, rxlen_usart_2);
//				//printf("rxlen_usart_2 = %d\r\n", rxlen_usart_2);
				rxlen_usart_2=0;
				USART2_RX_STA=0;
				BSP_USART_StartIT_LL( USART2 );
					 //启动下一次接收
//				mavlink_msg_request_data_stream_pack(54,0,&msg_request_data_stream,0,0,12,10,1);
//				len = mavlink_msg_to_send_buffer(MAVLink_TX_BUF, &msg_request_data_stream);
//				BSP_USART_SendArray_LL(USART2, MAVLink_TX_BUF, len);
				}
			
				//执行指令的当作
				if(2==RC_Read())//读取是否直通
				{ 
					 Set_PWM_Mode(4500);
					 switch (cmd)
					 {
							case 1: 
								printf("left  \r\n"); 
								Loiter(Attitude.Position_x,Attitude.Position_y,Attitude.SetPoint_x,Attitude.SetPoint_y);	
							  //RC_bridge_Test();
								heartbeat = 0;
								break;	
							case 2: 
								printf("right \r\n"); 
							  Go_right(4500+10);
							  //RC_bridge_Test();
								heartbeat = 0;
								break;											
							case 3: 
								printf("right \r\n"); 
							  Go_left(4500+10); 							
							  //RC_bridge_Test();
								heartbeat = 0;
								break;											 
							default:
								printf("default\r\n");
								Back_to_Center();//没有UART数据输入时各通道回中
							  break;
					 }
					 heartbeat++;
					 if(heartbeat>3000)
					 {
						 cmd =0;
						 heartbeat = 0;
					 }
				 }
				 else if(3==RC_Read())
				 {
					 //桥接模式
					 RC_bridge();
				 }
				 else if(1==RC_Read())
				 {
					 Set_PWM_Mode(6000);
					 Set_PWM_Thr(4500);
					 Set_PWM_Pitch(4500);
					 Set_PWM_Roll(4500);
					 Set_PWM_Yaw(4500);
					 
				 }
				 //LED_G_Flash(); 
				 RC_bridge_Test(); 
		}
		//指示灯
		RunTime = HAL_GetTick() - tickstart;
		if(RunTime>1000)
		{
			//printf("RunTime>1000%d \r\n",RunTime); 
			tickstart = HAL_GetTick();
			LED_G_Flash(); 
			HAL_Delay(10);
			LED_R_Flash(); 
			HAL_Delay(20);
			LED_B_Flash(); 
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00C0EAFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00C0EAFF;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 33;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 33;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 60000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 33;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 60000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 33;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 60000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART5;
  PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**UART5 GPIO Configuration
  PB12   ------> UART5_RX
  PB13   ------> UART5_TX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* UART5 interrupt Init */
  NVIC_SetPriority(UART5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(UART5_IRQn);

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(UART5, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(UART5);
  LL_USART_Enable(UART5);
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**USART1 GPIO Configuration
  PB14   ------> USART1_TX
  PB15   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
  /**USART2 GPIO Configuration
  PD5   ------> USART2_TX
  PD6   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
  /**USART3 GPIO Configuration
  PD8   ------> USART3_TX
  PD9   ------> USART3_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART3);
  LL_USART_Enable(USART3);
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_R_Pin|LED_G_Pin|LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_R_Pin LED_G_Pin LED_B_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_G_Pin|LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY1_Pin KEY2_Pin KEY3_Pin KEY4_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin|KEY3_Pin|KEY4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BEEP_Pin */
  GPIO_InitStruct.Pin = BEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BEEP_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
*********************************************************************************************************
*	函 数 名: PrintfLogo
*	功能说明: 打印例程名称和例程发布日期, 接上串口线后，打开PC机的超级终端软件可以观察结果
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void PrintfLogo(void)
{
	printf("*************************************************************\n\r");
	printf("* 例程名称   : %s\r\n", EXAMPLE_NAME);	/* 打印例程名称 */
	printf("* 例程版本   : %s\r\n", DEMO_VER);		/* 打印例程版本 */
	printf("* 发布日期   : %s\r\n", EXAMPLE_DATE);	/* 打印例程日期 */

	/* 打印ST固件库版本，这3个定义宏在stm32f40x.h文件中 */
	printf("* 固件库版本 :STM32F10x_StdPeriph_Driver)\r\n");
	printf("* \n\r");	/* 打印一行空格 */
	printf("* QQ    : 665836518 \r\n");
	printf("* 淘宝店地址 : https://shop144519723.taobao.com/index.htm?spm=2013.1.w5002-13163471369.2.71db1223NyFC4j \r\n");
	printf("* 梦创电子 \r\n");
	printf("*************************************************************\n\r");
}

/*
*********************************************************************************************************
*	函 数 名: PrintfHardInfo
*	功能说明: 打印硬件接线信息
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void PrintfHardInfo(void)
{
  printf("*****************************************************************\r\n");
	printf("  接线方法: \r\n");
	printf("**梦创电子四旋翼飞行器室内自动飞行****\r\n");
  printf("  +5V       <------   5.0V      5V供电\r\n");
  printf("  GND       -------   GND       地\r\n");
	printf("  PB0       ------>   PWM3      与遥控器的pitch通道相连\r\n");
	printf("  PB1       ------>   PWM4      与遥控器的接收机的第五通道相连 \r\n");
	printf("  PA6       ------>   PWM1      与遥控器接收机油门相连 \r\n");
	printf("  PB7       ------>   PWM2      与遥控器的偏航通道(YAW)相连\r\n");
	printf("  PA0       ------>   ADC_CH1   模拟超声波高度测量\r\n");
	printf("  打印采集数据: \r\n");
	printf("*****************************************************************\r\n");
}



unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){      
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}

//*********************OutPut_Data****************************************//
void OutPut_Data(void)
{
  int temp[4] = {0};
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i;
  unsigned short CRC16 = 0;
  for(i=0;i<4;i++)
   {
    
    temp[i]  = (int16_t)OutData[i];
    temp1[i] = (int16_t)temp[i];
    
   }
   
  for(i=0;i<4;i++) 
  {
    databuf[i*2]   = (int8_t)(temp1[i]%256);
    databuf[i*2+1] = (int8_t)(temp1[i]/256);
  }
  
  CRC16 = CRC_CHECK(databuf,8);
  databuf[8] = CRC16%256;
  databuf[9] = CRC16/256;
  
  for(i=0;i<10;i++)
	{
		BSP_USART_SendArray_LL( USART1,databuf,sizeof(databuf));
		//HAL_UART_Transmit(&huart1,(uint8_t *)&databuf[i],1,10);       //串口发送
	}
}



//****************************Data_to_VisualScope***************************************//
void Data_to_VisualScope(void)
{	
		int16_t i;	
    for(i = 0; i < CH_NUM; i++)
    {
      OutData[i] = s_volt[i];
      OutPut_Data();
    }
}


/*
*********************************************************************************************************
*	函 数 名: AD转化函数
*	功能说明: 处理采样后的数据
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
float ADC_CvtVolt(void)
{
	float volt=0;
	uint8_t i;
	int16_t adc;
	for(i=0;i<CH_NUM;i++)
	{

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,0xffff);

		ADC_ReadBuffer[i] = HAL_ADC_GetValue(&hadc1);
		s_dat[i] = ADC_ReadBuffer[i];
		adc = s_dat[i];
		s_volt[i] = (adc * 3.30f) / 4096;
	}	
	HAL_ADC_Stop(&hadc1);
	volt=s_volt[0]*voltage_ratio ; 
	return  volt;
}


/*
*********************************************************************************************************
*	函 数 名: AD7606_Mak
*	功能说明: 处理采样后的数据
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
float ADC_CvtVolt_and_filter(void)
{
	float ADC_read=0.0;
	ADC_read=filter_av(1);
	return  ADC_read;
}


//***********读取ADC****************//
float get_adc(char adc_id)
{
	float volt=0;
	int16_t adc;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,0xffff);

	adc=HAL_ADC_GetValue(&hadc1);
	volt = ((adc * 3.30f) / 4096)*voltage_ratio;
	HAL_ADC_Stop(&hadc1);
	HAL_Delay(1);	
	//printf("V=%2.2f V Data_after_Deal=%2.2f cm  H=%2.2f cm  (count0=%4.1d) pwm_duty_out=%2.2f pwm_out=%d control_val=%2.2d \r\n",volt_0/voltage_ratio,Data_after_Deal,volt_0,count0,pwm_duty_out,PWM_Out,control_val); 
	return  volt;
}

//********算数平均滤波法**************//
float filter_av(char filter_id)
{
	char count=0;
	float sum = 0.0;
	for ( count=0;count<N;count++)
	{
	 sum  = sum + get_adc(1);
	}
	return sum/N;
}


//************************************数据处理函数********************************//
float DataProcessing(float IN_Data)
{
	static float out_Data=0,last_Data=0,filter_Data=0; //static只初始化一次
	out_Data  = 0.7*IN_Data+0.3*last_Data;
	filter_Data  = filter(out_Data);
	last_Data = filter_Data;
	return filter_Data;
}


float filter(float new_value)
{
if(new_value>200)
	new_value=200;
if(new_value<10)
	new_value=10;
if ( ( new_value - old_value > A ) || ( old_value - new_value > A ))
 {
	old_value = old_value;
	return old_value;
 }
 old_value = new_value;
 return new_value;
}



void   USART_RxCallback(USART_TypeDef *huart)
{ 
	mavlink_message_t msg;
	mavlink_status_t status;
	//***********串口1中断**********************************
	if(LL_USART_IsActiveFlag_RXNE(huart) && LL_USART_IsEnabledIT_RXNE(huart))
  { // 接收到来自上位机的命令
		if(huart == USART1)
		{
			uint8_t data = LL_USART_ReceiveData8(huart);
			if(data == 0x23)
			{
				UART1_Frame_Flag = 1;
			}
			//printf("USART1_RX_STA =%d data = %d \r\n",USART1_RX_STA , data);
      if(((USART1_RX_STA  & (1<<15))==0) && (UART1_Frame_Flag == 1))		//还可以接收数据 ,最高位不为1.
			{
				//TIM4->CNT=0;				//计数器清空
				USART1_RX_BUF[USART1_RX_STA++] = data;				//记录接收到的值
				Recv_Cnt ++;
				if(USART1_RX_STA == 0)
				{
					Recv_Cnt=0;
				//	TIM4_Set(1);	 	                //使能定时器4的中断 
				}
				else if(Recv_Cnt>=11)
				{
				 Recv_Cnt=0;
				 UART1_Frame_Flag = 0;
				 USART1_RX_STA |= 1<<15;				 //强制标记接收完成
			   LL_USART_DisableIT_RXNE(USART1);
				}
				//BSP_USART_SendArray_LL( USART1,&USART1_RX_BUF[USART1_RX_STA],1);
				//printf("USART1 INT =%d \r\n",USART1_RX_STA);
			}	
		}
	 //***********串口2中断**********************************
		else if(huart == USART2)
		{
			uint8_t data = LL_USART_ReceiveData8(huart);
			if(data == 0xFE)
			{
				UART2_Frame_Flag = 1;
			}
			//printf("USART2_RX_STA =%d data = %d \r\n",USART2_RX_STA , data);
      if((USART2_RX_STA  & (1<<15))==0 && (UART2_Frame_Flag == 1))		//还可以接收数据 ,最高位不为1.
			{
				//TIM1->CNT=0;				//计数器清空
				USART2_RX_BUF[USART2_RX_STA++] = data;
				Recv_Cnt_UART2 ++;
				if(USART2_RX_STA == 0)
				{
					//TIM1_Set(1);	 	                //使能定时器1的中断
						Recv_Cnt_UART2 = 0;
				}
        //printf("USART2 INT =%d \r\n",USART2_RX_STA);				
				else if(Recv_Cnt_UART2>=50)
				{
					Recv_Cnt_UART2 = 0;
					UART2_Frame_Flag = 0;
					USART2_RX_STA |= 1<<15;					//强制标记接收完成
					LL_USART_DisableIT_RXNE(USART2);
				}	
			}
		}
	}
}



//*******定时器中断服务程序	*************************************//
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		static uint16_t tim3_1ms= 0;//中断次数计数器
	  static uint16_t tim5_1ms= 0;//中断次数计数器

		 //*****定时器3中断服务函数*********************
		if (htim->Instance == htim3.Instance) //是更新中断
		{
			tim3_1ms++;
			if(tim3_1ms==5)		    //每5次中断执行一次,20ms X 5
			{
				//USART2_RX_STA|= (1<<15);	//标记接收完成
				TIM3->SR&=~(1<<0);		//清除中断标志位		   
				TIM3_Set(0);			    //关闭TIM3
				tim3_1ms=0;
				//printf("TIME 4 INT \r\n");
			} 
		}
		 //*****定时器3中断服务函数*********************
		if (htim->Instance == htim5.Instance) //是更新中断
		{
			tim5_1ms++;
			if(tim5_1ms==5)		    //每5次中断执行一次,20ms
			{
				//USART1_RX_STA |= (1<<15);	//标记接收完成
				TIM5->SR&=~(1<<0);		//清除中断标志位		   
				TIM5_Set(0);			    //关闭TIM5
				tim5_1ms=0;
				//printf("TIME 5 INT \r\n");
			} 
		}
}

//定时器4
void TIM3_Set(uint8_t sta)
{
	if(sta)
	{
		TIM3->CNT=0;                   //计数器清空
		HAL_TIM_Base_Start_IT(&htim3); //使能定时器3
	}else 
		HAL_TIM_Base_Stop_IT(&htim3);  //关闭定时器3
}


//定时器4
//********************************
//采用定时器轮询的方式实现延时。
//		for(int q=0;q<1000;q++)
//		{
//				Delay_us(1000);
//		}
//调用示例
//********************************

void TIM5_Set(uint8_t sta)
{
	if(sta)
	{
		TIM5->CNT=0;                   //计数器清空
		HAL_TIM_Base_Start_IT(&htim5); //使能定时器5
	}else 
		HAL_TIM_Base_Stop_IT(&htim5);  //关闭定时器5 
}

// 定时器输入捕获PWM
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//printf("TIM_IC \r\n");
  TIM_IC_InitTypeDef sConfigIC;
	if(htim->Instance == htim4.Instance)
	{
		switch(htim->Channel)
		{
			case HAL_TIM_ACTIVE_CHANNEL_1:
				if(ICFLAG_1){
					CHANNEL_1_RISE = HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_1);
					
					sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;//下降沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim4,TIM_CHANNEL_1);
					HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1);
					
					__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC1);
					if(HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1)!= HAL_OK)
					{
						printf("ERROR\r\n");
					}
					
					ICFLAG_1 = 0;
				}
				else{
					CHANNEL_1_FALL = HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_1);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;//上升沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim4,TIM_CHANNEL_1);
					HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1);
					
					__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC1);
					if(HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1)!= HAL_OK)
					{
						printf("ERROR\r\n");
					}

					
					CHANNEL_1_PULSE_WIDE = (CHANNEL_1_FALL > CHANNEL_1_RISE ? (CHANNEL_1_FALL - CHANNEL_1_RISE):(CHANNEL_1_FALL - CHANNEL_1_RISE + 60000));
					ICFLAG_1 = 1;
				}
				//CH1_PWM_test(CHANNEL_1_PULSE_WIDE);
//				printf("channel1 pulsewidth = %d \r\n", CHANNEL_1_PULSE_WIDE);
				break;
	//	 /*************************************************************************/   
			case HAL_TIM_ACTIVE_CHANNEL_2:
					if(ICFLAG_2){
					CHANNEL_2_RISE = HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_2);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;//下降沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim4,TIM_CHANNEL_2);
					////////////////////////////这里没有&，是漏打了还是本来就这样？
					HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2);
					
					__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC2);
					if(HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_2)!= HAL_OK)
					{
						printf("ERROR\r\n");
					}  
					
					ICFLAG_2 = 0;
				}
				else{
					CHANNEL_2_FALL = HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_2);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;//上升沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim4,TIM_CHANNEL_2);
					HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2);
					
					__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC2);
					if(HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_2)!= HAL_OK)
					{
						printf("ERROR\r\n");
					} 
					
					CHANNEL_2_PULSE_WIDE = (CHANNEL_2_FALL > CHANNEL_2_RISE ? CHANNEL_2_FALL - CHANNEL_2_RISE:CHANNEL_2_FALL - CHANNEL_2_RISE + 60000);
					ICFLAG_2 = 1;
				}
				//CH2_PWM_test(CHANNEL_2_PULSE_WIDE);
//				printf("channel2 pulsewidth = %d \r\n", CHANNEL_2_PULSE_WIDE);
				break;
	//	/**************************************************************************/    
			case HAL_TIM_ACTIVE_CHANNEL_3:
				if(ICFLAG_3){
					CHANNEL_3_RISE = HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_3);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;//下降沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim4,TIM_CHANNEL_3);
					HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3);
					
					__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC3);
					if(HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_3)!= HAL_OK)
					{
						printf("ERROR\r\n");
					}  
					
					ICFLAG_3 = 0;
				}
				else{
					CHANNEL_3_FALL = HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_3);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;//上升沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim4,TIM_CHANNEL_3);
					HAL_TIM_IC_ConfigChannel(htim, &sConfigIC, TIM_CHANNEL_3);
					
					__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC3);
					if(HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_3)!= HAL_OK)
					{
						printf("ERROR\r\n");
					}  
					
					CHANNEL_3_PULSE_WIDE = (CHANNEL_3_FALL > CHANNEL_3_RISE ? CHANNEL_3_FALL - CHANNEL_3_RISE:CHANNEL_3_FALL - CHANNEL_3_RISE + 60000);
					ICFLAG_3 = 1;      
				}
				//CH3_PWM_test(CHANNEL_3_PULSE_WIDE);
//				printf("channel3 pulsewidth = %d \r\n", CHANNEL_3_PULSE_WIDE);
				break;
	//			/**************************************************************************/    
			case HAL_TIM_ACTIVE_CHANNEL_4:
				if(ICFLAG_4){
					CHANNEL_4_RISE = HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_4);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;//下降沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim4,TIM_CHANNEL_4);
					HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4);
					
					__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC4);
					if(HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_4)!= HAL_OK)
					{
						printf("ERROR\r\n");
					}    
					
					ICFLAG_4 = 0;
				}
				else{
					CHANNEL_4_FALL = HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_4);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;//上升沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim4,TIM_CHANNEL_4);
					////////////////同上
					HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4);
					
					__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC4);
					if(HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_4)!= HAL_OK)
					{
						printf("ERROR\r\n");
					}  
					
					CHANNEL_4_PULSE_WIDE = (CHANNEL_4_FALL > CHANNEL_4_RISE ? CHANNEL_4_FALL - CHANNEL_4_RISE:CHANNEL_4_FALL - CHANNEL_4_RISE + 60000);
					ICFLAG_4 = 1;      
				}
				//CH4_PWM_test(CHANNEL_3_PULSE_WIDE);
//				printf("channel4 pulsewidth = %d \r\n", CHANNEL_4_PULSE_WIDE);
				break;
			default: break;
		}
	}
	
	//****************************定时器1捕获****************************************
	if(htim->Instance == htim5.Instance)
	{
		switch(htim->Channel)
		{
			case HAL_TIM_ACTIVE_CHANNEL_1:
				if(ICFLAG_5){
					CHANNEL_5_RISE = HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_1);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;//下降沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim5,TIM_CHANNEL_1); 
					HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1);
					
					__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC1);
					HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_1);  
					
					ICFLAG_5 = 0;
				}
				else{
					CHANNEL_5_FALL = HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_1);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;//上升沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim5,TIM_CHANNEL_1); 
					HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1);
					
				__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC1);
					HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_1); 
					
					CHANNEL_5_PULSE_WIDE = (CHANNEL_5_FALL > CHANNEL_5_RISE ? CHANNEL_5_FALL - CHANNEL_5_RISE:CHANNEL_5_FALL - CHANNEL_5_RISE + 60000);
					ICFLAG_5 = 1;
				}
				//CH1_PWM_test(CHANNEL_1_PULSE_WIDE);
//				printf("channel5 pulsewidth = %d \r\n", CHANNEL_5_PULSE_WIDE);
				break;
			case HAL_TIM_ACTIVE_CHANNEL_2:
				if(ICFLAG_6){
					CHANNEL_6_RISE = HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_2);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;//下降沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim5,TIM_CHANNEL_2); 
					HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2);
					
					__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC2);
					HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_2);  
					
					ICFLAG_6 = 0;
				}
				else{
					CHANNEL_6_FALL = HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_2);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;//上升沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim5,TIM_CHANNEL_2); 
					HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2);
					
				__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC2);
					HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_2); 
					
					CHANNEL_6_PULSE_WIDE = (CHANNEL_6_FALL > CHANNEL_6_RISE ? CHANNEL_6_FALL - CHANNEL_6_RISE:CHANNEL_6_FALL - CHANNEL_6_RISE + 60000);
					ICFLAG_6 = 1;
				}
				//CH1_PWM_test(CHANNEL_1_PULSE_WIDE);
//				printf("channel5 pulsewidth = %d \r\n", CHANNEL_5_PULSE_WIDE);
				break;
			case HAL_TIM_ACTIVE_CHANNEL_3:
				if(ICFLAG_7){
					CHANNEL_7_RISE = HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_3);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;//下降沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim5,TIM_CHANNEL_3); 
					HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_3);
					
					__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC3);
					HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_3);  
					
					ICFLAG_7 = 0;
				}
				else{
					CHANNEL_7_FALL = HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_3);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;//上升沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim5,TIM_CHANNEL_3); 
					HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_3);
					
				__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC3);
					HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_3); 
					
					CHANNEL_7_PULSE_WIDE = (CHANNEL_7_FALL > CHANNEL_7_RISE ? CHANNEL_7_FALL - CHANNEL_7_RISE:CHANNEL_7_FALL - CHANNEL_7_RISE + 60000);
					ICFLAG_7 = 1;
				}
				//CH1_PWM_test(CHANNEL_1_PULSE_WIDE);
//				printf("channel5 pulsewidth = %d \r\n", CHANNEL_5_PULSE_WIDE);
				break;
		 /*************************************************************************/   
			case HAL_TIM_ACTIVE_CHANNEL_4:
					if(ICFLAG_8){
					CHANNEL_8_RISE = HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_4);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;//下降沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim5,TIM_CHANNEL_4);
					HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_4);
					
					__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC4);
					HAL_TIM_IC_Start_IT(htim,TIM_CHANNEL_4);  
					
					ICFLAG_8 = 0;
				}
				else{
					CHANNEL_8_FALL = HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_4);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;//上升沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim5,TIM_CHANNEL_4);
					HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_4);
					
					__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC4);
					HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_4); 
					
					CHANNEL_8_PULSE_WIDE = (CHANNEL_8_FALL > CHANNEL_8_RISE ? CHANNEL_8_FALL - CHANNEL_8_RISE:CHANNEL_8_FALL - CHANNEL_8_RISE + 60000);
					ICFLAG_8 = 1;
				}
				//CH2_PWM_test(CHANNEL_2_PULSE_WIDE);
//				printf("channel6 pulsewidth = %d \r\n", CHANNEL_6_PULSE_WIDE);
				break;
			default: break;
		}
	}
}


//手动发送mavlink信号
void MANUAL_CONTROL_Send(int16_t xpoint,int16_t ypoint)
{
	uint8_t system_id = 255;          // 发送本条消息帧的设备的系统编号（sys）   
	uint8_t component_id = 0;         // 发送本条消息帧的设备的单元编号（comp）
	uint8_t target = 0x01;            //目标系统
	int16_t x=0;
	int16_t y=ypoint;
	int16_t z=0; 
	int16_t r=0; 
	uint16_t buttons=0;
	mavlink_message_t msg;             // msg The MAVLink message to compress the data into
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;	

	mavlink_msg_manual_control_pack(system_id, component_id, &msg, target,x,y,z,r,buttons);
	len = mavlink_msg_to_send_buffer(buf, &msg);        
	//UART_Send_Str(buf,len);	
	BSP_USART_SendArray_LL( USART1,buf,sizeof(buf));
}



void RC_CHANNELS_OVERRIDE_Send(int16_t xpoint,int16_t ypoint)
{
	uint8_t system_id=255;
	uint8_t component_id=0;
	mavlink_message_t msg; 
	uint8_t target_system=1;
	uint8_t target_component=0;
	uint16_t chan1_raw=ypoint;
	uint16_t chan2_raw=65535;
	uint16_t chan3_raw=65535;
	uint16_t chan4_raw=65535; 
	uint16_t chan5_raw=65535;
	uint16_t chan6_raw=65535;
	uint16_t chan7_raw=65535;
	uint16_t chan8_raw=65535;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;	

	mavlink_msg_rc_channels_override_pack(system_id,component_id,&msg,target_system,target_component,
		chan1_raw,chan2_raw,chan3_raw,chan4_raw,chan5_raw,chan6_raw,chan7_raw,chan8_raw);
	len = mavlink_msg_to_send_buffer(buf, &msg); 
	BSP_USART_SendArray_LL( USART1,buf,sizeof(buf));
	//UART_Send_Str(buf,len);	


	mavlink_msg_rc_channels_override_pack(system_id,component_id,&msg,target_system,target_component,
		0,chan2_raw,chan3_raw,chan4_raw,chan5_raw,chan6_raw,chan7_raw,chan8_raw);

	len = mavlink_msg_to_send_buffer(buf, &msg); 
	//UART_Send_Str(buf,len);	
	BSP_USART_SendArray_LL( USART1,buf,sizeof(buf));
}


//*****心跳信号*************************
void heartbeat_Mavlink(void)
{
	uint8_t system_id=255;
	uint8_t component_id=0;
	mavlink_message_t heart_msg;
	uint8_t type=0x06;
	uint8_t autopilot=0x08;
	uint8_t base_mode=0xc0;
	uint32_t custom_mode=0x0000; 
	uint8_t system_status=0x04;

	uint8_t buf_head[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	mavlink_msg_heartbeat_pack( system_id,component_id, &heart_msg,type, autopilot,base_mode,custom_mode,system_status);
	len = mavlink_msg_to_send_buffer(buf_head, &heart_msg);
	//UART_Send_Str(buf_head,len);
	BSP_USART_SendArray_LL( USART1,buf_head,sizeof(buf_head));
}



//读取工作模式
int RC_Read(void)
{
	if(CHANNEL_6_PULSE_WIDE>=PWM_Mode_N1 && CHANNEL_6_PULSE_WIDE<=PWM_Mode_N2)
	{
		
	  return 1;
	}
  else if(CHANNEL_6_PULSE_WIDE>=PWM_Mode_N2 && CHANNEL_6_PULSE_WIDE<=PWM_Mode_N3)
	{
	  return 2;
	}
	else if(CHANNEL_6_PULSE_WIDE>=PWM_Mode_N3 && CHANNEL_6_PULSE_WIDE<=PWM_Mode_N4)
	{
	  return 3;
	}
	else return 1;
}

//各通道回中

void Back_to_Center(void)
{
	Set_PWM_Thr(4500);
	Set_PWM_Pitch(4500);
	Set_PWM_Roll(4500);
	Set_PWM_Yaw(4500);
	Set_PWM_Mode(4500);
}
/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
