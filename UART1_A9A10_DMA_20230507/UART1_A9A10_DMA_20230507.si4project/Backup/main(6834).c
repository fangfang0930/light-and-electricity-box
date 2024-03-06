/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stm32f4xx_hal_flash.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{ 	
	uint8_t  ThyristorWorkPattern;  //阀组类型
	uint8_t  ThyristorVoltageStage;      //阀组电压等级
	uint8_t  ThyristorNumPreArm1;         //阀臂1晶闸管个数
	uint8_t  ThyristorNumPreArm2;         //阀臂2晶闸管个数
	uint8_t  ThyristorNumPreArm3;         //阀臂3晶闸管个数
	uint8_t  ThyristorNumPreArm4;         //阀臂4晶闸管个数
	uint8_t  ThyristorTripNumPreArm;     //击穿动作个数
	uint8_t  ThyristorCheckCycles;       //击穿检测时间ms
	uint8_t  BodTripTripNumPreArm;     //BOD动作个数
	uint8_t  ThyristorCheckCycles1;       //BOD检测时间min
	uint8_t  GYH_CheckNum;               //高压合检测周波
	uint8_t  GYF_CheckNum;               //高压分检测周波
	uint8_t  CTL_Dead_CheckNum;               //主机死机检测
	uint8_t  FaultReset;             //复位	
	uint8_t  JC_ProtectChoice;		//击穿保护使能 
	uint8_t  BOD_ProtectChoice;		//BOD保护使能  
	uint8_t  PulseState;             //脉冲启动 0xaa
	uint32_t ABJCState;
	uint32_t BCJCState;
	uint32_t CAJCState;
	uint32_t ABBODState;
	uint32_t BABODState;
	uint32_t BCBODState;
	uint32_t CBBODState;
	uint32_t CABODState;	
  uint32_t ACBODState;
	uint8_t JC_Fault_flag;
	uint8_t BOD_Fault_flag;
	uint8_t BK_GYH;
	uint8_t ALARM;
	uint8_t FAULT;	
}SYS_PAR;

SYS_PAR Sys_Par;

typedef struct
{   
	//????????????
	uint16_t	NumOfGYH;
	uint16_t	NumOfGYF;
	uint32_t  NumOfParametersLoadError;

	uint16_t	NumResetStart;	//
	
	uint16_t  NumOfCheckThyristorBreakDownAB;  
	uint16_t  NumOfCheckThyristorBreakDownBC;
	uint16_t  NumOfCheckThyristorBreakDownCA; 

	uint16_t  NumOfThyristorCheckAntiJamming;  
	uint32_t NumOfParametersSaveError;
	uint16_t NumOfCheckBreakDownBoardNum;
	uint16_t NumOfTrip;

	//每相晶闸管抗干扰检测次数
	uint32_t ABJCAntiJammingCheckNum[MAX_CHECK_NUM_PRE_ARM_FOR_ANTI_JAMMING];
	uint32_t BAJCAntiJammingCheckNum[MAX_CHECK_NUM_PRE_ARM_FOR_ANTI_JAMMING];
	uint32_t BCJCAntiJammingCheckNum[MAX_CHECK_NUM_PRE_ARM_FOR_ANTI_JAMMING];
	uint32_t CBJCAntiJammingCheckNum[MAX_CHECK_NUM_PRE_ARM_FOR_ANTI_JAMMING];
	uint32_t CAJCAntiJammingCheckNum[MAX_CHECK_NUM_PRE_ARM_FOR_ANTI_JAMMING];
	uint32_t ACJCAntiJammingCheckNum[MAX_CHECK_NUM_PRE_ARM_FOR_ANTI_JAMMING];
	//每相BOD抗干扰检测次数
	uint32_t ABBodAntiJammingCheckNum[MAX_CHECK_NUM_PRE_ARM_FOR_ANTI_JAMMING];
	uint32_t BABodAntiJammingCheckNum[MAX_CHECK_NUM_PRE_ARM_FOR_ANTI_JAMMING];
	uint32_t BCBodAntiJammingCheckNum[MAX_CHECK_NUM_PRE_ARM_FOR_ANTI_JAMMING];
	uint32_t CBBodAntiJammingCheckNum[MAX_CHECK_NUM_PRE_ARM_FOR_ANTI_JAMMING];
	uint32_t CABodAntiJammingCheckNum[MAX_CHECK_NUM_PRE_ARM_FOR_ANTI_JAMMING];
	uint32_t ACBodAntiJammingCheckNum[MAX_CHECK_NUM_PRE_ARM_FOR_ANTI_JAMMING];

	uint16_t	NumResetEnd;  //
	
}CHECK_NUM;

CHECK_NUM CheckNum;

typedef struct
{ 	
  uint16_t   CtrlStart;
	uint16_t   ThyristorBreakDownStart;
	uint16_t	 BodActionCheckStart;

	/*************************************************/
	//?1ms??-1??????
	uint16_t  CounterPreMsSubStart;//
	uint16_t CounterFlashLED;
	uint16_t  CounterCheckLostPulse;
  uint16_t  CounterPreMsSubEnd;//

	/*************************************************/
	//?1ms??+1??????
	uint16_t CounterPreMsAddStart;//
	uint16_t Counter_FaultTrip;	
	uint16_t  CounterPreMsAddEnd;//
	
}COUNT;

COUNT count;

typedef struct
{
  uint16_t PulseStart;
	uint16_t ThyristorCheckCyclesStart;
	uint16_t UpdateActionAB[32];
  uint16_t UpdateActionBA[32];
	uint16_t UpdateActionBC[32];
	uint16_t UpdateActionCB[32];
	uint16_t UpdateActionCA[32];
	uint16_t UpdateActionAC[32];
	uint16_t PulseStart_delay;
	uint16_t PulseStart_delay_counter;//20
	uint32_t BODWarningCycles;
	uint32_t BODTripTripNumPreArm;
	uint32_t JCABState;
	uint32_t JCBCState;
	uint32_t JCCAState;//25
	uint16_t JCABNum;
	uint16_t JCBCNum;
	uint16_t JCCANum;
	uint32_t BODABState;
	uint32_t BODBCState;
	uint32_t BODCAState;
	uint32_t BODBAState;
	uint32_t BODCBState;//45
	uint32_t BODACState;
	uint16_t BODABNum;
	uint16_t BODBCNum;
	uint16_t BODCANum;
	uint16_t BODBANum;//50
	uint16_t BODCBNum;
	uint16_t BODACNum;
	uint32_t JCABStateTemp;
	uint32_t JCBCStateTemp;
	uint32_t JCCAStateTemp;//55
	uint32_t BODABStateTemp;
	uint32_t BODBCStateTemp;
	uint32_t BODCAStateTemp;
	uint16_t BODStartFlag;//65
	uint32_t BODTiming1;
	uint32_t BODTiming2;
	uint32_t BODTiming3;
	uint32_t BODTiming4;
	uint32_t BODTiming5;
	uint32_t BODTiming6;
	uint32_t BODTiming7;
	uint32_t BODTiming8;
	uint32_t BODTiming9;
	uint32_t BODTiming10;
	uint32_t BODTiming11;
	uint32_t BODTiming12;
	uint16_t ThyristorCheckCyclesStartBOD;
	uint16_t PulseEnd;	
	uint16_t ABTemp1;
	uint16_t BATemp1;
	uint16_t BCTemp1;
	uint16_t CBTemp1;
	uint16_t CATemp1;
	uint16_t ACTemp1;
	uint16_t ABTemp2;
	uint16_t BATemp2;
	uint16_t BCTemp2;
	uint16_t CBTemp2;
	uint16_t CATemp2;
	uint16_t ACTemp2;
	uint16_t ABTemp3;
	uint16_t BATemp3;
	uint16_t BCTemp3;
	uint16_t CBTemp3;
	uint16_t CATemp3;
	uint16_t ACTemp3;
	uint16_t ABTemp4;
	uint16_t BATemp4;
	uint16_t BCTemp4;
	uint16_t CBTemp4;
	uint16_t CATemp4;
	uint16_t ACTemp4;
	uint16_t BodABTemp1;
	uint16_t BodABTemp2;
	uint16_t BodABTemp3;
	uint16_t BodABTemp4;
		
	uint16_t BodBATemp1;
	uint16_t BodBATemp2;
	uint16_t BodBATemp3;
	uint16_t BodBATemp4;
		
	uint16_t BodBCTemp1;
	uint16_t BodBCTemp2;
	uint16_t BodBCTemp3;
	uint16_t BodBCTemp4;
		
	uint16_t BodCBTemp1;
	uint16_t BodCBTemp2;
	uint16_t BodCBTemp3;
	uint16_t BodCBTemp4;
		
	uint16_t BodCATemp1;
	uint16_t BodCATemp2;
	uint16_t BodCATemp3;
	uint16_t BodCATemp4;
		
	uint16_t BodACTemp1;
	uint16_t BodACTemp2;
	uint16_t BodACTemp3;
	uint16_t BodACTemp4;

  uint32_t JCABTemp1;
	uint32_t JCABTemp2;
	uint32_t JCBCTemp1;
	uint32_t JCBCTemp2;
	uint32_t JCCATemp1;
	uint32_t JCCATemp2;
	uint32_t JCABTemp3;
	uint32_t JCABTemp4;
	uint32_t JCBCTemp3;
	uint32_t JCBCTemp4;
	uint32_t JCCATemp3;
	uint32_t JCCATemp4;
	uint32_t JCABTemp;
	uint32_t JCBCTemp;
	uint32_t JCCATemp;
	
}SIMPLE_Par;

SIMPLE_Par Simple;

typedef struct
{    
	uint16_t  WarningNum;     	//
	uint16_t  FaultNum;			//
	uint16_t  TripFlag; 			//TCR跳闸
	uint16_t  ForbidPulseFlag;	//
	uint16_t  FaultTripStatus;   //
}PROTECT;

PROTECT Protect_Ctrl;
/*
typedef struct
{
		uint16_t num,numtemp,numcrc,numcrc1;
		uint16_t addr,addrtemp;
		uint16_t crc,crcin;
	  uint16_t id;
}UART_RX;

UART_RX uart_rx;*/
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define MAX_CoilState_LEN  255

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void SetLED(void);
//uint16_t STM32F407FLASH_Write(uint32_t startaddr,uint32_t endaddr,uint32_t *data);
uint16_t STM32F407FLASH_Write(uint32_t startaddr,uint32_t endaddr,uint8_t *data);
//uint8_t STM32F407FLASH_Read(uint32_t startaddr,uint32_t endaddr,uint32_t *data);
uint8_t STM32F407FLASH_Read(uint32_t startaddr,uint32_t endaddr,uint8_t *data);
void ALLDatainit(void);
void ModbusR_T(void);
void Send_Response(void);
void Run_Message(void);
void Load_Message(uint8_t *data,uint8_t addr);
void DealFault(PROTECT *P);
void ThyristorBreakDownProtectLightElectricity(SYS_PAR *V,COUNT *C, CHECK_NUM *N, PROTECT *P);
void BodActionProtect(SYS_PAR *V, COUNT *C, CHECK_NUM *N, PROTECT *P);
void Sys_Par_Int(void);
void BodActionCheck(uint32_t  BodInputTemp,//BOD??
					  uint16_t  ThyristorNumPreArm,//?????
					  uint32_t* BodCheckNum,//BOD?????
					  uint32_t  BODTiming1,//BOD?????
					  uint32_t  BodTripCycles,//BOD?????
					  uint16_t  ThyristorTripNumPreArm,//?????
					  uint32_t* BodActionRtu,
					  uint32_t* BodActiveNum,
					  uint32_t* BodTiming3,
					  uint32_t* BodTiming4,
					  uint32_t BODTiming2);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t senbuff[] = "Hello, I am iCore3-----!\r\n";
uint8_t senbuff1[] = "RTX init osEventFlagsId_t error-----!\r\n";
uint8_t sendtxif = 0,sendtxcnt = 0;
uint8_t sendtxif1 = 0,sendtxcnt1 = 0;
uint8_t syn_jc = 0,syn_bod = 0,syn_fpga = 0,GYH = 0;
uint8_t Send_Buffer[ComTxBufferSize];
uint8_t Run_Msg_Buffer[64];
uint8_t gyhchecknum,gyfchecknum;
uint16_t buffer_fpga_temp[FPGA_RAM_BUF];//FPGA_RAM_BUF刷没了
uint16_t CRC16 (uint8_t *puchMsg, uint16_t usDataLen);
uint8_t key1_status,key2_status,key3_status;
uint16_t ThyristorBreakDownCheck(uint32_t  ScrTemp, 					//
																 uint16_t  ThyristorNumPreArm,			//
																 uint32_t* SCRAntiJammingCheckNum,		//
																 uint16_t  NumOfThyristorCheckAntiJamming,//
																 uint16_t  ThyristorCheckCycles,			//						
																 uint16_t  ThyristorTripNumPreArm,		//
																 uint32_t* ThyristorBreakDwonState,		//
																 uint32_t* ThyristorBreakDwonNum,			//
																 uint16_t ThyristorCheckCycles1);			//
volatile int LED_character=0; // ??????
volatile int LED_num=0;
//uint8_t flashwrite[30] = {12,22,33,44,55,66,47,58,232,23,232,12,34,24,44,42,21,23,32,32,56,66,88,11,22,33,44,55,66,77};
//uint8_t flashread[30] = {0};
uint8_t rom_data[30] = {0};
uint32_t Addr_start = 0x80e0000;
uint32_t Addr_end = 0x80e0030;
uint8_t ii;
uint32_t LED_scan[8]={0x00100000,0x00200000,0x00400000,0x00800000,0x01000000,0x02000000,0x04000000,0x00000000};
// ??, 0-9, A-Z 
uint32_t LED_content[38][8]={{0x00000000,0x000E0000,0x00060000,0x000A0000,0x000C0000,0x000E0000,0x00000000,0x00000000},\
                           {0x001B0000,0x00190000,0x001A0000,0x001B0000,0x001B0000,0x001B0000,0x00000000,0x00000000},\
						   {0x00110000,0x000E0000,0x000F0000,0x00100000,0x001E0000,0x001E0000,0x00000000,0x00000000},\
						   {0x00110000,0x000E0000,0x000F0000,0x00110000,0x000F0000,0x000E0000,0x00110000,0x00000000},\
						   {0x001A0000,0x001A0000,0x001A0000,0x00000000,0x001B0000,0x001B0000,0x001B0000,0x00000000},\
						   {0x00000000,0x001E0000,0x00100000,0x000F0000,0x000F0000,0x000E0000,0x00110000,0x00000000},\
						   {0x00110000,0x000E0000,0x001E0000,0x00100000,0x000E0000,0x000E0000,0x00110000,0x00000000},\
						   {0x00000000,0x000F0000,0x00170000,0x001B0000,0x001D0000,0x001D0000,0x001D0000,0x00000000},\
						   {0x00110000,0x000E0000,0x000E0000,0x00110000,0x000E0000,0x000E0000,0x00110000,0x00000000},\
						   {0x00110000,0x000E0000,0x000E0000,0x00010000,0x000F0000,0x000E0000,0x00110000,0x00000000},\
							 {0x001B0000,0x00150000,0x000E0000,0x00000000,0x000E0000,0x000E0000,0x000E0000,0x00000000},\
							 {0x00100000,0x000E0000,0x000E0000,0x00100000,0x000E0000,0x000E0000,0x00100000,0x00000000},\
							 {0x00110000,0x000E0000,0x001E0000,0x001E0000,0x001E0000,0x000E0000,0x00110000,0x00000000},\
							 {0x00180000,0x00160000,0x000E0000,0x000E0000,0x000E0000,0x00160000,0x00180000,0x00000000},\
							 {0x00000000,0x001E0000,0x001E0000,0x00000000,0x001E0000,0x001E0000,0x00000000,0x00000000},\
							 {0x00000000,0x001E0000,0x001E0000,0x00000000,0x001E0000,0x001E0000,0x001E0000,0x00000000},\
							 {0x00110000,0x000E0000,0x001E0000,0x001E0000,0x00060000,0x000E0000,0x00010000,0x00000000},\
							 {0x000E0000,0x000E0000,0x000E0000,0x00000000,0x000E0000,0x000E0000,0x000E0000,0x00000000},\
							 {0x00110000,0x001B0000,0x001B0000,0x001B0000,0x001B0000,0x001B0000,0x00110000,0x00000000},\
							 {0x00010000,0x00170000,0x00170000,0x00170000,0x00170000,0x00160000,0x00190000,0x00000000},\
							 {0x000E0000,0x00160000,0x001A0000,0x001C0000,0x001A0000,0x00160000,0x000E0000,0x00000000},\
							 {0x001E0000,0x001E0000,0x001E0000,0x001E0000,0x001E0000,0x001E0000,0x00000000,0x00000000},\
							 {0x001F0000,0x00000000,0x000A0000,0x000A0000,0x000A0000,0x000A0000,0x001F0000,0x00000000},\
							 {0x001F0000,0x000E0000,0x000C0000,0x00080000,0x00060000,0x000E0000,0x001F0000,0x00000000},\
							 {0x001F0000,0x00110000,0x000E0000,0x000E0000,0x000E0000,0x00110000,0x001F0000,0x00000000},\
							 {0x00100000,0x000E0000,0x000E0000,0x00100000,0x001E0000,0x001E0000,0x001E0000,0x00000000},\
							 {0x00110000,0x000E0000,0x000E0000,0x000E0000,0x000A0000,0x00060000,0x00010000,0x00000000},\
							 {0x00100000,0x000E0000,0x000E0000,0x00100000,0x001A0000,0x00160000,0x000E0000,0x00000000},\
							 {0x00110000,0x000E0000,0x001E0000,0x00110000,0x000F0000,0x000E0000,0x00110000,0x00000000},\
							 {0x00000000,0x001B0000,0x001B0000,0x001B0000,0x001B0000,0x001B0000,0x001B0000,0x00000000},\
							 {0x000E0000,0x000E0000,0x000E0000,0x000E0000,0x000E0000,0x000E0000,0x00110000,0x00000000},\
							 {0x000E0000,0x000E0000,0x000E0000,0x000E0000,0x000E0000,0x00150000,0x001B0000,0x00000000},\
							 {0x001F0000,0x000A0000,0x000A0000,0x000A0000,0x000A0000,0x00150000,0x001F0000,0x00000000},\
							 {0x001F0000,0x000E0000,0x00150000,0x001B0000,0x00150000,0x000E0000,0x001F0000,0x00000000},\
							 {0x000E0000,0x00150000,0x001B0000,0x001B0000,0x001B0000,0x001B0000,0x001B0000,0x00000000},\
							 {0x001F0000,0x00000000,0x00170000,0x001B0000,0x001D0000,0x00000000,0x001F0000,0x00000000},\
							 {0x001D0000,0x001B0000,0x00000000,0x001B0000,0x00110000,0x001B0000,0x00000000,0x00000000},\
							 {0x001F0000,0x00150000,0x00150000,0x00150000,0x00150000,0x000A0000,0x001F0000,0x00000000}};
														

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
 //   static int key_status = 0;
  /* USER CODE END 1 */

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_FSMC_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
	Sys_Par_Int();
	ALLDatainit();
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_UART_Receive_DMA(&huart1,ComData3.ucRxBuf,ComRxBufferSize);
	//--------参数读取---------//
	unsigned short int i;//FPGA读写例程
/*	unsigned short int fsmc_read_data,i;//读写例程
	for(i = 0;i < 512;i++){
		 fpga_write(i,i);                  
	}	
	for(i = 0;i < 512;i++){
		 fsmc_read_data = fpga_read(i);  
		 if(fsmc_read_data != i){
				LED_RED_ON;
		 }
//		 buffer_fpga_temp[i] = fsmc_read_data;
	}	
	LED_BLUE_ON;*/
	Sys_Par.BK_GYH = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//flash读写
		//STM32F407FLASH_Write(0x80e0000,0x80e0020,flashwrite);
		//LED_BLUE_ON;
    //HAL_Delay(1000);
		//STM32F407FLASH_Read(0x80e0000,0x80e0020,flashread);
		//LED_BLUE_OFF;
		//HAL_Delay(1000);
		//led灯读写
		//LED_character = 6;//W
		//SetLED();
		LED1_OFF;LED2_OFF;LED3_OFF;LED4_OFF;LED5_OFF;LED6_OFF;LED7_OFF;
		LED11_OFF;LED12_OFF;LED13_OFF;LED14_OFF;LED15_OFF;
		
	
    //读取FPGA双口RAM数据
		if(syn_fpga == 0xaa)
		{
			syn_fpga = 0;
			for(i=0;i<FPGA_RAM_BUF;i++)
			{
				buffer_fpga_temp[i] = fpga_read(i);
	//				fpga_write(i,i);                  //向FPGA写入数据
			}		
		}
//	ThyristorBreakDownProtectLightElectricity(&Sys_Par, &count, &CheckNum, &Protect_Ctrl);	
		//--------击穿和BOD保护检测-------------//
		if((GYH)&&(Protect_Ctrl.FaultNum == 0))
		{
			if((Sys_Par.JC_ProtectChoice & ENABLE_TRANSISTOR_BREAKDOWN_PROTECT) == ENABLE_TRANSISTOR_BREAKDOWN_PROTECT)
			{
				ThyristorBreakDownProtectLightElectricity(&Sys_Par, &count, &CheckNum, &Protect_Ctrl);
			}
			else
			{
					syn_jc = 0;
			    Simple.JCABState = 0;
			    Simple.JCBCState = 0;
			    Simple.JCCAState = 0;	
					Simple.JCABNum = 0;	
					Simple.JCBCNum = 0;	
					Simple.JCCANum = 0;	
					Sys_Par.JC_Fault_flag = 0;			
			}
			if((Sys_Par.BOD_ProtectChoice & ENABLE_TRANSISTOR_BOD_PROTECT) == ENABLE_TRANSISTOR_BOD_PROTECT)
			{
				BodActionProtect(&Sys_Par, &count, &CheckNum, &Protect_Ctrl);
			}
			else
			{
				syn_bod = 0;
				Simple.BODABState = 0;
				Simple.BODBAState = 0;
				Simple.BODBCState = 0;
				Simple.BODCBState = 0;
				Simple.BODCAState = 0;
				Simple.BODACState = 0;				
			}		
		}
		else
		{
			if(Protect_Ctrl.FaultNum == 0) 
			{
				syn_jc = 0;	
				syn_bod = 0;
				Simple.JCABState = 0;
				Simple.JCBCState = 0;
				Simple.JCCAState = 0;
				Simple.BODABState = 0;
				Simple.BODBAState = 0;
				Simple.BODBCState = 0;
				Simple.BODCBState = 0;
				Simple.BODCAState = 0;
				Simple.BODACState = 0;			
			}
		}
	
		//--------故障复位-------------//
		
/*		if(IN1_STATE == KEY_UP) 
			key_status = KEY_UP;
		else if(IN1_STATE == KEY_DOWN)  
			key_status = KEY_DOWN;*/
		
		key1_status = IN1_STATE;
		key2_status = IN2_STATE;
		key3_status = IN3_STATE;
		
		
		
		//--------保护动作跳闸-------------//
		if(Protect_Ctrl.FaultNum > 0)
		{	
			TRIP_OFF;
			FAULT_OFF;
			DEAD_OFF;
		  Sys_Par.FAULT = 0x55;//有故障
		}	
		if(Protect_Ctrl.WarningNum > 0)
		{
			Sys_Par.ALARM = 0x55;//有告警
		}
		else
		{
			Sys_Par.ALARM = 0xaa;//有告警
		}
		
		if(Sys_Par.FaultReset == 0xaa)
		{
			Sys_Par.FaultReset = 0;
			TRIP_ON;
			FAULT_ON;
			DEAD_ON;
			Sys_Par.JC_Fault_flag = 0;
			Sys_Par.BOD_Fault_flag = 0;
			Sys_Par.ALARM = 0;
			Sys_Par.FAULT = 0;
			Protect_Ctrl.FaultNum = 0;
			Protect_Ctrl.TripFlag = 0;	
			Protect_Ctrl.ForbidPulseFlag = 0; 
			Simple.JCABState = 0;
			Simple.JCBCState = 0;
			Simple.JCCAState = 0;
			Simple.BODABState = 0;
			Simple.BODBAState = 0;
			Simple.BODBCState = 0;
			Simple.BODCBState = 0;
			Simple.BODCAState = 0;
			Simple.BODACState = 0;
			Simple.JCABNum = 0;	
			Simple.JCBCNum = 0;	
			Simple.JCCANum = 0;	
			Simple.ABTemp1 = 0;	
			Simple.ABTemp2 = 0;	
			Simple.ABTemp3 = 0;	
			Simple.ABTemp4 = 0;
			Simple.BCTemp1 = 0;	
			Simple.BCTemp2 = 0;	
			Simple.BCTemp3 = 0;	
			Simple.BCTemp4 = 0;	
			Simple.CATemp1 = 0;	
			Simple.CATemp2 = 0;	
			Simple.CATemp3 = 0;	
			Simple.CATemp4 = 0;

			Simple.BATemp1 = 0;	
			Simple.BATemp2 = 0;	
			Simple.BATemp3 = 0;	
			Simple.BATemp4 = 0;
			Simple.CBTemp1 = 0;	
			Simple.CBTemp2 = 0;	
			Simple.CBTemp3 = 0;	
			Simple.CBTemp4 = 0;	
			Simple.ACTemp1 = 0;	
			Simple.ACTemp2 = 0;	
			Simple.ACTemp3 = 0;	
			Simple.ACTemp4 = 0;
			
			Sys_Par.JC_Fault_flag = 0;
			for(ii=0;ii<MAX_CHECK_NUM_PRE_ARM_FOR_ANTI_JAMMING;ii++)
			{
				CheckNum.ABJCAntiJammingCheckNum[ii] = 0;
				CheckNum.BAJCAntiJammingCheckNum[ii] = 0;
				CheckNum.BCJCAntiJammingCheckNum[ii] = 0;
				CheckNum.CBJCAntiJammingCheckNum[ii] = 0;
				CheckNum.CAJCAntiJammingCheckNum[ii] = 0;
				CheckNum.ACJCAntiJammingCheckNum[ii] = 0;
				CheckNum.ABBodAntiJammingCheckNum[ii] = 0;
				CheckNum.BABodAntiJammingCheckNum[ii] = 0;
				CheckNum.BCBodAntiJammingCheckNum[ii] = 0;
				CheckNum.CBBodAntiJammingCheckNum[ii] = 0;
				CheckNum.CABodAntiJammingCheckNum[ii] = 0;
				CheckNum.ACBodAntiJammingCheckNum[ii] = 0;			
			}
		}
		//底板DB9串口1(Modbus)
		Run_Message();
		if(sendtxif)
		{
			ModbusR_T();		//串口收发处理		
			sendtxif = sendtxcnt = 0;
		}
		
//核心板板载串口4		
/*		if(sendtxif1){
			HAL_UART_Transmit_DMA(&huart4,ComData3.ucTxBuf,sendtxcnt1);
			//HAL_UART_Transmit_DMA(&huart1,senbuff1,sendtxcnt);
			if(sendtxcnt1 != 25){
				sendtxcnt1 = 0;
			}
			sendtxif1 = sendtxcnt1 = 0;
			HAL_Delay(50);
			for(count = 0;count < 200;count++){
				ComData3.ucTxBuf[count] = 0;
			}
		}*/
		
		//HAL_Delay(500);
		//LED_BLUE_OFF;
		
		
		//1000接收一组数据
		
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*****************ALLDatainit*******************************/
void ALLDatainit(void){
	
	uint16_t i = 0;
	//uart init
	ComData3.uiRxStartFlag		= 0;
	ComData3.uiTxStartFlag      = 0;
	ComData3.uiTxCount          = 0;
	ComData3.uiRxCount          = 0;
	ComData3.uiTxCountAll	    = 0;   
	ComData3.uiRxReadyFlag      = 0;
	ComData3.uiRxTimeOut        = 0;
	ComData3.uiRxTimeOutTh      = 5;//9600 3.5T 
	for(i=0;i<ComRxBufferSize;i++)
	{
		ComData3.ucRxBuf[i]=0;
	}
	for(i=0;i<ComTxBufferSize;i++)
	{
		ComData3.ucTxBuf[i]=0;
	}
	
}
uint16_t CRC16 (uint8_t *puchMsg, uint16_t usDataLen) /* The function returns the CRC as a unsigned short type */
{  
/* message to calculate CRC upon */
/* quantity of bytes in message */
  {
  uint8_t uchCRCHi = 0xFF ; /* high byte of CRC initialized */
  uint8_t uchCRCLo = 0xFF ; /* low byte of CRC initialized */
  uint8_t uIndex ; /* will index into CRC lookup table */
  while (usDataLen--) /* pass through message buffer */
  {
  uIndex = uchCRCLo ^ *puchMsg++ ; /* calculate the CRC */
  uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex] ;
  uchCRCHi = auchCRCLo[uIndex] ;
  }
  return (uchCRCHi << 8 | uchCRCLo) ;
  }
}
//串口收发
void ModbusR_T(void)
{
		uint8_t i=0,j=0,CRC_L,CRC_H;
		uint16_t num,numtemp,numcrc,numcrc1;
		uint16_t addr;
		uint16_t crc,crcin;
	  uint16_t id;
//	  uint8_t RxBuf[64]={0};
	
		id = ComData3.ucRxBuf[0];
	  addr = (ComData3.ucRxBuf[2]<<8) + ComData3.ucRxBuf[3];      //起始地址
//	  addrtemp = (addr-1)<<1;
		num =  (ComData3.ucRxBuf[4]<<8) + ComData3.ucRxBuf[5];  //读取长度，字节
	  numtemp = num<<1;
		if(ComData3.ucRxBuf[1] == FUNCTION_CODE3)
			numcrc = numtemp + 3;
		else if(ComData3.ucRxBuf[1] == FUNCTION_CODE10) 
			numcrc = numtemp + 7;
		else
			numcrc = 0;
		numcrc1 = numcrc + 1;
		crc = CRC16(ComData3.ucRxBuf,numcrc);
		crcin = ComData3.ucRxBuf[numcrc] + ((uint16_t)ComData3.ucRxBuf[numcrc1]<<8);
		
//		if((id!= ADDR_SELF) || (addr > 0xffff) || (num>2000) || (crc != crcin)) 
		if((id!= ADDR_SELF) || (addr > 0xffff) || (num>2000)) 
		{
		return;
		}   //地址，长度，CRC错误 
			
		i = 0;
		ComData3.ucTxBuf[i++] = ADDR_SELF;         //本机地址	
	
		if(ComData3.ucRxBuf[1] == FUNCTION_CODE3 && ComData3.ucRxBuf[3] == 0x16){
			ComData3.ucTxBuf[i++] = FUNCTION_CODE3;    // 功能码3
			ComData3.ucTxBuf[i++] = numtemp;           //字节数
			for(j=0;j<16;j++)  //数据
			{
				ComData3.ucTxBuf[i++] = Run_Msg_Buffer[j+42];
			}
		}else if(ComData3.ucRxBuf[1] == FUNCTION_CODE3 && ComData3.ucRxBuf[3] == 0x3){
			ComData3.ucTxBuf[i++] = FUNCTION_CODE3;    // 功能码3
			ComData3.ucTxBuf[i++] = numtemp;           //字节数0x2A
			for(j=0;j<38;j++){
				ComData3.ucTxBuf[i++] = Run_Msg_Buffer[j+4];
			}
		}else if(ComData3.ucRxBuf[1] == FUNCTION_CODE10 && ComData3.ucRxBuf[3] == 0x47){
			Load_Message(ComData3.ucRxBuf,addr);
			ComData3.ucTxBuf[i++] = FUNCTION_CODE10;    // 功能码10
			ComData3.ucTxBuf[i++] = 0;               //字
			ComData3.ucTxBuf[i++] = 0x47;               //字
			ComData3.ucTxBuf[i++] = 0;               //字
			ComData3.ucTxBuf[i++] = 1;
		}else if(ComData3.ucRxBuf[1] == FUNCTION_CODE10 && ComData3.ucRxBuf[3] == 0x41){
			Load_Message(ComData3.ucRxBuf,addr);
			ComData3.ucTxBuf[i++] = FUNCTION_CODE10;    // 功能码10
			ComData3.ucTxBuf[i++] = 0;               //字
			ComData3.ucTxBuf[i++] = 0x41;               //字
			ComData3.ucTxBuf[i++] = 0;               //字
			ComData3.ucTxBuf[i++] = num;		
		}
		
			CRC_L= CRC16(ComData3.ucTxBuf,i);
			CRC_H= CRC16(ComData3.ucTxBuf,i)>>8;
			ComData3.ucTxBuf[i++] = CRC_L;			
			ComData3.ucTxBuf[i++] = CRC_H;
			sendtxcnt = i;
			HAL_UART_Transmit_DMA(&huart1,ComData3.ucTxBuf,sendtxcnt);//发送
//		HAL_UART_Transmit_DMA(&huart1,ComData3.ucTxBuf,sendtxcnt);//发送
/*    for(j=0;j<i;j++)
    {
      Uart_SendByte(Send_Buffer[j]);  //????
    }*/
}
void Run_Message(void)
{
/*	Sys_Par.ABJCState = buffer_fpga_temp[1];
	  Sys_Par.ABJCState = (Sys_Par.ABJCState<<16)+buffer_fpga_temp[0];
	  Sys_Par.BCJCState = buffer_fpga_temp[3];
	  Sys_Par.BCJCState = (Sys_Par.BCJCState<<16)+buffer_fpga_temp[2];	
		Sys_Par.CAJCState = buffer_fpga_temp[5];
	  Sys_Par.CAJCState = (Sys_Par.CAJCState<<16)+buffer_fpga_temp[4];

	  Sys_Par.ABBODState = buffer_fpga_temp[7];
	  Sys_Par.ABBODState = (Sys_Par.ABBODState<<16)+buffer_fpga_temp[6];
	  Sys_Par.BABODState = buffer_fpga_temp[9];
	  Sys_Par.BABODState = (Sys_Par.BABODState<<16)+buffer_fpga_temp[8];
	  
	  Sys_Par.BCBODState = buffer_fpga_temp[11];
	  Sys_Par.BCBODState = (Sys_Par.BCBODState<<16)+buffer_fpga_temp[10];
	  Sys_Par.CBBODState = buffer_fpga_temp[13];
	  Sys_Par.CBBODState = (Sys_Par.CBBODState<<16)+buffer_fpga_temp[12];
	
		Sys_Par.CABODState = buffer_fpga_temp[15];
	  Sys_Par.CABODState = (Sys_Par.CABODState<<16)+buffer_fpga_temp[14];
	  Sys_Par.ACBODState = buffer_fpga_temp[17];
	  Sys_Par.ACBODState = (Sys_Par.ACBODState<<16)+buffer_fpga_temp[16];*/
		
	  Sys_Par.ABJCState = Simple.JCABState;
	  Sys_Par.BCJCState = Simple.JCBCState;
		Sys_Par.CAJCState = Simple.JCCAState;

	  Sys_Par.ABBODState = Simple.BODABState;
	  Sys_Par.BABODState = Simple.BODBAState;
	  
	  Sys_Par.BCBODState = Simple.BODBCState;
	  Sys_Par.CBBODState = Simple.BODCBState;
	
		Sys_Par.CABODState = Simple.BODCAState;
	  Sys_Par.ACBODState = Simple.BODACState;
		if((Simple.JCABState>0)||(Simple.JCBCState>0)||(Simple.JCCAState>0)
			||(Simple.BODABState>0)||(Simple.BODBAState>0)||(Simple.BODBCState>0)
		||(Simple.BODCBState>0)||(Simple.BODCAState>0)||(Simple.BODACState>0))
		{
			Sys_Par.ALARM = 0x55; 
		}
		else
		{
			Sys_Par.ALARM = 0; 
		}

	  Run_Msg_Buffer[0] = 0;
		Run_Msg_Buffer[1] = 0;
		Run_Msg_Buffer[2] = 0;
		Run_Msg_Buffer[3] = 0;
		Run_Msg_Buffer[4] = Sys_Par.ALARM;//告警
		Run_Msg_Buffer[5] = Sys_Par.FAULT;//故障
		Run_Msg_Buffer[6] = (uint8_t)Sys_Par.ABJCState;         //AB晶闸管击穿1~8
		Run_Msg_Buffer[7] = (uint8_t)(Sys_Par.ABJCState>>8);    //AB晶闸管击穿9~16
		Run_Msg_Buffer[8] = (uint8_t)(Sys_Par.ABJCState>>16);   //AB晶闸管击穿17~24
	  Run_Msg_Buffer[9] = (uint8_t)(Sys_Par.ABJCState>>24);   //AB晶闸管击穿25~32
		Run_Msg_Buffer[10] = (uint8_t)Sys_Par.BCJCState;        //BC晶闸管击穿1~8
		Run_Msg_Buffer[11] = (uint8_t)(Sys_Par.BCJCState>>8);   //BC晶闸管击穿9~16
		Run_Msg_Buffer[12] = (uint8_t)(Sys_Par.BCJCState>>16);  //BC晶闸管击穿17~24
	  Run_Msg_Buffer[13] = (uint8_t)(Sys_Par.BCJCState>>24);  //BC晶闸管击穿25~32		
	  Run_Msg_Buffer[14] = (uint8_t)Sys_Par.CAJCState;        //CA晶闸管击穿1~8
		Run_Msg_Buffer[15] = (uint8_t)(Sys_Par.CAJCState>>8);   //CA晶闸管击穿9~16
		Run_Msg_Buffer[16] = (uint8_t)(Sys_Par.CAJCState>>16);  //CA晶闸管击穿17~24
	  Run_Msg_Buffer[17] = (uint8_t)(Sys_Par.CAJCState>>24);  //CA晶闸管击穿25~32	
	  Run_Msg_Buffer[18] = (uint8_t)Sys_Par.ABBODState;        //AB+ BOD动作 1~8
		Run_Msg_Buffer[19] = (uint8_t)(Sys_Par.ABBODState>>8);   //AB+ BOD动作 9~16
		Run_Msg_Buffer[20] = (uint8_t)(Sys_Par.ABBODState>>16);  //AB+ BOD动作 17~24
	  Run_Msg_Buffer[21] = (uint8_t)(Sys_Par.ABBODState>>24);  //AB+ BOD动作 25~32	
	  Run_Msg_Buffer[22] = (uint8_t)Sys_Par.BABODState;        //AB- BOD动作 1~8
		Run_Msg_Buffer[23] = (uint8_t)(Sys_Par.BABODState>>8);   //AB- BOD动作 9~16
		Run_Msg_Buffer[24] = (uint8_t)(Sys_Par.BABODState>>16);  //AB- BOD动作 17~24
	  Run_Msg_Buffer[25] = (uint8_t)(Sys_Par.BABODState>>24);  //AB- BOD动作 25~32			
	  Run_Msg_Buffer[26] = (uint8_t)Sys_Par.BCBODState;        //BC+ BOD动作 1~8
		Run_Msg_Buffer[27] = (uint8_t)(Sys_Par.BCBODState>>8);   //BC+ BOD动作 9~16
		Run_Msg_Buffer[28] = (uint8_t)(Sys_Par.BCBODState>>16);  //BC+ BOD动作 17~24
	  Run_Msg_Buffer[29] = (uint8_t)(Sys_Par.BCBODState>>24);  //BC+ BOD动作 25~32
	  Run_Msg_Buffer[30] = (uint8_t)Sys_Par.CBBODState;        //BC- BOD动作 1~8
		Run_Msg_Buffer[31] = (uint8_t)(Sys_Par.CBBODState>>8);   //BC- BOD动作 9~16
		Run_Msg_Buffer[32] = (uint8_t)(Sys_Par.CBBODState>>16);  //BC- BOD动作 17~24
	  Run_Msg_Buffer[33] = (uint8_t)(Sys_Par.CBBODState>>24);  //BC- BOD动作 25~32
	  Run_Msg_Buffer[34] = (uint8_t)Sys_Par.CABODState;        //CA+ BOD动作 1~8
		Run_Msg_Buffer[35] = (uint8_t)(Sys_Par.CABODState>>8);   //CA+ BOD动作 9~16
		Run_Msg_Buffer[36] = (uint8_t)(Sys_Par.CABODState>>16);  //CA+ BOD动作 17~24
	  Run_Msg_Buffer[37] = (uint8_t)(Sys_Par.CABODState>>24);  //CA+ BOD动作 25~32		
	  Run_Msg_Buffer[38] = (uint8_t)Sys_Par.ACBODState;        //CA- BOD动作 1~8
		Run_Msg_Buffer[39] = (uint8_t)(Sys_Par.ACBODState>>8);   //CA- BOD动作 9~16
		Run_Msg_Buffer[40] = (uint8_t)(Sys_Par.ACBODState>>16);  //CA- BOD动作 17~24
	  Run_Msg_Buffer[41] = (uint8_t)(Sys_Par.ACBODState>>24);  //CA- BOD动作 25~32	
		Run_Msg_Buffer[42] = Sys_Par.ThyristorWorkPattern;
		Run_Msg_Buffer[43] = Sys_Par.ThyristorVoltageStage;	
	  Run_Msg_Buffer[44] = Sys_Par.ThyristorNumPreArm1;
		Run_Msg_Buffer[45] = Sys_Par.ThyristorNumPreArm2;
		Run_Msg_Buffer[46] = Sys_Par.ThyristorNumPreArm3;
		Run_Msg_Buffer[47] = Sys_Par.ThyristorNumPreArm4;
		Run_Msg_Buffer[48] = Sys_Par.ThyristorTripNumPreArm;
		Run_Msg_Buffer[49] = Sys_Par.ThyristorCheckCycles;
		Run_Msg_Buffer[50] = Sys_Par.BodTripTripNumPreArm;
		Run_Msg_Buffer[51] = Sys_Par.ThyristorCheckCycles1;
		Run_Msg_Buffer[52] = Sys_Par.GYH_CheckNum;
		Run_Msg_Buffer[53] = Sys_Par.GYF_CheckNum;
		Run_Msg_Buffer[54] = Sys_Par.CTL_Dead_CheckNum;
		Run_Msg_Buffer[55] = Sys_Par.FaultReset;
		Run_Msg_Buffer[56] = Sys_Par.JC_ProtectChoice;
		Run_Msg_Buffer[57] = Sys_Par.BOD_ProtectChoice;
		Run_Msg_Buffer[58] = 0;
		Run_Msg_Buffer[59] = 0;
		Run_Msg_Buffer[60] = 0;
		Run_Msg_Buffer[61] = 0;
		Run_Msg_Buffer[62] = 0;
		Run_Msg_Buffer[63] = 0;
		
		gyhchecknum = Sys_Par.GYH_CheckNum * 4;
		gyfchecknum = Sys_Par.GYF_CheckNum * 4;
}
void Load_Message(uint8_t *data,uint8_t addr)
{
//		int i;

	switch(addr)
		{
			case 0x41:
				Sys_Par.ThyristorWorkPattern = data[7];
				Sys_Par.ThyristorVoltageStage = data[8];
				Sys_Par.ThyristorNumPreArm1 = data[9];
				Sys_Par.ThyristorNumPreArm2 = data[10];
				Sys_Par.ThyristorNumPreArm3 = data[11];
				Sys_Par.ThyristorNumPreArm4 = data[12];	
				Sys_Par.ThyristorTripNumPreArm = data[13];
				Sys_Par.ThyristorCheckCycles = data[14];
				Sys_Par.BodTripTripNumPreArm = data[15];
				Sys_Par.ThyristorCheckCycles1 = data[16];
				Sys_Par.GYH_CheckNum = data[17];
				Sys_Par.GYF_CheckNum = data[18];
				Sys_Par.CTL_Dead_CheckNum = data[19];
				Sys_Par.FaultReset = data[20];
				Sys_Par.JC_ProtectChoice = data[21];
				Sys_Par.BOD_ProtectChoice = data[22];	
			
/*			  for(i=7;i<22;i++)
			  {
					FLASH_Program_Byte(Addr+i-7,data[i]);
			  }*/
			  STM32F407FLASH_Write(Addr_start,Addr_end,data);
						
        //--------参数存储---------//	
			  //从Sys_Par.ThyristorWorkPattern到Sys_Par.BOD_ProtectChoice
			
			break;
			case 0x47:
				Sys_Par.FaultReset = data[7];//复位
			break;
			default:
       
      break;
		}
}

uint16_t ThyristorBreakDownCheck(uint32_t  ScrTemp, 					//???????
							 uint16_t  ThyristorNumPreArm,			//???????
							 uint32_t* SCRAntiJammingCheckNum,		//????????????
							 uint16_t  NumOfThyristorCheckAntiJamming,//????????????
							 uint16_t  ThyristorCheckCycles,			//?????							
							 uint16_t  ThyristorTripNumPreArm,		//?????
							 uint32_t* ThyristorBreakDwonState,		//????,??
							 uint32_t* ThyristorBreakDwonNum,			//????
							 uint16_t ThyristorCheckCycles1			//???????
			          		  )
{
	uint32_t JCTemp=0,i,j,ScrJCTemp=0,NumOfCheckThyristorBreakDown = 0;
	uint16_t count =0 ;
	
	JCTemp = 1;
	for(i=0;i<ThyristorNumPreArm;i++)
	{
		//??????
		if((ScrTemp&JCTemp) == JCTemp)
		{
			SCRAntiJammingCheckNum[NumOfThyristorCheckAntiJamming] |= JCTemp;//?????????????
		}
		else
		{				
			SCRAntiJammingCheckNum[NumOfThyristorCheckAntiJamming] &= (~JCTemp);
		}

		JCTemp <<= 1;
	}
	(*ThyristorBreakDwonState) = ScrTemp;//???????
	if(Simple.ThyristorCheckCyclesStart == 0)//??????
	{			
		for(i=1;( (i<ThyristorCheckCycles)&&(i<MAX_CHECK_NUM_PRE_ARM_FOR_ANTI_JAMMING) );i++)
		{
			SCRAntiJammingCheckNum[i] = SCRAntiJammingCheckNum[0];			
		}
	}

	//????????
	JCTemp=1;	
	for(i=0;( i<ThyristorNumPreArm );i++)
	{					
		
		//ScrJcTemp &= SCRAntiJammingCheckNum[i]; 
		count = 0;
		
		for(j=0;( (j<ThyristorCheckCycles)&&(j<MAX_CHECK_NUM_PRE_ARM_FOR_ANTI_JAMMING) );j++)
		{					
			ScrJCTemp = SCRAntiJammingCheckNum[j];
			if((ScrJCTemp & JCTemp) == JCTemp)
			{			
				count++;			
			}
		}
		JCTemp <<= 1;
		if(count >= ThyristorCheckCycles1)
		{			
			NumOfCheckThyristorBreakDown++;			
		}
	}
/*	if( ThyristorTripNumPreArm < ThyristorNumPreArm)
	{
		(*ThyristorBreakDwonNum) = NumOfCheckThyristorBreakDown;		
	}
	else
	{
	  (*ThyristorBreakDwonNum) = 0;
	}*/
	(*ThyristorBreakDwonNum) = NumOfCheckThyristorBreakDown;//????????? 
	//????????
	if(NumOfCheckThyristorBreakDown >= ThyristorTripNumPreArm)
	{
		return 0;		
	}
	return 1;

}
//????????
//????????
void ThyristorBreakDownProtectLightElectricity(SYS_PAR *V,COUNT *C, CHECK_NUM *N, PROTECT *P)
{
	
//	uint32_t JCABTemp1=0,JCABTemp2=0,JCBCTemp1=0,JCBCTemp2=0,JCCATemp1=0,JCCATemp2=0;
//	uint32_t JCABTemp3=0,JCABTemp4=0,JCBCTemp3=0,JCBCTemp4=0,JCCATemp3=0,JCCATemp4=0;
	uint32_t JCABAction=0,JCBCAction=0,JCCAAction=0;
	uint32_t JCABActiveNum=0,JCBCActiveNum=0,JCCAActiveNum=0; 
	uint16_t Num1=0,Num2=0,Num3=0,Num4=0,FullNum=0;
	Num1 = V->ThyristorNumPreArm1;
	Num2 = V->ThyristorNumPreArm2;
	Num3 = V->ThyristorNumPreArm3;
	Num4 = V->ThyristorNumPreArm4;

	FullNum = Num1 + Num2+Num3 + Num4;
	
	if(syn_jc == 0xAA)////20ms周期
	{
		syn_jc = 0;
		
		Simple.ABTemp1 = (buffer_fpga_temp[0]&0x007F)>>1;
		Simple.ABTemp2 = (buffer_fpga_temp[0]&0x1F80)>>8;
    Simple.ABTemp3 = (((buffer_fpga_temp[0]&0xE000)>>13) + ((buffer_fpga_temp[1]&0x000F)<<3))>>1;
		Simple.ABTemp4 = (buffer_fpga_temp[1]&0x3F0)>>5;

		Simple.BCTemp1 = (buffer_fpga_temp[4]&0x007F)>>1;
		Simple.BCTemp2 = (buffer_fpga_temp[4]&0x1F80)>>8;
    Simple.BCTemp3 = (((buffer_fpga_temp[4]&0xE000)>>13) + ((buffer_fpga_temp[5]&0x000F)<<3))>>1;
		Simple.BCTemp4 = (buffer_fpga_temp[5]&0x3F0)>>5;
		
		Simple.CATemp1 = (buffer_fpga_temp[8]&0x007F)>>1;
		Simple.CATemp2 = (buffer_fpga_temp[8]&0x1F80)>>8;
    Simple.CATemp3 = (((buffer_fpga_temp[8]&0xE000)>>13) + ((buffer_fpga_temp[9]&0x000F)<<3))>>1;
		Simple.CATemp4 = (buffer_fpga_temp[9]&0x3F0)>>5;

		Simple.BATemp1 = (buffer_fpga_temp[2]&0x003F);
		Simple.BATemp2 = (buffer_fpga_temp[2]&0x0F80)>>7;
		Simple.BATemp3 = ((buffer_fpga_temp[2]&0xE000)>>13) + ((buffer_fpga_temp[3]&0x0007)<<3);
		Simple.BATemp4 = (buffer_fpga_temp[3]&0x1F0)>>4;

		Simple.CBTemp1 = (buffer_fpga_temp[6]&0x003F);
		Simple.CBTemp2 = (buffer_fpga_temp[6]&0x0F80)>>7;
		Simple.CBTemp3 = ((buffer_fpga_temp[6]&0xE000)>>13) + ((buffer_fpga_temp[7]&0x0007)<<3);
		Simple.CBTemp4 = (buffer_fpga_temp[7]&0x1F0)>>4;

		Simple.ACTemp1 = (buffer_fpga_temp[10]&0x003F);
		Simple.ACTemp2 = (buffer_fpga_temp[10]&0x0F80)>>7;
		Simple.ACTemp3 = ((buffer_fpga_temp[10]&0xE000)>>13) + ((buffer_fpga_temp[11]&0x0007)<<3);
		Simple.ACTemp4 = (buffer_fpga_temp[11]&0x1F0)>>4;

		Simple.JCABTemp1 = Simple.ABTemp1  & Simple.BATemp1;
		Simple.JCABTemp2 = Simple.ABTemp2  & Simple.BATemp2;
		Simple.JCABTemp3 = Simple.ABTemp3  & Simple.BATemp3;
		Simple.JCABTemp4 = Simple.ABTemp4  & Simple.BATemp4;

		Simple.JCBCTemp1 = Simple.BCTemp1  & Simple.CBTemp1;
		Simple.JCBCTemp2 = Simple.BCTemp2  & Simple.CBTemp2;
		Simple.JCBCTemp3 = Simple.BCTemp3  & Simple.CBTemp3;
		Simple.JCBCTemp4 = Simple.BCTemp4  & Simple.CBTemp4;

		Simple.JCCATemp1 = Simple.CATemp1  & Simple.ACTemp1;
		Simple.JCCATemp2 = Simple.CATemp2  & Simple.ACTemp2;
		Simple.JCCATemp3 = Simple.CATemp3  & Simple.ACTemp3;
		Simple.JCCATemp4 = Simple.CATemp4  & Simple.ACTemp4;

		Simple.JCABTemp =	(Simple.JCABTemp4 << (Num1+Num2+Num3)) + (Simple.JCABTemp3 << (Num1+Num2)) + (Simple.JCABTemp2 << (Num1)) + Simple.JCABTemp1;
		Simple.JCBCTemp =	(Simple.JCBCTemp4 << (Num1+Num2+Num3)) + (Simple.JCBCTemp3 << (Num1+Num2)) + (Simple.JCBCTemp2 << (Num1)) + Simple.JCBCTemp1;
		Simple.JCCATemp =	(Simple.JCCATemp4 << (Num1+Num2+Num3)) + (Simple.JCCATemp3 << (Num1+Num2)) + (Simple.JCCATemp2 << (Num1)) + Simple.JCCATemp1;

//1.1 AB击穿检测
		ThyristorBreakDownCheck( Simple.JCABTemp,
									  FullNum,
									  N->ABJCAntiJammingCheckNum,
									  N->NumOfThyristorCheckAntiJamming,
									  V->ThyristorCheckCycles,
									  V->ThyristorTripNumPreArm,
									  &JCABAction,
									  &JCABActiveNum,
									  V->ThyristorCheckCycles1
									  );
		if(JCABActiveNum>=FullNum)
			Simple.JCABState = 0;
		else
		  Simple.JCABState = JCABAction;
		Simple.JCABNum = JCABActiveNum;
		if((Simple.JCABNum < FullNum) && (Simple.JCABNum >=  V->ThyristorTripNumPreArm)) //
//		if(Simple.JCABNum >= V->ThyristorTripNumPreArm)
		{						
			if((V->JC_Fault_flag & 0x01) != 0x01)				
			{
				V->JC_Fault_flag	= V->JC_Fault_flag | 0x01;
				if(P->FaultNum == 0)
				{
					P->FaultNum = LED_A;//A
					P->TripFlag = 1;	
					P->ForbidPulseFlag = 1; 
				}
			}
		}		
		if((Simple.JCABNum < V->ThyristorTripNumPreArm) && (Simple.JCABNum > 0))
		{
			P->WarningNum = P->WarningNum & 0x01; 
		}
		else
		{
			P->WarningNum = P->WarningNum | 0xFE; 
		}
		
		//1.2 BC击穿检测
		ThyristorBreakDownCheck( Simple.JCBCTemp,               //
									  FullNum,					          //
									  N->BCJCAntiJammingCheckNum, //
									  N->NumOfThyristorCheckAntiJamming, //
									  V->ThyristorCheckCycles, //			
									  V->ThyristorTripNumPreArm, //
									  &JCBCAction, //
									  &JCBCActiveNum,
									  V->ThyristorCheckCycles1 //
									  );
		if(JCBCActiveNum>=FullNum)
			Simple.JCBCState = 0;
		else
		  Simple.JCBCState = JCBCAction;
		Simple.JCBCNum = JCBCActiveNum;

		if((Simple.JCBCNum < FullNum) && (Simple.JCBCNum >= V->ThyristorTripNumPreArm)) //??????
//		if(Simple.JCBCNum >= V->ThyristorTripNumPreArm)		
		{						
			if((V->JC_Fault_flag & 0x02) != 0x02)				
			{
				V->JC_Fault_flag	= V->JC_Fault_flag | 0x02;
				if(P->FaultNum == 0)
				{
					P->FaultNum = LED_B;//B
					P->TripFlag = 1;	
					P->ForbidPulseFlag = 1; 
				}
			}
		}
		if((Simple.JCBCNum < V->ThyristorTripNumPreArm) && (Simple.JCBCNum > 0))
		{
			P->WarningNum = P->WarningNum & 0x02; 
		}
		else
		{
			P->WarningNum = P->WarningNum | 0xFD; 
		}
		
		//1.3 CA击穿检测
		ThyristorBreakDownCheck( Simple.JCCATemp,               //???????
									  FullNum,					//???????
									  N->CAJCAntiJammingCheckNum,//????????????
									  N->NumOfThyristorCheckAntiJamming,		//????????????
									  V->ThyristorCheckCycles,	//????				
									  V->ThyristorTripNumPreArm,//?????
									  &JCCAAction,				//????,??
									  &JCCAActiveNum,
									  V->ThyristorCheckCycles1	//???????
									  );
		if(JCCAActiveNum>=FullNum)
			Simple.JCCAState = 0;
		else
		  Simple.JCCAState = JCCAAction;
		Simple.JCCANum = JCCAActiveNum;

		if((Simple.JCCANum < FullNum) && (Simple.JCCANum >=  V->ThyristorTripNumPreArm)) //??????
//		if(Simple.JCCANum >= V->ThyristorTripNumPreArm)		
		{						
			if((V->JC_Fault_flag & 0x04) != 0x04)				
			{
				V->JC_Fault_flag	= V->JC_Fault_flag | 0x04;
				if(P->FaultNum == 0)
				{
					P->FaultNum = LED_C;//C
					P->TripFlag = 1;	
					P->ForbidPulseFlag = 1; 
				}
			}
		}
		if((Simple.JCCANum < V->ThyristorTripNumPreArm) && (Simple.JCCANum > 0))
		{
			P->WarningNum = P->WarningNum & 0x04; 
		}
		else
		{
			P->WarningNum = P->WarningNum | 0xFB; 
		}
		
		N->NumOfThyristorCheckAntiJamming++;
		if((N->NumOfThyristorCheckAntiJamming >=MAX_CHECK_NUM_PRE_ARM_FOR_ANTI_JAMMING) || 
		   (N->NumOfThyristorCheckAntiJamming > V->ThyristorCheckCycles) )
		{
			N->NumOfThyristorCheckAntiJamming = 0;
		}
		if(Simple.ThyristorCheckCyclesStart == 0)//初次上电检测
		{
			Simple.ThyristorCheckCyclesStart = 1;
		}
	}
}
void BodActionCheck(uint32_t  BodInputTemp,//BOD??
					  uint16_t  ThyristorNumPreArm,//?????
					  uint32_t* BodCheckNum,//BOD?????
					  uint32_t  BODTiming1,//BOD?????
					  uint32_t  BodTripCycles,//BOD?????
					  uint16_t  ThyristorTripNumPreArm,//?????
					  uint32_t* BodActionRtu,
					  uint32_t* BodActiveNum,
					  uint32_t* BodTiming3,
					  uint32_t* BodTiming4,
					  uint32_t BODTiming2)
{
	uint16_t BodTemp=0,i=0;
	//??BOD??
	if(BodInputTemp > 0)//????1
	{	
		BODTiming1++;
		if(BODTiming1 >=2)
		{
			BODTiming2 = 0;
		}			
	}
	else				
	{	
		BODTiming2++;
		if(BODTiming2 >=2)
		{
			BODTiming1 = 0;
		}
	}
	*BodTiming3 = BODTiming1;
	*BodTiming4 = BODTiming2;	
	BodTemp = 1;
	for(i=0;i<ThyristorNumPreArm;i++)
	{
		
		if((BodInputTemp&BodTemp)==BodTemp)
		{
			(*BodActionRtu) |= BodTemp;//BOD??
		}
		else
		{
			(*BodActionRtu) &= (~BodTemp);
		}
		BodTemp <<= 1;		
	}
}

void BodActionProtect(SYS_PAR *V, COUNT *C, CHECK_NUM *N, PROTECT *P)
{
	uint16_t BodABTemp1=0,BodBATemp1=0,BodBCTemp1=0,BodCBTemp1=0,BodCATemp1=0,BodACTemp1=0;//???1??????????
	uint16_t BodABTemp2=0,BodBATemp2=0,BodBCTemp2=0,BodCBTemp2=0,BodCATemp2=0,BodACTemp2=0;//???2??????????
	uint16_t BodABTemp3=0,BodBATemp3=0,BodBCTemp3=0,BodCBTemp3=0,BodCATemp3=0,BodACTemp3=0;//???1??????????
	uint16_t BodABTemp4=0,BodBATemp4=0,BodBCTemp4=0,BodCBTemp4=0,BodCATemp4=0,BodACTemp4=0;//???2??????????
	
	uint32_t BodABActiveNum=0,BodBAActiveNum=0,BodBCActiveNum=0,BodCBActiveNum=0,BodCAActiveNum=0,BodACActiveNum=0; 
	uint32_t BodABAction=0,BodBAAction=0,BodBCAction=0,BodCBAction=0,BodCAAction=0,BodACAction=0; 
	uint32_t BodABTemp=0,BodBCTemp=0,BodCATemp=0,BodBATemp=0,BodCBTemp=0,BodACTemp=0;//????
	uint32_t BodABTiming1 = 0,BodBATiming1 = 0,BodBCTiming1 = 0,BodCBTiming1 = 0,BodCATiming1 = 0,BodACTiming1 = 0;
	uint32_t BodABTiming2 = 0,BodBATiming2 = 0,BodBCTiming2 = 0,BodCBTiming2 = 0,BodCATiming2 = 0,BodACTiming2 = 0;
	uint16_t Num1=0,Num2=0,Num3=0,Num4=0,FullNum=0;
	Num1 = V->ThyristorNumPreArm1;//???1?????? 
	Num2 = V->ThyristorNumPreArm2;//???2??????
	Num3 = V->ThyristorNumPreArm3;//???1?????? 
	Num4 = V->ThyristorNumPreArm4;//???2??????
	FullNum = Num1 + Num2+Num3 + Num4;//??????
	
	if(syn_bod == 0xAA)//20ms周期
	{
		syn_bod = 0;
	
		Simple.BodABTemp1 = (buffer_fpga_temp[12]&0x007F)>>1;
		Simple.BodABTemp2 = (buffer_fpga_temp[12]&0x1F80)>>8;
    Simple.BodABTemp3 = (((buffer_fpga_temp[12]&0xE000)>>13) + ((buffer_fpga_temp[13]&0x000F)<<3))>>1;
		Simple.BodABTemp4 = (buffer_fpga_temp[13]&0x3F0)>>5;

		Simple.BodBATemp1 = (buffer_fpga_temp[14]&0x007F);
		Simple.BodBATemp2 = (buffer_fpga_temp[14]&0x1F80)>>7;
		Simple.BodBATemp3 = ((buffer_fpga_temp[14]&0xE000)>>13) + ((buffer_fpga_temp[15]&0x000F)<<3);
		Simple.BodBATemp4 = (buffer_fpga_temp[15]&0x3F0)>>5;		
		
		Simple.BodBCTemp1 = (buffer_fpga_temp[16]&0x007F)>>1;
		Simple.BodBCTemp2 = (buffer_fpga_temp[16]&0x1F80)>>8;
    Simple.BodBCTemp3 = (((buffer_fpga_temp[16]&0xE000)>>13) + ((buffer_fpga_temp[17]&0x000F)<<3))>>1;
		Simple.BodBCTemp4 = (buffer_fpga_temp[17]&0x3F0)>>5;

		Simple.BodCBTemp1 = (buffer_fpga_temp[18]&0x007F);
		Simple.BodCBTemp2 = (buffer_fpga_temp[18]&0x1F80)>>7;
		Simple.BodCBTemp3 = ((buffer_fpga_temp[18]&0xE000)>>13) + ((buffer_fpga_temp[19]&0x000F)<<3);
		Simple.BodCBTemp4 = (buffer_fpga_temp[19]&0x3F0)>>4;	

		Simple.BodCATemp1 = (buffer_fpga_temp[20]&0x007F)>>1;
		Simple.BodCATemp2 = (buffer_fpga_temp[20]&0x1F80)>>8;
    Simple.BodCATemp3 = (((buffer_fpga_temp[20]&0xE000)>>13) + ((buffer_fpga_temp[21]&0x000F)<<3))>>1;
		Simple.BodCATemp4 = (buffer_fpga_temp[21]&0x3F0)>>5;

		Simple.BodACTemp1 = (buffer_fpga_temp[22]&0x007F);
		Simple.BodACTemp2 = (buffer_fpga_temp[22]&0x1F80)>>7;
		Simple.BodACTemp3 = ((buffer_fpga_temp[22]&0xE000)>>13) + ((buffer_fpga_temp[23]&0x000F)<<3);
		Simple.BodACTemp4 = (buffer_fpga_temp[23]&0x3F0)>>4;	

		BodABTemp =	(BodABTemp4 << (Num1+Num2+Num3)) + (BodABTemp3 << (Num1+Num2)) + (BodABTemp2 << (Num1)) + BodABTemp1;//AB????????
		BodBATemp =	(BodBATemp4 << (Num1+Num2+Num3)) + (BodBATemp3 << (Num1+Num2)) + (BodBATemp2 << (Num1)) + BodBATemp1;//AB????????
    BodBCTemp =	(BodBCTemp4 << (Num1+Num2+Num3)) + (BodBCTemp3 << (Num1+Num2)) + (BodBCTemp2 << (Num1)) + BodBCTemp1;//AB????????
		BodCBTemp =	(BodCBTemp4 << (Num1+Num2+Num3)) + (BodCBTemp3 << (Num1+Num2)) + (BodCBTemp2 << (Num1)) + BodCBTemp1;//AB????????
		BodCATemp =	(BodCATemp4 << (Num1+Num2+Num3)) + (BodCATemp3 << (Num1+Num2)) + (BodCATemp2 << (Num1)) + BodCATemp1;//AB????????
		BodACTemp =	(BodACTemp4 << (Num1+Num2+Num3)) + (BodACTemp3 << (Num1+Num2)) + (BodACTemp2 << (Num1)) + BodACTemp1;//AB????????

		//1.??AB+????BOD??
		BodActionCheck( BodABTemp,               //???????
							  FullNum,					//???????
							  N->ABBodAntiJammingCheckNum,//????????????
							  Simple.BODTiming1,		//BOD???????
							  V->BodTripTripNumPreArm,	//BOD???????				
							  1,//?????
							  &BodABAction,				//????,??
							  &BodABActiveNum,
							  &BodABTiming1,
							  &BodABTiming2,
							  Simple.BODTiming2);
								
		if(BodABActiveNum>=FullNum)
			Simple.BODABState = 0;
		else
		  Simple.BODABState = BodABAction;								
															
//		Simple.BODABState = BodABAction;
		Simple.BODABNum = BodABActiveNum;
		Simple.BODTiming1 = BodABTiming1;
		Simple.BODTiming2 = BodABTiming2;
		if(Simple.BODTiming1 >= Simple.BODTripTripNumPreArm)//????
		{
			if((V->BOD_Fault_flag && 0x01) != 0x01)
			{
				V->BOD_Fault_flag	= V->BOD_Fault_flag | 0x01;

				if(P->FaultNum == 0)
				{
					P->FaultNum = LED_1;//1
					P->TripFlag = 1;	
					P->ForbidPulseFlag = 1; 
				}
			}
		}
		//2.??AB-????BOD??
		BodActionCheck( BodBATemp,               //???????
							  FullNum,					//???????
							  N->BABodAntiJammingCheckNum,//????????????
							  Simple.BODTiming3,		//BOD???????
							  V->BodTripTripNumPreArm,	//BOD???????				
							  1,//?????
							  &BodBAAction,				//????,??
							  &BodBAActiveNum,
							  &BodBATiming1,
						      &BodBATiming2,
							  Simple.BODTiming4);
								
		if(BodBAActiveNum>=FullNum)
			Simple.BODBAState = 0;
		else
		  Simple.BODBAState = BodBAAction;
		
		//		Simple.BODBAState = BodBAAction;
		Simple.BODBANum = BodBAActiveNum;
		Simple.BODTiming3 = BodBATiming1;
		Simple.BODTiming4 = BodBATiming2;
		if(Simple.BODTiming3 >= Simple.BODTripTripNumPreArm)//????
		{
			if((V->BOD_Fault_flag & 0x02) != 0x02)
			{
				V->BOD_Fault_flag	= V->BOD_Fault_flag | 0x02;

				if(P->FaultNum == 0)
				{
					P->FaultNum = LED_2;//W
					P->TripFlag = 1;	
					P->ForbidPulseFlag = 1; 
				}
			}
		}
		//3.??BC+????BOD??
		BodActionCheck( BodBCTemp,               //???????
							  FullNum,					//???????
							  N->BCBodAntiJammingCheckNum,//????????????
							  Simple.BODTiming5,		//BOD???????
							  V->BodTripTripNumPreArm,	//BOD???????				
							  1,//?????
							  &BodBCAction,				//????,??
							  &BodBCActiveNum,
							  &BodBCTiming1,
						      &BodBCTiming2,
							  Simple.BODTiming6);
								
		if(BodBCActiveNum>=FullNum)
			Simple.BODBCState = 0;
		else
		  Simple.BODBCState = BodBCAction;		
		
//		Simple.BODBCState = BodBCAction;
		Simple.BODBCNum = BodBCActiveNum;
		Simple.BODTiming5 = BodBCTiming1;
		Simple.BODTiming6 = BodBCTiming2;
		if(Simple.BODTiming5 >= Simple.BODTripTripNumPreArm)//????
		{
			if((V->BOD_Fault_flag & 0x04) != 0x04)
			{
				V->BOD_Fault_flag	= V->BOD_Fault_flag | 0x04;

				if(P->FaultNum == 0)
				{
					P->FaultNum = LED_3;//W
					P->TripFlag = 1;	
					P->ForbidPulseFlag = 1; 
				}
			}
		}
		//4.??BC-????BOD??
		BodActionCheck( BodCBTemp,               //???????
							  FullNum,					//???????
							  N->CBBodAntiJammingCheckNum,//????????????
							  Simple.BODTiming7,		//BOD???????
							  V->BodTripTripNumPreArm,	//BOD???????					
							  1,//?????
							  &BodCBAction,				//????,??
							  &BodCBActiveNum,
							  &BodCBTiming1,
						  	  &BodCBTiming2,
							  Simple.BODTiming8);	
										
		if(BodCBActiveNum>=FullNum)
			Simple.BODCBState = 0;
		else
		  Simple.BODCBState = BodCBAction;
		
//		Simple.BODCBState = BodCBAction;
		Simple.BODCBNum = BodCBActiveNum;
		Simple.BODTiming7 = BodCBTiming1;
		Simple.BODTiming8 = BodCBTiming2;
		if(Simple.BODTiming7 >= Simple.BODTripTripNumPreArm)//????
		{
			if((V->BOD_Fault_flag & 0x08) != 0x08)
			{
				V->BOD_Fault_flag	= V->BOD_Fault_flag | 0x08;

				if(P->FaultNum == 0)
				{
					P->FaultNum = LED_4;//W
					P->TripFlag = 1;	
					P->ForbidPulseFlag = 1; 
				}
			}
		}
		//5.??CA+????BOD??
		BodActionCheck( BodCATemp,               //???????
							  FullNum,					//???????
							  N->CABodAntiJammingCheckNum,//????????????
							  Simple.BODTiming9,		//BOD???????
							  V->BodTripTripNumPreArm,	//BOD???????					
							  1,//?????
							  &BodCAAction,				//????,??
							  &BodCAActiveNum,
							  &BodCATiming1,
						 	  &BodCATiming2,
							  Simple.BODTiming10);
		
		if(BodCAActiveNum>=FullNum)
			Simple.BODCAState = 0;
		else
		  Simple.BODCBState = BodCBAction;		
		
//		Simple.BODCAState = BodCAAction;
		Simple.BODCANum = BodCAActiveNum;
		Simple.BODTiming9 = BodCATiming1;
		Simple.BODTiming10 = BodCATiming2;
		if(Simple.BODTiming9 >= Simple.BODTripTripNumPreArm)//????
		{
			if((V->BOD_Fault_flag & 0x10) != 0x10)
			{
				V->BOD_Fault_flag	= V->BOD_Fault_flag | 0x10;

				if(P->FaultNum == 0)
				{
					P->FaultNum = LED_5;//W
					P->TripFlag = 1;	
					P->ForbidPulseFlag = 1; 
				}
			}
		}
		//6.??CA-????BOD??
		BodActionCheck( BodACTemp,               //???????
						  FullNum,					//???????
						  N->ACBodAntiJammingCheckNum,//????????????
						  Simple.BODTiming11,		//BOD???????
						  V->BodTripTripNumPreArm,	//BOD???????					
						  1,//?????
						  &BodACAction,				//????,??
						  &BodACActiveNum,
						  &BodACTiming1,
						  &BodACTiming2,
						  Simple.BODTiming12);
		
		if(BodACActiveNum>=FullNum)
			Simple.BODACState = 0;
		else
		  Simple.BODACState = BodACAction;		
//		Simple.BODACState = BodACAction;
		Simple.BODACNum = BodACActiveNum;
		Simple.BODTiming11 = BodACTiming1;
		Simple.BODTiming12 = BodACTiming2;
		if(Simple.BODTiming11 >= Simple.BODTripTripNumPreArm)//????
		{
			if((V->BOD_Fault_flag & 0x20) != 0x20)
			{
				V->BOD_Fault_flag	= V->BOD_Fault_flag | 0x20;

				if(P->FaultNum == 0)
				{
					P->FaultNum = LED_6;//W
					P->TripFlag = 1;	
					P->ForbidPulseFlag = 1; 
				}
			}
		}
	}
}
void SetLED(void)
{
	  int32_t temp1,temp2;
	  //LED_GREEN_ON;
	  
	  //temp1=GpioDataRegs.GPADAT.all & 0xF80FFFFF;
    //temp2=GpioDataRegs.GPBDAT.all & 0xFFE0FFFF;
	
    LED_num=1&0x0007;
			 		
	  //GpioDataRegs.GPADAT.all =  temp1 | LED_scan[LED_num];
    //GpioDataRegs.GPBDAT.all =  temp2 | LED_content[LED_character][LED_num];
	
		temp2 = LED_scan[LED_num];
		temp1 = (temp2 >> 20)%1;
		if(temp1 == 1)LED1_ON;else LED1_OFF;
		temp1 = (temp2 >> 21)%1;
		if(temp1 == 1)LED2_ON;else LED2_OFF;
	  temp1 = (temp2 >> 22)%1;
		if(temp1 == 1)LED3_ON;else LED3_OFF;
	  temp1 = (temp2 >> 23)%1;
		if(temp1 == 1)LED4_ON;else LED4_OFF;
	  temp1 = (temp2 >> 24)%1;
		if(temp1 == 1)LED5_ON;else LED5_OFF;
		temp1 = (temp2 >> 25)%1;
		if(temp1 == 1)LED6_ON;else LED6_OFF;
	  temp1 = (temp2 >> 26)%1;
		if(temp1 == 1)LED7_ON;else LED7_OFF;	
		
		temp2 = LED_content[LED_character][LED_num];
		temp1 = (temp2 >> 16)%1;
		if(temp1 == 1)LED11_ON;else LED11_OFF;
		temp1 = (temp2 >> 17)%1;
		if(temp1 == 1)LED12_ON;else LED12_OFF;
		temp1 = (temp2 >> 18)%1;
		if(temp1 == 1)LED13_ON;else LED13_OFF;
		temp1 = (temp2 >> 19)%1;
		if(temp1 == 1)LED14_ON;else LED14_OFF;
		temp1 = (temp2 >> 20)%1;
		if(temp1 == 1)LED15_ON;else LED15_OFF;
	
}

void DealFault(PROTECT *P)
{
	
}

void Sys_Par_Int(void)
{
//	    uint8_t rom_data[30];
	     int i;
	    STM32F407FLASH_Read(Addr_start,Addr_end,rom_data);
	    LED_BLUE_ON;
      HAL_Delay(1000);
/**/	
/**/	    Sys_Par.ThyristorWorkPattern = rom_data[7];
			Sys_Par.ThyristorVoltageStage = rom_data[8];
			Sys_Par.ThyristorNumPreArm1 = rom_data[9];
			Sys_Par.ThyristorNumPreArm2 = rom_data[10];
			Sys_Par.ThyristorNumPreArm3 = rom_data[11];
			Sys_Par.ThyristorNumPreArm4 = rom_data[12];	
			Sys_Par.ThyristorTripNumPreArm = rom_data[13];
			Sys_Par.ThyristorCheckCycles = rom_data[14];
			Sys_Par.BodTripTripNumPreArm = rom_data[15];
			Sys_Par.ThyristorCheckCycles1 = rom_data[16];
			Sys_Par.GYH_CheckNum = rom_data[17];
			Sys_Par.GYF_CheckNum = rom_data[18];
			Sys_Par.CTL_Dead_CheckNum = rom_data[19];
			Sys_Par.FaultReset = rom_data[20];
			Sys_Par.JC_ProtectChoice = rom_data[21];
			Sys_Par.BOD_ProtectChoice = rom_data[22];
		  LED_BLUE_OFF;
      HAL_Delay(1000);
      Sys_Par.JC_Fault_flag = 0;
			Sys_Par.BOD_Fault_flag = 0;
			Sys_Par.ALARM = 0;
			Sys_Par.FAULT = 0;
			Protect_Ctrl.FaultNum = 0;
			Protect_Ctrl.TripFlag = 0;	
			Protect_Ctrl.ForbidPulseFlag = 0; 
			TRIP_ON;
			FAULT_ON;
			DEAD_ON;
			
		
			Sys_Par.FaultReset = 0;
			Sys_Par.JC_Fault_flag = 0;
			Sys_Par.BOD_Fault_flag = 0;
			Sys_Par.ALARM = 0;
			Sys_Par.FAULT = 0;
			Simple.JCABState = 0;
			Simple.JCBCState = 0;
			Simple.JCCAState = 0;
			Simple.BODABState = 0;
			Simple.BODBAState = 0;
			Simple.BODBCState = 0;
			Simple.BODCBState = 0;
			Simple.BODCAState = 0;
			Simple.BODACState = 0;
			Simple.JCABNum = 0;	
			Simple.JCBCNum = 0;	
			Simple.JCCANum = 0;	
			Simple.ABTemp1 = 0;	
			Simple.ABTemp2 = 0;	
			Simple.ABTemp3 = 0;	
			Simple.ABTemp4 = 0;
			Simple.BCTemp1 = 0;	
			Simple.BCTemp2 = 0;	
			Simple.BCTemp3 = 0;	
			Simple.BCTemp4 = 0;	
			Simple.CATemp1 = 0;	
			Simple.CATemp2 = 0;	
			Simple.CATemp3 = 0;	
			Simple.CATemp4 = 0;
			Simple.BATemp1 = 0;	
			Simple.BATemp2 = 0;	
			Simple.BATemp3 = 0;	
			Simple.BATemp4 = 0;
			Simple.CBTemp1 = 0;	
			Simple.CBTemp2 = 0;	
			Simple.CBTemp3 = 0;	
			Simple.CBTemp4 = 0;	
			Simple.ACTemp1 = 0;	
			Simple.ACTemp2 = 0;	
			Simple.ACTemp3 = 0;	
			Simple.ACTemp4 = 0;
			for(i=0;i<MAX_CHECK_NUM_PRE_ARM_FOR_ANTI_JAMMING;i++)
			{
				CheckNum.ABJCAntiJammingCheckNum[i] = 0;
				CheckNum.BAJCAntiJammingCheckNum[i] = 0;
				CheckNum.BCJCAntiJammingCheckNum[i] = 0;
				CheckNum.CBJCAntiJammingCheckNum[i] = 0;
				CheckNum.CAJCAntiJammingCheckNum[i] = 0;
				CheckNum.ACJCAntiJammingCheckNum[i] = 0;
				CheckNum.ABBodAntiJammingCheckNum[i] = 0;
				CheckNum.BABodAntiJammingCheckNum[i] = 0;
				CheckNum.BCBodAntiJammingCheckNum[i] = 0;
				CheckNum.CBBodAntiJammingCheckNum[i] = 0;
				CheckNum.CABodAntiJammingCheckNum[i] = 0;
				CheckNum.ACBodAntiJammingCheckNum[i] = 0;			
			}
}

//读flash,只能写到0x80e0000-0x8100000,在这中间写数
/*uint8_t STM32F407FLASH_Read(uint32_t startaddr,uint32_t endaddr,uint32_t *data){
	uint32_t i;
	uint8_t j = 0;
	
	for(i = startaddr;i<endaddr;i += 4){
		data[j++] = *(uint32_t *)(i);
	}
	
	return HAL_OK;
}*/
uint8_t STM32F407FLASH_Read(uint32_t startaddr,uint32_t endaddr,uint8_t *data){
	
	uint32_t i;
	uint8_t j = 0;
	
	for(i = startaddr;i<endaddr;i ++){
		data[j++] = *(uint32_t *)(i);
	}
	
	return HAL_OK;
}
//写flash,只能写到0x80e0000-0x8100000,在这中间写数，读之前整个扇区都会擦除
uint16_t STM32F407FLASH_Write(uint32_t startaddr,uint32_t endaddr,uint8_t *data){
	uint32_t i = 0;
	uint8_t j = 0;
	uint32_t PageError = 0;//错误转跳地址
	uint32_t flasherror = 0;
	uint32_t czerror = 0;
	FLASH_EraseInitTypeDef f;
	
	f.TypeErase = FLASH_TYPEERASE_SECTORS ;//PAGES
	f.Sector = 11;//第11扇区
	f.NbSectors = 1;
	f.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	
	HAL_FLASH_Unlock();
	__HAL_FLASH_DATA_CACHE_DISABLE();
	
	flasherror = HAL_FLASHEx_Erase(&f,&PageError);
	//FLASH_WaitForLastOperation(flash_Waite_time);
	//data = (uint32_t *)(&Saved_Param);
	if(flasherror == HAL_OK){
		for(i = startaddr; i < endaddr ; i ++){
			HAL_FLASH_Program(TYPEPROGRAM_BYTE,i,data[j++]);
		}
	}else{
		czerror = HAL_ERROR;
	}
	
	__HAL_FLASH_DATA_CACHE_ENABLE();
	HAL_FLASH_Lock();
	
	return czerror;
	
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
