/*
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    MKE06Z4_bootloader-hello.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKE06Z4.h"
#include "fsl_debug_console.h"
#include "fsl_mscan.h"
#include "fsl_flash.h"
#define APP_ADDR  0x6000
#define APP_SIZE  0x7000
#define FAC_FLAG 0x0026
#define SW_VER 0x01;
#define HW_VER 0x01;
#define PRO_VER 0x01;
#define BS 30
uint8_t jump_flag=0;
uint8_t fast_jump_data[7]={0x0,0x0,0x0,0x0,0x0,0x0,0x0};
uint8_t update_mode[7]={0x0,0x0,0x1,0x0,0x0,0x0,0x0};
uint8_t normal_mode[7]={0x0,0x0,0x2,0x0,0x0,0x0,0x0};
uint8_t send_data[8]={0};
uint16_t update_len,update_crc;
uint8_t update_package_data[264]={0};
uint16_t package_num;
uint16_t package_current_num=0;
uint8_t is_first_cnt=1;
uint32_t offset_addr=0;
uint8_t num_cnt=0;
void send_first_msg();
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
volatile uint32_t g_systickCounter;
void SysTick_Handler(void)
{
    if (g_systickCounter != 0U)
    {
    	g_systickCounter--;
    }
}

void SysTick_DelayTicks(uint32_t n)
{
    g_systickCounter = n;
    while (g_systickCounter != 0U)
    {
    }
}
uint8_t num_iter()
{
	num_cnt++;
	if(num_cnt==32)
		num_cnt = 1;
	return num_cnt;
}
void JumpToImage(uint32_t addr)
{
    uint32_t *vectorTable = (uint32_t*)addr;
    uint32_t sp = vectorTable[0];
    uint32_t pc = vectorTable[1];

    typedef void(*app_entry_t)(void);

    uint32_t s_stackPointer = 0;
    uint32_t s_applicationEntry = 0;
    app_entry_t s_application = 0;

    s_stackPointer = sp;
    s_applicationEntry = pc;
    s_application = (app_entry_t)s_applicationEntry;

    // Change MSP and PSP
    __set_MSP(s_stackPointer);
    __set_PSP(s_stackPointer);

    SCB->VTOR = addr;

    // Jump to application
    s_application();

    // Should never reach here.
    __NOP();
}
const uint8_t crctablehi[] ={
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40
};
/* Table of CRC values for low–order byte */
const uint8_t crctablelo[]={
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
	0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
	0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
	0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
	0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
	0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
	0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
	0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
	0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
	0x40
};

uint16_t crc16table(uint8_t *ptr, uint16_t len)
{
	uint8_t crchi = 0xff;
	uint8_t crclo = 0xff;
	uint16_t index;
	while (len--)
	{
		index = crclo ^ *ptr++;
		crclo = crchi ^ crctablehi[index];
		crchi = crctablelo[index];
	}
	return (crchi << 8 | crclo);
}
void InvertUint8(unsigned char *DesBuf, unsigned char *SrcBuf)
 {
    int i;
    unsigned char temp = 0;

    for(i = 0; i < 8; i++)
    {
    	if(SrcBuf[0] & (1 << i))
    	{
	       temp |= 1<<(7-i);
    	}
	}
	DesBuf[0] = temp;
}
void InvertUint16(unsigned short *DesBuf, unsigned short *SrcBuf)
{
	int i;
	unsigned short temp = 0;
	for(i = 0; i < 16; i++)
	{
		if(SrcBuf[0] & (1 << i))
		{
			temp |= 1<<(15 - i);
		}
	}
	DesBuf[0] = temp;
}
unsigned short CRC16_MODBUS(unsigned char *puchMsg, unsigned int usDataLen)
{
   unsigned short wCRCin = 0xFFFF;
   unsigned short wCPoly = 0x8005;
   unsigned char wChar = 0;

	while (usDataLen--)
	{
		wChar = *(puchMsg++);
		InvertUint8(&wChar, &wChar);
		wCRCin ^= (wChar << 8);

		for(int i = 0; i < 8; i++)
		{
			if(wCRCin & 0x8000)
			{
				wCRCin = (wCRCin << 1) ^ wCPoly;
			}
			else
			{
				wCRCin = wCRCin << 1;
			}
       }
   }
   InvertUint16(&wCRCin, &wCRCin);
   return (wCRCin) ;
}
#define MSCAN_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)
#define FLASH_CLOCK CLOCK_GetFreq(kCLOCK_FlashClk)
typedef enum{
	ONE_NO_ACK,
	FIRST,
	CONTINUE,
	FLOW,
	ONE_RESPONSE,
	ONE_HAVE_ACK
}ACK_FLAG;
typedef enum{
	ALARM,
	CTRL,
	STATE,
	UPDATE,
	CHECK
}LEVEL;
typedef struct {
	uint8_t HW_ver[4];
	uint32_t ota_flag;
	uint8_t SN_H[8];
	uint8_t SN_L[8];
}MYFLASH;
MYFLASH myflash;
static flash_config_t s_flashDriver;
mscan_frame_t txFrame, rxFrame;
uint8_t recv_ack_flag=0,recv_src=0,recv_dest,recv_num;

uint32_t  Erase_App_Flash()
{
	return FLASH_Erase(&s_flashDriver, (uint32_t)APP_ADDR, APP_SIZE, kFLASH_ApiEraseKey);
}
uint32_t Erase_FlASH()
{
	return FLASH_Erase(&s_flashDriver, (uint32_t)0x1FE00, sizeof(myflash), kFLASH_ApiEraseKey);
}
uint32_t Write_FLASH()
{
	return FLASH_Program(&s_flashDriver, 0x1FE00, (uint32_t *)&myflash, sizeof(myflash));
}
void Write_Package(uint32_t offset,uint16_t len)
{
	FLASH_Program(&s_flashDriver, APP_ADDR+offset, (uint32_t *)(update_package_data+4), len);
}
void Read_FLASH()
{
	memset(&myflash,0,sizeof(myflash));
	memcpy(&myflash,(uint8_t *)0x1FE00,sizeof(myflash));
}
uint8_t flash_init()
{
	memset(&s_flashDriver, 0, sizeof(flash_config_t));
	memset(&myflash,0,sizeof(myflash));

	FLASH_SetProperty(&s_flashDriver, kFLASH_PropertyFlashClockFrequency, FLASH_CLOCK);

	return FLASH_Init(&s_flashDriver);
}
void clear_ota_flag()
{
	//SysTick->CTRL &=0xFFF8;
	myflash.ota_flag = 0;
	Erase_FlASH();
	Write_FLASH();
}
uint32_t comb_ExtID(uint8_t level,uint8_t charge_flag,uint8_t ACK_flag,uint8_t cnt,uint8_t dest,uint8_t src)
{
	uint32_t ID = level<<26|charge_flag<<25|ACK_flag<<17|cnt<<12|dest<<6|src;
	return ID;
}
void analyze_ExtID(uint32_t ID)
{
	recv_ack_flag = ID>>17&0x07;
	recv_num = ID>>12&0x1F;
	recv_dest = ID>>6&0x3F;
	recv_src = ID&0x3F;
}
void can_send_msg(uint32_t ID,uint8_t *data ,uint8_t len)
{
	txFrame.ID_Type.ID = ID;
	txFrame.format     = kMSCAN_FrameFormatExtend;
	txFrame.type       = kMSCAN_FrameTypeData;
	txFrame.DLR        = len;
	memcpy(txFrame.DSR,data,len);
	MSCAN_TransferSendBlocking(MSCAN,  &txFrame);
}
void analyze_data(uint8_t data[8])
{

	if(data[0]==0x87)
	{
		if(memcmp(&data[1],fast_jump_data,7)==0)
		{
			send_data[0] = 0x87;
			memcpy(send_data+1,fast_jump_data,7);
			//SysTick->CTRL &=0xFFF8;
			clear_ota_flag();
			can_send_msg(comb_ExtID(CTRL,1,ONE_RESPONSE,recv_num,0x01,0x09),send_data,8);
			jump_flag = 1;
		}
	}
	else if(data[0]==0x81)
	{
		if(memcmp(&data[1],normal_mode,7)==0)
		{
			memcpy(send_data,data,8);
			clear_ota_flag();
			can_send_msg(comb_ExtID(UPDATE,1,ONE_RESPONSE,recv_num,0x01,0x09),send_data,8);
			send_data[0] = 0x01;
			send_data[3] = 0x02;
			can_send_msg(comb_ExtID(UPDATE,1,ONE_HAVE_ACK,num_iter(),0x01,0x09),send_data,8);

			jump_flag=1;
		}
		else if(memcmp(&data[1],update_mode,7)==0)
		{
			jump_flag=0;
			memcpy(send_data,data,8);
			can_send_msg(comb_ExtID(UPDATE,1,ONE_RESPONSE,recv_num,0x01,0x09),send_data,8);
			Erase_App_Flash();
			send_data[0] = 0x01;
			can_send_msg(comb_ExtID(UPDATE,1,ONE_HAVE_ACK,num_iter(),0x01,0x09),send_data,8);
		}
	}
	else if(data[0] == 0x85&&data[1] == 0x00)
	{
		can_send_msg(comb_ExtID(CTRL,1,ONE_RESPONSE,recv_num,0x01,0x09),rxFrame.DSR,rxFrame.DLR);
		NVIC_SystemReset();
	}
}
void analyze_no_ack_data(uint8_t data[8])
{
	if(data[0]==0x81&&data[1]==0x01)
	{
		memset(send_data,0,8);
		send_data[0]=0x01;
		send_data[1]=0x01;
		send_data[2]=0x00;
		send_data[3]=0x01;
		can_send_msg(comb_ExtID(UPDATE,1,ONE_NO_ACK,num_iter(),0x01,0x09),send_data,8);
	}
	else if(data[0]==0x80&&data[1]==0x03)
	{
		send_first_msg();
	}
}
void analyze_first_update_data(uint8_t data[8])
{
	if(data[0]==0x80&&data[1]==0x00)
	{
		update_len = data[2]<<8|data[3];
		update_crc = data[4]<<8|data[5];
		package_num = (update_len)/8;
		if((update_len)%8!=0)
		{
			package_num++;
		}
		memset(send_data,0,8);
		send_data[0] = BS; //32
		send_data[1] = 0x01;
		send_data[2] = 0x0A;
		send_data[3] = 0x00;
		memset(update_package_data,0,264);
		can_send_msg(comb_ExtID(UPDATE,1,FLOW,0,0x01,0x09),send_data,8);
	}
}
void analyze_continue_data(uint8_t data[8])
{
	memcpy(update_package_data+package_current_num*8,data,8);
	package_current_num++;
	if(recv_num==BS)
	{
		memset(send_data,0,8);
		send_data[0] = BS; //32
		send_data[1] = 0x01;
		send_data[2] = 0x0A;
		send_data[3] = 0x02;
		can_send_msg(comb_ExtID(UPDATE,1,FLOW,0,0x01,0x09),send_data,8);
	}
	else if(package_current_num==package_num)
	{
		memset(send_data,0,8);
		send_data[0] = BS; //32
		send_data[1] = 0x01;
		send_data[2] = 0x0A;
		if(crc16table(update_package_data,update_len)==update_crc)
		{
			send_data[3] =1;
			offset_addr = update_package_data[0]<<24|update_package_data[1]<<16|
							update_package_data[2]<<8|update_package_data[3];
			Write_Package(offset_addr,update_len-4);
		}
		else
			send_data[3] = -3;
		can_send_msg(comb_ExtID(UPDATE,1,FLOW,0,0x01,0x09),send_data,8);
		package_current_num =0;
	}

	/*
	if(recv_num ==1&&is_first_cnt==1)
	{
		offset_addr = data[0]<<24|data[1]<<16|data[2]<<8|data[3];
		memcpy(update_package_data,&data[4],4);
	}
	else if(is_first_cnt==1)
	{
		memcpy(update_package_data+4+(recv_num-2)*8,data,8);
		if(recv_num==0x12)
		{
			is_first_cnt =0;
			memset(send_data,0,8);
			send_data[0] = 0x12; //32
			send_data[1] = 0x01;
			send_data[2] = 0x0A;
			send_data[3] = 0x02;
			can_send_msg(comb_ExtID(UPDATE,1,FLOW,0,0x01,0x09),send_data,8);
		}
	}
	else if(is_first_cnt==0)
	{
		if(recv_num==0x0F)
		{
			memcpy(update_package_data+4+(recv_num+0x10)*8,data,4);
			memset(send_data,0,8);
			send_data[0] = 0x12; //32
			send_data[1] = 0x01;
			send_data[2] = 0x0A;
			if(crc16table(update_package_data,256)==update_crc)
			{
				send_data[3] =1;
				Write_Package(offset_addr);
			}
			else
				send_data[3] = -3;
			can_send_msg(comb_ExtID(UPDATE,1,FLOW,0,0x01,0x09),send_data,8);
		}
		else
		{
			memcpy(update_package_data+4+(recv_num+0x10)*8,data,8);
		}
	}*/
}
void MSCAN_1_IRQHandler(void)
{
	if (MSCAN_GetRxBufferFullFlag(MSCAN))
	{
	    MSCAN_ReadRxMb(MSCAN, &rxFrame);
	    MSCAN_ClearRxBufferFullFlag(MSCAN);
	    analyze_ExtID(rxFrame.ID_Type.ID);
	    if(recv_ack_flag==ONE_HAVE_ACK&&recv_src == 0x01&&recv_dest == 0x09)
	    {
	    	analyze_data(rxFrame.DSR);
	    }
	    else if(recv_ack_flag==ONE_NO_ACK&&recv_src == 0x01&&recv_dest == 0x09)
	    {
	    	analyze_no_ack_data(rxFrame.DSR);
	    }
	    else if(recv_ack_flag==FIRST&&recv_src == 0x01&&recv_dest == 0x09)
	    {
	    	jump_flag=0;
	    	analyze_first_update_data(rxFrame.DSR);
	    }
	    else if(recv_ack_flag==CONTINUE&&recv_src == 0x01&&recv_dest == 0x09)
	    {
	    	analyze_continue_data(rxFrame.DSR);
	    }
	}
	SDK_ISR_EXIT_BARRIER;
}
void MSCAN_init()
{
	mscan_config_t mscanConfig;
	MSCAN_GetDefaultConfig(&mscanConfig);
	mscanConfig.baudRate = 500000U;
	mscanConfig.clkSrc = kMSCAN_ClkSrcBus;
	mscanConfig.filterConfig.u32IDAR0 = 0;
	mscanConfig.filterConfig.u32IDAR1 = 0;
	mscanConfig.filterConfig.u32IDMR0 = 0xffffffff;
	mscanConfig.filterConfig.u32IDMR1 = 0xffffffff;
	MSCAN_Init(MSCAN, &mscanConfig, MSCAN_CLK_FREQ);
	MSCAN_EnableRxInterrupts(MSCAN, kMSCAN_RxFullInterruptEnable);
	EnableIRQ(MSCAN_1_IRQn);
}
void send_first_msg()
{
	send_data[0] = 0x00;
	send_data[1] = 0x00;
	send_data[2] = FAC_FLAG>>8&0xFF;
	send_data[3] = FAC_FLAG&0xFF;
	send_data[4] = SW_VER;
	send_data[5] = HW_VER;
	send_data[6] = PRO_VER;
	send_data[7] = 0x96;
	can_send_msg(comb_ExtID(STATE,1,ONE_NO_ACK,num_iter(),0x3F,0x09),send_data,8);//
}
int main(void) {
	uint8_t jump_cnt=0;
    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    MSCAN_init();
    send_first_msg();
    flash_init();
    Read_FLASH();
    SysTick_Config(SystemCoreClock / 1000U);
    if(myflash.ota_flag ==1)
    {
    	jump_flag=0;
    	Erase_App_Flash();
    }
    while(1) {
    	jump_cnt++;
    	SysTick_DelayTicks(100);
    	if(jump_cnt == 15&&myflash.ota_flag == 0)
    		jump_flag =1;
    	if(jump_flag)
    	    JumpToImage(APP_ADDR);
        __asm volatile ("nop");
    }
    return 0 ;
}
