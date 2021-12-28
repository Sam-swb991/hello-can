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
 * @file    MKE06Z4_hello.c
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
#include "fsl_gpio.h"
#include "fsl_tpm.h"
#include "fsl_adc.h"
#include "fsl_flash.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
#define MSCAN_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)
#define TPM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_TimerClk)
#define TPM_TIMER_SOURCE_CLOCK (CLOCK_GetFreq(kCLOCK_TimerClk) / 128)
#define FLASH_CLOCK CLOCK_GetFreq(kCLOCK_FlashClk)
#define FAC_FLAG 0x0001
#define SW_VER 0x01;
#define HW_VER 0x01;
#define PRO_VER 0x01;
typedef struct {
	uint8_t HW_ver[4];
	uint32_t ota_flag;
	uint8_t SN_H[8];
	uint8_t SN_L[8];
	uint8_t undefined[488];
}MYFLASH;
typedef enum {
	UNLOCK,
	LOCK,
	ERROR,
	RUNNING,
	UNLOCK_PRE,
	LOCK_PRE
}LOCK_STATE;
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
typedef enum{
	NO_ERROR,
	SWITCH_ERROR,
	HAVE_RPM,
	MOTORA_ERROR=4,
	HUOER_ERROR=8,
	XTAL_ERROR=16
}ERROR_CODE;
static flash_config_t s_flashDriver;
adc_channel_config_t adcChannelConfigStruct;
mscan_frame_t txFrame, rxFrame;
uint8_t send_data[8]={0};
uint8_t recv_data[8]={0};
uint8_t lock_state=0;
uint8_t error_code=0;
uint8_t tpm1_timer_cnt=0;
uint8_t wheel_speed=0;
MYFLASH myflash;
uint8_t recv_ack_flag=0,recv_src=0,recv_dest;
uint8_t update_mode[7]={0x0,0x0,0x1,0x0,0x0,0x0,0x0};
uint8_t normal_mode[7]={0x0,0x0,0x2,0x0,0x0,0x0,0x0};
void StopMotoraRun();
uint32_t get_adc_value();
uint32_t comb_ExtID(uint8_t level,uint8_t charge_flag,uint8_t ACK_flag,uint8_t cnt,uint8_t dest,uint8_t src);
void can_send_msg(uint32_t ID,uint8_t *data ,uint8_t len);
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
void setErrorCode(uint8_t code)
{
	error_code |=code;
}
void clearErrorCode(uint8_t code)
{
	error_code &=~code;
}
void unlock()
{
	//can_send_msg(0xEF,buf,8);

	GPIO_PinWrite(BOARD_INITPINS_PINB_GPIO_PORT, BOARD_INITPINS_PINB_PIN,0);
	GPIO_PinWrite(BOARD_INITPINS_PINA_GPIO_PORT, BOARD_INITPINS_PINA_PIN,1);
	lock_state = UNLOCK_PRE;
	TPM_EnableInterrupts(TPM1, kTPM_TimeOverflowInterruptEnable);
	EnableIRQ(TPM1_IRQn);
	TPM_StartTimer(TPM1, kTPM_SystemClock);
}
void lock()
{
	//can_send_msg(0xEF,buf,8);
	GPIO_PinWrite(BOARD_INITPINS_PINA_GPIO_PORT, BOARD_INITPINS_PINA_PIN,0);
	GPIO_PinWrite(BOARD_INITPINS_PINB_GPIO_PORT, BOARD_INITPINS_PINB_PIN,1);
	lock_state = LOCK_PRE;
	TPM_EnableInterrupts(TPM1, kTPM_TimeOverflowInterruptEnable);
	EnableIRQ(TPM1_IRQn);
	TPM_StartTimer(TPM1, kTPM_SystemClock);
}
void StopMotoraRun()
{
	GPIO_PinWrite(BOARD_INITPINS_PINA_GPIO_PORT, BOARD_INITPINS_PINA_PIN,0);
	GPIO_PinWrite(BOARD_INITPINS_PINB_GPIO_PORT, BOARD_INITPINS_PINB_PIN,0);
}
void TPM1_IRQHandler(void)
{
	uint32_t ret;
	uint32_t adcvalue;
	TPM_ClearStatusFlags(TPM1, kTPM_TimeOverflowFlag);
	tpm1_timer_cnt++;
	if(tpm1_timer_cnt==20)
	{
		adcvalue = get_adc_value();
		//can_send_msg(0x2A4,(uint8_t *)&adcvalue,4);
		TPM_DisableInterrupts(TPM1,kTPM_TimeOverflowInterruptEnable);
		DisableIRQ(TPM1_IRQn);
		TPM_StopTimer(TPM1);
		StopMotoraRun();
		send_data[0] = 0x00;
		if(lock_state ==UNLOCK_PRE)
		{
			send_data[1] = 0xC9;
			if(adcvalue >0x18)
			{
				lock_state = ERROR;
				setErrorCode(MOTORA_ERROR);
			}
			else
			{
				clearErrorCode(MOTORA_ERROR);
			}
			ret = GPIO_PinRead(BOARD_INITPINS_PIA_1_GPIO_PORT, BOARD_INITPINS_PIA_1_PIN);
			if(ret==0)
			{
				send_data[2] = 0x01;
				lock_state = UNLOCK;
			}
			else
			{
				send_data[2] = 0x00;
				lock_state = LOCK;
				//setLockState(UNLOCK);
			}
			send_data[3] = error_code;
			send_data[4] = 0x00;
			send_data[5] = 0x00;
			can_send_msg(comb_ExtID(CTRL,1,ONE_NO_ACK,0,0x01,0x09),send_data,6);
		}
		else if(lock_state == LOCK_PRE)
		{
			send_data[1] = 0xCA;
			if(adcvalue >0x18)
			{
				lock_state = ERROR;
				setErrorCode(MOTORA_ERROR);
				tpm1_timer_cnt = 0;
			}
			else
			{
				clearErrorCode(MOTORA_ERROR);
			}
			ret = GPIO_PinRead(BOARD_INITPINS_PIA_1_GPIO_PORT, BOARD_INITPINS_PIA_1_PIN);
			if(ret==1)
			{
				lock_state = LOCK;
				send_data[2] = 0x00;
			}
			else
			{
				send_data[2] = 0x02;
				//setLockState(LOCK);
			}
			send_data[3] = error_code;
			send_data[4] = 0x00;
			send_data[5] = 0x00;
			can_send_msg(comb_ExtID(CTRL,1,ONE_NO_ACK,0,0x01,0x09),send_data,6);
		}
		tpm1_timer_cnt =0;
	}
	__DSB();
}
void clear_send_data()
{
	memset(send_data,0x55,8);
}
uint32_t comb_ExtID(uint8_t level,uint8_t charge_flag,uint8_t ACK_flag,uint8_t cnt,uint8_t dest,uint8_t src)
{
	uint32_t ID = level<<26|charge_flag<<25|ACK_flag<<17|cnt<<12|dest<<6|src;
	return ID;
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
void analyze_ExtID(uint32_t ID)
{
	recv_ack_flag = ID>>17&0x07;
	recv_dest = ID>>6&0x3F;
	recv_src = ID&0x3F;
}
void analyze_data(uint8_t data[8])
{
	if(data[0] == 0x80)
	{
		if(data[1]==0xC9)
		{
			if(lock_state == UNLOCK)
			{
				send_data[0] = 0x00;
				send_data[1] = 0xC9;
				send_data[2] = 0x00;
				send_data[3] = error_code;
				send_data[4] = 0x00;
				send_data[5] = 0x00;
				can_send_msg(comb_ExtID(CTRL,1,ONE_NO_ACK,0,0x01,0x09),send_data,6);
			}
			else
				unlock();
		}
		else if(data[1]==0xCA)
		{
			if(wheel_speed >15)
			{
				send_data[0] = 0x00;
				send_data[1] = 0xCA;
				if(lock_state == LOCK)
					send_data[2] = 0x00;
				else if(lock_state == UNLOCK)
					send_data[2] = 0x01;
				else
					send_data[2] = 0x02;
				setErrorCode(HAVE_RPM);
				send_data[3] = error_code;
				send_data[4] = 0x00;
				send_data[5] = 0x00;
				can_send_msg(comb_ExtID(CTRL,1,ONE_NO_ACK,0,0x01,0x09),send_data,6);
			}
			else if(lock_state ==LOCK)
			{
				send_data[0] = 0x00;
				send_data[1] = 0xCA;
				send_data[2] = 0x00;
				send_data[3] = error_code;
				send_data[4] = 0x00;
				send_data[5] = 0x00;
				can_send_msg(comb_ExtID(CTRL,1,ONE_NO_ACK,0,0x01,0x09),send_data,6);
			}
			else
				lock();
		}
	}
	else if(data[0]== 0x81)
	{
		if(data[1] == 0xC0)
		{
			send_data[0] = 0x01;
			send_data[1] = 0xC0;

			if(lock_state == LOCK)
			{
				send_data[2] = 0x00;
			}
			else if(lock_state == UNLOCK)
			{
				send_data[2] = 0x01;
			}
			else
				send_data[2] = 0x02;
			send_data[3] = error_code;
			send_data[4] = 0x00;
			send_data[5] = 0x00;
			can_send_msg(comb_ExtID(CTRL,1,ONE_NO_ACK,0,0x01,0x09),send_data,6);
		}
		else if(memcmp(data+1,normal_mode,7)==0)
		{
			memcpy(send_data,data,8);
			can_send_msg(comb_ExtID(UPDATE,1,ONE_RESPONSE,0,0x01,0x09),send_data,8);
		}
		else if(memcmp(data+1,update_mode,7)==0)
		{
			memcpy(send_data,data,8);
			can_send_msg(comb_ExtID(UPDATE,1,ONE_RESPONSE,0,0x01,0x09),send_data,8);
			NVIC_SystemReset();
		}
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
		send_data[3]=0x02;
		can_send_msg(comb_ExtID(UPDATE,1,ONE_NO_ACK,0,0x01,0x09),send_data,8);
	}
}
void MSCAN_1_IRQHandler(void)
{
	if (MSCAN_GetRxBufferFullFlag(MSCAN))
	{
		MSCAN_ReadRxMb(MSCAN, &rxFrame);
		MSCAN_ClearRxBufferFullFlag(MSCAN);
		analyze_ExtID(rxFrame.ID_Type.ID);
		if(recv_ack_flag == ONE_HAVE_ACK&&recv_src == 0x01&&recv_dest == 0x09)
		{
			analyze_data(rxFrame.DSR);
		}
		else if(recv_ack_flag==ONE_NO_ACK&&recv_src == 0x01&&recv_dest == 0x09)
		{
			analyze_no_ack_data(rxFrame.DSR);
		}
	}
	SDK_ISR_EXIT_BARRIER;
}
void MOTORA_timer_init()
{
	tpm_config_t tpmInfo;
	TPM_GetDefaultConfig(&tpmInfo);
	tpmInfo.prescale = kTPM_Prescale_Divide_128;
	TPM_Init(TPM1, &tpmInfo);
	TPM_SetTimerPeriod(TPM1, MSEC_TO_COUNT(100U, TPM_TIMER_SOURCE_CLOCK));

	//TPM_EnableInterrupts(TPM1, kTPM_TimeOverflowInterruptEnable);

	//EnableIRQ(TPM1_IRQn);

	//TPM_StartTimer(TPM1, kTPM_SystemClock);
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
void ADC_init()
{
	adc_config_t adcConfigStrcut;

	ADC_GetDefaultConfig(&adcConfigStrcut);
	ADC_Init(ADC, &adcConfigStrcut);
	ADC_EnableHardwareTrigger(ADC, false);

	    /* Configure the user channel and interrupt. */
	adcChannelConfigStruct.channelNumber                        = 5U;
	adcChannelConfigStruct.enableInterruptOnConversionCompleted = false;
	adcChannelConfigStruct.enableContinuousConversion           = false;

	    /* Enable the releated analog pins. */
	ADC_EnableAnalogInput(ADC, 1U << 5U, true);

}
uint32_t get_adc_value()
{
	ADC_SetChannelConfig(ADC, &adcChannelConfigStruct);
	while (!ADC_GetChannelStatusFlags(ADC));
	return ADC_GetChannelConversionValue(ADC);
}
uint8_t flash_init()
{
	memset(&s_flashDriver, 0, sizeof(flash_config_t));
	memset(&myflash,0,sizeof(myflash));

	FLASH_SetProperty(&s_flashDriver, kFLASH_PropertyFlashClockFrequency, FLASH_CLOCK);

	return FLASH_Init(&s_flashDriver);
}
uint8_t calc_wheel_speed(float wheel_r)
{
	uint16_t cnt = 2000;
	uint8_t result;
	uint32_t inputA,lastinputA,inputB,lastinputB;
	uint16_t wheel_cntA=0,wheel_cntB=0,wheel_cnt=0;
	lastinputA = GPIO_PinRead(BOARD_INITPINS_HA_GPIO_PORT, BOARD_INITPINS_HA_PIN);
	lastinputB = GPIO_PinRead(BOARD_INITPINS_HB_GPIO_PORT, BOARD_INITPINS_HB_PIN);
	while(cnt--)
	{
		inputA = GPIO_PinRead(BOARD_INITPINS_HA_GPIO_PORT, BOARD_INITPINS_HA_PIN);
		inputB = GPIO_PinRead(BOARD_INITPINS_HB_GPIO_PORT, BOARD_INITPINS_HB_PIN);
		if(inputA!=lastinputA)
		{
			wheel_cntA ++;
		}
		if(inputB!=lastinputB)
		{
			wheel_cntB ++;
		}
		SysTick_DelayTicks(1);
	}
	wheel_cnt = wheel_cntA>wheel_cntB?wheel_cntA:wheel_cntB;
	if((wheel_cntA==0&&wheel_cnt-wheel_cntA>10)||(wheel_cntB==0&&wheel_cnt-wheel_cntB>10))
	{
		setErrorCode(HUOER_ERROR);
	}
	result = wheel_cnt*60/16;
	return result;
}
int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    SysTick_Config(SystemCoreClock / 1000U);
    MSCAN_init();
    MOTORA_timer_init();
    ADC_init();
    flash_init();
    send_data[0] = 0x00;
    send_data[1] = 0x01;
    send_data[2] = FAC_FLAG>>8&0xFF;
    send_data[3] = FAC_FLAG&0xFF;
    send_data[4] = SW_VER;
    send_data[5] = HW_VER;
    send_data[6] = PRO_VER;
    send_data[7] = 0x00;
    can_send_msg(comb_ExtID(STATE,1,ONE_NO_ACK,0,0x3F,0x09),send_data,8);
    while(1) {

        wheel_speed = calc_wheel_speed(0);
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
    }
    return 0 ;
}
