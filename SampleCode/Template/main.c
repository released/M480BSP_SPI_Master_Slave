/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define LED_R						(PH0)
#define LED_Y						(PH1)
#define LED_G						(PH2)

#define SPI_MASTER_TX_DMA_CH 		(0)
#define SPI_MASTER_RX_DMA_CH 		(1)
#define SPI_MASTER_OPENED_CH   	((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH))

#define SPI_SLAVE_TX_DMA_CH  		(2)
#define SPI_SLAVE_RX_DMA_CH  		(3)
#define SPI_SLAVE_OPENED_CH   		((1 << SPI_SLAVE_TX_DMA_CH) | (1 << SPI_SLAVE_RX_DMA_CH))

#define SPI_TARGET_FREQ				(1000000ul)	//(48000000ul)

#define DATA_NUM					(16)

uint8_t g_au8MasterToSlaveTestPattern[DATA_NUM]={0};
uint8_t g_au8SlaveToMasterTestPattern[DATA_NUM]={0};
uint8_t g_au8MasterRxBuffer[DATA_NUM]={0};
uint8_t g_au8SlaveRxBuffer[DATA_NUM]={0};

enum
{
	SPI_TX = 0,
	SPI_RX = 1,		
};

void SPI_Slave_PDMA_ClrBuffer(uint8_t TxRx)
{
    uint16_t i = 0;

	if (TxRx == SPI_TX)
	{
	    for (i = 0; i < DATA_NUM; i++)
	    {
	        g_au8SlaveToMasterTestPattern[i] = 0xFF;
	    }
	}
	else
	{
	    for (i = 0; i < DATA_NUM; i++)
	    {
	        g_au8SlaveRxBuffer[i] = 0xFF;
	    }
	}
}

void SPI_Slave_PDMA_PreInit(void)
{
	SPI_Slave_PDMA_ClrBuffer(SPI_TX);
	SPI_Slave_PDMA_ClrBuffer(SPI_RX);
	
    PDMA_Open(PDMA, SPI_SLAVE_OPENED_CH);

	//RX
    PDMA_SetTransferCnt(PDMA,SPI_SLAVE_RX_DMA_CH, PDMA_WIDTH_8, DATA_NUM);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,SPI_SLAVE_RX_DMA_CH, (uint32_t)&SPI0->RX, PDMA_SAR_FIX, (uint32_t)g_au8SlaveRxBuffer, PDMA_DAR_INC);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,SPI_SLAVE_RX_DMA_CH, PDMA_SPI0_RX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA,SPI_SLAVE_RX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA->DSCT[SPI_SLAVE_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

	//TX
    PDMA_SetTransferCnt(PDMA,SPI_SLAVE_TX_DMA_CH, PDMA_WIDTH_8, DATA_NUM);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,SPI_SLAVE_TX_DMA_CH, (uint32_t)g_au8SlaveToMasterTestPattern, PDMA_SAR_INC, (uint32_t)&SPI0->TX, PDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,SPI_SLAVE_TX_DMA_CH, PDMA_SPI0_TX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA,SPI_SLAVE_TX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA->DSCT[SPI_SLAVE_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    SPI_TRIGGER_RX_PDMA(SPI0);
    SPI_TRIGGER_TX_PDMA(SPI0);

    PDMA_EnableInt(PDMA, SPI_SLAVE_TX_DMA_CH, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA, SPI_SLAVE_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQn);	
	
}

void SPI_Slave_PDMA_Enable(uint8_t TxRx)
{
    uint16_t i = 0;

	if (TxRx == SPI_TX)
	{
		//prepare slave TX data
		if ((g_au8SlaveRxBuffer[0] == 0xAA)&& 
			(g_au8SlaveRxBuffer[1] == 0xDD))
		{
		    for (i = 0; i < DATA_NUM; i++)
		    {
		        g_au8SlaveToMasterTestPattern[i] = i;
		    }
		}
	
		//TX
	    PDMA_SetTransferCnt(PDMA,SPI_SLAVE_TX_DMA_CH, PDMA_WIDTH_8, DATA_NUM);
   	 	PDMA_SetTransferAddr(PDMA,SPI_SLAVE_TX_DMA_CH, (uint32_t)g_au8SlaveToMasterTestPattern, PDMA_SAR_INC, (uint32_t)&SPI0->TX, PDMA_DAR_FIX);		
	    /* Set request source; set basic mode. */
	    PDMA_SetTransferMode(PDMA,SPI_SLAVE_TX_DMA_CH, PDMA_SPI0_TX, FALSE, 0);
	    SPI_TRIGGER_TX_PDMA(SPI0);
		
//		SPI_Slave_PDMA_ClrBuffer(SPI_RX);
	}
	else
	{	
		//RX
	    PDMA_SetTransferCnt(PDMA,SPI_SLAVE_RX_DMA_CH, PDMA_WIDTH_8, DATA_NUM);
	    /* Set request source; set basic mode. */
	    PDMA_SetTransferMode(PDMA,SPI_SLAVE_RX_DMA_CH, PDMA_SPI0_RX, FALSE, 0);
	    SPI_TRIGGER_RX_PDMA(SPI0);	
		
//		SPI_Slave_PDMA_ClrBuffer(SPI_TX);		
	}
}

void SPI_Slave_Init(void)
{
    SPI_Open(SPI0, SPI_SLAVE, SPI_MODE_0, 8, (uint32_t)NULL);
}

void SPI_Master_PDMA_PreInit(void)
{
    uint16_t i = 0;
	
	//prepare data
    for (i=0; i < DATA_NUM; i++)
    {
        g_au8MasterToSlaveTestPattern[i] = i;
        g_au8MasterRxBuffer[i] = 0xFF;
    }

    PDMA_Open(PDMA, SPI_MASTER_OPENED_CH);

	//TX
    PDMA_SetTransferCnt(PDMA,SPI_MASTER_TX_DMA_CH, PDMA_WIDTH_8, DATA_NUM);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,SPI_MASTER_TX_DMA_CH, (uint32_t)g_au8MasterToSlaveTestPattern, PDMA_SAR_INC, (uint32_t)&SPI0->TX, PDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,SPI_MASTER_TX_DMA_CH, PDMA_SPI0_TX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA,SPI_MASTER_TX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

	//RX	
    PDMA_SetTransferCnt(PDMA,SPI_MASTER_RX_DMA_CH, PDMA_WIDTH_8, DATA_NUM);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,SPI_MASTER_RX_DMA_CH, (uint32_t)&SPI0->RX, PDMA_SAR_FIX, (uint32_t)g_au8MasterRxBuffer, PDMA_DAR_INC);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,SPI_MASTER_RX_DMA_CH, PDMA_SPI0_RX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA,SPI_MASTER_RX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    SPI_TRIGGER_RX_PDMA(SPI0);
    SPI_TRIGGER_TX_PDMA(SPI0);

    PDMA_EnableInt(PDMA, SPI_MASTER_TX_DMA_CH, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA, SPI_MASTER_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQn);
	
}

void SPI_Master_PDMA_Enable(uint8_t TxRx)
{
    uint16_t i = 0;
    static uint16_t j = 0;
	
	if (TxRx == SPI_TX)
	{
		//prepare master TX data
		g_au8MasterToSlaveTestPattern[0] = 0xAA;
		g_au8MasterToSlaveTestPattern[1] = 0xDD;

		j = 1;
	    for (i = 2; i < DATA_NUM ; i++)
	    {
	        g_au8MasterToSlaveTestPattern[i] = (i + 0x10*(j++));
	    }
		j = 0;
	
		//TX
	    PDMA_SetTransferCnt(PDMA,SPI_MASTER_TX_DMA_CH, PDMA_WIDTH_8, DATA_NUM);
		PDMA_SetTransferAddr(PDMA,SPI_MASTER_TX_DMA_CH, (uint32_t)g_au8MasterToSlaveTestPattern, PDMA_SAR_INC, (uint32_t)&SPI0->TX, PDMA_DAR_FIX);		
	    /* Set request source; set basic mode. */
	    PDMA_SetTransferMode(PDMA,SPI_MASTER_TX_DMA_CH, PDMA_SPI0_TX, FALSE, 0);
	    SPI_TRIGGER_TX_PDMA(SPI0);	
	}
	else
	{
		//RX	
		PDMA_SetTransferCnt(PDMA,SPI_MASTER_RX_DMA_CH, PDMA_WIDTH_8, DATA_NUM);
		/* Set request source; set basic mode. */
		PDMA_SetTransferMode(PDMA,SPI_MASTER_RX_DMA_CH, PDMA_SPI0_RX, FALSE, 0);
		SPI_TRIGGER_RX_PDMA(SPI0);
	}
}

//PA0 : SPI0_MOSI , PA1 : SPI0_MISO , PA2 : SPI0_CLK , PA3 : SPI0_SS
void SPI_Master_Init(void)
{
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 8, SPI_TARGET_FREQ);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI0, SPI_SS, SPI_SS_ACTIVE_LOW);
}


void PDMA_IRQHandler(void)
{
	#if defined (ENABLE_SPI_MASTER)	
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);
	
    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
		#if 1
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_GET_ABORT_STS(PDMA));
		#else
        if (PDMA_GET_ABORT_STS(PDMA) & (1 << SPI_MASTER_TX_DMA_CH))
        {

        }
        PDMA_CLR_ABORT_FLAG(PDMA, (1 << SPI_MASTER_TX_DMA_CH));

        if (PDMA_GET_ABORT_STS(PDMA) & (1 << SPI_MASTER_RX_DMA_CH))
        {

        }
        PDMA_CLR_ABORT_FLAG(PDMA, (1 << SPI_MASTER_RX_DMA_CH));
		#endif
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        if((PDMA_GET_TD_STS(PDMA) & SPI_MASTER_OPENED_CH) == SPI_MASTER_OPENED_CH)
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA, SPI_MASTER_OPENED_CH);

			//insert process
			SPI_DISABLE_TX_PDMA(SPI0);
			SPI_DISABLE_RX_PDMA(SPI0);
			LED_Y ^= 1;
			SPI_Master_PDMA_Enable(SPI_TX);
			SPI_Master_PDMA_Enable(SPI_RX);
        }        		
    }
    else if (status & (PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk))     /* Check the DMA time-out interrupt flag */
    {
		LED_G ^= 1;
        PDMA_CLR_TMOUT_FLAG(PDMA,SPI_MASTER_TX_DMA_CH);
        PDMA_CLR_TMOUT_FLAG(PDMA,SPI_MASTER_RX_DMA_CH);
    }
    else
    {

    }

	#elif defined (ENABLE_SPI_SLAVE)
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);
	
    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
		#if 1
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_GET_ABORT_STS(PDMA));
		#else
        if (PDMA_GET_ABORT_STS(PDMA) & (1 << SPI_SLAVE_TX_DMA_CH))
        {

        }
        PDMA_CLR_ABORT_FLAG(PDMA, (1 << SPI_SLAVE_TX_DMA_CH));

        if (PDMA_GET_ABORT_STS(PDMA) & (1 << SPI_SLAVE_RX_DMA_CH))
        {

        }
        PDMA_CLR_ABORT_FLAG(PDMA, (1 << SPI_SLAVE_RX_DMA_CH));
		#endif
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        if((PDMA_GET_TD_STS(PDMA) & SPI_SLAVE_OPENED_CH) == SPI_SLAVE_OPENED_CH)
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA, SPI_SLAVE_OPENED_CH);

			//insert process
			SPI_DISABLE_TX_PDMA(SPI0);
			SPI_DISABLE_RX_PDMA(SPI0);			
			LED_Y ^= 1;
			SPI_Slave_PDMA_Enable(SPI_TX);
			SPI_Slave_PDMA_Enable(SPI_RX);			
        }        		
    }
    else if (status & (PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk))     /* Check the DMA time-out interrupt flag */
    {
		LED_G ^= 1;
        PDMA_CLR_TMOUT_FLAG(PDMA,SPI_SLAVE_TX_DMA_CH);
        PDMA_CLR_TMOUT_FLAG(PDMA,SPI_SLAVE_RX_DMA_CH);
    }
    else
    {

    }

	#endif
	
}

void TMR1_IRQHandler(void)
{
	static uint16_t CNT = 0;	
	static uint32_t log = 0;	
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
	
		if (CNT++ > 1000)
		{		
			CNT = 0;
			printf("%s : %2d\r\n" , __FUNCTION__ , log++);
			LED_R ^= 1;
		}
    }
}

void TIMER1_HW_Init(void)
{
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void TIMER0_HW_Init(void)
{
	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
}

void TIMER0_Polling(uint32_t u32Usec)
{
	TIMER_Delay(TIMER0, u32Usec);
}

void LED_Init(void)
{
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);
    CLK_EnableModuleClock(SPI0_MODULE);

    CLK_EnableModuleClock(PDMA_MODULE);

//	TIMER0_HW_Init();
	TIMER1_HW_Init();
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Setup SPI0 multi-function pins */
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_SPI0_MOSI | SYS_GPA_MFPL_PA1MFP_SPI0_MISO | SYS_GPA_MFPL_PA2MFP_SPI0_CLK | SYS_GPA_MFPL_PA3MFP_SPI0_SS;

    /* Enable SPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* Enable SPI0 I/O high slew rate */
    GPIO_SetSlewCtl(PA, 0xF, GPIO_SLEWCTL_HIGH);
	
    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
	
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());

	LED_Init();
	TIMER1_Init();

	#if defined (ENABLE_SPI_MASTER)
	
	SPI_Master_Init();
	SPI_Master_PDMA_PreInit();
	
	#elif defined (ENABLE_SPI_SLAVE)
	
	SPI_Slave_Init();
	SPI_Slave_PDMA_PreInit();

	#endif

    /* Got no where to go, just loop forever */
    while(1)
    {
//		TIMER0_Polling(100);
		

    }

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
