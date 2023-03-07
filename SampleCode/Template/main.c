/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "misc_config.h"

#include "cdc_serial.h"
/*_____ D E C L A R A T I O N S ____________________________________________*/

struct flag_32bit flag_PROJ_CTL;
#define FLAG_PROJ_TIMER_PERIOD_1000MS                 	(flag_PROJ_CTL.bit0)
#define FLAG_PROJ_REVERSE1                   			(flag_PROJ_CTL.bit1)
#define FLAG_PROJ_REVERSE2                 				(flag_PROJ_CTL.bit2)
#define FLAG_PROJ_REVERSE3                              (flag_PROJ_CTL.bit3)
#define FLAG_PROJ_REVERSE4                              (flag_PROJ_CTL.bit4)
#define FLAG_PROJ_REVERSE5                              (flag_PROJ_CTL.bit5)
#define FLAG_PROJ_REVERSE6                              (flag_PROJ_CTL.bit6)
#define FLAG_PROJ_REVERSE7                              (flag_PROJ_CTL.bit7)


/*_____ D E F I N I T I O N S ______________________________________________*/

volatile unsigned int counter_systick = 0;
volatile uint32_t counter_tick = 0;


/* If crystal-less is enabled, system won't use any crystal as clock source
   If using crystal-less, system will be 48MHz, otherwise, system is 72MHz
*/ 
#define CRYSTAL_LESS                                    0
#define HIRC_AUTO_TRIM                                  0x611   /* Use USB signal to fine tune HIRC 48MHz */
#define TRIM_INIT                                       (SYS_BASE+0x110)

STR_VCOM_LINE_CODING gLineCoding = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t gCtrlSignal = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

#define RXBUFSIZE                                       512 /* RX buffer size */
#define TXBUFSIZE                                       512 /* RX buffer size */

#define TX_FIFO_SIZE                                    16  /* TX Hardware FIFO size */

// #define VCOM_UART                                       (UART1)
// #define VCOM_UART_IRQHandler                            (UART1_IRQHandler)

volatile uint8_t comRbuf[RXBUFSIZE];
volatile uint16_t comRbytes = 0;
volatile uint16_t comRhead = 0;
volatile uint16_t comRtail = 0;

volatile uint8_t comTbuf[TXBUFSIZE];
volatile uint16_t comTbytes = 0;
volatile uint16_t comThead = 0;
volatile uint16_t comTtail = 0;

uint8_t gRxBuf[64] = {0};
volatile uint8_t *gpu8RxBuf = 0;
volatile uint32_t gu32RxSize = 0;
volatile uint32_t gu32TxSize = 0;

volatile int8_t gi8BulkOutReady = 0;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

unsigned int get_systick(void)
{
	return (counter_systick);
}

void set_systick(unsigned int t)
{
	counter_systick = t;
}

void systick_counter(void)
{
	counter_systick++;
}

void SysTick_Handler(void)
{

    systick_counter();

    if (get_systick() >= 0xFFFFFFFF)
    {
        set_systick(0);      
    }

    // if ((get_systick() % 1000) == 0)
    // {
       
    // }

    #if defined (ENABLE_TICK_EVENT)
    TickCheckTickEvent();
    #endif    
}

void SysTick_delay(unsigned int delay)
{  
    
    unsigned int tickstart = get_systick(); 
    unsigned int wait = delay; 

    while((get_systick() - tickstart) < wait) 
    { 
    } 

}

void SysTick_enable(unsigned int ticks_per_second)
{
    set_systick(0);
    if (SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while (1);
    }

    #if defined (ENABLE_TICK_EVENT)
    TickInitTickEvent();
    #endif
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void tick_counter(void)
{
	counter_tick++;
    if (get_tick() >= 60000)
    {
        set_tick(0);
    }
}

// void delay_ms(uint16_t ms)
// {
// 	TIMER_Delay(TIMER0, 1000*ms);
// }


void VCOM_TransferData(void)
{
    int32_t i, i32Len;

    /* Check whether USB is ready for next packet or not */
    if(gu32TxSize == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(comRbytes)
        {
            i32Len = comRbytes;
            if(i32Len > EP2_MAX_PKT_SIZE)
                i32Len = EP2_MAX_PKT_SIZE;

            for(i = 0; i < i32Len; i++)
            {
                gRxBuf[i] = comRbuf[comRhead++];
                // printf("R(%2d):0x%2X\r\n",i,comRbuf[i]);
                if(comRhead >= RXBUFSIZE)
                    comRhead = 0;
            }

            __set_PRIMASK(1);
            comRbytes -= i32Len;
            __set_PRIMASK(0);

            gu32TxSize = i32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)gRxBuf, i32Len);
            USBD_SET_PAYLOAD_LEN(EP2, i32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP2_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            i32Len = USBD_GET_PAYLOAD_LEN(EP2);
            if(i32Len == EP2_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP2, 0);
        }
    }

    /* Process the Bulk out data when bulk out data is ready. */
    if(gi8BulkOutReady && (gu32RxSize <= TXBUFSIZE - comTbytes))
    {
        for(i = 0; i < gu32RxSize; i++)
        {
            comTbuf[comTtail++] = gpu8RxBuf[i];
            // printf("T(%2d):0x%2X\r\n",i,gpu8RxBuf[i]);
            if(comTtail >= TXBUFSIZE)
                comTtail = 0;
        }

        __set_PRIMASK(1);
        comTbytes += gu32RxSize;
        __set_PRIMASK(0);

        gu32RxSize = 0;
        gi8BulkOutReady = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }

    /* Process the software TX FIFO */
    if(comTbytes)
    {
        /* Check if TX is working */
        if((VCOM_UART->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            VCOM_UART->DAT = comTbuf[comThead++];
            if(comThead >= TXBUFSIZE)
                comThead = 0;

            comTbytes--;

            /* Enable TX Empty Interrupt. (Trigger first one) */
            VCOM_UART->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
}

void VCOM_USB_polling(void)
{

#if CRYSTAL_LESS
        /* Start USB trim if it is not enabled. */
        if((SYS->IRCTCTL & SYS_IRCTCTL_FREQSEL_Msk) != 1)
        {
            /* Start USB trim only when SOF */
            if(USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

                /* Re-enable crystal-less */
                SYS->IRCTCTL = HIRC_AUTO_TRIM | (8 << SYS_IRCTCTL_BOUNDARY_Pos);
            }
        }

        /* Disable USB Trim when error */
        if(SYS->IRCTISTS & (SYS_IRCTISTS_CLKERRIF_Msk | SYS_IRCTISTS_TFAILIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Disable crystal-less */
            SYS->IRCTCTL = 0;

            /* Clear error flags */
            SYS->IRCTISTS = SYS_IRCTISTS_CLKERRIF_Msk | SYS_IRCTISTS_TFAILIF_Msk;

            /* Clear SOF */
            USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
        }
#endif

        /* Enter power down when USB suspend */
        // if(g_u8Suspend)
        //     PowerDown();

        VCOM_TransferData();

}

void VCOM_USB_Init(void)
{
    /* Open USB controller */
    USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);

    /* Endpoint configuration */
    VCOM_Init();

    /* Start USB device */
    USBD_Start();

    /* Enable USB device interrupt */
    NVIC_EnableIRQ(USBD_IRQn);

#if CRYSTAL_LESS
    /* Backup default trim */
    u32TrimInit = M32(TRIM_INIT);
#endif

    /* Clear SOF */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

    NVIC_EnableIRQ(VCOM_UART_IRQn);
}

void VCOM_UART_IRQHandler(void)
{
    uint32_t u32IntStatus;
    uint8_t bInChar;
    int32_t size;

    u32IntStatus = VCOM_UART->INTSTS;

    if((u32IntStatus & 0x1 /* RDAIF */) || (u32IntStatus & 0x10 /* TOUT_IF */))
    {
        /* Receiver FIFO threshold level is reached or RX time out */

        /* Get all the input characters */
        while((VCOM_UART->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            /* Get the character from UART Buffer */
            bInChar = VCOM_UART->DAT;

            /* Check if buffer full */
            if(comRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                comRbuf[comRtail++] = bInChar;
                if(comRtail >= RXBUFSIZE)
                    comRtail = 0;
                comRbytes++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & 0x2 /* THRE_IF */)
    {

        if(comTbytes && (VCOM_UART->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the TX FIFO */
            size = comTbytes;
            if(size >= TX_FIFO_SIZE)
            {
                size = TX_FIFO_SIZE;
            }

            while(size)
            {
                bInChar = comTbuf[comThead++];
                VCOM_UART->DAT = bInChar;
                if(comThead >= TXBUFSIZE)
                    comThead = 0;
                comTbytes--;
                size--;
            }
        }
        else
        {
            /* No more data, just stop TX (Stop work) */
            VCOM_UART->INTEN &= (~UART_INTEN_THREIEN_Msk);
        }
    }

}

void UARTx_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    #if defined (ENABLE_UART0)
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;
    #elif defined (ENABLE_UART1)
    SYS->IPRST1 |=  SYS_IPRST1_UART1RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART1RST_Msk;
    #endif

    /* Configure UART1 and set UART1 Baudrate */
    VCOM_UART->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER((__HIRC >> 1), 115200);
    VCOM_UART->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    VCOM_UART->INTEN = UART_INTEN_TOCNTEN_Msk | UART_INTEN_RDAIEN_Msk;

	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
	#endif	
}

void TMR1_IRQHandler(void)
{
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_1000MS = 1;//set_flag(flag_timer_period_1000ms ,ENABLE);
		}

		if ((get_tick() % 50) == 0)
		{

		}	
    }
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void loop(void)
{
	// static uint32_t LOG1 = 0;
	// static uint32_t LOG2 = 0;

    if ((get_systick() % 1000) == 0)
    {
        // printf("%s(systick) : %4d\r\n",__FUNCTION__,LOG2++);    
    }

    if (FLAG_PROJ_TIMER_PERIOD_1000MS)//(is_flag_set(flag_timer_period_1000ms))
    {
        FLAG_PROJ_TIMER_PERIOD_1000MS = 0;//set_flag(flag_timer_period_1000ms ,DISABLE);

        // printf("%s(timer) : %4d\r\n",__FUNCTION__,LOG1++);
        // PB14 ^= 1; 
        PA11 ^= 1;               
    }

    VCOM_USB_polling();
}

// void UARTx_Process(void)
// {
// 	uint8_t res = 0;
// 	res = UART_READ(UART0);

// 	if (res > 0x7F)
// 	{
// 		printf("invalid command\r\n");
// 	}
// 	else
// 	{
// 		printf("press : %c\r\n" , res);
// 		switch(res)
// 		{
// 			case '1':
// 				break;

// 			case 'X':
// 			case 'x':
// 			case 'Z':
// 			case 'z':
//                 SYS_UnlockReg();
// 				// NVIC_SystemReset();	// Reset I/O and peripherals , only check BS(FMC_ISPCTL[1])
//                 // SYS_ResetCPU();     // Not reset I/O and peripherals
//                 SYS_ResetChip();    // Reset I/O and peripherals ,  BS(FMC_ISPCTL[1]) reload from CONFIG setting (CBS)	
// 				break;
// 		}
// 	}
// }

// void UART0_IRQHandler(void)
// {
//     if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
//     {
//         while(UART_GET_RX_EMPTY(UART0) == 0)
//         {
// 			UARTx_Process();
//         }
//     }

//     if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
//     {
//         UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
//     }	
// }

// void UART0_Init(void)
// {
//     SYS_ResetModule(UART0_RST);

//     /* Configure UART0 and set UART0 baud rate */
//     UART_Open(UART0, 115200);
//     UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
//     NVIC_EnableIRQ(UART0_IRQn);
	
// 	#if (_debug_log_UART_ == 1)	//debug
// 	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
// 	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());
// 	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
// 	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
// 	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
// 	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
// 	#endif	

//     #if 0
//     printf("FLAG_PROJ_TIMER_PERIOD_1000MS : 0x%2X\r\n",FLAG_PROJ_TIMER_PERIOD_1000MS);
//     printf("FLAG_PROJ_REVERSE1 : 0x%2X\r\n",FLAG_PROJ_REVERSE1);
//     printf("FLAG_PROJ_REVERSE2 : 0x%2X\r\n",FLAG_PROJ_REVERSE2);
//     printf("FLAG_PROJ_REVERSE3 : 0x%2X\r\n",FLAG_PROJ_REVERSE3);
//     printf("FLAG_PROJ_REVERSE4 : 0x%2X\r\n",FLAG_PROJ_REVERSE4);
//     printf("FLAG_PROJ_REVERSE5 : 0x%2X\r\n",FLAG_PROJ_REVERSE5);
//     printf("FLAG_PROJ_REVERSE6 : 0x%2X\r\n",FLAG_PROJ_REVERSE6);
//     printf("FLAG_PROJ_REVERSE7 : 0x%2X\r\n",FLAG_PROJ_REVERSE7);
//     #endif

// }

void GPIO_Init (void)
{
    // SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk)) | (SYS_GPB_MFPH_PB14MFP_GPIO);
    // SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB15MFP_Msk)) | (SYS_GPB_MFPH_PB15MFP_GPIO);

    // GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);
    // GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT);	

    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA11MFP_Msk)) | (SYS_GPA_MFPH_PA11MFP_GPIO);

    GPIO_SetMode(PA, BIT11, GPIO_MODE_OUTPUT);

}

void SYS_Init(void)
{
    SYS_UnlockReg();	

    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
//    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_XT1_OUT;
//    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_XT1_IN;

    /* Enable GPIO Port F module clock */
//    CLK_EnableModuleClock(GPIOF_MODULE);

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) as input mode to use HXT */
//    GPIO_SetMode(PF, BIT2|BIT3, GPIO_MODE_INPUT);


//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);	

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);		
	

#if (!CRYSTAL_LESS)
    /* Enable GPIO Port F module clock */
    CLK_EnableModuleClock(GPIOF_MODULE);

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) as input mode to use HXT */
    GPIO_SetMode(PF, BIT2|BIT3, GPIO_MODE_INPUT);

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock to 72MHz */
    CLK_SetCoreClock(72000000);

    /* Select USB clock source as PLL and USB clock divider as 3 */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_PLL, CLK_CLKDIV0_USB(3));
#else
    /* Select HCLK clock source to HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Select USB clock source as HIRC and USB clock divider as 1 */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_HIRC, CLK_CLKDIV0_USB(1));
#endif

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
//    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD_MODULE);

    #if defined (ENABLE_UART0)
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC_DIV2, CLK_CLKDIV0_UART0(1));
    #elif defined (ENABLE_UART1)
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC_DIV2, CLK_CLKDIV0_UART1(1));
    #endif

    CLK_EnableModuleClock(TMR1_MODULE);
  	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC_DIV2, 0);

    #if defined (ENABLE_UART0)
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
    #elif defined (ENABLE_UART1)
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA8MFP_Msk | SYS_GPA_MFPH_PA9MFP_Msk)) |
                    (SYS_GPA_MFPH_PA8MFP_UART1_RXD | SYS_GPA_MFPH_PA9MFP_UART1_TXD);
    #endif

   /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

int main()
{
    SYS_Init();

	GPIO_Init();
	// UART0_Init();
	TIMER1_Init();

    SysTick_enable(1000);
    #if defined (ENABLE_TICK_EVENT)
    TickSetTickEvent(1000, TickCallback_processA);  // 1000 ms
    TickSetTickEvent(5000, TickCallback_processB);  // 5000 ms
    #endif

    UARTx_Init();
    VCOM_USB_Init();


    /* Got no where to go, just loop forever */
    while(1)
    {
        loop();

    }
}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
