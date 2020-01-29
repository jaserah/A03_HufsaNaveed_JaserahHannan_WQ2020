//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - SPI Demo
// Application Overview - The demo application focuses on showing the required
//                        initialization sequence to enable the CC3200 SPI
//                        module in full duplex 4-wire master and slave mode(s).
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup SPI_Demo
//! @{
//
//*****************************************************************************

// Standard includes
#include <string.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"

// Common interface includes
#include "uart_if.h"
#include "pinmux.h"

#include "Adafruit_GFX.h"
#include "Adafruit_OLED.h"
#include "Adafruit_SSD1351.h"

#include "i2c_if.h"


#define APPLICATION_VERSION     "1.4.0"
//*****************************************************************************
//
// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode
// MASTER_MODE = 0 : Application in slave mode
//
//*****************************************************************************
#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

#define MASTER_MSG       "This is CC3200 SPI Master Application\n\r"
#define SLAVE_MSG        "This is CC3200 SPI Slave Application\n\r"

#define BLACK 0x0000
#define RED 0xF800
#define BLUE 0x001F

#define FOREVER                 1
#define CONSOLE                 UARTA0_BASE
#define FAILURE                 -1
#define SUCCESS                 0
#define UART_PRINT              Report
#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}
#define negx                   1
#define posx                   2
#define negy                   3
#define posy                   4
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile int xdiff;
volatile int xpos;
volatile int xold;
volatile int ydiff;
volatile int ypos;
volatile int yold;

static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
static unsigned char g_ucRxBuff[TR_BUFF_SIZE];
static unsigned char ucTxBuffNdx;
static unsigned char ucRxBuffNdx;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************



//*****************************************************************************
//
//! SPI Slave Interrupt handler
//!
//! This function is invoked when SPI slave has its receive register full or
//! transmit register empty.
//!
//! \return None.
//
//*****************************************************************************
static void SlaveIntHandler()
{
    unsigned long ulRecvData;
    unsigned long ulStatus;

    ulStatus = MAP_SPIIntStatus(GSPI_BASE,true);

    MAP_SPIIntClear(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);

    if(ulStatus & SPI_INT_TX_EMPTY)
    {
        MAP_SPIDataPut(GSPI_BASE,g_ucTxBuff[ucTxBuffNdx%TR_BUFF_SIZE]);
        ucTxBuffNdx++;
    }

    if(ulStatus & SPI_INT_RX_FULL)
    {
        MAP_SPIDataGetNonBlocking(GSPI_BASE,&ulRecvData);
        g_ucTxBuff[ucRxBuffNdx%TR_BUFF_SIZE] = ulRecvData;
        Report("%c",ulRecvData);
        ucRxBuffNdx++;
    }
}

//*****************************************************************************
//
//! SPI Master mode main loop
//!
//! This function configures SPI modelue as master and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
void MasterMain()
{
   // unsigned long ulUserData;
   // unsigned long ulDummy;

    //
    // Initialize the message
    //
 //   memcpy(g_ucTxBuff,MASTER_MSG,sizeof(MASTER_MSG));

 /*   //
    // Set Tx buffer index
    //
    ucTxBuffNdx = 0;
    ucRxBuffNdx = 0;*/

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    //
    // Print mode on uart
    //
    Message("Enabled SPI Interface in Master Mode\n\r");

    //
    // User input
    //
  //  Report("Press any key to transmit data....");

    //
    // Read a character from UART terminal
    //
 //   ulUserData = MAP_UARTCharGet(UARTA0_BASE);


    //
    // Send the string to slave. Chip Select(CS) needs to be
    // asserted at start of transfer and deasserted at the end.
    //
    //MAP_SPITransfer(GSPI_BASE,g_ucTxBuff,g_ucRxBuff,50,
      //      SPI_CS_ENABLE|SPI_CS_DISABLE);

    //
    // Report to the user
    //
   // Report("\n\rSend      %s",g_ucTxBuff);
    //Report("Received  %s",g_ucRxBuff);

    //
    // Print a message
    //
   // Report("\n\rType here (Press enter to exit) :");

    //
    // Initialize variable
    //
   // ulUserData = 0;

    //
    // Enable Chip select
    //
    MAP_SPICSEnable(GSPI_BASE);
    Adafruit_Init();

    fillScreen(BLACK);
    xdiff = 0;
    xpos = 64;
    ydiff = 0;
    ypos = 64;

    while(1)
    {
        int working = ProcessReadRegCommand(negx);
        if (working == 0)
        {
            UART_PRINT("Success");
        }
        else if (working > 0)
        {
            UART_PRINT("Error");
        }
    }

    //
    // Disable chip select
    //
    MAP_SPICSDisable(GSPI_BASE);
}

//*****************************************************************************
//
//! SPI Slave mode main loop
//!
//! This function configures SPI modelue as slave and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
void SlaveMain()
{
    //
    // Initialize the message
    //
    memcpy(g_ucTxBuff,SLAVE_MSG,sizeof(SLAVE_MSG));

    //
    // Set Tx buffer index
    //
    ucTxBuffNdx = 0;
    ucRxBuffNdx = 0;

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_SLAVE,SPI_SUB_MODE_0,
                     (SPI_HW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Register Interrupt Handler
    //
    MAP_SPIIntRegister(GSPI_BASE,SlaveIntHandler);

    //
    // Enable Interrupts
    //
    MAP_SPIIntEnable(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    //
    // Print mode on uart
    //
    Message("Enabled SPI Interface in Slave Mode\n\rReceived : ");
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

void
DisplayBuffer(unsigned char *pucDataBuf, unsigned char ucLen)
{
    unsigned char ucBufIndx = 0;
    UART_PRINT("Read contents");
    UART_PRINT("\n\r");
    while(ucBufIndx < ucLen)
    {
        UART_PRINT(" 0x%x, ", pucDataBuf[ucBufIndx]);
        ucBufIndx++;
        if((ucBufIndx % 8) == 0)
        {
            UART_PRINT("\n\r");
        }
    }
    UART_PRINT("\n\r");
}

int
ProcessReadRegCommand(int xy)
{
    unsigned char ucDevAddr, ucRegOffset, ucRdLen;
    unsigned char aucRdDataBuf[256];
    unsigned char aucRdDataBufy[256];

    ucDevAddr = 0x18; //to accelerate
    ucRdLen = 1;

    if (xy == negx)
    {
        ucRegOffset = 0x3;
    }
    if (xy == negy)
    {
        ucRegOffset = 0x5;
    }
    if (xy == posx)
    {
        ucRegOffset = 0x3;
    }
    if (xy == posy)
    {
        ucRegOffset = 0x5;
    }

    //
    // Write the register address to be read from.
    // Stop bit implicitly assumed to be 0.
    //
    RET_IF_ERR(I2C_IF_Write(ucDevAddr,&ucRegOffset,1,0));

    //
    // Read the specified length of data
    //
    RET_IF_ERR(I2C_IF_Read(ucDevAddr, &aucRdDataBuf[0], ucRdLen));

    UART_PRINT("I2C Read From address complete\n\r");

    //
    // Display the buffer over UART on successful readreg
    //
    DisplayBuffer(aucRdDataBuf, ucRdLen);

    /*FROM Z
    ucREG_OFFSET = 0x5;
    RET_IF_ERR(I2C_IF_Write(DEVICE_ADDRESS,&ucREG_OFFSET,1,0));
    I2C_IF_Read(DEVICE_ADDRESS, &aucRdDataBufy[0], ucREAD_LENGTH);
    UART_PRINT("I2C Read From address complete Y\n\r");
    DisplayBuffer(aucRdDataBufy, ucREAD_LENGTH);*/

    if ((aucRdDataBuf[0]>= 0x3) && (aucRdDataBuf[0]<= 0x4))
    {
        xdiff = 0;
    }
    if ((aucRdDataBuf[0]>= 0x27) && (aucRdDataBuf[0]<= 0x29))
    {
        xdiff = 1;
    }
    if ((aucRdDataBuf[0]>= 0x30) && (aucRdDataBuf[0]<= 0x32))
    {
        xdiff = 2;
    }
    if ((aucRdDataBuf[0]>= 0x34) && (aucRdDataBuf[0]<= 0x35))
    {
        xdiff = 3;
    }
    if ((aucRdDataBuf[0]>= 0x36) && (aucRdDataBuf[0]<= 0x37))
    {
        xdiff = 4;
    }
    if ((aucRdDataBuf[0]>= 0x38) && (aucRdDataBuf[0]<= 0x40))
    {
        xdiff = 5;
    }
    if ((aucRdDataBuf[0]>= 0xe6) && (aucRdDataBuf[0]<= 0xe0))
    {
        xdiff = -1;
    }
    if ((aucRdDataBuf[0]>= 0xdf) && (aucRdDataBuf[0]<= 0xdc))
    {
        xdiff = -2;
    }
    if ((aucRdDataBuf[0]>= 0xd6) && (aucRdDataBuf[0]<= 0xcf))
    {
        xdiff = -3;
    }
    if ((aucRdDataBuf[0]>= 0xc6) && (aucRdDataBuf[0]<= 0xc3))
    {
        xdiff = -5;
    }

    if ((aucRdDataBufy[0]>= 0xfb) && (aucRdDataBufy[0]<= 0xfe))
    {
        ydiff = 0;
    }
    if ((aucRdDataBufy[0]>= 0x10) && (aucRdDataBufy[0]<= 0x14))
    {
        ydiff = 1;
    }
    if ((aucRdDataBufy[0]>= 0x1c) && (aucRdDataBufy[0]<= 0x20))
    {
        ydiff = 2;
    }
    if ((aucRdDataBufy[0]>= 0x25) && (aucRdDataBufy[0]<= 0x2c))
    {
        ydiff = 3;
    }
    if ((aucRdDataBufy[0]>= 0x30) && (aucRdDataBufy[0]<= 0x35))
    {
        ydiff = 4;
    }
    if ((aucRdDataBufy[0]>= 0x3a) && (aucRdDataBufy[0]<= 0x3c))
    {
        ydiff = 5;
    }
    if ((aucRdDataBufy[0]>= 0xe0) && (aucRdDataBufy[0]<= 0xe3))
    {
        ydiff = -1;
    }
    if ((aucRdDataBufy[0]>= 0xe5) && (aucRdDataBufy[0]<= 0xe9))
    {
        ydiff = -2;
    }
    if ((aucRdDataBufy[0]>= 0xd6) && (aucRdDataBufy[0]<= 0xc0))
    {
        ydiff = -3;
    }
    if ((aucRdDataBufy[0]>= 0xc6) && (aucRdDataBufy[0]<= 0xc9))
    {
        ydiff = -4;
    }
    if ((aucRdDataBufy[0]>= 0xb7) && (aucRdDataBufy[0]<= 0xbd))
    {
        ydiff = -1;
    }

    xold = xpos;
    yold = ypos;

    int xtot = xpos + xdiff;
    if (xtot > 127)
    {
        xpos = 125;
    }
    else if (xtot < 2)
    {
        xpos = 2;
    }
    else
    {
        xpos = xpos + xdiff;
    }


    int ytot = ypos + ydiff;
    if (ytot < 2)
    {
        ypos = 2;
    }
    else if (ytot > 127)
    {
        ypos = 125;
    }
    else
    {
        ypos = ypos + ydiff;
    }

    fillCircle( yold, xold, 2, BLACK);
    fillCircle( ypos, xpos, 2, RED);
    return SUCCESS;
}


//*****************************************************************************
//
//! Main function for spi demo application
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
void main()
{
    int iRetVal;
    char acCmdStore[512];
    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    //
    // I2C Init
    //
    I2C_IF_Open(I2C_MASTER_MODE_FST);

    //
    // Initialising the Terminal.
    //
    InitTerm();

    //
    // Clearing the Terminal.
    //
    ClearTerm();

    //
    // Display the Banner
    //
    Message("\n\n\n\r");
    Message("\t\t   ********************************************\n\r");
    Message("\t\t        CC3200 SPI Demo Application  \n\r");
    Message("\t\t   ********************************************\n\r");
    Message("\n\n\n\r");

    //
    // Reset the peripheral
    //
    MAP_PRCMPeripheralReset(PRCM_GSPI);

#if MASTER_MODE

    MasterMain();
    Adafruit_Init();
#else

    SlaveMain();

#endif

    while(1)
    {

    }

}

