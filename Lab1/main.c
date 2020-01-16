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
// Application Name     - UART Demo
// Application Overview - The objective of this application is to showcase the 
//                        use of UART. The use case includes getting input from 
//                        the user and display information on the terminal. This 
//                        example take a string as input and display the same 
//                        when enter is received.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup uart_demo
//! @{
//
//*****************************************************************************

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"
#include "gpio_if.h"
#include "pin_mux_config.h"
#include "uart.h"

// Common interface include
#include "uart_if.h"

//*****************************************************************************
//                          MACROS                                  
//*****************************************************************************
#define APPLICATION_VERSION  "1.4.0"
#define APP_NAME             "GPIO"
#define CONSOLE              UARTA0_BASE
#define UartGetChar()        MAP_UARTCharGet(CONSOLE)
#define UartPutChar(c)       MAP_UARTCharPut(CONSOLE,c)
#define MAX_STRING_LENGTH    80


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile int g_iCounter = 0;

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
//                      LOCAL DEFINITION                                   
//*****************************************************************************

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void
DisplayBanner(char * AppName)
{

    Report("\n\n\n\r");
    Report("\t\t *************************************************\n\r");
    Report("\t\t        CC3200 %s Application       \n\r", AppName);
    Report("\t\t *************************************************\n\r");
    Report("\n\n\n\r");
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
void binary(void)
{

    while (GPIOPinRead(GPIOA2_BASE, 0x40) != 0x40)
    {
        GPIO_IF_LedOff(MCU_ALL_LED_IND);
        int i;
        for (i = 1; i <= 7; i++)
        {
            if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40)
            {
                break;
            } //if clicked at any point in this loop, break out!!

            MAP_UtilsDelay(8000000);

            int counter = i;

            switch(counter)
            {
            case 1:
                GPIO_IF_LedOn(MCU_RED_LED_GPIO);
                break;
            case 2:
                GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
                break;
            case 3:
                GPIO_IF_LedOn(MCU_RED_LED_GPIO);
                GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
                break;
            case 4:
                GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
                break;
            case 5:
                GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
                GPIO_IF_LedOn(MCU_RED_LED_GPIO);
                break;
            case 6:
                GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
                GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
                break;
            case 7:
                GPIO_IF_LedOn(MCU_RED_LED_GPIO);
                GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
                GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
                break;
            }
            MAP_UtilsDelay(8000000);
            GPIO_IF_LedOff(MCU_ALL_LED_IND);
        }
    }
}
void blink(void)
{
    GPIO_IF_LedOff(MCU_ALL_LED_IND);

    while (GPIOPinRead(GPIOA1_BASE, 0x20) != 0x20)
    {
        GPIO_IF_LedOff(MCU_ALL_LED_IND);
        MAP_UtilsDelay(8000000);
        GPIO_IF_LedOn(MCU_ALL_LED_IND);
        MAP_UtilsDelay(8000000);
    }
}
//*****************************************************************************
//
//! Main function handling the uart echo. It takes the input string from the
//! terminal while displaying each character of string. whenever enter command 
//! is received it will echo the string(display). if the input the maximum input
//! can be of 80 characters, after that the characters will be treated as a part
//! of next string.
//!
//! \param  None
//!
//! \return None
//! 
//*****************************************************************************
void main()
{
    //
    // Initailizing the board
    //
    BoardInit();
    //
    // Muxing for Enabling UART_TX and UART_RX.
    //
    PinMuxConfig();
    GPIO_IF_LedConfigure(LED1|LED2|LED3);
    GPIO_IF_LedOff(MCU_ALL_LED_IND);

        //
    //
    // Initialising the Terminal.
    //
    InitTerm();
    //
    // Clearing the Terminal.
    //
    ClearTerm();
    DisplayBanner(APP_NAME);
    Message("\t\t****************************************************\n\r");
    Message("\t\t Push SW3 to start LED binary counting \n\r");
    Message("\t\t Push SW2 to blink LEDs on and off \n\r") ;
    Message("\t\t ****************************************************\n\r");
    Message("\n\n\n\r");

    while(1)
    {
        int j =3;
        //
        // Fetching the input from the board.
        //

        MAP_UtilsDelay(2000000);
        if (GPIOPinRead(GPIOA1_BASE, 0x20) == 0x20)
        {
            if (j !=0){
            Message("\t\t SW3 pressed\n\r");}
            j = 0;
            GPIOPinWrite(GPIOA3_BASE, 0x10, 0);
            binary();
        }
        else if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40)
        {
            if (j != 1){
            Message("\t\t SW2 pressed\n\r");}
            j = 1;
            GPIOPinWrite(GPIOA3_BASE, 0x10, 0x10);
            blink();
        }
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

    

