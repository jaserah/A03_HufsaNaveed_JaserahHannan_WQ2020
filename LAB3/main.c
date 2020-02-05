
//*****************************************************************************
//
// Application Name     - int_sw
// Application Overview - The objective of this application is to demonstrate
//                          GPIO interrupts using SW2 and SW3.
//                          NOTE: the switches are not debounced!
//
//*****************************************************************************

//****************************************************************************
//
//! \addtogroup int_sw
//! @{
//
//****************************************************************************

// Standard includes
#include <stdio.h>
#include <string.h>
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

// Common interface includes
#include "uart_if.h"
#include "timer_if.h"
#include "timer.h"

#include "pin_mux_config.h"


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
extern void (* const g_pfnVectors[])(void);

volatile unsigned long SW2_intcount;
volatile unsigned long SW3_intcount;
volatile unsigned char SW2_intflag;
volatile unsigned char SW3_intflag;

volatile long time;
volatile long oldTime;
char arr[50];
char zero[25];
char one[25];
char two[25];
char three[25];
char four[25];
char five[25];
char six[25];
char seven[25];
char eight[25];
char nine[25];
char last[25];
char mute[25];

char prevButton;
int numPresses;

//letter arrays
char two_letters[3] = "ABC";
char three_letters[3] = "DEF";
char four_letters[3] = "GHI";
char five_letters[3] = "JKL";
char six_letters[3] = "MNO";
char seven_letters[4] = "PQRS";
char eight_letters[3] = "TUV";
char nine_letters[4] = "WXYZ";

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES                           
//*****************************************************************************
static void BoardInit(void);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                         
//*****************************************************************************

static void GPIOA2IntHandler(void) {    // GPIO Handler
    unsigned long ulStatus;
    int timeNow;
    ulStatus = MAP_GPIOIntStatus (GPIOA0_BASE, 0x20);
    MAP_GPIOIntClear(GPIOA0_BASE, ulStatus);       // clear interrupts on GPIOA0
    MAP_TimerDisable(TIMERA0_BASE, TIMER_A);
    timeNow = Timer_IF_GetCount(TIMERA0_BASE, TIMER_A);
    time = timeNow - oldTime;
    oldTime = timeNow;
    MAP_TimerEnable(TIMERA0_BASE, TIMER_A);

    if (SW2_intcount > 37){
        MAP_GPIOIntDisable(GPIOA0_BASE, 0x20);
    }


   if (time < 0){
        time = time + 400001;
        if (time < 100000){
                    arr[SW2_intcount] = '0';
                } else {
                    arr[SW2_intcount] = '1';
                }
   } else {
       if (time < 100000){
                           arr[SW2_intcount] = '0';
                       } else {
                           arr[SW2_intcount] = '1';
                       }
    }

    SW2_intcount++;
    SW2_intflag=1;
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
BoardInit(void) {
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
    
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

int compareStrings(char* first, char* second){
    if (strstr(first, second) != NULL){
            return 1;
    }
    return 0;
}

static void myStrings(void){
    strcpy(zero, "1000001101111100001");
    strcpy(one, "11011111100010000111");
    strcpy(two, "1000001101111101001");
    strcpy(three, "1000001101111111001");
    strcpy(four, "1101111100101000110");
    strcpy(five, "1111110101000010101");
    strcpy(six, "1101111101101000100");
    strcpy(seven, "1000001101111111101");
    strcpy(eight, "1000001101111100011");
    strcpy(nine, "1101111110011000011");
    strcpy(last, "1101111101011000101");
    strcpy(mute, "1101111110010000011");

}
static void arrayLoop(void){
    int i;
    for (i = 0; i < 50; i++){
        arr[i] = '0';
    }
}

static void decode(void){
    Report("Array: %s\r\n", arr);
    char button = '?';
    if (compareStrings(arr, zero) == 1){
        MAP_UtilsDelay(800000);
        arrayLoop();
        Message("0\n");
        button = '0';
    }
    else if (compareStrings(arr, one) == 1){
        MAP_UtilsDelay(800000);
        arrayLoop();
        Message("1\n");
        button = '1';
    }
    else if (compareStrings(arr, two) == 1){
        MAP_UtilsDelay(800000);
        arrayLoop();
        Message("2\n");
        button = '2';
    }
    else if (compareStrings(arr, three) == 1){
        MAP_UtilsDelay(800000);
        arrayLoop();
        Message("3\n");
        button = '3';
    }
    else if (compareStrings(arr, four) == 1){
        MAP_UtilsDelay(800000);
        arrayLoop();
        Message("4\n");
        button = '4';
    }
    else if (compareStrings(arr, five) == 1){
        MAP_UtilsDelay(800000);
        arrayLoop();
        Message("5\n");
        button = '5';
    }
    else if (compareStrings(arr, six) == 1){
        MAP_UtilsDelay(800000);
        arrayLoop();
        Message("6\n");
        button = '6';
    }
    else if (compareStrings(arr, seven) == 1){
        MAP_UtilsDelay(800000);
        arrayLoop();
        Message("7\n");
        button = '7';
    }
    else if (compareStrings(arr, eight) == 1){
        MAP_UtilsDelay(800000);
        arrayLoop();
        Message("8\n");
        button = '8';
    }
    else if (compareStrings(arr, nine) == 1){
        MAP_UtilsDelay(800000);
        arrayLoop();
        Message("9\n");
        button = '9';
    }
    else if (compareStrings(arr, last) == 1){
        MAP_UtilsDelay(800000);
        arrayLoop();
        Message("LAST(DELETE)\n");
        button = 'L';
    }
    else if (compareStrings(arr, mute) == 1){
        MAP_UtilsDelay(800000);
        arrayLoop();
        Message("MUTE(ENTER)\n");
        button = 'M';
    }
    else{
        arrayLoop();
        Message("unknown\n");
    }

    MAP_UtilsDelay(800000);
    if(button == prevButton){
        numPresses++;
    }
    else{
        numPresses = 0;
        Timer_IF_Start(TIMERA1_BASE, TIMER_A, 2000);
    }

    prevButton = button;

}

int find(char* first, char c){

    int i;

    for (i = 0; i < strlen(first); i++){
        if (c == first[i]) return 1;
    }

    return 0;
}

void letters(){
    char letter[1] = "x";
    if(prevButton == '2'){
        letter[0] = two_letters[numPresses];
    }
    else if(prevButton == '3'){
        letter[0] = three_letters[numPresses];
    }
    else if(prevButton == '4'){
        letter[0] = four_letters[numPresses];
    }
    else if(prevButton == '5'){
        letter[0] = five_letters[numPresses];
    }
    else if(prevButton == '6'){
        letter[0] = six_letters[numPresses];
    }
    else if(prevButton == '7'){
        letter[0] = seven_letters[numPresses];
    }
    else if(prevButton == '8'){
        letter[0] = eight_letters[numPresses];
    }
    else if(prevButton == '9'){
        letter[0] = nine_letters[numPresses];
    }
    else{
        return;
    }

    Report("Letter: %c\n", letter[0]);

}

void TimerIntHandler(){
    Timer_IF_InterruptClear(TIMERA1_BASE);
    letters();
    Timer_IF_Stop(TIMERA1_BASE, TIMER_A);
}

static void initTimer(void){
    Timer_IF_Init(PRCM_TIMERA1, TIMERA1_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA1_BASE, TIMER_A, TimerIntHandler);

    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    MAP_TimerLoadSet(TIMERA0_BASE, TIMER_A, MILLISECONDS_TO_TICKS(5));
    MAP_TimerEnable(TIMERA0_BASE, TIMER_A);
}
//****************************************************************************
//
//! Main function
//!
//! \param none
//! 
//!
//! \return None.
//
//****************************************************************************
int main() {
    unsigned long ulStatus;

    BoardInit();
    
    PinMuxConfig();
    
    InitTerm();

    ClearTerm();

    //
    // Register the interrupt handlers
    //
    MAP_GPIOIntRegister(GPIOA0_BASE, GPIOA2IntHandler);

    //
    // Configure rising edge interrupts on SW2 and SW3
    //
    MAP_GPIOIntTypeSet(GPIOA0_BASE, 0x20, GPIO_RISING_EDGE);    // SW2

    ulStatus = MAP_GPIOIntStatus (GPIOA0_BASE, false);
    MAP_GPIOIntClear(GPIOA0_BASE, ulStatus);           // clear interrupts on GPIOA2

    // clear global variables
    SW2_intcount=0;
    SW3_intcount=0;
    SW2_intflag=0;
    SW3_intflag=0;
    time = 0;
    oldTime = -4000001;
    prevButton = '?';
    numPresses = 0;

    MAP_UtilsDelay(800000);
    // Enable SW2 and SW3 interrupts
    MAP_GPIOIntEnable(GPIOA0_BASE, 0x20);
    arrayLoop();
    initTimer();
    myStrings();
    Message("\t\t****************************************************\n\r");
    Message("\t\t\tPush SW3 or SW2 to generate an interrupt\n\r");
    Message("\t\t ****************************************************\n\r");
    Message("\n\n\n\r");


    while (1) {
       while ((SW2_intflag==0) && (SW3_intflag==0)) {;}
        if (SW2_intcount > 38) {
           MAP_GPIOIntDisable(GPIOA0_BASE, 0x20);
            decode();
            SW2_intcount = 0;
           MAP_GPIOIntEnable(GPIOA0_BASE, 0x20);
        }
        SW2_intflag = 0;
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
