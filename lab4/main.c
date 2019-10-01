/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * | SUMMARY: main.c
 * | This program contains 2 main parts (part2_main and part3_main).
 * | Depending on whether PT2 or PT3 is defined, the selected portion
 * | of code will be compiled and run on the board.
 * |
 * | part2_main:
 * | Displays a string on the LCD using C LCD drivers located in  
 * | OneToOneI2CDriver.
 * |
 * | part3_main:
 * | This program contains 2 tests (selectable with the left button).
 * | The first test writes out 0xAA to all SRAM locations and reads 
 * | the data back to make sure all data was correctly written.
 * | The second test is identical to the first test except that it 
 * | writes incrementing values to the SRAM.
 * | 
 * | Pt3 tests also uses C SRAM drivers located in RTDSP_SramSpi.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */ 

#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "RTDSP.h"

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                      CONFIGURATIONS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */
#define QUIZ

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                       PROTOTYPES
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */
void pt2_main();
void pt3_main();
void quiz_main();
void test1();
void test2();
interrupt void ISR_rightButton(void);
interrupt void ISR_leftButton(void);
void interruptInit();

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                        GLOBALS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

Uint16 switchTest = 0x0;
Uint16 writeOrRead = 0x0;
Uint16 recv = 0x0;
Uint16 errorData = 0xDEAD;
Uint32 errorAddr;

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                         MAINS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

/*
 * +-----+-----+-----+-----+-----+-----+-----+
 *
 * +-----+-----+-----+-----+-----+-----+-----+
 */
void main(void)
{
#ifdef PT2
    pt2_main();
#endif

#ifdef PT3
    pt3_main();
#endif

#ifdef QUIZ
    quiz_main();
#endif
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: pt2_main
 * This program outputs the following string to the 16x2
 * LCD:
 *                  "Daniel Hamilton"
 *                  "EEL4511        "
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void pt2_main()
{
    // local variable initializations
    Uint16 slaveAddress = 0x3F;
    float32 sysClkMhz = 10.0f;
    float32 I2CClkKHz = 25.0f; // 25KHz handles LCD delay required to process commands

    // initialization functions
    InitSysCtrl(); // disable watchdog
    I2C_O2O_Master_Init(slaveAddress, sysClkMhz, I2CClkKHz);
    lcdInit();

    char stringA[] = "Daniel Hamilton";
    char stringB[] = "EEL4511";

    // Write strings out to the LCD
    lcdRow1();
    lcdString((Uint16 *)&stringA);
    lcdRow2();
    lcdString((Uint16 *)&stringB);

    // EOP -- infinite loop
    while (1) {DELAY_US(100);}
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY:
 * Fill SRAM with a string.. read the string back and
 * display on the LCD.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void quiz_main()
{
    // local variable initializations ------------------------------------------------
    Uint16 slaveAddress = 0x3F;
    float32 sysClkMhz = 200.0f;
    float32 I2CClkKHz = 25.0f; // 25KHz handles LCD delay required to process commands

    // initialization functions ------------------------------------------------------
    InitSysCtrl(); // disable watchdog
    InitSysPll(XTAL_OSC,IMULT_20,FMULT_0,PLLCLK_BY_2);
    I2C_O2O_Master_Init(slaveAddress, sysClkMhz, I2CClkKHz);
    lcdInit();
    sramSpiInit();

    Uint16 stringLength = 6;
    char stringOut[] = "Hello";
    char stringIn[6] = "     ";

    // fill SRAM with string data ----------------------------------------------------
    for (Uint32 addr = 0x000000; addr < (Uint32)SRAM_MAX_ADDR; addr += stringLength)
    {
        sramVirtualWrite(addr, (Uint16*)&stringOut, stringLength);
    }

    // read string data back from the SRAM and store into memory ---------------------
    sramVirtualRead((Uint32)0x000000, (Uint16*)&stringIn, stringLength);

    // output string to the LCD ------------------------------------------------------
    lcdClear();
    lcdRow1();
    lcdString((Uint16 *)&stringIn);

    while(1);
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: pt3_main
 * If the left button is pushed, switch between test 1
 * and test 2.
 *
 * If the right button is pushed, begin a write on the test
 * currently selected.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void pt3_main()
{
    // local variable initializations ------------------------------------------------
    Uint16 slaveAddress = 0x3F;
    float32 sysClkMhz = 200.0f;
    float32 I2CClkKHz = 25.0f; // 25KHz handles LCD delay required to process commands

    Uint16 boolTest1 = 0x1; // boolean chooses between running test1 and test2

    void (*runTest)(void) = &test1; // points at the test method to be executed.

    // initialization functions ------------------------------------------------------
    InitSysCtrl(); // disable watchdog
    InitSysPll(XTAL_OSC,IMULT_20,FMULT_0,PLLCLK_BY_2);
    I2C_O2O_Master_Init(slaveAddress, sysClkMhz, I2CClkKHz);
    lcdInit();
    sramSpiInit();
    interruptInit();

    while (1)
    {

        // Interrupt causes switch between test1 and test2.
        if (switchTest == 0x1)
        {
            if (boolTest1 == 0x1)
                boolTest1 = 0x0;
            else
                boolTest1 = 0x1;

            switchTest = 0x0; // stops test case from switching constantly
        }

        // Switches which test is being pointed at and updates
        // user with a string "Test 1" or "Test 2".
        if (boolTest1 == 0x1)
        {
            char string[] = "Run Test 1?";
            lcdRow1();
            lcdString((Uint16 *)&string);
            runTest = &test1; // points at test1 method
            DELAY_US(100);
        }
        else
        {
            char string[] = "Run Test 2?";
            lcdRow1();
            lcdString((Uint16 *)&string);
            runTest = &test2; // points at test2 method
            DELAY_US(100);
        }

        // execute the currently selected test when the right
        // button is pressed.
        if (writeOrRead == 0x1)
        {
            writeOrRead = 0x0; // allows test function to use this variable for read.
            (*runTest)(); // call the currently selected test.
        }
    }
}

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                       FUNCTIONS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: test1
 * This test has two parts. The (WRITE) portion of the test
 * writes 0xAA to all SRAM locations. The (READ) portion
 * of the test
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void test1()
{
    lcdClear();
    lcdRow1();
    char string[] = "Test 1 Write..";
    lcdString((Uint16 *)&string);

    // write 0xAA data to the SRAM
    // ----------------------------------------------------
    Uint16 testData = 0xAA;
    for (Uint32 i = 0; i < SRAM_LENGTH; i++)
        sramVirtualWrite(SRAM_MIN_ADDR + i, (Uint16*)&testData, 1);

    lcdClear();
    lcdRow1();
    char newString[] = "Ready for Read?";
    lcdString((Uint16 *)&newString);

    // wait for the read button to be pushed
    // ----------------------------------------------------
    while(1)
    {
        if (writeOrRead == 0x1)
        {
            writeOrRead = 0x0; // clear for main

            lcdClear();
            lcdRow1();
            char string[] = "Test 1 Read..";
            lcdString((Uint16 *)&string);

            // Read a large buffer of values in from the SRAM to test
            // --------------------------------------------
            Uint16 retData[128]; // buffer to reduce read time by reading larger chunks of memory
            Uint16 testLength = 128;
            Uint16 errorBool = 0;

            for (Uint32 i = 0; i < SRAM_MAX_ADDR; i += testLength)
            {
                sramVirtualRead(SRAM_MIN_ADDR + i, (Uint16*)&retData, testLength);

                // test that the returned buffer was equal to the test value.
                for (Uint16 j = 0; j < testLength; j++)
                {
                    if (retData[j] != testData && errorBool == 0)
                    {
                        errorBool = 1;              // only allow the first error store to memory
                        errorData = retData[j];     // save the error data
                        errorAddr = SRAM_MIN_ADDR + i;  // save the error address
                        break;
                    }
                }
            }

            // if the test passed, write the success message to the user
            // otherwise, write the error message to the user.
            // -----------------------------------------------------------------
            lcdClear();
            lcdRow1();

            if (errorBool == 0)
            {
                char string[] = "0xAA OK!";
                lcdString((Uint16 *)&string);
            }
            else
            {
                char string[] = "0xAA Error!";
                lcdString((Uint16 *)&string);
            }

            DELAY_US(2000000); // display message for 2 seconds before returning to main loop
            lcdClear();
            break;
        }
    }
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: test2
 * This test has two parts. The (WRITE) portion of the test
 * writes incrementing values to the SRAM and reads the data
 * back to verify that the writes worked.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void test2()
{
    lcdClear();
    lcdRow1();
    char string[] = "Test 2 Write..";
    lcdString((Uint16 *)&string);

    // write 0xAA data to the SRAM
    // ----------------------------------------------------
    for (Uint32 i = 0; i < SRAM_LENGTH; i++)
    {
        Uint16 testData = (Uint16)i; // truncates i when 32 bit length overflows 16 bit value
        sramVirtualWrite(SRAM_MIN_ADDR + i, (Uint16*)&testData, 1);
    }

    lcdClear();
    lcdRow1();
    char newString[] = "Ready for Read?";
    lcdString((Uint16 *)&newString);

    // wait for the read button to be pushed
    // ----------------------------------------------------
    while(1)
    {
        if (writeOrRead == 0x1)
        {
            writeOrRead = 0x0; // clear for main

            lcdClear();
            lcdRow1();
            char string[] = "Test 2 Read..";
            lcdString((Uint16 *)&string);

            // Read a large buffer of values in from the SRAM to test
            // --------------------------------------------
            Uint16 retData[128]; // buffer to reduce read time by reading larger chunks of memory
            Uint16 testLength = 128;
            Uint16 errorBool = 0;

            // i = base address
            // j = offset
            for (Uint32 i = 0; i < SRAM_MAX_ADDR; i += testLength)
            {
                sramVirtualRead(SRAM_MIN_ADDR + i, (Uint16*)&retData, testLength);

                // test that the returned buffer was equal to the test value.
                for (Uint16 j = 0; j < testLength; j++)
                {
                    if (retData[j] != (Uint16)(i + j) && errorBool == 0)
                    {
                        errorBool = 1;              // only allow the first error store to memory
                        errorData = retData[j];     // save the error data
                        errorAddr = SRAM_MIN_ADDR + i;  // save the error address
                        break;
                    }
                }
            }

            // if the test passed, write the success message to the user
            // otherwise, write the error message to the user.
            // -----------------------------------------------------------------
            lcdClear();
            lcdRow1();

            if (errorBool == 0)
            {
                char string[] = "Inc Test No Err";
                lcdString((Uint16 *)&string);
            }
            else
            {
                char string[] = "Inc Test Error";
                lcdString((Uint16 *)&string);
            }

            DELAY_US(2000000); // display message for 2 seconds before returning to main loop
            lcdClear();
            break;
        }
    }
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: interruptInit
 * This function initializes GPIO interrupts on buttons
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void interruptInit()
{
    DINT; // disable interrupts
    InitPieCtrl(); // initialize PIE control register to default state
    EALLOW;
    IER = 0x0000; // disable CPU interrupts
    IFR = 0x0000; // clear all CPU interrupt flags
    InitPieVectTable(); // intialize PIE vector table with pointers to shell ISR

    // assign the interrupt vectors to the appropriate functions
    EALLOW;
    PieVectTable.XINT1_INT = &ISR_rightButton;
    PieVectTable.XINT2_INT = &ISR_leftButton;

    // enable the interrupts INT1/2
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1; // Enable the PIE block
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1; // Enable PIE Group 1 INT4
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1; // Enable PIE Group 1 INT5
    IER |= M_INT1;                     // Enable CPU INT1

    // configure the GPIO as inputs
    EALLOW;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;         // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 0;          // input
    GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 0;        // XINT1 Synch to SYSCLKOUT only
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;

    GpioCtrlRegs.GPAGMUX2.bit.GPIO16 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;         // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO16 = 0;          // input
    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 0;        // XINT1 Synch to SYSCLKOUT only
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;

    // configure XINT1 and XINT2
    XintRegs.XINT1CR.bit.POLARITY = 1; // Rising edge interrupt
    XintRegs.XINT2CR.bit.POLARITY = 1; // Rising edge interrupt

    XintRegs.XINT1CR.bit.ENABLE = 1; // Enable XINT1
    XintRegs.XINT2CR.bit.ENABLE = 1; // Enable XINT2
    EDIS;

    // assign interrupts INT1/2 to GPIO 14/16
    GPIO_SetupXINT1Gpio(14);
    GPIO_SetupXINT2Gpio(16);

    EINT; // Enable Global Interrupts
}

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                       INTERRUPTS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

interrupt void ISR_rightButton(void)
{
    writeOrRead = 0x1;

    DINT;
    DELAY_US(5000);
    EINT;

    // Acknowledge interrupt for group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void ISR_leftButton(void)
{
    switchTest  = 0x1;

    DINT;
    DELAY_US(5000);
    EINT;

    // Acknowledge interrupt for group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
