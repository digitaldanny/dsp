/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * | SUMMARY: main.c                                                 |
 * | This program contains 2 main parts (part1_main and part2_main). |
 * | Depending on whether PT1 or PT2 is defined, the selected portion|
 * | of code will be compiled and run on the board.                  |
 * |                                                                 |
 * | part1_main:                                                     |
 * | This program reads in a voltage on one of the MCU's A/D pins    |
 * | and outputs it on the LCD in the following format:              |
 * |                    "Voltage = X.XX V"                           |
 * |                                                                 |
 * | part2_main:                                                     |
 * | This program echos an input sound out onto the Codec's DAC jack.|
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
#define PT1
#define SAMPLING_FREQUENCY_A        46875.0f
#define SAMPLING_FREQUENCY          SAMPLING_FREQUENCY_A
#define TIMER_PERIOD                (float)1/SAMPLING_FREQUENCY

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                        DEFINES
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */
#define VREF                        3.0f
#define INT_TO_ASCII(VAL)           VAL + 0x30

#define LEFT_BUTTON                 0x4
#define MIDDLE_BUTTON               0x2
#define RIGHT_BUTTON                0x1

#define LED0                        0x01
#define LED1                        0x02
#define LED2                        0x04
#define LED3                        0x08
#define LED4                        0x10
#define LED5                        0x20
#define LED6                        0x40
#define LED7                        0x80
/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                        TYPEDEFS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

typedef enum i2sSide
{
    LEFT,
    RIGHT
} i2sSide_t;


/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                       PROTOTYPES
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */
void pt1_main();
void pt2_main();

void adcA0Init();

void timer1Init(void);
void initCPUTimers(void);
void configCPUTimer(uint32_t, float, float);

__interrupt void cpuTimer1ISR(void);
__interrupt void Mcbsp_RxINTB_ISR(void);

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                        GLOBALS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

volatile Uint16 boolTimer1;  // allows main to determine if the timer has run or not

volatile i2sSide_t ch_sel;
volatile Uint32 DataInLeft;
volatile Uint32 DataInRight;
volatile Uint16 LR_received;

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                         MAINS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * User can select between parts of the lab to compile and
 * run using the #define located under CONFIGURATIONS.
 *
 * #define PT1 = pt1_main (voltmeter)
 * #define Pt2 = pt2_main (codec sound in/out)
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void main(void)
{
#ifdef PT1
    pt1_main();
#endif

#ifdef PT2
    pt2_main();
#endif
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * This function runs the sound-in/sound-out main of lab5.pt2
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void pt1_main()
{
    // global variable initialization
    ch_sel      = LEFT;
    DataInLeft  = 0;
    DataInRight = 0;
    LR_received = 0;

    InitSysCtrl();              // disable watchdog
    Interrupt_initModule();     // initialize PIE and clear PIE registers.
    Interrupt_initVectorTable(); // initializes the PIE vector table with pointers to the shell ISRs.
    Interrupt_register(INT_TIMER1, &cpuTimer1ISR); // set the timer interrupt to point at the cpuTimerISR
    Interrupt_disable(INT_TIMER1);
    EALLOW;

    sramSpiInit();              // SPI module for SRAM reads and writes
    lcdInit();                  // initialize GPIO for I2C communication to LCD
    timer1Init(TIMER_PERIOD);   // initialize timer1 interrupt on Int13 at 10 Hz
    initCodecLeds();            // turned off by default
    initCodecButtons();         // set as inputs
    initCodecSwitches();        // set as inputs

    InitMcBSPb();               // initialize I2S for sound input/output
    InitSPIA();
    InitBigBangedCodecSPI();
    Interrupt_enable(INT_MCBSPB_RX); //INT_MCBSPB_RX);
    Interrupt_register(INT_MCBSPB_RX, &Mcbsp_RxINTB_ISR); // set I2S RX interrupt to ISR address

    InitAIC23();                // initialize Codec's command registers

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    // Initialize LCD with output string format
    char stringA[] = "Pt. 1: Mixing";
    lcdRow1();
    lcdString((Uint16 *)&stringA);
    lcdDisableCursorBlinking();

    clearCodecLeds();

    while(1)
    {
        Uint16 buttons = getCodecButtons();
        Uint16 dataOut[4]; // lower + upper word of the two 32-bit samples
        Uint16 dataIn[4];

        // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
        // LEFT - fill up the buffer
        // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
        if (buttons & LEFT_BUTTON)
        {
            clearCodecLeds();

            for (Uint32 i = 0; i < (Uint32)SRAM_LENGTH; i += 4)
            {
                // wait for a new sample to be received from the codec
                // and wait for the sampling frequency interrupt
                while (LR_received == 0);

                // write out 2 32-bit samples (left and right)
                dataOut[0] = (DataInLeft >> 0 ) & 0xFFFF;
                dataOut[1] = (DataInLeft >> 16) & 0xFFFF;
                dataOut[2] = (DataInRight >> 0 ) & 0xFFFF;
                dataOut[3] = (DataInRight >> 16) & 0xFFFF;

                sramVirtualWrite(SRAM_MIN_ADDR + i, (Uint16 *)&dataOut, 4);
                LR_received = 0;
                boolTimer1 = 0;

                Interrupt_enable(INT_MCBSPB_RX);
            }

            setCodecLeds(LED0);
            DELAY_US(100000);
        }

        // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
        // MIDDLE - average the buffer with new samples
        // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
        else if (buttons & MIDDLE_BUTTON)
        {
            clearCodecLeds();

            for (Uint32 i = 0; i < (Uint32)SRAM_LENGTH; i += 4)
            {
                // wait for a new sample to be received from the codec
                while (LR_received == 0);

                // READ -------------------------------------------------------

                // read in the currently stored samples
                sramVirtualRead(SRAM_MIN_ADDR + i, (Uint16 *)&dataIn, 4);

                // MODIFY -----------------------------------------------------

                // convert old samples from 2 Uint16's into Uint64's
                Uint64 prevLeftSample = ((Uint64)dataIn[1] << 16) | dataIn[0];
                Uint64 prevRightSample = ((Uint64)dataIn[3] << 16) | dataIn[2];

                // convert new samples from Uint32 to Uint64
                Uint64 newLeftSample = (Uint64)DataInLeft;
                Uint64 newRightSample = (Uint64)DataInRight;

                // add the old and new Uint64 samples together.
                // divide the sum by 2 to average.
                // typecast back to Uint16's to write out
                newLeftSample += prevLeftSample; // sum
                newRightSample += prevRightSample; // sum

                newLeftSample >>= 1; // average
                newRightSample >>= 1; // average

                dataOut[0] = (newLeftSample >> 0 ) & 0xFFFF;  // package for output
                dataOut[1] = (newLeftSample >> 16) & 0xFFFF;  // package for output
                dataOut[2] = (newRightSample >> 0 ) & 0xFFFF; // package for output
                dataOut[3] = (newRightSample >> 16) & 0xFFFF; // package for output

                // WRITE -----------------------------------------------------

                sramVirtualWrite(SRAM_MIN_ADDR + i, (Uint16 *)&dataOut, 4);
                LR_received = 0;
                boolTimer1 = 0;
                Interrupt_enable(INT_MCBSPB_RX);
            }

            setCodecLeds(LED1);
            DELAY_US(100000);
        }

        // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
        // RIGHT - send the averaged data to the codec output
        // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
        else if (buttons & RIGHT_BUTTON)
        {
            clearCodecLeds();

            for (Uint32 i = 0; i < (Uint32)SRAM_LENGTH; i += 4)
            {
                // read in the currently stored samples to write to codec
                sramVirtualRead(SRAM_MIN_ADDR + i, (Uint16 *)&dataIn, 4);

                Uint32 newLeftDataOut = ((Uint32)dataIn[1] << 16) | dataIn[0];
                Uint32 newRightDataOut = ((Uint32)dataIn[3] << 16) | dataIn[2];

                // wait for the timer to go off
                while (boolTimer1 == 0);
                boolTimer1 = 0;

                // send out data to just left channel
                McbspbRegs.DXR2.all = newLeftDataOut >> 16;

                // Transfer won't happen until DXR1 is written to
                McbspbRegs.DXR1.all = newLeftDataOut;

                // send out data to just right channel
                McbspbRegs.DXR2.all = newRightDataOut >> 16;

                // Transfer won't happen until DXR1 is written to
                McbspbRegs.DXR1.all = newRightDataOut;
            }

            setCodecLeds(LED2);
            DELAY_US(100000);
        }
    }
}

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 * INTERRUPT SERVICE ROUTINES (ISR)
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

__interrupt void cpuTimer1ISR(void)
{
    boolTimer1 = 1;
}

__interrupt void Mcbsp_RxINTB_ISR(void)
{
    if (ch_sel == LEFT)
    {
        ch_sel = RIGHT;

        // read in left channel
        DataInLeft = McbspbRegs.DRR2.all;
        DataInLeft <<= 16;
        DataInLeft |= McbspbRegs.DRR1.all;
    }


    else
    {
        ch_sel = LEFT;

        // read in right channel
        DataInRight = McbspbRegs.DRR2.all;
        DataInRight <<= 16;
        DataInRight |= McbspbRegs.DRR1.all;

        LR_received = 1;
        Interrupt_disable(INT_MCBSPB_RX);
    }

    // acknowledge interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}
