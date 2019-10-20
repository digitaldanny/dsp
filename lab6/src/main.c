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
#define SAMPLING_FREQUENCY          (2.0f*SAMPLING_FREQUENCY_A)
#define TIMER_PERIOD                (float)(1000000.0f/SAMPLING_FREQUENCY) // in uSeconds

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

#define SW0                         0x1
#define SW1                         0x2
#define SW2                         0x4

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
void pt3_main();
void pt4_main();

void adcA0Init();

void initCPUTimers(void);
void configCPUTimer(uint32_t, float, float);
void gpioTimerCheckInit();

__interrupt void cpuTimer1ISR(void);
__interrupt void Mcbsp_RxINTB_ISR(void);

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                        GLOBALS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

volatile Uint16 boolTimer1;  // allows main to determine if the timer has run or not

volatile i2sSide_t ch_sel;
volatile Uint16 DataInLeft;
volatile Uint16 DataInRight;
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

#ifdef PT3
    pt3_main();
#endif

#ifdef PT4
    pt4_main();
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
    gpioTimerCheckInit();       // GPIO 123 used to probe timer interrupt
    timer1Init(TIMER_PERIOD);   // initialize timer1 interrupt on Int13 at 10 Hz
    initCodecLeds();            // turned off by default
    initCodecButtons();         // set as inputs
    initCodecSwitches();        // set as inputs

    InitSPIA();
    InitBigBangedCodecSPI();
    InitMcBSPb();               // initialize I2S for sound input/output
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

    // amplify the input lines to the max volume
    Uint16 command = lhp_volctl(0x7F);
    BitBangedCodecSpiTransmit (command);
    SmallDelay();

    command = rhp_volctl(0x7F);
    BitBangedCodecSpiTransmit (command);
    SmallDelay();

    while(1)
    {
        Uint16 buttons = getCodecButtons();
        int16 dataOut[2]; // lower + upper word of the two 32-bit samples
        int16 dataIn[2];

        // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
        // LEFT - fill up the buffer with top 16 bits of the sample
        // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
        if (buttons & LEFT_BUTTON)
        {
            clearCodecLeds();

            for (Uint32 i = 0; i < (Uint32)SRAM_LENGTH; i += 2)
            {
                // wait for a new sample to be received from the codec
                // and wait for the sampling frequency interrupt
                while (LR_received == 0);
                LR_received = 0;

                dataOut[0] = DataInLeft & 0xFFFF;
                dataOut[1] = DataInRight & 0xFFFF;

                sramCircularWrite(SRAM_MIN_ADDR + i, (Uint16 *)&dataOut, 2);
                // Interrupt_enable(INT_MCBSPB_RX);
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

            for (Uint32 i = 0; i < (Uint32)SRAM_LENGTH; i += 2)
            {
                // READ -------------------------------------------------------

                // read in the currently stored samples
                sramCircularRead(SRAM_MIN_ADDR + i, (Uint16 *)&dataIn, 2);

                // wait for a new sample to be received from the codec
                while (LR_received == 0);
                LR_received = 0;

                // convert new samples from Uint16 to Uint32
                float newLeftSample = (int16)DataInLeft;
                float newRightSample = (int16)DataInRight;

                // MODIFY -----------------------------------------------------

                // convert old samples from Uint16's into Uint32's
                float prevLeftSample = dataIn[0];
                float prevRightSample = dataIn[1];

                // add the old and new Uint32 samples together.
                // divide the sum by 2 to average.
                // typecast back to Uint16's to write out
                newLeftSample += prevLeftSample; // sum
                newRightSample += prevRightSample; // sum

                newLeftSample /= 2.0f; // average
                newRightSample /= 2.0f; // average

                dataOut[0] = (int16)newLeftSample;  // package for output
                dataOut[1] = (int16)newRightSample; // package for output

                // WRITE -----------------------------------------------------

                sramCircularWrite(SRAM_MIN_ADDR + i, (Uint16 *)&dataOut, 2);
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

            for (Uint32 i = 0; i < (Uint32)SRAM_LENGTH; i += 2)
            {
                // read in the currently stored samples to write to codec
                sramCircularRead(SRAM_MIN_ADDR + i, (Uint16 *)&dataIn, 2);

                // wait for the timer to go off
                while (boolTimer1 == 0);
                boolTimer1 = 0;

                // send out data to just left channel
                McbspbRegs.DXR2.all = dataIn[0];

                // Transfer won't happen until DXR1 is written to
                McbspbRegs.DXR1.all = 0x0000; // newLeftDataOut;

                // send out data to just right channel
                McbspbRegs.DXR2.all = dataIn[1];

                // Transfer won't happen until DXR1 is written to
                McbspbRegs.DXR1.all = 0x0000; // newRightDataOut;
            }

            setCodecLeds(LED2);
            DELAY_US(100000);
        }
    }
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * This function tests the attenuation for various sampling
 * frequencies.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void pt2_main()
{

}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: This function is used to initialize a gpio
 * to be an output. This is used to probe the board and
 * check that the interrupt is triggering at the time
 * expected.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void gpioTimerCheckInit()
{
    EALLOW;

    // timer probe
    GpioDataRegs.GPDDAT.bit.GPIO122 = 1;
    GpioDataRegs.GPDDAT.bit.GPIO122 = 1;
    GpioCtrlRegs.GPDDIR.bit.GPIO122 = GPIO_DIR_MODE_OUT;
    GpioCtrlRegs.GPDGMUX2.bit.GPIO122 = 0x0000;
    GpioCtrlRegs.GPDMUX2.bit.GPIO122 = 0x0000;
    GpioCtrlRegs.GPDPUD.bit.GPIO122 = 0x0000;

    // mcbsp probe
    GpioDataRegs.GPDDAT.bit.GPIO123 = 1;
    GpioDataRegs.GPDDAT.bit.GPIO123 = 1;
    GpioCtrlRegs.GPDDIR.bit.GPIO123 = GPIO_DIR_MODE_OUT;
    GpioCtrlRegs.GPDGMUX2.bit.GPIO123 = 0x0000;
    GpioCtrlRegs.GPDMUX2.bit.GPIO123 = 0x0000;
    GpioCtrlRegs.GPDPUD.bit.GPIO123 = 0x0000;
}

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 * INTERRUPT SERVICE ROUTINES (ISR)
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

__interrupt void cpuTimer1ISR(void)
{
    boolTimer1 = 1;

    // Probe to check interrupt timing..
    GpioDataRegs.GPDDAT.bit.GPIO122 = 1;
    GpioDataRegs.GPDDAT.bit.GPIO122 = 0;
}

/*
 * Codec Audio input interrupt - DSP mode requires 1 interrupt
 */
__interrupt void Mcbsp_RxINTB_ISR(void)
{
    if (ch_sel == LEFT)
    {
        ch_sel = RIGHT;

        // read in left channel
        DataInLeft = McbspbRegs.DRR2.all;
        McbspbRegs.DRR1.all;
    }

    else
    {
        ch_sel = LEFT;

        // read in right channel
        DataInRight = McbspbRegs.DRR2.all;
        McbspbRegs.DRR1.all;

        LR_received = 1;
    }

    // // Left data in DSP mode
    // DataInLeft = McbspbRegs.DRR2.all;
    // McbspbRegs.DRR1.all;
    //
    // // Right data in DSP mode
    // DataInRight = McbspbRegs.DRR2.all;
    // McbspbRegs.DRR1.all;

    // LR_received = 1;

    // Probe to check interrupt timing
    GpioDataRegs.GPDDAT.bit.GPIO123 = 1;
    GpioDataRegs.GPDDAT.bit.GPIO123 = 0;

    // acknowledge interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}
