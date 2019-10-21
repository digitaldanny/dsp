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
#define I2S

#define SAMPLING_FREQUENCY_A        46875.0f
#define SAMPLING_FREQUENCY_48       46875.0f
#define SAMPLING_FREQUENCY_32       32000.0f
#define SAMPLING_FREQUENCY_8        8000.0f
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
volatile float DataInLeft;
volatile float DataInRight;
volatile int16 DataInMono;
volatile Uint16 LR_received;

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                         MAINS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

#ifdef PT1
/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * This function runs the sound-in/sound-out main of lab5.pt2
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void main()
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
        int16 dataOut; // lower + upper word of the two 32-bit samples
        int16 dataIn;

        // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
        // LEFT - fill up the buffer with top 16 bits of the sample
        // +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
        if (buttons & LEFT_BUTTON)
        {
            clearCodecLeds();

            for (Uint32 i = 0; i < (Uint32)SRAM_LENGTH; i++)
            {
                // wait for a new sample to be received from the codec
                // and wait for the sampling frequency interrupt
                while (LR_received == 0);
                LR_received = 0;

                dataOut = (int16)DataInMono; // float -> int16
                //Interrupt_enable(INT_MCBSPB_RX);

                sramCircularWrite(SRAM_MIN_ADDR + i, (Uint16 *)&dataOut, 1);
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

            for (Uint32 i = 0; i < (Uint32)SRAM_LENGTH; i++)
            {
                // READ -------------------------------------------------------

                // read in the currently stored samples
                sramCircularRead(SRAM_MIN_ADDR + i, (Uint16 *)&dataIn, 1);

                // wait for a new sample to be received from the codec
                while (LR_received == 0);
                LR_received = 0;

                // convert new samples from Uint16 to Uint32
                float newMonoSample = (int16)DataInMono;

                // MODIFY -----------------------------------------------------

                // convert old samples from Uint16's into Uint32's
                float prevMonoSample = (int16)dataIn;

                // add the old and new Uint32 samples together.
                // divide the sum by 2 to average.
                // typecast back to Uint16's to write out
                newMonoSample += prevMonoSample; // sum

                newMonoSample /= 2.0f; // average

                dataOut = (int16)newMonoSample;  // package for output

                // WRITE -----------------------------------------------------

                sramCircularWrite(SRAM_MIN_ADDR + i, (Uint16 *)&dataOut, 1);
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

            for (Uint32 i = 0; i < (Uint32)SRAM_LENGTH; i++)
            {
                // read in the currently stored samples to write to codec
                sramCircularRead(SRAM_MIN_ADDR + i, (Uint16 *)&dataIn, 1);

                // wait for the timer to go off
                while (boolTimer1 == 0);
                boolTimer1 = 0;

                // send out data to just right channel
                McbspbRegs.DXR2.all = dataIn;

                // send out data to just left channel
                McbspbRegs.DXR1.all = dataIn;
            }

            setCodecLeds(LED2);
            DELAY_US(100000);
        }
    }
}
#endif
#ifdef PT2
/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * This function tests the attenuation for various sampling
 * frequencies.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void main()
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
    char stringA[] = "Pt. 2: Sampling";
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

    Uint32 switches = 0;
    Uint32 prevSwitches = 0xDEAD; // dummy value to force frequency change initiallly

    while(1)
    {
        switches = getCodecSwitches();

        // set the Codec sampling frequency and timer frequency
        // if it is not equal to the previous frequency value.
        if (switches != prevSwitches)
        {
            DINT;
            DRTM;

            if (switches == SW0)
            {
                // 8 KHz sampling rate
                command = CLKsampleratecontrol (SR8);

                float period = (1000000.0f/(2.0f*SAMPLING_FREQUENCY_8));
                configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, period);
            }
            else if (switches == SW1)
            {
                // 32 KHz sampling rate
                command = CLKsampleratecontrol (SR44_1);

                float period = (1000000.0f/(2.0f*SAMPLING_FREQUENCY_32));
                configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, period);
            }
            else
            {
                // 46.875 KHz sampling rate
                command = CLKsampleratecontrol (SR48);

                float period = (1000000.0f/(2.0f*SAMPLING_FREQUENCY_48));
                configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, period);
            }

            // send the updated sampling rate to the codec
            BitBangedCodecSpiTransmit (command);
            SmallDelay();

            // start the timer interrupt again
            CPUTimer_startTimer(CPUTIMER1_BASE);

            EINT;
            ERTM;
        }

        prevSwitches = switches;

        // wait for the timer to go off before sending new
        // data out to the headphones.
        while (boolTimer1 == 0);
        boolTimer1 = 0;

        // send out data to just left channel
        McbspbRegs.DXR2.all = DataInLeft;

        // Transfer won't happen until DXR1 is written to
        McbspbRegs.DXR1.all = 0x0000; // newLeftDataOut;

        // send out data to just right channel
        McbspbRegs.DXR2.all = DataInRight;

        // Transfer won't happen until DXR1 is written to
        McbspbRegs.DXR1.all = 0x0000; // newRightDataOut;
    }
}
#endif

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

    DataInRight = (int16)McbspbRegs.DRR2.all;               // Right data in DSP mode
    DataInLeft = (int16)McbspbRegs.DRR1.all;                // Left data in DSP mode
    DataInMono = (int16)((DataInRight + DataInLeft)/2.0f);  //

    // McbspbRegs.DXR2.all = (int16)DataInMono; // ..DEBUGGING..
    // McbspbRegs.DXR1.all = (int16)DataInMono; // ..DEBUGGING..

    LR_received = 1;

    // Probe to check interrupt timing
    GpioDataRegs.GPDDAT.bit.GPIO123 = 1;
    GpioDataRegs.GPDDAT.bit.GPIO123 = 0;

    // acknowledge interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}
