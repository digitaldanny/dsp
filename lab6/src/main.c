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
#define PT4
#define DECIMATION_SIZE             2

#define SAMPLING_FREQUENCY_A        46875.0f
#define SAMPLING_FREQUENCY_48       46875.0f
#define SAMPLING_FREQUENCY_32       32000.0f
#define SAMPLING_FREQUENCY_8        8000.0f
#define SAMPLING_FREQUENCY          (2.0f*SAMPLING_FREQUENCY_A)
#define TIMER_PERIOD                (float)(1000000.0f/SAMPLING_FREQUENCY) // in uSeconds
#define REVERB_INCREMENT_TIME       0.01f // in seconds

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

#define lcdClearBottomRow()         char clearString[] = "                "; \
                                    lcdRow2(); \
                                    lcdString((Uint16 *)&clearString) \

#define lcdClearTopRow()            char clearString[] = "                "; \
                                    lcdRow1(); \
                                    lcdString((Uint16 *)&clearString) \

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

void pt3_io(void);
void pt3_interpolation(void);
void pt3_decimation(void);

void reverb(Uint16 p, float a, Uint32 index);
void echo(Uint16 p, float a, Uint32 index);
int16 avg2(int16 a, int16 b);

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
        McbspbRegs.DXR2.all = DataInMono;

        // Transfer won't happen until DXR1 is written to
        McbspbRegs.DXR1.all = DataInMono;
    }
}
#endif
#ifdef PT3
/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * This function implements interpolation and decimation.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void main()
{
    // global variable initialization
    ch_sel      = LEFT;
    DataInLeft  = 0;
    DataInRight = 0;
    LR_received = 0;

    DINT;  // Enable Global interrupt INTM
    DRTM;  // Enable Global realtime interrupt DBGM

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
    Interrupt_enable(INT_MCBSPB_RX); //INT_MCBSPB_RX);
    Interrupt_register(INT_MCBSPB_RX, &Mcbsp_RxINTB_ISR); // set I2S RX interrupt to ISR address
    InitMcBSPb();               // initialize I2S for sound input/output

    InitAIC23();                // initialize Codec's command registers

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    clearCodecLeds();

    // amplify the input lines to the max volume
    Uint16 command = lhp_volctl(0x7F);
    BitBangedCodecSpiTransmit (command);
    SmallDelay();

    command = rhp_volctl(0x7F);
    BitBangedCodecSpiTransmit (command);
    SmallDelay();

    lcdDisableCursorBlinking();

    Uint16 buttons = 0;
    Uint32 switches = 0;
    Uint32 prevSwitches = 0xDEAD; // dummy value to force frequency change initially

    while(1)
    {
        switches = getCodecSwitches();
        buttons = getCodecButtons();

        // +-----+-----+-----+-----+ FREQUENCY SELECTION +-----+-----+-----+-----+
        // set the Codec sampling frequency and timer frequency
        // if it is not equal to the previous frequency value.
        if (switches != prevSwitches)
        {
            DINT;
            DRTM;

            lcdClearBottomRow();

            // input, output frequency select
            switch (switches)
            {
                case 0x00:
                {
                    // 48K INPUTS -------------------------------------------------

                    // 48 KHz
                    command = CLKsampleratecontrol (SR48);

                    // 48 KHz output
                    float period = (1000000.0f/(2.0f*SAMPLING_FREQUENCY_48));
                    configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, period);

                    // update frequencies on LCD
                    char freqString[] = "I: 48K - O: 48K";
                    lcdRow2();
                    lcdString((Uint16 *)&freqString);
                    break;
                }

                case 0x01:
                {
                    // 48 KHz sampling rate
                    command = CLKsampleratecontrol (SR48);

                    // 32 KHz sampling rate
                    float period = (1000000.0f/(2.0f*SAMPLING_FREQUENCY_32));
                    configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, period);

                    // update frequencies on LCD
                    char freqString[] = "I: 48K - O: 32K";
                    lcdRow2();
                    lcdString((Uint16 *)&freqString);
                    break;
                }

                case 0x02:
                {
                    // 48 KHz sampling rate
                    command = CLKsampleratecontrol (SR48);

                    // 8 KHz sampling rate
                    float period = (1000000.0f/(2.0f*SAMPLING_FREQUENCY_8));
                    configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, period);

                    // update frequencies on LCD
                    char freqString[] = "I: 48K - O: 8K";
                    lcdRow2();
                    lcdString((Uint16 *)&freqString);
                    break;
                }

                case 0x03:
                {
                    // 32K INPUTS -------------------------------------------------

                    // 32 KHz sampling rate
                    command = CLKsampleratecontrol (SR32);

                    // 48 KHz sampling rate
                    float period = (1000000.0f/(2.0f*SAMPLING_FREQUENCY_48));
                    configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, period);

                    // update frequencies on LCD
                    char freqString[] = "I: 32K - O: 48K";
                    lcdRow2();
                    lcdString((Uint16 *)&freqString);
                    break;
                }

                case 0x04:
                {
                    // 32 KHz sampling rate
                    command = CLKsampleratecontrol (SR32);

                    // 32 KHz sampling rate
                    float period = (1000000.0f/(2.0f*SAMPLING_FREQUENCY_32));
                    configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, period);

                    // update frequencies on LCD
                    char freqString[] = "I: 32K - O: 32K";
                    lcdRow2();
                    lcdString((Uint16 *)&freqString);
                    break;
                }

                case 0x05:
                {
                    // 32 KHz sampling rate
                    command = CLKsampleratecontrol (SR32);

                    // 48 KHz sampling rate
                    float period = (1000000.0f/(2.0f*SAMPLING_FREQUENCY_8));
                    configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, period);

                    // update frequencies on LCD
                    char freqString[] = "I: 32K - O: 8K";
                    lcdRow2();
                    lcdString((Uint16 *)&freqString);
                    break;
                }

                case 0x06:
                {
                    // 8K INPUTS -------------------------------------------------

                    // 8 KHz sampling rate
                    command = CLKsampleratecontrol (SR8);

                    // 48 KHz sampling rate
                    float period = (1000000.0f/(2.0f*SAMPLING_FREQUENCY_48));
                    configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, period);

                    // update frequencies on LCD
                    char freqString[] = "I: 8K - O: 48K";
                    lcdRow2();
                    lcdString((Uint16 *)&freqString);
                    break;
                }

                case 0x07:
                {
                    // 8 KHz sampling rate
                    command = CLKsampleratecontrol (SR8);

                    // 48 KHz sampling rate
                    float period = (1000000.0f/(2.0f*SAMPLING_FREQUENCY_32));
                    configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, period);

                    // update frequencies on LCD
                    char freqString[] = "I: 8K - O: 32K";
                    lcdRow2();
                    lcdString((Uint16 *)&freqString);
                    break;
                }

                default:
                {
                    // 8 KHz sampling rate
                    command = CLKsampleratecontrol (SR8);

                    // 48 KHz sampling rate
                    float period = (1000000.0f/(2.0f*SAMPLING_FREQUENCY_8));
                    configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, period);

                    // update frequencies on LCD
                    char freqString[] = "I: 8K - O: 8K";
                    lcdRow2();
                    lcdString((Uint16 *)&freqString);
                    break;
                }
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

        // +-----+-----+-----+-----+ TEST SELECTION +-----+-----+-----+-----+

        if (buttons == LEFT_BUTTON)
        {
            lcdClearTopRow();

            // update test running on LCD
            char testString[] = "I/O";
            lcdRow1();
            lcdString((Uint16 *)&testString);

            pt3_io();
        }
        else if (buttons == MIDDLE_BUTTON)
        {
            lcdClearTopRow();

            // update test running on LCD
            char testString[] = "Interpolation";
            lcdRow1();
            lcdString((Uint16 *)&testString);

            pt3_interpolation();
        }
        else if (buttons == RIGHT_BUTTON)
        {
            lcdClearTopRow();

            // update test running on LCD
            char testString[] = "Decimation";
            lcdRow1();
            lcdString((Uint16 *)&testString);

            pt3_decimation();
        }
    }
}
#endif
#ifdef PT4
/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * This function implements interpolation and decimation.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void main()
{
    // global variable initialization
    ch_sel      = LEFT;
    DataInLeft  = 0;
    DataInRight = 0;
    LR_received = 0;

    DINT;  // Enable Global interrupt INTM
    DRTM;  // Enable Global realtime interrupt DBGM

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
    Interrupt_enable(INT_MCBSPB_RX); //INT_MCBSPB_RX);
    Interrupt_register(INT_MCBSPB_RX, &Mcbsp_RxINTB_ISR); // set I2S RX interrupt to ISR address
    InitMcBSPb();               // initialize I2S for sound input/output

    InitAIC23();                // initialize Codec's command registers

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    clearCodecLeds();

    // amplify the input lines to the max volume
    Uint16 command = lhp_volctl(0x7F);
    BitBangedCodecSpiTransmit (command);
    SmallDelay();

    command = rhp_volctl(0x7F);
    BitBangedCodecSpiTransmit (command);
    SmallDelay();

    // command = aaudpath(); // enable the microphone
    // BitBangedCodecSpiTransmit (command);
    // SmallDelay();
    //
    // command = fullpowerup(); // turn on the mic for testing
    // BitBangedCodecSpiTransmit (command);
    // SmallDelay();

    lcdDisableCursorBlinking();

    // 48 KHz
    command = CLKsampleratecontrol (SR48);

    // 48 KHz output
    float period = (1000000.0f/(2.0f*SAMPLING_FREQUENCY_48));
    configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, period);

    // update frequencies on LCD
    char string[] = "Part 4";
    lcdRow1();
    lcdString((Uint16 *)&string);

    Uint16 p;
    float a = 0.4f;
    Uint32 index = 0;

    Uint16 buttons = 0;
    Uint32 switches = 0;
    Uint32 prevSwitches = 0xDEAD; // dummy value to force frequency change initially
    void (*effect)(Uint16, float, Uint32) = &reverb;

    // initialize the buffer by filling with values..
    for (Uint32 i = 0; i < (Uint32)SRAM_LENGTH; i++)
    {
        while (LR_received == 0);
        LR_received = 0;
        Uint16 dataOut = (int16)DataInMono; // float -> int16
        sramCircularWrite(SRAM_MIN_ADDR + i, (Uint16 *)&dataOut, 1);
    }

    while(1)
    {
        switches = getCodecSwitches();
        buttons = getCodecButtons();

        // increase sample delay in increments of 10 ms
        p = (Uint16)((float)switches*(float)REVERB_INCREMENT_TIME*SAMPLING_FREQUENCY);

        // buttons determine which effect the effect point at.
        if (buttons == RIGHT_BUTTON)
        {
            lcdClear();
            char s[] = "Reverb";
            lcdRow1();
            lcdString((Uint16 *)&s);
            effect = &reverb;
        }
        else if (buttons & LEFT_BUTTON)
        {
            lcdClear();
            char s[] = "Echo";
            lcdRow1();
            lcdString((Uint16 *)&s);
            effect = &echo;
        }

        // call the effect and increment the buffer address
        (*effect)(p, a, index);
        index++;
    }
}
#endif

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+
 * FUNCTIONS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: avg2
 * This function averages together 2 int16 samples and
 * returns an int16 result.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
int16 avg2(int16 a, int16 b)
{
    float af = (int16)a;
    float bf = (int16)b;
    af += bf;               // sum
    af /= 2.0f;             // average
    return (int16)a;        // package for output
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: reverb
 * Implements the following function..
 * y[n] = (1-a)x[n] + ax[n-p]
 *
 * GLOBALS:
 * DataInMono - int16 sample from the McBSP codec isr.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void reverb(Uint16 p, float a, Uint32 index)
{
    // Get the new sample to output to the codec ----------------
    while (LR_received == 0);
    LR_received = 0;

    // save the new sample in SRAM to be used by the reverb later
    int16 sample = DataInMono;     // x[n]
    sramCircularWrite(SRAM_MIN_ADDR + index, (Uint16 *)&sample, 1);

    // (1-a)x[n] (new sample)
    int16 xn = (int16)((1.0f - a)*(float)((int16)sample));

    // x[n-p] sample from buffer (sample = old_sample)
    sramCircularRead(SRAM_MIN_ADDR + index - p, (Uint16 *)&sample, 1);

    // a*x[n-p] (old sample)
    int16 xn_p = (int16)(a*((float)sample));

    // final reverb formula
    xn += xn_p;

    McbspbRegs.DXR2.all = xn;   // tx
    McbspbRegs.DXR1.all = xn;   // tx
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: echo
 * Implements the following function..
 * y[n] = (1-a)x[n] + ay[n-p]
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void echo(Uint16 p, float a, Uint32 index)
{
    McbspbRegs.DXR2.all = DataInMono;   // tx
    McbspbRegs.DXR1.all = DataInMono;   // tx
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: Sample at a selected input frequency, store,
 * and output at the other selected frequency.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void pt3_io(void)
{
    int16 dataOut;
    int16 dataIn;

    clearCodecLeds();

    // fill the buffer with samples at selected frequency
    for (Uint32 i = 0; i < (Uint32)SRAM_LENGTH; i++)
    {
        // wait for a new sample to be received from the codec
        // and wait for the sampling frequency interrupt
        while (LR_received == 0);
        LR_received = 0;

        dataOut = (int16)DataInMono; // float -> int16

        sramCircularWrite(SRAM_MIN_ADDR + i, (Uint16 *)&dataOut, 1);
    }

    setCodecLeds(LED0);

    // use the output frequency for outputting the samples
    // fill the buffer with samples at selected frequency
    for (Uint32 i = 0; i < (Uint32)SRAM_LENGTH; i++)
    {
        sramCircularRead(SRAM_MIN_ADDR + i, (Uint16 *)&dataIn, 1);

        // wait for a new sample to be received from the codec
        // and wait for the sampling frequency interrupt
        while (boolTimer1 == 0);
        boolTimer1 = 0;

        // send out data to just right channel
        McbspbRegs.DXR2.all = dataIn;

        // Transfer data to the left channel
        McbspbRegs.DXR1.all = dataIn;
    }

    setCodecLeds(LED1);
    DELAY_US(100000);
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: Sample at a selected input frequency, store,
 * interpolate, and output at the other selected frequency.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void pt3_interpolation(void)
{
    int16 dataOut[2];
    float prevDataOut;
    int16 dataIn;

    clearCodecLeds();

    // fill the buffer with samples at selected frequency
    for (Uint32 i = 0; i < (Uint32)SRAM_LENGTH; i+=2)
    {
        // wait for a new sample to be received from the codec
        // and wait for the sampling frequency interrupt
        while (LR_received == 0);
        LR_received = 0;

        // store the new sample value
        dataOut[1] = (int16)DataInMono;

        // interpolate the previous value and the new sample
        dataOut[0] = (int16)(((float)dataOut[1] + prevDataOut)/2.0f);

        // assign the new sample to be the previous sample
        prevDataOut = (float)dataOut[1];

        sramCircularWrite(SRAM_MIN_ADDR + i, (Uint16 *)&dataOut, 2);
    }

    setCodecLeds(LED0);

    // use the output frequency for outputting the samples
    // fill the buffer with samples at selected frequency
    for (Uint32 i = 0; i < (Uint32)SRAM_LENGTH; i++)
    {
        sramCircularRead(SRAM_MIN_ADDR + i, (Uint16 *)&dataIn, 1);

        // wait for a new sample to be received from the codec
        // and wait for the sampling frequency interrupt
        while (boolTimer1 == 0);
        boolTimer1 = 0;

        // send out data to just right channel
        McbspbRegs.DXR2.all = dataIn;

        // Transfer data to the left channel
        McbspbRegs.DXR1.all = dataIn;
    }

    setCodecLeds(LED1);
    DELAY_US(100000);
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: Sample at a selected input frequency, store every
 * nth sample, and output at another frequency.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void pt3_decimation(void)
{
    int16 dataOut;
    int16 dataIn;

    clearCodecLeds();

    // fill the buffer with samples at selected frequency until
    // j reaches the final address of the SRAM
    Uint32 j = 0;
    for (Uint64 i = 0; j < (Uint32)SRAM_LENGTH; i++)
    {
        // wait for a new sample to be received from the codec
        // and wait for the sampling frequency interrupt
        while (LR_received == 0);
        LR_received = 0;

        // skip every sample until it reaches the DECIMATION_SIZE
        if (i % DECIMATION_SIZE > 0)
        {
            continue;
        }

        dataOut = (int16)DataInMono; // float -> int16

        sramCircularWrite(SRAM_MIN_ADDR + j, (Uint16 *)&dataOut, 1);
        j++;
    }

    setCodecLeds(LED0);

    // use the output frequency for outputting the samples
    // fill the buffer with samples at selected frequency
    for (Uint32 i = 0; i < (Uint32)SRAM_LENGTH; i++)
    {
        sramCircularRead(SRAM_MIN_ADDR + i, (Uint16 *)&dataIn, 1);

        // wait for a new sample to be received from the codec
        // and wait for the sampling frequency interrupt
        while (boolTimer1 == 0);
        boolTimer1 = 0;

        // send out data to just right channel
        McbspbRegs.DXR2.all = dataIn;

        // Transfer data to the left channel
        McbspbRegs.DXR1.all = dataIn;
    }

    setCodecLeds(LED1);
    DELAY_US(100000);
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
