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
#define PT2
#define SAMPLES_PER_MEASUREMENT     16

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                        DEFINES
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */
#define VREF                        3.0f
#define INT_TO_ASCII(VAL)           VAL + 0x30

#define FORCE_ADC_CONVERSION        AdcaRegs.ADCSOCFRC1.bit.SOC0 = 1
#define WAIT_FOR_ADC_CONVERSION     while (AdcaRegs.ADCCTL1.bit.ADCBSY == 1)
#define CLEAR_ADC_FLAG              AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 0x0001

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

volatile Uint16 adcAResult;  // used to store the ADC results
volatile float adcAResultVector[SAMPLES_PER_MEASUREMENT]; // buffer used to average the samples
volatile float adcFloat;     // used to store ADC floating point conversion
volatile Uint16 wr[5];       // stores the ASCII ADC results
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
 * This function runs the Voltmeter main of lab5.pt1
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void pt1_main()
{
    InitSysCtrl();              // disable watchdog
    Interrupt_initModule();     // initialize PIE and clear PIE registers.
    Interrupt_initVectorTable(); // initializes the PIE vector table with pointers to the shell ISRs.
    Interrupt_register(INT_TIMER1, &cpuTimer1ISR); // set the timer interrupt to point at the cpuTimerISR

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    lcdInit();                  // initialize GPIO for I2C communication to LCD
    timer1Init();               // initialize timer1 interrupt on Int13 at 10 Hz
    adcA0Init();                // initialize ADC A0

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    // Initialize LCD with output string format
    lcdDisableCursorBlinking();
    char stringA[] = "Voltage = X.XXV";
    lcdRow1();
    lcdString((Uint16 *)&stringA);

    while (1)
    {

        if (boolTimer1)
        {
            boolTimer1 = 0; // clear the timer bool

            // increase the sample accuracy with a large buffer average
            for (int i = 0; i < SAMPLES_PER_MEASUREMENT; i++)
            {
                FORCE_ADC_CONVERSION;     // force ADC to convert on A0

                WAIT_FOR_ADC_CONVERSION;  // wait for first conversion to complete

                CLEAR_ADC_FLAG;           // clear the interrupt flag generated by the conversion

                adcAResult = AdcaResultRegs.ADCRESULT0; // save results of the conversion
                adcFloat = (VREF * (float)adcAResult) / 4096; // digital to voltage conversion reason.

                adcAResultVector[i] = adcFloat;
            }

            // average all the samples taken
            adcFloat = 0.0f;
            for (int i = 0; i < SAMPLES_PER_MEASUREMENT; i++)
                adcFloat += adcAResultVector[i];
            adcFloat /= (float)SAMPLES_PER_MEASUREMENT;

            float ones         = adcFloat;
            float tenths       = (adcFloat - (float)((Uint16)ones)) * 10;
            float hundredths   = (tenths - (float)((Uint16)tenths)) * 10;

            // Convert voltage to characters and store to the LCD
            wr[0] = INT_TO_ASCII((Uint16)ones); // 1's place (Ex. [1].23)
            wr[1] = '.';
            wr[2] = INT_TO_ASCII((Uint16)tenths); // 10th's place (Ex. 1.[2]3)
            wr[3] = INT_TO_ASCII((Uint16)hundredths); // 10th's place (Ex. 1.2[3])
            wr[4] = '\0';

            lcdCursor(10); // offset t.o the X.XX decimal voltage value
            lcdString((Uint16*)&wr);
            DELAY_US(10000);
        }
    }
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * This function runs the sound-in/sound-out main of lab5.pt2
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void pt2_main()
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

    EALLOW;

    lcdInit();                  // initialize GPIO for I2C communication to LCD
    timer1Init();               // initialize timer1 interrupt on Int13 at 10 Hz
    adcA0Init();                // initialize ADC A

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
    char stringA[] = "Sound I/O";
    lcdRow1();
    lcdString((Uint16 *)&stringA);
    lcdDisableCursorBlinking();

    while(1)
    {

        if (LR_received)
        {
            LR_received = 0;
        }
    }
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: adcA0Init
 * This function sets single-ended 12-bit ADC conversions on
 * ADCIN A0 for 200MHz system clock.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void adcA0Init()
{
    EALLOW;
    AdcaRegs.ADCCTL2.bit.RESOLUTION = 0; // 12 bit resolution
    AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0; // single ended
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;   // divide by 5

    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1; // power up the circuitry + enable ADC
    DELAY_US(1000);

    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0; // software trigger only
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0; // single ended channel 0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 15; // #clock cycles for sample time
    DELAY_US(1000);
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

        // send out data to just left channel
        McbspbRegs.DXR2.all = DataInLeft >> 16;

        // Transfer won't happen until DXR1 is written to
        McbspbRegs.DXR1.all = DataInLeft;
    }


    else
    {
        ch_sel = LEFT;

        // read in right channel
        DataInRight = McbspbRegs.DRR2.all;
        DataInRight <<= 16;
        DataInRight |= McbspbRegs.DRR1.all;

        // send out data to just right channel
        McbspbRegs.DXR2.all = DataInRight >> 16;

        // Transfer won't happen until DXR1 is written to
        McbspbRegs.DXR1.all = DataInRight;

        LR_received = 1;
    }

    // acknowledge interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}
