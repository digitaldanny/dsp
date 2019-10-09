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
#define EX_ADC_RESOLUTION
/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                       PROTOTYPES
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */
void pt1_main();
void pt2_main();

void adcA0Init();

__interrupt void cpuTimer1ISR(void);
void timer1Init(void);
void initCPUTimers(void);
void configCPUTimer(uint32_t, float, float);

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                        GLOBALS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */
Uint16 adcAResult;  // used to store the ADC results
Uint16 wr[5];       // stores the ASCII ADC results
Uint16 boolTimer1;  // allows main to determine if the timer has run or not

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
    lcdInit();                  // initialize GPIO for I2C communication to LCD
    Interrupt_initModule();     // initialize PIE and clear PIE registers.
    Interrupt_initVectorTable(); // initializes the PIE vector table with pointers to the shell ISRs.
    Interrupt_register(INT_TIMER1, &cpuTimer1ISR); // set the timer interrupt to point at the cpuTimerISR

    //EALLOW;
    //AdcSetMode(ADC_ADCA, ADC_BITRESOLUTION_16BIT, ADC_SIGNALMODE_DIFFERENTIAL);

    EALLOW;
    adcA0Init();
    // initADCs();                 // initialize ADC module A
    // initADCSOCs();              // initialize conversions on ADC module A, pin A0
    timer1Init();               // initialize timer1 interrupt on Int13 at 10 Hz

    // enable global interrupt and realtime interrupt
    EINT;
    ERTM;

    // Initialize LCD with output string format
    char stringA[] = "Voltage = X.XXV";
    lcdRow1();
    lcdString((Uint16 *)&stringA);

    while (1)
    {
        // force ADC to convert on A0
        // ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER0);
        AdcaRegs.ADCSOCFRC1.all = 0x000F; // set SOC flags for SOC0 to SOC3

        if (boolTimer1)
        {
            boolTimer1 = 0; // clear the timer bool

            // Wait for ADCA to complete then clear the flag
            // while(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1) == false);
            // ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

            // store the ADC read results before converting to an ASCII character
            // adcAResult = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
            while ();
            adcAResult =

            wr[0] = '1'; // ascii character of 'i'
            wr[1] = '.';
            wr[2] = '2';
            wr[3] = '3';
            wr[4] = '\0';

            lcdCursor(10); // offset t.o the X.XX decimal voltage value
            lcdString((Uint16*)&wr);
            DELAY_US(10000);
        }
    }
}

void adcA0Init()
{
    EALLOW;
    // AdcSetMode(ADC_ADCA, ADC_BITRESOLUTION_16BIT, ADC_SIGNALMODE_DIFFERENTIAL);
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN10, 10U);
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER5);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    EDIS;
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
