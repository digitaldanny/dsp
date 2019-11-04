/*
 * Using this file requires definining the following variables and functions to be defined in the
 * global section of your source file.
 *
 * volatile i2sSide_t ch_sel;
 * volatile float DataInLeft;
 * volatile float DataInRight;
 * volatile int16 DataInMono;
 * volatile Uint16 LR_received;
 * __interrupt void Mcbsp_RxINTB_ISR(void);
 *
 */

#include "RTDSP_Sampling.h"

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: initCodec
 * This function initializes the following parameters..
 *
 * (extern) ch_sel
 * (extern) DataInLeft
 * (extern) DataInRight
 * (extern) LR_received
 *
 * ~ Sets Codec input / output gains to the max
 * ~ Initializes McBSP interrupt for receiving codec samples (assumes Mcbsp_RxINTB_ISR is available)
 * ~ Clear the codec LEDs
 *
 * FUTURE IMPLEMENTATION:
 * __interrupt void cpuTimer1ISR(void);
 * interrupt void ISR_rightButton(void);
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void initCodec(void)
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
    //Interrupt_register(INT_TIMER1, &cpuTimer1ISR); // set the timer interrupt to point at the cpuTimerISR
    //Interrupt_disable(INT_TIMER1);
    EALLOW;

    sramSpiInit();              // SPI module for SRAM reads and writes
    lcdInit();                  // initialize GPIO for I2C communication to LCD
    lcdDisableCursorBlinking();
    //gpioTimerCheckInit();       // GPIO 123 used to probe timer interrupt
    //timer1Init(TIMER_PERIOD);   // initialize timer1 interrupt on Int13 at 10 Hz
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

    AIC23MaxIOGain(); // amplify the input lines to the max volume

    // Set codec interrupt to 48 KHz
    Uint16 command = CLKsampleratecontrol (SR48);
    BitBangedCodecSpiTransmit (command);
    SmallDelay();

    // 48 KHz output
    //float period = (1000000.0f/(2.0f*SAMPLING_FREQUENCY_48));
    //configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, period);
}
