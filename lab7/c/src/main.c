/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * | SUMMARY: main.c                                                 |
 * | This program contains 2 main parts (part1_main and part2_main). |
 * | Depending on whether PT1 or PT2 is defined, the selected portion|
 * | of code will be compiled and run on the board.                  |
 * |                                                                 |
 * | part1_main: Low Pass FIR Filter
 * | part2_main: High Pass FIR Filter
 * | part3_main: Band Pass IIR Filter
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */

#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "RTDSP.h"
#include <math.h>

//#include "examples_setup.h"
#include <filter.h>

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                      CONFIGURATIONS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */
#define PT1

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
#define LPF_SIZE                    42
#define HPF_SIZE                    31

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
void interruptInit();

__interrupt void cpuTimer1ISR(void);
__interrupt void Mcbsp_RxINTB_ISR(void);
interrupt void ISR_rightButton(void);

void hpf(void);
void lpf(void);
void bpf(void);
int16 avg2(int16 a, int16 b);

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                        GLOBALS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

#define SIGNAL_LENGTH   100

// IIR BEGIN----------------------------------------------------------
#define IIR32_LPF_COEFF {\
            -2071624,4153705,235,471,235,\
            -2077764,4166241,10204,20408,10204,\
            -2084877,4159918,720913,-1441826,720913,\
            -2090775,4181980,761219546,-1522439093,761219546}

#define IIR32_LPF_ISF   1784
#define IIR32_LPF_NBIQ  4
#define IIR32_LPF_QFMAT 21

IIR5BIQ32  iir = IIR5BIQ32_DEFAULTS;
int32_t dbuffer[2*IIR32_LPF_NBIQ];
int32_t sigIn[SIGNAL_LENGTH];
int32_t sigOut[SIGNAL_LENGTH];

int32_t xn = 0, yn = 0;
float RadStep = 0.1963495408494f;
float Rad = 0.0f;
uint32_t i = 0;
const int32_t coeff[5*IIR32_LPF_NBIQ] = IIR32_LPF_COEFF;
// IIR END ----------------------------------------------------------

volatile Uint16 boolTimer1;  // allows main to determine if the timer has run or not

volatile i2sSide_t ch_sel;
volatile float DataInLeft;
volatile float DataInRight;
volatile int16 DataInMono;
volatile Uint16 LR_received;

volatile Uint16 writeOrRead;

float lpfCoef[LPF_SIZE] = {
    -0.006839789359258000f, -0.004422361230581264f, -0.005313594533317870f, -0.005837626257671144f,
    -0.005793749493992318f, -0.004982662638984723f, -0.003214819060461159f, -0.000334148931615800f,
    0.003749824274853075f, 0.009065964703267525f, 0.015580924086943160f, 0.023135913375469132f,
    0.031526776234822251f, 0.040444876502862381f, 0.049531542147211255f, 0.058385243649591467f,
    0.066589078020969944f, 0.073733839041622493f, 0.079454449643651920f, 0.083449188230976773f,
    0.085501202079672101f, 0.085501202079672101f, 0.083449188230976773f, 0.079454449643651920f,
    0.073733839041622493f, 0.066589078020969944f, 0.058385243649591467f, 0.049531542147211255f,
    0.040444876502862381f, 0.031526776234822251f, 0.023135913375469132f, 0.015580924086943160f,
    0.009065964703267525f, 0.003749824274853075f, -0.000334148931615800f, -0.003214819060461159f,
    -0.004982662638984723f, -0.005793749493992318f, -0.005837626257671144f, -0.005313594533317870f,
    -0.004422361230581264f, -0.006839789359258000f
};

volatile int16 lpfCircularBuffer[LPF_SIZE] = {0};
volatile Uint16 lpfWriteIndex = 0;

float hpfCoef[HPF_SIZE] = {
    -0.050570093448268481f, 0.014605362781376828f, 0.017323704930409724f, 0.021052889922485024f,
    0.024050823432879852f, 0.024247716168742803f, 0.019858738554946624f, 0.009988602356382733f,
    -0.005631060962993745f, -0.026389614600094487f, -0.050616649901898056f, -0.075965769135735453f,
    -0.099706857997926640f, -0.119121118681522586f, -0.131795609185881346f, 0.863792865079370253f,
    -0.131795609185881346f, -0.119121118681522586f, -0.099706857997926640f, -0.075965769135735453f,
    -0.050616649901898056f, -0.026389614600094487f, -0.005631060962993745f, 0.009988602356382733f,
    0.019858738554946624f, 0.024247716168742803f, 0.024050823432879852f, 0.021052889922485024f,
    0.017323704930409724f, 0.014605362781376828f, -0.050570093448268481f
};

volatile int16 hpfCircularBuffer[HPF_SIZE] = {0};
volatile Uint16 hpfWriteIndex = 0;

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                         MAINS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

#ifdef PT1
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
    // interruptInit();            // enables GPIO interrupts for the RIGHT button

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

    // 48 KHz
    command = CLKsampleratecontrol (SR48);

    // 48 KHz output
    //float period = (1000000.0f/(2.0f*SAMPLING_FREQUENCY_48));
    //configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, period);

    // update frequencies on LCD
    char string[] = "Low-Pass";
    lcdRow1();
    lcdString((Uint16 *)&string);

    Uint16 buttons = 0;
    Uint32 switches = 0;
    void (*effect)(void) = &lpf;

    // IIR Filter Initialization
    iir.dbuffer_ptr = dbuffer;
    iir.coeff_ptr   = (long *)coeff;
    iir.qfmat       = IIR32_LPF_QFMAT;
    iir.nbiq        = IIR32_LPF_NBIQ;
    iir.isf         = IIR32_LPF_ISF;
    iir.init(&iir);

    while(1)
    {
        switches = getCodecSwitches();
        buttons = getCodecButtons();

        // LEFT - echo becomes the output effect
        // MIDDLE - reverb becomes the output effect
        if (buttons == MIDDLE_BUTTON)
        {
            lcdClearTopRow();
            char s[] = "Low-Pass";
            lcdRow1();
            lcdString((Uint16 *)&s);
            effect = &lpf;
        }
        else if (buttons & LEFT_BUTTON)
        {
            lcdClearTopRow();
            char s[] = "High-Pass";
            lcdRow1();
            lcdString((Uint16 *)&s);
            effect = &hpf;
        }
        else if (buttons & RIGHT_BUTTON)
        {
            lcdClearTopRow();
            char s[] = "Band-Pass";
            lcdRow1();
            lcdString((Uint16 *)&s);
            effect = &bpf;
        }

        while (LR_received == 0);
        LR_received = 0;
        (*effect)(); // call the filter effect
    }
}
#endif

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: lpf
 * Implements a low pass filter with the following specs
 * Pass-band: 0-1400 Hz, 3 dB ripple
 * Stop-band: 3k-24k Hz, -40 dB attenuation
 * Sampling Rate: 48 kHz
 *
 * GLOBALS:
 * DataInMono - int16 sample from the McBSP codec isr.
 * lpfCircularBuffer
 * lpfWriteIndex
 * lpfReadIndex
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void lpf(void)
{
    int16 output = 0;
    float convolved = 0.0f;

    // wait for a new sample xn and store to the circular buffer
    lpfCircularBuffer[lpfWriteIndex] = DataInMono;

    // run the filter with the weights and previous samples
    // from the circular buffer
    Uint16 iterator = lpfWriteIndex;
    for (Uint16 i = 0; i < LPF_SIZE; i++)
    {
        convolved += (float)lpfCircularBuffer[iterator] * lpfCoef[i];

        iterator++;
        if (iterator == LPF_SIZE)
            iterator = 0;
    }

    output = (int16)convolved;

    // increase the write index for the circular buffer
    lpfWriteIndex++;
    if (lpfWriteIndex == LPF_SIZE)
        lpfWriteIndex = 0;

    McbspbRegs.DXR2.all = output;   // tx
    McbspbRegs.DXR1.all = output;   // tx
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: hpf
 * Implements a high pass filter with the following specs
 * Pass-band: 4k-24k Hz, 3 dB ripple
 * Stop-band: 0-2k Hz, -40 dB attenuation
 * Sampling Rate: 48 kHz
 *
 * GLOBALS:
 * DataInMono - int16 sample from the McBSP codec isr.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void hpf(void)
{
    int16 output = 0;
    float convolved = 0.0f;

    // wait for a new sample xn and store to the circular buffer
    hpfCircularBuffer[hpfWriteIndex] = DataInMono;

    // run the filter with the weights and previous samples
    // from the circular buffer
    Uint16 iterator = hpfWriteIndex;
    for (Uint16 i = 0; i < HPF_SIZE; i++)
    {
        convolved += (float)hpfCircularBuffer[iterator] * hpfCoef[i];

        iterator++;
        if (iterator == HPF_SIZE)
            iterator = 0;
    }

    output = (int16)convolved;

    // increase the write index for the circular buffer
    hpfWriteIndex++;
    if (hpfWriteIndex == HPF_SIZE)
        hpfWriteIndex = 0;

    McbspbRegs.DXR2.all = output;   // tx
    McbspbRegs.DXR1.all = output;   // tx
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: bpf
 * Implements a band pass filter with the following specs
 * Pass-band: 400 - 800, 3 dB ripple
 * Stop-band: 0 - 100, 1.2K - 24K, 40 dB attenuation
 * Sampling Rate: 48 kHz
 *
 * GLOBALS:
 * DataInMono - int16 sample from the McBSP codec isr.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void bpf(void)
{
    xn = DataInMono;

    iir.input  = xn << 16;
    iir.calc(&iir);
    yn         = iir.output32;
    sigOut[i]  = yn;
    Rad        = Rad + RadStep;

    McbspbRegs.DXR2.all = yn >> 16;   // tx
    McbspbRegs.DXR1.all = yn >> 16;   // tx
}

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 * INTERRUPT SERVICE ROUTINES (ISR)
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

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
