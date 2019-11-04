/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * | SUMMARY: main.c                                                 |
 * | This program runs a LPF, HPF, and BPF in real time on the input |
 * | sound from the line in. The filter can be selected using the    |
 * | 3 codec push buttons.                                           |
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

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                        DEFINES
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                       PROTOTYPES
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

void adcA0Init();
void interruptInit();

__interrupt void Mcbsp_RxINTB_ISR(void);

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                        GLOBALS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

volatile Uint16 boolTimer1;  // allows main to determine if the timer has run or not

// RTDSP_Sampling Requirements
volatile i2sSide_t ch_sel;
volatile float DataInLeft;
volatile float DataInRight;
volatile int16 DataInMono;
volatile Uint16 LR_received;

volatile Uint16 writeOrRead;

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                         MAINS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

#ifdef PT1
/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * This function implements 256 point DFT
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void main()
{
    // sets up codec and processor for sampling at 48 KHz
    initCodec();

    // update frequencies on LCD
    char s1[] = "Max Freq = XXXX Hz";
    lcdRow1();
    lcdString((Uint16 *)&s1);

    char s2[] = "Max Mag = XXXX dB";
    lcdRow2();
    lcdString((Uint16 *)&s2);

    while(1)
    {
        while (LR_received == 0);
        LR_received = 0;
    }
}
#endif

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
