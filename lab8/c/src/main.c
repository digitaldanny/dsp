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

#define SIZE_OF_DFT     256
#define NUM_DFT_BINS    128
#define FREQ_PER_BIN    187.5f //(48000.0f / (float)SIZE_OF_DFT)

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                       TYPEDEFS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

typedef struct frame
{
    int16 buffer[SIZE_OF_DFT];        // stores all samples
    Uint16 count;                     // current size of the buffer
    struct frame * nextFrame;         // points to the next frame for processing
    Uint16 waitingToProcess;          // 1 => do not switch to next frame, 0 => ready to switch frames
    void (*process)(int16*, float*, Uint16, Uint16);  // points to DFT or FFT function and returns pointer to bin array
} frame_t;

typedef struct polar
{
    float magnitude;
    float freq;
} polar_t;

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                       PROTOTYPES
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

void adcA0Init();
void interruptInit();
void dft (int16*, float*, Uint16, Uint16);
polar_t searchMaxBin (float * bin, Uint16 len, float freqPerBin);

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

// ping pong buffers ---------------------------
frame_t frames[2];     // 2 buffers for ping-ponging processing and sampling
frame_t * dftFrame;    // points at the frame to be processed
frame_t * storeFrame;  // points at the frame to store new values
float bin[NUM_DFT_BINS]; // stores result of dft256 function
polar_t testPointMax;

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
    initCodec(); // sets up codec and processor for sampling at 48 KHz

    // update frequencies on LCD
    char s1[] = "Max Freq = XXXX Hz";
    lcdRow1();
    lcdString((Uint16 *)&s1);

    char s2[] = "Max Mag = XXXX dB";
    lcdRow2();
    lcdString((Uint16 *)&s2);

    // **************************************************//
    // initialize globals                                //
    // **************************************************//
    frames[0].count         = 0;                         //
    frames[0].nextFrame     = &frames[1];                //
    frames[0].process       = &dft;                      //
                                                         //
    frames[1].count         = 0;                         //
    frames[1].nextFrame     = &frames[0];                //
    frames[1].process       = &dft;                      //
                                                         //
    dftFrame                = &frames[0];                //
    storeFrame              = &frames[0];                //
    // **************************************************//

    while(1)
    {
        // ------------------------------------------------------
        // DFT PROCESSING
        // ------------------------------------------------------
        if (dftFrame->count == SIZE_OF_DFT-1)
        {
            // 1.) perform dft on dftBuffer if buffer is full
            dftFrame->process(dftFrame->buffer, bin, NUM_DFT_BINS, SIZE_OF_DFT);

            // 2.) stores max magnitude and frequency to the LCD
            testPointMax = searchMaxBin (bin, NUM_DFT_BINS, FREQ_PER_BIN);

            // 3.) reset count/lock for the current DFT frame so it can store samples later
            dftFrame->count = 0;
            dftFrame->waitingToProcess = 0;

            // 4.) switch dft pointer to point at the next buffer
            dftFrame = dftFrame->nextFrame;
        }
    }
}
#endif

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: dft
 * This function performs a 256 point dft on a buffer of samples and
 * returns a pointer to the resulting bin array.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void dft (int16 * buffer, float * bin, Uint16 numBins, Uint16 dftSize)
{
    float Re;
    float Im;
    float xn;
    float theta;

    for (Uint16 k = 0; k < numBins; k++)
    {
        Re = 0;
        Im = 0;

        // DFT for the selected bin (k)
        for (Uint16 n = 0; n < dftSize; n++)
        {
            xn = (float)(*(buffer + n));
            theta = (2.0f * M_PI * (float)k * (float)n) / 256.0f;

            Re += xn*cosf(theta);
            Im += xn*sinf(theta);
        }

        bin[k] = sqrtf(Re*Re + Im*Im);
    }
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: searchMaxBin
 * This function searches a bin array for the max magnitude and outputs
 * the max magnitude and frequency to the LCD
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
polar_t searchMaxBin (float * bin, Uint16 len, float freqPerBin)
{
    char wr[5] = "X.XX"; // store ASCII versions of DFT magnitude in here..
    float max = 0.0f;
    Uint16 maxK = 0;
    polar_t conversion;

    // 1.) search for the max bin value
    for (Uint16 k = 0; k < len; k++)
    {
        if (bin[k] > max)
        {
            max = bin[k];
            maxK = k;
        }

        bin[k] = 0.0f; // reset to be used later
    }

    max = 10.0f*log10f(max);

    // 2.) Output the max bin magnitude to the LCD
    float ones         = max;
    float tenths       = (max - (float)((Uint16)ones)) * 10;
    float hundredths   = (tenths - (float)((Uint16)tenths)) * 10;

    // Convert voltage to characters and store to the LCD
    wr[0] = INT_TO_ASCII((Uint16)ones); // 1's place (Ex. [1].23)
    wr[1] = '.';
    wr[2] = INT_TO_ASCII((Uint16)tenths); // 10th's place (Ex. 1.[2]3)
    wr[3] = INT_TO_ASCII((Uint16)hundredths); // 10th's place (Ex. 1.2[3])
    wr[4] = '\0';

    lcdCursorRow2(10); // offset t.o the X.XX decimal voltage value
    lcdString((Uint16*)&wr);

    // 3.) Output the max frequency to the LCD
    float maxFreq = (float)maxK * freqPerBin;
    //float thousands = maxFreq / 1000.0f;
    //float hundreds = maxFreq
    ones         = maxFreq;
    tenths       = (maxFreq - (float)((Uint16)ones)) * 10;
    hundredths   = (tenths - (float)((Uint16)tenths)) * 10;

    // Convert voltage to characters and store to the LCD
    wr[0] = INT_TO_ASCII((Uint16)ones); // 1's place (Ex. [1].23)
    wr[2] = INT_TO_ASCII((Uint16)tenths); // 10th's place (Ex. 1.[2]3)
    wr[3] = INT_TO_ASCII((Uint16)hundredths); // 10th's place (Ex. 1.2[3])

    lcdCursorRow1(11);
    lcdString((Uint16*)&wr);

    // create the return structure
    conversion.magnitude = max;
    conversion.freq = maxFreq;
    return conversion;
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
    DataInMono = (int16)((DataInRight + DataInLeft)/2.0f);  // Mono = average of left and right

    LR_received = 1; // flag for main

    // ------------------------------------------------------
    // NEW SAMPLE STORAGE
    // ------------------------------------------------------

    // store new sample to sample buffer
    if (storeFrame->count < SIZE_OF_DFT-1)
    {
        storeFrame->buffer[storeFrame->count] = DataInMono;
        storeFrame->count++;
    }

    // point to the new sample buffer
    else if (storeFrame->waitingToProcess == 0)
    {
        storeFrame->waitingToProcess = 1;   // doesn't allow loop to immediately switch back
        storeFrame = storeFrame->nextFrame; // point to the next frame for storing data
    }

    // Probe to check interrupt timing
    GpioDataRegs.GPDDAT.bit.GPIO123 = 1;
    GpioDataRegs.GPDDAT.bit.GPIO123 = 0;

    // acknowledge interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}
