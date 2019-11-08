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
#define PT2

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                        DEFINES
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

#pragma DATA_SECTION(frames, "share")  // DMA-accessible RAM

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
    int16 buffer[1024];               // buffer is large enough for a 512 point DFT using DMA
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

int16 avg2(int16 a, int16 b);
void dftLR (int16*, float*, Uint16, Uint16);
void dft (int16*, float*, Uint16, Uint16);
polar_t searchMaxBin (float * bin, Uint16 len, float freqPerBin);

__interrupt void Mcbsp_RxINTB_ISR(void);
__interrupt void DMA_FRAME_COMPLETE_ISR(void);

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

// ping pong buffers
volatile Uint16 dma_flag;   // notifies program to begin dft computation
frame_t frames[2];          // 2 buffers for ping-ponging processing and sampling
frame_t * dftFrame;         // points at the frame to be processed
frame_t * storeFrame;       // points at the frame to store new values
float bin[NUM_DFT_BINS];    // stores result of dft256 function
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
    DINT;  // Enable Global interrupt INTM
    DRTM;  // Enable Global realtime interrupt DBGM

    InitSysCtrl();              // disable watchdog
    InitPieCtrl();              // set PIE ctrl registers to default state
    InitPieVectTable();         // set PIE vectors to default shell ISRs

    // sets up codec and processor for sampling at 48 KHz
    initCodec(CODEC_MCBSPB_INT_EN);

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM
    // update frequencies on LCD
    char s1[] = "Freq = XXXX Hz";
    lcdRow1();
    lcdString((Uint16 *)&s1);

    char s2[] = "Mag = XXXX dB";
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
#ifdef PT2
/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * This function implements 256 point DFT with DMA for
 * ping-pong buffer switching.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void main()
{
    DINT;  // Enable Global interrupt INTM
    DRTM;  // Enable Global realtime interrupt DBGM

    InitSysCtrl();              // disable watchdog
    InitPieCtrl();              // set PIE ctrl registers to default state
    InitPieVectTable();         // set PIE vectors to default shell ISRs

    // sets up codec and processor for sampling at 48 KHz
    initDmaPingPong(&frames[0].buffer[0], &frames[1].buffer[0], SIZE_OF_DFT, &DMA_FRAME_COMPLETE_ISR);
    initCodec(CODEC_MCBSPB_INT_DIS);

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    // update frequencies on LCD
    char s1[] = "Freq = XXXX Hz";
    lcdRow1();
    lcdString((Uint16 *)&s1);

    char s2[] = "Mag = XXXX dB";
    lcdRow2();
    lcdString((Uint16 *)&s2);

    // **************************************************//
    // initialize globals                                //
    // **************************************************//
    frames[0].process       = &dftLR;                    //
    frames[0].nextFrame     = &frames[1];                //
                                                         //
    frames[1].process       = &dftLR;                    //
    frames[1].nextFrame     = &frames[0];                //
                                                         //
    dftFrame                = &frames[0];                //
    dma_flag                = 0;                         //
    // **************************************************//

    while(1)
    {
        if (dma_flag)
        {
            // perform DFT or FFT on the processing buffer..
            dftFrame->process(dftFrame->buffer, bin, NUM_DFT_BINS, SIZE_OF_DFT);
            dma_flag = 0;

            // next frame to be processed
            dftFrame = dftFrame->nextFrame;

            // search bins for the max frequency and display info to LCD
            testPointMax = searchMaxBin (bin, NUM_DFT_BINS, FREQ_PER_BIN);
        }
    }
}
#endif

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 * FUNCTIONS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
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
    return (int16)af;       // package for output
}

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
            theta = (2.0f * M_PI * (float)k * (float)n) / (float)dftSize;

            Re += xn*cosf(theta);
            Im += xn*sinf(theta);
        }

        bin[k] = sqrtf(Re*Re + Im*Im);
    }
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: dftLR
 * This function performs a dftSize point dft on a buffer of samples and
 * returns a pointer to the resulting bin array. The expected input buffer
 * order is as follows..
 *
 * buffer[2*dftSize] = {LEFT, RIGHT, LEFT, RIGHT, ...}
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void dftLR (int16 * buffer, float * bin, Uint16 numBins, Uint16 dftSize)
{
    float Re;
    float Im;
    float xn;
    float theta;

    int16 left;
    int16 right;
    int16 monoAudio;

    for (Uint16 k = 0; k < numBins; k++)
    {
        Re = 0;
        Im = 0;

        // DFT for the selected bin (k)
        for (Uint16 n = 0; n < dftSize; n++)
        {
            left = *(buffer + 2*n);
            right = *(buffer + 2*n + 1);
            monoAudio = avg2(left, right);

            xn = (float)monoAudio;
            theta = (2.0f * M_PI * (float)k * (float)n) / (float)dftSize;

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
    char wr2[8] = "XXXXX.X"; // store ASCII versions of the max frequency here..
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
    float tens         = max / 10.0f;
    float ones         = (tens - (Uint16)tens) * 10;
    float tenths       = (ones - (Uint16)ones) * 10;
    float hundredths   = (tenths - (Uint16)tenths) * 10;

    // Convert voltage to characters and store to the LCD
    wr[0] = INT_TO_ASCII((Uint16)tens);
    wr[1] = INT_TO_ASCII((Uint16)ones); // 1's place (Ex. [1].23)
    wr[2] = '.';
    wr[3] = INT_TO_ASCII((Uint16)tenths); // 10th's place (Ex. 1.[2]3)
    wr[4] = '\0';

    lcdCursorRow2(6); // offset t.o the X.XX decimal voltage value
    lcdString((Uint16*)&wr);

    // 3.) Output the max frequency to the LCD
    float maxFreq       = (float)maxK * freqPerBin;
    float tenThousands  = maxFreq / 10000.0f;
    float thousands     = (tenThousands - (Uint16)tenThousands) * 10;
    float hundreds      = (thousands - (Uint16)thousands) * 10;
    tens                = (hundreds - (Uint16)hundreds) * 10;
    ones                = (tens - (Uint16)tens) * 10;
    tenths              = (ones - (Uint16)ones) * 10;

    // Convert voltage to characters and store to the LCD
    wr2[0] = INT_TO_ASCII((Uint16)tenThousands);
    wr2[1] = INT_TO_ASCII((Uint16)thousands);
    wr2[2] = INT_TO_ASCII((Uint16)hundreds);
    wr2[3] = INT_TO_ASCII((Uint16)tens);
    wr2[4] = INT_TO_ASCII((Uint16)ones);
    wr2[5] = '.';
    wr2[6] = INT_TO_ASCII((Uint16)tenths);
    wr2[7] = '\0';

    lcdCursorRow1(7);
    lcdString((Uint16*)&wr2);

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
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * Codec Audio input interrupt - DSP mode requires 1 interrupt
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
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
    EALLOW;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
    EDIS;
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * DMA_FRAME_COMPLETE_ISR - DMA ISR for channel 6
 * This interrupt ping-pongs the sampling buffer and the processing
 * buffer.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
__interrupt void DMA_FRAME_COMPLETE_ISR(void)
{
    EALLOW;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP7; // ACK to receive more interrupts from this PIE groups
    EDIS;

    pingPong();     // switch buffer end-points on DMA channels
    dma_flag = 1;   // used in program to trigger DFT calculations

    startDmaChannels();
}
