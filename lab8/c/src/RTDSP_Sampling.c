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

#define TWO_CHANNEL

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
 * INPUTS:
 * --------------------
 * mcbspIntEn
 * --------------------
 * CODEC_MCBSPB_INT_DIS - disable McBSPb interrupts (used for DMA ping-pong)
 * CODEC_MCBSPB_INT_EN - enable McBSPb interrupts
 *
 * FUTURE IMPLEMENTATION:
 * __interrupt void cpuTimer1ISR(void);
 * interrupt void ISR_rightButton(void);
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void initCodec(Uint16 mcbspIntEn)
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

    if (mcbspIntEn == CODEC_MCBSPB_INT_EN)
    {
        Interrupt_enable(INT_MCBSPB_RX);
        Interrupt_register(INT_MCBSPB_RX, &Mcbsp_RxINTB_ISR); // set I2S RX interrupt to ISR address
        InitMcBSPb(1); // initialize I2S for sound input/output with receive interrupt enabled
    }
    else
    {
        Interrupt_disable(INT_MCBSPB_RX);
        InitMcBSPb(0); // initialize I2S for sound input/output with receive interrupt disabled
    }

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

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: init_dma
 * Initialize DMA for ping-pong sampling for word sizes == 16.
 *
 * INPUTS:
 * ping - address of the first sample buffer
 * pong - address of the second sample buffer
 * transferSize - size of the frame to transfer before interrupting
 *
 * GLOBALS (must be declared in external file):
 * __interrupt void DMA_CH1_ISR(void)
 * __interrupt void DMA_CH2_ISR(void)
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void init_dma(int16 * ping, int16 * pong, Uint32 transferSize)
{
    EALLOW;    // Allow access to EALLOW protected registers
    PieVectTable.DMA_CH1_INT= &RTDSP_DMA_CH1_ISR;

    CpuSysRegs.SECMSEL.bit.PF2SEL = 1;

    DmaRegs.DMACTRL.bit.HARDRESET = 1;
    __asm(" NOP");                        // Only 1 NOP needed per Design

    DmaRegs.CH1.MODE.bit.CHINTE = 0;

    // Channel 1, McBSPB transmit buffer 1
    DmaRegs.CH1.BURST_SIZE.all = 1;       // 2 word/burst
    DmaRegs.CH1.SRC_BURST_STEP = 1;       // no effect when using 1 word/burst
    DmaRegs.CH1.DST_BURST_STEP = 1;       // no effect when using 1 word/burst
    DmaRegs.CH1.TRANSFER_SIZE = transferSize-1; // Interrupt every frame
                                          // (127 bursts/transfer)
    DmaRegs.CH1.SRC_TRANSFER_STEP = 1;    // Move to next word in buffer after
                                          // each word in a burst
    DmaRegs.CH1.DST_TRANSFER_STEP = 0;    // Don't move destination address
    DmaRegs.CH1.SRC_ADDR_SHADOW = (Uint32)&McbspbRegs.DRR1.all; // Start address = McBSPB DRR
    DmaRegs.CH1.SRC_BEG_ADDR_SHADOW = (Uint32)&McbspbRegs.DRR1.all;
    DmaRegs.CH1.DST_ADDR_SHADOW = (Uint32) &ping; // ping buffer is the destination
    //
    // Not needed unless using wrap function
    //
    DmaRegs.CH1.DST_BEG_ADDR_SHADOW = (Uint32) &ping;
    DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;   // Clear peripheral interrupt event flag.
    DmaRegs.CH1.CONTROL.bit.ERRCLR = 1;      // Clear sync error flag
    DmaRegs.CH1.DST_WRAP_SIZE = 0xFFFF;      // Put to maximum - don't want
                                             // destination wrap.
    DmaRegs.CH1.SRC_WRAP_SIZE = 0xFFFF;      // Put to maximum - don't want
                                             // source wrap.
    DmaRegs.CH1.MODE.bit.CHINTE = 1;         // Enable channel interrupt
    DmaRegs.CH1.MODE.bit.CHINTMODE = 1;      // Interrupt at end of transfer
    DmaRegs.CH1.MODE.bit.PERINTE = 1;        // Enable peripheral interrupt event
    DmaRegs.CH1.MODE.bit.PERINTSEL = 1;      // Peripheral interrupt select = McBSP MXSYNCA

    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH1 = DMA_MREVTB; // Trigger on McBSPb_RX
    DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;   // Clear any spurious interrupt flags

    PieCtrlRegs.PIEIER7.bit.INTx1 = 1;   // Enable PIE Group 7, INT 1 (DMA CH1)

#ifdef TWO_CHANNEL
    //
    // Channel 2, McBSPB Receive buffer 2
    //
    PieVectTable.DMA_CH2_INT= &RTDSP_DMA_CH2_ISR;

    DmaRegs.CH2.MODE.bit.CHINTE = 1;
    DmaRegs.CH2.BURST_SIZE.all = 1;        // 1 word/burst
    DmaRegs.CH2.SRC_BURST_STEP = 1;        // no effect when using 1 word/burst
    DmaRegs.CH2.DST_BURST_STEP = 1;        // no effect when using 1 word/burst
    DmaRegs.CH2.TRANSFER_SIZE = transferSize - 1;       // Interrupt every 127 bursts/transfer
    DmaRegs.CH2.SRC_TRANSFER_STEP = 0;     // Don't move source address
    DmaRegs.CH2.DST_TRANSFER_STEP = 1;     // Move to next word in buffer after
                                           // each word in a burst
    DmaRegs.CH2.SRC_ADDR_SHADOW = (Uint32) &McbspbRegs.DRR1.all; // Start address
                                                                 // = McBSPB DRR
    //
    // Not needed unless using wrap function
    //
    DmaRegs.CH2.SRC_BEG_ADDR_SHADOW = (Uint32) &McbspbRegs.DRR1.all;
    DmaRegs.CH2.DST_ADDR_SHADOW = (Uint32) &pong;      // Start address =
                                                           // Receive buffer
                                                           // (for McBSP-B)
    DmaRegs.CH2.DST_BEG_ADDR_SHADOW = (Uint32) &pong;  // Not needed unless
                                                           // using wrap function
    DmaRegs.CH2.CONTROL.bit.PERINTCLR = 1; // Clear peripheral interrupt event
                                           // flag.
    DmaRegs.CH2.CONTROL.bit.ERRCLR = 1;    // Clear sync error flag
    DmaRegs.CH2.DST_WRAP_SIZE = 0xFFFF;    // Put to maximum - don't want
                                           // destination wrap.
    DmaRegs.CH2.SRC_WRAP_SIZE = 0xFFFF;    // Put to maximum - don't want
                                           // source wrap.
    DmaRegs.CH2.MODE.bit.CHINTE = 1;       // Enable channel interrupt
    DmaRegs.CH2.MODE.bit.CHINTMODE = 1;    // Interrupt at end of transfer
    DmaRegs.CH2.MODE.bit.PERINTE = 1;      // Enable peripheral interrupt event
    DmaRegs.CH2.MODE.bit.PERINTSEL = 2;    // Peripheral interrupt select =
                                           // McBSP MRSYNCB
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH2 = DMA_MREVTB; // Trigger on McBSPb_RX
    DmaRegs.CH2.CONTROL.bit.PERINTCLR = 1; // Clear any spurious interrupt flags

    PieCtrlRegs.PIEIER7.bit.INTx2 = 1;   // Enable PIE Group 7, INT 2 (DMA CH2)
#endif

    IER |= 0x40;                            // Enable CPU INT groups 6 and 7
    EDIS;
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: start_dma
 * Start DMA on channels 1 and 2
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void start_dma (void)
{
  EALLOW;
  DmaRegs.CH1.CONTROL.bit.RUN = 1;      // Start DMA Transmit from McBSP-B

#ifdef TWO_CHANNEL
  DmaRegs.CH2.CONTROL.bit.RUN = 1;      // Start DMA Receive from McBSP-B
#endif
  EDIS;
}
