/*
 * RTDSP_SramSpi.c
 *
 *  Created on: Sep 23, 2019
 *      Author: Daniel Hamilton
 */

#include "RTDSP_SramSpi.h"

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: initSramSpi
 *
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void sramSpiInit(void)
{
    EALLOW;

    // initialize the CS GPIO (66/67) for output ------------------------------
    GpioDataRegs.GPCDAT.bit.GPIO66 = 1; // initialize CS0 to be high
    GpioCtrlRegs.GPCDIR.bit.GPIO66 = GPIO_DIR_MODE_OUT;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO66 = 0x0000;
    GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 0x0000;
    GpioCtrlRegs.GPCPUD.bit.GPIO66 = 0x0000;

    GpioDataRegs.GPCDAT.bit.GPIO67 = 1; // initialize CS1 to be high
    GpioCtrlRegs.GPCDIR.bit.GPIO67 = GPIO_DIR_MODE_OUT;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO67 = 0x0000;
    GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0x0000;
    GpioCtrlRegs.GPCPUD.bit.GPIO67 = 0x0000;

    // SPI GPIO muxes ---------------------------------------------------------
    GpioCtrlRegs.GPBDIR.bit.GPIO63 = GPIO_DIR_MODE_OUT;  // GPIO63 => SPI_SIMO_B
    GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 3;                // GPIO63 => SPI_SIMO_B
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 3;                 // GPIO63 => SPI_SIMO_B
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0x0000;

    GpioCtrlRegs.GPCDIR.bit.GPIO64 = GPIO_DIR_MODE_OUT;  // GPIO64 => SPI_SOMI_B
    GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 3;                // GPIO64 => SPI_SOMI_B
    GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 3;                 // GPIO64 => SPI_SOMI_B
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0x0000;

    GpioCtrlRegs.GPCDIR.bit.GPIO65 = GPIO_DIR_MODE_OUT;  // GPIO65 => SPI_SCLK_B
    GpioCtrlRegs.GPCGMUX1.bit.GPIO65 = 3;                // GPIO65 => SPI_SCLK_B
    GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 3;                 // GPIO65 => SPI_SCLK_B
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0x0000;

    // SPI_B CONFIGURATIONS ---------------------------------------------------
    SpibRegs.SPICCR.bit.SPISWRESET = 0; // sw reset

    SpibRegs.SPISTS.bit.INT_FLAG = 0;
    SpibRegs.SPISTS.bit.OVERRUN_FLAG = 0;

    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0;        // LSPCLK = 200 MHz
    SpibRegs.SPICCR.bit.CLKPOLARITY = 1;        // Clocks on rising edge
    SpibRegs.SPICTL.bit.CLK_PHASE   = 0;        // 1 => shifted over by 1/2 clock
    SpibRegs.SPICCR.bit.SPICHAR     = (8-1);    // size of the data word
    //SpibRegs.SPICCR.bit.SPILBK      = 1;        // enable spi loopback
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1;       // 1 = master mode (pg. 2148)
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 200;     //4;   // SYSCLOCK/(PRESCALER + 1) = SPICLK (pg. 2134)
    SpibRegs.SPICTL.bit.TALK = 1;               // DAT used for serial transmits (pg. 2156)

    // SPI FIFO CONFIGURATIONS -------------------------------------------------

    // /*
    //  * SPIRST => 1, FIFO can resume tx/rx
    //  * SPIFFENA => 1, FIFO enhancements
    //  * TXFIFO => 1, Release from reset
    //  * TXFFINTCLR => 1, Clear SPI interrupt flag
    //  */
    // SpibRegs.SPIFFTX.all = 0xE040;
    //
    // /*
    //  *  RXFIFORESET => 1, Re-enable receive FIFO options
    //  *  RXFFIL => 4, Reset when 4 or more words in FIFO RX buffer
    //  *  RXFFINTCLR => Clear FFRX interrupt flag
    //  */
    // SpibRegs.SPIFFRX.all = 0x2044;
    // SpibRegs.SPIFFCT.all = 0x0;
    //
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // release SPI from sw reset
    SpibRegs.SPIPRI.bit.FREE = 1; // run SPI even when emulation suspended
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: spiTransmit
 * This function transmits a byte of SPI data without
 * consideration of the CS lines. The MISO value is returned
 * as a Uint16.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
Uint16 spiTransmit(Uint16 data)
{
    Uint16 recv = 0xDEAD;

    // data transmit + receive
    SpibRegs.SPIDAT = (data << 8) & 0xFF00;     // fill the buffer with left-justified data.
    while (SpibRegs.SPISTS.bit.INT_FLAG != 1);  // wait until data has been received.
    recv = SpibRegs.SPIDAT & 0x00FF;            // Right-justified data returns

    /*
     * SPI FIFO TRANSMIT
     */
    // SpibRegs.SPITXBUF = (data << 8) & 0xFF00; // left justified data
    // while (SpibRegs.SPIFFRX.bit.RXFFST != 1); // wait until data is received
    // recv = SpibRegs.SPIRXBUF & 0x00FF;        // return data is right justified

    // clear flags for future transmits
    data = SpibRegs.SPIRXBUF;                   // clears flags without resetting SPI

    return recv;
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: sramWrite16
 * This function burst writes a variable number of bytes
 * to the selected SRAM device through SPI.
 *
 * INPUTS:
 * addr - starting address to write data to.
 * data - pointer to the array of Uint16's to write to
 * len -
 * cs -
 *
 * BYTE ORDER: MSB to LSB
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void sramWrite(Uint32 addr, Uint16 * data, Uint16 len, Uint16 cs)
{
    EALLOW;

    // Select the appropriate SPI slave to enable
    if (cs == 0)
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // select SRAM 0
    else
        GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1; // select SRAM 1

    // instruction transfer
    Uint16 instruction = 0x02;
    spiTransmit(instruction);

    // Transmit the address..
    Uint16 uAddr = (addr & 0x00FF0000) >> 16; // bits 24..16 of address
    Uint16 mAddr = (addr & 0x0000FF00) >> 8; // bits 15..8 of address
    Uint16 lAddr = (addr & 0x000000FF) >> 0; // bits 7..0 of address
    spiTransmit(uAddr);
    spiTransmit(mAddr);
    spiTransmit(lAddr);

    // Transmit bursts of data..
    for (int i = 0; i < len; i++)
    {
        spiTransmit(data[i]);
    }

    // make sure all SPI slaves are disabled.
    //DELAY_US(4); // delay allows waveforms to translate SPI
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // select SRAM 0
    GpioDataRegs.GPCSET.bit.GPIO67 = 1; // select SRAM 1
    EDIS;
}
