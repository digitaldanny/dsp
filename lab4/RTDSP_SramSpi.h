/*
 * RTDSP_SramSpi.h
 *
 *  Created on: Sep 23, 2019
 *      Author: dhami
 */

#ifndef SRC_RTDSP_SRAMSPI_H_
#define SRC_RTDSP_SRAMSPI_H_

#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"

void sramSpiInit(void);

/*
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: spiTransmit
 * This function transmits a byte of SPI data without
 * consideration of the CS lines. The MISO value is returned
 * as a Uint16.
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
Uint16 spiTransmit(Uint16 data);

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
void sramWrite(Uint32 addr, Uint16 * data, Uint16 len, Uint16 cs);

#endif /* SRC_RTDSP_SRAMSPI_H_ */
