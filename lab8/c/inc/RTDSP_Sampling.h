#ifndef SRC_RTDSP_SAMPLING_H__
#define SRC_RTDSP_SAMPLING_H_

#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "InitAIC23.h"
#include "AIC23.h"
#include "OneToOneI2CDriver.h"
#include "RTDSP_SramSpi.h"
#include "RTDSP_Timer.h"
#include "RTDSP_CodecGpio.h"

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
 *                        GLOBALS
 *           Must be defined in main.c source code
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

extern volatile i2sSide_t ch_sel;
extern volatile float DataInLeft;
extern volatile float DataInRight;
extern volatile int16 DataInMono;
extern volatile Uint16 LR_received;
extern __interrupt void Mcbsp_RxINTB_ISR(void);

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
 *  FUTURE IMPLEMENTATION:
 * __interrupt void cpuTimer1ISR(void);
 * interrupt void ISR_rightButton(void);
 * +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 */
void initCodec(void);

#endif // SRC_RTDSP_SAMPLING_H_ //
