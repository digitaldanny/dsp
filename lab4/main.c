#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "RTDSP.h"

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                      CONFIGURATIONS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */
#define PT2

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                       PROTOTYPES
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */
void pt2_main();
void pt3_main();

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                         MAINS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */

/*
 * +-----+-----+-----+-----+-----+-----+-----+
 *
 * +-----+-----+-----+-----+-----+-----+-----+
 */
void main(void)
{
#ifdef PT2
    pt2_main();
#endif

#ifdef PT3
    pt3_main();
#endif
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+
 * SUMMARY: pt2_main
 * This program outputs the following string
 * to the 16x2 LCD:
 * "Daniel Hamilton"
 * "EEL4511        "
 * +-----+-----+-----+-----+-----+-----+-----+
 */
void pt2_main()
{
    // local variable initializations
    Uint16 slaveAddress = 0x3F;
    float32 sysClkMhz = 10.0f;
    float32 I2CClkKHz = 25.0f; // 25KHz handles LCD delay required to process commands

    // initialization functions
    I2C_O2O_Master_Init(slaveAddress, sysClkMhz, I2CClkKHz);
    lcdInit();

    char stringA[] = "Daniel Hamilton";
    char stringB[] = "EEL4511";

    // Write strings out to the LCD
    lcdRow1();
    lcdString((Uint16 *)&stringA);
    lcdRow2();
    lcdString((Uint16 *)&stringB);

    // EOP -- infinite loop
    while (1) {DELAY_US(100);}
}

/*
 * +-----+-----+-----+-----+-----+-----+-----+
 *
 * +-----+-----+-----+-----+-----+-----+-----+
 */
void pt3_main()
{

}

/*
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 *                       FUNCTIONS
 * +=====+=====+=====+=====+=====+=====+=====+=====+=====+
 */
