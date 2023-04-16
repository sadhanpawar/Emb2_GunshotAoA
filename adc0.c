#include <stdint.h>
#include "gpio.h"
#include "adc0.h"
#include "tm4c123gh6pm.h"
#include "nvic.h"

/********************************
 *       3 *******************
 *         * Micrcontroller **
 *         *******************
 *  2           1
*********************************/
void initAdc0(void)
{
    SYSCTL_RCGCADC_R = SYSCTL_RCGCADC_R0;  //TODO: R1 ??
    __delay_cycles(3);

    //enablePort(PORTD); needed if 4th mic is used
    enablePort(PORTE);

    selectPinAnalogInput(MIC_BSD_1);
    selectPinAnalogInput(MIC_FAR_2);
    selectPinAnalogInput(MIC_FRONT_3);

    /*
    selectPinAnalogInput(MIC_FAR_1);
    selectPinAnalogInput(MIC_FRONT_2);
    selectPinAnalogInput(MIC_BSD_3);
    */

    ADC0_ACTSS_R = 0x00;
    ADC0_EMUX_R = 0x000F;
    ADC0_SSMUX0_R = 0x321321;
    ADC0_SSCTL0_R = (1 << 21) | (1<<22); /*6 samples and interrupt enable for 6 samples */
    ADC0_IM_R = 0x01;
    ADC0_CTL_R = (1<<6);
    ADC0_ACTSS_R = 0x01;
    ADC0_PSSI_R = (0x01);

    enableNvicInterrupt(INT_ADC0SS0);
}
