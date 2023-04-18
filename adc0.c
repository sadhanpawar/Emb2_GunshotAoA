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
    __delay_cycles(16);

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
    ADC0_SSMUX0_R = 0x12012012;//initial 0x12 are discarded while reading
    ADC0_SSCTL0_R = (1 << 30) | (1 << 29); /*8 samples and interrupt enable for 8 samples */
    ADC0_IM_R = 0x01;
    ADC0_CTL_R = (1<<6); //dither enable
    ADC0_CC_R = ADC_CC_CS_SYSPLL;
    ADC0_PC_R = ADC_PC_SR_1M;
    ADC0_ACTSS_R = 0x01;
    enableNvicInterrupt(INT_ADC0SS0);

    ADC0_PSSI_R = 0x01;
}
