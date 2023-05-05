/*
 * main.c
 *
 *  Created on: Apr 8, 2023
 *      Author: sadhan
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <string.h>
#include <inttypes.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "gpio.h"
#include "wait.h"
#include "nvic.h"
#include "userInterface.h"
#include "uart0.h"
#include "adc0.h"
#include "main.h"
#include "timer.h"
#include "eeprom.h"
#include "dma.h"
#include "userInterface.h"
#include "rgb_led.h"

/******************************************************************
 * NOTE : Optimisation level should be local optimisation to overcome
 * buffer overflow
 */
bool const iirFilterEnable = false;
bool const dmaEnable = false;
uint16_t mic1Samples[NO_OF_SAMPLES] = {0};
uint16_t mic2Samples[NO_OF_SAMPLES] = {0};
uint16_t mic3Samples[NO_OF_SAMPLES] = {0};
uint16_t idx = 0;
bool calAvgNow = false;
const float alpha = 0.9f;

uint32_t mic1SamplesAvg = UINT_MAX;
uint32_t mic2SamplesAvg = UINT_MAX;
uint32_t mic3SamplesAvg = UINT_MAX;

uint32_t threshold = 500;
uint16_t timeConstant = 0;
uint16_t backOff = 0;
uint16_t holdOff = 0;
uint16_t hysteresis = 0;
bool tdoaEnable = true;
bool partialSets = false;
uint16_t aoaValue = 0;

uint8_t wherePeek = 0;
uint8_t secondPeek = 0;

static uint8_t aoAFSM = AVERAGE_FSM;
uint32_t adcTicks = 0;
bool adcTimeConstantFlag = false;
bool holdOffFlag = false;
bool hysteresisFlag = false;

peekDetectOrder_t peekDetectionOrder[3] = {0};
static uint8_t orderIdx = 0;

int rolledIdx = 0;

uint8_t threeSampleDetCount = 0;
uint32_t peekTimeout = 0;

uint16_t angles[] = {0,330,210,90}; //0th idx discarded

float const constK = 1.2;

#define DEBUG
#define FIRFILTER 0
#define IIRFILTER 1
#define FILTER FIRFILTER
//#define DMAENABLED

uint16_t trash = 0;
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();
    enablePort(PORTF);
    selectPinPushPullOutput(PEAKS_DETECT_LED);

    initUart0();
    setUart0BaudRate(115200, 40e6);

    #ifdef DMAENABLED
        initDma();
    #endif

    initEeprom();
    initAdc0();
    initRgb();
}

void aoaAdc0InterruptHandler(void)
{
    #if 1
        
        #if FILTER == FIRFILTER

        mic1Samples[idx] = ADC0_SSFIFO0_R;
        mic2Samples[idx] = ADC0_SSFIFO0_R;
        mic3Samples[idx] = ADC0_SSFIFO0_R;

        idx = (idx+1)%NO_OF_SAMPLES;

        mic1Samples[idx] = ADC0_SSFIFO0_R;
        mic2Samples[idx] = ADC0_SSFIFO0_R;
        mic3Samples[idx] = ADC0_SSFIFO0_R;

        
        trash = ADC0_SSFIFO0_R;
        trash = ADC0_SSFIFO0_R;


        if((aoAFSM == AVERAGE_FSM) && !holdOffFlag)
        {
            rolledIdx = idx - 1;
            
            if(rolledIdx < 0) {
                rolledIdx = (NO_OF_SAMPLES - 1);
            }

            if(mic1Samples[rolledIdx] > (mic1SamplesAvg + threshold))
            {
                aoAFSM = PEAKS_FSM;
                wherePeek = 1;
                peekTimeout = adcTicks;
                peekDetectionOrder[0].order = 1;
                peekDetectionOrder[0].timer = adcTicks;
                peekDetectionOrder[0].index = rolledIdx;
                orderIdx = 1;
                threeSampleDetCount = 1;
                #ifdef DEBUG
                //UART0_DR_R = '1';    
                #endif
            }
            else if(mic2Samples[rolledIdx] > (mic2SamplesAvg + threshold))
            {
                aoAFSM = PEAKS_FSM;
                wherePeek = 2;
                peekTimeout = adcTicks;
                peekDetectionOrder[0].order = 2;
                peekDetectionOrder[0].timer = adcTicks;
                peekDetectionOrder[0].index = rolledIdx;
                orderIdx = 1;
                threeSampleDetCount = 1;
                #ifdef DEBUG
                //UART0_DR_R = '2';
                #endif
            }
            else if(mic3Samples[rolledIdx] > (mic3SamplesAvg + threshold))
            {
                aoAFSM = PEAKS_FSM;
                wherePeek = 3;
                peekTimeout = adcTicks;
                peekDetectionOrder[0].order = 3;
                peekDetectionOrder[0].timer = adcTicks;
                peekDetectionOrder[0].index = rolledIdx;
                orderIdx = 1;
                threeSampleDetCount = 1;
                #ifdef DEBUG
                //UART0_DR_R = '3';
                #endif
            }
            else
            {
                /* Do Nothing */
            }
        }
        
        if(aoAFSM == AVERAGE_FSM) {
        mic1SamplesAvg +=   mic1Samples[idx];
        mic2SamplesAvg +=   mic2Samples[idx];
        mic3SamplesAvg +=   mic3Samples[idx];

        mic1SamplesAvg +=   mic1Samples[idx - 1];
        mic2SamplesAvg +=   mic2Samples[idx - 1];
        mic3SamplesAvg +=   mic3Samples[idx - 1];
        }

        idx = (idx+1)%NO_OF_SAMPLES;

        if(aoAFSM == AVERAGE_FSM && !hysteresisFlag)
        {
            if((adcTicks%(timeConstant*3)) == 0) {
                mic3SamplesAvg /= NO_OF_SAMPLES;
            }
            else if((adcTicks%(timeConstant*2)) == 0) {
                mic2SamplesAvg /= NO_OF_SAMPLES;
            }
            else if((adcTicks%(timeConstant*1)) == 0) {
                mic1SamplesAvg /= NO_OF_SAMPLES;
            }
        }


        #elif FILTER == IIRFILTER
        //} else {

            mic1SamplesAvg = (alpha* ADC0_SSFIFO0_R) + (alpha - 1)*mic1SamplesAvg;
            mic2SamplesAvg = (alpha* ADC0_SSFIFO0_R) + (alpha - 1)*mic2SamplesAvg;
            mic3SamplesAvg = (alpha* ADC0_SSFIFO0_R) + (alpha - 1)*mic3SamplesAvg;

            mic1SamplesAvg = (alpha* ADC0_SSFIFO0_R) + (alpha - 1)*mic1SamplesAvg;
            mic2SamplesAvg = (alpha* ADC0_SSFIFO0_R) + (alpha - 1)*mic2SamplesAvg;
            mic3SamplesAvg = (alpha* ADC0_SSFIFO0_R) + (alpha - 1)*mic3SamplesAvg;
        //}
        #endif
    
        
    #endif

    #ifdef DMAENABLED
        if(UDMA_CHIS_R & (1<<14)) {
            UART0_DR_R = 'D';
            UDMA_CHIS_R = (1<<14);
        }
    #endif

    ++adcTicks;
    ADC0_ISC_R = 0x1;
}

void audioProcess(void)
{
    char str[50];
    static bool startTimers = false;
    static uint32_t holdOffTemp = 0;
    static uint32_t hysteresisTemp = 0;
    
    if(startTimers)
    {
        if( (adcTicks - holdOffTemp)  >= (uint32_t)(holdOff*1000))
        {
            holdOffFlag = false;
        }

        if( (adcTicks - hysteresisTemp)  >= (uint32_t)(hysteresis*1000))
        {
            hysteresisFlag = false;
        }
    }

    switch(aoAFSM)
    {
        case AVERAGE_FSM:
        {
            //holdOffFlag = false;
            //hysteresisFlag = false;
            wherePeek = 0;
            orderIdx = 0;
            secondPeek = 0;

            #if 0
            if(partialSets) {
                snprintf(str,sizeof(str),"Mic1 Avg: %"PRIu32"\n",mic1SamplesAvg);
                //putsUart0(str);

                snprintf(str,sizeof(str),"Mic2 Avg: %"PRIu32"\n",mic2SamplesAvg);
                //putsUart0(str);

                snprintf(str,sizeof(str),"Mic3 Avg: %"PRIu32"\n",mic3SamplesAvg);
                //putsUart0(str);

                snprintf(str,sizeof(str),"Partial Peek detected at: %"PRIu8"\n",wherePeek);
                putsUart0(str);

                putsUart0("***********************************************************\n");
                snprintf(str,sizeof(str),"3 sample detect: %"PRIu8"\n",threeSampleDetCount);
                putsUart0(str);
                putsUart0("***********************************************************\n");

                snprintf(str,sizeof(str),"Threshold: %"PRIu32"\n",threshold);
                putsUart0(str);

                snprintf(str,sizeof(str),"BackOff: %"PRIu16"\n",backOff);
                putsUart0(str);
                waitMicrosecond(10000);
            }
            #endif
        }
        break;

        case PEAKS_FSM:
        {
            #if 1
            if(partialSets) {
                snprintf(str,sizeof(str),"Partial peek detected at %"PRIu8"\n",wherePeek);
                putsUart0(str);

                #if 1
                putsUart0("***********************************************************\n");
                snprintf(str,sizeof(str),"3 sample detect: %"PRIu8"\n",threeSampleDetCount);
                putsUart0(str);
                putsUart0("***********************************************************\n");
                #endif
            }
            #endif

            if(threeSampleDetCount == 3) {   

                // only after valid event
                startTimers = true;
                holdOffTemp = adcTicks;
                hysteresisTemp = adcTicks;
                threeSampleDetCount = 0;
                secondPeek = 0;
                holdOffFlag = true;
                hysteresisFlag = true;
                calcAoa();
                aoAFSM = AVERAGE_FSM;
            }    
        }
        break;

        default: break;
    }
}
void calcAoa(void)
{
    char str[50];
    uint8_t i;

    switch(peekDetectionOrder[0].order)
    {
        case 1:
        {
            if(peekDetectionOrder[1].order > peekDetectionOrder[2].order) {
                if(peekDetectionOrder[2].timer - peekDetectionOrder[1].timer > 10) {
                    aoaValue = (angles[peekDetectionOrder[0].order] + (uint16_t)(constK * (peekDetectionOrder[2].timer - peekDetectionOrder[1].timer)));
                    aoaValue %= 360;
                }
                else
                {
                    aoaValue = (angles[peekDetectionOrder[0].order] + (uint16_t)((peekDetectionOrder[2].timer - peekDetectionOrder[1].timer)));
                    aoaValue %= 360;
                }

            } else {
                if(peekDetectionOrder[2].timer - peekDetectionOrder[1].timer > 10) {
                    aoaValue = (angles[peekDetectionOrder[0].order] - (uint16_t)(constK * (peekDetectionOrder[2].timer - peekDetectionOrder[1].timer)));
                    aoaValue %= 360;
                } else {
                    aoaValue = (angles[peekDetectionOrder[0].order] - (uint16_t)((peekDetectionOrder[2].timer - peekDetectionOrder[1].timer)));
                    aoaValue %= 360;
                }
            }
        }break;

        case 2:
        {
            if(peekDetectionOrder[1].order < peekDetectionOrder[2].order) {
                if(peekDetectionOrder[2].timer - peekDetectionOrder[1].timer > 10) {
                    aoaValue = (angles[peekDetectionOrder[0].order] + (uint16_t)(constK * (peekDetectionOrder[2].timer - peekDetectionOrder[1].timer)));
                    aoaValue %= 360;
                }
                else
                {
                    aoaValue = (angles[peekDetectionOrder[0].order] + (uint16_t)((peekDetectionOrder[2].timer - peekDetectionOrder[1].timer)));
                    aoaValue %= 360;
                }

            } else {
                if(peekDetectionOrder[2].timer - peekDetectionOrder[1].timer > 10) {
                    aoaValue = (angles[peekDetectionOrder[0].order] - (uint16_t)(constK * (peekDetectionOrder[2].timer - peekDetectionOrder[1].timer)));
                    aoaValue %= 360;
                } else {
                    aoaValue = (angles[peekDetectionOrder[0].order] - (uint16_t)((peekDetectionOrder[2].timer - peekDetectionOrder[1].timer)));
                    aoaValue %= 360;
                }
            }

        }break;

        case 3:
        {
            if(peekDetectionOrder[1].order > peekDetectionOrder[2].order) {
                if(peekDetectionOrder[2].timer - peekDetectionOrder[1].timer > 10) {
                    aoaValue = (angles[peekDetectionOrder[0].order] + (uint16_t)(constK * (peekDetectionOrder[2].timer - peekDetectionOrder[1].timer)));
                    aoaValue %= 360;
                }
                else
                {
                    aoaValue = (angles[peekDetectionOrder[0].order] + (uint16_t)((peekDetectionOrder[2].timer - peekDetectionOrder[1].timer)));
                    aoaValue %= 360;
                }

            } else {
                if(peekDetectionOrder[2].timer - peekDetectionOrder[1].timer > 10) {
                    aoaValue = (angles[peekDetectionOrder[0].order] - (uint16_t)(constK * (peekDetectionOrder[2].timer - peekDetectionOrder[1].timer)));
                    aoaValue %= 360;
                } else {
                    aoaValue = (angles[peekDetectionOrder[0].order] - (uint16_t)((peekDetectionOrder[2].timer - peekDetectionOrder[1].timer)));
                    aoaValue %= 360;
                }
            }

        }break;

        default:break;
    }

    if(tdoaEnable) {
    snprintf(str,sizeof(str),"peek detected at %"PRIu8"\n",peekDetectionOrder[0].order);
    putsUart0(str);
    putsUart0("Peeks order\n");
    for(i = 0; i < 3; i++) {
        snprintf(str,sizeof(str),"%"PRIu8" %"PRIu32"\n",peekDetectionOrder[i].order, peekDetectionOrder[i].timer);
        putsUart0(str);
    }
    }
    if(alwaysEventAoa) {
        snprintf(str,sizeof(str),"AOA : %"PRIu16"\n",aoaValue);
        putsUart0(str);
        setColorWheel(aoaValue);
    }
}

void setColorWheel(uint16_t angle)
{
    if( angle >= 0 && angle <= 30) {
        setRgbColor(1023, 0, 0); // red
    } else if ( angle > 30 && angle <= 60) {
        setRgbColor(1023, 384, 0); //orange
    } else if (angle > 60 && angle <= 90) {
        setRgbColor(1023, 1023, 8); //yellow
    } else if (angle > 90 && angle <= 150) {
        setRgbColor(0, 1023, 0);    //green
    } else if (angle > 150 && angle <= 210) {
        setRgbColor(0, 1023, 1023); //cyan
    } else if (angle > 210 && angle <= 270) {
        setRgbColor(0, 0, 1023);    //blue
    } else if (angle > 270 && angle <= 330) {
        setRgbColor(1023, 0, 1023);    //magenta
    } else if (angle > 330 && angle <= 360) {
        setRgbColor(1023, 0, 0);    //red
    } else {
        /* Do nothing */
    }
}

void calculateAvgs(void)
{
	int roldIdx = 0;

    if(aoAFSM == PEAKS_FSM)
    {
        #if 1
        if((adcTicks - peekTimeout) >= 300)
        {
            if(partialSets) {
                char str[50];
                uint8_t i;

                snprintf(str,sizeof(str),"Partial peek detected at %"PRIu8"\n",wherePeek);
                putsUart0(str);

                //putsUart0("***********************************************************\n");
                snprintf(str,sizeof(str),"3 sample detect: %"PRIu8"\n",threeSampleDetCount);
                putsUart0(str);
                //putsUart0("***********************************************************\n");
                
                putsUart0("Peeks order\n");
                for(i = 0; i < 3; i++) {
                    snprintf(str,sizeof(str),"%"PRIu8" %"PRIu32"\n",peekDetectionOrder[i].order, peekDetectionOrder[i].timer);
                    putsUart0(str);
                }
                putsUart0("Timeout");
            }

            peekTimeout = 0;
            orderIdx = 0;
            threeSampleDetCount = 0;
            wherePeek = 0;
            secondPeek = 0;
		    aoAFSM = AVERAGE_FSM;
            
            return ;
        }
        #endif

        roldIdx = (peekDetectionOrder[0].index);
        roldIdx = (roldIdx + 1) % NO_OF_SAMPLES;

        if(roldIdx < 0) {
            roldIdx = (NO_OF_SAMPLES - 1);
        }

        if((wherePeek == 1) && (threeSampleDetCount <3) )
        {
            if(((secondPeek == 0) || (secondPeek == 2)) && (mic2Samples[roldIdx] > (mic2SamplesAvg + threshold - backOff)))
            {
                peekDetectionOrder[orderIdx].order = 2;
                peekDetectionOrder[orderIdx].timer = adcTicks;
                orderIdx = (orderIdx + 1)% 3;
                ++threeSampleDetCount;
                (peekDetectionOrder[0].index) = (peekDetectionOrder[0].index + 1) % NO_OF_SAMPLES;
                secondPeek = 3; // go to next peek
            }

            if(((secondPeek == 0) || (secondPeek == 3)) && (mic3Samples[roldIdx] > (mic3SamplesAvg + threshold - backOff)))
            {
                peekDetectionOrder[orderIdx].order = 3;
                peekDetectionOrder[orderIdx].timer = adcTicks;
                orderIdx = (orderIdx + 1)% 3;
                ++threeSampleDetCount;
                (peekDetectionOrder[0].index) = (peekDetectionOrder[0].index + 1) % NO_OF_SAMPLES;
                secondPeek = 2; // go to next peek
            }
        }

        if((wherePeek == 2) && (threeSampleDetCount <3))
        {
            if(((secondPeek == 0) || (secondPeek == 3)) && (mic3Samples[roldIdx] > (mic3SamplesAvg + threshold - backOff)))
            {
                peekDetectionOrder[orderIdx].order = 3;
                peekDetectionOrder[orderIdx].timer = adcTicks;
                orderIdx = (orderIdx + 1)% 3;
                ++threeSampleDetCount;
                (peekDetectionOrder[0].index) = (peekDetectionOrder[0].index + 1) % NO_OF_SAMPLES;
                secondPeek = 1;
            }
            if(((secondPeek == 0) || (secondPeek == 1)) && (mic1Samples[roldIdx] > (mic1SamplesAvg + threshold - backOff)))
            {
                peekDetectionOrder[orderIdx].order = 1;
                peekDetectionOrder[orderIdx].timer = adcTicks;
                orderIdx = (orderIdx + 1)% 3;
                ++threeSampleDetCount;
                (peekDetectionOrder[0].index) = (peekDetectionOrder[0].index + 1) % NO_OF_SAMPLES;
                secondPeek = 3;
            } 
        }

        if((wherePeek == 3) && (threeSampleDetCount <3))
        {
            if(((secondPeek == 0) || (secondPeek == 1)) && (mic1Samples[roldIdx] > (mic1SamplesAvg + threshold - backOff)))
            {
                peekDetectionOrder[orderIdx].order = 1;
                peekDetectionOrder[orderIdx].timer = adcTicks;
                orderIdx = (orderIdx + 1)% 3;
                ++threeSampleDetCount;
                (peekDetectionOrder[0].index) = (peekDetectionOrder[0].index + 1) % NO_OF_SAMPLES;
                secondPeek = 2;
            }
            if(((secondPeek == 0) || (secondPeek == 2)) && (mic2Samples[roldIdx] > (mic2SamplesAvg + threshold - backOff)))
            {
                peekDetectionOrder[orderIdx].order = 2;
                peekDetectionOrder[orderIdx].timer = adcTicks;
                orderIdx = (orderIdx + 1)% 3;   
                ++threeSampleDetCount;
                (peekDetectionOrder[0].index) = (peekDetectionOrder[0].index + 1) % NO_OF_SAMPLES;
                secondPeek = 1;
            }
        }
    }

}
void initApp(void)
{
    backOff = readEeprom(EEP_BACK_OFF);
    holdOff = readEeprom(EEP_HOLD_OFF);
    hysteresis = readEeprom(EEP_HYSTERESIS);
    timeConstant = readEeprom(EEP_TIME_CONSTANT);
}

void main(void)
{
    initHw();
    initApp();
    

    putsUart0("\n\nStarting AoA project\n");

    while(true) {

        #ifdef DEBUG
        if(ADC0_USTAT_R & ADC_USTAT_UV0) {
            ADC0_USTAT_R = ADC_USTAT_UV0;
        }

        if(ADC0_OSTAT_R & ADC_OSTAT_OV0) {
            ADC0_OSTAT_R = ADC_OSTAT_OV0;
            putsUart0("ADC0 overflow \n");
        }
        #endif

        processShell();
        calculateAvgs();
        audioProcess();
        alwaysEvents();

        #ifdef DMAENABLED
        if(UDMA_ERRCLR_R & 0x1) {
            putsUart0("Bus error\n");
        }
        #endif 

    }
}