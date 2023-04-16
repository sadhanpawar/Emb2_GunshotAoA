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

bool const iirFilterEnable = false;
uint16_t mic1Samples[NO_OF_SAMPLES] = {0};
uint16_t mic2Samples[NO_OF_SAMPLES] = {0};
uint16_t mic3Samples[NO_OF_SAMPLES] = {0};
uint16_t idx = 0;
bool calAvgNow = false;
const float alpha = 0.9f;

uint32_t mic1SamplesAvg = UINT_MAX;
uint32_t mic2SamplesAvg = UINT_MAX;
uint32_t mic3SamplesAvg = UINT_MAX;

uint32_t threshold = 100;
uint16_t timeConstant = 0;
uint16_t backOff = 0;
uint16_t holdOff = 0;
uint16_t hysteresis = 0;
bool tdoaEnable = false;
bool partialSets = false;
uint16_t aoaValue = 0;

uint8_t wherePeek = 0;

static uint8_t aoAFSM = AVERAGE_FSM;
uint32_t adcTicks = 0;
bool adcTimeConstantFlag = false;
bool holdOffFlag = false;
bool hysteresisFlag = false;

peekDetectOrder_t peekDetectionOrder[3] = {0};
static uint8_t orderIdx = 0;

int rolledIdx = 0;

void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();
    enablePort(PORTF);
    selectPinPushPullOutput(PEAKS_DETECT_LED);

    initAdc0();
    initUart0();
    setUart0BaudRate(115200, 40e6);
    initEeprom();
}

void aoaAdc0InterruptHandler(void)
{
    if((adcTicks%166666) == 0) {
        togglePinValue(PEAKS_DETECT_LED);
    }
    
        if(iirFilterEnable == false) {
        

        mic1SamplesAvg -=   mic1Samples[idx];
        mic2SamplesAvg -=   mic2Samples[idx];
        mic3SamplesAvg -=   mic3Samples[idx];

        mic1Samples[idx] = ADC0_SSFIFO0_R;
        mic2Samples[idx] = ADC0_SSFIFO0_R;
        mic3Samples[idx] = ADC0_SSFIFO0_R;

        mic1SamplesAvg +=   mic1Samples[idx];
        mic2SamplesAvg +=   mic2Samples[idx];
        mic3SamplesAvg +=   mic3Samples[idx];

        idx = (idx+1)%NO_OF_SAMPLES;

        mic1SamplesAvg -=   mic1Samples[idx];
        mic2SamplesAvg -=   mic2Samples[idx];
        mic3SamplesAvg -=   mic3Samples[idx];

        mic1Samples[idx] = ADC0_SSFIFO0_R;
        mic2Samples[idx] = ADC0_SSFIFO0_R;
        mic3Samples[idx] = ADC0_SSFIFO0_R;

        mic1SamplesAvg +=   mic1Samples[idx];
        mic2SamplesAvg +=   mic2Samples[idx];
        mic3SamplesAvg +=   mic3Samples[idx];

        idx = (idx+1)%NO_OF_SAMPLES;

        } else {

            mic1SamplesAvg = (alpha* ADC0_SSFIFO0_R) + (alpha - 1)*mic1SamplesAvg;
            mic2SamplesAvg = (alpha* ADC0_SSFIFO0_R) + (alpha - 1)*mic2SamplesAvg;
            mic3SamplesAvg = (alpha* ADC0_SSFIFO0_R) + (alpha - 1)*mic3SamplesAvg;

            mic1SamplesAvg = (alpha* ADC0_SSFIFO0_R) + (alpha - 1)*mic1SamplesAvg;
            mic2SamplesAvg = (alpha* ADC0_SSFIFO0_R) + (alpha - 1)*mic2SamplesAvg;
            mic3SamplesAvg = (alpha* ADC0_SSFIFO0_R) + (alpha - 1)*mic3SamplesAvg;
        }

    
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

        if((aoAFSM == AVERAGE_FSM && !holdOffFlag))
        {
            rolledIdx = idx - 1;
            
//            if(rolledIdx < 0) {
//                rolledIdx = (NO_OF_SAMPLES - 1);
//            }

            if(mic1Samples[idx] > mic1SamplesAvg + threshold)
            {
                aoAFSM = PEAKS_FSM;
                wherePeek = 1;
                peekDetectionOrder[orderIdx].order = 1;
                peekDetectionOrder[orderIdx].timer = adcTicks;
                orderIdx = (orderIdx + 1)% 3;
            }
            else if(mic2Samples[idx] > mic2SamplesAvg + threshold)
            {
                aoAFSM = PEAKS_FSM;
                wherePeek = 2;
                peekDetectionOrder[orderIdx].order = 2;
                peekDetectionOrder[orderIdx].timer = adcTicks;
                orderIdx = (orderIdx + 1)% 3;
            }
            else if(mic3Samples[idx] > mic3SamplesAvg + threshold)
            {
                aoAFSM = PEAKS_FSM;
                wherePeek = 3;
                peekDetectionOrder[orderIdx].order = 3;
                peekDetectionOrder[orderIdx].timer = adcTicks;
                orderIdx = (orderIdx + 1)% 3;
            }
            else
            {
                /* Do Nothing */
            }
        }

        if(aoAFSM == PEAKS_FSM)
        {
            if((adcTicks % 300) == 0) //Timeout of 280 ~ 300us 
            {
                aoAFSM = AVERAGE_FSM;
                ++adcTicks;
                ADC0_ISC_R = 0x1;
                orderIdx = 0;
                return ;
            }

            if(wherePeek == 1)
            {
                if((mic2Samples[idx] > mic2SamplesAvg + threshold - backOff))
                {
                    peekDetectionOrder[orderIdx].order = 2;
                    peekDetectionOrder[orderIdx].timer = adcTicks;
                    orderIdx = (orderIdx + 1)% 3;
                }

                if((mic3Samples[idx] > mic3SamplesAvg + threshold - backOff))
                {
                    peekDetectionOrder[orderIdx].order = 3;
                    peekDetectionOrder[orderIdx].timer = adcTicks;
                    orderIdx = (orderIdx + 1)% 3;
                }
            }

            if(wherePeek == 2)
            {
                if((mic1Samples[idx] > mic1SamplesAvg + threshold - backOff))
                {
                    peekDetectionOrder[orderIdx].order = 1;
                    peekDetectionOrder[orderIdx].timer = adcTicks;
                    orderIdx = (orderIdx + 1)% 3;
                }
                if((mic3Samples[idx] > mic3SamplesAvg + threshold - backOff))
                {
                    peekDetectionOrder[orderIdx].order = 3;
                    peekDetectionOrder[orderIdx].timer = adcTicks;
                    orderIdx = (orderIdx + 1)% 3;
                }
            }

            if(wherePeek == 3)
            {
                if((mic1Samples[idx] > mic1SamplesAvg + threshold - backOff))
                {
                    peekDetectionOrder[orderIdx].order = 1;
                    peekDetectionOrder[orderIdx].timer = adcTicks;
                    orderIdx = (orderIdx + 1)% 3;
                }
                if((mic2Samples[idx] > mic2SamplesAvg + threshold - backOff))
                {
                    peekDetectionOrder[orderIdx].order = 2;
                    peekDetectionOrder[orderIdx].timer = adcTicks;
                    orderIdx = (orderIdx + 1)% 3;   
                }
            }
        }
        
    ++adcTicks;
    ADC0_ISC_R = 0x1;
}

void audioProcess(void)
{
    char str[50];
    uint8_t i;
    static bool startTimers = false;
    static uint32_t holdOffTemp = 0;
    static uint32_t hysteresisTemp = 0;
    
    if(startTimers)
    {
        if( (adcTicks - holdOffTemp)  == holdOff)
        {
            holdOffFlag = true;
        }

        if( (adcTicks - hysteresisTemp)  == hysteresis)
        {
            hysteresisFlag = true;
        }
    }

    switch(aoAFSM)
    {
        case AVERAGE_FSM:
        {
            //setPinValue(PEAKS_DETECT_LED,0);
            memset(peekDetectionOrder,0,sizeof(peekDetectionOrder));
            holdOffFlag = false;
            hysteresisFlag = false;
            startTimers = false;

//            snprintf(str,sizeof(str),"Mic1 Avg: %"PRIu32"\n",mic1SamplesAvg);
//            putsUart0(str);
//
//            snprintf(str,sizeof(str),"Mic2 Avg: %"PRIu32"\n",mic2SamplesAvg);
//            putsUart0(str);
//
//            snprintf(str,sizeof(str),"Mic3 Avg: %"PRIu32"\n",mic3SamplesAvg);
//            putsUart0(str);
//            waitMicrosecond(100000);
        }
        break;

        case PEAKS_FSM:
        {
            //setPinValue(PEAKS_DETECT_LED,1);
            snprintf(str,sizeof(str),"peek detected at %"PRIu8"\n",wherePeek);
            putsUart0(str);
            
            if(orderIdx == 0) {
                putsUart0("Peeks order\n");
                for(i = 0; i < 3; i++) {
                    snprintf(str,sizeof(str),"%"PRIu8" %"PRIu32"\n",peekDetectionOrder[i].order, peekDetectionOrder[i].timer);
                    putsUart0(str);
                }
                startTimers = true;
                holdOffTemp = adcTicks;
                hysteresisTemp = adcTicks;
                aoAFSM == AVERAGE_FSM;
            }
        }
        break;

        default: break;
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

    //startPeriodicTimer(adcFreeRunTimer,1000); //1 ms

    putsUart0("Starting AoA project\n");

    while(true) {
        processShell();
        audioProcess();
        alwaysEvents();
    }
}
