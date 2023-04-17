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

uint32_t threshold = 1000;
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

uint8_t threeSampleDetCount = 0;
uint32_t peekTimeout = 0;

#define DEBUG
#define FILTER FIRFILTER

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
    uint32_t* p;

    if((adcTicks%166666) == 0) {
        //togglePinValue(PEAKS_DETECT_LED);
        p = (uint32_t*)PORTF + 1;
        *p ^= 1;
    }
    
    #if 1
        //if(iirFilterEnable == false) {
        
        #if FILTER == FIRFILTER

        mic1Samples[idx] = ADC0_SSFIFO0_R;
        mic2Samples[idx] = ADC0_SSFIFO0_R;
        mic3Samples[idx] = ADC0_SSFIFO0_R;

        mic1SamplesAvg +=   mic1Samples[idx];
        mic2SamplesAvg +=   mic2Samples[idx];
        mic3SamplesAvg +=   mic3Samples[idx];

        idx = (idx+1)%NO_OF_SAMPLES;

        mic1Samples[idx] = ADC0_SSFIFO0_R;
        mic2Samples[idx] = ADC0_SSFIFO0_R;
        mic3Samples[idx] = ADC0_SSFIFO0_R;

        mic1SamplesAvg +=   mic1Samples[idx];
        mic2SamplesAvg +=   mic2Samples[idx];
        mic3SamplesAvg +=   mic3Samples[idx];

        idx = (idx+1)%NO_OF_SAMPLES;

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
            
            if(rolledIdx < 0) {
                rolledIdx = (NO_OF_SAMPLES - 1);
            }

            if(mic1Samples[rolledIdx] > mic1SamplesAvg + threshold)
            {
                aoAFSM = PEAKS_FSM;
                wherePeek = 1;
                peekTimeout = adcTicks;
                peekDetectionOrder[orderIdx].order = 1;
                peekDetectionOrder[orderIdx].timer = adcTicks;
                orderIdx = (orderIdx + 1)% 3;
                ++threeSampleDetCount;
                #ifdef DEBUG
                UART0_DR_R = '1';    
                #endif
            }
            else if(mic2Samples[rolledIdx] > mic2SamplesAvg + threshold)
            {
                aoAFSM = PEAKS_FSM;
                wherePeek = 2;
                peekTimeout = adcTicks;
                peekDetectionOrder[orderIdx].order = 2;
                peekDetectionOrder[orderIdx].timer = adcTicks;
                orderIdx = (orderIdx + 1)% 3;
                ++threeSampleDetCount;
                #ifdef DEBUG
                UART0_DR_R = '2';
                #endif
            }
            else if(mic3Samples[rolledIdx] > mic3SamplesAvg + threshold)
            {
                aoAFSM = PEAKS_FSM;
                wherePeek = 3;
                peekTimeout = adcTicks;
                peekDetectionOrder[orderIdx].order = 3;
                peekDetectionOrder[orderIdx].timer = adcTicks;
                orderIdx = (orderIdx + 1)% 3;
                ++threeSampleDetCount;
                #ifdef DEBUG
                UART0_DR_R = '3';
                #endif
            }
            else
            {
                /* Do Nothing */
            }
        }

        if(aoAFSM == PEAKS_FSM)
        {
            //if((adcTicks - peekTimeout) >= 2*166666) //Timeout of 280 ~ 300us 
            if((adcTicks - peekTimeout) >= 8333) //0.5s
            {
                peekTimeout = 0;
                aoAFSM = AVERAGE_FSM;
                ++adcTicks;
                ADC0_ISC_R = 0x1;
                orderIdx = 0;
                threeSampleDetCount = 0;
                wherePeek = 0;
                #ifdef DEBUG
                UART0_DR_R = 'T';
                #endif
                return ;
            }

            rolledIdx = idx - 1;

            if(rolledIdx < 0) {
                rolledIdx = (NO_OF_SAMPLES - 1);
            }

            if(wherePeek == 1)
            {
                if((mic2Samples[rolledIdx] > mic2SamplesAvg + threshold - backOff))
                {
                    peekDetectionOrder[orderIdx].order = 2;
                    peekDetectionOrder[orderIdx].timer = adcTicks;
                    orderIdx = (orderIdx + 1)% 3;
                    ++threeSampleDetCount;
                }

                if((mic3Samples[rolledIdx] > mic3SamplesAvg + threshold - backOff))
                {
                    peekDetectionOrder[orderIdx].order = 3;
                    peekDetectionOrder[orderIdx].timer = adcTicks;
                    orderIdx = (orderIdx + 1)% 3;
                    ++threeSampleDetCount;
                }
            }

            if(wherePeek == 2)
            {
                if((mic1Samples[rolledIdx] > mic1SamplesAvg + threshold - backOff))
                {
                    peekDetectionOrder[orderIdx].order = 1;
                    peekDetectionOrder[orderIdx].timer = adcTicks;
                    orderIdx = (orderIdx + 1)% 3;
                    ++threeSampleDetCount;
                }
                if((mic3Samples[rolledIdx] > mic3SamplesAvg + threshold - backOff))
                {
                    peekDetectionOrder[orderIdx].order = 3;
                    peekDetectionOrder[orderIdx].timer = adcTicks;
                    orderIdx = (orderIdx + 1)% 3;
                    ++threeSampleDetCount;
                }
            }

            if(wherePeek == 3)
            {
                if((mic1Samples[rolledIdx] > mic1SamplesAvg + threshold - backOff))
                {
                    peekDetectionOrder[orderIdx].order = 1;
                    peekDetectionOrder[orderIdx].timer = adcTicks;
                    orderIdx = (orderIdx + 1)% 3;
                    ++threeSampleDetCount;
                }
                if((mic2Samples[rolledIdx] > mic2SamplesAvg + threshold - backOff))
                {
                    peekDetectionOrder[orderIdx].order = 2;
                    peekDetectionOrder[orderIdx].timer = adcTicks;
                    orderIdx = (orderIdx + 1)% 3;   
                    ++threeSampleDetCount;
                }
            }
        }
    #endif
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
        if( (adcTicks - holdOffTemp)  >= holdOff*1000)
        {
            holdOffFlag = true;
        }

        if( (adcTicks - hysteresisTemp)  >= hysteresis*1000)
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
            threeSampleDetCount = 0;
            wherePeek = 0;
            orderIdx = 0;

            if(partialSets) {
                snprintf(str,sizeof(str),"Mic1 Avg: %"PRIu32"\n",mic1SamplesAvg);
                putsUart0(str);

                snprintf(str,sizeof(str),"Mic2 Avg: %"PRIu32"\n",mic2SamplesAvg);
                putsUart0(str);

                snprintf(str,sizeof(str),"Mic3 Avg: %"PRIu32"\n",mic3SamplesAvg);
                putsUart0(str);

                snprintf(str,sizeof(str),"Threshold: %"PRIu32"\n",threshold);
                putsUart0(str);

                snprintf(str,sizeof(str),"BackOff: %"PRIu16"\n",backOff);
                putsUart0(str);
                waitMicrosecond(100000);
            }
        }
        break;

        case PEAKS_FSM:
        {
            //setPinValue(PEAKS_DETECT_LED,1);
            
            if(partialSets) {
                snprintf(str,sizeof(str),"peek detected at %"PRIu8"\n",wherePeek);
                putsUart0(str);

                #if 0
                putsUart0("***********************************************************\n");
                snprintf(str,sizeof(str),"3 sample detect: %"PRIu8"\n",threeSampleDetCount);
                putsUart0(str);
                putsUart0("***********************************************************\n");
                #endif
            }
            
            if(tdoaEnable && threeSampleDetCount >= 3) {
            //if(tdoaEnable &&  orderIdx == 0) {    
                snprintf(str,sizeof(str),"peek detected at %"PRIu8"\n",wherePeek);
                putsUart0(str);

                putsUart0("Peeks order\n");
                for(i = 0; i < 3; i++) {
                    snprintf(str,sizeof(str),"%"PRIu8" %"PRIu32"\n",peekDetectionOrder[i].order, peekDetectionOrder[i].timer);
                    putsUart0(str);
                }

                // only after valid event
                startTimers = true;
                holdOffTemp = adcTicks;
                hysteresisTemp = adcTicks;
                aoAFSM = AVERAGE_FSM;
                threeSampleDetCount = 0;
            }    
        }
        break;

        default: break;
    }
}

void calculateAvgs(void)
{
    threshold = (1.50 * (mic1SamplesAvg + mic2SamplesAvg + mic3SamplesAvg)) / 3;
    backOff = (uint16_t)(0.75 * threshold);
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

    //initDma();

    //startPeriodicTimer(adcFreeRunTimer,1000); //1 ms

    putsUart0("Starting AoA project\n");

    while(true) {
        processShell();
        calculateAvgs();
        audioProcess();
        alwaysEvents();

        /*
        if(UDMA_ERRCLR_R & 0x1) {
            putsUart0("Bus error\n");
        } 
        */
    }
}
