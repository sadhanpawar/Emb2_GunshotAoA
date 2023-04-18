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

uint32_t threshold = 300;
uint16_t timeConstant = 0;
uint16_t backOff = 0;
uint16_t holdOff = 0;
uint16_t hysteresis = 0;
bool tdoaEnable = true;
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

uint32_t const constK = 2;

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
}

void aoaAdc0InterruptHandler(void)
{
    #if 1
        
        #if FILTER == FIRFILTER

        mic1Samples[idx] = ADC0_SSFIFO0_R;
        mic2Samples[idx] = ADC0_SSFIFO0_R;
        mic3Samples[idx] = ADC0_SSFIFO0_R;

        if(aoAFSM == AVERAGE_FSM) {
        mic1SamplesAvg +=   mic1Samples[idx];
        mic2SamplesAvg +=   mic2Samples[idx];
        mic3SamplesAvg +=   mic3Samples[idx];
        }

        idx = (idx+1)%NO_OF_SAMPLES;

        mic1Samples[idx] = ADC0_SSFIFO0_R;
        mic2Samples[idx] = ADC0_SSFIFO0_R;
        mic3Samples[idx] = ADC0_SSFIFO0_R;

        if(aoAFSM == AVERAGE_FSM) {
        mic1SamplesAvg +=   mic1Samples[idx];
        mic2SamplesAvg +=   mic2Samples[idx];
        mic3SamplesAvg +=   mic3Samples[idx];
        }

        idx = (idx+1)%NO_OF_SAMPLES;

        trash = ADC0_SSFIFO0_R;
        trash = ADC0_SSFIFO0_R;

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
                peekDetectionOrder[orderIdx].index = rolledIdx;
                orderIdx = (orderIdx + 1)% 3;
                ++threeSampleDetCount;
                #ifdef DEBUG
                //UART0_DR_R = '1';    
                #endif
            }
            else if(mic2Samples[rolledIdx] > mic2SamplesAvg + threshold)
            {
                aoAFSM = PEAKS_FSM;
                wherePeek = 2;
                peekTimeout = adcTicks;
                peekDetectionOrder[orderIdx].order = 2;
                peekDetectionOrder[orderIdx].timer = adcTicks;
                peekDetectionOrder[orderIdx].index = rolledIdx;
                orderIdx = (orderIdx + 1)% 3;
                ++threeSampleDetCount;
                #ifdef DEBUG
                //UART0_DR_R = '2';
                #endif
            }
            else if(mic3Samples[rolledIdx] > mic3SamplesAvg + threshold)
            {
                aoAFSM = PEAKS_FSM;
                wherePeek = 3;
                peekTimeout = adcTicks;
                peekDetectionOrder[orderIdx].order = 3;
                peekDetectionOrder[orderIdx].timer = adcTicks;
                peekDetectionOrder[orderIdx].index = rolledIdx;
                orderIdx = (orderIdx + 1)% 3;
                ++threeSampleDetCount;
                #ifdef DEBUG
                //UART0_DR_R = '3';
                #endif
            }
            else
            {
                /* Do Nothing */
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
            holdOffFlag = false;
            hysteresisFlag = false;
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
                waitMicrosecond(10000);
            }
        }
        break;

        case PEAKS_FSM:
        {
            
            if(partialSets) {
                snprintf(str,sizeof(str),"peek detected at %"PRIu8"\n",wherePeek);
                putsUart0(str);

                #if 1
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
                aoaValue = 60 + constK * (peekDetectionOrder[2].timer - peekDetectionOrder[1].timer);
            }    
        }
        break;

        default: break;
    }
}

void calculateAvgs(void)
{
    //threshold = (1.50 * (mic1SamplesAvg + mic2SamplesAvg + mic3SamplesAvg)) / 3;
    //backOff = (uint16_t)(0.50 * threshold);

    if(aoAFSM == AVERAGE_FSM && !hysteresisFlag)
    {
        //threshold = (1.5 * (mic1SamplesAvg + mic2SamplesAvg + mic3SamplesAvg)) / 3;
        #if 0
        if((adcTicks%(timeConstant*3)) == 0) {
            mic3SamplesAvg /= NO_OF_SAMPLES;
            //threshold = (1.50 * (mic1SamplesAvg + mic2SamplesAvg + mic3SamplesAvg)) / 3;
            //backOff = (uint16_t)(0.50 * threshold);
        }
        else if((adcTicks%(timeConstant*2)) == 0) {
            mic2SamplesAvg /= NO_OF_SAMPLES;
            //threshold = (1.50 * (mic1SamplesAvg + mic2SamplesAvg + mic3SamplesAvg)) / 3;
            //backOff = (uint16_t)(0.50 * threshold);
        }
        else if((adcTicks%(timeConstant*1)) == 0) {
            mic1SamplesAvg /= NO_OF_SAMPLES;
            //threshold = (1.50 * (mic1SamplesAvg + mic2SamplesAvg + mic3SamplesAvg)) / 3;
            //backOff = (uint16_t)(0.50 * threshold);
        }
        #endif     
    }

    #if 0
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
            peekDetectionOrder[orderIdx].index = rolledIdx;
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
            peekDetectionOrder[orderIdx].index = rolledIdx;
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
            peekDetectionOrder[orderIdx].index = rolledIdx;
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
    #endif

    if(aoAFSM == PEAKS_FSM)
    {
        //if((adcTicks - peekTimeout) >= 2*166666) //Timeout of 280 ~ 300us 
        #if 1
        if((adcTicks - peekTimeout) >= 166666) //1s working
        {
            peekTimeout = 0;
            aoAFSM = AVERAGE_FSM;
            orderIdx = 0;
            threeSampleDetCount = 0;
            wherePeek = 0;
            #ifdef DEBUG
            UART0_DR_R = 'T';
            #endif
            return ;
        }
        #endif

        rolledIdx = (peekDetectionOrder[0].index);
        rolledIdx = (rolledIdx + 1) % NO_OF_SAMPLES;

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
                (peekDetectionOrder[0].index) = (peekDetectionOrder[0].index + 1) % NO_OF_SAMPLES;
            }

            if((mic3Samples[rolledIdx] > mic3SamplesAvg + threshold - backOff))
            {
                peekDetectionOrder[orderIdx].order = 3;
                peekDetectionOrder[orderIdx].timer = adcTicks;
                orderIdx = (orderIdx + 1)% 3;
                ++threeSampleDetCount;
                (peekDetectionOrder[0].index) = (peekDetectionOrder[0].index + 1) % NO_OF_SAMPLES;
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
                (peekDetectionOrder[0].index) = (peekDetectionOrder[0].index + 1) % NO_OF_SAMPLES;
            }
            if((mic3Samples[rolledIdx] > mic3SamplesAvg + threshold - backOff))
            {
                peekDetectionOrder[orderIdx].order = 3;
                peekDetectionOrder[orderIdx].timer = adcTicks;
                orderIdx = (orderIdx + 1)% 3;
                ++threeSampleDetCount;
                (peekDetectionOrder[0].index) = (peekDetectionOrder[0].index + 1) % NO_OF_SAMPLES;
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
                (peekDetectionOrder[0].index) = (peekDetectionOrder[0].index + 1) % NO_OF_SAMPLES;
            }
            if((mic2Samples[rolledIdx] > mic2SamplesAvg + threshold - backOff))
            {
                peekDetectionOrder[orderIdx].order = 2;
                peekDetectionOrder[orderIdx].timer = adcTicks;
                orderIdx = (orderIdx + 1)% 3;   
                ++threeSampleDetCount;
                (peekDetectionOrder[0].index) = (peekDetectionOrder[0].index + 1) % NO_OF_SAMPLES;
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
    

    putsUart0("Starting AoA project\n");

    while(true) {

        #ifdef DEBUG
        if(ADC0_USTAT_R & ADC_USTAT_UV0) {
            ADC0_USTAT_R = ADC_USTAT_UV0;
            //putsUart0("ADC0 underflow \n");
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
