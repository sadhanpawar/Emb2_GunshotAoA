/*
 * userInterface.c
 *
 *  Created on: Apr 8, 2023
 *      Author: sadhan
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <inttypes.h>
#include "tm4c123gh6pm.h"
#include "userInterface.h"
#include "main.h"
#include "uart0.h"
#include "eeprom.h"
#include "wait.h"
#include <math.h>

uint8_t count = 0;
char strInput[MAX_CHARS+1];
char* token;
bool alwaysEventAoa = true;

void processShell()
{
    char str[50] = {0};
    bool end;
    char c;

    if (kbhitUart0())
    {
        c = getcUart0();

        end = (c == 13) || (count == MAX_CHARS);
        if (!end)
        {
            if ((c == 8 || c == 127) && count > 0)
                count--;
            if (c >= ' ' && c < 127)
                strInput[count++] = c;
        }
        else
        {
            strInput[count] = '\0';
            count = 0;
            token = strtok(strInput, " ");

            if (strcmp(token, "reset") == 0)
            {
                NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            }
            if (strcmp(token, "average") == 0)
            {
                displayAvgs();
            }
            if (strcmp(token, "tc") == 0)
            {
                token = strtok(NULL, " ");
                if (strlen(token) > 0)
                {
                    setTimeConstant(asciiToUint16(token));
                }
                else
                {
                    snprintf(str,sizeof(str),"%"PRIu16"\n",timeConstant);
                    putsUart0(str);
                }
            }
            if (strcmp(token, "backoff") == 0)
            {
                token = strtok(NULL, " ");
                if (strlen(token) > 0)
                {
                    setBackOff(asciiToUint16(token));
                }
                else
                {
                    snprintf(str,sizeof(str),"%"PRIu16"\n",backOff);
                    putsUart0(str);
                }
            }
            if (strcmp(token, "holdoff") == 0)
            {
                token = strtok(NULL, " ");
                if (strlen(token) > 0)
                {
                    setHoldoff(asciiToUint16(token));
                }
                else
                {
                    snprintf(str,sizeof(str),"%"PRIu16"\n",holdOff);
                    putsUart0(str);
                }
            }
            if (strcmp(token, "hysteresis") == 0)
            {
                token = strtok(NULL, " ");
                if (strlen(token) > 0)
                {
                    setHysteresis(asciiToUint16(token));
                }
                else
                {
                    snprintf(str,sizeof(str),"%"PRIu16"\n",hysteresis);
                    putsUart0(str);
                }
            }
            if (strcmp(token, "aoa") == 0)
            {
                token = strtok(NULL, " ");
                if (strlen(token) > 0)
                {
                    if (strcmp(token, "always") == 0)
                    {
                        alwaysEventAoa = true;
                    }
                }
                else
                {
                    snprintf(str,sizeof(str),"%"PRIu16"\n",aoaValue);
                    putsUart0(str);  
                }
            }
            
            if (strcmp(token, "tdoa") == 0)
            {
                token = strtok(NULL, " ");
                if (strcmp(token, "on") == 0)
                {
                    tdoaEnable = true;
                }
                else if(strcmp(token, "off") == 0)
                {
                    tdoaEnable = false;
                }
            }
            if (strcmp(token, "fail") == 0)
            {
                token = strtok(NULL, " ");
                if (strcmp(token, "on") == 0)
                {
                    partialSets = true;
                }
                else if(strcmp(token, "off") == 0)
                {
                    partialSets = false;
                }
            }

            if (strcmp(token, "help") == 0)
            {
                putsUart0("Commands:\r");
                putsUart0("  reset\r");
                putsUart0("  average\r");
                putsUart0("  tc [T]\r");
                putsUart0("  backoff [B]\r");
                putsUart0("  holdoff [H]\r");
                putsUart0("  hysteresis [Y]\r");
                putsUart0("  aoa\r");
                putsUart0("  aoa always\r");
                putsUart0("  tdoa ON|OFF\r");
                putsUart0("  fail ON|OFF\r");
            }
        }
    }
}

uint16_t asciiToUint16(const char str[])
{
    uint16_t data;
    if (str[0] == '0' && tolower(str[1]) == 'x')
        sscanf(str, "%hx", &data);
    else
        sscanf(str, "%hu", &data);
    return data;
}

void displayAvgs(void)
{
    char str[60];
    float voltage = 0;
    float spl = 0;

    voltage = (3.3 * (mic1SamplesAvg + 0.5))/4096;
    spl = 20*log10(voltage) + 44 + 94;
    putsUart0("mic 1: \n");
    snprintf(str,sizeof(str),"raw : %lu, dac: %f, spl : %f\n",mic1SamplesAvg,voltage,spl);
    putsUart0(str);

    voltage = (3.3 * (mic2SamplesAvg + 0.5))/4096;
    spl = 20*log10(voltage) + 44 + 94;
    putsUart0("\nmic 2: \n");
    snprintf(str,sizeof(str),"raw : %lu,dac: %f, spl : %f\n",mic2SamplesAvg,voltage,spl);
    putsUart0(str);

    voltage = (3.3 * (mic3SamplesAvg + 0.5))/4096;
    spl = 20*log10(voltage) + 44 + 94;
    putsUart0("\nmic 3: \n");
    snprintf(str,sizeof(str),"raw : %lu,dac: %f, spl : %f\n",mic3SamplesAvg,voltage,spl);
    putsUart0(str);

    /*
    putsUart0("mic 2 Avg: ");
    snprintf(str,sizeof(str),"%"PRIu32"\n",mic2SamplesAvg);
    putsUart0(str);

    putsUart0("mic 3 Avg: ");
    snprintf(str,sizeof(str),"%"PRIu32"\n",mic3SamplesAvg);
    putsUart0(str);
    */
}

void setTimeConstant(uint16_t value)
{
    timeConstant = value;
    writeEeprom(EEP_TIME_CONSTANT,value);
}

void setBackOff(uint16_t value)
{
    backOff = value;
    writeEeprom(EEP_BACK_OFF,value);
}

void setHoldoff(uint16_t value)
{
    holdOff = value;
    writeEeprom(EEP_HOLD_OFF,value);
}

void setHysteresis(uint16_t value)
{
    hysteresis = value;
    writeEeprom(EEP_HYSTERESIS,value);
}
void alwaysEvents(void)
{
    char str[30] = {0};

    if(alwaysEventAoa) {
        //snprintf(str,sizeof(str),"AOA :%"PRIu16"\n",aoaValue);
        //putsUart0(str);
        //waitMicrosecond(100);
    }

    /*
    if(tdoaEnable) {

    }

    if(partialSets) {

    }
    */

}
