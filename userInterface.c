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

uint8_t count = 0;
char strInput[MAX_CHARS+1];
char* token;
bool alwaysEventAoa = false;

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
                    snprintf(str,sizeof(str),"%"PRIu16"ticks\n",timeConstant);
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
                snprintf(str,sizeof(str),"%"PRIu16"\n",aoaValue);
                putsUart0(str);  
            }
            if (strcmp(token, "aoa always") == 0)
            {
                alwaysEventAoa = true;
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
    char str[30];

    putsUart0("mic 1 Avg: ");
    snprintf(str,sizeof(str),"%"PRIu32"\n",mic1SamplesAvg);
    putsUart0(str);

    putsUart0("mic 2 Avg: ");
    snprintf(str,sizeof(str),"%"PRIu32"\n",mic2SamplesAvg);
    putsUart0(str);

    putsUart0("mic 3 Avg: ");
    snprintf(str,sizeof(str),"%"PRIu32"\n",mic3SamplesAvg);
    putsUart0(str);
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
        snprintf(str,sizeof(str),"%"PRIu16"\n",aoaValue);
        putsUart0(str); 
    }

    if(tdoaEnable) {

    }

    if(partialSets) {

    }

}
