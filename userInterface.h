/*
 * userInterface.h
 *
 *  Created on: Apr 8, 2023
 *      Author: sadhan
 */

#ifndef USERINTERFACE_H_
#define USERINTERFACE_H_

#define MAX_CHARS 80

extern bool alwaysEventAoa;

void processShell(void);
uint16_t asciiToUint16(const char str[]);

void displayAvgs(void);
void setTimeConstant(uint16_t value);
void setBackOff(uint16_t value);
void setHoldoff(uint16_t value);
void setHysteresis(uint16_t value);
void alwaysEvents(void);


#endif /* USERINTERFACE_H_ */
