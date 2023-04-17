
#ifndef MAIN_H_
#define MAIN_H_
#include <stdint.h>
#include <stdbool.h>
#include "timer.h"

#define NO_OF_SAMPLES   (16u)

#define EEP_TIME_CONSTANT       (10u)
#define EEP_BACK_OFF            (12u)
#define EEP_HOLD_OFF            (14u)
#define EEP_HYSTERESIS          (16u)

#define AVERAGE_FSM         (1)
#define PEAKS_FSM           (2)

#define PEAKS_DETECT_LED    PORTF,1

extern bool const iirFilterEnable;
extern uint16_t mic1Samples[NO_OF_SAMPLES];
extern uint16_t mic2Samples[NO_OF_SAMPLES];
extern uint16_t mic3Samples[NO_OF_SAMPLES];
extern uint16_t idx;
extern const float alpha;

extern uint32_t mic1SamplesAvg;
extern uint32_t mic2SamplesAvg;
extern uint32_t mic3SamplesAvg;

extern uint32_t threshold;
extern uint16_t timeConstant;
extern uint16_t backOff;
extern uint16_t holdOff;
extern uint16_t hysteresis;
extern bool tdoaEnable;
extern bool partialSets;
extern uint16_t aoaValue;

typedef struct
{
    uint32_t timer;
    uint32_t order;
}peekDetectOrder_t;

void initApp(void);
void calculateAvgs(void);

/*60+k(t3-t2)*/
#endif
