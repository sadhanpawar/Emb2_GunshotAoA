
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "dma.h"

/*
uint16_t mic1SamplesPrimary[10] = {0};
uint16_t mic2SamplesPrimary[10] = {0};
uint16_t mic3SamplesPrimary[10] = {0};

uint16_t mic1SamplesAlternate[10] = {0};
uint16_t mic2SamplesAlternate[10] = {0};
uint16_t mic3SamplesAlternate[10] = {0};
*/
#if 0
typedef struct
{
    uint16_t mic1SamplesAlternate[10];
    uint16_t mic2SamplesAlternate[10];
    uint16_t mic3SamplesAlternate[10];
} adcBuffer_t;

typedef struct
{
    volatile uint32_t srcEndPtr;
    volatile uint32_t dstEndPtr;
    volatile uint32_t controlWord;
    volatile uint32_t unused;
} controlStructure_t;

controlStructure_t dmaControlBlockMem[0x3F0] __attribute__ ((aligned (1024))) = {0};

void initDma(void)
{
    SYSCTL_RCGCDMA_R |= 0x1;
    _delay_cycles(3);

    UDMA_CFG_R = 0x1;
    UDMA_CTLBASE_R = (uint32_t)dmaControlBlockMem;

    UDMA_PRIOSET_R = (1 << 14);
    UDMA_ALTCLR_R = (1<< 14);
    UDMA_USEBURSTCLR_R = (1<<14);
    UDMA_REQMASKCLR_R = (1<<14);

    UDMA_CHMAP1_R = (0xF << 24);
    UDMA_SWREQ_R = 0x0;

    dmaControlBlockMem[14].srcEndPtr = &ADC0_SSFIFO0_R;
    dmaControlBlockMem[14].dstEndPtr = ;
    dmaControlBlockMem[14].controlWord = ;

}
#endif
