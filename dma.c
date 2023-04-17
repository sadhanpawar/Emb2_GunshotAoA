
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "dma.h"

#if 0
typedef struct
{
    uint16_t mic1SamplesAlternate[10];
    uint16_t mic2SamplesAlternate[10];
    uint16_t mic3SamplesAlternate[10];
} adcBuffer_t;

uint32_t micsSamplesAlternate[18 + 18];

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

    dmaControlBlockMem[14].srcEndPtr = &ADC0_SSFIFO0_R + 12;
    dmaControlBlockMem[14].dstEndPtr = &micsSamplesAlternate[0] + 18;
    dmaControlBlockMem[14].controlWord = UDMA_CHCTL_DSTINC_32   | 
                                        UDMA_CHCTL_DSTSIZE_16   | 
                                        UDMA_CHCTL_SRCINC_NONE  |
                                        UDMA_CHCTL_SRCSIZE_16   |
                                        UDMA_CHCTL_ARBSIZE_8    |
                                        UDMA_CHCTL_XFERMODE_PINGPONG 
                                        ;

    dmaControlBlockMem[0x200+14].srcEndPtr = &ADC0_SSFIFO0_R + 12;
    dmaControlBlockMem[0x200+14].dstEndPtr = &micsSamplesAlternate[18] + 18;
    dmaControlBlockMem[0x200+14].controlWord = UDMA_CHCTL_DSTINC_32   | 
                                        UDMA_CHCTL_DSTSIZE_16   | 
                                        UDMA_CHCTL_SRCINC_NONE  |
                                        UDMA_CHCTL_SRCSIZE_16   |
                                        UDMA_CHCTL_ARBSIZE_8    |
                                        UDMA_CHCTL_XFERMODE_PINGPONG 
                                        ;

    UDMA_ENASET_R = (1<< 14);
}

void checkMyDmaTransfer(void)
{
    int a = 1;
    int sum = a + 1;

}
#endif
