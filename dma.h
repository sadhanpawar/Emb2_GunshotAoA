
#ifdef DMAENABLED
#ifndef DMA_H_
#define DMA_H_

//#define DMA_BASE_ADDR   ((volatile uint32_t *)0x20000000U)

void initDma(void);
void checkMyDmaTransfer(void);
void DmaBusError(void);

#endif
#endif
