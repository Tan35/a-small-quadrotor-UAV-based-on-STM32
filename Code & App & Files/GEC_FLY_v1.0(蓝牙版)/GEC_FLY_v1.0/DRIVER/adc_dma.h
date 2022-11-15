#ifndef __ADC_DMA_H 
#define __ADC_DMA_H

#define ADC3_DR_Address    ((u32)0x40013C4C)
#define ADC1_DR_Address    ((uint32_t)0x4001244C)

extern u16 ADC_Value[10];
extern void ADC_DMA_Init(void);
extern u16 GetADCValue(void);



#endif 
