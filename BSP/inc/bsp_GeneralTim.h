#ifndef __BSP_GENERALTIME_H
#define __BSP_GENERALTIME_H


#include "stm32f10x.h"


/**************ͨ�ö�ʱ��TIM�������壬ֻ��TIM2��3��4��5************/
// ����Ҫ�ĸ���ʱ����ʱ��ֻ��Ҫ������ĺ궨��ĳ�1����
#define GENERALTIM2    1
#define GENERALTIM3    1
#define GENERALTIM4    1
#define GENERALTIM5    1

#if  GENERALTIM2
#define            GENERAL_TIM2                   TIM2
#define            GENERAL_TIM2_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            GENERAL_TIM2_CLK               RCC_APB1Periph_TIM2
#define            GENERAL_TIM2_Period            (50000-1)    /*1 ���ж�*/
#define            GENERAL_TIM2_Prescaler         1420
#define            GENERAL_TIM2_IRQ               TIM2_IRQn
#define            GENERAL_TIM2_IRQHandler        TIM2_IRQHandler

#endif

#if  GENERALTIM3
#define            GENERAL_TIM3                   TIM3
#define            GENERAL_TIM3_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            GENERAL_TIM3_CLK               RCC_APB1Periph_TIM3
#define            GENERAL_TIM3_Period            (10000-1)
#define            GENERAL_TIM3_Prescaler         71
#define            GENERAL_TIM3_IRQ               TIM3_IRQn
#define            GENERAL_TIM3_IRQHandler        TIM3_IRQHandler

#endif

#if   GENERALTIM4
#define            GENERAL_TIM4                   TIM4
#define            GENERAL_TIM4_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            GENERAL_TIM4_CLK               RCC_APB1Periph_TIM4
#define            GENERAL_TIM4_Period            (1000-1)
#define            GENERAL_TIM4_Prescaler         71
#define            GENERAL_TIM4_IRQ               TIM4_IRQn
#define            GENERAL_TIM4_IRQHandler        TIM4_IRQHandler

#endif

#if   GENERALTIM5
#define            GENERAL_TIM5                   TIM5
#define            GENERAL_TIM5_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            GENERAL_TIM5_CLK               RCC_APB1Periph_TIM5
#define            GENERAL_TIM5_Period            (1000-1)
#define            GENERAL_TIM5_Prescaler         71
#define            GENERAL_TIM5_IRQ               TIM5_IRQn
#define            GENERAL_TIM5_IRQHandler        TIM5_IRQHandler

#endif
/**************************��������********************************/

extern uint32_t time_0;     // ms ��ʱ����
extern uint32_t timeMsg;    // ms ��ʱ����


void generalTIM2Init(void);
void generalTIM3Init(void);
void generalTIM4Init(void);
void generalTIM5Init(void);


#endif  /* __BSP_GENERALTIME_H */


