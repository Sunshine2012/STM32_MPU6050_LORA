#include "includes.h"
#include "bsp_GeneralTim.h"

uint32_t time_0 = 0; // ms ��ʱ����
uint32_t timeMsg = 0; // ms ��ʱ����
/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
void  GENERAL_TIM2_IRQHandler (void)
{
    // 1s�ж�һ��
    if ( TIM_GetITStatus( GENERAL_TIM2, TIM_IT_Update) != RESET )
    {
        TIM_ClearITPendingBit(GENERAL_TIM2 , TIM_FLAG_Update);       // ���ж�

        timeMsg++;
        macLED1_TOGGLE();
        if (timeMsg == g_uiTransInternal)    // Ĭ��30�뷢��һ������
        {
            timeMsg = 0;
            g_ucTransFlag = 1;
        }
    }
}
/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
void  GENERAL_TIM3_IRQHandler (void)
{
    // 10ms�ж�һ��
    if ( TIM_GetITStatus( GENERAL_TIM3, TIM_IT_Update) != RESET )
    {
        TIM_ClearITPendingBit(GENERAL_TIM3 , TIM_FLAG_Update);       // ���ж�
        time_0++;
        Angle_Calculate();
    }
}

// �ж����ȼ�����
static void GENERAL_TIM2_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    // �����ж���Ϊ0
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    // �����ж���Դ
    NVIC_InitStructure.NVIC_IRQChannel = GENERAL_TIM2_IRQ ;
    // ���������ȼ�Ϊ 0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    // ������ռ���ȼ�Ϊ3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

///*
// * ע�⣺TIM_TimeBaseInitTypeDef�ṹ��������5����Ա��TIM6��TIM7�ļĴ�������ֻ��
// * TIM_Prescaler��TIM_Period������ʹ��TIM6��TIM7��ʱ��ֻ���ʼ����������Ա���ɣ�
// * ����������Ա��ͨ�ö�ʱ���͸߼���ʱ������.
// *-----------------------------------------------------------------------------
// *typedef struct
// *{ TIM_Prescaler            ����
// *  TIM_CounterMode          TIMx,x[6,7]û�У���������
// *  TIM_Period               ����
// *  TIM_ClockDivision        TIMx,x[6,7]û�У���������
// *  TIM_RepetitionCounter    TIMx,x[1,8,15,16,17]����
// *}TIM_TimeBaseInitTypeDef;
// *-----------------------------------------------------------------------------
// */
static void GENERAL_TIM2_Mode_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    // ������ʱ��ʱ��,���ڲ�ʱ��CK_INT=72M
    GENERAL_TIM2_APBxClock_FUN(GENERAL_TIM2_CLK, ENABLE);
    // �Զ���װ�ؼĴ�����ֵ���ۼ�TIM_Period+1��Ƶ�ʺ����һ�����»����ж�
    TIM_TimeBaseStructure.TIM_Period=GENERAL_TIM2_Period;
    // ʱ��Ԥ��Ƶ��
    TIM_TimeBaseStructure.TIM_Prescaler= GENERAL_TIM2_Prescaler;
    // ʱ�ӷ�Ƶ���� ��û�õ����ù�
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    // ����������ģʽ������Ϊ���ϼ���
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
    // �ظ���������ֵ��û�õ����ù�
    TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
    // ��ʼ����ʱ��
    TIM_TimeBaseInit(GENERAL_TIM2, &TIM_TimeBaseStructure);

    // ����������жϱ�־λ
    TIM_ClearFlag(GENERAL_TIM2, TIM_FLAG_Update);

    // �����������ж�
    TIM_ITConfig(GENERAL_TIM2,TIM_IT_Update,ENABLE);

    // ʹ�ܼ�����
    TIM_Cmd(GENERAL_TIM2, ENABLE);
}

// �ж����ȼ�����
static void GENERAL_TIM3_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    // �����ж���Ϊ0
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    // �����ж���Դ
    NVIC_InitStructure.NVIC_IRQChannel = GENERAL_TIM3_IRQ ;
    // ���������ȼ�Ϊ 0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    // ������ռ���ȼ�Ϊ3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

///*
// * ע�⣺TIM_TimeBaseInitTypeDef�ṹ��������5����Ա��TIM6��TIM7�ļĴ�������ֻ��
// * TIM_Prescaler��TIM_Period������ʹ��TIM6��TIM7��ʱ��ֻ���ʼ����������Ա���ɣ�
// * ����������Ա��ͨ�ö�ʱ���͸߼���ʱ������.
// *-----------------------------------------------------------------------------
// *typedef struct
// *{ TIM_Prescaler            ����
// *  TIM_CounterMode          TIMx,x[6,7]û�У���������
// *  TIM_Period               ����
// *  TIM_ClockDivision        TIMx,x[6,7]û�У���������
// *  TIM_RepetitionCounter    TIMx,x[1,8,15,16,17]����
// *}TIM_TimeBaseInitTypeDef;
// *-----------------------------------------------------------------------------
// */
static void GENERAL_TIM3_Mode_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    // ������ʱ��ʱ��,���ڲ�ʱ��CK_INT=72M
    GENERAL_TIM3_APBxClock_FUN(GENERAL_TIM3_CLK, ENABLE);
    // �Զ���װ�ؼĴ�����ֵ���ۼ�TIM_Period+1��Ƶ�ʺ����һ�����»����ж�
    TIM_TimeBaseStructure.TIM_Period=GENERAL_TIM3_Period;
    // ʱ��Ԥ��Ƶ��
    TIM_TimeBaseStructure.TIM_Prescaler= GENERAL_TIM3_Prescaler;
    // ʱ�ӷ�Ƶ���� ��û�õ����ù�
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    // ����������ģʽ������Ϊ���ϼ���
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
    // �ظ���������ֵ��û�õ����ù�
    TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
    // ��ʼ����ʱ��
    TIM_TimeBaseInit(GENERAL_TIM3, &TIM_TimeBaseStructure);

    // ����������жϱ�־λ
    TIM_ClearFlag(GENERAL_TIM3, TIM_FLAG_Update);

    // �����������ж�
    TIM_ITConfig(GENERAL_TIM3,TIM_IT_Update,ENABLE);

    // ʹ�ܼ�����
    TIM_Cmd(GENERAL_TIM3, ENABLE);
}

void generalTIM2Init(void)
{
    GENERAL_TIM2_NVIC_Config();
    GENERAL_TIM2_Mode_Config();
}
void generalTIM3Init(void)
{
    GENERAL_TIM3_NVIC_Config();
    GENERAL_TIM3_Mode_Config();
}

/*********************************************END OF FILE**********************/
