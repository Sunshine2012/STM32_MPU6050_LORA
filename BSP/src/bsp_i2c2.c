/**
  ******************************************************************************

  *
  ******************************************************************************
  */



#include <includes.h>
#include "bsp_I2C2.h"

static __IO uint32_t  IICTimeout = IICT_LONG_TIMEOUT;

/*��Ϣ���*/
#define IIC_DEBUG_ON         1

#define IIC_INFO(fmt,arg...)           printf("<<-IIC-FLASH-INFO->> "fmt"\n",##arg)
#define IIC_ERROR(fmt,arg...)          printf("<<-IIC-FLASH-ERROR->> "fmt"\n",##arg)
#define IIC_DEBUG(fmt,arg...)          do{\
                                            if(IIC_DEBUG_ON)\
                                            printf("<<-IIC-FLASH-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                            }while(0)


 /**
  * @brief  I2C_Configuration����ʼ��Ӳ��IIC����
  * @param  ��
  * @retval ��
  */
void I2C_Configuration(void)
{
    I2C_InitTypeDef  I2C_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

    /* ���� RESET ���� */


    /*STM32F103RET6оƬ��Ӳ��I2C2: PC11 -- SCL; PC12 -- SDA */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;//I2C���뿪©���
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    I2C_DeInit(I2C2);//ʹ��I2C
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x0;//������I2C��ַ
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;//100K

    I2C_Cmd(I2C2, ENABLE);
    I2C_Init(I2C2, &I2C_InitStructure);
    I2C_AcknowledgeConfig(I2C2, ENABLE);
}


 /**
  * @brief  I2C_WriteByte����OLED�Ĵ�����ַдһ��byte������
  * @param  addr���Ĵ�����ַ
    *                    data��Ҫд�������
  * @retval ��
  */
void I2C_WriteByte(uint8_t addr,uint8_t data)
{
    IICTimeout = IICT_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
    {
        if ((IICTimeout--) == 0) // ��ʱ�˳�
        {
           IIC_TIMEOUT_UserCallback(0);
           goto fault;
        }
    }

    I2C_GenerateSTART(I2C2, ENABLE);//����I2C2

    IICTimeout = IICT_LONG_TIMEOUT;
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))/*EV5,��ģʽ*/
    {
        if ((IICTimeout--) == 0)
        {
           IIC_TIMEOUT_UserCallback(0);
           goto fault;
        }
    }

    I2C_Send7bitAddress(I2C2, OLED_ADDRESS, I2C_Direction_Transmitter);//������ַ -- Ĭ��0x78

    IICTimeout = IICT_LONG_TIMEOUT;
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        if ((IICTimeout--) == 0)
        {
           IIC_TIMEOUT_UserCallback(0);
           goto fault;
        }
    }

    I2C_SendData(I2C2, addr);//�Ĵ�����ַ

    IICTimeout = IICT_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if ((IICTimeout--) == 0)
        {
           IIC_TIMEOUT_UserCallback(0);
           goto fault;
        }
    }

    I2C_SendData(I2C2, data);//��������

    IICTimeout = IICT_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if ((IICTimeout--) == 0)
        {
           IIC_TIMEOUT_UserCallback(0);
           goto fault;
        }
    }

fault:
    I2C_GenerateSTOP(I2C2, ENABLE);//�ر�I2C2����
}


 /**
  * @brief  WriteCmd����OLEDд������
  * @param  I2C_Command���������
  * @retval ��
  */
void WriteCmd(unsigned char I2C_Command)//д����
{
    I2C_WriteByte(0x00, I2C_Command);
}


 /**
  * @brief  WriteDat����OLEDд������
  * @param  I2C_Data������
  * @retval ��
  */
void WriteDat(unsigned char I2C_Data)//д����
{
    I2C_WriteByte(0x40, I2C_Data);
}


/**
  * @brief  �ȴ���ʱ�ص�����
  * @param  None.
  * @retval None.
  */
static  uint16_t IIC_TIMEOUT_UserCallback(uint8_t errorCode)
{
    /* �ȴ���ʱ��Ĵ���,���������Ϣ */
    IIC_ERROR("IIC �ȴ���ʱ!errorCode = %d",errorCode);
    return 0;
}
