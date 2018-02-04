
#include <includes.h>
#include "stm32f10x.h"
#include "bsp_mpu6050.h"

void I2C_Congiguration( void )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    /* ʹ���� I2C1 �йص�ʱ�� */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );

    /* PB6-I2C1_SCL��PB7-I2C1_SDA*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; // ��©���
    GPIO_Init( GPIOB, &GPIO_InitStructure );

    /* I2C ���� */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = I2C1_MPU6050;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;

    /* ʹ�� I2C1 */
    I2C_Cmd( I2C1, ENABLE );

    /* I2C1 ��ʼ�� */
    I2C_Init( I2C1, &I2C_InitStructure );

    /*����1�ֽ�1Ӧ��ģʽ*/
    I2C_AcknowledgeConfig( I2C1, ENABLE );
}
void MPU6050_Init( void )
{
    delayMs( 100 );
    I2C_WriteByte( PWR_MGMT_1, 0x00 );
    delayMs( 10 );
    I2C_WriteByte( SMPLRT_DIV, 0x07 );
    delayMs( 10 );
    I2C_WriteByte( CONFIG, 0x06 );
    delayMs( 10 );
    I2C_WriteByte( GYRO_CONFIG, 0x18 );
    delayMs( 10 );
    I2C_WriteByte( ACCEL_CONFIG, 0x01 );
    delayMs( 100 );
}

#if 1
void I2C_WriteByte( uint8_t REG_Address, uint8_t Write_Data )
{
    while ( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) )
        ; //

    /* Send STRAT condition */
    I2C_GenerateSTART( I2C1, ENABLE );

    /* Test on EV5 and clear it */
    while ( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) )
        ;

    /* Send EEPROM address for write */
    I2C_Send7bitAddress( I2C1, I2C1_MPU6050, I2C_Direction_Transmitter );

    /* Test on EV6 and clear it */
    while ( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) )
        ;

    /* Send the EEPROM's internal address to write to */
    I2C_SendData( I2C1, REG_Address );

    /* Test on EV8 and clear it */
    while ( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) )
        ;

    /* Send the byte to be written */
    I2C_SendData( I2C1, Write_Data );

    /* Test on EV8 and clear it */
    while ( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) )
        ;

    /* Send STOP condition */
    I2C_GenerateSTOP( I2C1, ENABLE );
    //I2C_WaitEepromStandbyState();   GD32 ����ȥ�����д���  stm32��Ҫ���д���
}
uint8_t I2C_ReadByte( uint8_t REG_Address )
{
    uint8_t data_byte;

    //*((uint8_t *)0x4001080c) |=0x80;
    while ( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) )
        ; //

    /* Send START condition */
    I2C_GenerateSTART( I2C1, ENABLE );

    //*((uint8_t *)0x4001080c) &=~0x80;

    /* Test on EV5 and clear it */
    while ( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) )
        ;

    /* Send EEPROM address for write */
    I2C_Send7bitAddress( I2C1, I2C1_MPU6050, I2C_Direction_Transmitter );

    /* Test on EV6 and clear it */
    while ( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) )
        ;

    /* Clear EV6 by setting again the PE bit */
    I2C_Cmd( I2C1, ENABLE );

    /* Send the EEPROM's internal address to write to */
    I2C_SendData( I2C1, REG_Address );

    /* Test on EV8 and clear it */
    while ( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) )
        ;

    /* Send STRAT condition a second time */
    I2C_GenerateSTART( I2C1, ENABLE );

    /* Test on EV5 and clear it */
    while ( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) )
        ;

    /* Send EEPROM address for read */
    I2C_Send7bitAddress( I2C1, I2C1_MPU6050, I2C_Direction_Receiver );

    /* Test on EV6 and clear it */
    while ( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) )
        ;

    /* While there is data to be read */
    /* Disable Acknowledgement */
    I2C_AcknowledgeConfig( I2C1, DISABLE );

    /* Send STOP Condition */
    I2C_GenerateSTOP( I2C1, ENABLE );

    /* Test on EV7 and clear it */
    if ( I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED ) ) /* Read a byte from the EEPROM */
        data_byte = I2C_ReceiveData( I2C1 );

    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig( I2C1, ENABLE );

    //I2C_EE_WaitEepromStandbyState();
    return data_byte;
}
void I2C_ReadBuffer( uint8_t * Data_Buffer, uint8_t REG_Address, uint8_t Num_Byte )
{
    //*((uint8_t *)0x4001080c) |=0x80;
    while ( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) )
        ; //

    /* Send START condition */
    I2C_GenerateSTART( I2C1, ENABLE );

    //*((uint8_t *)0x4001080c) &=~0x80;

    /* Test on EV5 and clear it */
    while ( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) )
        ;

    /* Send EEPROM address for write */
    I2C_Send7bitAddress( I2C1, I2C1_MPU6050, I2C_Direction_Transmitter );

    /* Test on EV6 and clear it */
    while ( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) )
        ;

    /* Clear EV6 by setting again the PE bit */
    I2C_Cmd( I2C1, ENABLE );

    /* Send the EEPROM's internal address to write to */
    I2C_SendData( I2C1, REG_Address );

    /* Test on EV8 and clear it */
    while ( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) )
        ;

    /* Send STRAT condition a second time */
    I2C_GenerateSTART( I2C1, ENABLE );

    /* Test on EV5 and clear it */
    while ( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) )
        ;

    /* Send EEPROM address for read */
    I2C_Send7bitAddress( I2C1, I2C1_MPU6050, I2C_Direction_Receiver );

    /* Test on EV6 and clear it */
    while ( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) )
        ;

    /* While there is data to be read */
    while ( Num_Byte )
    {
        if ( Num_Byte == 1 )
        {
            /* Disable Acknowledgement */
            I2C_AcknowledgeConfig( I2C1, DISABLE );

            /* Send STOP Condition */
            I2C_GenerateSTOP( I2C1, ENABLE );
        }

        /* Test on EV7 and clear it */
        if ( I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED ) )
        {
            /* Read a byte from the EEPROM */
            *Data_Buffer = I2C_ReceiveData( I2C1 );

            /* Point to the next location where the byte read will be saved */
            Data_Buffer++;

            /* Decrement the read bytes counter */
            Num_Byte--;
        }
    }

    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig( I2C1, ENABLE );

    //I2C_EE_WaitEepromStandbyState();
}
void I2C_WaitEepromStandbyState( void )
{
    vu16 SR1_Tmp = 0;

    do
    {
        /* Send START condition */
        I2C_GenerateSTART( I2C1, ENABLE );

        /* Read I2C1 SR1 register */
        SR1_Tmp = I2C_ReadRegister( I2C1, I2C_Register_SR1 );

        /* Send EEPROM address for write */
        I2C_Send7bitAddress( I2C1, I2C1_MPU6050, I2C_Direction_Transmitter );
    }
    while( ! ( I2C_ReadRegister( I2C1, I2C_Register_SR1 ) & 0x0002 ) );

    /* Clear AF flag */
    I2C_ClearFlag( I2C1, I2C_FLAG_AF );

    /* STOP condition */
    I2C_GenerateSTOP( I2C1, ENABLE );   //
}

#else if

void I2C_WriteByte(uint8_t reg, uint8_t data)
{
    IIC_Start();
    IIC_Send_Byte((I2C1_MPU6050 << 1) | 0);             //����������ַ+д����

    if (IIC_Wait_Ack()) //�ȴ�Ӧ��
    {
        IIC_Stop();
        //return 1;
        return ;
    }

    IIC_Send_Byte(reg);                             //д�Ĵ�����ַ
    IIC_Wait_Ack();                                 //�ȴ�Ӧ��
    IIC_Send_Byte(data);                            //��������

    if (IIC_Wait_Ack()) //�ȴ�ACK
    {
        IIC_Stop();
        //return 1;
        return ;
    }

    IIC_Stop();
    //return 0;
}

//IIC��һ���ֽ�
//reg:�Ĵ�����ַ
//����ֵ:����������
uint8_t I2C_ReadByte(uint8_t reg)
{
    uint8_t  res;

    IIC_Start();
    IIC_Send_Byte((I2C1_MPU6050 << 1) | 0);         //����������ַ+д����
    IIC_Wait_Ack();                                 //�ȴ�Ӧ��
    IIC_Send_Byte(reg);                             //д�Ĵ�����ַ
    IIC_Wait_Ack();                                 //�ȴ�Ӧ��
    IIC_Start();
    IIC_Send_Byte((I2C1_MPU6050 << 1) | 1);         //�����ڼ��ַ+������
    IIC_Wait_Ack();                                 //�ȴ�Ӧ��
    res = IIC_Read_Byte(0);                         //��ȡ���ݣ�����nACK
    IIC_Stop();                                     //����һ��ֹͣ����
    return res;
}

//IIC����д
//addr:������ַ
//reg: �Ĵ�����ַ
//len: д�볤��
//buf: ������
//����ֵ: 0,����
//              �������������
uint8_t I2C_Write_Len(uint8_t * buf, uint8_t reg, uint8_t len)
{
    uint8_t  i;

    IIC_Start();
    IIC_Send_Byte((I2C1_MPU6050 << 1) | 0);                 //����������ַ+д����

    if (IIC_Wait_Ack()) //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }

    IIC_Send_Byte(reg);                             //д�Ĵ�����ַ
    IIC_Wait_Ack();                                 //�ȴ�Ӧ��

    for (i = 0; i < len; i++)
    {
        IIC_Send_Byte(buf[i]);                      //��������

        if (IIC_Wait_Ack()) //�ȴ�ACK
        {
            IIC_Stop();
            return 1;
        }
    }

    IIC_Stop();
    return 0;
}

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ó���
//buf:��ȡ�������ݴ洢��
//����ֵ: 0,����
//              �������������
void I2C_ReadBuffer(uint8_t * buf, uint8_t reg, uint8_t len)
{
    IIC_Start();
    IIC_Send_Byte((I2C1_MPU6050 << 1) | 0);         //����������ַ+д����

    if (IIC_Wait_Ack()) //�ȴ�Ӧ��
    {
        IIC_Stop();
        //return 1;
        return ;
    }

    IIC_Send_Byte(reg);                             //д�Ĵ�����ַ
    IIC_Wait_Ack();                                 //�ȴ�Ӧ��
    IIC_Start();
    IIC_Send_Byte((I2C1_MPU6050 << 1) | 1);         //����������ַ+������
    IIC_Wait_Ack();                                 //�ȴ�Ӧ��

    while (len)
    {
        if (len == 1)
        {
            *buf = IIC_Read_Byte(0); //�����ݣ�����nACK
        }
        else
        {
            *buf = IIC_Read_Byte(1); //�����ݣ�����ACK
        }
        len--;
        buf++;
    }

    IIC_Stop();                                     //����һ��ֹͣ����
    //return 0;
}
#endif