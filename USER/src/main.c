#include <includes.h>

//******���ٶȲ���************
float g_fGyro_x = 0; //X�������������ݴ�
float g_fGyro_y = 0; //Y�������������ݴ�
float g_fGyro_z = 0; //Z�������������ݴ�

//******���ٶȲ���************
s16 g_s_x_base = 0; //X�����������ƫ��
s16 g_s_y_base = 0; //Y�����������ƫ��
s16 g_s_z_base = 0; //Z�����������ƫ��

float g_fAngle_gy = 0; //�ɽ��ٶȼ������б�Ƕ�
float g_fAccel_x = 0; //X����ٶ�ֵ�ݴ�
float g_fAngle_ax = 0; //�ɼ��ٶȼ������б�Ƕ�
float g_fAngle = 0; //���ղ����Ƕ�

u8 g_ucTransFlag = 0;           // ���ݷ��ͼ��ʱ���־
u32 g_uiTransInternal = 2;     // Ĭ�Ϸ������ݵļ��ʱ��30s
u32 g_uiGyro  =  3;             // ���ٶ���ֵ


CanQueue  g_tCanRxQueue = {0};        // CAN���տ������ݶ���
UartQueue g_tUARTRxQueue = {0};       // UART����PC�����ݶ���
CanRxMsg  g_tCanRxMsg = {0};          // CAN���ݳ���Ԫ��
u8 g_ucaUartRxMsg[100] = {0};        // UART���ݳ���Ԫ��

void bspInit( void )
{
    delayInit();                                                                // ��ʱ����
    //LED_Init();                                                                 // ��ʼ�� LED
    USART1_Config();                                                            // ��ʼ�� USART1

    //canInitQueue( &g_tCanRxQueue );
    uartInitQueue( &g_tUARTRxQueue);

    I2C_Congiguration();

    MPU6050_Init();

}

/*************�������˲�*********************************/
void Kalman_Filter(float Accel,float Gyro)
{

	static const float Q_angle=0.001;
	static const float Q_gyro=0.003;
	static const float R_angle=0.5;
	static const float dt=0.01;	                  //dtΪkalman�˲�������ʱ��;
	static const char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };

	g_fAngle+=(Gyro - Q_bias) * dt; //�������

	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

	Pdot[1]= -PP[1][1];
	Pdot[2]= -PP[1][1];
	Pdot[3]=Q_gyro;

	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;

	Angle_err = Accel - g_fAngle;	//zk-�������

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	g_fAngle	+= K_0 * Angle_err;	 //�������
	Q_bias	+= K_1 * Angle_err;	 //�������
	g_fGyro_y   = Gyro - Q_bias;	 //���ֵ(�������)��΢��=���ٶ�
}

void Angle_Calculate( void )
{
    static uint8_t DataBuffer[2];       //���ݻ���

    /****************************���ٶ�****************************************/
    I2C_ReadBuffer( DataBuffer, ACCEL_XOUT_H, 2 );
    g_fAccel_x = ( short ) ( ( DataBuffer[0] << 8 ) +DataBuffer[1] ); //��ȡX����ٶ�
    //printf ("g_fAccel_x = %f\n",g_fAccel_x);
    g_fAngle_ax = ( g_fAccel_x + 950 ) / 16384; //ȥ�����ƫ��,����õ��Ƕȣ����ȣ�
    g_fAngle_ax = g_fAngle_ax * 1.2 * 180 / 3.14; //����ת��Ϊ��,

    //printf ("g_fAngle_ax = %f\n",g_fAngle_ax);

    /****************************���ٶ�****************************************/
    I2C_ReadBuffer( DataBuffer, GYRO_YOUT_H, 2 );
    g_fGyro_y = ( short ) ( ( DataBuffer[0] << 8 ) +DataBuffer[1] ); //��ֹʱ���ٶ�Y�����Ϊ-52����
    //printf ("g_fGyro_y = %f\n",g_fGyro_y);
    g_fGyro_y = ( g_fGyro_y + 55 ) / 16.4;    //ȥ�����ƫ�ƣ�������ٶ�ֵ

    //printf ("g_fGyro_y = %f\n",g_fGyro_y);

    //		Angle_gy = Angle_gy + Gyro_y*0.01;  //���ٶȻ��ֵõ���б�Ƕ�.

    /***************************�������˲�+�Ƕ��ں�*************************************/
    Kalman_Filter( g_fAngle_ax, g_fGyro_y );  //�������˲��������

    /*******************************�����˲�******************************************/

    /*����ԭ����ȡ��ǰ��Ǻͼ��ٶȻ�
        ����ǲ�ֵ���зŴ�Ȼ��������
        �ǽ��ٶȵ��Ӻ��ٻ��֣��Ӷ�ʹ��
        �������Ϊ���ٶȻ�õĽǶ�0.5
        Ϊ�Ŵ������ɵ��ڲ�����;
        0.01Ϊϵͳ����10ms
    */
    //	Angle = Angle + (((Angle_ax-Angle)*0.5 + Gyro_y)*0.01);*/
}

int main( void )
{
    u8 ret = 1;
    u32 temp = 0xffffffff;
    u32 i = 0;

    s16 sGyro_x = 0;                    // x���ԭʼ�����ݴ�
    s16 sGyro_y = 0;                    // y���ԭʼ�����ݴ�
    s16 sGyro_z = 0;                    // z���ԭʼ�����ݴ�

    u8 DataBufferX[2];                  // x������
    u8 DataBufferY[2];                  // y������
    u8 DataBufferZ[2];                  // z������

    u16 usGyro_x = 0;                   // x��������ݴ�
    u16 usGyro_y = 0;                   // y��������ݴ�
    u16 usGyro_z = 0;                   // z��������ݴ�

    s32 siGyro_x = 0;                    // x���10�ζ�ȡ�����ݴ�
    s32 siGyro_y = 0;                    // y���10�ζ�ȡ�����ݴ�
    s32 siGyro_z = 0;                    // z���10�ζ�ȡ�����ݴ�

    bspInit();
    printf ("AT+RESET\r\n");
    delayMs( 1000 );
    printf ("AT\r\n");
    delayMs( 1000 );
    printf ("AT+ID\r\n");
    delayMs( 1000);
    printf ("AT+CMSGHEX=\"A5\"\r\n");

    STMFLASH_Read(FLASH_SAVE_ADDR, (short*)&g_s_x_base, 2); // ��ȡ������flash��x�����Ư�ƵĻ�׼ֵ
    delayMs( 2 );
    STMFLASH_Read(FLASH_SAVE_ADDR + 2, (short*)&g_s_y_base, 2); // ��ȡ������flash��y�����Ư�ƵĻ�׼ֵ
    delayMs( 2 );
    STMFLASH_Read(FLASH_SAVE_ADDR + 4, (short*)&g_s_z_base, 2); // ��ȡ������flash��z�����Ư�ƵĻ�׼ֵ

    if (g_s_x_base == 0xffff)
    {
        g_s_x_base = 0;
    }
    if (g_s_y_base == 0xffff)
    {
         g_s_y_base = 0;
    }
    if (g_s_z_base == 0xffff)
    {
         g_s_z_base = 0;
    }

    //IWDG_Init( 6, 625 );                                                        // ��Ƶ��Ϊ256,����ֵΪ625,���ʱ��Ϊ4s   (1/40000)* 256 * 625  = 4s          40000�����Ŷ������Ź���RC����Ϊ40KHz
    generalTIM2Init();          // ��ʱ��2��ʼ��,1���ж�һ��,����ʱ����ȫ�ֵ����ݾ���
    //generalTIM3Init ();         // ��ʱ��3��ʼ��,10ms�ж�һ��,���������ǵ�����

    while ( 1 )
    {

        siGyro_x = 0;
        siGyro_y = 0;
        siGyro_z = 0;
        for (i = 0; i < 10; i++)
        {
            I2C_ReadBuffer( DataBufferX, GYRO_XOUT_H, 2);
            I2C_ReadBuffer( DataBufferY, GYRO_YOUT_H, 2);
            I2C_ReadBuffer( DataBufferZ, GYRO_ZOUT_H, 2);

            sGyro_x = ( DataBufferX[0] << 8 ) + DataBufferX[1];
            sGyro_y = ( DataBufferY[0] << 8 ) + DataBufferY[1];
            sGyro_z = ( DataBufferZ[0] << 8 ) + DataBufferZ[1];

            siGyro_x += sGyro_x;
            siGyro_y += sGyro_y;
            siGyro_z += sGyro_z;
        }
        sGyro_x = siGyro_x / 10;
        sGyro_y = siGyro_y / 10;
        sGyro_z = siGyro_z / 10;

        g_fGyro_x = ( sGyro_x + g_s_x_base ) / 16.4;    //ȥ�����ƫ�ƣ�������ٶ�ֵ
        g_fGyro_y = ( sGyro_y + g_s_y_base ) / 16.4;    //ȥ�����ƫ�ƣ�������ٶ�ֵ
        g_fGyro_z = ( sGyro_z + g_s_z_base ) / 16.4;    //ȥ�����ƫ�ƣ�������ٶ�ֵ


        memset ( g_ucaUartRxMsg,0,100 );
        ret = uartOutQueue( &g_tUARTRxQueue, g_ucaUartRxMsg );;

        if ( 0 == ret )
        {
            ret = 1;

            if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"11\"\r\n") == 0)
            {
                g_uiGyro = 2;
                printf ("AT+MSGHEX=\"11\"\r\n");
            }
            else if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"12\"\r\n") == 0)
            {
                g_uiGyro = 4;
                printf ("AT+MSGHEX=\"12\"\r\n");
            }
            else if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"13\"\r\n") == 0)
            {
                g_uiGyro = 6;
                printf ("AT+MSGHEX=\"13\"\r\n");
            }
            else if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"14\"\r\n") == 0)
            {
                g_uiGyro = 8;
                printf ("AT+MSGHEX=\"14\"\r\n");
            }
            else if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"15\"\r\n") == 0)
            {
                g_uiGyro = 10;
                printf ("AT+MSGHEX=\"15\"\r\n");
            }
            else if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"51\"\r\n") == 0)
            {
                g_uiTransInternal = 10;
                printf ("AT+MSGHEX=\"51\"\r\n");
            }
            else if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"52\"\r\n") == 0)
            {
                g_uiTransInternal = 20;
                printf ("AT+MSGHEX=\"52\"\r\n");
            }
            else if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"53\"\r\n") == 0)
            {
                g_uiTransInternal = 30;
                printf ("AT+MSGHEX=\"53\"\r\n");
            }
            else if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"54\"\r\n") == 0)
            {
                g_uiTransInternal = 40;
                printf ("AT+MSGHEX=\"54\"\r\n");
            }
            else if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"55\"\r\n") == 0)
            {
                g_uiTransInternal = 50;
                printf ("AT+MSGHEX=\"55\"\r\n");
            }
            else if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"56\"\r\n") == 0)
            {
                g_uiTransInternal = 60;
                printf ("AT+MSGHEX=\"56\"\r\n");
            }
            else if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"60\"\r\n") == 0)
            {
                g_uiTransInternal = 120;
                printf ("AT+MSGHEX=\"60\"\r\n");
            }
            else if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"61\"\r\n") == 0)
            {
                g_uiTransInternal = 180;
                printf ("AT+MSGHEX=\"61\"\r\n");
            }
            else if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"62\"\r\n") == 0)
            {
                g_uiTransInternal = 240;
                printf ("AT+MSGHEX=\"62\"\r\n");
            }
            else if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"63\"\r\n") == 0)
            {
                g_uiTransInternal = 300;
                printf ("AT+MSGHEX=\"63\"\r\n");
            }
            else if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"AA\"\r\n") == 0)
            {
                siGyro_x = 0;
                siGyro_y = 0;
                siGyro_z = 0;
                g_s_x_base = 0;
                g_s_y_base = 0;
                g_s_z_base = 0;
                for (i = 0; i < 10; i++)
                {
                    I2C_ReadBuffer( DataBufferX, GYRO_XOUT_H, 2 );
                    I2C_ReadBuffer( DataBufferY, GYRO_YOUT_H, 2 );
                    I2C_ReadBuffer( DataBufferZ, GYRO_ZOUT_H, 2 );

                    sGyro_x = ( DataBufferX[0] << 8 ) + DataBufferX[1];
                    sGyro_y = ( DataBufferY[0] << 8 ) + DataBufferY[1];
                    sGyro_z = ( DataBufferZ[0] << 8 ) + DataBufferZ[1];

                    siGyro_x += sGyro_x;
                    siGyro_y += sGyro_y;
                    siGyro_z += sGyro_z;
                }
                g_s_x_base = siGyro_x / 10;
                g_s_y_base = siGyro_y / 10;
                g_s_z_base = siGyro_z / 10;
                STMFLASH_Write(FLASH_SAVE_ADDR, &g_s_x_base, 2);
                STMFLASH_Write(FLASH_SAVE_ADDR + 2, &g_s_y_base, 2);
                STMFLASH_Write(FLASH_SAVE_ADDR + 4, &g_s_z_base, 2);

                printf ("AT+MSGHEX=\"AA\"\r\n");
            }

        }
        if (g_ucTransFlag == 1)
        {
            g_ucTransFlag = 0;
            usGyro_x = abs((short)g_fGyro_x);    // ���о���ֵת��
            usGyro_y = abs((short)g_fGyro_y);    // ���о���ֵת��
            usGyro_z = abs((short)g_fGyro_z);    // ���о���ֵת��
            if ( usGyro_z < g_uiGyro || usGyro_y < g_uiGyro || usGyro_z < g_uiGyro)
            {
                usGyro_x = 0;
                usGyro_y = 0;
                usGyro_z = 0;
                printf ("AT+CMSGHEX=\"30\"\r\n");
            }
            else
            {
                printf ("AT+CMSGHEX=\"31\"\r\n");
            }
        }

        delayMs( 10 );
        IWDG_Feed();                // ���û�в���Ӳ������,ι��,�Է�Ӳ��������ɵ�����,��������Ӧ
    }
}
