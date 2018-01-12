#include <includes.h>

u8 g_ucConnectMode      = 1;            // 1Ϊ����ģʽ,����Ϊ��������ģʽ
u8 g_ucIsUpdateMenu     = 0;            // ������ʾ
u8 g_ucCurDlg           = 0;            // ��ǰ��ʾ�Ĳ˵�ID
u8 g_ucHighLightRow     = 0;            // ��ǰ��ʾ�Ĳ˵���Ҫ��������
u8 g_ucCurID            = 1;            // ��ǰͨ���豸�ĺ�
u8 g_ucIsNewWarningCode = 0;            // ���µı���,�ٴθ��½���,��ͬһʱ��,�ж��δ����ı���
u8 g_ucUpWorkingID      = 1;            // �Ϲ�λ����������
u8 g_ucUpBackingID      = 2;            // �Ϲ�λ���ÿ�����
u8 g_ucDownWorkingID    = 3;            // �¹�λ����������
u8 g_ucDownBackingID    = 4;            // �¹�λ���ÿ�����
u8 g_ucaCardIsReady[4]  = {1, 1, 1, 1}; // ������
u8 g_ucaFaultCode[4]    = {0, 0, 0, 0}; // �����Ƿ���δ����Ĺ���
u8 g_ucaDeviceIsSTBY[4] = {1, 1, 1, 1}; // �ϻ��������������ڴ���(Standby)״̬��,��������,�����յ�����������Ϣ,��ʱֻ����������,���ֻ�յ�һ��������Ϣ,��ֱ�ӷ���
u8 g_ucaMechineExist[4] = {0, 0, 0, 0}; // �����Ƿ���ڲ�ͨ������


//******�ǶȲ���************
float Gyro_y = 0; //Y�������������ݴ�
float Angle_gy = 0; //�ɽ��ٶȼ������б�Ƕ�
float Accel_x = 0; //X����ٶ�ֵ�ݴ�
float Angle_ax = 0; //�ɼ��ٶȼ������б�Ƕ�
float Angle = 0; //���ղ����Ƕ�

u8 g_ucTransFlag = 0;           // ���ݷ��ͼ��ʱ���־
u32 g_uiTransInternal = 30;     // Ĭ�Ϸ������ݵļ��ʱ��30s
u32 g_uiGyro_y = 10;            // ���ٶ���ֵ


CanQueue  g_tCanRxQueue = {0};        // CAN���տ������ݶ���
UartQueue g_tUARTRxQueue = {0};       // UART����PC�����ݶ���
CanRxMsg  g_tCanRxMsg = {0};          // CAN���ݳ���Ԫ��
u8 g_ucaUartRxMsg[100] = {0};          // UART���ݳ���Ԫ��

void bspInit( void )
{
    delayInit();                                                                // ��ʱ����
    //LED_Init();                                                                 // ��ʼ�� LED
    USART1_Config();                                                            // ��ʼ�� USART1

    //USART4_Config ();         // ��ʼ�� USART4
    //DAC_init();
    //matrixKeyboardInit();
    //lcdInit();
    //canInitQueue( &g_tCanRxQueue );
    uartInitQueue( &g_tUARTRxQueue);
    //canInit();                                                                  // ��ʼ��CANͨ��

	I2C_Congiguration();

	MPU6050_Init();

 
    //IWDG_Init( 6, 625 );                                                        // ��Ƶ��Ϊ256,����ֵΪ625,���ʱ��Ϊ4s   (1/40000)* 256 * 625  = 4s          40000�����Ŷ������Ź���RC����Ϊ40KHz
}
void lcdRef()
{
    u8 key = KEY_NUL;

    if ( ( g_ucKeyValues != KEY_NUL ) || g_ucIsUpdateMenu )
    {
        g_ucIsUpdateMenu = 0;

        if ( ( g_ucaFaultCode[0] != 0 ) || ( g_ucaFaultCode[1] != 0 ) || ( g_ucaFaultCode[2] != 0 ) || ( g_ucaFaultCode[3] != 0 ) )
        {
            if ( ( g_ucIsNewWarningCode == 1 ) || ( g_ucCurDlg != DLG_FAULT_CODE ) || ( g_ucKeyValues == KEY_CANCEL ) )
            {
                doShowFaultCode( DLG_FAULT_CODE, 5, NULL );
            }

            g_ucIsNewWarningCode = 0;
            g_ucKeyValues = KEY_NUL;
            return;
        }

        switch ( g_ucCurDlg )
        {
            case DLG_STATUS:
                doShowStatusMenu( DLG_STATUS, 5, NULL ); // ��ʾ������˵�,��ǰ״̬
                break;

            case DLG_MAIN:
                doShowMainMenu( DLG_MAIN, 0, NULL );    // ��������״̬
                break;

            case DLG_WORKING_SET:
                doShowWorkingSet( DLG_WORKING_SET, 1, NULL );
                break;

            case DLG_STATUS_ONE:
                doShowStatusOne( DLG_STATUS_ONE, 5, NULL );
                break;

            case DLG_STATUS_TWO:
                doShowStatusTwo( DLG_STATUS_TWO, 5, NULL );
                break;

            case DLG_CARD_COUNT_SET:
                doShowCardCountSet( DLG_CARD_COUNT_SET, 0, NULL );
                break;

            case DLG_DEBUG_MAIN:
                doShowDebugMain( DLG_DEBUG_MAIN, 0, NULL );
                break;

            case DLG_DEBUG_ONE:
                doShowDebugOne( DLG_DEBUG_ONE, 5, NULL );
                break;

            case DLG_DEBUG_TWO:
                doShowDebugTwo( DLG_DEBUG_TWO, 5, NULL );
                break;

            default:
                break;
        }
    }
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

	Angle+=(Gyro - Q_bias) * dt; //�������

	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

	Pdot[1]= -PP[1][1];
	Pdot[2]= -PP[1][1];
	Pdot[3]=Q_gyro;

	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;

	Angle_err = Accel - Angle;	//zk-�������

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

	Angle	+= K_0 * Angle_err;	 //�������
	Q_bias	+= K_1 * Angle_err;	 //�������
	Gyro_y   = Gyro - Q_bias;	 //���ֵ(�������)��΢��=���ٶ�
}

void Angle_Calculate( void )
{
    static uint8_t DataBuffer[2];       //���ݻ���

    /****************************���ٶ�****************************************/
    I2C_ReadBuffer( DataBuffer, ACCEL_XOUT_H, 2 );
    Accel_x = ( short ) ( ( DataBuffer[0] << 8 ) +DataBuffer[1] ); //��ȡX����ٶ�
    Angle_ax = ( Accel_x + 220 ) / 16384; //ȥ�����ƫ��,����õ��Ƕȣ����ȣ�
    Angle_ax = Angle_ax * 1.2 * 180 / 3.14; //����ת��Ϊ��,

    /****************************���ٶ�****************************************/
    I2C_ReadBuffer( DataBuffer, GYRO_YOUT_H, 2 );
    Gyro_y = ( short ) ( ( DataBuffer[0] << 8 ) +DataBuffer[1] ); //��ֹʱ���ٶ�Y�����Ϊ-18����
    Gyro_y = ( Gyro_y + 13 ) / 16.4;    //ȥ�����ƫ�ƣ�������ٶ�ֵ

    //		Angle_gy = Angle_gy + Gyro_y*0.01;  //���ٶȻ��ֵõ���б�Ƕ�.

    /***************************�������˲�+�Ƕ��ں�*************************************/
    Kalman_Filter( Angle_ax, Gyro_y );  //�������˲��������

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
    u32 uiGyro_y = 0;
    bspInit();
    printf ("AT+RESET\r\n");
    delayMs( 1000 );
    printf ("AT\r\n");
    delayMs( 1000 );
    printf ("AT+ID\r\n");
    delayMs( 1000);
    printf ("AT+CMSGHEX=\"A5\"\r\n");
    IWDG_Init( 6, 625 );                                                        // ��Ƶ��Ϊ256,����ֵΪ625,���ʱ��Ϊ4s   (1/40000)* 256 * 625  = 4s          40000�����Ŷ������Ź���RC����Ϊ40KHz
    generalTIM2Init();          // ��ʱ��2��ʼ��,1���ж�һ��,����ʱ����ȫ�ֵ����ݾ���
    generalTIM3Init ();         // ��ʱ��3��ʼ��,10ms�ж�һ��,���������ǵ�����
    while ( 1 )
    {

        memset ( g_ucaUartRxMsg,0,100 );
        ret = uartOutQueue( &g_tUARTRxQueue, g_ucaUartRxMsg );;
        if ( 0 == ret )
        {
            ret = 1;

            if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"10\"\r\n") == 0)
            {
                g_uiGyro_y = 10;
                printf ("AT+MSGHEX=\"10\"\r\n");
            }
            else if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"20\"\r\n") == 0)
            {
                g_uiGyro_y = 20;
                printf ("AT+MSGHEX=\"20\"\r\n");
            }
            else if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"30\"\r\n") == 0)
            {
                g_uiGyro_y = 30;
                printf ("AT+MSGHEX=\"30\"\r\n");
            }
            else if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"40\"\r\n") == 0)
            {
                g_uiGyro_y = 40;
                printf ("AT+MSGHEX=\"40\"\r\n");
            }
            else if (strcmp(g_ucaUartRxMsg,"+CMSGHEX:\ PORT:\ 8;\ RX:\ \"50\"\r\n") == 0)
            {
                g_uiGyro_y = 50;
                printf ("AT+MSGHEX=\"50\"\r\n");
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

        }
        if (g_ucTransFlag == 1)
        {
            g_ucTransFlag = 0;
            uiGyro_y = abs((int)Gyro_y);    // ���о���ֵת��
            if ( uiGyro_y < g_uiGyro_y )
            {
                printf ("AT+CMSGHEX=\"30\"\r\n");
            }
            else
            {
                printf ("AT+CMSGHEX=\"31\"\r\n");
            }
        }
        //printf("Angle(���ղ����Ƕ�) = %4.7f	Gyro_y(Y�������������ݴ�) = %4.7f Angle_ax(�ɼ��ٶȼ������б�Ƕ�) = %4.7f\r\n",
        printf("���ղ����Ƕ� = %4.7f, Y������������ = %4.7f\r\n",
        Angle,Gyro_y);

        delayMs( 1 );
        IWDG_Feed();                // ���û�в���Ӳ������,ι��,�Է�Ӳ��������ɵ�˾��,��������Ӧ
    }
}

