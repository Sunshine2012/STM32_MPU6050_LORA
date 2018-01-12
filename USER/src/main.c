#include <includes.h>

u8 g_ucConnectMode      = 1;            // 1为联机模式,其他为单机测试模式
u8 g_ucIsUpdateMenu     = 0;            // 更新显示
u8 g_ucCurDlg           = 0;            // 当前显示的菜单ID
u8 g_ucHighLightRow     = 0;            // 当前显示的菜单需要高亮的行
u8 g_ucCurID            = 1;            // 当前通信设备的号
u8 g_ucIsNewWarningCode = 0;            // 有新的报警,再次更新界面,在同一时候,有多个未处理的报警
u8 g_ucUpWorkingID      = 1;            // 上工位工作卡机号
u8 g_ucUpBackingID      = 2;            // 上工位备用卡机号
u8 g_ucDownWorkingID    = 3;            // 下工位工作卡机号
u8 g_ucDownBackingID    = 4;            // 下工位备用卡机号
u8 g_ucaCardIsReady[4]  = {1, 1, 1, 1}; // 卡就绪
u8 g_ucaFaultCode[4]    = {0, 0, 0, 0}; // 卡机是否有未处理的故障
u8 g_ucaDeviceIsSTBY[4] = {1, 1, 1, 1}; // 上或下两个卡机处于待机(Standby)状态下,按键按下,主机收到两条按键信息,此时只处理主机的,如果只收到一条按键信息,则直接发卡
u8 g_ucaMechineExist[4] = {0, 0, 0, 0}; // 卡机是否存在并通信正常


//******角度参数************
float Gyro_y = 0; //Y轴陀螺仪数据暂存
float Angle_gy = 0; //由角速度计算的倾斜角度
float Accel_x = 0; //X轴加速度值暂存
float Angle_ax = 0; //由加速度计算的倾斜角度
float Angle = 0; //最终测量角度

u8 g_ucTransFlag = 0;           // 数据发送间隔时间标志
u32 g_uiTransInternal = 30;     // 默认发送数据的间隔时间30s
u32 g_uiGyro_y = 10;            // 角速度阈值


CanQueue  g_tCanRxQueue = {0};        // CAN接收卡机数据队列
UartQueue g_tUARTRxQueue = {0};       // UART接收PC机数据队列
CanRxMsg  g_tCanRxMsg = {0};          // CAN数据出队元素
u8 g_ucaUartRxMsg[100] = {0};          // UART数据出队元素

void bspInit( void )
{
    delayInit();                                                                // 定时函数
    //LED_Init();                                                                 // 初始化 LED
    USART1_Config();                                                            // 初始化 USART1

    //USART4_Config ();         // 初始化 USART4
    //DAC_init();
    //matrixKeyboardInit();
    //lcdInit();
    //canInitQueue( &g_tCanRxQueue );
    uartInitQueue( &g_tUARTRxQueue);
    //canInit();                                                                  // 初始化CAN通信

	I2C_Congiguration();

	MPU6050_Init();

 
    //IWDG_Init( 6, 625 );                                                        // 分频数为256,重载值为625,溢出时间为4s   (1/40000)* 256 * 625  = 4s          40000代表着独立看门狗的RC振荡器为40KHz
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
                doShowStatusMenu( DLG_STATUS, 5, NULL ); // 显示主界面菜单,当前状态
                break;

            case DLG_MAIN:
                doShowMainMenu( DLG_MAIN, 0, NULL );    // 进入设置状态
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


/*************卡尔曼滤波*********************************/
void Kalman_Filter(float Accel,float Gyro)
{

	static const float Q_angle=0.001;
	static const float Q_gyro=0.003;
	static const float R_angle=0.5;
	static const float dt=0.01;	                  //dt为kalman滤波器采样时间;
	static const char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };

	Angle+=(Gyro - Q_bias) * dt; //先验估计

	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]= -PP[1][1];
	Pdot[2]= -PP[1][1];
	Pdot[3]=Q_gyro;

	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;

	Angle_err = Accel - Angle;	//zk-先验估计

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	Angle	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	Gyro_y   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
}

void Angle_Calculate( void )
{
    static uint8_t DataBuffer[2];       //数据缓存

    /****************************加速度****************************************/
    I2C_ReadBuffer( DataBuffer, ACCEL_XOUT_H, 2 );
    Accel_x = ( short ) ( ( DataBuffer[0] << 8 ) +DataBuffer[1] ); //读取X轴加速度
    Angle_ax = ( Accel_x + 220 ) / 16384; //去除零点偏移,计算得到角度（弧度）
    Angle_ax = Angle_ax * 1.2 * 180 / 3.14; //弧度转换为度,

    /****************************角速度****************************************/
    I2C_ReadBuffer( DataBuffer, GYRO_YOUT_H, 2 );
    Gyro_y = ( short ) ( ( DataBuffer[0] << 8 ) +DataBuffer[1] ); //静止时角速度Y轴输出为-18左右
    Gyro_y = ( Gyro_y + 13 ) / 16.4;    //去除零点偏移，计算角速度值

    //		Angle_gy = Angle_gy + Gyro_y*0.01;  //角速度积分得到倾斜角度.

    /***************************卡尔曼滤波+角度融合*************************************/
    Kalman_Filter( Angle_ax, Gyro_y );  //卡尔曼滤波计算倾角

    /*******************************互补滤波******************************************/

    /*补偿原理是取当前倾角和加速度获
        得倾角差值进行放大，然后与陀螺
        仪角速度叠加后再积分，从而使倾
        角最跟踪为加速度获得的角度0.5
        为放大倍数，可调节补偿度;
        0.01为系统周期10ms
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
    IWDG_Init( 6, 625 );                                                        // 分频数为256,重载值为625,溢出时间为4s   (1/40000)* 256 * 625  = 4s          40000代表着独立看门狗的RC振荡器为40KHz
    generalTIM2Init();          // 定时器2初始化,1秒中断一次,更新时间由全局的数据决定
    generalTIM3Init ();         // 定时器3初始化,10ms中断一次,计算陀螺仪的数据
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
            uiGyro_y = abs((int)Gyro_y);    // 进行绝对值转换
            if ( uiGyro_y < g_uiGyro_y )
            {
                printf ("AT+CMSGHEX=\"30\"\r\n");
            }
            else
            {
                printf ("AT+CMSGHEX=\"31\"\r\n");
            }
        }
        //printf("Angle(最终测量角度) = %4.7f	Gyro_y(Y轴陀螺仪数据暂存) = %4.7f Angle_ax(由加速度计算的倾斜角度) = %4.7f\r\n",
        printf("最终测量角度 = %4.7f, Y轴陀螺仪数据 = %4.7f\r\n",
        Angle,Gyro_y);

        delayMs( 1 );
        IWDG_Feed();                // 如果没有产生硬件错误,喂狗,以防硬件问题造成的司机,程序无响应
    }
}

