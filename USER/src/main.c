#include <includes.h>

//******角速度参数************
float g_fGyro_x = 0; //X轴陀螺仪数据暂存
float g_fGyro_y = 0; //Y轴陀螺仪数据暂存
float g_fGyro_z = 0; //Z轴陀螺仪数据暂存

//******角速度参数************
s16 g_s_x_base = 0; //X轴陀螺仪零点偏移
s16 g_s_y_base = 0; //Y轴陀螺仪零点偏移
s16 g_s_z_base = 0; //Z轴陀螺仪零点偏移

float g_fAngle_gy = 0; //由角速度计算的倾斜角度
float g_fAccel_x = 0; //X轴加速度值暂存
float g_fAngle_ax = 0; //由加速度计算的倾斜角度
float g_fAngle = 0; //最终测量角度

u8 g_ucTransFlag = 0;           // 数据发送间隔时间标志
u32 g_uiTransInternal = 2;     // 默认发送数据的间隔时间30s
u32 g_uiGyro  =  3;             // 角速度阈值


CanQueue  g_tCanRxQueue = {0};        // CAN接收卡机数据队列
UartQueue g_tUARTRxQueue = {0};       // UART接收PC机数据队列
CanRxMsg  g_tCanRxMsg = {0};          // CAN数据出队元素
u8 g_ucaUartRxMsg[100] = {0};        // UART数据出队元素

void bspInit( void )
{
    delayInit();                                                                // 定时函数
    //LED_Init();                                                                 // 初始化 LED
    USART1_Config();                                                            // 初始化 USART1

    //canInitQueue( &g_tCanRxQueue );
    uartInitQueue( &g_tUARTRxQueue);

    I2C_Congiguration();

    MPU6050_Init();

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

	g_fAngle+=(Gyro - Q_bias) * dt; //先验估计

	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]= -PP[1][1];
	Pdot[2]= -PP[1][1];
	Pdot[3]=Q_gyro;

	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;

	Angle_err = Accel - g_fAngle;	//zk-先验估计

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

	g_fAngle	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	g_fGyro_y   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
}

void Angle_Calculate( void )
{
    static uint8_t DataBuffer[2];       //数据缓存

    /****************************加速度****************************************/
    I2C_ReadBuffer( DataBuffer, ACCEL_XOUT_H, 2 );
    g_fAccel_x = ( short ) ( ( DataBuffer[0] << 8 ) +DataBuffer[1] ); //读取X轴加速度
    //printf ("g_fAccel_x = %f\n",g_fAccel_x);
    g_fAngle_ax = ( g_fAccel_x + 950 ) / 16384; //去除零点偏移,计算得到角度（弧度）
    g_fAngle_ax = g_fAngle_ax * 1.2 * 180 / 3.14; //弧度转换为度,

    //printf ("g_fAngle_ax = %f\n",g_fAngle_ax);

    /****************************角速度****************************************/
    I2C_ReadBuffer( DataBuffer, GYRO_YOUT_H, 2 );
    g_fGyro_y = ( short ) ( ( DataBuffer[0] << 8 ) +DataBuffer[1] ); //静止时角速度Y轴输出为-52左右
    //printf ("g_fGyro_y = %f\n",g_fGyro_y);
    g_fGyro_y = ( g_fGyro_y + 55 ) / 16.4;    //去除零点偏移，计算角速度值

    //printf ("g_fGyro_y = %f\n",g_fGyro_y);

    //		Angle_gy = Angle_gy + Gyro_y*0.01;  //角速度积分得到倾斜角度.

    /***************************卡尔曼滤波+角度融合*************************************/
    Kalman_Filter( g_fAngle_ax, g_fGyro_y );  //卡尔曼滤波计算倾角

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
    u32 temp = 0xffffffff;
    u32 i = 0;

    s16 sGyro_x = 0;                    // x轴的原始数据暂存
    s16 sGyro_y = 0;                    // y轴的原始数据暂存
    s16 sGyro_z = 0;                    // z轴的原始数据暂存

    u8 DataBufferX[2];                  // x轴数据
    u8 DataBufferY[2];                  // y轴数据
    u8 DataBufferZ[2];                  // z轴数据

    u16 usGyro_x = 0;                   // x轴的数据暂存
    u16 usGyro_y = 0;                   // y轴的数据暂存
    u16 usGyro_z = 0;                   // z轴的数据暂存

    s32 siGyro_x = 0;                    // x轴的10次读取数据暂存
    s32 siGyro_y = 0;                    // y轴的10次读取数据暂存
    s32 siGyro_z = 0;                    // z轴的10次读取数据暂存

    bspInit();
    printf ("AT+RESET\r\n");
    delayMs( 1000 );
    printf ("AT\r\n");
    delayMs( 1000 );
    printf ("AT+ID\r\n");
    delayMs( 1000);
    printf ("AT+CMSGHEX=\"A5\"\r\n");

    STMFLASH_Read(FLASH_SAVE_ADDR, (short*)&g_s_x_base, 2); // 获取保存在flash中x轴零点漂移的基准值
    delayMs( 2 );
    STMFLASH_Read(FLASH_SAVE_ADDR + 2, (short*)&g_s_y_base, 2); // 获取保存在flash中y轴零点漂移的基准值
    delayMs( 2 );
    STMFLASH_Read(FLASH_SAVE_ADDR + 4, (short*)&g_s_z_base, 2); // 获取保存在flash中z轴零点漂移的基准值

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

    //IWDG_Init( 6, 625 );                                                        // 分频数为256,重载值为625,溢出时间为4s   (1/40000)* 256 * 625  = 4s          40000代表着独立看门狗的RC振荡器为40KHz
    generalTIM2Init();          // 定时器2初始化,1秒中断一次,更新时间由全局的数据决定
    //generalTIM3Init ();         // 定时器3初始化,10ms中断一次,计算陀螺仪的数据

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

        g_fGyro_x = ( sGyro_x + g_s_x_base ) / 16.4;    //去除零点偏移，计算角速度值
        g_fGyro_y = ( sGyro_y + g_s_y_base ) / 16.4;    //去除零点偏移，计算角速度值
        g_fGyro_z = ( sGyro_z + g_s_z_base ) / 16.4;    //去除零点偏移，计算角速度值


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
            usGyro_x = abs((short)g_fGyro_x);    // 进行绝对值转换
            usGyro_y = abs((short)g_fGyro_y);    // 进行绝对值转换
            usGyro_z = abs((short)g_fGyro_z);    // 进行绝对值转换
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
        IWDG_Feed();                // 如果没有产生硬件错误,喂狗,以防硬件问题造成的死机,程序无响应
    }
}
