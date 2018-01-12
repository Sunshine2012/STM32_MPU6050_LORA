#include <includes.h>
#include <frame.h>
#include <WAV_C_xiexie.h>
#include <WAV_C_quka.h>

u32 g_uiaInitCardCount[5]    = {1000, 999, 999, 999, 999};    // ����ʼ����ֵ,[0]Ϊ�ܿ�����,��1�ſ�,��1,[1~4]Ϊÿ��������ʼ������,��1�ſ�,��1.
u32 g_uiaSpitCardCount[5]    = {0, 0, 0, 0, 0};    // ��������,[0]Ϊ����������,��1�ſ�,��1,[1~4]Ϊÿ��������������,��1�ſ�,��1.


u8 g_ucSerNum = '0';  // ֡���   ȫ��

RSCTL_FREME g_tP_RsctlFrame = {'<','0','0','>'};        // ��Ӧ��֡
RSCTL_FREME g_tN_sctlFrame =  {'<','1','0','>'};        // ��Ӧ��֡

/* �����ϵ���Ϣ(41H)֡          4�ֽ� */
CARD_MACHINE_POWER_ON_FREME      g_tCardMechinePowerOnFrame = {'<', '0', CARD_MACHINE_POWER_ON, '>'};;

/* ״̬��Ϣ(42H)֡             30�ֽ� */
CARD_MACHINE_STATUES_FRAME       g_tCardMechineStatusFrame =    {'<', '0', 'B', '1','3',
                                                                '0', '0', '9', '9', '9', '1',
                                                                '0', '0', '9', '9', '9', '1',
                                                                '0', '0', '9', '9', '9', '1',
                                                                '0', '0', '9', '9', '9', '1',
                                                                '>'};

/* �ѳ�����Ϣ(43H)֡            6�ֽ� */
CARD_MECHINE_TO_PC_FRAME        g_tCardSpitOutFrame = {'<', '0', CARD_SPIT_OUT, '1', '1', '>'};

/* ��ťȡ����Ϣ(44H)֡          6�ֽ� */
CARD_MECHINE_TO_PC_FRAME        g_tCardKeyPressFrame = {'<', '0', CARD_KEY_PRESS, '1', '1', '>'};

/* ����ȡ����Ϣ(45H)֡          6�ֽ� */
CARD_MECHINE_TO_PC_FRAME        g_tCardTakeAwayFrame = {'<', '0', CARD_TAKE_AWAY, '1', '1', '>'};

/* �ϱ����кű����Ϣ(46H)֡   36�ֽ� */
CARD_REPORT_SPIT_STATUES_FRAME   g_tCardReportSpitStatusFrame = {'<', '0', CARD_REPORT_SPIT_STATUES, '0', '0', '0', '0', '0', '0', '0', '0',
                                                                                                     '0', '0', '0', '0', '0', '0', '0', '0',
                                                                                                     '0', '0', '0', '0', '0', '0', '0', '0',
                                                                                                     '0', '0', '0', '0', '0', '0', '0', '0', '>'};



/* ��ʼ��������Ϣ(61H)֡        20�ֽ� */
PC_TO_CARD_INIT_FREME           g_tPcToCardInitFrame = {'<', '0', PC_INIT_MECHINE, '9', '9', '9', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0','>'};

/* ������Ϣ(62H)֡              5�ֽ� */
PC_TO_CARD_MECHINE_FRAME        g_tPcSpitOutCardFrame = {'<', '0', PC_SPIT_OUT_CARD, '1', '>'};

/* ������Ϣ(63H)֡              5�ֽ� */
PC_TO_CARD_MECHINE_FRAME        g_tPcBadCardFrame = {'<', '0', PC_BAD_CARD, '1', '>'};

/* ��ѯ����״̬(65H)֡          5�ֽ� */
PC_TO_CARD_MECHINE_FRAME        g_tPcQuetyCardMechineFrame = {'<', '0', PC_QUERY_CARD_MECHINE, '1', '>'};

 /* ��ѯ����(66H)֡              5�ֽ� */
PC_TO_CARD_MECHINE_FRAME        g_tPcQuetyCardCpipFrame = {'<', '0', PC_QUERY_CARD_CLIP, '1', '>'};

/* ���ÿ��п���(67H)֡          8�ֽ�*/
PC_SET_CARD_NUM_FRAME           g_tPcSetCardNumFrame = {'<', '0', PC_SET_CARD_NUM, '1', '9', '9' , '9', '>'};





//#pragma  diag_suppress 870          // ����ʾ����

const Print_msg g_taPri_msg[] = {
                            {'$',                           "/* ��Ч��Ϣ */"},
                            {CARD_MACHINE_POWER_ON,         "/* �����ϵ���Ϣ(41H)֡          4�ֽ� */"},
                            {CARD_MACHINE_STATUES,          "/* ״̬��Ϣ(42H)֡             30�ֽ� */"},
                            {CARD_SPIT_OUT,                 "/* �ѳ�����Ϣ(43H)֡            6�ֽ� */"},
                            {CARD_KEY_PRESS,                "/* ��ťȡ����Ϣ(44H)֡          6�ֽ� */"},
                            {CARD_TAKE_AWAY,                "/* ����ȡ����Ϣ(45H)֡          6�ֽ� */"},
                            {CARD_REPORT_SPIT_STATUES,      "/* �ϱ����кű����Ϣ(46H)֡   36�ֽ� */"},

                            {PC_INIT_MECHINE,               "/* ��ʼ��������Ϣ(61H)֡       20�ֽ� */"},
                            {PC_SPIT_OUT_CARD,              "/* ������Ϣ(62H)֡              5�ֽ� */"},
                            {PC_BAD_CARD,                   "/* ������Ϣ(63H)֡              5�ֽ� */"},
                            {PC_QUERY_CARD_MECHINE,         "/* ��ѯ����״̬(65H)֡          5�ֽ� */"},
                            {PC_QUERY_CARD_CLIP,            "/* ��ѯ����(66H)֡              5�ֽ� */"},
                            {PC_SET_CARD_NUM,               "/* ���ÿ��п���(67H)֡          8�ֽ� */"},
                            {PC_GET_DIST,                   "/* ���(74H)֡                  5�ֽ� */"},
                            {'0',NULL}
                        };

const Print_msg g_taShow_msg[] = {
                            {0,                             "NULL"},
                            {MACHINE_CHECK_CARD,            "�鿨"},
                            {KEY_PRESS,                     "����"},
                            {CARD_SPIT_NOTICE,              "����"},
                            {CARD_TAKE_AWAY_NOTICE,         "ȡ��"},
                            {CARD_IS_READY,                 "������"},
                            {0xfe,                          "      "},
                            {0xff,NULL}
                        };

const Print_msg g_taShowStatus_msg[] = {
                            {0,                             "NULL"},
                            {CARD_IS_OK,                    "�ÿ�"},
                            {CARD_IS_BAD,                   "����"},
                            {IS_WORKING,                    "����"},
                            {IS_BACKING,                    "����"},
                            {0xfe,                          "    "},
                            {0xff,NULL}
                        };
const Print_msg g_taShowFaultCode_msg[] = {
                            {0,                             "NULL"},
                            {CARD_IS_OK,                    "�ÿ�"},
                            {CARD_IS_BAD,                   "����"},
                            {0xfe,                          "    "},
                            {0xff,NULL}
                        };

// ����Ҫ��ʾ�Ĳ˵����ݵ�������
void copyMenu (u8 num, u8 cmd, u8 values, u8 addr, u8 count)
{
    u8 *str_id = checkShowMsg(cmd);
    u8 i, n;
    n = check_menu (DLG_STATUS);
    for (i = addr; i < 16; i++)
    {
        g_dlg[n].MsgRow[num - 1][i] = ' ';
    }
    for (i = 0; i < count; i++)
    {
        g_dlg[n].MsgRow[num - 1][i + addr] = str_id[i];
    }
    g_ucIsUpdateMenu = 1;      // ���½���
}

// ����Ҫ��ʾ�Ĳ˵����ݵ�������
void copyStatusMsg (u8 num, u8 cmd, u8 values, u8 addr, u8 count)
{
    u8 *str_id = checkShowStatusMsg(cmd);
    u8 i, n;
    //strcpy() = CheckShowMsg(id);
    n = check_menu (DLG_STATUS);
    for (i = 0; i < count; i++)
    {
        g_dlg[n].MsgRow[num - 1][i + addr] = str_id[i];
    }
}

// ����Ҫ��ʾ�Ĳ˵����ݵ�������
void copyFaultMsg (u8 num, u8 cmd, u8 values, u8 addr, u8 count)
{
    u8 *str_id = checkShowStatusMsg(cmd);
    u8 i, n;
    //strcpy() = CheckShowMsg(id);
    n = check_menu (DLG_FAULT_CODE);
    for (i = 0; i < count; i++)
    {
        g_dlg[n].MsgRow[num - 1][i + addr] = str_id[i];
    }
}

// �ҵ���ӡ���ַ��������������׵�ַ
u8 * checkShowFaultCode (u8 ch)
{
    u8 i = 0;
    for (i = 0; i < (sizeof (g_taShowFaultCode_msg) / sizeof (g_taShowFaultCode_msg[0])); i++)
    {
        if(g_taShowFaultCode_msg[i].CTL == ch)
        {
            return (u8 *)g_taShowFaultCode_msg[i].Msg;
        }
    }
    return (u8 *)g_taShowFaultCode_msg[0].Msg;
}

// �ҵ���ӡ���ַ��������������׵�ַ
u8 * checkShowStatusMsg (u8 ch)
{
    u8 i = 0;
    for (i = 0; i < (sizeof (g_taShowStatus_msg) / sizeof (g_taShowStatus_msg[0])); i++)
    {
        if(g_taShowStatus_msg[i].CTL == ch)
        {
            return (u8 *)g_taShowStatus_msg[i].Msg;
        }
    }
    return (u8 *)g_taShowStatus_msg[0].Msg;
}

// �ҵ���ӡ���ַ��������������׵�ַ
u8 * checkShowMsg (u8 ch)
{
    u8 i = 0;
    for (i = 0; i < (sizeof (g_taShow_msg) / sizeof (g_taShow_msg[0])); i++)
    {
        if(g_taShow_msg[i].CTL == ch)
        {
            return (u8 *)g_taShow_msg[i].Msg;
        }
    }
    return (u8 *)g_taShow_msg[0].Msg;
}

// �ҵ���ӡ���ַ��������������׵�ַ
u8 * checkPriMsg (u8 ch)
{
    u8 i = 0;
    for (i = 0; i < (sizeof (g_taPri_msg) / sizeof (g_taPri_msg[0])); i++)
    {
        if(g_taPri_msg[i].CTL == ch)
        {
             return (u8 *)g_taPri_msg[i].Msg;
         }
     }
     return (u8 *)g_taPri_msg[0].Msg;
 }

// CAN�������ݴ���
 u8 analyzeCANFrame ( CanRxMsg arg )
 {
    CanRxMsg mtRxMessage = arg;                       // can���ݽ��ջ���
    u8 ID_temp = 0;
    static u8 count = 0;
    g_uiSerNum = mtRxMessage.Data[0];                               // ����֡��Ų���,�����ݻظ�

    switch(mtRxMessage.Data[3])
    {
        case KEY_PRESS:                                 // ˾���Ѱ���
            if ( (g_ucaDeviceIsSTBY[0] == 1) && (g_ucaDeviceIsSTBY[1] == 1)
              && (g_ucaDeviceIsSTBY[2] == 1) && (g_ucaDeviceIsSTBY[3] == 1) || (mtRxMessage.Data[2] == 0xff ))
            {
                if ( mtRxMessage.Data[4] == 0x10 ) // δ���뷢������,���п�
                {

                    g_tCardKeyPressFrame.MECHINE_ID = mtRxMessage.Data[1] + '0';		// ������ת��δ�ַ�,Ȼ�����ݷ��ͳ�ȥ
                    g_tCardKeyPressFrame.CARD_MECHINE = mtRxMessage.Data[1] <= 2 ? '1' : '2';   //
                    if ( g_ucConnectMode == 1 )
                    {
                        printf ( "%s\n", ( char * ) &g_tCardKeyPressFrame );
                    }
                    else
                    {
                        myCANTransmit ( gt_TxMessage, mtRxMessage.Data[1], 0, WRITE_CARD_STATUS, CARD_IS_OK, 0, 0, NO_FAIL );
                    }
                    g_ucaDeviceIsSTBY[mtRxMessage.Data[1] -1] = 0; // �����������̿�ʼ֮���ٴΰ���������Ӧ
                    copyMenu ( mtRxMessage.Data[1], KEY_PRESS, 0, 8, 4 );
                    DEBUG_printf ( "%s\n", ( char * ) checkPriMsg ( CARD_KEY_PRESS ) );
                }
                else
                {
                    myCANTransmit ( gt_TxMessage, mtRxMessage.Data[1], 0, WRITE_CARD_STATUS, 0x10, 0, 0, NO_FAIL );
                    g_ucaDeviceIsSTBY[mtRxMessage.Data[1] -1] = 1;
                }
            }
            else
            {
                myCANTransmit ( gt_TxMessage, mtRxMessage.Data[1], 0, WRITE_CARD_STATUS, 0x10, 0, 0, NO_FAIL );
            }

            break;
        case CARD_SPIT_NOTICE:                          // ����֪ͨ
            myCANTransmit ( gt_TxMessage, mtRxMessage.Data[1], 0, CARD_SPIT_NOTICE_ACK, 0, 0, 0, NO_FAIL );
            //dacSet ( DATA_quka, SOUND_LENGTH_quka );
            copyMenu ( mtRxMessage.Data[1], CARD_SPIT_NOTICE, 0, 8, 4 );
            break;
        case CARD_TAKE_AWAY_NOTICE:                     // ���ѱ�ȡ��֪ͨ
            g_ucaDeviceIsSTBY[mtRxMessage.Data[1] -1] = 1;  // �������Ѿ���ȡ��,��λ״̬
            myCANTransmit ( gt_TxMessage, mtRxMessage.Data[1], 0, CARD_TAKE_AWAY_NOTICE_ACK, 0, 0, 0, NO_FAIL );

            switch ( mtRxMessage.Data[1] )
            {
                case 1:
                    if ( (g_ucaFaultCode[1] == 0) && (g_ucaMechineExist[1] == 1) )   // �޹���,��ͨ������
                    {
                        g_ucaMechineExist[0] = 0;
                        g_ucaMechineExist[1] = 0;
                        g_ucUpWorkingID     = 2;
                        g_ucUpBackingID     = 1;
                        myCANTransmit ( gt_TxMessage, g_ucUpWorkingID, 0, SET_MECHINE_STATUS, WORKING_STATUS, 0, 0, NO_FAIL );
                        myCANTransmit ( gt_TxMessage, g_ucUpBackingID, 0, SET_MECHINE_STATUS, BACKING_STATUS, 0, 0, NO_FAIL ); // ���ù���̬
                    }
                    break;
                case 2:
                    if ( (g_ucaFaultCode[0] == 0) && (g_ucaMechineExist[0] == 1) )   // �޹���,��ͨ������
                    {
                        g_ucaMechineExist[0] = 0;
                        g_ucaMechineExist[1] = 0;
                        g_ucUpWorkingID     = 1;
                        g_ucUpBackingID     = 2;
                        myCANTransmit ( gt_TxMessage, g_ucUpWorkingID, 0, SET_MECHINE_STATUS, WORKING_STATUS, 0, 0, NO_FAIL );
                        myCANTransmit ( gt_TxMessage, g_ucUpBackingID, 0, SET_MECHINE_STATUS, BACKING_STATUS, 0, 0, NO_FAIL ); // ���ù���̬
                    }
                    break;
                case 3:
                    if ( (g_ucaFaultCode[3] == 0) && (g_ucaMechineExist[3] == 1) )   // �޹���,��ͨ������
                    {
                        g_ucaMechineExist[2] = 0;
                        g_ucaMechineExist[3] = 0;
                        g_ucDownWorkingID   = 4;
                        g_ucDownBackingID   = 3;
                        myCANTransmit ( gt_TxMessage, g_ucDownWorkingID, 0, SET_MECHINE_STATUS, WORKING_STATUS, 0, 0, NO_FAIL );
                        myCANTransmit ( gt_TxMessage, g_ucDownBackingID, 0, SET_MECHINE_STATUS, BACKING_STATUS, 0, 0, NO_FAIL ); // ���ù���̬
                    }
                    break;
                case 4:
                    if ( (g_ucaFaultCode[2] == 0) && (g_ucaMechineExist[2] == 1) )   // �޹���,��ͨ������
                    {
                        g_ucaMechineExist[2] = 0;
                        g_ucaMechineExist[3] = 0;
                        g_ucDownWorkingID   = 3;
                        g_ucDownBackingID   = 4;
                        myCANTransmit ( gt_TxMessage, g_ucDownWorkingID, 0, SET_MECHINE_STATUS, WORKING_STATUS, 0, 0, NO_FAIL );
                        myCANTransmit ( gt_TxMessage, g_ucDownBackingID, 0, SET_MECHINE_STATUS, BACKING_STATUS, 0, 0, NO_FAIL ); // ���ù���̬
                    }
                    break;
                default:
                    break;
            }

            g_tCardKeyPressFrame.MECHINE_ID = mtRxMessage.Data[1] + '0';
            printf ( "%s\n", ( char * ) &g_tCardTakeAwayFrame );
            //dacSet ( DATA_xiexie, SOUND_LENGTH_xiexie );
            copyMenu ( mtRxMessage.Data[1], CARD_TAKE_AWAY_NOTICE, 0, 8, 4 );
            DEBUG_printf ( "%s\n", ( char * ) checkPriMsg ( CARD_TAKE_AWAY ) );

            g_ucIsUpdateMenu    = 1;                    // ���½���
            break;
        case CARD_IS_READY:                             // ������
            myCANTransmit ( gt_TxMessage, mtRxMessage.Data[1], 0, CARD_READY_ACK, 0, 0, 0, NO_FAIL );
            copyMenu ( mtRxMessage.Data[1], CARD_IS_READY, 0, 8, 6 );
            break;
        case MECHINE_WARNING:                           // ����
            myCANTransmit ( gt_TxMessage, mtRxMessage.Data[1], 0, FAULT_CODE_ACK, 0, 0, 0, NO_FAIL ); // �ظ�������
            g_ucaDeviceIsSTBY[0] = 1;                       // �������Ѿ���ȡ��,��λ״̬
            g_ucaDeviceIsSTBY[1] = 1;                       // �������Ѿ���ȡ��,��λ״̬
            g_ucaDeviceIsSTBY[2] = 1;                       // �������Ѿ���ȡ��,��λ״̬
            g_ucaDeviceIsSTBY[3] = 1;                       // �������Ѿ���ȡ��,��λ״̬

            if ( ( mtRxMessage.Data[2] != 0xff ) && ( mtRxMessage.Data[4] == 0x21 ) && ( mtRxMessage.Data[7] <= FAULT_CODE11 ) )
            {
                switch ( mtRxMessage.Data[1] )
                {
                    case 1:
                        if ( (g_ucaFaultCode[1] == 0) && (g_ucaMechineExist[1] == 1) )   // �޹���,��ͨ������
                        {
                            g_ucaMechineExist[0] = 0;
                            g_ucaMechineExist[1] = 0;
                            g_ucUpWorkingID     = 2;
                            g_ucUpBackingID     = 1;
                            myCANTransmit ( gt_TxMessage, g_ucUpWorkingID, 0, SET_MECHINE_STATUS, WORKING_STATUS, 0, 0, NO_FAIL );
                            myCANTransmit ( gt_TxMessage, g_ucUpBackingID, 0, SET_MECHINE_STATUS, BACKING_STATUS, 0, 0, NO_FAIL ); // ���ù���̬
                        }
                        break;
                    case 2:
                        if ( (g_ucaFaultCode[0] == 0) && (g_ucaMechineExist[0] == 1) )   // �޹���,��ͨ������
                        {
                            g_ucaMechineExist[0] = 0;
                            g_ucaMechineExist[1] = 0;
                            g_ucUpWorkingID     = 1;
                            g_ucUpBackingID     = 2;
                            myCANTransmit ( gt_TxMessage, g_ucUpWorkingID, 0, SET_MECHINE_STATUS, WORKING_STATUS, 0, 0, NO_FAIL );
                            myCANTransmit ( gt_TxMessage, g_ucUpBackingID, 0, SET_MECHINE_STATUS, BACKING_STATUS, 0, 0, NO_FAIL ); // ���ù���̬
                        }
                        break;
                    case 3:
                        if ( (g_ucaFaultCode[3] == 0) && (g_ucaMechineExist[3] == 1) )   // �޹���,��ͨ������
                        {
                            g_ucaMechineExist[2] = 0;
                            g_ucaMechineExist[3] = 0;
                            g_ucDownWorkingID   = 4;
                            g_ucDownBackingID   = 3;
                            myCANTransmit ( gt_TxMessage, g_ucDownWorkingID, 0, SET_MECHINE_STATUS, WORKING_STATUS, 0, 0, NO_FAIL );
                            myCANTransmit ( gt_TxMessage, g_ucDownBackingID, 0, SET_MECHINE_STATUS, BACKING_STATUS, 0, 0, NO_FAIL ); // ���ù���̬
                        }
                        break;
                    case 4:
                        if ( (g_ucaFaultCode[2] == 0) && (g_ucaMechineExist[2] == 1) )   // �޹���,��ͨ������
                        {
                            g_ucaMechineExist[2] = 0;
                            g_ucaMechineExist[3] = 0;
                            g_ucDownWorkingID   = 3;
                            g_ucDownBackingID   = 4;
                            myCANTransmit ( gt_TxMessage, g_ucDownWorkingID, 0, SET_MECHINE_STATUS, WORKING_STATUS, 0, 0, NO_FAIL );
                            myCANTransmit ( gt_TxMessage, g_ucDownBackingID, 0, SET_MECHINE_STATUS, BACKING_STATUS, 0, 0, NO_FAIL ); // ���ù���̬
                        }
                        break;
                    default:
                        break;

                }
                g_ucaFaultCode[mtRxMessage.Data[1] - 1] = mtRxMessage.Data[7]; // ���������
            }
            else if ( mtRxMessage.Data[7] > FAULT_CODE11 )
            {
                g_ucaFaultCode[mtRxMessage.Data[1] - 1] = FAULT_CODE11 + 1; // ���������
            }
            g_ucIsNewWarningCode = 1;                       // ���µı���,�ٴθ��½���
            g_ucIsUpdateMenu    = 1;                        // ���½���
            break;
        case CARD_MACHINE_INIT_ACK:
            g_ucaMechineExist[mtRxMessage.Data[1] - 1] = 1;    // �����������,���豸��������,�����лظ�,�����������������,��ͨ������
            break;
        case CYCLE_ACK:                 // ��ʱ��ѯ�ظ�
            break;
        case SET_MECHINE_STATUS_ACK:    // ���ÿ�����ÿ�����������״̬�ɹ�,�������ظ���״̬,����״̬λΪ1
            //g_ucaMechineExist[mtRxMessage.Data[1] - 1] = 1;    // �����������,���豸��������,�����лظ�,�����������������,��ͨ������
            //myCANTransmit ( gt_TxMessage, 5,g_ucaMechineExist[ID_temp],5,5,5,5,5 );
            break;
        default:
            //myCANTransmit ( gt_TxMessage, 0, 0, 0, 0, 0, 0, 0 );
            break;
    }
    return 0;
}

u8  analyzeUartFrame ( u8 argv[] , u32 size)
{

    u8 ucaFrame[50] = {0};
    u8 ucSerNum = 0;
    u8 ucNum = argv[1];
    u8 type_frame = argv[2];
    //strcpy (ucaFrame, argv);
    if (POSITIVE_ACK == type_frame)    // ��Ӧ��֡
    {
        return 0;
    }
    else if (NAGATIVE_ACK == type_frame)    // ��Ӧ��֡
    {

    }
    else if (PC_INIT_MECHINE <= type_frame <= PC_SET_CARD_NUM)  // ������ݺϷ���
    {
        g_tP_RsctlFrame.RSCTL = ucNum;
        printf("%s\r\n",(char *)&g_tP_RsctlFrame);   //������Ӧ��֡

        switch(type_frame)
        {
            case PC_INIT_MECHINE:               /* ��ʼ��������Ϣ(61H)֡ */
                //displayGB2312String (0, 0, argv, 1);
                //displayGB2312String (0, 2, "��ʼ��", 0);
                break;
            case PC_SPIT_OUT_CARD:              /* ������Ϣ(62H)֡ */
                //displayGB2312String (0, 0, argv, 1);
                //displayGB2312String (0, 2, "������Ϣ", 0);
                switch (argv[3])
                {
                    case '1':
                    case '2':
                    case '3':
                    case '4':
                        myCANTransmit ( gt_TxMessage, argv[3] - '0', 0, WRITE_CARD_STATUS, CARD_IS_OK, 0, 0, NO_FAIL );
                        g_uiaInitCardCount[argv[3] - '0']--;
                        //dacSet ( DATA_quka, SOUND_LENGTH_quka );
                        copyMenu ( argv[3] - '0', CARD_SPIT_NOTICE, 0, 8, 4 );
                        copyStatusMsg ( argv[3] - '0', 0xfe, 0, 12, 4 ); //
                        break;
                    case '5':
                        myCANTransmit ( gt_TxMessage, g_ucUpWorkingID, 0, WRITE_CARD_STATUS, CARD_IS_OK, 0, 0, NO_FAIL );
                        g_uiaInitCardCount[g_ucUpWorkingID]--;
                        //dacSet ( DATA_quka, SOUND_LENGTH_quka );
                        copyMenu ( g_ucUpWorkingID, CARD_SPIT_NOTICE, 0, 8, 4 );
                        copyStatusMsg ( g_ucUpWorkingID, 0xfe, 0, 12, 4 ); //
                        break;
                    case '6':
                        myCANTransmit ( gt_TxMessage, g_ucDownWorkingID, 0, WRITE_CARD_STATUS, CARD_IS_OK, 0, 0, NO_FAIL );
                        g_uiaInitCardCount[g_ucDownWorkingID]--;
                        //dacSet ( DATA_quka, SOUND_LENGTH_quka );
                        copyMenu ( g_ucDownWorkingID, CARD_SPIT_NOTICE, 0, 8, 4 );
                        copyStatusMsg ( g_ucDownWorkingID, 0xfe, 0, 12, 4 ); //
                        break;
                    default:
                        break;
                }
                break;
            case PC_BAD_CARD:                /* ������Ϣ(63H)֡ */
                //displayGB2312String (0, 0 ,argv, 1);
                //displayGB2312String (0, 2, "����", 0);
                if ( argv[3] )
                {
                    myCANTransmit ( gt_TxMessage, argv[3] - '0', 0, WRITE_CARD_STATUS, CARD_IS_BAD, 0, 0, NO_FAIL );
                }
                break;
            case PC_QUERY_CARD_MECHINE:      /* ��ѯ����״̬(65H)֡ */
                //displayGB2312String (0, 0, argv, 1);
                //displayGB2312String (0, 2, "��ѯ����", 0);
                break;
            case PC_QUERY_CARD_CLIP:
                //displayGB2312String (0, 0, argv, 1);   /* ��ѯ����(66H)֡ */
                //displayGB2312String (0, 2, "��ѯ����", 0);
                break;
            case PC_SET_CARD_NUM:
                //displayGB2312String (0, 0, argv, 1);   /* ���ÿ��п���(67H)֡ */
                //displayGB2312String (0, 2, "���ÿ���", 0);
                break;
            case PC_GET_DIST:
                //displayGB2312Char(0,0,argv,1);   /* ���֡ */
                //displayGB2312String (0, 2, "���", 0);
                break;
            case PC_CAR_HAS_COMING:
                //displayGB2312String (0, 0, argv, 1);   /* ������ */
                //displayGB2312String (0, 2, "������", 0);
                break;
            case PC_CAR_HAS_GONE:
                //displayGB2312String (0, 0, argv, 1);   /* ������ */
                //displayGB2312String (0, 2, "������", 0);
                break;
            default:
                displayGB2312String (0, 0, argv, 1);   /* ��Ч��Ϣ */
                displayGB2312String (0, 2, "��Ч��Ϣ", 0);
                break;
        }
        DEBUG_printf ("%s\n",(char *)checkPriMsg(type_frame));
    }
    ucSerNum = (g_ucSerNum++) % 10 + '0';
    return 0;
}
