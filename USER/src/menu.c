#include "includes.h"
#include "menu.h"


Dlg g_dlg[] =           {
                        {DLG_CLEAR_LCD,      "                ", "                ", "                ", "                ", 5},

                        {DLG_LOGO,           "    ****电子    ", " www.*****.com  ", "   ****发卡机   ", "   版本: V1.0   ", 5},
                        {DLG_STATUS,         "1.工作:         ", "2.备用:         ", "3.工作:         ", "4.备用:         ", 5},
                        {DLG_MAIN,           "1.卡机状态      ", "2.卡数设置      ", "3.卡机设置      ", "4.调机运行      ", 0},
                        {DLG_CARD_COUNT_SET, "1 号机卡数      ", "2 号机卡数      ", "3 号机卡数      ", "4 号机卡数      ", 5},
                        {DLG_CARD_ID,        "   设置卡机ID   ", "搜索卡机        ", "卡机号:         ", "通信ID号:       ", 5},
                        {DLG_WORKING_SET,    "  卡机工作设置  ", "1:工作    2:备用", "3:工作    4:备用", "                ", 1},

                        {DLG_CARD_MAIN,      "卡机状态:1号卡机", "         2号卡机", "         3号卡机", "         4号卡机", 0},
                        {DLG_STATUS_ONE,     " 1号出卡        ", " 2号出卡        ", " 3号出卡        ", " 4号出卡        ", 5},
                        {DLG_STATUS_TWO,     " 1号出坏卡      ", " 2号出坏卡      ", " 3号出坏卡      ", " 4号出坏卡      ", 5},

                        {DLG_DEBUG_MAIN,     "    号卡机调试  ", "1:联动运行      ", "2:单动运行      ", "                ", 0},
                        {DLG_DEBUG_ONE,      "↑: 翻一张好卡   ", "↓: 翻一张坏卡   ", "←: 勾一张卡     ", "→: 循环出卡     ", 5},
                        {DLG_DEBUG_TWO,      "↑: 单动正翻卡   ", "↓: 单动反翻卡   ", "←: 单动正勾卡   ", "→: 单动反勾卡   ", 5},

                        {DLG_FAULT_CODE,     "    号卡机故障  ", "故障码:         ", "说明:           ", "                ", 5},

                     };
Dlg g_dlg_fault_code[] = {
                        {DLG_CLEAR_LCD,      "                ", "                ", "                ", "                ", 5},

                        {FAULT_CODE01,       "    号卡机故障  ", "故障码:(01H)    ", "说明: 初始化勾卡", "电机不能回位    ", 5},
                        {FAULT_CODE02,       "    号卡机故障  ", "故障码:(02H)    ", "说明: 初始化翻卡", "电机反转不能回位", 5},
                        {FAULT_CODE03,       "    号卡机故障  ", "故障码:(03H)    ", "说明: 初始化翻卡", "电机正转不能回位", 5},
                        {FAULT_CODE04,       "    号卡机故障  ", "故障码:(04H)    ", "说明: 工作时勾卡", "电机向前堵转    ", 5},
                        {FAULT_CODE05,       "    号卡机故障  ", "故障码:(05H)    ", "说明: 工作时勾卡", "电机向后堵转    ", 5},
                        {FAULT_CODE06,       "    号卡机故障  ", "故障码:(06H)    ", "说明: 工作时翻卡", "电机前翻堵转    ", 5},
                        {FAULT_CODE07,       "    号卡机故障  ", "故障码:(07H)    ", "说明: 工作时翻卡", "电机前翻回程堵转", 5},
                        {FAULT_CODE08,       "    号卡机故障  ", "故障码:(08H)    ", "说明: 工作时翻卡", "电机反翻堵转    ", 5},
                        {FAULT_CODE09,       "    号卡机故障  ", "故障码:(09H)    ", "说明: 工作时翻卡", "电机反翻回程堵转", 5},
                        {FAULT_CODE0A,       "    号卡机故障  ", "故障码:(0AH)    ", "说明: 工作时勾卡", "电机不转        ", 5},
                        {FAULT_CODE0B,       "    号卡机故障  ", "故障码:(0BH)    ", "说明: 工作时翻卡", "电机不转        ", 5},
                        {FAULT_CODE0C,       "    号卡机故障  ", "故障码:(0CH)    ", "说明: 通讯故障  ", "                ", 5},
                        {FAULT_CODE0D,       "    号卡机故障  ", "故障码:(未知)   ", "                ", "                ", 5},
                        {FAULT_CODE0E,       "    号卡机故障  ", "故障码:(未知)   ", "                ", "                ", 5},
                        {FAULT_CODE0F,       "    号卡机故障  ", "故障码:(未知)   ", "                ", "                ", 5},
                        {FAULT_CODE10,       "    号卡机故障  ", "说明: 卡机中无卡", "请装载IC卡      ", "                ", 5},
                        {FAULT_CODE11,       "    号卡机故障  ", "故障码:(11H)    ", "说明:CAN总线故障", "                ", 5},

                        {FAULT_CODE11+1,     "    号卡机故障  ", "故障码:(未知)   ", "                ", "                ", 5},
                     };

// 找到显示的菜单ID,并返回其在数组中的索引
u8 check_menu(u8 ch)
{
    u8 i = 0;
    for (i = 0; i < (sizeof (g_dlg) / sizeof (g_dlg[0])); i++)
    {
        if(g_dlg[i].ID == ch)
        {
            return i;
        }
    }
    return 0;
}

// 显示状态菜单,如果有一行需要反显示,则设置当前行反显示,传递参数地址
void doShowStatusMenu (u8 dlg_id, u8 isNotRow, void * p_parm)
{
    u8 i;
    u8 dlgId = check_menu(dlg_id);
    u8 master1[] = "工作";
    u8 master2[] = "备用";
    u8 key = KEY_NUL;

    g_ucCurDlg = dlg_id;    // 记录当前显示的ID

    if(g_ucUpWorkingID == 1 && g_ucUpBackingID == 2)
    {
        for (i = 0; i < 4; i++)
        {
            g_dlg[dlgId].MsgRow[0][i + 2] = master1[i];
            g_dlg[dlgId].MsgRow[1][i + 2] = master2[i];
        }
    }
    else if (g_ucUpWorkingID == 2 && g_ucUpBackingID == 1)
    {
        for (i = 0; i < 4; i++)
        {
            g_dlg[dlgId].MsgRow[0][i + 2] = master2[i];
            g_dlg[dlgId].MsgRow[1][i + 2] = master1[i];
        }

    }
    if (g_ucDownWorkingID == 3 && g_ucDownBackingID == 4)
    {

        for (i = 0; i < 4; i++)
        {
            g_dlg[dlgId].MsgRow[2][i + 2] = master1[i];
            g_dlg[dlgId].MsgRow[3][i + 2] = master2[i];
        }

    }
    else if (g_ucDownWorkingID == 4 && g_ucDownBackingID == 3)
    {
        for (i = 0; i < 4; i++)
        {
            g_dlg[dlgId].MsgRow[2][i + 2] = master2[i];
            g_dlg[dlgId].MsgRow[3][i + 2] = master1[i];
        }
    }

    for (i = 0; i < 4; i++)
    {
        displayGB2312String (0, i * 2, g_dlg[dlgId].MsgRow[i], i == g_dlg[dlgId].highLightRow ? 1 : 0);
    }
    isTurnShow(0,g_dlg[dlgId].highLightRow);

    key = g_ucKeyValues;
    g_ucKeyValues = KEY_NUL;
    switch (key)
    {
        case KEY_ENTRY:
            doShowMainMenu (DLG_MAIN, 0, NULL);         // 进入设置状态
            break;
        default:
            break;
    }

}

// 显示菜单,如果有一行需要反显示,则设置当前行反显示,传递参数地址
void doShowMainMenu (u8 dlg_id, u8 isNotRow, void * p_parm)
{
    u8 i = 0;
    u8 key = KEY_NUL;
    u8 dlgId = check_menu(dlg_id);

    g_ucCurDlg = dlg_id;    // 记录当前显示的ID

    for (i = 0; i < 4; i++)
    {
        displayGB2312String (0, i * 2, g_dlg[dlgId].MsgRow[i], i == g_dlg[dlgId].highLightRow ? 1 : 0);
    }
    isTurnShow(0,g_dlg[dlgId].highLightRow);

    key = g_ucKeyValues;
    g_ucKeyValues = KEY_NUL;
    switch (key)
    {
        case KEY_ENTRY:
            switch (g_dlg[dlgId].highLightRow)
            {
                case 0:
                    doShowStatusOne (DLG_STATUS_ONE, 5, NULL);
                    break;
                case 1:
                    doShowCardCountSet (DLG_CARD_COUNT_SET, 0, NULL);
                    break;
                case 2:
                    doShowWorkingSet (DLG_WORKING_SET, 1, NULL);
                    break;
                case 3:
                    doShowDebugMain (DLG_DEBUG_MAIN, 0, NULL);
                    break;
                default:
                    break;
            }
            break;
        case KEY_UP:
            if (0 < g_dlg[dlgId].highLightRow)
            {
                g_dlg[dlgId].highLightRow--;
                isTurnShow(0,g_dlg[dlgId].highLightRow);
            }
            break;
        case KEY_DOWN:
            if (3 > g_dlg[dlgId].highLightRow)
            {
                g_dlg[dlgId].highLightRow++;
                isTurnShow(0,g_dlg[dlgId].highLightRow);
            }
            break;
        case KEY_QUIT:
            g_dlg[dlgId].highLightRow = 0;
            g_ucCurDlg = DLG_STATUS;    // 退出到主菜单
            g_ucIsUpdateMenu = 1;
            break;
        case KEY_CANCEL:    // 退出
            g_dlg[dlgId].highLightRow = 0;
            g_ucCurDlg = DLG_STATUS;    // 退出到主菜单
            g_ucIsUpdateMenu = 1;
            break;
        default:
            break;
    }
}

// 设置卡机卡数量,如果有一行需要反显示,则设置当前行反显示,传递参数地址
void doShowCardCountSet (u8 dlg_id, u8 isNotRow, void * p_parm)
{
    u8 isSetMode = 0;   // 如果是设置模式/
    u8 i = 0;
    u8 j = 0;
    u8 seek = 0;        // 指示当前调节的数据位置
    u8 temp = 0;
    u8 dlgId = check_menu(dlg_id);
    u8 key = KEY_NUL;
    u16 id = 0x7810 | g_ucCurID;                        // CAN通信的ID
    u8 id_h = ( id >> 8 ) & 0xff;                       // CAN通信的ID高字节
    u8 id_l = id & 0xff;                                // CAN通信的ID低字节
    u8 cardCountShowSeek = 11;                          // 卡数量字符从此行中的第几个显示,默认从11个字节开始显示
    u16 usaCardCount[4] = {1000,1001,1002,1003};        // 所有卡机初始化卡数量缓存
    u8 str_aCardCount[4][6] = {0};                      // 所有卡机初始化卡数量缓存显示字符串

    g_ucCurDlg = dlg_id;    // 记录当前显示的ID

    for (i = 0; i < 4; i++)
    {
        usaCardCount[i] = g_uiaInitCardCount[i + 1];    // 复制初始化卡数量到缓存，以供修改
    }
    for (i = 0; i < 4; i++)
    {
        sprintf(str_aCardCount[i],"%04d",usaCardCount[i]);      // 格式化初始化卡数量到字符数组，以供显示,右对齐,高位不足用0补齐
    }

    for (i = 0; i < 4; i++)     // 复制初始化卡数量字符数组到显示缓存
    {
        for (j = 0; j < 4; j++)
        {
            g_dlg[dlgId].MsgRow[i][j + cardCountShowSeek] = str_aCardCount[i][j];
        }
    }
    for (i = 0; i < 4; i++) // 显示
    {
        displayGB2312String (0, i * 2, g_dlg[dlgId].MsgRow[i], i == g_dlg[dlgId].highLightRow ? 1 : 0);
    }
    isTurnShow(0,g_dlg[dlgId].highLightRow);

    key = g_ucKeyValues;
    g_ucKeyValues = KEY_NUL;
    switch (key)
    {
        case KEY_ENTRY:
            switch (g_dlg[dlgId].highLightRow)
            {
                case 0:
                case 1:
                case 2:
                case 3:
                    isSetMode = 1;      // 进入设置模式,按取消键退出设置，再次按取消键才退回上一级菜单
                    displayGB2312String (0, isNotRow * 2, g_dlg[dlgId].MsgRow[isNotRow], 0);
                    sprintf(str_aCardCount[isNotRow],"%04d",usaCardCount[isNotRow]);      // 格式化初始化卡数量到字符数组，以供显示,右对齐,高位不足用0补齐

                    displayGB2312StringLen (cardCountShowSeek, isNotRow * 2, &str_aCardCount[isNotRow][i], i == seek ? 1 : 0, 4); // 显示4字节

                    isTurnShow(cardCountShowSeek / 2,isNotRow);
                    key = KEY_NUL;      // 不再更新界面了
                    g_ucIsUpdateMenu = 0;
                    break;
                default:
                    break;
            }
            break;
        case KEY_UP:
            if (isSetMode)
            {
                switch(seek)
                {
                    case 0:
                        temp = usaCardCount[isNotRow] / 1000;
                        usaCardCount[isNotRow] = temp < 9 ? usaCardCount[isNotRow] + 1000 : usaCardCount[isNotRow];
                        break;
                    case 1:
                        temp = usaCardCount[isNotRow] % 1000 / 100;
                        usaCardCount[isNotRow] = temp < 9 ? usaCardCount[isNotRow] + 100 : usaCardCount[isNotRow];
                        break;
                    case 2:
                        temp = usaCardCount[isNotRow] % 100 / 10;
                        usaCardCount[isNotRow] = temp < 9 ? usaCardCount[isNotRow] + 10 : usaCardCount[isNotRow];
                        break;
                    case 3:
                        temp = usaCardCount[isNotRow] % 10;
                        usaCardCount[isNotRow] = temp < 9 ? usaCardCount[isNotRow] + 1 : usaCardCount[isNotRow];
                        break;
                }
                sprintf(str_aCardCount[isNotRow],"%04d",usaCardCount[isNotRow]);        // 格式化初始化卡数量到字符数组，以供显示,右对齐,高位不足用0补齐
                for (i = 0; i < 4; i++)
                {
                    displayGB2312String (cardCountShowSeek + i, isNotRow * 2, &str_aCardCount[isNotRow][i], i == seek ? 1 : 0);
                }
                isTurnShow(0,isNotRow);
                key = KEY_NUL;      // 不再更新界面了
                g_ucIsUpdateMenu = 0;
            }
            else            // 如果是设置模式,则调整的是值
            {
                if (0 < isNotRow)
                {
                    isNotRow--;
                    isTurnShow(0,isNotRow);
                    key = KEY_NUL;
                }
                else
                {
                    key = KEY_NUL;
                }
            }
            break;
        case KEY_DOWN:
            if (isSetMode)
            {
                switch(seek)
                {
                    case 0:
                        temp = usaCardCount[isNotRow] / 1000;
                        usaCardCount[isNotRow] = temp > 0 ? usaCardCount[isNotRow] - 1000 : usaCardCount[isNotRow];
                        break;
                    case 1:
                        temp = usaCardCount[isNotRow] % 1000 / 100;
                        usaCardCount[isNotRow] = temp > 0 ? usaCardCount[isNotRow] - 100 : usaCardCount[isNotRow];
                        break;
                    case 2:
                        temp = usaCardCount[isNotRow] % 100 / 10;
                        usaCardCount[isNotRow] = temp > 0 ? usaCardCount[isNotRow] - 10 : usaCardCount[isNotRow];
                        break;
                    case 3:
                        temp = usaCardCount[isNotRow] % 10;
                        usaCardCount[isNotRow] = temp > 0 ? usaCardCount[isNotRow] - 1 : usaCardCount[isNotRow];
                        break;
                }
                sprintf(str_aCardCount[isNotRow],"%04d",usaCardCount[isNotRow]);        // 格式化初始化卡数量到字符数组，以供显示,右对齐,高位不足用0补齐
                for (i = 0; i < 4; i++)
                {
                    displayGB2312String (cardCountShowSeek + i, isNotRow * 2, &str_aCardCount[isNotRow][i], i == seek ? 1 : 0);
                }
                isTurnShow(0,isNotRow);
                key = KEY_NUL;      // 不再更新界面了
                g_ucIsUpdateMenu = 0;
            }
            else            // 如果是设置模式,则调整的是值
            {
                if (isNotRow < 3)
                {
                    isNotRow++;
                    isTurnShow(0,isNotRow);
                    key = KEY_NUL;
                }
                else
                {
                    key = KEY_NUL;
                }
            }

            break;
        case KEY_LEFT:
            if (isSetMode && seek > 0)
            {
                seek--;
                for (i = 0; i < 4; i++)
                {
                    displayGB2312String (cardCountShowSeek + i, isNotRow * 2, &str_aCardCount[isNotRow][i], i == seek ? 1 : 0);
                }
                isTurnShow(0,isNotRow);
            }
            key = KEY_NUL;
            g_ucIsUpdateMenu = 0;
            break;
        case KEY_RIGHT:
            if (isSetMode && seek < 3)
            {
                seek++;
                for (i = 0; i < 4; i++)
                {
                    displayGB2312String (cardCountShowSeek + i, isNotRow * 2, &str_aCardCount[isNotRow][i], i == seek ? 1 : 0);
                }
                isTurnShow(0,isNotRow);
            }
            key = KEY_NUL;      // 不再更新界面了
            g_ucIsUpdateMenu = 0;
            break;
        case KEY_OK:
            isSetMode = 0;      // 退出设置模式
            seek = 0;                   // 归零当前调节的数据位置
            g_uiaInitCardCount[isNotRow + 1] = usaCardCount[isNotRow];
            for (i = 0; i < 4; i++)     // 复制修改过的卡数量到显示缓存
            {
                for (j = 0; j < 4; j++)
                {
                    g_dlg[dlgId].MsgRow[i][j + cardCountShowSeek] = str_aCardCount[i][j];
                }
            }
            break;
        case KEY_CANCEL:
            if (!isSetMode)
            {
                return;                     // 如果是非设置模式,则直接退出
            }
            else
            {
                isSetMode = 0;              // 退出设置模式
                seek = 0;                   // 归零当前调节的数据位置
            }
            break;
        default:
            break;
    }
}

// 状态菜单1,如果有一行需要反显示,则设置当前行反显示,传递参数地址
void doShowStatusOne (u8 dlg_id, u8 isNotRow, void * p_parm)
{
    u8 i = 0;
    u8 dlgId = check_menu(dlg_id);
    u8 key = KEY_NUL;
    u8 str_num[5] = {0};
    u8 str_id[5] = {0};
    u16 id = 0x7810 | g_ucCurID;                    // CAN通信的ID
    u8 id_h = ( id >> 8 ) & 0xff;                   // CAN通信的ID高字节
    u8 id_l = id & 0xff;                            // CAN通信的ID低字节

    g_ucCurDlg = dlg_id;    // 记录当前显示的ID

    for (i = 0; i < 4; i++)
    {
        displayGB2312String (0, i * 2, g_dlg[dlgId].MsgRow[i], i == g_dlg[dlgId].highLightRow ? 1 : 0);
    }
    //isTurnShow(0,g_dlg[dlgId].highLightRow);

    key = g_ucKeyValues;
    g_ucKeyValues = KEY_NUL;
    switch (key)
    {
        case KEY_RIGHT:
            doShowStatusTwo (DLG_STATUS_TWO, 5, NULL);
            break;
        case KEY_QUIT:
            g_dlg[dlgId].highLightRow = 0;
            g_ucCurDlg = DLG_STATUS;
            g_ucIsUpdateMenu = 1;
            break;
        case KEY_CANCEL:
            g_dlg[dlgId].highLightRow = 0;
            g_ucCurDlg = DLG_MAIN;
            g_ucIsUpdateMenu = 1;
            break;
        default:
            break;
    }
}

// 状态菜单2,如果有一行需要反显示,则设置当前行反显示,传递参数地址
void doShowStatusTwo (u8 dlg_id, u8 isNotRow, void * p_parm)
{
    u8 i;
    u8 dlgId = check_menu(dlg_id);
    u8 key = KEY_NUL;
    u8 str_num[5] = {0};
    u8 str_id[5] = {0};
    u16 id = 0x7810 | g_ucCurID;                    // CAN通信的ID
    u8 id_h = ( id >> 8 ) & 0xff;                   // CAN通信的ID高字节
    u8 id_l = id & 0xff;                            // CAN通信的ID低字节

    g_ucCurDlg = dlg_id;                            // 记录当前显示的ID

    for (i = 0; i < 4; i++)
    {
        displayGB2312String (0, i * 2, g_dlg[dlgId].MsgRow[i], i == g_dlg[dlgId].highLightRow ? 1 : 0);
    }
    //isTurnShow(0,g_dlg[dlgId].highLightRow);

    key = g_ucKeyValues;
    g_ucKeyValues = KEY_NUL;
    switch (key)
    {
        case KEY_LEFT:
            g_dlg[dlgId].highLightRow = 0;
            g_ucCurDlg = DLG_STATUS_ONE;
            g_ucIsUpdateMenu = 1;
            break;
        case KEY_QUIT:
            g_dlg[dlgId].highLightRow = 0;
            g_ucCurDlg = DLG_STATUS;
            g_ucIsUpdateMenu = 1;
            break;
        case KEY_CANCEL:
            g_dlg[dlgId].highLightRow = 0;
            g_ucCurDlg = DLG_CARD_MAIN;
            g_ucIsUpdateMenu = 1;
            break;
        default:
            break;
    }
}

// ID设置菜单,如果有一行需要反显示,则设置当前行反显示,传递参数地址
void doShowWorkingSet (u8 dlg_id, u8 isNotRow, void * p_parm)
{
    u8 i = 0;
    u8 dlgId = check_menu(dlg_id);
    u8 key = KEY_NUL;
    u8 str_num[10] = {0};
    u8 str_id[10] = {0};
    u8 master1[] = "1:工作    2:备用";
    u8 master2[] = "1:备用    2:工作";
    u8 master3[] = "3:工作    4:备用";
    u8 master4[] = "3:备用    4:工作";

    g_ucCurDlg = dlg_id;    // 记录当前显示的ID

    if(g_ucUpWorkingID == 1 && g_ucUpBackingID == 2)
    {
        for (i = 0; i < 16; i++)
        {
            g_dlg[dlgId].MsgRow[1][i] = master1[i];
        }
    }
    else if (g_ucUpWorkingID == 2 && g_ucUpBackingID == 1)
    {
        for (i = 0; i < 16; i++)
        {
            g_dlg[dlgId].MsgRow[1][i] = master2[i];
        }
    }
    if (g_ucDownWorkingID == 3 && g_ucDownBackingID == 4)
    {

        for (i = 0; i < 16; i++)
        {
            g_dlg[dlgId].MsgRow[2][i] = master3[i];
        }
    }
    else if (g_ucDownWorkingID == 4 && g_ucDownBackingID == 3)
    {
        for (i = 0; i < 16; i++)
        {
            g_dlg[dlgId].MsgRow[2][i] = master4[i];
        }
    }
    for (i = 0; i < 4; i++)
    {
        displayGB2312String (0, i * 2, g_dlg[dlgId].MsgRow[i], i == g_dlg[dlgId].highLightRow ? 1 : 0);
    }
    isTurnShow(0,g_dlg[dlgId].highLightRow);

    key = g_ucKeyValues;
    g_ucKeyValues = KEY_NUL;
    switch (key)
    {
        case KEY_ENTRY:
            key = KEY_NUL;
            break;
        case KEY_UP:
            if (1 < g_dlg[dlgId].highLightRow)
            {
                g_dlg[dlgId].highLightRow--;
                isTurnShow(0,g_dlg[dlgId].highLightRow);
            }
            break;
        case KEY_DOWN:
            if (2 > g_dlg[dlgId].highLightRow)
            {
                g_dlg[dlgId].highLightRow++;
                isTurnShow(0,g_dlg[dlgId].highLightRow);
            }
            break;
        case KEY_LEFT:
            switch (g_dlg[dlgId].highLightRow)
            {
                case 1:
                    for (i = 0; i < 16; i++)
                    {
                        g_dlg[dlgId].MsgRow[g_dlg[dlgId].highLightRow][i] = master1[i];
                    }
                    g_ucUpWorkingID = 1;
                    g_ucUpBackingID = 2;
                    break;
                case 2:
                    for (i = 0; i < 16; i++)
                    {
                        g_dlg[dlgId].MsgRow[g_dlg[dlgId].highLightRow][i] = master3[i];
                    }
                    g_ucDownWorkingID = 3;
                    g_ucDownBackingID = 4;
                    break;
                default:
                    break;
            }
            g_ucIsUpdateMenu = 1;
            break;
        case KEY_RIGHT:
            switch (g_dlg[dlgId].highLightRow)
            {
                case 1:
                    for (i = 0; i < 16; i++)
                    {
                        g_dlg[dlgId].MsgRow[g_dlg[dlgId].highLightRow][i] = master2[i];
                    }
                    g_ucUpWorkingID = 2;
                    g_ucUpBackingID = 1;
                    break;
                case 2:
                    for (i = 0; i < 16; i++)
                    {
                        g_dlg[dlgId].MsgRow[g_dlg[dlgId].highLightRow][i] = master4[i];
                    }
                    g_ucDownWorkingID = 4;
                    g_ucDownBackingID = 3;
                default:
                    break;

            }
            g_ucIsUpdateMenu = 1;
            break;
        case KEY_OK:
            for (i = 0; i < 4; i++) // 每次设置主备机,复位状态
            {
                g_ucaDeviceIsSTBY[i] = 1;
            }
            switch (g_dlg[dlgId].highLightRow)
            {
                case 1:
                    myCANTransmit(gt_TxMessage, g_ucUpWorkingID, 0, SET_MECHINE_STATUS, WORKING_STATUS, 0, 0, NO_FAIL);   // 设置工作态
                    myCANTransmit(gt_TxMessage, g_ucUpBackingID, 0, SET_MECHINE_STATUS, BACKING_STATUS, 0, 0, NO_FAIL);   // 设置备用态
                    g_dlg[dlgId].highLightRow++;
                    isTurnShow(0,g_dlg[dlgId].highLightRow);
                    break;
                case 2:
                    myCANTransmit(gt_TxMessage, g_ucDownWorkingID, 0, SET_MECHINE_STATUS, WORKING_STATUS, 0, 0, NO_FAIL);   // 设置工作态
                    myCANTransmit(gt_TxMessage, g_ucDownBackingID, 0, SET_MECHINE_STATUS, BACKING_STATUS, 0, 0, NO_FAIL);   // 设置备用态
                    g_dlg[dlgId].highLightRow--;
                    isTurnShow(0,g_dlg[dlgId].highLightRow);
                    break;
                default:
                    break;
            }
            break;
        case KEY_QUIT:
            g_dlg[dlgId].highLightRow = 0;
            g_ucCurDlg = DLG_STATUS;
            g_ucIsUpdateMenu = 1;
            break;
        case KEY_CANCEL:
            g_dlg[dlgId].highLightRow = 0;
            g_ucCurDlg = DLG_MAIN;
            g_ucIsUpdateMenu = 1;
            break;
        default:
            break;

    }
}

// 调试菜单主选择,如果有一行需要反显示,则设置当前行反显示,传递参数地址
void doShowDebugMain (u8 dlg_id, u8 isNotRow, void * p_parm)
{
    u8 i = 0;
    u8 dlgId = check_menu(dlg_id);
    u8 key = KEY_NUL;
    u8 str_num[10] = {0};
    u8 str_id[10] = {0};
    u16 id = 0x7810 | g_ucCurID;                // CAN通信的ID
    u8 id_h = ( id >> 8 ) & 0xff;               // CAN通信的ID高字节
    u8 id_l = id & 0xff;                        // CAN通信的ID低字节

    sprintf(str_num,"0%d",g_ucCurID);
    str_num [3] = 0;

    for (i = 0; i < 2; i++)
    {
        g_dlg[dlgId].MsgRow[0][i] = str_num[i];
    }

    g_ucCurDlg = dlg_id;    // 记录当前显示的ID

    for (i = 0; i < 4; i++)
    {
        displayGB2312String (0, i * 2, g_dlg[dlgId].MsgRow[i], i == g_dlg[dlgId].highLightRow ? 1 : 0);
    }
    isTurnShow(0,g_dlg[dlgId].highLightRow);

    key = g_ucKeyValues;
    g_ucKeyValues = KEY_NUL;
    switch (key)
    {
        case KEY_ENTRY:
            switch (g_dlg[dlgId].highLightRow)
            {
                case 0:
                    break;
                case 1:
                    doShowDebugOne (DLG_DEBUG_ONE, 5, NULL);
                    break;
                case 2:
                    doShowDebugTwo (DLG_DEBUG_TWO, 5, NULL);
                    break;
                default:
                    break;
            }
            break;
        case KEY_UP:
            if (0 < g_dlg[dlgId].highLightRow)
            {
                g_dlg[dlgId].highLightRow--;
                isTurnShow(0,g_dlg[dlgId].highLightRow);
            }
            break;
        case KEY_DOWN:
            if (g_dlg[dlgId].highLightRow < 2)
            {
                g_dlg[dlgId].highLightRow++;
                isTurnShow(0,g_dlg[dlgId].highLightRow);
            }
            break;
        case KEY_LEFT:
            if ((g_dlg[dlgId].highLightRow == 0) && g_ucCurID > 1 )
            {
                g_ucCurID--;
                sprintf(str_num, "0%d", g_ucCurID);
                str_num [3] = 0;
                for (i = 0; i < 2; i++)
                {
                    g_dlg[dlgId].MsgRow[0][i] = str_num[i];
                }
                displayGB2312String (0, 0, g_dlg[dlgId].MsgRow[0], 0);
                isTurnShow(0,g_dlg[dlgId].highLightRow);
            }
            break;
        case KEY_RIGHT:
            if ((g_dlg[dlgId].highLightRow == 0) && g_ucCurID < 4)
            {
                g_ucCurID++;
                sprintf(str_num, "0%d", g_ucCurID);
                str_num [3] = 0;
                for (i = 0; i < 2; i++)
                {
                    g_dlg[dlgId].MsgRow[0][i] = str_num[i];
                }
                displayGB2312String (0, 0, g_dlg[dlgId].MsgRow[0], 0);
                isTurnShow(0,g_dlg[dlgId].highLightRow);
            }
            break;
        case KEY_QUIT:
            g_dlg[dlgId].highLightRow = 0;
            g_ucCurDlg = DLG_STATUS;
            g_ucIsUpdateMenu = 1;
        case KEY_CANCEL:
            g_dlg[dlgId].highLightRow = 0;
            g_ucCurDlg = DLG_MAIN;
            g_ucIsUpdateMenu = 1;
            break;
        default:
            break;
    }
}



// 调试菜单(联动),如果有一行需要反显示,则设置当前行反显示,传递参数地址
void doShowDebugOne (u8 dlg_id, u8 isNotRow, void * p_parm)
{
    u8 i = 0;
    u8 dlgId = check_menu(dlg_id);
    u8 key = KEY_NUL;
    u8 str_num[5] = {0};
    u8 str_id[5] = {0};
    u16 id = 0x7810 | g_ucCurID;       // CAN通信的ID
    u8 id_h = ( id >> 8 ) & 0xff;                // CAN通信的ID高字节
    u8 id_l = id & 0xff;                         // CAN通信的ID低字节

    g_ucCurDlg = dlg_id;    // 记录当前显示的ID

    for (i = 0; i < 4; i++)
    {
        displayGB2312String (0, i * 2, g_dlg[dlgId].MsgRow[i], i == isNotRow ? 1 : 0);
    }
    isTurnShow(0,isNotRow);

    for (i = 0; i < 4; i++)
    {
        g_ucaDeviceIsSTBY[i] = 1;
    }

    key = g_ucKeyValues;
    g_ucKeyValues = KEY_NUL;
    switch (key)
    {
        case KEY_UP:
            myCANTransmit(gt_TxMessage, g_ucCurID, g_ucCurID, SPIT_ONE_OK_CARD, g_ucCurID, id_h, id_l, NO_FAIL);
            g_dlg[dlgId].highLightRow = 0;
            isTurnShow(0,g_dlg[dlgId].highLightRow);
            break;
        case KEY_DOWN:
            myCANTransmit(gt_TxMessage, g_ucCurID, g_ucCurID, SPIT_ONE_BAD_CARD, g_ucCurID, id_h, id_l, NO_FAIL);
            g_dlg[dlgId].highLightRow = 1;
            isTurnShow(0,g_dlg[dlgId].highLightRow);
            break;
        case KEY_LEFT:
            myCANTransmit(gt_TxMessage, g_ucCurID, g_ucCurID, HOOK_ONE_CARD, g_ucCurID, id_h, id_l, NO_FAIL);
            g_dlg[dlgId].highLightRow = 2;
            isTurnShow(0,g_dlg[dlgId].highLightRow);
            break;
        case KEY_RIGHT:
            myCANTransmit(gt_TxMessage, g_ucCurID, g_ucCurID, CYCLE_SPIT_ONE_CARD, g_ucCurID, id_h, id_l, NO_FAIL);
            g_dlg[dlgId].highLightRow = 3;
            isTurnShow(0,g_dlg[dlgId].highLightRow);
            break;
        case KEY_QUIT:
            g_dlg[dlgId].highLightRow = 0;
            g_ucCurDlg = DLG_STATUS;
            g_ucIsUpdateMenu = 1;
            break;
        case KEY_CANCEL:
            g_dlg[dlgId].highLightRow = 0;
            g_ucCurDlg = DLG_DEBUG_MAIN;
            g_ucIsUpdateMenu = 1;
            break;
        default:
            break;
    }
}

// 调试菜单(单动),如果有一行需要反显示,则设置当前行反显示,传递参数地址
void doShowDebugTwo (u8 dlg_id, u8 isNotRow, void * p_parm)
{
    u8 i = 0;
    u8 dlgId = check_menu(dlg_id);
    u8 key = KEY_NUL;
    u8 str_num[10] = {0};
    u8 str_id[10] = {0};
    u16 id = 0x7810 | g_ucCurID;                          // CAN通信的ID
    u8 id_h = ( id >> 8 ) & 0xff;                   // CAN通信的ID高字节
    u8 id_l = id & 0xff;                            // CAN通信的ID低字节
    g_ucKeyContinu = 1;                             // 进入单动模式

    g_ucCurDlg = dlg_id;                            // 记录当前显示的ID

    for (i = 0; i < 4; i++)
    {
        displayGB2312String (0, i * 2, g_dlg[dlgId].MsgRow[i], i == isNotRow ? 1 : 0);
    }
    isTurnShow(0,isNotRow);

    for (i = 0; i < 4; i++)
    {
        g_ucaDeviceIsSTBY[i] = 1;
    }
    key = g_ucKeyValues;
    g_ucKeyValues = KEY_NUL;
    switch (key)
    {
        case KEY_UP:
            myCANTransmit(gt_TxMessage, g_ucCurID, g_ucCurID, SPIT_MOTOR_POSITIVE_STEP, g_ucCurID, id_h, id_l, NO_FAIL);
            g_dlg[dlgId].highLightRow = 0;
            isTurnShow(0,g_dlg[dlgId].highLightRow);
            break;
        case KEY_DOWN:
            myCANTransmit(gt_TxMessage, g_ucCurID, g_ucCurID, SPIT_MOTOR_NEGATIVE_STEP, g_ucCurID, id_h, id_l, NO_FAIL);
            g_dlg[dlgId].highLightRow = 1;
            isTurnShow(0,g_dlg[dlgId].highLightRow);
            break;
        case KEY_LEFT:
            myCANTransmit(gt_TxMessage, g_ucCurID, g_ucCurID, HOOK_MOTOR_POSITIVE_STEP, g_ucCurID, id_h, id_l, NO_FAIL);
            g_dlg[dlgId].highLightRow = 2;
            isTurnShow(0,g_dlg[dlgId].highLightRow);
            break;
        case KEY_RIGHT:
            myCANTransmit(gt_TxMessage, g_ucCurID, g_ucCurID, HOOK_MOTOR_NEGATIVE_STEP, g_ucCurID, id_h, id_l, NO_FAIL);
            g_dlg[dlgId].highLightRow = 3;
            isTurnShow(0,g_dlg[dlgId].highLightRow);
            break;
        case KEY_QUIT:
            g_dlg[dlgId].highLightRow = 0;
            g_ucCurDlg = DLG_STATUS;
            g_ucIsUpdateMenu = 1;
            g_ucKeyContinu = 0;             // 退出单动模式
            break;
        case KEY_CANCEL:
            g_dlg[dlgId].highLightRow = 0;
            g_ucCurDlg = DLG_DEBUG_MAIN;
            g_ucIsUpdateMenu = 1;
            g_ucKeyContinu = 0;             // 退出单动模式
            break;
        default:
            break;
    }

}


// 显示故障码,如果有一行需要反显示,则设置当前行反显示,传递参数地址
void doShowFaultCode (u8 dlg_id, u8 isNotRow, void * p_parm)
{
    u8 i = 0;
    u8 j = 0;
    u8 n = 0;
    u8 dlgId = check_menu(dlg_id);
    u8 key = KEY_NUL;
    u8 num = 1;      // 卡机号
    u8 str_num[5] = {0};
    u16 id = 0x7810 | num;       // CAN通信的ID
    u8 id_h = ( id >> 8 ) & 0xff;                // CAN通信的ID高字节
    u8 id_l = id & 0xff;                         // CAN通信的ID低字节
    u8 faultCodeIndex = 0;
    u8 faultCode = 0;

    g_ucCurDlg = dlg_id;    // 记录当前显示的ID

    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < (sizeof (g_dlg_fault_code) / sizeof (g_dlg_fault_code[0])); j++)
        {
            if (g_ucaFaultCode[i] != 0 && g_ucaFaultCode[i] == g_dlg_fault_code[j].ID) // 查找故障码对应的显示界面
            {
                faultCode = g_ucaFaultCode[i];   // 记下当前的未处理的故障和卡机号
                faultCodeIndex = i;
                //g_ucaFaultCode[i] = 0;
                num = i + 1;        // 记住是哪个卡机有故障,以清除故障
                sprintf(str_num,"%02d",num);
                for (n = 0; n < 2; n++)
                {
                    g_dlg_fault_code[j].MsgRow[0][n + 1] = str_num[n];
                }
                for (n = 0; n < 4; n++)
                {
                    displayGB2312String (0, n * 2, g_dlg_fault_code[j].MsgRow[n], 0);     // 显示故障界面
                }
                goto while_label;
            }
        }
    }

while_label:

    key = g_ucKeyValues;
    g_ucKeyValues = KEY_NUL;
    switch (key)
    {
        case KEY_ENTRY:     // 在故障码显示界面,按菜单键可以进入设置界面,且不清除按键值
            g_ucaFaultCode[faultCodeIndex] = faultCode; // 保存当前未处理的故障,以免下次再次处理
            g_dlg[dlgId].highLightRow = 0;
            break;
        case KEY_CANCEL:
            g_ucaFaultCode[faultCodeIndex] = 0;
            myCANTransmit(gt_TxMessage, num, NO_FAIL, CLEAR_FAULT_CODE, CLEAR_FAULT, NO_FAIL, NO_FAIL, faultCode);
            for (i = 0; i < 4; i++)     // 故障解除之后,清除标志,进入等待状态,等待正常的发卡流程
            {
                g_ucaDeviceIsSTBY[i] = 1;
            }
            g_dlg[dlgId].highLightRow = 0;
            g_ucIsUpdateMenu = 1;
            g_ucCurDlg = DLG_STATUS;
            break;
        default:
            break;
    }
}
