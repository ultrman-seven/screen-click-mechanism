
#include "communication.h"
#include "board.h"
#include "mcuPinCfg.h"
#include "wyUart.hpp"
#include "string.h"

extern UART::Serial com;
UART::Serial com(1, MCU_Pin_Uart1_Tx, MCU_Pin_Uart1_Rx);

#define comBuffLen 32
uint8_t comBuff[comBuffLen];

enum uartOptionIDs
{
    uartOptionID_ShowParaInStr,
    uartOptionID_ShowParaStruct,
    uartOptionID_Move2Position,
    uartOptionID_Click,
    uartOptionID_MoveAndClick,

    uartOptionID_SetPara_ServoLeft,
    uartOptionID_SetPara_ServoRight,
    uartOptionID_SetPara_LinksLeft,
    uartOptionID_SetPara_LinksRight,
    uartOptionID_SetPara_ClickTime,
    uartOptionID_SetPara_InspirePositions,

    uartOptionID_ResetCfg,
    uartOptionID_SaveCfg,

    uartOptionID_ServoDebug,

    uartOptionID_SetPara_ServoOffset,
    uartOptionID_SetPara_ServoIncTime,
    uartOptionID_ServoSlowMove,
    // uartOptionID_,
    // uartOptionID_,
};

void uartCallbackOption(uint8_t *argv, uint32_t argc)
{
    uint8_t optID;
    // float *f32ptr;

    if (!argc)
        return;
    optID = argv[0];
    ++argv;
    // f32ptr = (float *)argv;
    switch (optID)
    {
    case uartOptionID_ShowParaInStr:
        // com << "==========parameter=============\r\n";
        com << "servo left: (" << boardPara.ServoPosi_LeftX << ',' << boardPara.ServoPosi_LeftY;
        com << ")\r\nservo right: (" << boardPara.ServoPosi_RightX << ',' << boardPara.ServoPosi_RightY;
        com << ")\r\nservo offset | left: " << boardPara.ServoOffsetLeft << "|right: " << boardPara.ServoOffsetRight;
        com << "|\r\nleft link=" << boardPara.LinkLength_left1 << " & " << boardPara.LinkLength_left2;
        com << "\r\nright link=" << boardPara.LinkLength_right1 << " & " << boardPara.LinkLength_right2;
        com << "\r\ninspire positions = " << boardPara.InspireRobotPosiOff << " & " << boardPara.InspireRobotPosiOn;
        com << "\r\nclick time = " << (int)boardPara.ClickHoldTimeMs;
        break;

    case uartOptionID_ShowParaStruct:
        com.sendByte(sizeof(boardPara));
        com.sendByte((uint8_t *)&boardPara, sizeof(boardPara));
        break;

    case uartOptionID_SetPara_LinksLeft:
        if (argc < 9)
            return;
        memcpy(&boardPara.LinkLength_left1, argv, 4);
        memcpy(&boardPara.LinkLength_left2, argv + 4, 4);
        break;

    case uartOptionID_SetPara_LinksRight:
        if (argc < 9)
            return;
        memcpy(&boardPara.LinkLength_right1, argv, 4);
        memcpy(&boardPara.LinkLength_right2, argv + 4, 4);
        break;

    case uartOptionID_SetPara_ServoLeft:
        if (argc < 9)
            return;
        memcpy(&boardPara.ServoPosi_LeftX, argv, 4);
        memcpy(&boardPara.ServoPosi_LeftY, argv + 4, 4);
        break;
    case uartOptionID_SetPara_ServoRight:
        if (argc < 9)
            return;
        memcpy(&boardPara.ServoPosi_RightX, argv, 4);
        memcpy(&boardPara.ServoPosi_RightY, argv + 4, 4);
        break;

    case uartOptionID_SetPara_InspirePositions:
        if (argc < 9)
            return;
        memcpy(&boardPara.InspireRobotPosiOff, argv, 4);
        memcpy(&boardPara.InspireRobotPosiOn, argv + 4, 4);
        break;

    case uartOptionID_SetPara_ClickTime:
        if (argc < 3)
            return;
        memcpy(&boardPara.ClickHoldTimeMs, argv, 2);
        break;

    case uartOptionID_Click:
        boardCilckScreen();
        break;

    case uartOptionID_SaveCfg:
        boardParaSave();
        break;
    case uartOptionID_ResetCfg:
        boardParaReset();
        break;

    case uartOptionID_Move2Position:
    {
        float x, y;
        if (argc < 9)
            return;
        memcpy(&x, argv, 4);
        memcpy(&y, argv + 4, 4);
        boardMove2Position(x, y);
        break;
    }

    case uartOptionID_ServoDebug:
        if (argc == 1)
        {
            boardServoMove2Mid();
            return;
        }
        if (argc == 3)
            boardServoSetPosition(argv[0], argv[1]);
        break;

    case uartOptionID_SetPara_ServoOffset:
        if (argc < 6)
            return;
        if (argv[0])
            memcpy(&boardPara.ServoOffsetRight, argv + 1, 4);
        else
            memcpy(&boardPara.ServoOffsetLeft, argv + 1, 4);
        break;

    case uartOptionID_SetPara_ServoIncTime:
        if (argc < 3)
            return;
        memcpy(&boardPara.ServoIncTimeGapMs, argv, 2);
        break;

    case uartOptionID_ServoSlowMove:
    {
        float x, y;
        if (argc < 9)
            return;
        memcpy(&x, argv, 4);
        memcpy(&y, argv + 4, 4);
        boardServoSlowlyMove2Position(x, y);
        break;
    }

    default:
        break;
    }
}

void communicationInit()
{
    com.setInterrupt(comBuff, comBuffLen, "rcs", "\r\n");
    com.addCMD(uartCallbackOption);
}
