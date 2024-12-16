#include "board.h"
#include "wyTimer.hpp"
#include "wyGpio.hpp"
#include "mcuPinCfg.h"
#include "wyFlash.hpp"
#include "math.h"
#include "wySys.hpp"

#define Board_MCU_Timer1_Period 4800
#define Board_ServoCtrl_freq_Hz 50

const uint16_t Board_MCU_Timer1_PSC = (48000000) / (Board_MCU_Timer1_Period) / (Board_ServoCtrl_freq_Hz);
TIM::Timer boardTimer1(1, Board_MCU_Timer1_Period, Board_MCU_Timer1_PSC, 0);
TIM::pwmType boardCtrlSignal_Servo1;
TIM::pwmType boardCtrlSignal_Servo2;
TIM::pwmType boardCtrlSignal_Servo3;
GPIO::GpioPin boardElectromagnet(MCU_Pin_DCT);

#define ServoLeft boardCtrlSignal_Servo1
#define ServoRight boardCtrlSignal_Servo3
#define ServoInspireRobot boardCtrlSignal_Servo2

boardParaStructTypedef boardPara;
flash::FlashBlock boardParaFlash("U SUCK", sizeof(boardParaStructTypedef));

void boardInit()
{
    boardCtrlSignal_Servo1 = boardTimer1.setPwm(1, MCU_Pin_PWM_Tim1Ch1);
    boardCtrlSignal_Servo2 = boardTimer1.setPwm(2, MCU_Pin_PWM_Tim1Ch2);
    boardCtrlSignal_Servo3 = boardTimer1.setPwm(4, MCU_Pin_PWM_Tim1Ch4);
    boardElectromagnet = 0;

    if (boardParaFlash.isEmpty())
    {
        boardPara.InspireRobotPosiOff = 0;
        boardPara.InspireRobotPosiOn = 100;
        boardPara.ClickHoldTimeMs = 500;

        boardPara.LinkLength_left1 = 100;
        boardPara.LinkLength_left2 = 100;
        boardPara.LinkLength_right1 = 100;
        boardPara.LinkLength_right2 = 100;

        boardPara.ServoPosi_LeftX = -100;
        boardPara.ServoPosi_RightX = 100;
        boardPara.ServoPosi_LeftY = 0;
        boardPara.ServoPosi_RightY = 0;

        boardPara.ServoOffsetLeft = 0;
        boardPara.ServoOffsetRight = 0;

        boardParaFlash.save(&boardPara);
    }
    else
    {
        boardParaFlash.load(&boardPara);
    }
}

/// @brief
/// @param p target position of servo, from 0 to 100
/// @return pwm value
uint16_t boardServoPosition2PwmVal(uint8_t p)
{
    uint16_t retVal;
    float t = p;
    if (p > 100)
        return 0;
    t = (t * 0.02 + 0.5) * (Board_ServoCtrl_freq_Hz / 1000.0 * Board_MCU_Timer1_Period); // @ 50Hz
    retVal = t;
    return retVal;
}

void boardServoMove2Mid(void)
{
    *ServoLeft = boardServoPosition2PwmVal(50);
    *ServoRight = boardServoPosition2PwmVal(50);
}

void boardServoSetPosition(uint8_t idx, uint8_t p)
{
    if (p > 100)
        return;
    switch (idx)
    {
    case 0:
        *ServoLeft = boardServoPosition2PwmVal(p);
        break;
    case 1:
        *ServoRight = boardServoPosition2PwmVal(p);
        break;
    case 2:
        *ServoInspireRobot = boardServoPosition2PwmVal(p);
        break;
    default:
        break;
    }
}
#include "wyUart.hpp"
extern UART::Serial com;

#define __board_PI 3.141592653589793

inline float __boardGetServoAngle(float x, float y, float xServo, float yServo, float l1, float l2)
{
    float disSquare, dis;
    float cosVal;
    float theta;
    float px = x;
    float py = y;

    px -= xServo;
    py -= yServo;

    // disSquare = (x - xServo) * (x - xServo) + (y - yServo) * (y - yServo);
    disSquare = px * px + py * py;
    dis = sqrt(disSquare);

    cosVal = (disSquare + l1 * l1 - l2 * l2) / (2 * l1 * dis);
    com << "cos theta = " << (float)cosVal;
    cosVal = acosf(cosVal);
    com << "\r\ntheta = " << (float)cosVal;
    theta = atan2(py, px);
    // theta = -atan2f(-10, 140);

    com << "\r\n(x,y)=(" << (float)px << ',' << (float)py << ")\r\n";
    com << "atan(y,x) = " << (float)theta << "\r\n";

    if (theta < 0)
        theta = -theta;
    //     theta += (acos(cosVal) + (__board_PI / 2));
    // else
    //     theta -= (acos(cosVal) + (__board_PI / 2));
    return theta - cosVal - (__board_PI / 2);
}

void boardMove2Position(float x, float y)
{
    float angleLeft, angleRight;
    uint8_t servoPosi;

    // if (y > 0)
    //     return;

    angleLeft = -__boardGetServoAngle(
        x, y,
        boardPara.ServoPosi_LeftX, boardPara.ServoPosi_LeftY,
        boardPara.LinkLength_left1, boardPara.LinkLength_left2);

    angleRight = __boardGetServoAngle(
        x, y,
        boardPara.ServoPosi_RightX, boardPara.ServoPosi_RightY,
        boardPara.LinkLength_right1, boardPara.LinkLength_right2);

    while (angleRight < 0)
        angleRight += (__board_PI * 2);
    while (angleRight > (__board_PI * 2))
        angleRight -= (__board_PI * 2);

    while (angleLeft < 0)
        angleLeft += (__board_PI * 2);
    while (angleLeft > (__board_PI * 2))
        angleLeft -= (__board_PI * 2);

    angleLeft *= (100 / __board_PI);
    angleRight *= (100 / __board_PI);

    com << "left: " << (angleLeft * 1.8f) << '(' << angleLeft << ")\r\n";
    com << "right: " << (angleRight * 1.8f) << '(' << angleRight << ")\r\n";
    // angleRight = __board_PI - angleRight;

    servoPosi = angleLeft;
    *ServoLeft = boardServoPosition2PwmVal(servoPosi);
    servoPosi = angleRight;
    *ServoRight = boardServoPosition2PwmVal(servoPosi);
}

#include "string.h"
void boardParaSave()
{
    boardParaStructTypedef oldPara;

    boardParaFlash.load(&oldPara);
    if (memcmp(&boardPara, &oldPara, sizeof(boardParaStructTypedef)) != 0)
        boardParaFlash.save(&boardPara);
}

void boardParaReset() { boardParaFlash.load(&boardPara); }

void boardCilckScreen()
{
    boardElectromagnet = 1;
    *ServoInspireRobot = boardServoPosition2PwmVal(boardPara.InspireRobotPosiOn);
    sys::delayMs(boardPara.ClickHoldTimeMs);
    *ServoInspireRobot = boardServoPosition2PwmVal(boardPara.InspireRobotPosiOff);
    boardElectromagnet = 0;
}
