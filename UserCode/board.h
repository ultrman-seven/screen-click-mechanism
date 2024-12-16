#ifndef EC1515F8_9511_4A90_81A7_E4883C204EBA
#define EC1515F8_9511_4A90_81A7_E4883C204EBA
#include "stdint.h"

typedef struct
{
    float InspireRobotPosiOff;
    float InspireRobotPosiOn;

    float LinkLength_left1;
    float LinkLength_left2;
    float ServoPosi_LeftX;
    float ServoPosi_LeftY;
    float ServoOffsetLeft;

    float LinkLength_right1;
    float LinkLength_right2;
    float ServoPosi_RightX;
    float ServoPosi_RightY;
    float ServoOffsetRight;

    uint32_t ClickHoldTimeMs;
} boardParaStructTypedef;

extern boardParaStructTypedef boardPara;

// TODO: Write Comment

/// @brief initialization
void boardInit();

/// @brief
/// @param x
/// @param y
void boardMove2Position(float x, float y);

/// @brief
void boardParaSave();

/// @brief
void boardParaReset();

/// @brief
void boardServoMove2Mid();

/// @brief
void boardCilckScreen();

/// @brief
/// @param idx 0: left; 1: right; 2 inspire
/// @param p position
void boardServoSetPosition(uint8_t idx, uint8_t p);
#endif /* EC1515F8_9511_4A90_81A7_E4883C204EBA */
