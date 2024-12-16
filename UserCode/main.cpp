#include "wySys.hpp"
#include "mcuPinCfg.h"
#include "wyGpio.hpp"
#include "board.h"
#include "communication.h"

GPIO::GpioPin led(MCU_Pin_LED);

int main()
{
    led = 1;

    boardInit();
    communicationInit();

    while (1)
    {
        led.flip();
        sys::delayMs(500);
    }
    return 0;
}
