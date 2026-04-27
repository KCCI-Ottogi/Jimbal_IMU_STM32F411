//#include "driver/uart.h"

// #include "hw_def.h"
#include "hw.h"

void hwInit(void)
{
    ledInit();
    uartInit();
    cliInit();
    buttonInit();
    tempInit();
    // uartOpen(0, 9600);

}