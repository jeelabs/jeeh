#include <mcu.h>

#if 0
UartBufDev< PinA<2>, PinA<15>, 100 > console;

int printf(char const* fmt, ...) {
    va_list ap; va_start(ap, fmt); veprintf(console.putc, fmt, ap); va_end(ap);
    return 0;
}
#endif

int main () {
#if 0
    console.init();
    console.baud(921600, fullSpeedClock());
    wait_ms(500);
    printf("hello\n");
#else
    //enableSysTick();
#endif

    mcu::Pin led;
    led.define("B3:P");

    while (true) {
        led = 1;
        //wait_ms(100);
        led = 0;
        //wait_ms(400);
    }

    mcu::powerDown();
    mcu::systemReset();
}
