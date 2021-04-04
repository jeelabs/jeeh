#define JEEH 0

#if JEEH
#include <jee.h>
#endif

#include <mcu.h>

#if JEEH
UartDev< PinA<2>, PinA<15> > console;

int printf(char const* fmt, ...) {
    va_list ap; va_start(ap, fmt); veprintf(console.putc, fmt, ap); va_end(ap);
    return 0;
}
#endif

int main () {
#if JEEH
    console.init();
    printf("hello\n");
#else
    //enableSysTick();
#endif

    mcu::Pin led;
    led.define("B3:P");

    while (true) {
        led = 1;
        mcu::msWait(100);
        led = 0;
        mcu::msWait(400);
    }

    mcu::powerDown();
    mcu::systemReset();
}
