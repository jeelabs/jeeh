#define JEEH 1

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
#else

#endif

mcu::Pin leds [7];

int main () {
#if JEEH
    console.init();
    for (int i = 0; i < 1000000; ++i) asm ("");
    printf("hello\n");
#endif

    mcu::Pin::define("A6:P,A5:P,A4:P,A3:P,A1:P,A0:P,B3:P", leds, 7);

    struct Console : mcu::Uart {
        Console (int n) : Uart (n) {
            init();
            baud(115200, F_CPU);
        }

        void trigger () override {
            leds[6].toggle();
        }
    };

    mcu::Pin tx, rx; // TODO use the altpins info
#if JEEH
    tx.define("A9:PU7");
    rx.define("A10:PU7");
    Console console (1);
#else
    tx.define("A2:PU7");
    rx.define("A15:PU3");
    Console console (2);
#endif

    leds[6] = 1;
    while (true) {
        //leds[6] = 1;
        //mcu::msWait(100);
        //leds[6] = 0;
        mcu::msWait(400);
    }

    //mcu::powerDown();
    mcu::systemReset();
}
