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
            mcu::Pin tx, rx; // TODO use the altpins info
#if JEEH
            tx.define("A9:PU7");
            rx.define("A10:PU7");
#else
            tx.define("A2:PU7");
            rx.define("A15:PU3");
#endif
            init();
            baud(115200, mcu::systemClock());
            leds[4].toggle();
        }

        void trigger () override {
//printf("!");
            leds[5].toggle();
        }
    };

#if JEEH
    Console serial (1);
#else
    mcu::fastClock();
    Console serial (2);
    if (mcu::systemClock() < 4000000)
        serial.baud(4800, mcu::systemClock());
#endif
    leds[5] = 1;
    serial.txStart("abc", 3);

    while (true) {
        leds[6] = 1;
        mcu::msWait(100);
        leds[6] = 0;
        mcu::msWait(200);
        mcu::msWait(200);
        serial.txStart("abc", 3);
#if JEEH
        auto t = mcu::millis(), t2 = t;
        printf("%b:", t);
        while (t != 0) {
            auto i = __builtin_clz(t); // number of trailing zero bits
            printf(" %d", i);
            t ^= 1<<(31-i);
        }
        printf(" #%d\n", __builtin_clz(t));
#endif
    }

    //mcu::powerDown();
    mcu::systemReset();
}
