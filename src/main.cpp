#if JEEH
#include <jee.h>
#endif

#include <mcu.h>

#if JEEH
UartDev< PinA<2>, PinA<15> > console;

#if 0
int printf(char const* fmt, ...) {
    va_list ap; va_start(ap, fmt); veprintf(console.putc, fmt, ap); va_end(ap);
    return 0;
}
#endif
#endif

#include <cstdarg>
#include "printer.h"

Printer printer (nullptr, [](void*, uint8_t const* ptr, int len) {
    while (--len >= 0)
        console.putc(*ptr++);
});

extern "C" int printf (const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int result = printer.vprintf(fmt, ap);
    va_end(ap);

    return result;
}

mcu::Pin leds [7];

void mcu::failAt (void* pc, void const* lr) {
#if JEEH
    printf("failAt %p %p\n", pc, lr);
    for (uint32_t i = 0; i < systemClock() >> 16; ++i) {}
#else
    while (true) {
        leds[0].toggle(); // fast blink
        for (uint32_t i = 0; i < systemClock() >> 8; ++i) {}
    }
#endif
    mcu::SCB(0xD0C) = (0x5FA<<16) | (1<<2); // SCB AIRCR reset
    while (true) {}
}

void mcu::idle () {
    leds[1] = 0;
    asm ("wfi");
    leds[1] = 1;
}

extern "C" void SystemCoreClockUpdate ();

int main () {
#if JEEH
    console.init();
    enableClkAt80MHz(); // not systick etc
    SystemCoreClockUpdate();
    console.baud(115200, 80000000);
    for (int i = 0; i < 10000000; ++i) asm ("");
    auto n = printf("hello %u\n", sizeof (Printer));
    printf("%d bytes [%0*d]\n", n, 10, -1234567);
    char buf [5];
    n = mcu::snprintf(buf, sizeof buf, "<%d>", 123456789);
    printf("1: %s %d\n", buf, n);
    n = mcu::snprintf(buf, 0, "<%d>", 7654321);
    printf("2: %s %d\n", buf, n);
#endif

    mcu::Pin::define("A6:P,A5:P,A4:P,A3:P,A1:P,A0:P,B3:P", leds, 7);

    mcu::Pin tx, rx; // TODO use the altpins info
#if JEEH
    tx.define("A9:PU7");
    rx.define("A10:PU7");
    mcu::Uart serial (1);
#else
    mcu::fastClock();
    tx.define("A2:PU7");
    rx.define("A15:PU3");
    mcu::Uart serial (2);
#endif
    serial.init();
    auto hz = mcu::systemClock();
    serial.baud(hz < 4000000 ? 4800 : 921600, hz);
    printf("%d Hz\n", hz);
    for (int i = 0; i < 10000000; ++i) asm ("");

    leds[5] = 1;

    while (true) {
#if 1
        leds[6] = 1;
        mcu::msWait(100);
        leds[6] = 0;
        mcu::msWait(200);
        //mcu::msWait(200);
        serial.send((uint8_t const*) "abc\n", 4);
#else
        for (auto x = "12345678901234567890123456789012345678901234567890123456789012345678901234567890/\n";
                *x != 0; ++x)
            serial.send((uint8_t const*) x, strlen(x));

        auto [rp, rn] = serial.recv();
        printf("%p %u\n", rp, rn);
        serial.didRecv(rn);
#endif
#if 0
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
