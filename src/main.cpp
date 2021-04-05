#if JEEH
#include <jee.h>
#endif

#include <mcu.h>
#include <cstring>

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

namespace mcu {
    struct Serial : Uart {
        Serial (int n) : Uart (n) {
            Pin tx, rx; // TODO use the altpins info
#if JEEH
            tx.define("A9:PU7");
            rx.define("A10:PU7");
#else
            tx.define("A2:PU7");
            rx.define("A15:PU3");
#endif
            init();
            baud(115200, systemClock());
        }

        void trigger () override {
            leds[5].toggle();
        }

        void yield () {
            leds[1] = 0;
            asm ("wfi"); // TODO
            leds[1] = 1;
        }

        struct Chunk { uint8_t* buf; uint32_t len; };

        auto recv () -> Chunk {
            while (true) {
                auto end = rxFill();
                if (end != rxNext) {
                    if (end < rxNext)
                        end = sizeof rxBuf;
                    ensure(end > rxNext);
                    return {rxBuf+rxNext, (uint32_t) (end-rxNext)};
                }
                yield();
            }
        }

        void didRecv (uint32_t n) {
            rxNext = (rxNext + n) % sizeof rxBuf;
        }

        auto canSend () -> Chunk {
            while (txBusy())
                yield();
            txNext = 0;
            return {txBuf, sizeof txBuf}; // TODO no double buffering yet
        }

        void send (uint32_t n) {
            txStart(txBuf + txNext, n);
            txNext = (txNext + n) % sizeof txBuf;
        }

    private:
        uint16_t rxNext = 0, txNext = 0;
        uint8_t txBuf [100];
    };
}

int main () {
#if JEEH
    console.init();
    for (int i = 0; i < 1000000; ++i) asm ("");
    auto n = printf("hello %u\n", sizeof (Printer));
    printf("%d bytes [%0*d]\n", n, 10, -1234567);
    char buf [5];
    n = mcu::snprintf(buf, sizeof buf, "<%d>", 123456789);
    printf("1: %s %d\n", buf, n);
    n = mcu::snprintf(buf, 0, "<%d>", 7654321);
    printf("2: %s %d\n", buf, n);
#endif

    mcu::Pin::define("A6:P,A5:P,A4:P,A3:P,A1:P,A0:P,B3:P", leds, 7);


#if JEEH
    mcu::Serial serial (1);
#else
    mcu::fastClock();
    mcu::Serial serial (2);
    if (mcu::systemClock() < 4000000)
        serial.baud(4800, mcu::systemClock());
#endif
    leds[5] = 1;
    serial.txStart("abc", 3);

    while (true) {
#if 0
        leds[6] = 1;
        mcu::msWait(100);
        leds[6] = 0;
        mcu::msWait(200);
        mcu::msWait(200);
        serial.txStart("abc", 3);
#else
        auto [ptr, len] = serial.canSend();
        auto x = "akjsfh lakjsdhg laksjdfh laksjdfh alksdjgh asldkjfhalsdkj\n";
        auto n = strlen(x);
        ensure(n < len);
        memcpy(ptr, x, n);
        serial.send(n);
        //serial.txStart(x, n);

        auto [PTR, LEN] = serial.canSend();
        auto X = "AKJSFH LAKJSDHG LAKSJDFH LAKSJDFH ALKSDJGH ASLDKJFHALSDK\n";
        auto N = strlen(X);
        ensure(N < LEN);
        memcpy(PTR, X, N);
        serial.send(N);
        //serial.txStart(X, N);
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
