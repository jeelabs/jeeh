#pragma once

#include <stdarg.h>
#include <stdint.h>

#define MMIO32(x) (*(volatile uint32_t*) (x))
#define MMIO16(x) (*(volatile uint16_t*) (x))
#define MMIO8(x)  (*(volatile uint8_t*) (x))

// general-purpose ring buffer

template< int N >
class RingBuffer {
    uint16_t volatile in, out;
    uint8_t volatile buf [N];

public:
    RingBuffer () : in (0), out (0) {}

    int avail () const {
        int r = in - out;
        return r >= 0 ? r : r + N;
    }

    bool free () const {
        return avail() < N-1;
    }

    bool empty () const {
        return in == out;
    }

    void put (uint8_t v) {
        uint16_t pos = in;
        buf[pos++] = v;
        in = pos < N ? pos : 0;
    }

    uint8_t get () {
        uint16_t pos = out;
        uint8_t v = buf[pos++];
        out = pos < N ? pos : 0;
        return v;
    }
};

// interrupt vector table in ram

struct VTable;
extern VTable& VTableRam ();

extern void wait_ms (uint32_t ms);

// architecture-specific definitions

#if STM32F1
#include "arch/stm32f1.h"
#elif STM32F4
#include "arch/stm32f4.h"
#elif STM32F7
#include "arch/stm32f7.h"
#elif STM32G0
#include "arch/stm32g0.h"
#elif STM32H7
#include "arch/stm32h7.h"
#elif STM32L0
#include "arch/stm32l0.h"
#elif STM32L4
#include "arch/stm32l4.h"
#elif ARDUINO_ARCH_AVR
#include "arch/avr328.h"
#elif ARDUINO_ARCH_ESP32
#include "arch/esp32.h"
#elif ARDUINO_TEENSY40
#include "arch/teensy.h"
#elif ESP_PLATFORM
#include "arch/espidf.h"
#else
#error no architecture defined
#endif

// shorthand: PinA<1> expands to Pin<'A',1>, etc

template <int N> using PinA = Pin<'A',N>;
template <int N> using PinB = Pin<'B',N>;
template <int N> using PinC = Pin<'C',N>;
template <int N> using PinD = Pin<'D',N>;
template <int N> using PinE = Pin<'E',N>;
template <int N> using PinF = Pin<'F',N>;
template <int N> using PinG = Pin<'G',N>;
template <int N> using PinH = Pin<'H',N>;
template <int N> using PinI = Pin<'I',N>;
template <int N> using PinJ = Pin<'J',N>;
template <int N> using PinK = Pin<'K',N>;

// systick and delays

#ifndef ticks
extern uint32_t volatile ticks;
#endif

template< uint32_t HZ >
struct SysTick {
    // FIXME this is ARM-specific, must move elsewhere to support other µCs
    constexpr static uint32_t tick = 0xE000E000;

    static uint32_t micros () {
        uint32_t t, u;
        do {
            t = ticks;
            u = MMIO32(tick + 0x18);
        } while (t != ticks);
        uint32_t v = MMIO32(tick + 0x14) + 1 - u;
        // keep as much precision as possible, i.e. sysclk in 10 KHz steps
        return t * 1000 + ((v * 100) / (HZ / 10000));
    }

    static void wait_us (uint32_t n) {
        uint32_t t = micros();
        while ((uint32_t) (micros() - t) < n) ;
    }
};

// slowed-down pin, adds a configurable delay after setting the pin

template< typename T, int N >
struct SlowPin : public T {

    static void write (int v) {
        for (int i = 0; i <= N; ++i)
            T::write(v);  // simply set a few times to consume more time
    }

    // shorthand
    void operator= (int v) const { write(v); }
};

// dummy pin, this ignores all calls

struct NoPin {
    static void mode (Pinmode) {}
    static int read () { return 0; }
    static void write (int) {}
    static void toggle () {}

    operator int () const { return read(); }
    void operator= (int v) const { write(v); }
};

// spi, bit-banged on any gpio pins

template< typename MO, typename MI, typename CK, typename SS, int CP =0 >
struct SpiGpio {
    static void init () {
        SS::mode(Pinmode::out); disable();
        CK::mode(Pinmode::out); CK::write(CP);
        MI::mode(Pinmode::in_float);
        MO::mode(Pinmode::out);
    }

    static void enable () { SS::write(0); }
    static void disable () { SS::write(1); }

    static uint8_t transfer (uint8_t v) {
        for (int i = 0; i < 8; ++i) {
            MO::write(v & 0x80);
            v <<= 1;
            CK::write(!CP);
            v |= MI::read();
            CK::write(CP);
        }
        return v;
    }
};

// i2c, bit-banged on any gpio pins

template< typename SDA, typename SCL, int N =0 >
class I2cBus {
    static void hold () {
        for (int i = 0; i < N; ++i)
            __asm("");
    }
    static void sclLo () {
        hold();
        scl = 0;
    }
    static void sclHi () {
        hold();
        scl = 1;
        for (int i = 0; i < 10000; ++i)
            if (scl)
                break;
    }

public:
    I2cBus () {
        sda.mode(Pinmode::out_od); sda = 1;
        scl.mode(Pinmode::out_od); scl = 1;
    }

    static bool start(int addr) {
        sclLo();
        sclHi();
        sda = 0;
        return write(addr);
    }

    static void stop() {
        sda = 0;
        sclHi();
        sda = 1;
    }

    static bool write(int data) {
        sclLo();
        for (int mask = 0x80; mask != 0; mask >>= 1) {
            sda = data & mask;
            sclHi();
            sclLo();
        }
        sda = 1;
        sclHi();
        bool ack = !sda;
        sclLo();
        return ack;
    }

    static uint8_t read(bool last) {
        uint8_t data = 0;
        for (int mask = 0x80; mask != 0; mask >>= 1) {
            sclHi();
            if (sda)
                data |= mask;
            sclLo();
        }
        sda = last;
        sclHi();
        sclLo();
        if (last)
            stop();
        sda = 1;
        return data;
    }

    static SDA sda;
    static SCL scl;
};

template< typename SDA, typename SCL, int N >
SDA I2cBus<SDA,SCL,N>::sda;

template< typename SDA, typename SCL, int N >
SCL I2cBus<SDA,SCL,N>::scl;

// formatted output

extern void putInt (void (*emit)(int), int v, int b =10, int w =0, char f =' ');
extern void veprintf (void (*emit)(int), const char* fmt, va_list ap);
extern "C" int printf (const char* fmt, ...);  // to be defined in app
