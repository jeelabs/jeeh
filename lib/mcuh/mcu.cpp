#include "mcu.h"
#include <cstring>

namespace mcu {
    extern "C" void SystemCoreClockUpdate (); // from CMSIS

#if STM32F1
    void Pin::mode (int mval, int /*ignored*/) const {
        RCC(0x18) |= (1<<_port) | (1<<0); // enable GPIOx and AFIO clocks
        // FIXME wrong, mode bits have changed ...
        if (mval == 0b1000 || mval == 0b1100) {
            gpio32(ODR)[_pin] = mval & 0b0100;
            mval = 0b1000;
        }
        gpio32(0x00).mask(4*_pin, 4) = mval; // CRL/CRH
    }
#else
    void Pin::mode (int mval, int alt) const {
        // enable GPIOx clock
#if STM32F3
        RCC(0x14)[_port] = 1;
#elif STM32F4 | STM32F7
        RCC(0x30)[_port] = 1;
        asm ("dsb");
#elif STM32G0
        RCC(0x34)[_port] = 1;
#elif STM32H7
        RCC(0xE0)[_port] = 1;
        asm ("dsb");
#elif STM32L0
        RCC(0x2C)[_port] = 1;
#elif STM32L4
        RCC(0x4C)[_port] = 1;
#endif

        gpio32(0x00).mask(2*_pin, 2) = mval;      // MODER
        gpio32(0x04).mask(  _pin, 1) = mval >> 2; // TYPER
        gpio32(0x08).mask(2*_pin, 2) = mval >> 3; // OSPEEDR
        gpio32(0x0C).mask(2*_pin, 2) = mval >> 5; // PUPDR
        gpio32(0x20).mask(4*_pin, 4) = alt;       // AFRL/AFRH
    }
#endif

    auto Pin::mode (char const* desc) const -> bool {
        int m = 0, a = 0;
        for (auto s = desc; *s != ',' && *s != 0; ++s)
            switch (*s) {     // pp ss t mm
                case 'A': m  = 0b00'00'0'11; break; // m=11 analog
                case 'F': m  = 0b00'00'0'00; break; // m=00 float
                case 'D': m |= 0b10'00'0'00; break; // m=00 pull-down
                case 'U': m |= 0b01'00'0'00; break; // m=00 pull-up
                                          
                case 'P': m  = 0b00'01'0'01; break; // m=01 push-pull
                case 'O': m  = 0b00'01'1'01; break; // m=01 open drain
                                          
                case 'L': m &= 0b11'00'1'11; break; // s=00 low speed
                case 'N':                    break; // s=01 normal speed
                case 'H': m ^= 0b00'11'0'00; break; // s=10 high speed
                case 'V': m |= 0b00'10'0'00; break; // s=11 very high speed

                default:  if (*s < '0' || *s > '9' || a > 1) return false;
                          m = (m & ~0b00) | 0b10;   // m=10 alt mode
                          a = 10 * a + *s - '0';
                case ',': break; // valid as terminator
            }
        mode(m, a);
        return true;
    }

    auto Pin::define (char const* d, Pin* v, int n) -> char const* {
        while (--n >= 0 && v->define(d)) {
            ++v;
            auto p = strchr(d, ',');
            if (n == 0)
                return *d != 0 ? p : nullptr; // point to comma if more
            if (p == nullptr)
                break;
            d = p+1;
        }
        return d;
    }

    void powerDown (bool standby) {
        switch (FAMILY) {
            case STM_F4:
                RCC(0x40)[28] = 1; // PWREN
                PWR(0x00)[1] = standby; // PDDS if standby
                break;                
            case STM_L4:
                RCC(0x58)[28] = 1; // PWREN
                // set to either shutdown or stop 1 mode
                PWR(0x00) = (0b01<<9) | (standby ? 0b100 : 0b001);
                PWR(0x18) = 0b1'1111; // clear CWUFx
                break;
        }
        SCB(0x10)[2] = 1; // set SLEEPDEEP
        asm ("wfe");
    }

    void systemReset () {
        SCB(0x0C) = (0x5FA<<16) | (1<<2); // SCB AIRCR reset
        while (true) {}
    }
}
