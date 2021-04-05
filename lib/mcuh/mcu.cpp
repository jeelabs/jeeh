#include "mcu.h"
#include <cstring>

// from CMSIS
extern uint32_t SystemCoreClock;
extern "C" void SystemCoreClockUpdate ();

extern "C" void abort () { ensure(0); }

// this is the recommended way to bail out
void mcu::systemReset () __attribute__ ((alias ("abort")));
// this appears to be used as placeholder in abstract class destructors
extern "C" void __cxa_pure_virtual () __attribute__ ((alias ("abort")));
// also bypass the std::{get,set}_terminate logic, and just abort
namespace std { void terminate () __attribute__ ((alias ("abort"))); }

namespace mcu {
    uint8_t Device::irqMap [(int) device::IrqVec::limit];
    Device* Device::devMap [20];  // large enough to handle all device objects
    uint32_t volatile ticks;

    void Pin::mode (int mval, int alt) const {
#if STM32F1
        (void) alt;
        RCC(0x18) |= (1<<_port) | (1<<0); // enable GPIOx and AFIO clocks
        x // FIXME wrong, mode bits have changed ...
        if (mval == 0b1000 || mval == 0b1100) {
            gpio32(ODR)[_pin] = mval & 0b0100;
            mval = 0b1000;
        }
        gpio32(0x00).mask(4*_pin, 4) = mval; // CRL/CRH
#else
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
#endif // STM32F1
    }

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
                          m = (m & ~0b11) | 0b10;   // m=10 alt mode
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
        SCB(0xD10)[2] = 1; // set SLEEPDEEP
        asm ("wfe");
    }

    Device::Device () {
        for (auto& e : devMap)
            if (e == nullptr) {
                e = this;
                _id = &e - devMap;
                return;
            }
        ensure(0); // ran out of unused device slots
    }

    Device::~Device () {
        devMap[_id] = nullptr;
        // TODO clear irqMap entries and NVIC enables
    }

    void Device::installIrq (uint32_t irq) {
        ensure(irq < sizeof irqMap);
        irqMap[irq] = _id;
        NVIC(4*(irq>>5)) = 1 << (irq&0x1F); // not in bit-band region
    }

    auto systemClock () -> uint32_t {
        return SystemCoreClock;
    }

    extern "C" void SysTick_Handler () {
        ++ticks;
    }

    void msWait (uint16_t ms) {
        auto hz = SystemCoreClock;
        SCB(0x14) = ms*(hz/1000)-1; // reload
        SCB(0x18) = 0;              // current
        SCB(0x10) = 0b111;          // control & status

        auto t = ticks;
        while (t == ticks)
            asm ("wfi");
    }

    auto millis () -> uint32_t {
        return ticks;
    }

#if STM32L4
    void enableClkWithPLL () { // using internal 16 MHz HSI
        FLASH(0x00) = 0x704;                    // flash ACR, 4 wait states
        RCC(0x00)[8] = 1;                       // HSION
        while (RCC(0x00)[10] == 0) {}           // wait for HSIRDY
        RCC(0x0C) = (1<<24) | (10<<8) | (2<<0); // 160 MHz w/ HSI
        RCC(0x00)[24] = 1;                      // PLLON
        while (RCC(0x00)[25] == 0) {}           // wait for PLLRDY
        RCC(0x08) = 0b11;                       // PLL as SYSCLK
    }

    void enableClkMaxMSI () { // using internal 48 MHz MSI
        FLASH(0x00) = 0x702;            // flash ACR, 2 wait states
        RCC(0x00)[0] = 1;               // MSION
        while (RCC(0x00)[1] == 0) {}    // wait for MSIRDY
        RCC(0x00).mask(3, 5) = 0b10111; // MSI 48 MHz
        RCC(0x08) = 0b00;               // MSI as SYSCLK
    }

    void enableClkSaver (int range) { // using MSI at 100 kHz to 4 MHz
        RCC(0x00)[0] = 1;                // MSION
        while (RCC(0x00)[1] == 0) {}     // wait for MSIRDY
        RCC(0x08) = 0b00;                // MSI as SYSCLK
        RCC(0x00) = (range<<4) | (1<<3); // MSI 48 MHz, ~HSION, ~PLLON
        FLASH(0x00) = 0;                 // no ACR, no wait states
    }
#endif

    auto fastClock (bool pll) -> uint32_t {
        PWR(0x00).mask(9, 2) = 0b01;    // VOS range 1
        while (PWR(0x14)[10] != 0) {}   // wait for ~VOSF
        if (pll) enableClkWithPLL(); else enableClkMaxMSI();
        return SystemCoreClock = pll ? 80000000 : 48000000;
    }
    auto slowClock (bool low) -> uint32_t {
        enableClkSaver(low ? 0b0000 : 0b0110);
        PWR(0x00).mask(9, 2) = 0b10;    // VOS range 2
        return SystemCoreClock = low ? 100000 : 4000000;
    }

    extern "C" void irqDispatch () {
        uint8_t idx = SCB(0xD04); // ICSR
        Device::devMap[Device::irqMap[idx-16]]->irqHandler();
    }
}

// to re-generate "irqs.h", see the "all-irqs.sh" script

#define IRQ(f) void f () __attribute__ ((alias ("irqDispatch")));
extern "C" {
#include "irqs.h"
}
