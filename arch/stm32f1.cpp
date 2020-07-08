#if STM32F1

#include "jee.h"

void powerDown (bool standby) {
    Periph::bit(Periph::rcc+0x1C, 28) = 1; // PWREN
    Periph::bit(Periph::pwr, 1) = standby ? 1 : 0;  // PDDS if standby

    constexpr uint32_t scr = 0xE000ED10;
    MMIO32(scr) |= (1<<2);  // set SLEEPDEEP

    __asm("wfe");
}

void enableClkAt8MHz () {  // [1] p.49
    constexpr uint32_t rcc = Periph::rcc;

    Periph::bit(rcc+0x00, 16) = 1; // rcc cr, set HSEON
    while (Periph::bit(rcc+0x00, 17) == 0) {} // wait for HSERDY
    MMIO32(rcc+0x04) = (1<<0);  // hse, no pll [1] pp.100
}

void enableClkAt72MHz () {  // [1] p.49
    constexpr uint32_t rcc = Periph::rcc;

    MMIO32(Periph::flash+0x00) = 0x12; // flash acr, two wait states
    Periph::bit(rcc+0x00, 16) = 1; // rcc cr, set HSEON
    while (Periph::bit(rcc+0x00, 17) == 0) {} // wait for HSERDY
    // 8 MHz xtal src, pll 9x, pclk1 = hclk/2, adcpre = pclk2/6 [1] pp.100
    MMIO32(rcc+0x04) = (7<<18) | (1<<16) | (2<<14) | (4<<8) | (2<<0);
    Periph::bit(rcc+0x00, 24) = 1; // rcc cr, set PLLON
    while (Periph::bit(rcc+0x00, 25) == 0) {} // wait for PLLRDY
}

int fullSpeedClock () {
    constexpr uint32_t hz = 72000000;
    enableClkAt72MHz();                 // using external 8 MHz crystal
    enableSysTick(hz/1000);             // systick once every 1 ms
    return hz;
}

#endif
