#if STM32L0

#include <jee.h>

void enableClkHSI16 () {  // [1] p.49
    constexpr uint32_t rcc_cr   = Periph::rcc + 0x00;
    constexpr uint32_t rcc_cfgr = Periph::rcc + 0x0C;

    MMIO32(Periph::flash + 0x00) = 0x03; // flash acr, 1 wait, enable prefetch

    // switch to HSI 16 and turn everything else off
    MMIO32(rcc_cr) |= (1<<0); // turn hsi16 on
    MMIO32(rcc_cfgr) = 0x01;  // revert to hsi16, no PLL, no prescalers
    MMIO32(rcc_cr) = 0x01;    // turn off MSI, HSE, and PLL
    while ((MMIO32(rcc_cr) & (1<<25)) != 0) ; // wait for PPLRDY to clear
}

void enableClkPll () {
    constexpr uint32_t rcc_cr   = Periph::rcc + 0x00;
    constexpr uint32_t rcc_cfgr = Periph::rcc + 0x0C;

    MMIO32(rcc_cfgr) |= 1<<18 | 1<<22; // set PLL src HSI16, PLL x4, PLL div 2
    MMIO32(rcc_cr) |= 1<<24; // turn PLL on
    while ((MMIO32(rcc_cr) & (1<<25)) == 0) ; // wait for PPLRDY
    MMIO32(rcc_cfgr) |= 0x3; // set system clk to PLL
}

int fullSpeedClock (bool pll) {
    enableClkHSI16();
    uint32_t hz = 16000000;
    if (pll) {
        hz = 32000000;
        enableClkPll();
    }
    enableSysTick(hz/1000); // systick once every 1 ms
    return hz;
}

void powerDown (bool standby) {
    MMIO32(Periph::rcc+0x38) |= (1<<28); // PWREN

    // LDO range 2, FWU, ULP, DBP, CWUF, PDDS (if standby), LPSDSR
    MMIO32(Periph::pwr) = (0b10<<11) | (1<<10) | (1<<9) | (1<<8) | (1<<2) |
                            ((standby ? 1 : 0)<<1) | (1<<0);

    constexpr uint32_t scr = 0xE000ED10;
    MMIO32(scr) |= (1<<2);  // set SLEEPDEEP

    asm ("wfe");
}

#endif
