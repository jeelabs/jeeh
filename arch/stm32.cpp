#include "jee.h"

#if STM32F1

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
    enableClkAt72MHz();                 // using external crystal
    enableSysTick(hz/1000);             // systick once every 1 ms
    return hz;
}

void powerDown (bool standby) {
    Periph::bit(Periph::rcc+0x1C, 28) = 1; // PWREN
    Periph::bit(Periph::pwr, 1) = standby ? 1 : 0;  // PDDS if standby

    constexpr uint32_t scr = 0xE000ED10;
    MMIO32(scr) |= (1<<2);  // set SLEEPDEEP

    __asm("wfe");
}

#elif STM32F4

void enableClkAt168MHz () {
    MMIO32(Periph::flash+0x00) = 0x705; // flash acr, 5 wait states
    MMIO32(Periph::rcc+0x00) = (1<<16); // HSEON
    while (Periph::bit(Periph::rcc+0x00, 17) == 0) {} // wait for HSERDY
    MMIO32(Periph::rcc+0x08) = (4<<13) | (5<<10) | (1<<0); // prescaler w/ HSE
    MMIO32(Periph::rcc+0x04) = (7<<24) | (1<<22) | (0<<16) | (336<<6) |
                                (XTAL<<0);
    Periph::bit(Periph::rcc+0x00, 24) = 1; // PLLON
    while (Periph::bit(Periph::rcc+0x00, 25) == 0) {} // wait for PLLRDY
    MMIO32(Periph::rcc+0x08) = (4<<13) | (5<<10) | (2<<0);
}

int fullSpeedClock () {
    constexpr uint32_t hz = 168000000;
    enableClkAt168MHz();                 // using external crystal
    enableSysTick(hz/1000);              // systick once every 1 ms
    return hz;
}

void powerDown (bool standby) {
    Periph::bit(Periph::rcc+0x40, 28) = 1; // PWREN
    Periph::bit(Periph::pwr, 1) = standby ? 1 : 0;  // PDDS if standby

    constexpr uint32_t scr = 0xE000ED10;
    MMIO32(scr) |= (1<<2);  // set SLEEPDEEP

    __asm("wfe");
}

#elif STM32F7

void enableClkAt216MHz () {
    MMIO32(Periph::flash+0x00) = 0x307; // flash acr, 7 wait states
    MMIO32(Periph::rcc+0x00) = (1<<16); // HSEON
    while (Periph::bit(Periph::rcc+0x00, 17) == 0) {} // wait for HSERDY
    MMIO32(Periph::rcc+0x08) = (4<<13) | (5<<10) | (1<<0); // prescaler w/ HSE
    MMIO32(Periph::rcc+0x04) = (8<<24) | (1<<22) | (0<<16) | (432<<6) |
                                (XTAL<<0);
    Periph::bitSet(Periph::rcc+0x00, 24); // PLLON
    while (Periph::bit(Periph::rcc+0x00, 25) == 0) {} // wait for PLLRDY
    MMIO32(Periph::rcc+0x08) = (0b100<<13) | (0b101<<10) | (0b10<<0);
}

int fullSpeedClock () {
    constexpr uint32_t hz = 216000000;
    enableClkAt216MHz();                 // using external crystal
    enableSysTick(hz/1000);              // systick once every 1 ms
    return hz;
}

#elif STM32H7

void enableClkPll (int freq) {
    Periph::bitClear(Periph::pwr+0x0C, 2); // CR3 SCUEN
    MMIO32(Periph::pwr+0x18) = (0b11<<14); // D3CR VOS = Scale 1
    while (Periph::bit(Periph::pwr+0x18, 13) == 0) {} // wait for VOSRDY
    MMIO32(Periph::flash+0x00) = 0x24; // flash acr, 4 wait states

    Periph::bitSet(Periph::rcc+0x154, 1); // APB4ENR SYSCFGEN
    constexpr uint32_t syscfg = 0x58000400;
    Periph::bitSet(syscfg+0x20, 0); // CCCSR EN (I/O compensation)

    MMIO32(Periph::rcc+0x28) = (XTAL<<4) | (0b10<<0); // prescaler w/ HSE
    Periph::bitSet(Periph::rcc+0x00, 16); // HSEON
    while (Periph::bit(Periph::rcc+0x00, 17) == 0) {} // wait for HSERDY
    MMIO32(Periph::rcc+0x2C) = 0x00070000; // powerup default is 0! (doc err?)
    MMIO32(Periph::rcc+0x30) = (0<<24) | (1<<16) | (0<<9) | ((freq-1)<<0);
    Periph::bitSet(Periph::rcc+0x00, 24); // PLL1ON
    while (Periph::bit(Periph::rcc+0x00, 25) == 0) {} // wait for PLL1RDY
    MMIO32(Periph::rcc+0x18) = (0b100<<4) | (0b1000<<0); // APB3/2, AHB/2
    MMIO32(Periph::rcc+0x1C) = (0b100<<8) | (0b100<<4);  // APB2/2, APB1/2
    MMIO32(Periph::rcc+0x20) = (0b100<<4);               // APB4/2
    MMIO32(Periph::rcc+0x10) = (0b11<<0); // switch to PLL1
    while ((MMIO32(Periph::rcc+0x10) & 0b11000) != 0b11000) {} // wait switched
}

int fullSpeedClock () {
    bool max480 = (MMIO32(0x5C001000)>>16) == 0x2003; // revision V
    uint32_t mhz = max480 ? 480 : 400;
    enableClkPll(mhz);                   // using external crystal
    enableSysTick(1000*mhz);             // systick once every 1 ms
    return 1000000 * mhz;
}

#elif STM32L0

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

#elif STM32L4

void enableClkAt80MHz () {
    MMIO32(Periph::flash + 0x00) = 0x704; // flash acr, 4 wait states
    MMIO32(Periph::rcc + 0x00) = (1<<8); // HSION
    while ((MMIO32(Periph::rcc + 0x00) & (1<<10)) == 0) ; // wait for HSIRDY
    MMIO32(Periph::rcc + 0x0C) = (1<<24) | (10<<8) | (2<<0); // 160 MHz w/ HSI
    MMIO32(Periph::rcc + 0x00) |= (1<<24); // PLLON
    while ((MMIO32(Periph::rcc + 0x00) & (1<<25)) == 0) ; // wait for PLLRDY
	MMIO32(Periph::rcc + 0x08) = (3<<0); // select PLL as SYSCLK, 80 MHz
}

int fullSpeedClock () {
    constexpr uint32_t hz = 80000000;
    enableClkAt80MHz();              // using internal 16 MHz HSI
    enableSysTick(hz/1000);          // systick once every 1 ms
    MMIO32(0x40011008) = hz/115200;  // usart1: 115200 baud @ 80 MHz
    return hz;
}

void powerDown (bool standby) {
    MMIO32(Periph::rcc+0x58) |= (1<<28); // PWREN

    // set to either shutdown or stop 1 mode
    MMIO32(Periph::pwr+0x00) = (0b01<<9) | (standby ? 0b100 : 0b001);
    MMIO32(Periph::pwr+0x18) = 0b11111; // clear CWUFx in PWR_SCR

    constexpr uint32_t scr = 0xE000ED10;
    MMIO32(scr) |= (1<<2);  // set SLEEPDEEP

    asm ("wfe");
}

#endif
