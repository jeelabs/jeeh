#if STM32H7

#include <jee.h>

void enableClkAt400MHz () {
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
    MMIO32(Periph::rcc+0x30) = (0<<24) | (1<<16) | (0<<9) | (399<<0);
    Periph::bitSet(Periph::rcc+0x00, 24); // PLL1ON
    while (Periph::bit(Periph::rcc+0x00, 25) == 0) {} // wait for PLL1RDY
    MMIO32(Periph::rcc+0x18) = (0b100<<4) | (0b1000<<0); // APB3/2, AHB/2
    MMIO32(Periph::rcc+0x1C) = (0b100<<8) | (0b100<<4);  // APB2/2, APB1/2
    MMIO32(Periph::rcc+0x20) = (0b100<<4);               // APB4/2
    MMIO32(Periph::rcc+0x10) = (0b11<<0); // switch to PLL1
    while ((MMIO32(Periph::rcc+0x10) & 0b11000) != 0b11000) {} // wait switched
}

int fullSpeedClock () {
    constexpr uint32_t hz = 400000000;
    enableClkAt400MHz();                 // using external crystal
    enableSysTick(hz/1000);              // systick once every 1 ms
    return hz;
}

#endif
