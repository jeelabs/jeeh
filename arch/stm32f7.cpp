#if STM32F7

#include <jee.h>

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

#endif
