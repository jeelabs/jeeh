#if STM32L4

#include <jee.h>

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
