// see [1] https://jeelabs.org/ref/STM32L0x2-RM0376.pdf

namespace Periph {
    constexpr uint32_t rtc   = 0x40002800;
    constexpr uint32_t iwdg  = 0x40003000;
    constexpr uint32_t pwr   = 0x40007000;
    constexpr uint32_t rcc   = 0x40021000;
    constexpr uint32_t exti  = 0x40021800;
    constexpr uint32_t flash = 0x40022000;
    constexpr uint32_t gpio  = 0x50000000;
};

// interrupt vector table in ram

struct VTable {
    typedef void (*Handler)();

    uint32_t* initial_sp_value;
    Handler
        reset, nmi, hard_fault, memory_manage_fault, bus_fault, usage_fault,
        dummy_x001c[4], sv_call, debug_monitor, dummy_x0034, pend_sv, systick;
    Handler
        wwdg, pvd, rtc, flash, rcc, exti0_1, exti2_3, exti4_15, ucpd1_ucpd2,
        dma1_channel1, dma1_channel2_3, dma1_channel4_7_dmamux, adc_comp,
        tim1_brk_up_trg_com, tim1_cc, tim2, tim3, tim6_dac_lptim1, tim7_lptim2,
        tim14, tim15, tim16, tim17, i2c1, i2c2, spi1, spi2, usart1, usart2,
        usart3_usart4_lpuart1, cec, aes_rng;
};

// systick and delays

constexpr static int defaultHz = 16000000;
extern void enableSysTick (uint32_t divider =defaultHz/1000);

// gpio

enum class Pinmode {
    // mode (2), typer (1), pupdr (2)
    in_analog         = 0b0011000,
    in_float          = 0b0000000,
    in_pulldown       = 0b0000010,
    in_pullup         = 0b0000001,

    out               = 0b0101000,
    out_od            = 0b0101100,
    alt_out           = 0b0110000,
    alt_out_od        = 0b0110100,

    out_2mhz          = 0b0001000,
    out_od_2mhz       = 0b0001100,
    alt_out_2mhz      = 0b0010000,
    alt_out_od_2mhz   = 0b0010100,

    out_15mhz         = 0b1001000,
    out_od_15mhz      = 0b1001100,
    alt_out_15mhz     = 0b1010000,
    alt_out_od_15mhz  = 0b1010100,

    out_30mhz         = 0b1101000,
    out_od_30mhz      = 0b1101100,
    alt_out_30mhz     = 0b1110000,
    alt_out_od_30mhz  = 0b1110100,
};

template<char port>
struct Port {
    constexpr static uint32_t base    = Periph::gpio + 0x400*(port-'A');
    constexpr static uint32_t moder   = base + 0x00;
    constexpr static uint32_t typer   = base + 0x04;
    constexpr static uint32_t ospeedr = base + 0x08;
    constexpr static uint32_t pupdr   = base + 0x0C;
    constexpr static uint32_t idr     = base + 0x10;
    constexpr static uint32_t odr     = base + 0x14;
    constexpr static uint32_t bsrr    = base + 0x18;
    constexpr static uint32_t afrl    = base + 0x20;
    constexpr static uint32_t afrh    = base + 0x24;

    static void mode (int pin, Pinmode m, int alt =0) {
        // enable GPIOx clock
        MMIO32(Periph::rcc+0x34) |= (1<<(port-'A'));

        // set the alternate mode before switching to it
        uint32_t afr = pin & 8 ? afrh : afrl;
        int shift = 4 * (pin & 7);
        MMIO32(afr) = (MMIO32(afr) & ~(0xF << shift)) | (alt << shift);

        int p2 = 2*pin;
        auto mval = static_cast<int>(m);
        MMIO32(ospeedr) = (MMIO32(ospeedr) & ~(3<<p2)) | (((mval>>5)&3) << p2);
        MMIO32(moder) = (MMIO32(moder) & ~(3<<p2)) | (((mval>>3)&3) << p2);
        MMIO32(typer) = (MMIO32(typer) & ~(1<<pin)) | (((mval>>2)&1) << pin);
        MMIO32(pupdr) = (MMIO32(pupdr) & ~(3<<p2)) | ((mval&3) << p2);
    }

    static void modeMap (uint16_t pins, Pinmode m, int alt =0) {
        for (int i = 0; i < 16; ++i) {
            if (pins & 1)
                mode(i, m, alt);
            pins >>= 1;
        }
    }
};

template<char port,int pin>
struct Pin {
    typedef Port<port> gpio;
    constexpr static uint16_t mask = 1U << pin;
    constexpr static int id = 16 * (port-'A') + pin;

    static void mode (Pinmode m, int alt =0) {
        gpio::mode(pin, m, alt);
    }

    static int read () {
        return mask & MMIO32(gpio::idr) ? 1 : 0;
    }

    static void write (int v) {
        MMIO32(gpio::bsrr) = v ? mask : mask << 16;
    }

    // shorthand
    operator int () const { return read(); }
    void operator= (int v) const { write(v); }

    static void toggle () {
        // both versions below are non-atomic, they access and set in two steps
        // this is smaller and faster (1.6 vs 1.2 MHz on F103 @ 72 MHz):
        // MMIO32(gpio::odr) ^= mask;
        // but this code is safer, because it can't interfere with nearby pins:
        MMIO32(gpio::bsrr) = mask & MMIO32(gpio::odr) ? mask << 16 : mask;
    }
};

// u(s)art

template< typename TX, typename RX >
struct UartDev {
    // TODO does not recognise alternate TX pins
    constexpr static int uidx = TX::id ==  9 ? 0 :  // PA9, USART1
                                TX::id ==  2 ? 1 :  // PA2, USART2
                                TX::id == 22 ? 1 :  // PB6, USART2
                                               0;   // else USART1
    constexpr static uint32_t base = uidx == 0 ? 0x40013800 :
                                                 0x40004000 + 0x400 * uidx;
    constexpr static uint32_t cr1 = base + 0x00;
    //constexpr static uint32_t cr2 = base + 0x04;
    //constexpr static uint32_t cr3 = base + 0x08;
    constexpr static uint32_t brr = base + 0x0C;
    constexpr static uint32_t isr = base + 0x1C;
    constexpr static uint32_t icr = base + 0x20;
    constexpr static uint32_t rdr = base + 0x24;
    constexpr static uint32_t tdr = base + 0x28;

    static void init () {
        TX::mode(Pinmode::alt_out, 4);
        RX::mode(Pinmode::alt_out, 4);

        if (uidx == 0)
            MMIO32(Periph::rcc + 0x40) |= 1 << 14; // enable USART1 clock
        else
            MMIO32(Periph::rcc + 0x3C) |= 1 << (16+uidx); // USART 2..5

        MMIO32(brr) = defaultHz / 115200;  // 115200 baud @ 2.1 MHz
        MMIO32(cr1) = (1<<3) | (1<<2) | (1<<0);  // TE, RE, UE
    }

    static void baud (uint32_t baud, uint32_t hz =defaultHz) {
        MMIO32(cr1) &= ~(1<<0);              // disable
        MMIO32(brr) = (hz + baud/2) / baud;  // change while disabled
        MMIO32(cr1) |= (1<<0);                 // enable
    }

    static bool writable () {
        return (MMIO32(isr) & (1<<7)) != 0;  // TXE
    }

    static void putc (int c) {
        while (!writable())
            ;
        MMIO32(tdr) = (uint8_t) c;
    }

    static bool readable () {
        return (MMIO32(isr) & 0x24) != 0;  // RXNE or ORE
    }

    static int getc () {
        while (!readable())
            ;
        int c = MMIO32(rdr);
        MMIO32(icr) = 0xA; // clear ORE and FE, reading RDR is not enough
        return c;
    }
};

// interrupt-enabled uart, sits of top of polled uart

template< typename TX, typename RX, int N =50 >
struct UartBufDev : UartDev<TX,RX> {
    typedef UartDev<TX,RX> base;

    static void init () {
        UartDev<TX,RX>::init();

        auto handler = []() {
            if (base::readable()) {
                int c = base::getc();
                if (recv.free())
                    recv.put(c);
                // else discard the input
            }
            if (base::writable()) {
                if (xmit.avail() > 0)
                    base::putc(xmit.get());
                else
                    MMIO32(base::cr1) &= ~(1<<7);  // disable TXEIE
            }
        };

        switch (base::uidx) {
            case 0: VTableRam().usart1 = handler; break;
            case 1: VTableRam().usart2 = handler; break;
        }

        // nvic interrupt numbers are 27 and 28, respectively
        constexpr uint32_t nvic_en0r = 0xE000E100;
        constexpr int irq = 27 + base::uidx;
        MMIO32(nvic_en0r) = 1 << irq;  // enable USART interrupt

        MMIO32(base::cr1) |= (1<<5);  // enable RXNEIE
    }

    static bool writable () {
        return xmit.free();
    }

    static void putc (int c) {
        while (!writable())
            ;
        xmit.put(c);
        MMIO32(base::cr1) |= (1<<7);  // enable TXEIE
    }

    static bool readable () {
        return recv.avail() > 0;
    }

    static int getc () {
        while (!readable())
            ;
        return recv.get();
    }

    static RingBuffer<N> recv;
    static RingBuffer<N> xmit;
};

template< typename TX, typename RX, int N >
RingBuffer<N> UartBufDev<TX,RX,N>::recv;

template< typename TX, typename RX, int N >
RingBuffer<N> UartBufDev<TX,RX,N>::xmit;

// system clock

extern void enableClkHSI16 ();
extern void enableClkPll ();
extern int fullSpeedClock (bool pll =true);

// low-power modes

extern void powerDown (bool standby =true);

// hardware spi support

template< typename MO, typename MI, typename CK, typename SS, int CP =0 >
struct SpiHw {  // [1] pp.742
    constexpr static int sidx = MO::id ==  7 ? 0 :  // PA7,  SPI1
                                MO::id == 21 ? 0 :  // PB5,  SPI1, remapped
                                MO::id == 31 ? 1 :  // PB15, SPI2
                            // oops, this is not possible, also remapped SPI1!
                            //  MO::id == 21 ? 2 :  // PB5,  SPI3
                            //? MO::id == 44 ? 2 :  // PC12, SPI3, remapped
                                               0;   // else  SPI1
    constexpr static uint32_t base = sidx == 0 ? 0x40013000 :
                                                 0x40003400 + 0x400*sidx;
    constexpr static uint32_t cr1 = base + 0x00;
    constexpr static uint32_t cr2 = base + 0x04;
    constexpr static uint32_t sr  = base + 0x08;
    constexpr static uint32_t dr  = base + 0x0C;

    static void init (uint32_t div =1) {
        SS::mode(Pinmode::out); disable();
        CK::mode(Pinmode::alt_out);
        MI::mode(Pinmode::alt_out);
        MO::mode(Pinmode::alt_out);

        if (sidx == 0)
            MMIO32(Periph::rcc+0x34) |= 1<<12;  // SPI1
        else
            MMIO32(Periph::rcc+0x38) |= 1<<(sidx+13);  // SPI 2..3

        //(void) MMIO32(sr);  // may be needed to avoid hang in some cases?
        MMIO32(cr2) |= 1<<2;  // SSOE
        // SPE, BR=dif, MSTR, CPOL (for div=1 @ 32 MHz: clk/4, i.e. 8 MHz)
        MMIO32(cr1) = (1<<6) | (div<<3) | (1<<2) | (CP<<1);  // [1] p.742
    }

    static void enable () { SS::write(0); }
    static void disable () { SS::write(1); }

    static uint8_t transfer (uint8_t v) {
        MMIO32(dr) = v;
        while ((MMIO32(sr) & (1<<0)) == 0) {}
        return MMIO32(dr);
    }
};

// independent watchdog

struct Iwdg {  // [1] pp.495
    constexpr static uint32_t kr  = Periph::iwdg + 0x00;
    constexpr static uint32_t pr  = Periph::iwdg + 0x04;
    constexpr static uint32_t rlr = Periph::iwdg + 0x08;
    constexpr static uint32_t sr  = Periph::iwdg + 0x0C;

    Iwdg (int rate =7) {
        while (MMIO32(sr) & (1<<0)) {}  // wait until !PVU
        MMIO32(kr) = 0x5555;   // unlock PR
        MMIO32(pr) = rate;     // max timeout, 0 = 400ms, 7 = 26s
        MMIO32(kr) = 0xCCCC;   // start watchdog
    }

    static void kick () {
        MMIO32(kr) = 0xAAAA;  // reset the watchdog timout
    }

    static void reload (int n) {
        while (MMIO32(sr) & (1<<1)) {}  // wait until !RVU
        MMIO32(kr) = 0x5555;   // unlock PR
        MMIO32(rlr) = n;
        kick();
    }
};

// analog input

struct ADC {
    constexpr static uint32_t base   = 0x40012400;
    constexpr static uint32_t isr    = base + 0x000;
    constexpr static uint32_t ier    = base + 0x004;
    constexpr static uint32_t cr     = base + 0x008;
    constexpr static uint32_t cfgr1  = base + 0x00C;
    constexpr static uint32_t cfgr2  = base + 0x010;
    constexpr static uint32_t smpr   = base + 0x014;
    constexpr static uint32_t tr     = base + 0x020;
    constexpr static uint32_t chselr = base + 0x028;
    constexpr static uint32_t dr     = base + 0x040;
    constexpr static uint32_t ccr    = base + 0x308;

    static void init () {
        MMIO32(Periph::rcc+0x34) |= (1<<9);  // enable ADC
        MMIO32(cr) = (1<<31);  // ADCAL
        while (MMIO32(cr) & (1<<31)) {}  // wait until calibration completed
        MMIO32(cr) |= (1<<0);  // ADEN
    }

    // read analog, given a pin (which is also set to analog input mode)
    template< typename pin >
    static uint16_t read (pin& p) {
        pin::mode(Pinmode::in_analog);
        constexpr int off = pin::id < 16 ? 0 :   // A0..A7 => 0..7
                            pin::id < 32 ? -8 :  // B0..B1 => 8..9
                                           -22;  // C0..C5 => 10..15
        return read(pin::id + off);
    }

    // read direct channel number (also: 17 = vref, 18 = temp)
    static uint16_t read (uint8_t chan) {
        MMIO32(chselr) = (1<<chan);
        MMIO32(cr) |= (1<<2);  // start conversion
        while (MMIO32(cr) & (1<<2)) {}  // EOC wait until done
        return MMIO32(dr);
    }

    static void window (int low, int high) {
        MMIO32(tr) = (high<<16) | low;
    }

    static void watch (void (*f)() =0) {
        uint32_t cfg = MMIO32(cfgr1) & ~0x78C00000;
        if (f != 0) {
            VTableRam().adc_comp = f;
            cfg |= (1<<23); // AWDEN

            constexpr uint32_t nvic_en0r = 0xE000E100;
            MMIO32(nvic_en0r) = (1<<12);  // enable ADC_COMP interrupt

            MMIO32(ier) |= (1<<7); // AWDIE
        } else
            MMIO32(ier) &= ~(1<<7); // ~AWDIE

        MMIO32(cfgr1) = cfg;
    }
};

// analog output

struct DAC {
    constexpr static uint32_t base    = 0x40007400;
    constexpr static uint32_t cr      = base + 0x00;
    constexpr static uint32_t dhr12r1 = base + 0x08;

    static void init () {
        MMIO32(Periph::rcc+0x38) |= (1<<29);  // enable DAC
        MMIO32(cr) = (1<<0);  // EN1
    }

    static void write (uint32_t val) {
        MMIO32(dhr12r1) = val;
    }

    static void dmaWave (const uint16_t* ptr, uint16_t num, uint16_t div) {
        MMIO32(cr) |= (1<<12) | (1<<2);  // DMAEN1, TEN1

        { // DMA setup
            const uint32_t dma    = 0x40020000;
            const uint32_t ccr2   = dma + 0x1C;
            const uint32_t cndtr2 = dma + 0x20;
            const uint32_t cpar2  = dma + 0x24;
            const uint32_t cmar2  = dma + 0x28;
            const uint32_t cselr  = dma + 0xA8;

            MMIO32(Periph::rcc+0x30) |= (1<<0);  // DMAEN
            MMIO32(cselr) |= (9<<4);  // DAC on DMA chan 2
            MMIO32(cndtr2) = num;
            MMIO32(cpar2) = dhr12r1;
            MMIO32(cmar2) = (uint32_t) ptr;

            // msize 16b, psize 16b, minc, circ, m2p
            MMIO32(ccr2) = (1<<10) | (1<<8) | (1<<7) | (1<<5) | (1<<4);
            MMIO32(ccr2) |= (1<<0);  // EN
        }

        { // TIM6 setup
            const uint32_t tim6 = 0x40001000;
            const uint32_t cr1  = tim6 + 0x00;
            const uint32_t cr2  = tim6 + 0x04;
            const uint32_t dier = tim6 + 0x0C;
            const uint32_t arr  = tim6 + 0x2C;

            MMIO32(Periph::rcc+0x38) |= (1<<4);  // TIM6EN
            MMIO32(arr) = div-1;
            MMIO32(dier) = (1<<8); // UDE
            MMIO32(cr2) = (2<<4); // MMS update
            MMIO32(cr1) |= (1<<0); // CEN
        }
    }
};

// real-time clock

struct RTC {  // [1] pp.486
    constexpr static uint32_t tr   = Periph::rtc + 0x00;
    constexpr static uint32_t dr   = Periph::rtc + 0x04;
    constexpr static uint32_t cr   = Periph::rtc + 0x08;
    constexpr static uint32_t isr  = Periph::rtc + 0x0C;
    constexpr static uint32_t prer = Periph::rtc + 0x10;
    constexpr static uint32_t wutr = Periph::rtc + 0x14;
    constexpr static uint32_t wpr  = Periph::rtc + 0x24;
    constexpr static uint32_t bkpr = Periph::rtc + 0x50;

    struct DateTime {
        uint32_t yr :6;  // 00..63
        uint32_t mo :4;  // 1..12
        uint32_t dy :5;  // 1..31
        uint32_t hh :5;  // 0..23
        uint32_t mm :6;  // 0..59
        uint32_t ss :6;  // 0..59
    };

    RTC () {
        MMIO32(Periph::rcc+0x38) |= (1<<28);  // enable PWREN
        MMIO32(Periph::pwr) |= (1<<8);  // set DBP [1] p.481
    }

    void init (bool crystal =false) {
        const uint32_t csr = Periph::rcc + 0x50;

        if (crystal) {
            MMIO32(csr) |= (1<<8);                  // LSEON
            while ((MMIO32(csr) & (1<<9)) == 0) {}  // wait for LSERDY
            MMIO32(csr) = (MMIO32(csr) & ~(3<<16)) | (1<<16); // RTSEL = LSE
        } else {
            MMIO32(csr) |= (1<<0);                  // LSION
            while ((MMIO32(csr) & (1<<1)) == 0) {}  // wait for LSIRDY
            MMIO32(csr) = (MMIO32(csr) & ~(3<<16)) | (2<<16); // RTSEL = LSI
        }

        MMIO32(csr) |= (1<<18); // RTCEN

        // the LSE clock runs at 32.768 kHz
        unlockInit();
        MMIO32(prer) = crystal ? 255 : 296; // prescale for 32 or 37 kHz
        MMIO32(prer) |= (127<<16); // needs to be written in two steps!
        lockInit();
    }

    DateTime get () {
        unlock();
        MMIO32(isr) &= ~(1<<5);              // clear RSF
        while ((MMIO32(isr) & (1<<5)) == 0) {}   // wait for RSF
        lock();

        // shadow registers are now valid and won't change while being read
        uint32_t tod = MMIO32(tr);
        uint32_t doy = MMIO32(dr);

        DateTime dt;
        dt.ss = (tod & 0xF) + 10 * ((tod>>4) & 0x7);
        dt.mm = ((tod>>8) & 0xF) + 10 * ((tod>>12) & 0x7);
        dt.hh = ((tod>>16) & 0xF) + 10 * ((tod>>20) & 0x3);
        dt.dy = (doy & 0xF) + 10 * ((doy>>4) & 0x3);
        dt.mo = ((doy>>8) & 0xF) + 10 * ((doy>>12) & 0x1);
        // works until end 2063, will fail (i.e. roll over) in 2064 !
        dt.yr = ((doy>>16) & 0xF) + 10 * ((doy>>20) & 0x7);
        return dt;
    }

    void set (DateTime dt) {
        unlockInit();
        MMIO32(tr) = (dt.ss + 6 * (dt.ss/10)) |
                    ((dt.mm + 6 * (dt.mm/10)) << 8) |
                    ((dt.hh + 6 * (dt.hh/10)) << 16);
        MMIO32(dr) = (dt.dy + 6 * (dt.dy/10)) |
                    ((dt.mo + 6 * (dt.mo/10)) << 8) |
                    ((dt.yr + 6 * (dt.yr/10)) << 16);
        lockInit();
    }

    void wakeup (int count) {
        unlock();
        MMIO32(cr) &= ~(1<<10); // ~WUTE
        if (count > 0) {
            while ((MMIO32(isr) & (1<<2)) == 0) {} // wait for WUTWF
            MMIO32(wutr) = count;
            MMIO32(cr) = (1<<14) | (1<<10); // WUTIE, WUTE
        }
        lock();

        // make sure the RTC events will wakeup while in WFE
        MMIO32(Periph::exti+0x04) |= (1<<20); // EMR, unmask event 20
        MMIO32(Periph::exti+0x08) |= (1<<20); // RTSR, rising edge event 20
    }

    void arm () {
        MMIO32(isr) &= ~(1<<10); // clear WUTF
    }

    // access to the backup registers

    uint32_t getData (int reg) {
        return MMIO32(bkpr + 4 * reg);  // regs 0..4
    }

    void setData (int reg, uint32_t val) {
        MMIO32(bkpr + 4 * reg) = val;  // regs 0..4
    }

    void unlock () {
        MMIO32(wpr) = 0xCA;  // disable write protection
        MMIO32(wpr) = 0x53;
    }

    void lock () {
        MMIO32(wpr) = 0xFF;  // re-enable write protection
    }

    void unlockInit () {
        unlock();
        MMIO32(isr) |= (1<<7); // set INIT
        while ((MMIO32(isr) & (1<<6)) == 0) {}  // wait for INITF
    }

    void lockInit () {
        MMIO32(isr) &= ~(1<<7); // clear INIT
        lock();
    }
};
