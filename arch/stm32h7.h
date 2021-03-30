// Hardware access for STM32H7xx family microcontrollers
// see [1] https://jeelabs.org/ref/STM32H7xx-RM0433.pdf

#ifndef XTAL
#define XTAL 8  // the external crystal is usually 8 MHz
#endif

namespace Periph {
    constexpr uint32_t flash = 0x52002000;
    constexpr uint32_t fmc   = 0x52004000;
    constexpr uint32_t rtc   = 0x58004000;
    constexpr uint32_t iwdg  = 0x58004800;
    constexpr uint32_t gpio  = 0x58020000;
    constexpr uint32_t rcc   = 0x58024400;
    constexpr uint32_t pwr   = 0x58024800;
    constexpr uint32_t dwt   = 0xE0001000;

    inline uint32_t bit (uint32_t a, int b) { return (MMIO32(a) >> b) & 1; }
    inline void bitSet (uint32_t a, int b) { MMIO32(a) |= (1<<b); }
    inline void bitClear (uint32_t a, int b) { MMIO32(a) &= ~(1<<b); }
}

// interrupt vector table in ram

struct VTable {
    typedef void (*Handler)();

    uint32_t* initial_sp_value;
    Handler
        reset, nmi, hard_fault, memory_manage_fault, bus_fault, usage_fault,
        dummy_x001c[4], sv_call, debug_monitor, dummy_x0034, pend_sv, systick;
    Handler
        wwdg, pvd, tamp_stamp, rtc_wkup, flash, rcc, exti0, exti1, exti2,
        exti3, exti4, dma1_stream0, dma1_stream1, dma1_stream2, dma1_stream3,
        dma1_stream4, dma1_stream5, dma1_stream6, adc, can1_tx, can1_rx0,
        can1_rx1, can1_sce, exti9_5, tim1_brk_tim9, tim1_up_tim10,
        tim1_trg_com_tim11, tim1_cc, tim2, tim3, tim4, i2c1_ev, i2c1_er,
        i2c2_ev, i2c2_er, spi1, spi2, usart1, usart2, usart3, exti15_10,
        rtc_alarm, usb_fs_wkup, tim8_brk_tim12, tim8_up_tim13,
        tim8_trg_com_tim14, tim8_cc, dma1_stream7, fsmc, sdmmc1, tim5, spi3,
        uart4, uart5, tim6_dac, tim7, dma2_stream0, dma2_stream1, dma2_stream2,
        dma2_stream3, dma2_stream4, eth, eth_wkup, can2_tx, can2_rx0, can2_rx1,
        can2_sce, otg_fs, dma2_stream5, dma2_stream6, dma2_stream7, usart6,
        i2c3_ev, i2c3_er, otg_hs_ep1_out, otg_hs_ep1_in, otg_hs_wkup, otg_hs,
        dcmi, cryp, hash_rng, fpu, uart7, uart8, spi4, spi5, spi6, sai1,
        lcd_tft, lcd_tft_err, dma2d, sai2, quadspi, i2c4_ev, i2c4_er, spdifrx,
        usb2_out, usb2_in, usb2_wkup, usb2; // etc ... TODO
};

// systick and delays

constexpr static int defaultHz = 64000000;
extern void enableSysTick (uint32_t divider =defaultHz/1000) __attribute__((weak));

// gpio

enum class Pinmode {
    // speedr (2), moder (2), typer (1), pupdr (2)
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

    out_50mhz         = 0b1001000,
    out_od_50mhz      = 0b1001100,
    alt_out_50mhz     = 0b1010000,
    alt_out_od_50mhz  = 0b1010100,

    out_100mhz        = 0b1101000,
    out_od_100mhz     = 0b1101100,
    alt_out_100mhz    = 0b1110000,
    alt_out_od_100mhz = 0b1110100,
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
        Periph::bitSet(Periph::rcc+0xE0, port-'A');

        // set the alternate mode before switching to it
        uint32_t afr = pin & 8 ? afrh : afrl;
        int shift = 4 * (pin & 7);
        MMIO32(afr) = (MMIO32(afr) & ~(0xF << shift)) | (alt << shift);

        int p2 = 2*pin;
        auto mval = static_cast<int>(m);
        MMIO32(moder) = (MMIO32(moder) & ~(3<<p2)) | (((mval>>3)&3) << p2);
        MMIO32(typer) = (MMIO32(typer) & ~(1<<pin)) | (((mval>>2)&1) << pin);
        MMIO32(pupdr) = (MMIO32(pupdr) & ~(3<<p2)) | ((mval&3) << p2);
        MMIO32(ospeedr) = (MMIO32(ospeedr) & ~(3<<p2)) | (((mval>>5)&3) << p2);
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
    constexpr static int uidx = TX::id ==  2 ? 1 :  // PA2, USART2
                                TX::id ==  9 ? 0 :  // PA9, USART1
                                TX::id == 22 ? 0 :  // PB6, USART1
                                TX::id == 26 ? 2 :  // PB10, USART3
                                TX::id == 42 ? 2 :  // PC10, USART3
                                TX::id == 53 ? 1 :  // PD5, USART2
                                TX::id == 56 ? 2 :  // PD8, USART3
                                // TODO more possible, using alt mode 8 iso 7
                                               0;   // else USART1
    constexpr static uint32_t base = uidx == 0 ? 0x40011000 : // USART1
                                     uidx == 5 ? 0x40011400 : // USART6
                                                 0x40004000 + 0x400*uidx;
    constexpr static uint32_t cr1 = base + 0x00;
    constexpr static uint32_t cr3 = base + 0x08;
    constexpr static uint32_t brr = base + 0x0C;
    constexpr static uint32_t isr = base + 0x1C;
    constexpr static uint32_t icr = base + 0x20;
    constexpr static uint32_t rdr = base + 0x24;
    constexpr static uint32_t tdr = base + 0x28;

    static void init () {
        tx.mode(Pinmode::alt_out, 7);
        rx.mode(Pinmode::alt_out, 7);

        if (uidx == 0)
            Periph::bitSet(Periph::rcc+0xF0, 4); // enable USART1 clock
        else
            Periph::bitSet(Periph::rcc+0xE8, 16+uidx); // U(S)ART 2..5

        baud(115200);
        MMIO32(cr1) = (1<<3) | (1<<2) | (1<<0);  // TE, RE, UE
    }

    static void baud (uint32_t baud, uint32_t hz =defaultHz) {
        MMIO32(cr1) &= ~(1<<0);              // disable
        MMIO32(brr) = (hz + baud/2) / baud;  // change while disabled
        MMIO32(cr1) |= 1<<0;                 // enable
    }

    static bool writable () {
        return (MMIO32(isr) & (1<<7)) != 0;  // TXE
    }

    static void putc (int c) {
        while (!writable()) {}
        MMIO32(tdr) = (uint8_t) c;
    }

    static bool readable () {
        return (MMIO32(isr) & 0x2F) != 0;  // RXNE, ORE, NF, FE, PE
    }

    static int getc () {
        while (!readable()) {}
        MMIO32(icr) = 0x0F; // also clear error flags, RDR read is not enough
        return MMIO32(rdr);
    }

    static TX tx;
    static RX rx;
};

template< typename TX, typename RX >
TX UartDev<TX,RX>::tx;

template< typename TX, typename RX >
RX UartDev<TX,RX>::rx;

// interrupt-enabled uart, sits on top of polled uart

template< typename TX, typename RX, int N =50 >
struct UartBufDev : UartDev<TX,RX> {
    typedef UartDev<TX,RX> base;

    // handler is a function which returns a reference to a function pointer ...
    static void (*& handler ())() {
        switch (base::uidx) {
            default:
            case 0: return VTableRam().usart1;
            case 1: return VTableRam().usart2;
            case 2: return VTableRam().usart3;
            case 3: return VTableRam().uart4;
            case 4: return VTableRam().uart5;
            case 5: return VTableRam().usart6;
            case 6: return VTableRam().uart7;
            case 7: return VTableRam().uart8;
        }
    }

    static void init () {
        UartDev<TX,RX>::init();

        handler() = []() {
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
                    Periph::bitClear(base::cr1, 7);  // disable TXEIE
            }
        };

        // nvic interrupt numbers are 37, 38, 39, 52, and 53, respectively
        constexpr uint32_t nvic_en1r = 0xE000E104;
        constexpr int irq = (base::uidx < 3 ? 37 : 49) + base::uidx;
        MMIO32(nvic_en1r) = 1 << (irq-32);  // enable USART interrupt

        Periph::bitSet(base::cr1, 5);  // enable RXNEIE
        Periph::bitSet(base::cr3, 0);  // enable EIE
    }

    static bool writable () {
        return xmit.free();
    }

    static void putc (int c) {
        while (!writable()) {}
        xmit.put(c);
        Periph::bitSet(base::cr1, 7);  // enable TXEIE
    }

    static bool readable () {
        return recv.avail() > 0;
    }

    static int getc () {
        while (!readable()) {}
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

extern void enableClkPll (int freq);
extern int fullSpeedClock ();

// low-power modes

extern void powerDown (bool standby =true);

// can bus(es)

template< int N >
struct CanDev {
    constexpr static uint32_t base = N == 0 ? 0x40006400 : 0x40006800;

    constexpr static uint32_t mcr  = base + 0x000;
    constexpr static uint32_t msr  = base + 0x004;
    constexpr static uint32_t tsr  = base + 0x008;
    constexpr static uint32_t rfr  = base + 0x00C;
    constexpr static uint32_t btr  = base + 0x01C;
    constexpr static uint32_t tir  = base + 0x180;
    constexpr static uint32_t tdtr = base + 0x184;
    constexpr static uint32_t tdlr = base + 0x188;
    constexpr static uint32_t tdhr = base + 0x18C;
    constexpr static uint32_t rir  = base + 0x1B0;
    constexpr static uint32_t rdtr = base + 0x1B4;
    constexpr static uint32_t rdlr = base + 0x1B8;
    constexpr static uint32_t rdhr = base + 0x1BC;
    constexpr static uint32_t fmr  = base + 0x200;
    constexpr static uint32_t fsr  = base + 0x20C;
    constexpr static uint32_t far  = base + 0x21C;
    constexpr static uint32_t fr1  = base + 0x240;
    constexpr static uint32_t fr2  = base + 0x244;

    static void init (bool singleWire =false) {
        auto swMode = singleWire ? Pinmode::alt_out_od : Pinmode::alt_out;
        if (N == 0) {
            // alt mode CAN1:    5432109876543210
            Port<'B'>::modeMap(0b0000001100000000, swMode, 9);
            Periph::bitSet(Periph::rcc+0x40, 25);  // enable CAN1
        } else {
            // alt mode CAN2:    5432109876543210
            Port<'B'>::modeMap(0b0000000001100000, swMode, 9);
            Periph::bitSet(Periph::rcc+0x40, 26);  // enable CAN2
        }

        Periph::bitClear(mcr, 1); // exit sleep
        MMIO32(mcr) |= (1<<6) | (1<<0); // set ABOM, init req
        while (Periph::bit(msr, 0) == 0) {}
        MMIO32(btr) = (7<<20) | (5<<16) | (2<<0); // 1 MBps
        Periph::bitClear(mcr, 0); // init leave
        while (Periph::bit(msr, 0)) {}
        Periph::bitClear(fmr, 0); // ~FINIT
    }

    static void filterInit (int num, int id =0, int mask =0) {
        Periph::bitClear(far, num); // ~FACT
        Periph::bitSet(fsr, num); // FSC 32b
        MMIO32(fr1 + 8 * num) = id;
        MMIO32(fr2 + 8 * num) = mask;
        Periph::bitSet(far, num); // FACT
    }

    static bool transmit (int id, const void* ptr, int len) {
        if (Periph::bit(tsr, 26)) { // TME0
            MMIO32(tir) = (id<<21);
            MMIO32(tdtr) = (len<<0);
            // this assumes that misaligned word access works
            MMIO32(tdlr) = ((const uint32_t*) ptr)[0];
            MMIO32(tdhr) = ((const uint32_t*) ptr)[1];

            Periph::bitSet(tir, 0); // TXRQ
            return true;
        }
        return false;
    }

    static int receive (int* id, void* ptr) {
        int len = -1;
        if (MMIO32(rfr) & (3<<0)) { // FMP
            *id = MMIO32(rir) >> 21;
            len = MMIO32(rdtr) & 0x0F;
            ((uint32_t*) ptr)[0] = MMIO32(rdlr);
            ((uint32_t*) ptr)[1] = MMIO32(rdhr);
            Periph::bitSet(rfr, 5); // RFOM
        }
        return len;
    }
};

// real-time clock

struct RTC {  // [1] pp.486
    constexpr static uint32_t tr   = Periph::rtc + 0x00;
    constexpr static uint32_t dr   = Periph::rtc + 0x04;
    constexpr static uint32_t cr   = Periph::rtc + 0x08;
    constexpr static uint32_t isr  = Periph::rtc + 0x0C;
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
        Periph::bitSet(Periph::rcc+0x40, 28);  // enable PWREN
        Periph::bitSet(Periph::pwr, 8);  // set DBP [1] p.481
    }

    void init () {
        const uint32_t bdcr = Periph::rcc + 0x70;
        Periph::bitSet(bdcr, 0);             // LSEON backup domain
        while (Periph::bit(bdcr, 1) == 0) {}  // wait for LSERDY
        Periph::bitSet(bdcr, 8);             // RTSEL = LSE
        Periph::bitSet(bdcr, 15);            // RTCEN
    }

    DateTime get () {
        MMIO32(wpr) = 0xCA;  // disable write protection, [1] p.803
        MMIO32(wpr) = 0x53;

        Periph::bitClear(isr, 5);              // clear RSF
        while (Periph::bit(isr, 5) == 0) {}   // wait for RSF

        MMIO32(wpr) = 0xFF;  // re-enable write protection

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
        MMIO32(wpr) = 0xCA;  // disable write protection, [1] p.803
        MMIO32(wpr) = 0x53;

        Periph::bitSet(isr, 7);             // set INIT
        while (Periph::bit(isr, 6) == 0) {}  // wait for INITF
        MMIO32(tr) = (dt.ss + 6 * (dt.ss/10)) |
                    ((dt.mm + 6 * (dt.mm/10)) << 8) |
                    ((dt.hh + 6 * (dt.hh/10)) << 16);
        MMIO32(dr) = (dt.dy + 6 * (dt.dy/10)) |
                    ((dt.mo + 6 * (dt.mo/10)) << 8) |
                    ((dt.yr + 6 * (dt.yr/10)) << 16);
        Periph::bitClear(isr, 7);             // clear INIT

        MMIO32(wpr) = 0xFF;  // re-enable write protection
    }

    // access to the backup registers

    uint32_t getData (int reg) {
        return MMIO32(bkpr + 4 * reg);  // regs 0..18
    }

    void setData (int reg, uint32_t val) {
        MMIO32(bkpr + 4 * reg) = val;  // regs 0..18
    }
};

// flash memory writing and erasing

struct Flash {
    constexpr static uint32_t keyr = Periph::flash + 0x04;
    constexpr static uint32_t sr   = Periph::flash + 0x0C;
    constexpr static uint32_t cr   = Periph::flash + 0x10;

    static void write8 (void const* addr, uint8_t val) {
        if (*(uint8_t*) addr != 0xFF)
            return;
        unlock();
        MMIO32(cr) = (0<<8) | (1<<0); // PSIZE, PG
        MMIO8((uint32_t) addr | 0x08000000) = val;
        finish();
    }

    static void write16 (void const* addr, uint16_t val) {
        if (*(uint16_t*) addr != 0xFFFF)
            return;
        unlock();
        MMIO32(cr) = (1<<8) | (1<<0); // PSIZE, PG
        MMIO16((uint32_t) addr | 0x08000000) = val;
        finish();
    }

    static void write32 (void const* addr, uint32_t val) {
        if (*(uint32_t*) addr != 0xFFFFFFFF)
            return;
        unlock();
        MMIO32(cr) = (2<<8) | (1<<0); // PSIZE, PG
        MMIO32((uint32_t) addr | 0x08000000) = val;
        finish();
    }

    static void write32buf (void const* a, uint32_t const* ptr, int len) {
        if (*(uint32_t*) a != 0xFFFFFFFF)
            return;
        unlock();
        MMIO32(cr) = (2<<8) | (1<<0); // PSIZE, PG
        for (int i = 0; i < len; ++i)
            MMIO32(((uint32_t) a + 4*i) | 0x08000000) = ptr[i];
        finish();
    }

    static void erasePage (void const* addr) {
        uint32_t a = (uint32_t) addr & 0x07FFFFFF;
        // sectors are 16/16/16/16/64/128... KB
        int sector = a < 0x10000 ? (a >> 14) :
                     a < 0x20000 ? (a >> 16) + 3 :
                                   (a >> 17) + 4;
        unlock();
        MMIO32(cr) = (1<<16) | (2<<8) | (sector<<3) | (1<<1); // STRT PG SNB SER
        finish();
    }

    static void unlock () {
        MMIO32(keyr) = 0x45670123;
        MMIO32(keyr) = 0xCDEF89AB;
    }

    static void finish () {
        while (Periph::bit(sr, 16)) {}
        MMIO32(cr) = (1<<31); // LOCK
    }
};

// timers and PWM

template< int N >
struct Timer {
    constexpr static int tidx = N ==  1 ? 64 :  // TIM1,  APB2
                                N ==  2 ?  0 :  // TIM2,  APB1
                                N ==  3 ?  1 :  // TIM3,  APB1
                                N ==  4 ?  2 :  // TIM4,  APB1
                                N ==  5 ?  3 :  // TIM5,  APB1
                                N ==  6 ?  4 :  // TIM6,  APB1
                                N ==  7 ?  5 :  // TIM7,  APB1
                                N ==  8 ? 65 :  // TIM8,  APB2
                                N ==  9 ? 70 :  // TIM9,  APB2
                                N == 10 ? 71 :  // TIM10, APB2
                                N == 11 ? 72 :  // TIM11, APB2
                                N == 12 ?  6 :  // TIM12, APB1
                                N == 13 ?  7 :  // TIM13, APB1
                                N == 14 ?  8 :  // TIM14, APB1
                                          64;   // else TIM1

    constexpr static uint32_t base  = 0x40000000 + 0x400*tidx;
    constexpr static uint32_t cr1   = base + 0x00;
    constexpr static uint32_t ccmr1 = base + 0x18;
    constexpr static uint32_t ccer  = base + 0x20;
    constexpr static uint32_t psc   = base + 0x28;
    constexpr static uint32_t arr   = base + 0x2C;
    constexpr static uint32_t ccr1  = base + 0x34;

    static void init (uint32_t limit, uint32_t scale =0) {
        if (tidx < 64)
            Periph::bitSet(Periph::rcc+0x40, tidx);
        else
            Periph::bitSet(Periph::rcc+0x44, tidx-64);
        MMIO16(psc) = scale;
        MMIO32(arr) = limit-1;
        Periph::bitSet(cr1, 0); // CEN
    }

    // TODO TIM1 (and TIM8?) don't seem to work with PWM
    static void pwm (uint32_t match) {
        MMIO16(ccmr1) = 0x60; // PWM mode
        MMIO32(ccr1) = match;
        Periph::bitSet(ccer, 0); // CC1E
    }
};

// cycle counts, see https://stackoverflow.com/questions/11530593/
// and https://community.arm.com/developer/tools-software/tools/f/
//      keil-forum/41930/can-t-write-into-dwt-registers
// and https://dzone.com/articles/cycle-counting-on-an-arm-cortex-m-with-dwt

struct DWT {
    constexpr static uint32_t ctrl   = Periph::dwt + 0x000;
    constexpr static uint32_t cyccnt = Periph::dwt + 0x004;
    constexpr static uint32_t lar    = Periph::dwt + 0xFB0;
    constexpr static uint32_t scb_demcr = 0xE000EDFC;

    static void init () {
        MMIO32(lar) = 0xC5ACCE55;
        MMIO32(scb_demcr) |= (1<<24); // set TRCENA in DEMCR
    }

    static void start () { MMIO32(cyccnt) = 0; MMIO32(ctrl) |= 1<<0; }
    static void stop () { MMIO32(ctrl) &= ~(1<<0); }
    static uint32_t count () { return MMIO32(cyccnt); }
};

// independent watchdog

struct Iwdg {  // [1] pp.495
    constexpr static uint32_t kr  = Periph::iwdg + 0x00;
    constexpr static uint32_t pr  = Periph::iwdg + 0x04;
    constexpr static uint32_t rlr = Periph::iwdg + 0x08;
    constexpr static uint32_t sr  = Periph::iwdg + 0x0C;

    Iwdg (int rate =7) {
        while (Periph::bit(sr, 0)) {}  // wait until !PVU
        MMIO32(kr) = 0x5555;   // unlock PR
        MMIO32(pr) = rate;     // max timeout, 0 = 400ms, 7 = 26s
        MMIO32(kr) = 0xCCCC;   // start watchdog
    }

    static void kick () {
        MMIO32(kr) = 0xAAAA;  // reset the watchdog timout
    }

    static void reload (int n) {
        while (Periph::bit(sr, 1)) {}  // wait until !RVU
        MMIO32(kr) = 0x5555;   // unlock PR
        MMIO32(rlr) = n;
        kick();
    }
};
