struct Periph {
    constexpr static uint32_t pwr   = 0x40007000;
    constexpr static uint32_t rcc   = 0x40021000;
    constexpr static uint32_t flash = 0x40022000;
    constexpr static uint32_t gpio  = 0x48000000;
};

// interrupt vector table in ram

struct VTable {
    typedef void (*Handler)();

    uint32_t* initial_sp_value;
    Handler
        reset, nmi, hard_fault, memory_manage_fault, bus_fault, usage_fault,
        dummy_x001c[4], sv_call, debug_monitor, dummy_x0034, pend_sv, systick;
    Handler
        wwdg, pvd_pvm, tamp_stamp, rtc_wkup, flash, rcc, exti0, exti1, exti2,
        exti3, exti4, dma1_channel1, dma1_channel2, dma1_channel3,
        dma1_channel4, dma1_channel5, dma1_channel6, dma1_channel7, adc1_2,
        can1_tx, can1_rx0, can1_rx1, can1_sce, exti9_5, tim1_brk_tim15,
        tim1_up_tim16, tim1_trg_com_tim17, tim1_cc, tim2, tim3, tim4, i2c1_ev,
        i2c1_er, i2c2_ev, i2c2_er, spi1, spi2, usart1, usart2, usart3,
        exti15_10, rtc_alarm, dfsdm3, tim8_brk, tim8_up, tim8_trg_com, tim8_cc,
        adc3, fmc, sdmmc1, tim5, spi3, uart4, uart5, tim6_dacunder, tim7,
        dma2_channel1, dma2_channel2, dma2_channel3, dma2_channel4,
        dma2_channel5, dfsdm0, dfsdm1, dfsdm2, comp, lptim1, lptim2, otg_fs,
        dma2_channel6, dma2_channel7, lpuart1, quadspi, i2c3_ev, i2c3_er, sai1,
        sai2, swpmi1, tsc, lcd, aes, rng, fpu;
};

// systick and delays

constexpr static int defaultHz = 4000000;
extern void enableSysTick (uint32_t divider =defaultHz/1000);

// gpio

enum class Pinmode {
    // mode (2), typer (1), pupdr (2)
    in_analog        = 0b11000,
    in_float         = 0b00000,
    in_pulldown      = 0b00010,
    in_pullup        = 0b00001,

    out              = 0b01000,
    out_od           = 0b01100,
    alt_out          = 0b10000,
    alt_out_od       = 0b10100,
};

template<char port>
struct Port {
    constexpr static uint32_t base    = Periph::gpio + 0x400 * (port-'A');
    constexpr static uint32_t moder   = base + 0x00;
    constexpr static uint32_t typer   = base + 0x04;
    constexpr static uint32_t ospeedr = base + 0x08;
    constexpr static uint32_t pupdr   = base + 0x0C;
    constexpr static uint32_t idr     = base + 0x10;
    constexpr static uint32_t odr     = base + 0x14;
    constexpr static uint32_t bsrr    = base + 0x18;
    constexpr static uint32_t afrl    = base + 0x20;
    constexpr static uint32_t afrh    = base + 0x24;
    constexpr static uint32_t brr     = base + 0x28;

    static void mode (int pin, Pinmode m, int alt =0) {
        // enable GPIOx clock
        MMIO32(Periph::rcc + 0x4C) |= 1 << (port-'A');

        auto mval = static_cast<int>(m);
        MMIO32(moder) = (MMIO32(moder) & ~(3 << 2*pin))
                      | ((mval >> 3) << 2*pin);
        MMIO32(typer) = (MMIO32(typer) & ~(1 << pin))
                      | (((mval >> 2) & 1) << pin);
        MMIO32(pupdr) = (MMIO32(pupdr) & ~(3 << 2*pin))
                      | ((mval & 3) << 2*pin);
        MMIO32(ospeedr) = (MMIO32(ospeedr) & ~(3 << 2*pin)) | (0b11 << 2*pin);

        uint32_t afr = pin & 8 ? afrh : afrl;
        int shift = 4 * (pin & 7);
        MMIO32(afr) = (MMIO32(afr) & ~(0xF << shift)) | (alt << shift);
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
        // MMIO32(v ? gpio::bsrr : gpio::brr) = mask;
        // this is slightly faster when v is not known at compile time:
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
class UartDev {
public:
    // TODO does not recognise alternate TX pins
    constexpr static int uidx = TX::id ==   9 ? 0 :  // PA9, USART1
                                TX::id ==   2 ? 1 :  // PA2, USART2
                                TX::id ==  22 ? 2 :  // PB6, USART3
                                                0;   // else USART1
    constexpr static uint32_t base = uidx == 0 ? 0x40013800 :
                                                 0x40004000 + 0x400 * uidx;
    constexpr static uint32_t cr1 = base + 0x00;
    constexpr static uint32_t brr = base + 0x0C;
    constexpr static uint32_t isr = base + 0x1C;
    constexpr static uint32_t icr = base + 0x20;
    constexpr static uint32_t rdr = base + 0x24;
    constexpr static uint32_t tdr = base + 0x28;

    static void init () {
        tx.mode(Pinmode::alt_out, 7);
        rx.mode(Pinmode::alt_out, 7);

        if (uidx == 0)
            MMIO32(Periph::rcc + 0x60) |= 1 << 14; // enable USART1 clock
        else
            MMIO32(Periph::rcc + 0x58) |= 1 << (16+uidx); // USART 2..3

        MMIO32(brr) = 35;  // 115200 baud @ 4 MHz
        MMIO32(cr1) = (1<<3) | (1<<2) | (1<<0);  // TE, RE, UE
    }

    static void baud (uint32_t baud, uint32_t hz =defaultHz) {
        MMIO32(cr1) &= ~(1<<0);              // disable
        MMIO32(brr) = (hz + baud/2) / baud;  // change while disabled
        MMIO32(cr1) |= 1<<0;                 // enable
    }

    static bool writable () {
        return (MMIO32(isr) & 0x80) != 0;  // TXE
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

    static TX tx;
    static RX rx;
};

template< typename TX, typename RX >
TX UartDev<TX,RX>::tx;

template< typename TX, typename RX >
RX UartDev<TX,RX>::rx;

// interrupt-enabled uart, sits of top of polled uart

template< typename TX, typename RX, int N =50 >
struct UartBufDev : UartDev<TX,RX> {
    typedef UartDev<TX,RX> base;

    static void init () {
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
            case 2: VTableRam().usart3 = handler; break;
        }

        // nvic interrupt numbers are 37, 38, and 39, respectively
        constexpr uint32_t nvic_en1r = 0xE000E104;
        constexpr int irq = 37 + base::uidx;
        MMIO32(nvic_en1r) = 1 << (irq-32);  // enable USART interrupt

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

extern void enableClkAt80MHz ();
extern int fullSpeedClock ();

// low-power modes

extern void powerDown (bool standby =true);

// can bus

struct CanDev {
    constexpr static uint32_t base = 0x40006400;

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
        // alt mode CAN1:    5432109876543210
        Port<'A'>::modeMap(0b0001100000000000, swMode, 9);
        MMIO32(Periph::rcc + 0x58) |= (1<<25);  // enable CAN1

        MMIO32(mcr) &= ~(1<<1); // exit sleep
        MMIO32(mcr) |= (1<<6) | (1<<0); // set ABOM, init req
        while ((MMIO32(msr) & (1<<0)) == 0) {}
        MMIO32(btr) = (7<<20) | (10<<16) | (3<<0); // 1 MBps
        MMIO32(mcr) &= ~(1<<0); // init leave
        while (MMIO32(msr) & (1<<0)) {}
        MMIO32(fmr) &= ~(1<<0); // ~FINIT
    }

    static void filterInit (int num, int id =0, int mask =0) {
        MMIO32(far) &= ~(1<<num); // ~FACT
        MMIO32(fsr) |= (1<<num); // FSC 32b
        MMIO32(fr1 + 8 * num) = id;
        MMIO32(fr2 + 8 * num) = mask;
        MMIO32(far) |= (1<<num); // FACT
    }

    static bool transmit (int id, const void* ptr, int len) {
        if (MMIO32(tsr) & (1<<26)) { // TME0
            MMIO32(tir) = (id<<21);
            MMIO32(tdtr) = (len<<0);
            // this assumes that misaligned word access works
            MMIO32(tdlr) = ((const uint32_t*) ptr)[0];
            MMIO32(tdhr) = ((const uint32_t*) ptr)[1];

            MMIO32(tir) |= (1<<0); // TXRQ
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
            MMIO32(rfr) |= (1<<5); // RFOM
        }
        return len;
    }
};
