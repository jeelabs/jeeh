// Hardware access for STM32F103 family microcontrollers
// see [1] https://jeelabs.org/ref/STM32F1-RM0008.pdf

namespace Periph {  // [1] p.49-50
    constexpr uint32_t rtc   = 0x40002800;
    constexpr uint32_t iwdg  = 0x40003000;
    constexpr uint32_t usb   = 0x40005C00;
    constexpr uint32_t bkp   = 0x40006C00;
    constexpr uint32_t pwr   = 0x40007000;
    constexpr uint32_t afio  = 0x40010000;
    constexpr uint32_t exti  = 0x40010400;
    constexpr uint32_t gpio  = 0x40010800;
    constexpr uint32_t rcc   = 0x40021000;
    constexpr uint32_t flash = 0x40022000;
    constexpr uint32_t crc   = 0x40023000;
    constexpr uint32_t dwt   = 0xE0001000;

    inline volatile uint32_t& bit (uint32_t a, int b) {
        return MMIO32(0x42000000 + ((a & 0xFFFFF) << 5) + (b << 2));
    }
    inline void bitSet (uint32_t a, int b) { bit(a, b) = 1; }
    inline void bitClear (uint32_t a, int b) { bit(a, b) = 0; }
}

// interrupt vector table in ram

struct VTable {
    typedef void (*Handler)();

    uint32_t* initial_sp_value;
    Handler
        reset, nmi, hard_fault, memory_manage_fault, bus_fault, usage_fault,
        dummy_x001c[4], sv_call, debug_monitor, dummy_x0034, pend_sv, systick;
    Handler
        wwdg, pvd, tamper, rtc, flash, rcc, exti0, exti1, exti2, exti3, exti4,
        dma1_channel1, dma1_channel2, dma1_channel3, dma1_channel4,
        dma1_channel5, dma1_channel6, dma1_channel7, adc1_2, usb_hp_can_tx,
        usb_lp_can_rx0, can_rx1, can_sce, exti9_5, tim1_brk, tim1_up,
        tim1_trg_com, tim1_cc, tim2, tim3, tim4, i2c1_ev, i2c1_er, i2c2_ev,
        i2c2_er, spi1, spi2, usart1, usart2, usart3, exti15_10, rtc_alarm,
        usb_wakeup, tim8_brk, tim8_up, tim8_trg_com, tim8_cc, adc3, fsmc, sdio,
        tim5, spi3, uart4, uart5, tim6, tim7, dma2_channel1, dma2_channel2,
        dma2_channel3, dma2_channel4_5, dma2_channel5, eth, eth_wkup, can2_tx,
        can2_rx0, can2_rx1, can2_sce, otg_fs;
};

// systick and delays

constexpr static int defaultHz = 8000000;
extern void enableSysTick (uint32_t divider =defaultHz/1000) __attribute__((weak));

// gpio

enum class Pinmode {  // [1] p.170
    in_analog        = 0b0000,
    in_float         = 0b0100,
    in_pulldown      = 0b1000,  // pseudo mode, also clears output
    in_pullup        = 0b1100,  // pseudo mode, also sets output

    out              = 0b0001,
    out_od           = 0b0101,
    alt_out          = 0b1001,
    alt_out_od       = 0b1101,

    out_2mhz         = 0b0010,
    out_od_2mhz      = 0b0110,
    alt_out_2mhz     = 0b1010,
    alt_out_od_2mhz  = 0b1110,

    out_50mhz        = 0b0011,
    out_od_50mhz     = 0b0111,
    alt_out_50mhz    = 0b1011,
    alt_out_od_50mhz = 0b1111,
};

template<char port>
struct Port {  // [1] pp.170
    constexpr static uint32_t base = Periph::gpio + 0x400*(port-'A');
    constexpr static uint32_t crl  = base + 0x00;
    constexpr static uint32_t crh  = base + 0x04;
    constexpr static uint32_t idr  = base + 0x08;
    constexpr static uint32_t odr  = base + 0x0C;
    constexpr static uint32_t bsrr = base + 0x10;
    constexpr static uint32_t brr  = base + 0x14;

    static void mode (int pin, Pinmode m) {
        // enable GPIOx and AFIO clocks
        MMIO32(Periph::rcc+0x18) |= (1 << (port-'A'+2)) | (1<<0);

        auto mval = static_cast<int>(m);
        if (mval == 0b1000 || mval == 0b1100) {
            uint16_t mask = 1U << pin;
            MMIO32(bsrr) = mval & 0b0100 ? mask : mask << 16;
            mval = 0b1000;
        }

        uint32_t cr = pin & 8 ? crh : crl;
        int shift = 4 * (pin & 7);
        MMIO32(cr) = (MMIO32(cr) & ~(0xF << shift)) | (mval << shift);
    }

    static void modeMap (uint16_t pins, Pinmode m) {
        for (int i = 0; i < 16; ++i) {
            if (pins & 1)
                mode(i, m);
            pins >>= 1;
        }
    }
};

template<char port,int pin>
struct Pin {
    typedef Port<port> gpio;
    constexpr static uint16_t mask = 1U << pin;
    constexpr static int id = 16 * (port-'A') + pin;

    static void mode (Pinmode m) {
        gpio::mode(pin, m);
    }

    static int read () {
        return mask & MMIO32(gpio::idr) ? 1 : 0;
    }

    static void write (int v) {
        // MMIO32(v ? gpio::bsrr : gpio::brr) = mask;
        // this is slightly faster when v is not known at compile time:
        MMIO32(gpio::bsrr) = v ? mask : mask << 16;  // [1] p.172
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
struct UartDev {  // [1] pp.819
    constexpr static int uidx = TX::id ==  9 ? 0 :  // PA9,  USART1
                                TX::id == 22 ? 0 :  // PB6,  USART1, remapped
                                TX::id ==  2 ? 1 :  // PA2,  USART2
                                TX::id == 53 ? 1 :  // PD5,  USART2, remapped
                                TX::id == 26 ? 2 :  // PB10, USART3
                             // TX::id == 42 ? 2 :  // PC10, USART3, remapped
                                TX::id == 56 ? 2 :  // PD8,  USART3, remapped
                                TX::id == 42 ? 3 :  // PC10, UART4
                                TX::id == 44 ? 4 :  // PC12, UART5
                                               0;   // else  USART1
    constexpr static uint32_t base = uidx == 0 ? 0x40013800 :  // [1] p.50-51
                                                 0x40004000 + 0x400*uidx;
    constexpr static uint32_t sr  = base + 0x00;
    constexpr static uint32_t dr  = base + 0x04;
    constexpr static uint32_t brr = base + 0x08;
    constexpr static uint32_t cr1 = base + 0x0C;

    static void init () {
        TX::mode(Pinmode::alt_out);
        RX::mode(Pinmode::in_pullup);

        if (uidx == 0)
            Periph::bit(Periph::rcc+0x18, 14) = 1; // enable USART1 clock
        else
            Periph::bit(Periph::rcc+0x1C, 16+uidx) = 1; // U(S)ART 2..5

        baud(115200);
        MMIO32(cr1) = (1<<13) | (1<<3) | (1<<2);  // UE, TE, RE
    }

    static void baud (uint32_t baud, uint32_t hz =defaultHz) {
        MMIO32(brr) = (hz + baud/2) / baud;
    }

    static bool writable () {
        return (MMIO32(sr) & (1<<7)) != 0;  // TXE
    }

    static void putc (int c) {
        while (!writable()) {}
        MMIO32(dr) = (uint8_t) c;
    }

    static bool readable () {
        return (MMIO32(sr) & ((1<<5) | (1<<3))) != 0;  // RXNE or ORE
    }

    static int getc () {
        while (!readable()) {}
        return MMIO32(dr);
    }
};

// interrupt-enabled uart, sits on top of polled uart

template< typename TX, typename RX, int NTX =25, int NRX =NTX >
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
                    Periph::bit(base::cr1, 7) = 0;  // disable TXEIE
            }
        };

        // nvic interrupt numbers are 37, 38, 39, 52, and 53, respectively
        constexpr uint32_t nvic_en1r = 0xE000E104;
        constexpr int irq = (base::uidx < 3 ? 37 : 49) + base::uidx;
        MMIO32(nvic_en1r) = 1 << (irq-32);  // enable USART interrupt

        Periph::bit(base::cr1, 5) = 1;  // enable RXNEIE
    }

    static bool writable () {
        return xmit.free();
    }

    static void putc (int c) {
        while (!writable()) {}
        xmit.put(c);
        Periph::bit(base::cr1, 7) = 1;  // enable TXEIE
    }

    static bool readable () {
        return recv.avail() > 0;
    }

    static int getc () {
        while (!readable()) {}
        return recv.get();
    }

    static RingBuffer<NRX> recv;
    static RingBuffer<NTX> xmit;
};

template< typename TX, typename RX, int NTX, int NRX >
RingBuffer<NRX> UartBufDev<TX,RX,NTX,NRX>::recv;

template< typename TX, typename RX, int NTX, int NRX >
RingBuffer<NTX> UartBufDev<TX,RX,NTX,NRX>::xmit;

// system clock

void enableClkAt8MHz ();  // [1] p.49
void enableClkAt72MHz (); // [1] p.49
int fullSpeedClock ();    // returns clock speed

// real-time clock

struct RTC {  // [1] pp.486
    constexpr static uint32_t bdcr = Periph::rcc + 0x20;
    constexpr static uint32_t crl  = Periph::rtc + 0x04;
    constexpr static uint32_t prll = Periph::rtc + 0x0C;
    constexpr static uint32_t cnth = Periph::rtc + 0x18;
    constexpr static uint32_t cntl = Periph::rtc + 0x1C;

    RTC () {
        MMIO32(Periph::rcc+0x1C) |= (0b11<<27);  // enable PWREN and BKPEN
        Periph::bit(Periph::pwr, 8) = 1;  // set DBP [1] p.481
    }

    void init () {
        Periph::bit(bdcr, 0) = 1;       // LSEON backup domain
        wait();
        Periph::bit(bdcr, 8) = 1;       // RTSEL = LSE
        Periph::bit(bdcr, 15) = 1;      // RTCEN
        Periph::bit(crl, 3) = 0;        // clear RSF
        while (Periph::bit(crl, 3)) {}  // wait for RSF
        wait();
        Periph::bit(crl, 4) = 1;        // set CNF
        MMIO32(prll) = 32767;           // set PRLL for 32 kHz crystal
        Periph::bit(crl, 4) = 0;        // clear CNF
        wait();
    }

    void wait () {
        while (Periph::bit(bdcr, 1) == 0) {}
    }

    operator int () {
        while (true) {
            uint16_t lo = MMIO32(cntl);
            uint16_t hi = MMIO32(cnth);
            if (lo == MMIO32(cntl))
                return lo | (hi<<16);
            // if low word changed, try again
        }
    }

    void operator= (int v) {
        wait();
        Periph::bit(crl, 4) = 1;      // set CNF
        MMIO32(cntl) = (uint16_t) v;  // set lower 16 bits
        MMIO32(cnth) = v >> 16;       // set upper 16 bits
        Periph::bit(crl, 4) = 0;      // clear CNF
    }

    // access to the backup registers

    uint16_t getData (int reg) {
        return MMIO16(Periph::bkp + 4 * (reg+1));  // regs 0..9
    }

    void setData (int reg, uint16_t val) {
        MMIO16(Periph::bkp + 4 * (reg+1)) = val;  // regs 0..9
    }
};

// hardware spi support

template< typename MO, typename MI, typename CK, typename SS, int CP =0 >
struct SpiHw {  // [1] pp.742
    constexpr static int sidx = MO::id ==  7 ? 0 :  // PA7,  SPI1
                                MO::id == 21 ? 0 :  // PB5,  SPI1, remapped
                                MO::id == 31 ? 1 :  // PB15, SPI2
                            // oops, this is not possible, also remapped SPI1!
                            //  MO::id == 21 ? 2 :  // PB5,  SPI3
                                MO::id == 44 ? 2 :  // PC12, SPI3, remapped
                                               0;   // else  SPI1
    constexpr static uint32_t base = sidx == 0 ? 0x40013000 :
                                                 0x40003400 + 0x400*sidx;
    constexpr static uint32_t cr1 = base + 0x00;
    constexpr static uint32_t cr2 = base + 0x04;
    constexpr static uint32_t sr  = base + 0x08;
    constexpr static uint32_t dr  = base + 0x0C;

    static void init (uint32_t div =2) {
        SS::mode(Pinmode::out); disable();
        CK::mode(Pinmode::alt_out);
        MI::mode(Pinmode::in_float);
        MO::mode(Pinmode::alt_out);

        if (sidx == 0)
            Periph::bit(Periph::rcc+0x18, 12) = 1;  // SPI1
        else
            Periph::bit(Periph::rcc+0x1C, sidx+13) = 1;  // SPI 2..3

        // SPE, BR=2, MSTR, CPOL (clk/8, i.e. 9 MHz)
        MMIO32(cr1) = (1<<6) | (div<<3) | (1<<2) | (CP<<1);  // [1] p.742
        (void) MMIO32(sr);  // appears to be needed to avoid hang in some cases
        Periph::bit(cr2, 2) = 1;  // SSOE
    }

    static void enable () { SS::write(0); }
    static void disable () { SS::write(1); }

    static uint8_t transfer (uint8_t v) {
        MMIO32(dr) = v;
        while (Periph::bit(sr, 0) == 0) {}
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

// flash memory writing and erasing

struct Flash {
    constexpr static uint32_t keyr = Periph::flash + 0x04;
    constexpr static uint32_t sr   = Periph::flash + 0x0C;
    constexpr static uint32_t cr   = Periph::flash + 0x10;
    constexpr static uint32_t ar   = Periph::flash + 0x14;

    static void write16 (void const* addr, uint16_t val) {
        if (*(uint16_t const*) addr != 0xFFFF)
            return;
        unlock();
        MMIO32(cr) = 0x01;
        MMIO16(addr) = val;
        finish();
    }

    static void write32 (void const* addr, uint32_t val) {
        write16(addr, val);
        write16((uint16_t const*) addr + 1, val >> 16);
    }

    static void erasePage (void const* addr) {
        unlock();
        MMIO32(cr) = 0x02;
        MMIO32(ar) = (uint32_t) addr | 0x08000000;
        MMIO32(cr) = 0x42;
        finish();
    }

    static void unlock () {
        MMIO32(keyr) = 0x45670123;
        MMIO32(keyr) = 0xCDEF89AB;
    }

    static void finish () {
        while (MMIO32(sr) & (1<<0)) {}
        MMIO32(cr) = 0x80;
    }
};

#if USE_USB
// usb serial console

namespace USB {
#include "stm32f1-usb.h"
}

template< typename USBPIN, int USBPOL =0 >
struct UsbDev {
    static void init () {
        //fullSpeedClock();

        USBPIN::mode(Pinmode::out);
        USBPIN::write(USBPOL);
        wait_ms(2);
        USBPIN::write(!USBPOL);

        USB::usbd_init();
    }

    static bool writable () {
        poll();
        return USB::txFill < sizeof USB::txBuf;
    }

    static void putc (int c) {
        while (!writable()) {}
        USB::txBuf[USB::txFill++] = c;
    }

    static bool readable () {
        poll();
        return USB::rxBuf.avail() > 0;
    }

    static int getc () {
        while (!readable()) {}
        return USB::rxBuf.get();
    }

    static void poll () {
        if (USB::txFill > 0 && USB::ep_write(USB::CDC_TXD_EP, USB::txBuf,
                                                USB::txFill) == USB::txFill)
            USB::txFill = 0;
        USB::evt_poll();
    }
};
#endif

// analog input using ADC1 or ADC2

template< int N >
struct ADC {
    constexpr static uint32_t base  = 0x40012000 + 0x400*N;
    constexpr static uint32_t sr    = base + 0x00;
    constexpr static uint32_t cr1   = base + 0x04;
    constexpr static uint32_t cr2   = base + 0x08;
    constexpr static uint32_t smpr1 = base + 0x0C;
    constexpr static uint32_t smpr2 = base + 0x10;
    constexpr static uint32_t sqr3  = base + 0x34;
    constexpr static uint32_t dr    = base + 0x4C;

    static void init () {
        Periph::bit(Periph::rcc+0x18, N+8) = 1;  // enable ADC 1 or 2
        MMIO32(cr2) = (1<<23) | (1<<0);  // TSVREFE, ADON [1] pp.239
        wait_ms(2);  // see [1] p.222
        Periph::bit(cr2, 2) = 1;  // CAL
        while (Periph::bit(cr2, 2)) {}  // wait until calibration completed
        MMIO32(smpr1) = (7<<21) | (7<<18);  // slow temp/vref conv's [1] p.243
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

    // read direct channel number (also: 16 = temp, 17 = vref)
    static uint16_t read (uint8_t chan) {
        MMIO32(sqr3) = chan;
        Periph::bit(cr2, 0) = 1;  // start conversion
        while (Periph::bit(sr, 1) == 0) {}  // EOC [1] p.236
        return MMIO32(dr);
    }
};

// crc calculation

struct CRC32 {
    static uint32_t calculate (uint32_t const* ptr, int num) {
        Periph::bit(Periph::rcc+0x14, 6) = 1; // enable CRC unit
        MMIO32(Periph::crc+0x08) = 1;  // reset [1] p.64
        while (--num >= 0)
            MMIO32(Periph::crc+0x00) = *ptr++;
        return MMIO32(Periph::crc+0x00);  // calculated crc32
    }
};

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

    static void init (bool singleWire =false, bool alt =false) {
        auto swMode = singleWire ? Pinmode::alt_out_od : Pinmode::alt_out;
        if (alt) {
            MMIO32(Periph::afio+0x04) |= (2<<13); // CAN remap to B9+B8
            Pin<'B',8>::mode(Pinmode::in_float);
            Pin<'B',9>::mode(swMode);
        } else {
            Pin<'A',11>::mode(Pinmode::in_float);
            Pin<'A',12>::mode(swMode);
        }
        Periph::bit(Periph::rcc+0x1C, 25) = 1;  // enable CAN1

        Periph::bit(mcr, 1) = 0; // exit sleep
        MMIO32(mcr) |= (1<<6) | (1<<0); // set ABOM, init req
        while (Periph::bit(msr, 0) == 0) {}
        //MMIO32(btr) = (7<<20) | (5<<16) | (2<<0); // 1 MBps
        MMIO32(btr) = (6<<20) | (9<<16) | (1<<0); // 1 MBps
        Periph::bit(mcr, 0) = 0; // init leave
        while (Periph::bit(msr, 0)) {}
        Periph::bit(fmr, 0) = 0; // ~FINIT
    }

    static void filterInit (int num, int id =0, int mask =0) {
        Periph::bit(far, num) = 0; // ~FACT
        Periph::bit(fsr, num) = 1; // FSC 32b
        MMIO32(fr1 + 8*num) = id;
        MMIO32(fr2 + 8*num) = mask;
        Periph::bit(far, num) = 1; // FACT
    }

    static bool transmit (int id, const void* ptr, int len) {
        if (Periph::bit(tsr, 26)) { // TME0
            MMIO32(tir) = (id<<21);
            MMIO32(tdtr) = (len<<0);
            // this assumes that misaligned word access works
            MMIO32(tdlr) = ((const uint32_t*) ptr)[0];
            MMIO32(tdhr) = ((const uint32_t*) ptr)[1];

            Periph::bit(tir, 0) = 1; // TXRQ
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
            Periph::bit(rfr, 5) = 1; // RFOM
        }
        return len;
    }
};

// low-power modes

extern void powerDown (bool standby =true);

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
    constexpr static uint32_t cr2   = base + 0x04;
    constexpr static uint32_t dier  = base + 0x0C;
    constexpr static uint32_t ccmr1 = base + 0x18;
    constexpr static uint32_t ccmr2 = base + 0x1C;
    constexpr static uint32_t ccer  = base + 0x20;
    constexpr static uint32_t psc   = base + 0x28;
    constexpr static uint32_t arr   = base + 0x2C;
    constexpr static uint32_t ccr1  = base + 0x34;
    constexpr static uint32_t ccr2  = base + 0x38;
    constexpr static uint32_t ccr3  = base + 0x3C;
    constexpr static uint32_t ccr4  = base + 0x40;

    static void init (uint32_t limit, uint32_t scale =0) {
        if (tidx < 64)
            Periph::bit(Periph::rcc+0x1C, tidx) = 1;
        else
            Periph::bit(Periph::rcc+0x18, tidx-64) = 1;
        MMIO16(psc) = scale;
        MMIO16(arr) = limit-1;
        Periph::bit(dier, 8) = 1; // UDE
        MMIO16(cr2) = 2<<4; // MMS = update
        Periph::bit(cr1, 0) = 1; // CEN
    }

    // TODO TIM1 (and TIM8?) don't seem to work with PWM
    // TODO hard-coded for 3rd timer (to get to PB0)
    static void pwm (uint32_t match) {
        //MMIO32(ccmr1) = 0x60; // PWM mode
        MMIO16(ccmr2) = 0x60; // PWM mode
        //MMIO32(ccr1) = match;
        MMIO32(ccr3) = match;
        //Periph::bit(ccer, 0) = 1; // CC3E
        Periph::bit(ccer, 8) = 1; // CC3E
    }
};

// cycle counts, see https://stackoverflow.com/questions/11530593/

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
