#include <cstdint>

namespace device {
#include "prelude.h"
}

namespace altpins {
#include "altpins.h"
}

namespace mcu {
    enum ARM_Family { STM_F4, STM_L0, STM_L4 };
#if STM32F4
    constexpr auto FAMILY = STM_F4;
#elif STM32L0
    constexpr auto FAMILY = STM_L0;
#elif STM32L4
    constexpr auto FAMILY = STM_L4;
#endif

    auto micros ();
    auto millis ();
    void msWait (uint16_t ms);
    auto systemClock ();
    auto fastClock (bool pll =true) -> uint32_t;
    void powerDown (bool standby =true);
    [[noreturn]] void systemReset ();

    struct IOWord {
        uint32_t volatile& addr;

        constexpr IOWord (uint32_t volatile& a) : addr (a) {}

        operator uint32_t () const { return addr; }

        void operator= (uint32_t v) const { addr = v; }

#if STM32L0
        // simulated bit-banding, not atomic
        struct IOBit {
            uint32_t volatile& addr;
            uint8_t bit;

            constexpr IOBit (uint32_t a, uint32_t b)
                : addr (*(uint32_t volatile*) a), bit (b) {}

            operator uint32_t () const { return (addr >> bit) & 1; }

            void operator= (uint32_t v) const {
                if (v) addr |= (1<<bit); else addr &= ~(1<<bit);
            }
        };
        auto operator[] (int b) -> IOBit {
            return {(uint32_t) (&addr + (b >> 5)), b & 0x1FU};
        }
#else
        // bit-banding, only works for specific RAM and periperhal areas
        auto operator[] (int b) -> uint32_t volatile& {
            auto a = (uint32_t) &addr;
            return *(uint32_t volatile*)
                ((a & 0xF000'0000) + 0x0200'0000 + (a << 5) + (b << 2));
        }
#endif
        
        struct IOMask {
            uint32_t volatile& addr;
            uint8_t bit, width;

            constexpr IOMask (uint32_t a, uint32_t b, uint32_t w)
                : addr (*(uint32_t volatile*) a), bit (b), width (w) {}

            operator uint32_t () const {
                auto mask = (1<<width)-1;
                return (addr >> bit) & mask;
            }

            void operator= (uint32_t v) const {
                auto mask = (1<<width)-1;
                addr = (addr & ~(mask<<bit)) | ((v & mask)<<bit);
            }
        };
        auto mask (int b, uint8_t w) -> IOMask {
            return {(uint32_t) (&addr + (b>>5)), b&0x1FU, w};
        }
    };

    template <uint32_t A>
    auto io32 (int off =0) -> IOWord {
        return *(uint32_t volatile*) (A+off);
    }
    template <uint32_t A>
    auto io16 (int off =0) -> uint16_t volatile& {
        return *(uint16_t volatile*) (A+off);
    }
    template <uint32_t A>
    auto io8 (int off =0) -> uint8_t volatile& {
        return *(uint8_t volatile*) (A+off);
    }

#include "hw-map.h"

    struct Pin {
        uint8_t _port =0xFF, _pin =0xFF;
        
#if STM32F1
        enum { CRL=0x00, CRH=0x04, IDR=0x08, ODR=0x0C, BSRR=0x10, BRR=0x14 };
#else
        enum { MODER=0x00, TYPER=0x04, OSPEEDR=0x08, PUPDR=0x0C, IDR=0x10,
                ODR=0x14, BSRR=0x18, AFRL=0x20, AFRH=0x24, BRR=0x28 };
#endif

        auto isValid () const -> bool { return _port < 16 && _pin < 16; }

        auto read () const -> int { return gpio32(IDR)[_pin]; }
        void write (int v) const { gpio32(BSRR) = (v ? 1 : 1<<16)<<_pin; }

        // shorthand
        void toggle () const { write(read() ^ 1); }
        operator int () const { return read(); }
        void operator= (int v) const { write(v); }

        // mode string: [AFDUPO][LNHV][<n>][,]
        auto mode (char const* desc) const -> bool;

        // pin definition string: [A-P][<n>]:[<mode>][,]
        auto define (char const* desc) {
            _port = *desc++ - 'A';
            _pin = 0;
            while ('0' <= *desc && *desc <= '9')
                _pin = 10 * _pin + *desc++ - '0';
            return isValid() && (*desc++ != ':' || mode(desc));
        }

        // define multiple pins, returns ptr to error or nullptr
        static auto define (char const* d, Pin* v, int n) -> char const*;
    private:
        auto gpio32 (int off) const -> IOWord { return GPIO(0x400*_port+off); }
        void mode (int mval, int alt =0) const;
    };

    struct Device {
        virtual ~Device () =default;

        virtual void irqHandler () =0;
        virtual void trigger () {}

        // install the uart IRQ dispatch handler in the hardware IRQ vector
        void installIrq (int irq);

        static uint8_t irqMap [100]; // TODO wrong size ...
        static Device* devMap [20]; // large enough to handle all device objects
    };
}
