#include <Arduino.h>
#undef getc
#undef putc

#define main myMain
extern int myMain ();

#define wait_ms delay
#define ticks millis()

extern void enableSysTick (uint32_t divider =0) __attribute__((weak));

enum class Pinmode {
    in_float  = INPUT,
    in_pullup = INPUT_PULLUP,
    out       = OUTPUT,
    out_od    = INPUT_PULLUP,
};

template<char port,int pin>
struct Pin {
    constexpr static int offset = port == 'B' ?  8 :  // PB 0..5 =>  8..13
                                  port == 'C' ? 14 :  // PC 0..7 => 14..21
                                                 0;   // PD 0..7 =>  0..7
    constexpr static int id = offset + pin;

    static void mode (Pinmode m) { pinMode(id, (int) m); }
    static int read () { return digitalRead(id); }
    static void write (int v) { digitalWrite(id, v); }

    // shorthand
    operator int () const { return read(); }
    void operator= (int v) const { write(v); }

    static void toggle () { write(!read()); }
};

template< typename TX, typename RX >
class UartDev {
public:
    static bool init () { return true; }
    static void baud (int, int =0) {}
    static bool writable () { return true; }
    static void putc (int c) { Serial.write((char) c); }
    static bool readable () { return Serial.available(); }
    static int getc () { return Serial.read(); }
};

#define UartBufDev UartDev
#define fullSpeedClock() 1
