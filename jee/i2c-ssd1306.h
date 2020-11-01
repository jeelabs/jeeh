// Driver for an SSD1306-based 128x64 OLED display, connected over I2C
// see https://jeelabs.org/ref/SSD1306.pdf

template< typename I2C, bool BIG =false, int addr =0x3C >
struct SSD1306 {
    constexpr static int width = 128;
    constexpr static int height = BIG ? 64 : 32;

    static void init () {
        static uint8_t config [] = {
            0xAE,  // DISPLAYOFF
            0xA8,  // SETMULTIPLEX
            height-1,
            0xD3,  // SETDISPLAYOFFSET
               0,
            0x40,  // SETSTARTLINE
            0x20,  // MEMORYMODE
            0x00,
            0x21,  // SET COL ADDR
               0,  // COL START
             127,  // COL END
            0xA1,  // SEGREMAP | 0x1
            0xC8,  // COMSCANDEC
            0xDA,  // SETCOMPINS
            BIG ? 0x12 : 0x02,
            0x81,  // SETCONTRAST
            BIG ? 0xCF : 0x8F,
            0xD9,  // SETPRECHARGE
            0xF1,
            0xDB,  // SETVCOMDETECT
            0x40,
            0x2E,  // STOP SCROLL
            0xD5,  // SETDISPLAYCLOCKDIV
            0x80,
            0x8D,  // CHARGEPUMP
            0x14,  // switched capacitor
            0xA4,  // DISPLAYALLON_RESUME
            0xA6,  // NORMALDISPLAY
            0xAF,  // DISPLAYON
        };

        for (int i = 0; i < (int) sizeof config; ++i)
            cmd(config[i]);
    }

    static void clear () {
        cmd(0xB0);  // SET PAGE START
        cmd(0x00);  // SETLOWCOLUMN
        cmd(0x10);  // SETHIGHCOLUMN
        cmd(0x40);  // SETSTARTLINE

        I2C::start(addr<<1);
        I2C::write(0x40);
        for (int i = 0; i < width*height/8; ++i)
            I2C::write(0);
        I2C::stop();
    }

    // data is written in "bands" of 8 pixels high, bit 0 is the topmost line
    static void copyBand (int x, int y, uint8_t const* ptr, int len) {
        cmd(0xB0 + (y>>3));   // SET PAGE START
        cmd(0x00 + (x&0xF));  // SETLOWCOLUMN
        cmd(0x10 + (x>>4));   // SETHIGHCOLUMN

        I2C::start(addr<<1);
        I2C::write(0x40);
        for (int i = 0; i < len; ++i)
            I2C::write(ptr[i]);
        I2C::stop();
    }

    static void show64x64 (uint32_t const* logo) {
        // the input layout is nice for editing, but tricky to remap to pixels
        // ... because the respective bit and byte orders are totally different
        for (int y = 0; y < 64; y += 8) {
            uint8_t buf [64];
            for (int x = 0; x < 64; ++x)
                for (int i = 0; i < 8; ++i) {
                    buf[x] >>= 1;
                    buf[x] |= (((logo[((y+i)<<1)|(x>>5)]>>(~x&0x1F))&1) << 7);
                }
            copyBand(32, y, buf, sizeof buf);
        }
    }

    static void cmd (uint8_t c) {
        I2C::start(addr<<1);
        I2C::write(0x80);
        I2C::write(c);
        I2C::stop();
    }
};
