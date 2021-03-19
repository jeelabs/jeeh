// Driver for an ST7789-based 240x240 LCD TFT display, using 16b-mode FSMC
// ... used on STM's F412 and F723 Discovery boards

template< uint32_t ADDR >
struct ST7789 {
    constexpr static auto addr = ADDR;
    constexpr static int width = 240;
    constexpr static int height = 240;

    static void init () {
        static uint8_t const config [] = {
            // cmd, count, data bytes ...
            0x10, 0, 0xFF, 10,                      // SLEEP_IN
            0x01, 0, 0xFF, 200,                     // reset
            0x11, 0, 0xFF, 120,                     // SLEEP_OUT
            0x36, 1, 0x20,                          // NORMAL_DISPLAY
            0x3A, 1, 0x05,                          // COLOR_MODE
            0x21, 0,                                // DISPLAY_INVERSION
            0x2A, 4, 0x00, 0x00, 0x00, 0xEF,        // CASET
            0x2B, 4, 0x00, 0x00, 0x00, 0xEF,        // RASET
            0xB2, 5, 0x0C, 0x0C, 0x00, 0x33, 0x33,  // PORCH_CTRL
            0xB7, 1, 0x35,                          // GATE_CTRL
            0xBB, 1, 0x1F,                          // VCOM_SET
            0xC0, 1, 0x2C,                          // LCM_CTRL
            0xC2, 2, 0x01, 0xC3,                    // VDV_VRH_EN
            0xC4, 1, 0x20,                          // VDV_SET
            0xC6, 1, 0x0F,                          // FR_CTRL
            0xD0, 2, 0xA4, 0xA1,                    // POWER_CTRL
            // gamma ... 2x
            0x29, 0,                                // DISPLAY_ON
            0x11, 0,                                // SLEEP_OUT
          //0x35, 1, 0x00,                          // TEARING_EFFECT
          //0x33, 6, 0x00, 0x00, 0x01, 0xF0, 0x00, 0x00, // VSCRDEF
          //0x37, 2, 0x00, 0x50, // VSCSAD
        };

        for (uint8_t const* p = config; p < config + sizeof config; ++p) {
            if (*p == 0xFF)
                wait_ms(*++p);
            else {
                cmd(*p);
                int n = *++p;
                while (--n >= 0)
                    out16(*++p);
            }
        }

        //wait_ms(120);
        //cmd(0x29);      // DISPON
    }

    static void cmd (int v) {
        MMIO16(ADDR) = v;
        asm ("dsb");
    }

    static void out16 (int v) {
        MMIO16(ADDR+2) = v;
        asm ("dsb");
    }

    static void pixel (int x, int y, uint16_t rgb) {
        cmd(0x2A);
        out16(y>>8);
        out16(y);
        out16(yEnd>>8);
        out16(yEnd);

        cmd(0x2B);
        out16(x>>8);
        out16(x);
        out16(xEnd>>8);
        out16(xEnd);

        cmd(0x2C);
        out16(rgb);
    }

    static void pixels (int x, int y, uint16_t const* rgb, int len) {
        pixel(x, y, *rgb);

        for (int i = 1; i < len; ++i)
            out16(rgb[i]);
    }

    static void bounds (int xend =width-1, int yend =height-1) {
        xEnd = xend;
        yEnd = yend;
    }

    static void fill (int x, int y, int w, int h, uint16_t rgb) {
        bounds(x+w-1, y+h-1);
        pixel(x, y, rgb);

        int n = w * h;
        while (--n > 0)
            out16(rgb);
    }

    static void clear () {
        fill(0, 0, width, height, 0);
    }

    static void vscroll (int vscroll =0) {
        cmd(0x33);
        out16(vscroll>>8);
        out16(vscroll);
        out16(0x00);
        out16(0xF0);
        out16(0x00);
        out16(0x00);
        //cmd(0x37);
        //out16(vscroll>>8);
        //out16(vscroll);
    }

    static uint16_t xEnd, yEnd;
};

template< uint32_t ADDR>
uint16_t ST7789<ADDR>::xEnd = width-1;

template< uint32_t ADDR>
uint16_t ST7789<ADDR>::yEnd = height-1;
