// Driver for an ILI9325-based 320x240 LCD TFT display, using 16b parallel mode
// tested with http://www.hotmcu.com/x-p-44.html
// see https://jeelabs.org/ref/ILI9325.pdf

template< typename RD, typename WR, typename RS >
struct ILI9325 {
    constexpr static int width = 240;
    constexpr static int height = 320;

    static void init () {
        RS::mode(Pinmode::out); RS::write(1);
        RD::mode(Pinmode::out); RD::write(1);
        WR::mode(Pinmode::out); WR::write(1);

        Port<'A'>::modeMap(0b0000000011111111, Pinmode::out);
        Port<'B'>::modeMap(0b1111111100000000, Pinmode::out);

        static uint16_t const config [] = {
            0xE7, 0x0010, 0x00, 0x0001, 0x01, 0x0000, 0x02, 0x0700,
            0x03, 0x1038, 0x04, 0x0000, 0x08, 0x0207, 0x09, 0x0000,
            0x0A, 0x0000, 0x0C, 0x0001, 0x0D, 0x0000, 0x0F, 0x0000,
            0x10, 0x0000, 0x11, 0x0007, 0x12, 0x0000, 0x13, 0x0000,
             999,     50, 0x10, 0x1590, 0x11, 0x0227,  999,     50,
            0x12, 0x009C,  999,     50, 0x13, 0x1900, 0x29, 0x0023,
            0x2B, 0x000E,  999,     50, 0x20, 0x0000, 0x21, 0x0000,
             999,     50, 0x30, 0x0007, 0x31, 0x0707, 0x32, 0x0006,
            0x35, 0x0704, 0x36, 0x1F04, 0x37, 0x0004, 0x38, 0x0000,
            0x39, 0x0706, 0x3C, 0x0701, 0x3D, 0x000F,  999,     50,
            0x50, 0x0000, 0x51, 0x00EF, 0x52, 0x0000, 0x53, 0x013F,
            0x60, 0xA700, 0x61, 0x0003, 0x6A, 0x0000, 0x80, 0x0000,
            0x81, 0x0000, 0x82, 0x0000, 0x83, 0x0000, 0x84, 0x0000,
            0x85, 0x0000, 0x90, 0x0010, 0x92, 0x0000, 0x93, 0x0003,
            0x95, 0x0110, 0x97, 0x0000, 0x98, 0x0000, 0x07, 0x0133,
            0x20, 0x0000, 0x21, 0x0000, 9999
        };
        
        for (uint16_t const* p = config; p[0] != 9999; p += 2)
            if (p[0] == 999)
                wait_ms(p[1]);
            else
                write(p[0], p[1]);
    }

    static void write (int reg, int val) {
        RS::write(0);
        out16(reg);
        RS::write(1);
        out16(val);
    }

    static void out16 (int v) {
        static Port<'A'> pa;
        static Port<'B'> pb;

        MMIO32(pa.bsrr) = 0x00FF0000 | (v & 0x00FF);
        MMIO32(pb.bsrr) = 0xFF000000 | (v & 0xFF00);

        WR::write(0);
        WR::write(1);
    }

    static void pixel (int x, int y, uint16_t rgb) {
        write(0x20, x);
        write(0x50, x);
        write(0x21, y);
        write(0x52, y);
        write(0x22, rgb);
    }

    static void pixels (int x, int y, uint16_t const* rgb, int len) {
        pixel(x, y, *rgb);

        for (int i = 1; i < len; ++i)
            out16(rgb[i]);
    }

    static void bounds (int xend =width-1, int yend =height-1) {
        write(0x51, xend);
        write(0x53, yend);
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
        write(0x6A, vscroll);
    }
};
