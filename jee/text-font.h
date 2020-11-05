// Monospaced ASCII fonts, e.g. for OLED with copyBand support.

extern uint8_t const font5x7 [];

template< typename T, int S =6 >
struct Font5x7 {
    constexpr static int width = T::width;
    constexpr static int height = T::height;
    constexpr static int tabsize = 8 * S;

    static void putc (int c) {
        switch (c) {
            case '\f': x = y = 0; T::clear(); return;
            case '\t': x += tabsize - x % tabsize; return;
            case '\r': x = 0; return;
            case '\n': x = width; break;
        }
        if (x + S > width) {
            x = 0;
            y += 8;
            if (y >= height)
                y = 0;
            // fill the new line with spaces
            T::copyBand(0, y, (uint8_t const*) "", width, 0);
        }
        if (c != '\n') {
            if (c < ' ' || c > 127)
                c = 127;
            uint8_t const* p = font5x7 + 5 * (c-' ');
            T::copyBand(x, y, p, 5);
            x += S;
        }
    }

    static uint16_t y, x;
};

template< typename T, int S >
uint16_t Font5x7<T,S>::y;

template< typename T, int S >
uint16_t Font5x7<T,S>::x;

extern uint8_t const font5x8 [];

template< typename T, int S =5 >
struct Font5x8 {
    constexpr static int width = T::width;
    constexpr static int height = T::height;
    constexpr static int tabsize = 8 * S;

    static void putc (int c) {
        switch (c) {
            case '\f': x = y = 0; T::clear(); return;
            case '\t': x += tabsize - x % tabsize; return;
            case '\r': x = 0; return;
            case '\n': x = width; break;
        }
        if (x + S > width) {
            x = 0;
            y += 8;
            if (y >= height)
                y = 0;
            // fill the new line with spaces
            T::copyBand(0, y, (uint8_t const*) "", width, 0);
        }
        if (c != '\n') {
            if (c < ' ' || c > 127)
                c = 127;
            uint8_t const* p = font5x8 + 5 * (c-' ');
            T::copyBand(x, y, p, 5);
            x += S;
        }
    }

    static uint16_t y, x;
};

template< typename T, int S >
uint16_t Font5x8<T,S>::y;

template< typename T, int S >
uint16_t Font5x8<T,S>::x;

extern uint8_t const font11x16 [];

template< typename T, int S =11 >
struct Font11x16 {
    constexpr static int width = T::width;
    constexpr static int height = T::height;
    constexpr static int tabsize = 4 * S;

    static void putc (int c) {
        switch (c) {
            case '\f': x = y = 0; T::clear(); return;
            case '\t': x += tabsize - x % tabsize; return;
            case '\r': x = 0; return;
            case '\n': x = width; break;
        }
        if (x + S > width) {
            x = 0;
            y += 16;
            if (y >= height)
                y = 0;
            // fill the new line with spaces
            for (int j = 0; j < 2; ++j)
                T::copyBand(0, (y+8*j)%height, (uint8_t const*) "", width, 0);
        }
        if (c != '\n') {
            if (c < ' ' || c > 127)
                c = 127;
            uint8_t const* p = font11x16 + 2*11*(c-' ');
            T::copyBand(x, y, p, 11, 2);
            T::copyBand(x, (y+8)%height, p+1, 11, 2);
            x += S;
        }
    }

    static uint16_t y, x;
};

template< typename T, int S >
uint16_t Font11x16<T,S>::y;

template< typename T, int S >
uint16_t Font11x16<T,S>::x;

extern uint8_t const font17x24 [];

template< typename T, int S =17 >
struct Font17x24 {
    constexpr static int width = T::width;
    constexpr static int height = T::height;
    constexpr static int tabsize = 4 * S;

    static void putc (int c) {
        switch (c) {
            case '\f': x = y = 0; T::clear(); return;
            case '\t': x += tabsize - x % tabsize; return;
            case '\r': x = 0; return;
            case '\n': x = width; break;
        }
        if (x + S > width) {
            x = 0;
            y += 24;
            if (y >= height)
                y = 0;
            // fill the new line with spaces
            for (int j = 0; j < 3; ++j)
                T::copyBand(0, (y+8*j)%height, (uint8_t const*) "", width, 0);
        }
        if (c != '\n') {
            if (c < ' ' || c > 127)
                c = 127;
            uint8_t const* p = font17x24 + 3*17*(c-' ');
            T::copyBand(x, y, p, 17, 3);
            T::copyBand(x, (y+8)%height, p+1, 17, 3);
            T::copyBand(x, (y+16)%height, p+2, 17, 3);
            x += S;
        }
    }

    static uint16_t y, x;
};

template< typename T, int S >
uint16_t Font17x24<T,S>::y;

template< typename T, int S >
uint16_t Font17x24<T,S>::x;

// optional adapter to print scrolling text on pixel-oriented displays

template< typename LCD >
struct TextLcd : LCD {
    static void copyBand (int x, int y, uint8_t const* ptr, int len) {
        // not very efficient code, but it avoids a large buffer
        uint16_t col [8];
        for (int xi = 0; xi < len; ++xi) {
            uint8_t v = *ptr++;
            for (int i = 0; i < 8; ++i) {
                col[i] = v & 1 ? fg : bg;
                v >>= 1;
            }
            LCD::pixels(x+xi, y, col, 8);
        }

        // when in col 0: adjust v-scroll so the band is always at the bottom
        if (x == 0) {
            y += 8;
            LCD::vscroll(y < LCD::height ? y : 0);
        }
    }

    static uint16_t fg, bg;
};

template< typename LCD >
uint16_t TextLcd<LCD>::fg = 0xFFFF;

template< typename LCD >
uint16_t TextLcd<LCD>::bg;
