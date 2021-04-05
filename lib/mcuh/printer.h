struct Printer {
    Printer (void* arg, void (*fun) (void*, uint8_t const*, int))
        : fun (fun), arg (arg), fill (0), limit (sizeof buf) {}

    auto vprintf(char const* fmt, va_list ap) {
        count = 0;
        while (*fmt)
            if (char c = *fmt++; c != '%')
                emit(c);
            else {
                pad = *fmt == '0' ? '0' : ' ';
                width = radix = 0;
                while (radix == 0)
                    switch (c = *fmt++) {
                        case 'b': radix =  2; break;
                        case 'o': radix =  8; break;
                        case 'u':
                        case 'd': radix = 10; break;
                        case 'p': pad = '0'; width = 8;
                                  [[fallthrough]];
                        case 'x': radix = 16; break;
                        case 'c': putFiller(width - 1);
                                  c = va_arg(ap, int);
                                  [[fallthrough]];
                        case '%': emit(c); radix = 1; break;
                        case '*': width = va_arg(ap, int); break;
                        case 's': { char const* s = va_arg(ap, char const*);
                                    while (*s) {
                                        emit(*s++);
                                        --width;
                                    }
                                    putFiller(width);
                                  }
                                  [[fallthrough]];
                        default:  if ('0' <= c && c <= '9')
                                      width = 10 * width + c - '0';
                                  else
                                      radix = 1; // stop scanning
                    }
                if (radix > 1) {
                    // %b output needs up to 32 chars, the rest can do with 12
                    if (fill > limit - (radix == 2 ? 32 : 12))
                        flush(); // flush now, so there is always enough room
                    int val = va_arg(ap, int);
                    auto sign = val < 0 && c == 'd';
                    uint32_t num = sign ? -val : val;
                    do {
                        buf[--limit] = "0123456789ABCDEF"[num % radix];
                        num /= radix;
                    } while (num != 0);
                    if (sign) {
                        if (pad == ' ')
                            buf[--limit] = '-';
                        else {
                            --width;
                            emit('-');
                        }
                    }
                    putFiller(width - (sizeof buf - limit));
                    while (limit < (signed) sizeof buf)
                        emit(buf[limit++]);
                }
            }
        flush();
        return count;
    }

    auto operator() (const char* fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        int result = vprintf(fmt, ap);
        va_end(ap);
        return result;
    }

    void (*fun) (void*, uint8_t const*, int);
    void* arg;
private:
    uint16_t count;
    int8_t pad, width, radix, fill, limit;
    uint8_t buf [48];

    void flush () {
        auto len = fill;
        count += len;
        fill = 0;
        fun(arg, buf, len);
    }

    void emit (int c) {
        if (fill >= limit)
            flush();
        buf[fill++] = c;
    }

    void putFiller (int n) {
        while (--n >= 0)
            emit(pad);
    }
};
