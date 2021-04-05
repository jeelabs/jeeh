struct Printer {
    Printer (void* arg, void (*fun) (void*, uint8_t const*, int))
        : fun (fun), arg (arg), next (0), limit (sizeof buf) {}

    auto veprintf(char const* fmt, va_list ap) {
        count = 0;
        while (*fmt)
            if (char c = *fmt++; c != '%')
                emit(c);
            else {
                fill = *fmt == '0' ? '0' : ' ';
                width = base = 0;
                while (base == 0)
                    switch (c = *fmt++) {
                        case 'b': base =  2; break;
                        case 'o': base =  8; break;
                        case 'u':
                        case 'd': base = 10; break;
                        case 'p': fill = '0'; width = 8;
                                  [[fallthrough]];
                        case 'x': base = 16; break;
                        case 'c': putFiller(width - 1);
                                  c = va_arg(ap, int);
                                  [[fallthrough]];
                        case '%': emit(c); base = 1; break;
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
                                      base = 1; // stop scanning
                    }
                if (base > 1) {
                    // %b output needs up to 32 chars, the rest can do with 12
                    if (next > limit - (base == 2 ? 32 : 12))
                        flush(); // flush now, so there is always enough room
                    int val = va_arg(ap, int);
                    auto sign = val < 0 && c == 'd';
                    uint32_t num = sign ? -val : val;
                    do {
                        buf[--limit] = "0123456789ABCDEF"[num % base];
                        num /= base;
                    } while (num != 0);
                    if (sign) {
                        if (fill == ' ')
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
        int result = veprintf(fmt, ap);
        va_end(ap);
        return result;
    }

    void (*fun) (void*, uint8_t const*, int);
    void* arg;
private:
    uint32_t count;
    int8_t fill, width, base, next, limit;
    uint8_t buf [44];

    void flush () {
        auto len = next;
        count += len;
        next = 0;
        fun(arg, buf, len);
    }

    void emit (int c) {
        if (next >= limit)
            flush();
        buf[next++] = c;
    }

    void putFiller (int n) {
        while (--n >= 0)
            emit(fill);
    }
};
