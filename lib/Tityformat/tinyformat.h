#ifndef TINFMT_H
#define TINFMT_H

#include <stdarg.h>
#include <string.h>
#include <math.h>

// Простейший аналог snprintf для базовых типов
static int tinfmt_format(char* buf, int size, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    
    char* p = buf;
    const char* fmt_ptr = fmt;
    int count = 0;
    
    while (*fmt_ptr && count < size - 1) {
        if (*fmt_ptr == '%' && *(fmt_ptr + 1)) {
            fmt_ptr++;
            switch (*fmt_ptr) {
                case 'd': { // целое
                    int val = va_arg(args, int);
                    int len = 0;
                    char temp[20];
                    
                    if (val == 0) {
                        temp[len++] = '0';
                    } else {
                        if (val < 0) {
                            if (count < size - 1) buf[count++] = '-';
                            val = -val;
                        }
                        while (val > 0) {
                            temp[len++] = '0' + (val % 10);
                            val /= 10;
                        }
                        for (int i = len - 1; i >= 0; i--) {
                            if (count < size - 1) buf[count++] = temp[i];
                        }
                    }
                    break;
                }
                case 'f': { // float с ограниченной точностью
                    float val = (float)va_arg(args, double);
                    // Простейшая конвертация (2 знака после запятой)
                    int int_part = (int)val;
                    int frac_part = (int)(fabsf(val - int_part) * 100);
                    
                    // Целая часть
                    char temp[20];
                    int len = 0;
                    int ip = abs(int_part);
                    
                    do {
                        temp[len++] = '0' + (ip % 10);
                        ip /= 10;
                    } while (ip > 0);
                    
                    if (int_part < 0) {
                        if (count < size - 1) buf[count++] = '-';
                    }
                    
                    for (int i = len - 1; i >= 0; i--) {
                        if (count < size - 1) buf[count++] = temp[i];
                    }
                    
                    // Дробная часть
                    if (count < size - 1) buf[count++] = '.';
                    if (count < size - 1) buf[count++] = '0' + (frac_part / 10);
                    if (count < size - 1) buf[count++] = '0' + (frac_part % 10);
                    break;
                }
                case 's': { // строка
                    char* s = va_arg(args, char*);
                    while (*s && count < size - 1) {
                        buf[count++] = *s++;
                    }
                    break;
                }
                case '%': { // символ процента
                    if (count < size - 1) buf[count++] = '%';
                    break;
                }
            }
            fmt_ptr++;
        } else {
            if (count < size - 1) buf[count++] = *fmt_ptr++;
        }
    }
    
    buf[count] = '\0';
    va_end(args);
    return count;
}

#endif // TINFMT_H