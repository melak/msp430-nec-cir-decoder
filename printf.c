
/*
 * Kevin Timmerman, 2011
 *
 * http://forum.43oh.com/topic/1289-tiny-printf-c-version/
 */

#include <msp430.h>

#include "printf.h"

void UART_Transmit(char data);

void my_putc(uint8_t byte)
{
	UART_Transmit(byte);
}

void my_puts(const char *s)
{
	while(*s) my_putc(*s++);
}

static const unsigned long dv[] = {
//  4294967296      // 32 bit unsigned max
    1000000000,     // +0
     100000000,     // +1
      10000000,     // +2
       1000000,     // +3
        100000,     // +4
//       65535      // 16 bit unsigned max     
         10000,     // +5
          1000,     // +6
           100,     // +7
            10,     // +8
             1,     // +9
};

static void xtoa(unsigned long x, const unsigned long *dp)
{
    char c;
    unsigned long d;
    if(x) {
        while(x < *dp) ++dp;
        do {
            d = *dp++;
            c = '0';
            while(x >= d) ++c, x -= d;
            my_putc(c);
        } while(!(d & 1));
    } else
        my_putc('0');
}

static void puth(unsigned n)
{
    static const char hex[16] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
    my_putc(hex[n & 15]);
}
 
void my_printf(const char *format, ...)
{
    char c;
    int i;
    long n;
    
    va_list a;
    va_start(a, format);
    while((c = *format++)) {
        if(c == '%') {
            switch(c = *format++) {
                case 's':                       // String
                    my_puts(va_arg(a, char*));
                    break;
                case 'c':                       // Char
                    my_putc(va_arg(a, int));
                    break;
                case 'i':                       // 16 bit Integer
                case 'u':                       // 16 bit Unsigned
                    i = va_arg(a, int);
                    if(c == 'i' && i < 0) i = -i, my_putc('-');
                    xtoa((unsigned)i, dv + 5);
                    break;
                case 'l':                       // 32 bit Long
                case 'n':                       // 32 bit uNsigned loNg
                    n = va_arg(a, long);
                    if(c == 'l' &&  n < 0) n = -n, my_putc('-');
                    xtoa((unsigned long)n, dv);
                    break;
                case 'x':                       // 16 bit heXadecimal
                    i = va_arg(a, int);
                    puth(i >> 12);
                    puth(i >> 8);
                    puth(i >> 4);
                    puth(i);
                    break;
                case 'X':
                    i = va_arg(a, int);
                    puth(i >> 4);
		    puth(i);
                    break;
                case 0: return;
                default: goto bad_fmt;
            }
        } else
bad_fmt:    my_putc(c);
    }
    va_end(a);
}

