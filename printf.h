/*
 * Kevin Timmerman, 2011
 *
 * http://forum.43oh.com/topic/1289-tiny-printf-c-version/
 */

#include <stdarg.h>
#include <stdint.h>

void uart_initialize(void);
void my_putc(uint8_t byte);
void my_puts(const char *s);
void my_printf(const char *format, ...);


