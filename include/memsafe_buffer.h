#ifndef MEMSAFE_BUFFER_H
#define MEMSAFE_BUFFER_H
#include <stdio.h>

#define MEMSAFE_STRING(name, str) \
    static const char name[] = str;


size_t safe_strlen(const char *str, size_t max_len) ;
size_t safe_append(char *buffer, size_t buffer_size, const char *append_str);
uint8_t reset_buffer(uint8_t *buffer, uint8_t len);

MEMSAFE_STRING(str_hal_ok, "HAL_OK\r");
MEMSAFE_STRING(str_hal_error, "HAL_ERROR\r");
MEMSAFE_STRING(str_hal_busy, "HAL_BUSY\r");
MEMSAFE_STRING(str_hal_timeout, "HAL_TIMEOUT\r");
MEMSAFE_STRING(i2c_connect, "connected\r");
MEMSAFE_STRING(i2c_not_connect, "not connected\r");





#endif // MEMSAFE_BUFFER_H