#ifndef MEMSAFE_BUFFER_H
#define MEMSAFE_BUFFER_H
#include <stdio.h>

#define MEMSAFE_STRING(name, str) \
    static const char name[] = str;


MEMSAFE_STRING(str_hal_ok, "HAL_OK\r");
MEMSAFE_STRING(str_hal_error, "HAL_ERROR\r");
MEMSAFE_STRING(str_hal_busy, "HAL_BUSY\r");
MEMSAFE_STRING(str_hal_timeout, "HAL_TIMEOUT\r");
MEMSAFE_STRING(i2c_connect, "connected\r");
MEMSAFE_STRING(i2c_not_connect, "not connected\r");
MEMSAFE_STRING(init_step, "init\r");
MEMSAFE_STRING(dbg_loop, "loop\r");
MEMSAFE_STRING(isr_loop, "isr_loop\r");
MEMSAFE_STRING(isr_error, "isr_error\r");
MEMSAFE_STRING(Audio_error, "Audio_error\r");
MEMSAFE_STRING(half_tranfer, "half_tranfer\r");
MEMSAFE_STRING(full_transfer, "full_transfer\r");
MEMSAFE_STRING(error_transfer, "error_transfer\r");
MEMSAFE_STRING(ok_transfer, "ok_transfer\r");

#define DBG_STRING(name) \
    CDC_Transmit_FS((uint8_t *)name, sizeof(name));

#define DBG_STATUS(hal_status) \
if(hal_status==HAL_OK) \
	{ \
		DBG_STRING(str_hal_ok)\
	} \
    else \
    { \
        DBG_STRING(str_hal_error)\
    }


size_t safe_strlen(const char *str, size_t max_len) ;
size_t safe_append(char *buffer, size_t buffer_size, const char *append_str);
uint8_t reset_buffer(uint8_t *buffer, uint8_t len);








#endif // MEMSAFE_BUFFER_H