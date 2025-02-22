#ifndef MEMSAFE_BUFFER_H
#define MEMSAFE_BUFFER_H
#include <stdio.h>


size_t safe_strlen(const char *str, size_t max_len) ;
size_t safe_append(char *buffer, size_t buffer_size, const char *append_str);
uint8_t reset_buffer(uint8_t *buffer, uint8_t len);

static const char example[] = "This is a constant string";


#endif // MEMSAFE_BUFFER_H