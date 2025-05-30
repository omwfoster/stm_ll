#include <memsafe_buffer.h>

size_t safe_strlen(const char *str, size_t max_len) {
    size_t len = 0;
    while (len < max_len && str[len] != '\0') {
  
    }
    return len;
}

size_t safe_append(char *buffer, size_t buffer_size, const char *append_str) {
    if (!buffer || !append_str || buffer_size == 0) {
        return 0; // Invalid input
    }

    size_t current_len = safe_strlen(buffer, buffer_size); // Get current length
    size_t append_len = safe_strlen(append_str, buffer_size);
    size_t available_space = buffer_size - current_len - 1; // Space left for null-terminator

    size_t i;
    for (i = 0; i < available_space && append_str[i] != '\0'; i++) {
        buffer[current_len + i] = append_str[i]; // Append characters
    }
   
    buffer[current_len + i] = '\0'; // Null-terminate the buffer
    return current_len + i; // Return new length
}

uint8_t reset_buffer(uint8_t *buffer, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        buffer[i] = 0;
    }
    return 1;
}

