#include "network_utils.h"

#include <stddef.h>

size_t network_sanitize_payload(const char *input, size_t input_len,
                                char *output, size_t output_size)
{
    size_t i;
    size_t out_len = 0;

    if (output == NULL || output_size == 0) {
        return 0;
    }

    if (input == NULL || input_len == 0) {
        output[0] = '\0';
        return 0;
    }

    for (i = 0; i < input_len && out_len + 1 < output_size; ++i) {
        unsigned char c = (unsigned char)input[i];

        if (c == '\r' || c == '\n') {
            continue;
        }

        if (c < 32 || c > 126) {
            continue;
        }

        output[out_len++] = (char)c;
    }

    output[out_len] = '\0';
    return out_len;
}
