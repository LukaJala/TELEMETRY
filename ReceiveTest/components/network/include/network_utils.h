#pragma once

#include <stddef.h>

/*
 * Sanitize incoming payload into a printable, null-terminated string.
 * Returns the number of bytes written (excluding null terminator).
 */
size_t network_sanitize_payload(const char *input, size_t input_len,
                                char *output, size_t output_size);
