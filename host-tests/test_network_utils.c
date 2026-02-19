#include <stdio.h>
#include <string.h>

#include "network_utils.h"

static int test_pass_through_printable(void)
{
    const char *input = "Telemetry 123";
    char out[64];
    size_t n = network_sanitize_payload(input, strlen(input), out, sizeof(out));

    return (n == strlen("Telemetry 123") && strcmp(out, "Telemetry 123") == 0) ? 0 : 1;
}

static int test_remove_newlines(void)
{
    const char *input = "Line1\r\nLine2\n";
    char out[64];
    size_t n = network_sanitize_payload(input, strlen(input), out, sizeof(out));

    return (n == strlen("Line1Line2") && strcmp(out, "Line1Line2") == 0) ? 0 : 1;
}

static int test_remove_non_printable(void)
{
    const char input[] = { 'A', 0x01, 'B', 0x7F, 'C', '\0' };
    char out[16];
    size_t n = network_sanitize_payload(input, 5, out, sizeof(out));

    return (n == 3 && strcmp(out, "ABC") == 0) ? 0 : 1;
}

static int test_output_buffer_limit(void)
{
    const char *input = "123456789";
    char out[5];
    size_t n = network_sanitize_payload(input, strlen(input), out, sizeof(out));

    return (n == 4 && strcmp(out, "1234") == 0) ? 0 : 1;
}

static int test_null_input(void)
{
    char out[8];
    size_t n = network_sanitize_payload(NULL, 0, out, sizeof(out));

    return (n == 0 && strcmp(out, "") == 0) ? 0 : 1;
}

int main(void)
{
    int failed = 0;

    failed += test_pass_through_printable();
    failed += test_remove_newlines();
    failed += test_remove_non_printable();
    failed += test_output_buffer_limit();
    failed += test_null_input();

    if (failed != 0) {
        fprintf(stderr, "network_utils_tests failed: %d test(s)\n", failed);
        return 1;
    }

    printf("network_utils_tests passed\n");
    return 0;
}
