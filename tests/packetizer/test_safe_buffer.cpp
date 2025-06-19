/**
 * @file test_safe_buffer.cpp
 * @brief Unit tests for safe_buffer module
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>

extern "C" {
#include "safe_buffer.hpp"
#include "safe_buffer_error.hpp"
}

// Test fixture
struct safe_buffer_fixture {
    uint8_t buffer1[256];
    uint8_t buffer2[256];
    uint32_t length1;
    uint32_t length2;
    safe_buffer_t safe_buf1;
    safe_buffer_t safe_buf2;
};

static void *safe_buffer_setup(void)
{
    static struct safe_buffer_fixture fixture;
    memset(&fixture, 0, sizeof(fixture));
    
    // Initialize safe buffers
    fixture.safe_buf1.buffer = fixture.buffer1;
    fixture.safe_buf1.length = &fixture.length1;
    fixture.safe_buf1.max_length = sizeof(fixture.buffer1);
    fixture.length1 = 0;
    
    fixture.safe_buf2.buffer = fixture.buffer2;
    fixture.safe_buf2.length = &fixture.length2;
    fixture.safe_buf2.max_length = sizeof(fixture.buffer2);
    fixture.length2 = 0;
    
    return &fixture;
}

static void safe_buffer_teardown(void *f)
{
    struct safe_buffer_fixture *fixture = (struct safe_buffer_fixture *)f;
    
    // Clear buffers
    memset(fixture->buffer1, 0, sizeof(fixture->buffer1));
    memset(fixture->buffer2, 0, sizeof(fixture->buffer2));
    fixture->length1 = 0;
    fixture->length2 = 0;
}

ZTEST_SUITE(safe_buffer, NULL, safe_buffer_setup, NULL, NULL, safe_buffer_teardown);

ZTEST_F(safe_buffer, test_safe_buffer_valid_success)
{
    // Test valid buffer
    result_t result = safe_buffer_Valid(&fixture->safe_buf1);
    
    // Verify
    zassert_equal(result, RESULT_OK, "Valid buffer should return OK");
}

ZTEST(safe_buffer, test_safe_buffer_valid_null_target)
{
    // Test NULL target
    result_t result = safe_buffer_Valid(NULL);
    
    // Verify
    zassert_equal(result, RESULT_ERROR, "NULL target should return ERROR");
}

ZTEST_F(safe_buffer, test_safe_buffer_valid_null_buffer)
{
    // Test NULL buffer member
    fixture->safe_buf1.buffer = NULL;
    result_t result = safe_buffer_Valid(&fixture->safe_buf1);
    
    // Verify
    zassert_equal(result, RESULT_ERROR, "NULL buffer should return ERROR");
}

ZTEST_F(safe_buffer, test_safe_buffer_valid_null_length)
{
    // Test NULL length member
    fixture->safe_buf1.length = NULL;
    result_t result = safe_buffer_Valid(&fixture->safe_buf1);
    
    // Verify
    zassert_equal(result, RESULT_ERROR, "NULL length should return ERROR");
}

ZTEST_F(safe_buffer, test_safe_buffer_valid_zero_max_length)
{
    // Test zero max_length
    fixture->safe_buf1.max_length = 0;
    result_t result = safe_buffer_Valid(&fixture->safe_buf1);
    
    // Verify
    zassert_equal(result, RESULT_ERROR, "Zero max_length should return ERROR");
}

ZTEST_F(safe_buffer, test_safe_buffer_copy_success)
{
    // Setup source data
    const char *test_data = "Hello, World!";
    memcpy(fixture->buffer2, test_data, strlen(test_data));
    fixture->length2 = strlen(test_data);
    
    // Execute copy
    result_t result = safe_buffer_Copy(&fixture->safe_buf1, &fixture->safe_buf2, 
                                       fixture->length2);
    
    // Verify
    zassert_equal(result, RESULT_OK, "Copy should succeed");
    zassert_equal(fixture->length1, strlen(test_data), "Length should match");
    zassert_mem_equal(fixture->buffer1, test_data, strlen(test_data), 
                      "Data should be copied");
}

ZTEST_F(safe_buffer, test_safe_buffer_copy_zero_len)
{
    // Setup source data
    const char *test_data = "Test data";
    memcpy(fixture->buffer2, test_data, strlen(test_data));
    fixture->length2 = strlen(test_data);
    
    // Execute copy with len=0 (copy all)
    result_t result = safe_buffer_Copy(&fixture->safe_buf1, &fixture->safe_buf2, 0);
    
    // Verify
    zassert_equal(result, RESULT_OK, "Copy should succeed");
    zassert_equal(fixture->length1, fixture->length2, "Should copy all data");
}

ZTEST_F(safe_buffer, test_safe_buffer_copy_truncate)
{
    // Setup - destination smaller than source
    fixture->safe_buf1.max_length = 5;
    const char *test_data = "Hello, World!";
    memcpy(fixture->buffer2, test_data, strlen(test_data));
    fixture->length2 = strlen(test_data);
    
    // Execute copy with len=0 (should truncate)
    result_t result = safe_buffer_Copy(&fixture->safe_buf1, &fixture->safe_buf2, 0);
    
    // Verify
    zassert_equal(result, RESULT_OK, "Copy should succeed with truncation");
    zassert_equal(fixture->length1, 5, "Should copy only what fits");
}

ZTEST_F(safe_buffer, test_safe_buffer_copy_exact_fit)
{
    // Setup - request exact amount that fits
    const char *test_data = "12345";
    memcpy(fixture->buffer2, test_data, 5);
    fixture->length2 = 5;
    
    // Execute
    result_t result = safe_buffer_Copy(&fixture->safe_buf1, &fixture->safe_buf2, 5);
    
    // Verify
    zassert_equal(result, RESULT_OK, "Copy should succeed");
    zassert_equal(fixture->length1, 5, "Should copy exact amount");
}

ZTEST_F(safe_buffer, test_safe_buffer_copy_too_much)
{
    // Setup - request more than destination can hold
    fixture->safe_buf1.max_length = 5;
    const char *test_data = "Hello, World!";
    memcpy(fixture->buffer2, test_data, strlen(test_data));
    fixture->length2 = strlen(test_data);
    
    // Execute
    result_t result = safe_buffer_Copy(&fixture->safe_buf1, &fixture->safe_buf2, 10);
    
    // Verify
    zassert_equal(result, RESULT_ERROR, "Should fail when requesting too much");
    zassert_equal(fixture->length1, 5, "Should still copy what fits");
}

ZTEST_F(safe_buffer, test_safe_buffer_string_null_copy)
{
    // Setup
    char dest[32];
    const char *src = "Hello, World!";
    
    // Execute
    result_t result = safe_buffer_StringNullCopy(dest, src, sizeof(dest));
    
    // Verify
    zassert_equal(result, RESULT_OK, "String copy should succeed");
    zassert_str_equal(dest, src, "String should be copied");
}

ZTEST_F(safe_buffer, test_safe_buffer_string_null_copy_truncate)
{
    // Setup
    char dest[8];
    const char *src = "Hello, World!"; // Longer than dest
    
    // Execute
    result_t result = safe_buffer_StringNullCopy(dest, src, sizeof(dest));
    
    // Verify
    zassert_equal(result, RESULT_OK, "String copy should succeed");
    zassert_equal(strlen(dest), sizeof(dest) - 1, 
                  "String should be truncated and null terminated");
    zassert_equal(dest[sizeof(dest) - 1], '\0', "Should be null terminated");
}

ZTEST_F(safe_buffer, test_safe_buffer_append_success)
{
    // Setup - initial data in dest
    const char *initial = "Hello";
    memcpy(fixture->buffer1, initial, strlen(initial));
    fixture->length1 = strlen(initial);
    
    // Data to append
    const char *append_data = ", World!";
    memcpy(fixture->buffer2, append_data, strlen(append_data));
    fixture->length2 = strlen(append_data);
    
    // Execute
    result_t result = safe_buffer_Append(&fixture->safe_buf1, &fixture->safe_buf2,
                                        fixture->length2);
    
    // Verify
    zassert_equal(result, RESULT_OK, "Append should succeed");
    zassert_equal(fixture->length1, strlen(initial) + strlen(append_data),
                  "Length should be sum of both");
    zassert_mem_equal(fixture->buffer1, "Hello, World!", 
                      strlen("Hello, World!"), "Data should be appended");
}

ZTEST_F(safe_buffer, test_safe_buffer_append_zero_len)
{
    // Setup
    const char *initial = "Start";
    memcpy(fixture->buffer1, initial, strlen(initial));
    fixture->length1 = strlen(initial);
    
    const char *append_data = "End";
    memcpy(fixture->buffer2, append_data, strlen(append_data));
    fixture->length2 = strlen(append_data);
    
    // Execute with len=0 (append all)
    result_t result = safe_buffer_Append(&fixture->safe_buf1, &fixture->safe_buf2, 0);
    
    // Verify
    zassert_equal(result, RESULT_OK, "Append should succeed");
    zassert_equal(fixture->length1, strlen(initial) + strlen(append_data),
                  "Should append all data");
}

ZTEST_F(safe_buffer, test_safe_buffer_append_overflow)
{
    // Setup - fill destination almost full
    fixture->safe_buf1.max_length = 10;
    memset(fixture->buffer1, 'A', 8);
    fixture->length1 = 8;
    
    // Try to append more than fits
    memset(fixture->buffer2, 'B', 5);
    fixture->length2 = 5;
    
    // Execute
    result_t result = safe_buffer_Append(&fixture->safe_buf1, &fixture->safe_buf2, 5);
    
    // Verify
    zassert_equal(result, RESULT_ERROR, "Should fail when would overflow");
    zassert_equal(fixture->length1, 8, "Original length should be unchanged");
}

ZTEST_F(safe_buffer, test_safe_buffer_append_exact_fit)
{
    // Setup
    fixture->safe_buf1.max_length = 10;
    memset(fixture->buffer1, 'A', 5);
    fixture->length1 = 5;
    
    memset(fixture->buffer2, 'B', 5);
    fixture->length2 = 5;
    
    // Execute
    result_t result = safe_buffer_Append(&fixture->safe_buf1, &fixture->safe_buf2, 5);
    
    // Verify
    zassert_equal(result, RESULT_OK, "Should succeed with exact fit");
    zassert_equal(fixture->length1, 10, "Should use full buffer");
}

ZTEST_F(safe_buffer, test_safe_buffer_invalid_operations)
{
    // Test operations with invalid buffers
    safe_buffer_t invalid_buf = {0};
    
    // Copy with invalid destination
    result_t result = safe_buffer_Copy(&invalid_buf, &fixture->safe_buf2, 10);
    zassert_equal(result, RESULT_ERROR, "Should fail with invalid dest");
    
    // Copy with invalid source
    result = safe_buffer_Copy(&fixture->safe_buf1, &invalid_buf, 10);
    zassert_equal(result, RESULT_ERROR, "Should fail with invalid source");
    
    // Append with invalid buffers
    result = safe_buffer_Append(&invalid_buf, &fixture->safe_buf2, 10);
    zassert_equal(result, RESULT_ERROR, "Should fail with invalid dest");
    
    result = safe_buffer_Append(&fixture->safe_buf1, &invalid_buf, 10);
    zassert_equal(result, RESULT_ERROR, "Should fail with invalid source");
}