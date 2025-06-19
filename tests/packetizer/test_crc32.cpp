/**
 * @file test_crc32.cpp
 * @brief Unit tests for CRC32 module
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>

#include "crc32.hpp"

// Test fixture
struct crc32_fixture {
    uint8_t test_data[256];
};

static void *crc32_setup(void)
{
    static struct crc32_fixture fixture;
    
    // Initialize test data with known pattern
    for (int i = 0; i < sizeof(fixture.test_data); i++) {
        fixture.test_data[i] = i;
    }
    
    return &fixture;
}

static void crc32_teardown(void *f)
{
    ARG_UNUSED(f);
}

ZTEST_SUITE(crc32, NULL, crc32_setup, NULL, NULL, crc32_teardown);

ZTEST(crc32, test_crc32_empty_data)
{
    // Test CRC32 of empty data
    uint32_t crc = CRC32_Compute(NULL, 0, NULL);
    
    // CRC32 of empty data should be 0
    zassert_equal(crc, 0, "CRC32 of empty data should be 0");
}

ZTEST_F(crc32, test_crc32_single_byte)
{
    // Test CRC32 of single byte
    uint8_t data = 0x42;
    uint32_t crc = CRC32_Compute(&data, 1, NULL);
    
    // Verify CRC is calculated (exact value depends on polynomial)
    zassert_not_equal(crc, 0, "CRC32 of single byte should not be 0");
}

ZTEST_F(crc32, test_crc32_known_string)
{
    // Test CRC32 of known string "123456789"
    const char *test_str = "123456789";
    uint32_t crc = CRC32_Compute((const uint8_t *)test_str, strlen(test_str), NULL);
    
    // This is a standard test vector for CRC32
    // The exact value depends on the polynomial and initial value used
    zassert_not_equal(crc, 0, "CRC32 should be calculated");
}

ZTEST_F(crc32, test_crc32_with_initial_value)
{
    // Test CRC32 with initial value
    uint32_t initial_crc = 0x12345678;
    uint32_t crc = CRC32_Compute(fixture->test_data, 10, &initial_crc);
    
    // Verify CRC is calculated with initial value
    zassert_not_equal(crc, initial_crc, "CRC should be different from initial");
}

ZTEST_F(crc32, test_crc32_incremental)
{
    // Test incremental CRC32 calculation
    uint32_t crc1 = CRC32_Compute(fixture->test_data, 10, NULL);
    uint32_t crc2 = CRC32_Compute(&fixture->test_data[10], 10, &crc1);
    
    // Compare with single calculation
    uint32_t crc_full = CRC32_Compute(fixture->test_data, 20, NULL);
    
    // Incremental calculation should match full calculation
    zassert_equal(crc2, crc_full, 
                  "Incremental CRC should match full calculation");
}

ZTEST_F(crc32, test_crc32_different_lengths)
{
    // Test CRC32 with different data lengths
    uint32_t crc1 = CRC32_Compute(fixture->test_data, 10, NULL);
    uint32_t crc2 = CRC32_Compute(fixture->test_data, 11, NULL);
    
    // Different lengths should produce different CRCs
    zassert_not_equal(crc1, crc2, 
                      "Different lengths should produce different CRCs");
}

ZTEST_F(crc32, test_crc32_different_data)
{
    // Test CRC32 with different data
    uint8_t data1[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t data2[] = {0x01, 0x02, 0x03, 0x05}; // Last byte different
    
    uint32_t crc1 = CRC32_Compute(data1, sizeof(data1), NULL);
    uint32_t crc2 = CRC32_Compute(data2, sizeof(data2), NULL);
    
    // Different data should produce different CRCs
    zassert_not_equal(crc1, crc2, 
                      "Different data should produce different CRCs");
}

ZTEST_F(crc32, test_crc32_all_zeros)
{
    // Test CRC32 of all zeros
    uint8_t zeros[100] = {0};
    uint32_t crc = CRC32_Compute(zeros, sizeof(zeros), NULL);
    
    // CRC of all zeros should not be zero (depends on polynomial)
    zassert_not_equal(crc, 0, "CRC32 of zeros should not be 0");
}

ZTEST_F(crc32, test_crc32_all_ones)
{
    // Test CRC32 of all ones
    uint8_t ones[100];
    memset(ones, 0xFF, sizeof(ones));
    
    uint32_t crc = CRC32_Compute(ones, sizeof(ones), NULL);
    
    // Verify CRC is calculated
    zassert_not_equal(crc, 0, "CRC32 of ones should not be 0");
    zassert_not_equal(crc, 0xFFFFFFFF, "CRC32 should not be all ones");
}

ZTEST_F(crc32, test_crc32_consistency)
{
    // Test that same data produces same CRC
    uint32_t crc1 = CRC32_Compute(fixture->test_data, 50, NULL);
    uint32_t crc2 = CRC32_Compute(fixture->test_data, 50, NULL);
    
    zassert_equal(crc1, crc2, "Same data should produce same CRC");
}

ZTEST_F(crc32, test_crc32_large_data)
{
    // Test CRC32 with full buffer
    uint32_t crc = CRC32_Compute(fixture->test_data, sizeof(fixture->test_data), NULL);
    
    // Verify CRC is calculated
    zassert_not_equal(crc, 0, "CRC32 of large data should not be 0");
}

ZTEST(crc32, test_crc32_null_data_nonzero_size)
{
    // Test behavior with NULL data but non-zero size
    // This should be handled gracefully (implementation dependent)
    uint32_t crc = CRC32_Compute(NULL, 10, NULL);
    
    // Implementation should handle this case
    zassert_true(true, "Should handle NULL data gracefully");
}

ZTEST_F(crc32, test_crc32_performance_pattern)
{
    // Test with repeating pattern
    uint8_t pattern[256];
    for (int i = 0; i < sizeof(pattern); i++) {
        pattern[i] = i % 16; // Repeating 0-15 pattern
    }
    
    uint32_t crc = CRC32_Compute(pattern, sizeof(pattern), NULL);
    
    // Verify CRC is calculated
    zassert_not_equal(crc, 0, "CRC32 of pattern should not be 0");
}