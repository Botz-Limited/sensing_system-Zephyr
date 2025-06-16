#include <zephyr/ztest.h>

// Example function to test (replace with your own)
int add(int a, int b) {
    return a + b;
}

ZTEST(sample_math, test_add_positive) {
    zassert_equal(add(2, 3), 5, "2 + 3 should be 5");
}

ZTEST(sample_math, test_add_negative) {
    zassert_equal(add(-2, -3), -5, "-2 + -3 should be -5");
}

ZTEST_SUITE(sample_math, NULL, NULL, NULL, NULL, NULL);
