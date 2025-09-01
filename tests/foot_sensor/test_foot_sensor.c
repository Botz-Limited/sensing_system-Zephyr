#include <zephyr/ztest.h>

// Example function to test (replace with real foot_sensor logic)
int foot_sensor_add(int a, int b) {
    return a + b;
}

ZTEST(foot_sensor, test_add_positive) {
    zassert_equal(foot_sensor_add(2, 3), 5, "2 + 3 should be 5");
}

ZTEST(foot_sensor, test_add_negative) {
    zassert_equal(foot_sensor_add(-2, -3), -5, "-2 + -3 should be -5");
}

ZTEST_SUITE(foot_sensor, NULL, NULL, NULL, NULL, NULL);
