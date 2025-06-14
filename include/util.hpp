/**
 * @file util.hpp
 * @brief
 * @version 1.0
 * @date 15/5/2025
 *
 * @copyright Botz Innovation 2025
 *
 */
 
#ifndef APP_INCLUDE_UTIL_H_
#define APP_INCLUDE_UTIL_H_

#include <cstddef>
#include <type_traits>
#include <utility>

#include <zephyr/debug/stack.h>
#include <zephyr/debug/thread_analyzer.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

namespace util
{
constexpr std::size_t max_path_length = CONFIG_MAX_PATH_LEN;
constexpr int post_turn_off_assert_sleep_s = 1;

#ifndef UTIL_MAX_INTERNAL_PATH_LENGTH
#define UTIL_MAX_INTERNAL_PATH_LENGTH 64 // or whatever is safe for your filesystem
#endif

inline void turn_off_then_assert_on_failure(const gpio_dt_spec *sleep_enable_pin)
{
    gpio_pin_set_dt(sleep_enable_pin, 1);
    k_sleep(K_SECONDS(util::post_turn_off_assert_sleep_s));
    __ASSERT(false, "Alive after turn off");
}

} // namespace util

enum gpio_set_value_t
{
    GPIO_INACTIVE,
    GPIO_ACTIVE
};

// Get value of a enum
template <typename E> constexpr typename std::underlying_type<E>::type to_underlying(E e)
{
    return static_cast<typename std::underlying_type<E>::type>(e);
}

#ifdef __cplusplus
extern "C"
{
#endif
/**
 * @brief Collect and log thread statistics, including stack usage and CPU utilization.
 *
 * This function gathers and logs various statistics for a given thread, such as its stack usage
 * and CPU utilization. It retrieves the thread's name, stack size, unused stack space, and runtime
 * execution cycles. If the necessary configuration options are enabled, it also calculates and
 * logs the CPU utilization for the thread.
 *
 * @param thread A pointer to the thread whose statistics are to be collected.
 *
 * @note This function is only available when CONFIG_THREAD_ANALYZER is defined.
 */
static inline void thread_statistics(const struct k_thread *thread)
{
// Thread statistics
#if defined(CONFIG_THREAD_ANALYZER)
    // Iterate over all threads and log stack usage information
    size_t size = thread->stack_info.size;
    k_thread_runtime_stats_t rt_stats_all;
    struct thread_analyzer_info info = {};
    k_thread_runtime_stats_t rt_stats_thread;
    unsigned int pcnt = 0;
    int ret = 0;
    const uint8_t exec_factor = 100U;
    size_t unused;

    if (k_thread_stack_space_get(thread, &unused) == 0)
    {
        pcnt = ((size - unused) * exec_factor) / size;
        info.name = k_thread_name_get((k_tid_t)thread);
        if (info.name == NULL)
        {
            info.name = "unknown";
        }

    }

    // Get the runtime stats for the specific thread
    if (k_thread_runtime_stats_get((k_tid_t)thread, &rt_stats_thread) != 0)
    {
        ret++;
    }

    // Get the runtime stats for all threads
    if (k_thread_runtime_stats_all_get(&rt_stats_all) != 0)
    {
        ret++;
    }

    // Calculate CPU utilization if both stats retrievals were successful
    if (ret == 0)
    {
        info.utilization = (rt_stats_thread.execution_cycles * 100U) / rt_stats_all.execution_cycles;
        printk("execution_cycles of the thread *100 :%lld,  all thread execution_cycles : %lld\n",
               rt_stats_thread.execution_cycles * exec_factor, rt_stats_all.execution_cycles);
    }
    printk("Thread Name: %s \tunused: %zu\tusage size: %zu \t Size: %zu (%u %%) \tCPU Utilization: %u%% \n", info.name, unused,
           size - unused, size, pcnt,info.utilization);
#endif
}

#ifdef __cplusplus
}
#endif
#endif
