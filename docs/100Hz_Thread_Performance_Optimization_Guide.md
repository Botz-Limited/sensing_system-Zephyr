# 80Hz Thread Performance Optimization Guide

**Version:** 1.1  
**Date:** July 2025  
**Purpose:** Comprehensive guide for optimizing the sensor_data thread to maintain reliable 80Hz operation

> **Note:** This guide has been updated to reflect the actual implementation running at 80Hz, not 100Hz as originally designed.

---

## Table of Contents

1. [Performance Requirements](#performance-requirements)
2. [nRF5340 Hardware Capabilities](#nrf5340-hardware-capabilities)
3. [Optimization Strategies](#optimization-strategies)
4. [DSP Instructions and CMSIS-DSP](#dsp-instructions-and-cmsis-dsp)
5. [Fixed-Point vs Floating-Point Trade-offs](#fixed-point-vs-floating-point-trade-offs)
6. [Assembly Language Optimization](#assembly-language-optimization)
7. [Algorithm Design Patterns](#algorithm-design-patterns)
8. [Profiling and Measurement](#profiling-and-measurement)
9. [Implementation Guidelines](#implementation-guidelines)

---

## Performance Requirements

### 80Hz Thread Constraints

The sensor_data thread must process all sensor data within strict timing constraints:

- **Period**: 12.5ms (80Hz)
- **Target Processing Time**: <2.5ms per cycle
- **Maximum Processing Time**: 6ms (48% duty cycle)
- **Jitter Tolerance**: ±0.5ms
- **Stack Usage**: <2KB
- **Interrupt Latency**: <10μs

### Processing Tasks (Per Cycle)

1. Read 16 pressure values (8 per foot)
2. Read BHI360 data (quaternion, accel, gyro, steps)
3. Detect ground contact/flight phases
4. Calculate peak forces
5. Compute pressure distribution
6. Validate sensor data
7. Timestamp with microsecond precision
8. Send consolidated data to queues

---

## nRF5340 Hardware Capabilities

### ARM Cortex-M33 Features

The nRF5340 application core provides powerful hardware features:

```
- Clock Speed: 128 MHz
- Architecture: ARMv8-M Mainline
- FPU: Single-precision (float) hardware
- DSP Extensions: Yes (SIMD instructions)
- Cache: 8KB 2-way set associative
- MPU: Memory Protection Unit
- TrustZone: Security extensions
```

### Key Performance Features

1. **Hardware FPU**
   - Single-precision operations: 1-3 cycles
   - Double-precision: Software emulated (20-100+ cycles)
   - Supports IEEE 754 standard

2. **DSP Instructions**
   - SIMD operations on 16-bit data
   - Saturating arithmetic
   - Parallel add/subtract
   - Multiply-accumulate (MAC)

3. **Instruction Pipeline**
   - 3-stage pipeline
   - Branch prediction
   - Instruction prefetch

---

## Optimization Strategies

### 1. Algorithm Complexity Reduction

```c
// BAD: Complex calculation in hot path
float calculate_complex_metric(float data[100]) {
    float result = 0;
    for (int i = 0; i < 100; i++) {
        result += sqrt(data[i]) * sin(data[i]);  // Very expensive!
    }
    return result;
}

// GOOD: Simplified approximation
float calculate_simple_metric(float data[100]) {
    float result = 0;
    for (int i = 0; i < 100; i++) {
        result += data[i] * 1.5f;  // Good enough approximation
    }
    return result;
}
```

### 2. Memory Access Optimization

```c
// Align data structures for efficient access
typedef struct __attribute__((aligned(4))) {
    uint16_t pressure[8];      // 16 bytes
    float quaternion[4];       // 16 bytes
    float accel[3];           // 12 bytes
    float gyro[3];            // 12 bytes
    uint32_t timestamp;       // 4 bytes
    uint32_t step_count;      // 4 bytes
} sensor_data_t;  // Total: 64 bytes (cache-friendly)

// Use local copies to avoid repeated memory access
void process_pressure_fast(uint16_t pressure[8]) {
    // Copy to local variables for repeated access
    uint16_t p0 = pressure[0], p1 = pressure[1];
    uint16_t p2 = pressure[2], p3 = pressure[3];
    uint16_t p4 = pressure[4], p5 = pressure[5];
    uint16_t p6 = pressure[6], p7 = pressure[7];
    
    // Now use local variables (in registers)
    uint32_t heel = p0 + p1;
    uint32_t mid = p2 + p3 + p4;
    uint32_t fore = p5 + p6 + p7;
}
```

### 3. Loop Optimization

```c
// Loop unrolling for known sizes
static inline uint32_t sum_8_channels(uint16_t pressure[8]) {
    // Fully unrolled - no loop overhead
    return pressure[0] + pressure[1] + pressure[2] + pressure[3] +
           pressure[4] + pressure[5] + pressure[6] + pressure[7];
}

// Duff's device for variable-length loops
void process_samples(float *data, int count) {
    int n = (count + 7) / 8;
    switch (count % 8) {
        case 0: do { process(*data++);
        case 7:      process(*data++);
        case 6:      process(*data++);
        case 5:      process(*data++);
        case 4:      process(*data++);
        case 3:      process(*data++);
        case 2:      process(*data++);
        case 1:      process(*data++);
                } while (--n > 0);
    }
}
```

### 4. Compiler Optimization Directives

```c
// Function attributes for optimization
#define ALWAYS_INLINE __attribute__((always_inline)) static inline
#define HOT_FUNCTION __attribute__((hot))
#define COLD_FUNCTION __attribute__((cold))
#define PURE_FUNCTION __attribute__((pure))
#define CONST_FUNCTION __attribute__((const))
#define RESTRICT __restrict

// Branch prediction hints
#define LIKELY(x) __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)

// Example usage
HOT_FUNCTION ALWAYS_INLINE
bool detect_contact(uint16_t pressure_sum) {
    if (LIKELY(pressure_sum > CONTACT_THRESHOLD)) {
        return true;
    }
    return false;
}
```

---

## DSP Instructions and CMSIS-DSP

### Using CMSIS-DSP Library

The CMSIS-DSP library provides optimized functions that use DSP instructions:

```c
#include <arm_math.h>

// Fast vector operations
void process_with_dsp(void) {
    // Vector addition
    float32_t vec1[8], vec2[8], result[8];
    arm_add_f32(vec1, vec2, result, 8);  // Uses SIMD when possible
    
    // Dot product
    float32_t dot_result;
    arm_dot_prod_f32(vec1, vec2, 8, &dot_result);
    
    // RMS calculation
    float32_t rms;
    arm_rms_f32(vec1, 8, &rms);
    
    // Fast math approximations
    float32_t sqrt_result;
    arm_sqrt_f32(100.0f, &sqrt_result);  // Optimized square root
}

// Q15 fixed-point operations (even faster)
void process_fixed_point_dsp(void) {
    q15_t data[8], result[8];
    
    // Saturating addition
    arm_add_q15(data, data, result, 8);
    
    // Fast correlation
    q15_t correlation[15];
    arm_correlate_q15(data, 8, data, 8, correlation);
}
```

### Manual DSP Intrinsics

```c
#include <arm_acle.h>

// Parallel 16-bit operations
uint32_t fast_dual_add(uint32_t a, uint32_t b) {
    return __UADD16(a, b);  // Add two 16-bit values in parallel
}

// Saturating arithmetic
int32_t saturating_add(int32_t a, int32_t b) {
    return __QADD(a, b);  // Saturates instead of overflowing
}

// Multiply-accumulate
int32_t fast_mac(int32_t acc, int16_t a, int16_t b) {
    return __SMLABB(acc, a, b);  // acc + (a * b)
}

// Count leading zeros (useful for normalization)
uint32_t normalize_value(uint32_t value) {
    int shift = __CLZ(value);  // Count leading zeros
    return value << shift;      // Normalize to MSB
}
```

---

## Fixed-Point vs Floating-Point Trade-offs

### When to Use Hardware FPU (Float)

```c
// GOOD: Use float when precision is needed
typedef struct {
    float quaternion[4];      // Orientation needs precision
    float gyro_rad_s[3];      // Angular velocity
    float accel_m_s2[3];      // Linear acceleration
} imu_data_t;

// Fast float operations with FPU
float calculate_magnitude_fpu(float x, float y, float z) {
    return sqrtf(x*x + y*y + z*z);  // 10-15 cycles with FPU
}
```

### When to Use Fixed-Point

```c
// Fixed-point for simple calculations
typedef int16_t fixed16_t;  // Q8.8 format
#define FIXED_ONE 256
#define FIXED_SHIFT 8

// Convert float to fixed
fixed16_t float_to_fixed(float value) {
    return (fixed16_t)(value * FIXED_ONE);
}

// Fixed-point multiplication
fixed16_t fixed_mul(fixed16_t a, fixed16_t b) {
    return (fixed16_t)(((int32_t)a * b) >> FIXED_SHIFT);
}

// Example: Pressure percentage calculation
void calculate_pressure_percentages_fixed(uint16_t pressure[8], 
                                         uint8_t percentages[3]) {
    uint32_t heel = pressure[0] + pressure[1];
    uint32_t mid = pressure[2] + pressure[3] + pressure[4];
    uint32_t fore = pressure[5] + pressure[6] + pressure[7];
    uint32_t total = heel + mid + fore;
    
    if (total > 0) {
        // Use fixed-point to avoid division
        percentages[0] = (heel * 100) / total;
        percentages[1] = (mid * 100) / total;
        percentages[2] = (fore * 100) / total;
    }
}
```

### Hybrid Approach Recommendation

```c
// Use float for complex math, convert to fixed for transmission
typedef struct {
    // Internal processing - use float with FPU
    float quaternion_internal[4];
    float accel_internal[3];
    
    // For transmission - use fixed-point
    int16_t quaternion_fixed[4];  // Q15 format
    int16_t accel_fixed[3];       // Scaled integers
} hybrid_data_t;

// Process with float, transmit as fixed
void process_and_prepare(hybrid_data_t *data) {
    // Complex calculations with FPU
    float magnitude = sqrtf(data->accel_internal[0] * data->accel_internal[0] +
                           data->accel_internal[1] * data->accel_internal[1] +
                           data->accel_internal[2] * data->accel_internal[2]);
    
    // Convert to fixed for transmission
    for (int i = 0; i < 3; i++) {
        data->accel_fixed[i] = (int16_t)(data->accel_internal[i] * 1000.0f);
    }
}
```

---

## Assembly Language Optimization

### When Assembly Makes Sense

1. **Tight loops** with simple operations
2. **Bit manipulation** requiring specific instructions
3. **SIMD operations** not exposed by intrinsics
4. **Critical timing** requirements

### Assembly Examples for nRF5340

```c
// Fast 8-channel sum using SIMD
__attribute__((naked))
uint32_t sum_8_channels_asm(uint16_t *channels) {
    __asm volatile (
        "push {r4-r5}              \n\t"  // Save registers
        "ldmia r0!, {r1-r4}        \n\t"  // Load 8 values (4 regs)
        "uadd16 r1, r1, r2         \n\t"  // Add pairs in parallel
        "uadd16 r3, r3, r4         \n\t"  // Add pairs in parallel
        "uadd16 r1, r1, r3         \n\t"  // Combine results
        "uxtah r0, r1, r1, ror #16 \n\t"  // Extract and add halves
        "pop {r4-r5}               \n\t"  // Restore registers
        "bx lr                     \n\t"  // Return
        ::: "r0", "r1", "r2", "r3", "r4", "memory"
    );
}

// Ultra-fast threshold detection with hysteresis
static inline bool threshold_detect_asm(uint16_t value, 
                                       uint16_t threshold,
                                       bool previous_state) {
    bool result;
    uint16_t hyst = 50;  // Hysteresis value
    
    __asm volatile (
        "cmp %3, #0                \n\t"  // Check previous state
        "ite eq                    \n\t"  // If-then-else
        "addeq %2, %2, %4          \n\t"  // Add hysteresis if was low
        "subne %2, %2, %4          \n\t"  // Subtract if was high
        "cmp %1, %2                \n\t"  // Compare with threshold
        "ite gt                    \n\t"  // If greater than
        "movgt %0, #1              \n\t"  // Set result true
        "movle %0, #0              \n\t"  // Set result false
        : "=r" (result)
        : "r" (value), "r" (threshold), "r" (previous_state), "r" (hyst)
        : "cc"
    );
    
    return result;
}

// Fast center of pressure using MAC instructions
void calculate_cop_asm(uint16_t pressure[8], 
                      const int8_t x_pos[8],
                      const int8_t y_pos[8],
                      int16_t *cop_x, 
                      int16_t *cop_y) {
    __asm volatile (
        "push {r4-r11}             \n\t"  // Save registers
        "mov r4, #0                \n\t"  // x accumulator
        "mov r5, #0                \n\t"  // y accumulator
        "mov r6, #0                \n\t"  // total pressure
        "mov r7, #8                \n\t"  // loop counter
        
        "1:                        \n\t"  // Loop label
        "ldrh r8, [%0], #2         \n\t"  // Load pressure[i]
        "ldrsb r9, [%1], #1        \n\t"  // Load x_pos[i]
        "ldrsb r10, [%2], #1       \n\t"  // Load y_pos[i]
        
        "mla r4, r8, r9, r4        \n\t"  // x_acc += pressure * x_pos
        "mla r5, r8, r10, r5       \n\t"  // y_acc += pressure * y_pos
        "add r6, r6, r8            \n\t"  // total += pressure
        
        "subs r7, r7, #1           \n\t"  // Decrement counter
        "bne 1b                    \n\t"  // Loop if not zero
        
        "cmp r6, #0                \n\t"  // Check for divide by zero
        "itt ne                    \n\t"  // If not zero
        "sdivne r4, r4, r6         \n\t"  // cop_x = x_acc / total
        "sdivne r5, r5, r6         \n\t"  // cop_y = y_acc / total
        
        "strh r4, [%3]             \n\t"  // Store cop_x
        "strh r5, [%4]             \n\t"  // Store cop_y
        
        "pop {r4-r11}              \n\t"  // Restore registers
        :
        : "r" (pressure), "r" (x_pos), "r" (y_pos), 
          "r" (cop_x), "r" (cop_y)
        : "r4", "r5", "r6", "r7", "r8", "r9", "r10", "memory"
    );
}
```

---

## Algorithm Design Patterns

### 1. State Machine Pattern

```c
// Efficient phase detection using state machine
typedef enum {
    PHASE_SWING,
    PHASE_HEEL_STRIKE,
    PHASE_LOADING,
    PHASE_MIDSTANCE,
    PHASE_PUSH_OFF
} gait_phase_t;

typedef struct {
    gait_phase_t current_phase;
    uint32_t phase_start_time;
    uint16_t phase_threshold[5];
} phase_detector_t;

// Fast state machine update
static inline gait_phase_t update_phase(phase_detector_t *detector,
                                       uint16_t heel, 
                                       uint16_t mid, 
                                       uint16_t fore) {
    switch (detector->current_phase) {
        case PHASE_SWING:
            if (heel > detector->phase_threshold[0]) {
                detector->current_phase = PHASE_HEEL_STRIKE;
                detector->phase_start_time = k_uptime_get_32();
            }
            break;
            
        case PHASE_HEEL_STRIKE:
            if (mid > detector->phase_threshold[1]) {
                detector->current_phase = PHASE_LOADING;
            }
            break;
            
        // ... other transitions
    }
    
    return detector->current_phase;
}
```

### 2. Incremental Calculation Pattern

```c
// Instead of recalculating everything, update incrementally
typedef struct {
    uint32_t sum;
    uint32_t count;
    float mean;
    float m2;  // For variance calculation
} incremental_stats_t;

// Welford's algorithm for running statistics
static inline void update_stats(incremental_stats_t *stats, float value) {
    stats->count++;
    float delta = value - stats->mean;
    stats->mean += delta / stats->count;
    float delta2 = value - stats->mean;
    stats->m2 += delta * delta2;
}

static inline float get_variance(incremental_stats_t *stats) {
    return (stats->count > 1) ? stats->m2 / (stats->count - 1) : 0.0f;
}
```

### 3. Lookup Table Pattern

```c
// Pre-compute expensive calculations
static const uint8_t sqrt_lut[256] = {
    0, 16, 23, 28, 32, 36, 40, 43, 46, 48, 51, 53, 55, 57, 59, 61,
    // ... rest of table
};

// Fast approximate square root
static inline uint8_t fast_sqrt(uint16_t value) {
    if (value < 256) {
        return sqrt_lut[value];
    } else {
        // Scale down and adjust
        uint8_t scaled = value >> 8;
        return sqrt_lut[scaled] << 4;
    }
}

// Trigonometric approximations
static const int16_t sin_lut[91] = {  // 0-90 degrees, Q15 format
    0, 572, 1144, 1715, 2286, 2856, 3425, 3993, 4560, 5126,
    // ... rest of table
};

static inline int16_t fast_sin_q15(int16_t angle_deg) {
    angle_deg = angle_deg % 360;
    if (angle_deg < 0) angle_deg += 360;
    
    if (angle_deg <= 90) return sin_lut[angle_deg];
    else if (angle_deg <= 180) return sin_lut[180 - angle_deg];
    else if (angle_deg <= 270) return -sin_lut[angle_deg - 180];
    else return -sin_lut[360 - angle_deg];
}
```

### 4. Bit Manipulation Pattern

```c
// Fast operations using bit tricks
static inline bool is_power_of_2(uint32_t n) {
    return n && !(n & (n - 1));
}

static inline uint32_t next_power_of_2(uint32_t n) {
    n--;
    n |= n >> 1;
    n |= n >> 2;
    n |= n >> 4;
    n |= n >> 8;
    n |= n >> 16;
    return n + 1;
}

// Fast modulo for power-of-2
#define FAST_MOD(x, pow2) ((x) & ((pow2) - 1))

// Efficient flag checking
typedef enum {
    FLAG_CONTACT    = (1 << 0),
    FLAG_HEEL       = (1 << 1),
    FLAG_MIDFOOT    = (1 << 2),
    FLAG_FOREFOOT   = (1 << 3),
    FLAG_PRONATION  = (1 << 4),
} sensor_flags_t;

static inline void set_flags(uint8_t *flags, uint8_t mask) {
    *flags |= mask;
}

static inline bool check_flags(uint8_t flags, uint8_t mask) {
    return (flags & mask) == mask;
}
```

---

## Profiling and Measurement

### Cycle Counter Setup

```c
// Enable DWT cycle counter
void enable_cycle_counter(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// Profiling macros
#define PROFILE_START() uint32_t _start_cycles = DWT->CYCCNT
#define PROFILE_END(name) do { \
    uint32_t _cycles = DWT->CYCCNT - _start_cycles; \
    uint32_t _us = _cycles / 128; \
    LOG_DBG("%s took %u cycles (%u us)", name, _cycles, _us); \
} while(0)

// Usage example
void profile_function(void) {
    PROFILE_START();
    detect_ground_contact(pressure_data);
    PROFILE_END("ground_contact");
}
```

### Performance Monitoring

```c
typedef struct {
    uint32_t min_cycles;
    uint32_t max_cycles;
    uint32_t total_cycles;
    uint32_t count;
    uint32_t overruns;
} perf_stats_t;

static perf_stats_t perf_stats[10];  // For different functions

void update_perf_stats(int func_id, uint32_t cycles) {
    perf_stats_t *stats = &perf_stats[func_id];
    
    if (cycles < stats->min_cycles || stats->min_cycles == 0) {
        stats->min_cycles = cycles;
    }
    if (cycles > stats->max_cycles) {
        stats->max_cycles = cycles;
    }
    
    stats->total_cycles += cycles;
    stats->count++;
    
    // Check for overrun (>2ms = 256,000 cycles)
    if (cycles > 256000) {
        stats->overruns++;
        LOG_WRN("Function %d overrun: %u cycles", func_id, cycles);
    }
}

void print_perf_report(void) {
    for (int i = 0; i < 10; i++) {
        perf_stats_t *stats = &perf_stats[i];
        if (stats->count > 0) {
            uint32_t avg = stats->total_cycles / stats->count;
            LOG_INF("Func %d: min=%u, max=%u, avg=%u cycles, overruns=%u",
                    i, stats->min_cycles, stats->max_cycles, avg, stats->overruns);
        }
    }
}
```

---

## Implementation Guidelines

### 1. Development Process

```
1. Implement in C first
2. Profile to find bottlenecks
3. Optimize algorithms
4. Use DSP/intrinsics
5. Consider assembly only if needed
6. Always measure improvements
```

### 2. Code Organization

```c
// sensor_data_fast_processing.h
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <arm_math.h>

// Configuration
#define USE_DSP_INSTRUCTIONS 1
#define USE_FIXED_POINT 0
#define ENABLE_PROFILING 1

// Fast algorithms
bool detect_ground_contact_fast(uint16_t pressure[8]);
uint16_t calculate_peak_force_fast(uint16_t pressure[8]);
void calculate_pressure_distribution_fast(uint16_t pressure[8], 
                                         uint8_t percentages[3]);
int8_t estimate_pronation_fast(float quaternion[4], 
                              uint8_t pressure_distribution[3]);

// DSP-accelerated versions
#if USE_DSP_INSTRUCTIONS
void process_pressure_dsp(uint16_t pressure[8], 
                         pressure_metrics_t *metrics);
#endif

// Assembly versions (if needed)
#ifdef USE_ASSEMBLY_OPTIMIZATION
extern uint32_t sum_8_channels_asm(uint16_t *channels);
extern void calculate_cop_asm(uint16_t pressure[8], 
                             int16_t *cop_x, int16_t *cop_y);
#endif
```

### 3. Testing Strategy

```c
// Unit tests for optimization
void test_optimization_correctness(void) {
    uint16_t test_pressure[8] = {100, 200, 300, 400, 500, 600, 700, 800};
    
    // Test C version
    uint32_t sum_c = sum_8_channels_c(test_pressure);
    
    // Test DSP version
    uint32_t sum_dsp = sum_8_channels_dsp(test_pressure);
    
    // Test assembly version
    uint32_t sum_asm = sum_8_channels_asm(test_pressure);
    
    // Verify all produce same result
    assert(sum_c == sum_dsp);
    assert(sum_c == sum_asm);
    
    // Benchmark each version
    benchmark_function("C version", sum_8_channels_c, test_pressure);
    benchmark_function("DSP version", sum_8_channels_dsp, test_pressure);
    benchmark_function("ASM version", sum_8_channels_asm, test_pressure);
}
```

### 4. Optimization Checklist

- [ ] Profile baseline performance
- [ ] Identify top 3 bottlenecks
- [ ] Optimize algorithms first
- [ ] Use appropriate data types (float vs fixed)
- [ ] Apply compiler optimizations (-O2 or -O3)
- [ ] Use DSP instructions where applicable
- [ ] Consider lookup tables for complex math
- [ ] Implement state machines for phase detection
- [ ] Use incremental calculations
- [ ] Minimize memory access
- [ ] Align data structures
- [ ] Unroll critical loops
- [ ] Test with real sensor data
- [ ] Verify timing requirements met
- [ ] Document all optimizations

---

## Implementation Status

> **Note:** As of July 2025, the following optimizations are documented but not yet implemented:

### Not Implemented:
- **Performance Profiling**: DWT cycle counter setup and `perf_stats_t` structures
- **CMSIS-DSP Library**: No DSP library usage found in code
- **Assembly Optimizations**: No assembly code implementations
- **DSP Intrinsics**: No manual DSP intrinsic usage

### Currently Implemented:
- **Basic C algorithms** with inline functions
- **80Hz operation** (not 100Hz as originally planned)
- **Simple optimizations** like loop unrolling for 8-channel summation
- **Hardware FPU usage** for float calculations

### Recommendation:
The current C implementation appears sufficient for 80Hz operation. Advanced optimizations should only be implemented if profiling shows performance bottlenecks.

---

## Conclusion

The nRF5340 provides excellent hardware capabilities for 80Hz sensor processing:

1. **Use the FPU** for complex calculations - it's fast!
2. **Use DSP instructions** for parallel operations on sensor arrays (when needed)
3. **Start with C optimization** - often sufficient
4. **Profile everything** - measure before optimizing
5. **Assembly as last resort** - only for proven bottlenecks

The key is to design efficient algorithms from the start, use the hardware features available, and only optimize what actually needs it based on profiling data. The current implementation successfully achieves 80Hz operation without requiring the advanced optimizations documented here.