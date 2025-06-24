
# Pinout and Peripheral Assignment Reference

**Version:** 1.0  
**Date:** June 2025  
**Scope:** Pin assignments and peripheral mapping for nRF5340 DK and custom hardware  
**Purpose:** Reference guide for hardware connections and peripheral configuration

---

## Table of Contents

1. [Analog Inputs (Foot Sensor, SAADC)](#analog-inputs-foot-sensor-saadc)
2. [SPI1 (Motion Sensor, BHI360)](#spi1-motion-sensor-bhi360)
3. [SPI4 (WiFi nRF7002)](#spi4-wifi-nrf7002)
4. [I2C1](#i2c1)
5. [UARTs](#uarts)
6. [QSPI (External Flash)](#qspi-external-flash)
7. [Other Notable Pins](#other-notable-pins)
8. [Notes](#notes)

---

## 1. Analog Inputs (Foot Sensor, SAADC)

**Note:** The nRF5340 has 8 analog input channels (AIN0-AIN7). All 8 channels are functional on the nRF5340 DK and can be sampled. The standard Arduino header exposes AIN0-AIN5 through labeled pins. AIN6 and AIN7 are available but require specific pin configuration.

| Channel | nRF SAADC Input | Physical Pin | DK Usage | Custom PCB Usage |
|---------|-----------------|--------------|----------|------------------|
| 0       | AIN0            | P0.04        | Arduino A0 | Foot Sensor |
| 1       | AIN1            | P0.05        | Arduino A1 | Foot Sensor |
| 2       | AIN2            | P0.06        | Arduino A2 | Foot Sensor |
| 3       | AIN3            | P0.07        | Arduino A3 | Foot Sensor |
| 4       | AIN4            | P0.25        | Arduino A4 | Foot Sensor |
| 5       | AIN5            | P0.26        | Arduino A5 | Foot Sensor |
| 6       | AIN6            | P0.27        | Available (not on Arduino header) | Foot Sensor |
| 7       | AIN7            | P0.28        | Available (shared with LED0) | Foot Sensor |

**Verified:** All 8 analog channels (AIN0-AIN7) are confirmed working on the nRF5340 DK hardware.

---

## 2. SPI2 (Motion Sensor, BHI360)

**Note:** Using SPI2 instead of SPI1 to avoid conflicts with analog inputs. No pin remapping needed.

| Signal | Pin (Port, Number) | Usage/Comment                    |
|--------|--------------------|---------------------------------|
| SCK    | P1.09              | SPI2 SCK                        |
| MOSI   | P1.08              | SPI2 MOSI                       |
| MISO   | P1.07              | SPI2 MISO                       |
| CS     | P1.06              | SPI2 CS (BHI360)                |
| INT    | P1.02              | BHI360 INT                      |
| RESET  | P1.03              | BHI360 RESET (optional)         |

---

## 3. SPI4 (WiFi nRF7002 - Lab Version Only)

| Signal      | Pin (Port, Number) | Usage/Comment                    |
|-------------|--------------------|---------------------------------|
| SCK         | P1.15              | SPI4 SCK                        |
| MOSI        | P1.13              | SPI4 MOSI                       |
| MISO        | P1.14              | SPI4 MISO                       |
| CS          | P1.12              | SPI4 CS (nRF7002 chip select)   |
| HOST_IRQ    | P1.7               | nRF7002 interrupt to host       |
| IOVDD_CTRL  | P1.8               | nRF7002 IO power control        |
| BUCKEN      | P1.9               | nRF7002 buck enable             |

**Note:** The nRF7002 WiFi companion chip is only included in lab version builds. These pins can be repurposed in production hardware.

---

## 4. I2C1

| Signal | Pin (Port, Number) | Usage/Comment         |
|--------|--------------------|-----------------------|
| SDA    | P1.2               | I2C1 SDA (currently disabled, pin used for BHI360 INT) |
| SCL    | P1.3               | I2C1 SCL (currently disabled, pin used for BHI360 RESET) |

**Note:** I2C1 is disabled in the current configuration. P1.2 and P1.3 are repurposed for BHI360 interrupt and reset signals.

**I2C Availability:** All I2C interfaces (I2C0, I2C1, I2C2, I2C3) are available and can be mapped to any available GPIOs on the custom board.

---



## 5. UARTs

### UART0
| Signal | Pin (Port, Number) | Usage/Comment         |
|--------|--------------------|-----------------------|
| TX     | P0.20              | UART0 TX              |
| RX     | P0.22              | UART0 RX              |
| RTS    | P0.19              | UART0 RTS             |
| CTS    | P0.21              | UART0 CTS             |

### UART1
| Signal | Pin (Port, Number) | Usage/Comment         |
|--------|--------------------|-----------------------|
| TX     | P1.1               | UART1 TX              |
| RX     | P1.0               | UART1 RX              |
| RTS    | P0.11              | UART1 RTS             |
| CTS    | P0.10              | UART1 CTS             |

### UART2
| Signal | Pin (Port, Number) | Usage/Comment         |
|--------|--------------------|-----------------------|
| TX     | P1.11              | UART2 TX              |
| RX     | P1.10              | UART2 RX              |

**Note:**
- Some UARTs share pins with other peripherals (SPI, I2C, etc.) and cannot be used simultaneously with those peripherals due to hardware limitations on the nRF5340. Always check the nRF5340 documentation and the device tree for conflicts before enabling multiple UARTs or overlapping peripherals.

---

## 6. QSPI (External Flash)

| Signal | Pin (Port, Number) | Usage/Comment         |
|--------|--------------------|-----------------------|
| SCK    | P0.17              | QSPI SCK              |
| CSN    | P0.18              | QSPI CSN              |
| IO0    | P0.13              | QSPI IO0              |
| IO1    | P0.14              | QSPI IO1              |
| IO2    | P0.15              | QSPI IO2              |
| IO3    | P0.16              | QSPI IO3              |

---

## 7. Other Notable Pins (DK Only - Available on Custom PCB)

**Note:** These pins are used for LEDs and buttons on the nRF5340 DK but will be **available for other uses on the custom PCB** since we won't have LEDs or buttons.

| Function (DK)    | Pin (Port, Number) | Custom PCB Availability |
|------------------|--------------------|-----------------------|
| Green LED 0      | P0.28              | ⚠️ Used by AIN7 (Foot Sensor) |
| Green LED 1      | P0.29              | ✅ Available for reuse |
| Green LED 2      | P0.30              | ✅ Available for reuse |
| Green LED 3      | P0.31              | ✅ Available for reuse |
| Button 0         | P0.23              | ✅ Available for reuse |
| Button 1         | P0.24              | ✅ Available for reuse |
| Button 2         | P0.08              | ✅ Available for reuse |
| Button 3         | P0.09              | ✅ Available for reuse |

### Summary of Available GPIOs on Custom PCB:
- **P0.08, P0.09, P0.23, P0.24** - 4 GPIOs freed from buttons
- **P0.29, P0.30, P0.31** - 3 GPIOs freed from LEDs
- **P0.27** - Available (when not used as AIN6)
- **Total: 7-8 available GPIOs** for:
  - Additional sensor interrupts
  - Multiplexer control lines
  - Debug/test points
  - Future expansion

---

## 8. Notes

### Key Design Decisions:
1. **Using SPI2 for BHI360** to avoid conflicts with analog inputs:
   - SPI2 uses P1.07-P1.09 (no conflicts)
   - SPI1 would conflict with AIN2, AIN3, AIN4
   - No device tree changes needed

2. **All 8 analog channels available for foot sensors**:
   - AIN0-AIN7 are all functional on nRF5340
   - AIN0-AIN5: P0.04-P0.07, P0.25-P0.26
   - AIN6: P0.27
   - AIN7: P0.28 (shared with LED0 on DK)
   - No pin conflicts with current configuration

3. **Available GPIOs on custom PCB**:
   - P0.08, P0.09, P0.23, P0.24 (freed from buttons)
   - P0.29, P0.30, P0.31 (freed from LEDs)
   - P0.27 (when not used as AIN6)
   - Total: 7-8 additional GPIOs

4. **I2C Availability:** 
   - I2C1 is disabled (P1.2/P1.3 repurposed for BHI360)
   - All I2C interfaces (I2C0-I2C3) are available for use

5. **Multiplexer Support:**
   - If adding analog multiplexers for more sensors, use the freed GPIOs for select lines
   - P0.23, P0.24, P0.29-P0.31 would be good choices for mux control

### Device Tree Configuration:
```dts
/* No changes needed for SPI2 - already configured correctly */
/* ADC configuration for all 8 channels */
&adc {
    status = "okay";
    #address-cells = <1>;
    #size-cells = <0>;
    
    /* All 8 channels are available on nRF5340 */
    channel@0 {
        reg = <0>;
        zephyr,gain = "ADC_GAIN_1_6";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
        zephyr,input-positive = <NRF_SAADC_AIN0>;
        zephyr,resolution = <12>;
    };
    
    channel@1 {
        reg = <1>;
        zephyr,gain = "ADC_GAIN_1_6";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
        zephyr,input-positive = <NRF_SAADC_AIN1>;
        zephyr,resolution = <12>;
    };
    
    /* ... channels 2-5 similar ... */
    
    channel@6 {
        reg = <6>;
        zephyr,gain = "ADC_GAIN_1_6";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
        zephyr,input-positive = <NRF_SAADC_AIN6>;
        zephyr,resolution = <12>;
    };
    
    channel@7 {
        reg = <7>;
        zephyr,gain = "ADC_GAIN_1_6";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
        zephyr,input-positive = <NRF_SAADC_AIN7>;
        zephyr,resolution = <12>;
    };
};
```

---

**End of Reference**
