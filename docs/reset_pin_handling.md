# nRF5340 Reset Pin Handling Without Physical Reset Button

## Overview
When designing a PCB with the nRF5340 without a physical reset button, proper handling of the reset pin is crucial to prevent unintended resets and ensure stable operation.

## Hardware Considerations

### 1. Internal Pull-up Resistor
The nRF5340's reset pin (nRESET) has an internal pull-up resistor that can be enabled. This is the simplest solution when no external reset button is present.

### 2. External Pull-up Resistor (Recommended)
For maximum reliability, add an external pull-up resistor on your PCB:
- Value: 10kΩ to 100kΩ (typical: 47kΩ)
- Connect between nRESET pin and VDD
- This ensures the reset pin stays high even if internal pull-up is disabled

### 3. Capacitor for Noise Immunity
Add a small capacitor to filter noise:
- Value: 100nF
- Connect between nRESET pin and GND
- Place close to the microcontroller

### 4. ESD Protection
Consider adding ESD protection if the reset pin is accessible:
- Use a TVS diode rated for the operating voltage
- Connect between nRESET and GND

## Software Configuration

### 1. Device Tree Overlay
The provided `nrf5340dk_nrf5340_cpuapp_pcb_reset.overlay` configures:
- Internal pull-up on the reset pin
- Optional software-controlled reset GPIO for debugging

### 2. Kconfig Options
Important configuration options in your `prj.conf`:

```conf
# Disable automatic reset on fatal errors during development
CONFIG_RESET_ON_FATAL_ERROR=n

# Enable watchdog for production (optional)
CONFIG_WATCHDOG=y
CONFIG_WDT_DISABLE_AT_BOOT=n
```

### 3. Reset Methods Without Physical Button

#### A. Software Reset
```c
#include <zephyr/sys/reboot.h>

// Perform software reset
sys_reboot(SYS_REBOOT_COLD);
```

#### B. Using J-Link/SWD
- Connect J-Link debugger to SWD pins
- Use `nrfjprog --reset` command
- Or use debugger in your IDE

#### C. Power Cycling
- Remove and reapply power to the board
- Ensure proper power-on reset timing

#### D. Watchdog Reset
```c
#include <zephyr/drivers/watchdog.h>

// Configure watchdog for reset capability
const struct device *wdt = DEVICE_DT_GET(DT_NODELABEL(wdt0));
// ... configure and use watchdog
```

## PCB Design Checklist

1. **Pull-up Resistor**: 47kΩ between nRESET and VDD
2. **Bypass Capacitor**: 100nF between nRESET and GND
3. **Test Point**: Add a test point on nRESET for debugging
4. **Keep Traces Short**: Minimize trace length to reduce noise pickup
5. **Ground Plane**: Ensure solid ground plane under reset traces

## Debugging Tips

1. **Monitor Reset Pin**: Use oscilloscope to check for noise or glitches
2. **Check Voltage Levels**: Ensure VDD is stable before reset is released
3. **Power Sequencing**: Verify proper power-up sequence
4. **EMI/EMC**: Check for electromagnetic interference causing resets

## Example Schematic
```
        VDD
         |
        47kΩ
         |
nRESET --+-- 100nF -- GND
         |
    (Test Point)
```

## Production Considerations

1. **Firmware Updates**: Implement OTA or USB DFU for field updates
2. **Factory Programming**: Use SWD interface for initial programming
3. **Reset Monitoring**: Log reset reasons in production firmware
4. **Fail-Safe Mode**: Implement recovery mode accessible via UART/USB

## Troubleshooting

### Symptom: Random Resets
- Check for noise on reset pin
- Verify pull-up resistor value
- Check power supply stability

### Symptom: Device Won't Start
- Verify reset pin is high
- Check power sequencing
- Ensure bootloader is properly flashed

### Symptom: Can't Program Device
- Check SWD connections
- Verify reset pin isn't held low
- Try mass erase command

## References
- nRF5340 Product Specification
- Nordic Semiconductor Reset Pin Guidelines
- Zephyr Reset Subsystem Documentation