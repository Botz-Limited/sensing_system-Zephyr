/**
 * @file fota_fix.cpp
 * @brief FOTA fix utilities for nRF5340
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/storage/flash_map.h>

LOG_MODULE_REGISTER(fota_fix, LOG_LEVEL_INF);

/**
 * @brief Check and confirm MCUBoot image
 * @return 0 on success, negative error code on failure
 */
int fota_check_and_confirm_image(void)
{
#ifdef CONFIG_BOOTLOADER_MCUBOOT
    struct mcuboot_img_header header;
    int rc;

    LOG_INF("Checking MCUBoot image status...");

    // Check if we're running a test image
    if (!boot_is_img_confirmed()) {
        LOG_INF("Image is not confirmed, checking if it's valid...");
        
        // Get image header to verify
        rc = boot_read_bank_header(FIXED_PARTITION_ID(slot0_partition), 
                                   &header, sizeof(header));
        if (rc != 0) {
            LOG_ERR("Failed to read image header: %d", rc);
            return rc;
        }

        // Basic sanity check
        if (header.mcuboot_version == 0 || header.image_size == 0) {
            LOG_ERR("Invalid image header");
            return -EINVAL;
        }

        LOG_INF("Image appears valid, confirming...");
        
        // Confirm the image
        rc = boot_write_img_confirmed();
        if (rc != 0) {
            LOG_ERR("Failed to confirm image: %d", rc);
            return rc;
        }

        LOG_INF("Image confirmed successfully!");
        
        // For multi-image systems, we might need to confirm other slots too
        #ifdef CONFIG_NRF53_MULTI_IMAGE_UPDATE
        LOG_INF("Checking network core image...");
        // Network core confirmation is handled automatically by MCUBoot
        #endif
    } else {
        LOG_INF("Image already confirmed");
    }

    return 0;
#else
    LOG_WRN("MCUBoot not enabled");
    return -ENOTSUP;
#endif
}

/**
 * @brief Request upgrade and reset
 * @param force Force reset even if upgrade request fails
 * @return Does not return on success
 */
int fota_request_upgrade_and_reset(bool force)
{
#ifdef CONFIG_BOOTLOADER_MCUBOOT
    int rc;

    LOG_INF("Requesting upgrade...");
    
    rc = boot_request_upgrade(BOOT_UPGRADE_TEST);
    if (rc != 0) {
        LOG_ERR("Failed to request upgrade: %d", rc);
        if (!force) {
            return rc;
        }
        LOG_WRN("Forcing reset anyway...");
    }

    LOG_INF("Rebooting in 1 second...");
    k_sleep(K_SECONDS(1));
    
    sys_reboot(SYS_REBOOT_WARM);
    
    // Should never reach here
    return -EFAULT;
#else
    LOG_WRN("MCUBoot not enabled");
    return -ENOTSUP;
#endif
}

/**
 * @brief Get MCUBoot slot info
 */
void fota_print_slot_info(void)
{
#ifdef CONFIG_BOOTLOADER_MCUBOOT
    struct mcuboot_img_header header;
    const struct flash_area *fa;
    int rc;

    LOG_INF("=== MCUBoot Slot Information ===");

    // Check slot 0 (primary)
    rc = flash_area_open(FIXED_PARTITION_ID(slot0_partition), &fa);
    if (rc == 0) {
        rc = flash_area_read(fa, 0, &header, sizeof(header));
        if (rc == 0) {
            LOG_INF("Slot 0: size=%u, version=%u.%u.%u+%u", 
                    header.image_size,
                    header.h.v1.sem_ver.major,
                    header.h.v1.sem_ver.minor,
                    header.h.v1.sem_ver.revision,
                    header.h.v1.sem_ver.build_num);
        }
        flash_area_close(fa);
    }

    // Check slot 1 (secondary) 
    rc = flash_area_open(FIXED_PARTITION_ID(slot1_partition), &fa);
    if (rc == 0) {
        rc = flash_area_read(fa, 0, &header, sizeof(header));
        if (rc == 0 && header.mcuboot_version != 0) {
            LOG_INF("Slot 1: size=%u, version=%u.%u.%u+%u", 
                    header.image_size,
                    header.h.v1.sem_ver.major,
                    header.h.v1.sem_ver.minor,
                    header.h.v1.sem_ver.revision,
                    header.h.v1.sem_ver.build_num);
        } else {
            LOG_INF("Slot 1: empty or invalid");
        }
        flash_area_close(fa);
    }

    LOG_INF("Image confirmed: %s", boot_is_img_confirmed() ? "YES" : "NO");
#endif
}

/**
 * @brief Verify external flash is accessible
 */
int fota_verify_external_flash(void)
{
    const struct flash_area *fa;
    uint8_t test_pattern[256];
    uint8_t read_buffer[256];
    int rc;

    LOG_INF("Verifying external flash...");

    // Try to open the secondary slot (should be in external flash)
    rc = flash_area_open(FIXED_PARTITION_ID(slot1_partition), &fa);
    if (rc != 0) {
        LOG_ERR("Failed to open secondary slot: %d", rc);
        return rc;
    }

    // Generate test pattern
    for (int i = 0; i < sizeof(test_pattern); i++) {
        test_pattern[i] = i & 0xFF;
    }

    // Try to write and read back
    rc = flash_area_erase(fa, 0, sizeof(test_pattern));
    if (rc != 0) {
        LOG_ERR("Failed to erase external flash: %d", rc);
        flash_area_close(fa);
        return rc;
    }

    rc = flash_area_write(fa, 0, test_pattern, sizeof(test_pattern));
    if (rc != 0) {
        LOG_ERR("Failed to write to external flash: %d", rc);
        flash_area_close(fa);
        return rc;
    }

    rc = flash_area_read(fa, 0, read_buffer, sizeof(read_buffer));
    if (rc != 0) {
        LOG_ERR("Failed to read from external flash: %d", rc);
        flash_area_close(fa);
        return rc;
    }

    // Verify data
    if (memcmp(test_pattern, read_buffer, sizeof(test_pattern)) != 0) {
        LOG_ERR("External flash data verification failed!");
        flash_area_close(fa);
        return -EIO;
    }

    LOG_INF("External flash verified successfully");
    flash_area_close(fa);
    return 0;
}

/**
 * @brief Initialize FOTA fixes
 */
int fota_fix_init(void)
{
    int rc;

    LOG_INF("Initializing FOTA fixes...");

    // Print slot information
    fota_print_slot_info();

    // Check and confirm image if needed
    rc = fota_check_and_confirm_image();
    if (rc != 0) {
        LOG_ERR("Failed to check/confirm image: %d", rc);
        // Don't fail init, just log the error
    }

    // Verify external flash on primary device
    #ifdef CONFIG_PRIMARY_DEVICE
    rc = fota_verify_external_flash();
    if (rc != 0) {
        LOG_ERR("External flash verification failed: %d", rc);
        LOG_ERR("FOTA updates may not work properly!");
    }
    #endif

    return 0;
}