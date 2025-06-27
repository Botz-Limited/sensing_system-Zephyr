/**
 * @file fota_fix.hpp
 * @brief FOTA fix utilities header
 */

#ifndef FOTA_FIX_HPP
#define FOTA_FIX_HPP

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Check and confirm MCUBoot image
 * @return 0 on success, negative error code on failure
 */
int fota_check_and_confirm_image(void);

/**
 * @brief Request upgrade and reset
 * @param force Force reset even if upgrade request fails
 * @return Does not return on success
 */
int fota_request_upgrade_and_reset(bool force);

/**
 * @brief Print MCUBoot slot information
 */
void fota_print_slot_info(void);

/**
 * @brief Verify external flash is accessible
 * @return 0 on success, negative error code on failure
 */
int fota_verify_external_flash(void);

/**
 * @brief Initialize FOTA fixes
 * @return 0 on success, negative error code on failure
 */
int fota_fix_init(void);

#ifdef __cplusplus
}
#endif

#endif /* FOTA_FIX_HPP */