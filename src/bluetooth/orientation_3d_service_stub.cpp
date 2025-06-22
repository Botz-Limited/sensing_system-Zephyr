/**
 * @file orientation_3d_service_stub.cpp
 * @brief Stub implementation of 3D orientation service for secondary devices
 * @version 1.0
 * @date 2024-12-20
 *
 * @copyright Botz Innovation 2024
 */

#include "orientation_3d_service.hpp"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(bluetooth, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)

// Stub implementations for secondary devices
void orientation_3d_update_primary(float quat_w, float quat_x, float quat_y, float quat_z)
{
    // No-op on secondary device
    LOG_DBG("orientation_3d_update_primary called on secondary device (no-op)");
}

void orientation_3d_update_secondary(float quat_w, float quat_x, float quat_y, float quat_z)
{
    // No-op on secondary device
    LOG_DBG("orientation_3d_update_secondary called on secondary device (no-op)");
}

void orientation_3d_try_send(void)
{
    // No-op on secondary device
}

const char* orientation_3d_get_service_uuid_str(void)
{
    return "";
}

#endif // !CONFIG_PRIMARY_DEVICE