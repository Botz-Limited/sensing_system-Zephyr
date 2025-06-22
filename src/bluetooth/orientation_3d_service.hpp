/**
 * @file orientation_3d_service.hpp
 * @brief High-rate 3D orientation service for real-time visualization
 * @version 1.0
 * @date 2024-12-20
 *
 * @copyright Botz Innovation 2024
 */

#ifndef ORIENTATION_3D_SERVICE_HPP
#define ORIENTATION_3D_SERVICE_HPP

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Update quaternion data for primary shoe
 * 
 * Called from motion sensor at high rate (50Hz)
 * 
 * @param quat_w Quaternion W component
 * @param quat_x Quaternion X component
 * @param quat_y Quaternion Y component
 * @param quat_z Quaternion Z component
 */
void orientation_3d_update_primary(float quat_w, float quat_x, float quat_y, float quat_z);

/**
 * @brief Update quaternion data from secondary shoe
 * 
 * Called when D2D data is received from secondary
 * 
 * @param quat_w Quaternion W component
 * @param quat_x Quaternion X component
 * @param quat_y Quaternion Y component
 * @param quat_z Quaternion Z component
 */
void orientation_3d_update_secondary(float quat_w, float quat_x, float quat_y, float quat_z);

/**
 * @brief Try to send orientation packet if conditions are met
 */
void orientation_3d_try_send(void);

/**
 * @brief Get the service UUID string for advertising
 */
const char* orientation_3d_get_service_uuid_str(void);

#ifdef __cplusplus
}
#endif

#endif // ORIENTATION_3D_SERVICE_HPP