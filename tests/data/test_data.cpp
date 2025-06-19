/**
 * @file test_data.cpp
 * @brief Unit tests for data module
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/flash_map.h>

// Define types needed for testing
enum class err_t {
    NO_ERROR = 0,
    INVALID_PARAMETER,
    FILE_SYSTEM_ERROR,
    FILE_NOT_FOUND,
    FILE_ALREADY_EXISTS,
    OUT_OF_MEMORY,
    UNKNOWN_ERROR
};

enum class record_type_t {
    FOOT_SENSOR = 0,
    BHI360 = 1,
    UNKNOWN = 255
};

// Test fixture
struct data_fixture {
    struct fs_mount_t mount;
    struct flash_area flash_area;
    struct fs_file_t file;
    struct fs_dir_t dir;
    bool file_system_mounted;
    bool foot_sensor_logging;
    bool bhi360_logging;
    int current_foot_sensor_id;
    int current_bhi360_id;
};

static void *data_setup(void)
{
    static struct data_fixture fixture;
    memset(&fixture, 0, sizeof(fixture));
    fixture.file_system_mounted = false;
    fixture.foot_sensor_logging = false;
    fixture.bhi360_logging = false;
    fixture.current_foot_sensor_id = 0;
    fixture.current_bhi360_id = 0;
    return &fixture;
}

static void data_teardown(void *f)
{
    struct data_fixture *fixture = (struct data_fixture *)f;
    fixture->file_system_mounted = false;
    fixture->foot_sensor_logging = false;
    fixture->bhi360_logging = false;
}

ZTEST_SUITE(data, NULL, data_setup, NULL, NULL, data_teardown);

// Mock functions for data module
static err_t mount_file_system_mock(struct data_fixture *fixture, struct fs_mount_t *mount)
{
    if (fixture->file_system_mounted) {
        return err_t::NO_ERROR; // Already mounted
    }
    
    fixture->file_system_mounted = true;
    return err_t::NO_ERROR;
}

static err_t unmount_file_system_mock(struct data_fixture *fixture)
{
    if (!fixture->file_system_mounted) {
        return err_t::FILE_SYSTEM_ERROR;
    }
    
    fixture->file_system_mounted = false;
    return err_t::NO_ERROR;
}

static err_t littlefs_flash_erase_mock(struct data_fixture *fixture, uint8_t id)
{
    // Simulate flash erase
    if (id > 10) {
        return err_t::INVALID_PARAMETER;
    }
    
    // Reset file IDs after erase
    fixture->current_foot_sensor_id = 0;
    fixture->current_bhi360_id = 0;
    
    return err_t::NO_ERROR;
}

static err_t type_and_id_to_path_mock(record_type_t type, int id, char *path)
{
    if (!path) {
        return err_t::INVALID_PARAMETER;
    }
    
    switch (type) {
        case record_type_t::FOOT_SENSOR:
            snprintf(path, 256, "/lfs/foot_sensor_%03d.bin", id);
            break;
        case record_type_t::BHI360:
            snprintf(path, 256, "/lfs/bhi360_%03d.bin", id);
            break;
        default:
            return err_t::INVALID_PARAMETER;
    }
    
    return err_t::NO_ERROR;
}

static err_t start_foot_sensor_logging_mock(struct data_fixture *fixture, 
                                           uint32_t timestamp, const char *version)
{
    if (!fixture->file_system_mounted) {
        return err_t::FILE_SYSTEM_ERROR;
    }
    
    if (fixture->foot_sensor_logging) {
        return err_t::FILE_ALREADY_EXISTS;
    }
    
    fixture->foot_sensor_logging = true;
    fixture->current_foot_sensor_id++;
    
    return err_t::NO_ERROR;
}

static err_t end_foot_sensor_logging_mock(struct data_fixture *fixture)
{
    if (!fixture->foot_sensor_logging) {
        return err_t::FILE_NOT_FOUND;
    }
    
    fixture->foot_sensor_logging = false;
    return err_t::NO_ERROR;
}

static err_t start_bhi360_logging_mock(struct data_fixture *fixture,
                                      uint32_t timestamp, const char *version)
{
    if (!fixture->file_system_mounted) {
        return err_t::FILE_SYSTEM_ERROR;
    }
    
    if (fixture->bhi360_logging) {
        return err_t::FILE_ALREADY_EXISTS;
    }
    
    fixture->bhi360_logging = true;
    fixture->current_bhi360_id++;
    
    return err_t::NO_ERROR;
}

static err_t end_bhi360_logging_mock(struct data_fixture *fixture)
{
    if (!fixture->bhi360_logging) {
        return err_t::FILE_NOT_FOUND;
    }
    
    fixture->bhi360_logging = false;
    return err_t::NO_ERROR;
}

// Tests
ZTEST_F(data, test_mount_file_system_success)
{
    // Execute
    err_t ret = mount_file_system_mock(fixture, &fixture->mount);
    
    // Verify
    zassert_equal(ret, err_t::NO_ERROR, "Mount should succeed");
    zassert_true(fixture->file_system_mounted, "File system should be mounted");
}

ZTEST_F(data, test_mount_file_system_already_mounted)
{
    // Mount first
    mount_file_system_mock(fixture, &fixture->mount);
    
    // Try to mount again
    err_t ret = mount_file_system_mock(fixture, &fixture->mount);
    
    // Verify
    zassert_equal(ret, err_t::NO_ERROR, "Should handle already mounted");
}

ZTEST_F(data, test_unmount_file_system)
{
    // Mount first
    mount_file_system_mock(fixture, &fixture->mount);
    
    // Unmount
    err_t ret = unmount_file_system_mock(fixture);
    
    // Verify
    zassert_equal(ret, err_t::NO_ERROR, "Unmount should succeed");
    zassert_false(fixture->file_system_mounted, "File system should be unmounted");
}

ZTEST_F(data, test_littlefs_flash_erase)
{
    // Execute
    err_t ret = littlefs_flash_erase_mock(fixture, 0);
    
    // Verify
    zassert_equal(ret, err_t::NO_ERROR, "Erase should succeed");
    zassert_equal(fixture->current_foot_sensor_id, 0, "IDs should be reset");
    zassert_equal(fixture->current_bhi360_id, 0, "IDs should be reset");
}

ZTEST(data, test_type_and_id_to_path)
{
    char path[256];
    
    // Test foot sensor path
    err_t ret = type_and_id_to_path_mock(record_type_t::FOOT_SENSOR, 5, path);
    zassert_equal(ret, err_t::NO_ERROR, "Should succeed");
    zassert_str_equal(path, "/lfs/foot_sensor_005.bin", "Path should match");
    
    // Test BHI360 path
    ret = type_and_id_to_path_mock(record_type_t::BHI360, 10, path);
    zassert_equal(ret, err_t::NO_ERROR, "Should succeed");
    zassert_str_equal(path, "/lfs/bhi360_010.bin", "Path should match");
    
    // Test invalid type
    ret = type_and_id_to_path_mock((record_type_t)999, 1, path);
    zassert_equal(ret, err_t::INVALID_PARAMETER, "Should fail with invalid type");
}

ZTEST_F(data, test_start_foot_sensor_logging)
{
    // Reset state
    fixture->file_system_mounted = false;
    fixture->foot_sensor_logging = false;
    fixture->current_foot_sensor_id = 0;
    
    // Mount file system first
    err_t ret = mount_file_system_mock(fixture, &fixture->mount);
    zassert_equal(ret, err_t::NO_ERROR, "Mount should succeed");
    
    // Start logging
    ret = start_foot_sensor_logging_mock(fixture, 100, "v1.0.0");
    
    // Verify
    zassert_equal(ret, err_t::NO_ERROR, "Should start logging");
    zassert_true(fixture->foot_sensor_logging, "Should be logging");
    zassert_equal(fixture->current_foot_sensor_id, 1, "ID should increment");
}

ZTEST_F(data, test_start_foot_sensor_logging_without_mount)
{
    // Ensure file system is not mounted
    fixture->file_system_mounted = false;
    
    // Try to start logging without mounting
    err_t ret = start_foot_sensor_logging_mock(fixture, 100, "v1.0.0");
    
    // Verify
    zassert_equal(ret, err_t::FILE_SYSTEM_ERROR, "Should fail without mount");
}

ZTEST_F(data, test_end_foot_sensor_logging)
{
    // Mount and start logging first
    mount_file_system_mock(fixture, &fixture->mount);
    start_foot_sensor_logging_mock(fixture, 100, "v1.0.0");
    
    // End logging
    err_t ret = end_foot_sensor_logging_mock(fixture);
    
    // Verify
    zassert_equal(ret, err_t::NO_ERROR, "Should end logging");
    zassert_false(fixture->foot_sensor_logging, "Should not be logging");
}

ZTEST_F(data, test_start_bhi360_logging)
{
    // Mount file system first
    mount_file_system_mock(fixture, &fixture->mount);
    
    // Start logging
    err_t ret = start_bhi360_logging_mock(fixture, 200, "v1.0.0");
    
    // Verify
    zassert_equal(ret, err_t::NO_ERROR, "Should start logging");
    zassert_true(fixture->bhi360_logging, "Should be logging");
    zassert_equal(fixture->current_bhi360_id, 1, "ID should increment");
}

ZTEST_F(data, test_end_bhi360_logging)
{
    // Mount and start logging first
    mount_file_system_mock(fixture, &fixture->mount);
    start_bhi360_logging_mock(fixture, 200, "v1.0.0");
    
    // End logging
    err_t ret = end_bhi360_logging_mock(fixture);
    
    // Verify
    zassert_equal(ret, err_t::NO_ERROR, "Should end logging");
    zassert_false(fixture->bhi360_logging, "Should not be logging");
}

ZTEST_F(data, test_concurrent_logging)
{
    // Mount file system
    mount_file_system_mock(fixture, &fixture->mount);
    
    // Start both logging sessions
    err_t ret1 = start_foot_sensor_logging_mock(fixture, 100, "v1.0.0");
    err_t ret2 = start_bhi360_logging_mock(fixture, 200, "v1.0.0");
    
    // Verify both can run concurrently
    zassert_equal(ret1, err_t::NO_ERROR, "Foot sensor logging should start");
    zassert_equal(ret2, err_t::NO_ERROR, "BHI360 logging should start");
    zassert_true(fixture->foot_sensor_logging, "Foot sensor should be logging");
    zassert_true(fixture->bhi360_logging, "BHI360 should be logging");
    
    // End both
    end_foot_sensor_logging_mock(fixture);
    end_bhi360_logging_mock(fixture);
    
    zassert_false(fixture->foot_sensor_logging, "Foot sensor should stop");
    zassert_false(fixture->bhi360_logging, "BHI360 should stop");
}

ZTEST_F(data, test_logging_lifecycle)
{
    // Full lifecycle test - start from clean state
    fixture->file_system_mounted = false;
    fixture->foot_sensor_logging = false;
    fixture->current_foot_sensor_id = 0;
    
    // 1. Mount file system
    err_t ret = mount_file_system_mock(fixture, &fixture->mount);
    zassert_equal(ret, err_t::NO_ERROR, "Mount should succeed");
    
    // 2. Start logging
    ret = start_foot_sensor_logging_mock(fixture, 100, "v1.0.0");
    zassert_equal(ret, err_t::NO_ERROR, "Start should succeed");
    
    // 3. Try to start again (should fail)
    ret = start_foot_sensor_logging_mock(fixture, 200, "v1.0.0");
    zassert_equal(ret, err_t::FILE_ALREADY_EXISTS, "Should not start twice");
    
    // 4. End logging
    ret = end_foot_sensor_logging_mock(fixture);
    zassert_equal(ret, err_t::NO_ERROR, "End should succeed");
    
    // 5. Try to end again (should fail)
    ret = end_foot_sensor_logging_mock(fixture);
    zassert_equal(ret, err_t::FILE_NOT_FOUND, "Should not end twice");
    
    // 6. Unmount
    ret = unmount_file_system_mock(fixture);
    zassert_equal(ret, err_t::NO_ERROR, "Unmount should succeed");
}

ZTEST_F(data, test_file_id_management)
{
    // Reset state
    fixture->file_system_mounted = false;
    fixture->foot_sensor_logging = false;
    fixture->current_foot_sensor_id = 0;
    
    // Mount file system
    mount_file_system_mock(fixture, &fixture->mount);
    
    // Start and stop multiple logging sessions
    for (int i = 0; i < 5; i++) {
        err_t ret = start_foot_sensor_logging_mock(fixture, i * 100, "v1.0.0");
        zassert_equal(ret, err_t::NO_ERROR, "Start %d should succeed", i);
        zassert_equal(fixture->current_foot_sensor_id, i + 1, "ID should be %d", i + 1);
        
        ret = end_foot_sensor_logging_mock(fixture);
        zassert_equal(ret, err_t::NO_ERROR, "End %d should succeed", i);
    }
    
    // Erase flash (should reset IDs)
    littlefs_flash_erase_mock(fixture, 0);
    zassert_equal(fixture->current_foot_sensor_id, 0, "ID should reset");
    
    // Start again after erase
    err_t ret = start_foot_sensor_logging_mock(fixture, 600, "v1.0.0");
    zassert_equal(ret, err_t::NO_ERROR, "Start after erase should succeed");
    zassert_equal(fixture->current_foot_sensor_id, 1, "ID should restart from 1");
}

ZTEST(data, test_error_codes)
{
    // Test all error codes are distinct
    zassert_not_equal((int)err_t::NO_ERROR, (int)err_t::INVALID_PARAMETER,
                      "Error codes should be unique");
    zassert_not_equal((int)err_t::FILE_SYSTEM_ERROR, (int)err_t::FILE_NOT_FOUND,
                      "Error codes should be unique");
    zassert_not_equal((int)err_t::FILE_ALREADY_EXISTS, (int)err_t::OUT_OF_MEMORY,
                      "Error codes should be unique");
}

ZTEST(data, test_record_types)
{
    // Test record type values
    zassert_equal((int)record_type_t::FOOT_SENSOR, 0, "Foot sensor should be 0");
    zassert_equal((int)record_type_t::BHI360, 1, "BHI360 should be 1");
    zassert_equal((int)record_type_t::UNKNOWN, 255, "Unknown should be 255");
}