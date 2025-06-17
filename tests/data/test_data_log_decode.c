/*
 * Test for Data Module: Open and decode the latest log file
 *
 * This test locates the latest foot_*.pb log file in /lfs1/hardware,
 * decodes it using nanopb, and prints the decoded content.
 */

#include <zephyr/ztest.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <stdio.h>
#include <string.h>
#include <pb_decode.h>
#include "foot_sensor_messages.pb.h"

#define LOG_DIR "/lfs1/hardware"
#define LOG_PREFIX "foot_"
#define LOG_EXT ".pb"

static int find_latest_log_file(char *out_path, size_t out_path_len) {
    struct fs_dir_t dirp;
    static struct fs_dirent entry;
    uint8_t max_id = 0;
    char latest_name[64] = {0};
    fs_dir_t_init(&dirp);
    int res = fs_opendir(&dirp, LOG_DIR);
    if (res != 0) {
        return -1;
    }
    while (fs_readdir(&dirp, &entry) == 0 && entry.name[0] != '\0') {
        size_t name_len = strlen(entry.name);
        size_t prefix_len = strlen(LOG_PREFIX);
        size_t ext_len = strlen(LOG_EXT);
        if (entry.type == FS_DIR_ENTRY_FILE &&
            name_len > (prefix_len + ext_len) &&
            strncmp(entry.name, LOG_PREFIX, prefix_len) == 0 &&
            strcmp(entry.name + name_len - ext_len, LOG_EXT) == 0) {
            uint8_t id = 0;
            if (sscanf(entry.name + prefix_len, "%hhu.pb", &id) == 1) {
                if (id > max_id) {
                    max_id = id;
                    strncpy(latest_name, entry.name, sizeof(latest_name) - 1);
                }
            }
        }
    }
    fs_closedir(&dirp);
    if (max_id == 0) {
        return -2;
    }
    snprintf(out_path, out_path_len, "%s/%s", LOG_DIR, latest_name);
    return 0;
}

ZTEST(data_log, test_decode_latest_foot_log) {
    char file_path[128];
    int res = find_latest_log_file(file_path, sizeof(file_path));
    zassert_equal(res, 0, "No log file found");

    struct fs_file_t file;
    fs_file_t_init(&file);
    res = fs_open(&file, file_path, FS_O_READ);
    zassert_equal(res, 0, "Failed to open log file");

    uint8_t buffer[2048];
    ssize_t read_len = fs_read(&file, buffer, sizeof(buffer));
    zassert_true(read_len > 0, "Failed to read log file");

    fs_close(&file);

    pb_istream_t stream = pb_istream_from_buffer(buffer, read_len);
    sensor_data_messages_FootSensorLogMessage msg = sensor_data_messages_FootSensorLogMessage_init_default;
    bool status = pb_decode(&stream, sensor_data_messages_FootSensorLogMessage_fields, &msg);
    zassert_true(status, "Failed to decode protobuf");

    printk("Decoded log file: %s\n", file_path);
    printk("which_payload: %d\n", msg.which_payload);
    // Print more fields as needed
}

ZTEST_SUITE(data_log, NULL, NULL, NULL, NULL, NULL);
