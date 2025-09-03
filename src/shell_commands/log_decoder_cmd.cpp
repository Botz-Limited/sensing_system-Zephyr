/**
 * @file log_decoder_cmd.cpp
 * @brief Shell commands for decoding sensor log files
 * 
 * This provides shell commands to decode and display log files directly on the device.
 * Commands:
 *   - log list: List all log files
 *   - log decode foot: Decode latest foot sensor log
 *   - log decode bhi360: Decode latest BHI360 log
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <cstring>
#include <cstdio>

LOG_MODULE_REGISTER(log_decoder_cmd, LOG_LEVEL_WRN);

// Simple log file analysis without protobuf decoding
static int analyze_log_file(const struct shell *sh, const char *path)
{
    struct fs_file_t file;
    struct fs_dirent info;
    uint8_t buffer[64];
    int packet_count = 0;
    
    fs_file_t_init(&file);
    
    // Get file info
    int ret = fs_stat(path, &info);
    if (ret != 0) {
        shell_error(sh, "Cannot stat file: %d", ret);
        return ret;
    }
    
    shell_print(sh, "File: %s", path);
    shell_print(sh, "Size: %zu bytes", info.size);
    
    // Open file
    ret = fs_open(&file, path, FS_O_READ);
    if (ret != 0) {
        shell_error(sh, "Cannot open file: %d", ret);
        return ret;
    }
    
    // Read first few bytes to identify content
    ssize_t bytes = fs_read(&file, buffer, sizeof(buffer));
    if (bytes > 0) {
        shell_print(sh, "First %d bytes (hex):", (int)bytes);
        for (int i = 0; i < bytes && i < 32; i++) {
            if (i % 16 == 0) shell_fprintf(sh, SHELL_NORMAL, "\n  ");
            shell_fprintf(sh, SHELL_NORMAL, "%02X ", buffer[i]);
        }
        shell_print(sh, "");
        
        // Try to identify protobuf markers
        bool has_string = false;
        for (int i = 0; i < bytes - 4; i++) {
            // Look for version string pattern (field 1, string type)
            if (buffer[i] == 0x0A) { // Field 1, wire type 2 (string)
                int len = buffer[i+1];
                if (len > 0 && len < 32 && i + 2 + len <= bytes) {
                    shell_print(sh, "Possible version string at offset %d: %.*s", 
                               i, len, &buffer[i+2]);
                    has_string = true;
                }
            }
        }
    }
    
    // Scan file for packet boundaries
    fs_seek(&file, 0, FS_SEEK_SET);
    size_t offset = 0;
    int delta_sum = 0;
    int delta_count = 0;
    
    while (true) {
        bytes = fs_read(&file, buffer, sizeof(buffer));
        if (bytes <= 0) break;
        
        // Look for delta_ms patterns (small values after field tags)
        for (int i = 0; i < bytes - 2; i++) {
            // Field tags for delta_ms would be 0x18 (field 3) or 0x50 (field 10)
            if ((buffer[i] == 0x18 || buffer[i] == 0x50) && i + 1 < bytes) {
                uint8_t delta = buffer[i+1];
                if (delta > 0 && delta < 100) { // Reasonable delta range
                    delta_sum += delta;
                    delta_count++;
                }
            }
        }
        
        offset += bytes;
        packet_count++;
    }
    
    fs_close(&file);
    
    shell_print(sh, "\nAnalysis Summary:");
    shell_print(sh, "  Total bytes read: %zu", offset);
    shell_print(sh, "  Estimated packets: %d", packet_count);
    if (delta_count > 0) {
        shell_print(sh, "  Average delta: %d ms", delta_sum / delta_count);
    }
    
    return 0;
}

static int cmd_log_list(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    const char *dir_path = "/lfs1/hardware";
    struct fs_dir_t dir;
    struct fs_dirent entry;
    int foot_count = 0;
    int bhi360_count = 0;
    size_t total_size = 0;
    
    shell_print(sh, "=== LOG FILE INVENTORY ===");
    shell_print(sh, "Directory: %s\n", dir_path);
    
    fs_dir_t_init(&dir);
    
    int ret = fs_opendir(&dir, dir_path);
    if (ret != 0) {
        shell_error(sh, "Cannot open directory: %d", ret);
        return ret;
    }
    
    shell_print(sh, "%-25s %-10s %s", "FILENAME", "SIZE", "TYPE");
    shell_print(sh, "%-25s %-10s %s", "-------------------------", "----------", "----");
    
    while (fs_readdir(&dir, &entry) == 0 && entry.name[0] != '\0') {
        if (entry.type == FS_DIR_ENTRY_FILE) {
            const char *type = "Unknown";
            if (strncmp(entry.name, "foot_", 5) == 0) {
                type = "Foot Sensor";
                foot_count++;
            } else if (strncmp(entry.name, "bhi360_", 7) == 0) {
                type = "BHI360";
                bhi360_count++;
            }
            
            shell_print(sh, "%-25s %-10zu %s", entry.name, entry.size, type);
            total_size += entry.size;
        }
    }
    
    fs_closedir(&dir);
    
    shell_print(sh, "\nSUMMARY:");
    shell_print(sh, "  Foot Sensor Logs: %d", foot_count);
    shell_print(sh, "  BHI360 Logs: %d", bhi360_count);
    shell_print(sh, "  Total Files: %d", foot_count + bhi360_count);
    shell_print(sh, "  Total Size: %zu bytes (%.2f KB)", total_size, total_size / 1024.0f);
    
    return 0;
}

static int find_latest_log(const char *prefix, char *out_path, size_t path_size)
{
    struct fs_dir_t dir;
    struct fs_dirent entry;
    int latest_id = -1;
    char latest_name[64] = {0};
    
    fs_dir_t_init(&dir);
    
    int ret = fs_opendir(&dir, "/lfs1/hardware");
    if (ret != 0) {
        return -1;
    }
    
    while (fs_readdir(&dir, &entry) == 0 && entry.name[0] != '\0') {
        if (entry.type == FS_DIR_ENTRY_FILE && 
            strncmp(entry.name, prefix, strlen(prefix)) == 0) {
            
            int id = 0;
            if (sscanf(entry.name + strlen(prefix), "%d.pb", &id) == 1) {
                if (id > latest_id) {
                    latest_id = id;
                    strncpy(latest_name, entry.name, sizeof(latest_name) - 1);
                }
            }
        }
    }
    
    fs_closedir(&dir);
    
    if (latest_id >= 0) {
        snprintf(out_path, path_size, "/lfs1/hardware/%s", latest_name);
        return latest_id;
    }
    
    return -1;
}

static int cmd_log_decode_foot(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    char path[128];
    int id = find_latest_log("foot_", path, sizeof(path));
    
    if (id < 0) {
        shell_error(sh, "No foot sensor logs found");
        return -ENOENT;
    }
    
    shell_print(sh, "=== FOOT SENSOR LOG (ID: %d) ===", id);
    return analyze_log_file(sh, path);
}

static int cmd_log_decode_bhi360(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    char path[128];
    int id = find_latest_log("bhi360_", path, sizeof(path));
    
    if (id < 0) {
        shell_error(sh, "No BHI360 logs found");
        return -ENOENT;
    }
    
    shell_print(sh, "=== BHI360 LOG (ID: %d) ===", id);
    return analyze_log_file(sh, path);
}

static int cmd_log_decode_file(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_error(sh, "Usage: log decode file <path>");
        return -EINVAL;
    }
    
    return analyze_log_file(sh, argv[1]);
}

static int cmd_log_delete_all(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    shell_print(sh, "Deleting all log files...");
    
    struct fs_dir_t dir;
    struct fs_dirent entry;
    int deleted = 0;
    
    fs_dir_t_init(&dir);
    
    int ret = fs_opendir(&dir, "/lfs1/hardware");
    if (ret != 0) {
        shell_error(sh, "Cannot open directory: %d", ret);
        return ret;
    }
    
    while (fs_readdir(&dir, &entry) == 0 && entry.name[0] != '\0') {
        if (entry.type == FS_DIR_ENTRY_FILE && 
            (strstr(entry.name, "foot_") || strstr(entry.name, "bhi360_"))) {
            
            char path[128];
            snprintf(path, sizeof(path), "/lfs1/hardware/%s", entry.name);
            
            ret = fs_unlink(path);
            if (ret == 0) {
                shell_print(sh, "  Deleted: %s", entry.name);
                deleted++;
            } else {
                shell_error(sh, "  Failed to delete %s: %d", entry.name, ret);
            }
        }
    }
    
    fs_closedir(&dir);
    
    shell_print(sh, "Deleted %d files", deleted);
    return 0;
}

// Decode subcommands
SHELL_STATIC_SUBCMD_SET_CREATE(log_decode_cmds,
    SHELL_CMD(foot, NULL, "Decode latest foot sensor log", cmd_log_decode_foot),
    SHELL_CMD(bhi360, NULL, "Decode latest BHI360 log", cmd_log_decode_bhi360),
    SHELL_CMD(file, NULL, "Decode specific file", cmd_log_decode_file),
    SHELL_SUBCMD_SET_END
);

// Main log commands
SHELL_STATIC_SUBCMD_SET_CREATE(log_cmds,
    SHELL_CMD(list, NULL, "List all log files", cmd_log_list),
    SHELL_CMD(decode, &log_decode_cmds, "Decode log files", NULL),
    SHELL_CMD(delete_all, NULL, "Delete all log files", cmd_log_delete_all),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(log, &log_cmds, "Log file management commands", NULL);