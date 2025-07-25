/**
 * @file data_dispatcher.h
 * @brief Header file for data dispatcher module, defining tagged message structures and enums
 * @version 1.0
 */

#ifndef DATA_DISPATCHER_H
#define DATA_DISPATCHER_H

#include <zephyr/kernel.h>
#include <app.hpp>

// Message tags for data categorization
enum message_tag {
    PRIMARY_FOOT = 1,
    SECONDARY_FOOT = 2,
    PRIMARY_IMU = 3,
    SECONDARY_IMU = 4,
    SYNC_PAIR = 5,
    ACTIVITY_DATA = 6
};

// Extended message structure with tag
typedef struct {
    generic_message_t msg;
    enum message_tag tag;
} tagged_message_t;

#endif // DATA_DISPATCHER_H