#ifndef STATUS_CODES_H
#define STATUS_CODES_H

#include <stdint.h>

// Device status bitfield definitions
// You can OR these together for combined status
#define STATUS_ERROR        (1 << 0)
#define STATUS_CALIBRATING (1 << 1)
#define STATUS_READY       (1 << 2)
#define STATUS_IDLE        (1 << 3)
// Add more as needed

#endif // STATUS_CODES_H
