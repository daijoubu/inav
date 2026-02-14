#pragma once

#include "common/time.h"
#include "config/parameter_group.h"

void dronecanInit(void);
void dronecanUpdate(timeUs_t currentTimeUs);

typedef enum {
    DRONECAN_BITRATE_125KBPS = 0,
    DRONECAN_BITRATE_250KBPS,
    DRONECAN_BITRATE_500KBPS,
    DRONECAN_BITRATE_1000KBPS,
    DRONECAN_BITRATE_COUNT
} dronecanBitrate_e;

typedef struct dronecanConfig_s {
    uint8_t nodeID;
    dronecanBitrate_e bitRateKbps;
    uint8_t batteryId;   // Filter by battery_id in BatteryInfo message (0-255, 0 = any)
    uint8_t gpsNodeId;   // Filter GPS messages by source Node ID (0 = any)
} dronecanConfig_t;

PG_DECLARE(dronecanConfig_t, dronecanConfig);
