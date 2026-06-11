/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */


#include <stdbool.h>
#include <stdint.h>
#include "platform.h"
#include <math.h>
#include "drivers/time.h"

#if defined(USE_DRONECAN)

#include "common/log.h"
#include "common/utils.h"
#include "drivers/dronecan/dronecan.h"
#include "sensors/battery_sensor_dronecan.h"

static uint16_t dronecanVbat;
static int16_t dronecanAmperage;
static uint32_t last_batt_msg_ms = 0;
static uint16_t last_status_flags = 0;

typedef struct {
    uint16_t    flag;
    const char *name;
} battStatusFlagEntry_t;

static const battStatusFlagEntry_t battSafetyFlags[] = {
    { UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_TEMP_HOT,    "TEMP_HOT"    },
    { UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_TEMP_COLD,   "TEMP_COLD"   },
    { UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_OVERLOAD,    "OVERLOAD"    },
    { UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_BAD_BATTERY, "BAD_BATTERY" },
    { UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_NEED_SERVICE,"NEED_SERVICE" },
    { UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_BMS_ERROR,   "BMS_ERROR"   },
};

uint16_t dronecanBattSensorGetVBat(void)
{
    return dronecanVbat;
}

int16_t dronecanBattSensorGetAmperage(void)
{
    return dronecanAmperage;
}

void dronecanBatterySensorReceiveInfo(struct uavcan_equipment_power_BatteryInfo *pbatteryInfo, uint8_t sourceNodeId)
{
    // Filter by battery_id if configured (0 = any battery)
    if (dronecanConfig()->batteryId != 0 && pbatteryInfo->battery_id != dronecanConfig()->batteryId) {
        return;
    }

    uint16_t changed = pbatteryInfo->status_flags ^ last_status_flags;
    if (changed) {
        uint16_t newly_set   = changed &  pbatteryInfo->status_flags;
        uint16_t newly_clear = changed & ~pbatteryInfo->status_flags;
        for (unsigned i = 0; i < ARRAYLEN(battSafetyFlags); i++) {
            if (newly_set & battSafetyFlags[i].flag) {
                LOG_WARNING(CAN, "Battery %u: %s", pbatteryInfo->battery_id, battSafetyFlags[i].name);
            }
            if (newly_clear & battSafetyFlags[i].flag) {
                LOG_INFO(CAN, "Battery %u: %s cleared", pbatteryInfo->battery_id, battSafetyFlags[i].name);
            }
        }
        last_status_flags = pbatteryInfo->status_flags;
    }

    const dronecanNodeInfo_t *node = dronecanGetNodeByID(sourceNodeId);
    if (node && node->health >= UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR) {
        return;
    }

    last_batt_msg_ms = millis();

    dronecanVbat = (uint16_t)roundf(pbatteryInfo->voltage * 100.0F);
    dronecanAmperage = (int16_t)roundf(pbatteryInfo->current * 100.0F);
};

bool dronecanBattSensorIsHealthy(void)
{
    // Only one node supported today.  Will need more complex filters for multi battery.  
    return( (millis() - last_batt_msg_ms) < 5000);
}
#endif
