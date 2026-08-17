#pragma once

#include <stdint.h>
#include "common/time.h"

#ifdef USE_DRONECAN

/* Called from flight/servos.c's writeServos() for each active (compacted-
   index) servo output, alongside the local PWM write. */
void dronecanWriteServo(uint8_t servo, uint16_t value);

/* Called once per dronecanUpdate() tick while in STATE_DRONECAN_NORMAL.
   Owns the actuator.ArrayCommand on-change/floor/ceiling broadcast
   scheduling and the AP_Periph SafetyState heartbeat that unlocks it. */
void dronecanActuatorUpdate(timeUs_t currentTimeUs);

#endif // USE_DRONECAN
