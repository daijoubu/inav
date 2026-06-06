#pragma once

#include <stdint.h>
#include "libcanard/canard.h"

extern CanardInstance canard;

void dronecanDnaHandleAllocation(CanardInstance *ins, CanardRxTransfer *transfer);
