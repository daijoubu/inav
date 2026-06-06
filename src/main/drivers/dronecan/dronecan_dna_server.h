#pragma once

#include <stdint.h>
#include "libcanard/canard.h"

extern CanardInstance canard;

#define DNA_UNIQUE_ID_LENGTH       16

typedef struct {
    uint8_t uniqueId[DNA_UNIQUE_ID_LENGTH];
    uint8_t nodeId;
} dnaAllocationEntry_t;

typedef struct {
    dnaAllocationEntry_t entries[DRONECAN_MAX_NODES];
} dnaServerData_t;
  
PG_DECLARE(dnaServerData_t, dnaServerData);


void dronecanDnaHandleAllocation(CanardInstance *ins, CanardRxTransfer *transfer);
