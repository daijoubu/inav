/**
 * DroneCAN DNA Server Unit Tests
 *
 * Tests the three-part UID accumulation, node ID assignment, and allocation
 * table management in dronecan_dna_server.c compiled against real DSDL
 * encode/decode and INAV stubs.
 *
 * Coverage:
 *   DNA-1  Full 3-stage handshake → entry stored with correct UID and node ID
 *   DNA-2  Same UID on second handshake → same node ID returned, no new entry
 *   DNA-3  Two different UIDs → different node IDs assigned
 *   DNA-4  Table full (DRONECAN_MAX_NODES entries) → no allocation for next node
 *   DNA-5  Non-broadcast source node ID → transfer silently rejected
 *   DNA-6  Stage 2 without prior Stage 1 → stage mismatch, rejected
 *   DNA-7  Followup timeout mid-handshake → accumulator reset, Stage 2 rejected
 *   DNA-8  FC's own node ID is never assigned to a peripheral
 *   DNA-9  [FAILING] Peripheral requests specific node ID → not yet honoured
 */

#include "gtest/gtest.h"

extern "C" {
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "platform.h"

#include "uavcan.protocol.dynamic_node_id.Allocation.h"
#include "drivers/dronecan/libcanard/canard.h"
#include "drivers/dronecan/dronecan.h"
#include "drivers/dronecan/dronecan_dna_server.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

/* Global canard instance — extern-declared in dronecan_dna_server.h.
   PG_REGISTER in dronecan_dna_server.c already emits dnaServerData_System
   and dnaServerData_Copy, so we must NOT define them here. */
CanardInstance canard;

/* Controllable time source */
static uint32_t mock_time_ms = 0;
uint32_t millis(void) { return mock_time_ms; }

/* saveConfig — called when a new allocation is persisted */
void saveConfig(void) {}

/* Live node table stubs — no network in unit tests; allocator sees 0 active nodes */
uint8_t dronecanGetNodeCount(void) { return 0; }
const dronecanNodeInfo_t *dronecanGetNode(uint8_t index) { (void)index; return NULL; }

} /* extern "C" */

/* =========================================================================
 * Constants
 * ========================================================================= */

#define FC_NODE_ID  5u
#define MAX_LEN     UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST
#define FOLLOWUP_MS UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_FOLLOWUP_TIMEOUT_MS

/* =========================================================================
 * Helpers
 * ========================================================================= */

/* Build a broadcast Allocation transfer from an anonymous peripheral */
static CanardRxTransfer makeAllocationTransfer(
        uint8_t        requested_node_id,
        bool           first_part,
        const uint8_t *uid,
        uint8_t        uid_len,
        uint8_t       *buf)
{
    struct uavcan_protocol_dynamic_node_id_Allocation msg;
    memset(&msg, 0, sizeof(msg));
    msg.node_id                 = requested_node_id;
    msg.first_part_of_unique_id = first_part;
    msg.unique_id.len           = uid_len;
    memcpy(msg.unique_id.data, uid, uid_len);

    uint32_t len = uavcan_protocol_dynamic_node_id_Allocation_encode(&msg, buf);

    CanardRxTransfer xfer;
    memset(&xfer, 0, sizeof(xfer));
    xfer.transfer_type  = CanardTransferTypeBroadcast;
    xfer.data_type_id   = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID;
    xfer.source_node_id = CANARD_BROADCAST_NODE_ID;
    xfer.payload_head   = buf;
    xfer.payload_len    = (uint16_t)len;
    return xfer;
}

/* Run a complete 3-stage handshake for the given 16-byte UID.
   Stages are sent 100 ms apart (well within the 500 ms followup timeout).
   Returns the node_id stored in the PG allocation table for this UID,
   or 0 if no entry was created. */
static uint8_t runHandshake(const uint8_t uid[16], uint8_t requested_node_id = 0)
{
    uint8_t buf[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_SIZE + 4];
    CanardRxTransfer xfer;

    /* Stage 1: first MAX_LEN bytes, first_part=true */
    xfer = makeAllocationTransfer(requested_node_id, true, uid, MAX_LEN, buf);
    dronecanDnaHandleAllocation(NULL, &xfer);
    mock_time_ms += 100;

    /* Stage 2: next MAX_LEN bytes, first_part=false */
    xfer = makeAllocationTransfer(0, false, uid + MAX_LEN, MAX_LEN, buf);
    dronecanDnaHandleAllocation(NULL, &xfer);
    mock_time_ms += 100;

    /* Stage 3: remaining bytes, first_part=false */
    xfer = makeAllocationTransfer(0, false, uid + MAX_LEN * 2,
                                  (uint8_t)(16 - MAX_LEN * 2), buf);
    dronecanDnaHandleAllocation(NULL, &xfer);
    mock_time_ms += 100;

    /* Look up the result in the PG allocation table */
    for (int i = 0; i < DRONECAN_MAX_NODES; i++) {
        if (dnaServerData()->entries[i].nodeId != 0 &&
            memcmp(dnaServerData()->entries[i].uniqueId, uid, 16) == 0) {
            return dnaServerData()->entries[i].nodeId;
        }
    }
    return 0;
}

/* =========================================================================
 * Fixture
 * ========================================================================= */

class DroneCANDnaServerTest : public ::testing::Test {
protected:
    static uint32_t s_base_ms;
    uint8_t canard_pool[512];

    void SetUp() override {
        /* Jump 10 seconds forward each test so the DNA server's followup
           timeout fires on the very first call, resetting any accumulated
           UID state left by the previous test. */
        s_base_ms   += 10000;
        mock_time_ms = s_base_ms;

        memset(dnaServerDataMutable(), 0, sizeof(dnaServerData_t));

        canardInit(&canard, canard_pool, sizeof(canard_pool),
                   NULL, NULL, NULL);
        canardSetLocalNodeID(&canard, FC_NODE_ID);
    }
};
uint32_t DroneCANDnaServerTest::s_base_ms = 0;

/* =========================================================================
 * DNA-1: Full 3-stage handshake stores entry with correct UID and node ID
 * ========================================================================= */
TEST_F(DroneCANDnaServerTest, FullHandshakeAssignsNodeId)
{
    const uint8_t uid[16] = {
        0x01,0x02,0x03,0x04,0x05,0x06,
        0x07,0x08,0x09,0x0A,0x0B,0x0C,
        0x0D,0x0E,0x0F,0x10
    };

    uint8_t assigned = runHandshake(uid);

    EXPECT_NE(assigned, 0u);
    EXPECT_NE(assigned, FC_NODE_ID);
    EXPECT_LT(assigned, (uint8_t)CANARD_MAX_NODE_ID);

    /* Verify the table entry contains the correct UID */
    bool found = false;
    for (int i = 0; i < DRONECAN_MAX_NODES; i++) {
        if (dnaServerData()->entries[i].nodeId == assigned) {
            EXPECT_EQ(0, memcmp(dnaServerData()->entries[i].uniqueId, uid, 16));
            found = true;
            break;
        }
    }
    EXPECT_TRUE(found) << "Assigned ID not present in allocation table";
}

/* =========================================================================
 * DNA-2: Same UID on second handshake → same node ID, no new table entry
 * ========================================================================= */
TEST_F(DroneCANDnaServerTest, SameUidReturnsSameNodeId)
{
    const uint8_t uid[16] = {
        0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,
        0x11,0x22,0x33,0x44,0x55,0x66,
        0x77,0x88,0x99,0x00
    };

    uint8_t first = runHandshake(uid);
    ASSERT_NE(first, 0u);

    s_base_ms   += 10000;
    mock_time_ms = s_base_ms;

    uint8_t second = runHandshake(uid);

    EXPECT_EQ(first, second) << "Same UID must return same node ID on re-negotiation";

    int count = 0;
    for (int i = 0; i < DRONECAN_MAX_NODES; i++) {
        if (dnaServerData()->entries[i].nodeId != 0 &&
            memcmp(dnaServerData()->entries[i].uniqueId, uid, 16) == 0) {
            count++;
        }
    }
    EXPECT_EQ(count, 1) << "Re-negotiation must not create a second table entry";
}

/* =========================================================================
 * DNA-3: Two different UIDs receive different node IDs
 * ========================================================================= */
TEST_F(DroneCANDnaServerTest, TwoDifferentUidsGetDifferentNodeIds)
{
    const uint8_t uid1[16] = {
        0x01,0x01,0x01,0x01,0x01,0x01,
        0x01,0x01,0x01,0x01,0x01,0x01,
        0x01,0x01,0x01,0x01
    };
    const uint8_t uid2[16] = {
        0x02,0x02,0x02,0x02,0x02,0x02,
        0x02,0x02,0x02,0x02,0x02,0x02,
        0x02,0x02,0x02,0x02
    };

    uint8_t id1 = runHandshake(uid1);
    ASSERT_NE(id1, 0u);

    s_base_ms   += 10000;
    mock_time_ms = s_base_ms;

    uint8_t id2 = runHandshake(uid2);
    ASSERT_NE(id2, 0u);

    EXPECT_NE(id1, id2);
}

/* =========================================================================
 * DNA-4: Table full → the next peripheral receives no allocation
 * ========================================================================= */
TEST_F(DroneCANDnaServerTest, TableFullReturnsNoAllocation)
{
    /* Fill every slot with distinct fake UIDs and valid node IDs,
       skipping FC_NODE_ID in the assigned sequence. */
    uint8_t nextId = CANARD_MIN_NODE_ID;
    for (int i = 0; i < DRONECAN_MAX_NODES; i++) {
        if (nextId == FC_NODE_ID) nextId++;
        dnaServerDataMutable()->entries[i].nodeId = nextId++;
        memset(dnaServerDataMutable()->entries[i].uniqueId, (uint8_t)(i + 1), 16);
    }

    const uint8_t uid[16] = {
        0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
        0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
        0xFF,0xFF,0xFF,0xFF
    };

    uint8_t assigned = runHandshake(uid);
    EXPECT_EQ(assigned, 0u) << "Full table must not produce an allocation";

    for (int i = 0; i < DRONECAN_MAX_NODES; i++) {
        EXPECT_NE(0, memcmp(dnaServerData()->entries[i].uniqueId, uid, 16))
            << "Full table must not write a new entry at slot " << i;
    }
}

/* =========================================================================
 * DNA-5: Non-broadcast source node ID → rejected before any processing
 * ========================================================================= */
TEST_F(DroneCANDnaServerTest, NonBroadcastSourceRejected)
{
    uint8_t buf[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_SIZE + 4];
    const uint8_t uid[16] = {
        0xDE,0xAD,0xBE,0xEF,0xDE,0xAD,
        0xBE,0xEF,0xDE,0xAD,0xBE,0xEF,
        0xDE,0xAD,0xBE,0xEF
    };

    CanardRxTransfer xfer = makeAllocationTransfer(0, true, uid, MAX_LEN, buf);
    xfer.source_node_id = 42;   /* non-anonymous — must be rejected */
    dronecanDnaHandleAllocation(NULL, &xfer);

    for (int i = 0; i < DRONECAN_MAX_NODES; i++) {
        EXPECT_EQ(dnaServerData()->entries[i].nodeId, 0u)
            << "Non-broadcast source must not create a table entry";
    }
}

/* =========================================================================
 * DNA-6: Stage 2 without a preceding Stage 1 → stage mismatch, rejected
 * ========================================================================= */
TEST_F(DroneCANDnaServerTest, Stage2WithoutStage1Rejected)
{
    uint8_t buf[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_SIZE + 4];
    const uint8_t uid[16] = {
        0xCA,0xFE,0xBA,0xBE,0xCA,0xFE,
        0xBA,0xBE,0xCA,0xFE,0xBA,0xBE,
        0xCA,0xFE,0xBA,0xBE
    };

    /* first_part=false and len=MAX_LEN is a Stage 2 message */
    CanardRxTransfer xfer = makeAllocationTransfer(0, false, uid + MAX_LEN, MAX_LEN, buf);
    dronecanDnaHandleAllocation(NULL, &xfer);

    for (int i = 0; i < DRONECAN_MAX_NODES; i++) {
        EXPECT_EQ(dnaServerData()->entries[i].nodeId, 0u)
            << "Out-of-sequence Stage 2 must not create a table entry";
    }
}

/* =========================================================================
 * DNA-7: Followup timeout mid-handshake resets the UID accumulator
 * ========================================================================= */
TEST_F(DroneCANDnaServerTest, FollowupTimeoutResetsAccumulator)
{
    uint8_t buf[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_SIZE + 4];
    const uint8_t uid[16] = {
        0x10,0x20,0x30,0x40,0x50,0x60,
        0x70,0x80,0x90,0xA0,0xB0,0xC0,
        0xD0,0xE0,0xF0,0x00
    };

    /* Stage 1 */
    CanardRxTransfer s1 = makeAllocationTransfer(0, true, uid, MAX_LEN, buf);
    dronecanDnaHandleAllocation(NULL, &s1);

    /* Advance past the followup timeout — server must reset accumulator */
    mock_time_ms += FOLLOWUP_MS + 100;

    /* Stage 2 — now arrives as if no Stage 1 preceded it */
    CanardRxTransfer s2 = makeAllocationTransfer(0, false, uid + MAX_LEN, MAX_LEN, buf);
    dronecanDnaHandleAllocation(NULL, &s2);

    for (int i = 0; i < DRONECAN_MAX_NODES; i++) {
        EXPECT_EQ(dnaServerData()->entries[i].nodeId, 0u)
            << "Stage 2 after timeout must not create a table entry";
    }
}

/* =========================================================================
 * DNA-8: FC's own node ID is never assigned to a peripheral
 * ========================================================================= */
TEST_F(DroneCANDnaServerTest, FcNodeIdNeverAssigned)
{
    /* Run enough handshakes that sequential assignment would reach FC_NODE_ID
       if the skip were absent. FC_NODE_ID = 5, so IDs 1-4 are assigned first,
       then the 5th peripheral should get 6, not 5. */
    for (uint8_t n = 0; n < FC_NODE_ID; n++) {
        uint8_t uid[16];
        memset(uid, n + 1, 16);

        uint8_t assigned = runHandshake(uid);
        ASSERT_NE(assigned, 0u) << "Handshake " << (int)n << " produced no allocation";
        EXPECT_NE(assigned, FC_NODE_ID)
            << "FC node ID " << (int)FC_NODE_ID
            << " was incorrectly assigned at handshake " << (int)n;

        s_base_ms   += 10000;
        mock_time_ms = s_base_ms;
    }
}

/* =========================================================================
 * DNA-9 [FAILING]: Peripheral's preferred node ID is honoured
 *
 * The UAVCAN spec allows a peripheral to include a preferred node_id in its
 * Stage-1 request. The allocator should assign that ID when available.
 * This is not yet implemented — the handler ignores the field and assigns
 * sequentially. Expected to fail until the feature is added.
 * ========================================================================= */
TEST_F(DroneCANDnaServerTest, RequestedNodeIdIsHonoured)
{
    const uint8_t uid[16] = {
        0x55,0x55,0x55,0x55,0x55,0x55,
        0x55,0x55,0x55,0x55,0x55,0x55,
        0x55,0x55,0x55,0x55
    };
    const uint8_t requested = 42u;

    uint8_t assigned = runHandshake(uid, requested);

    EXPECT_EQ(assigned, requested)
        << "Peripheral requested node ID " << (int)requested
        << " but was assigned " << (int)assigned;
}
