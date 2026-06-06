#include "platform.h"
#include "common/log.h"
#include "drivers/time.h"
#include <string.h>
#include <stdio.h>
#include <dronecan_msgs.h>
#include "dronecan.h"
#include "dronecan_dna_server.h"

#define DNA_UNIQUE_ID_LENGTH       16
#define DNA_INVALID_STAGE          -1
#define DNA_STAGE_1                1
#define DNA_STAGE_2                2
#define DNA_STAGE_3                3

typedef struct {
    uint8_t uniqueId[DNA_UNIQUE_ID_LENGTH];
    uint8_t nodeId;
} dnaAllocationEntry_t;

static dnaAllocationEntry_t dnaAllocationTable[DRONECAN_MAX_NODES];

static int8_t detectRequestStage(struct uavcan_protocol_dynamic_node_id_Allocation *msg);
static int8_t getExpectedStage(uint8_t currentUniqueIdLength);
static uint8_t dnaLookupOrAssignNode(const uint8_t *uid);
static void dnaSendResponse(uint8_t nodeId, const uint8_t *uid, uint8_t uidLen);

/*
    Entry point for DroneCAN Dynamic Node Allocation messages.
    Called from onTransferReceived when a UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID
    broadcast is received.

    Assembles the complete unique identifier from up to three request stages,
    then assigns or confirms a node ID for the requesting peripheral.
    Follows the UAVCAN specification for non-redundant (single-master) allocators.
*/
void dronecanDnaHandleAllocation(CanardInstance *ins, CanardRxTransfer *transfer)
{
    UNUSED(ins);

    struct uavcan_protocol_dynamic_node_id_Allocation dynamicAllocation;

    static struct {
        uint8_t len;
        uint8_t data[DNA_UNIQUE_ID_LENGTH];
    } currentUniqueId;

    static uint32_t lastMessageTimestamp = 0;
    int8_t request_stage;

    if (transfer->source_node_id != CANARD_BROADCAST_NODE_ID)
        return;

    if ((millis() - lastMessageTimestamp) > UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_FOLLOWUP_TIMEOUT_MS) {
        memset(currentUniqueId.data, 0, DNA_UNIQUE_ID_LENGTH);
        currentUniqueId.len = 0;
    }

    if (uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &dynamicAllocation)) {
        LOG_ERROR(CAN, "DNA decode failed");
        return;
    }

    request_stage = detectRequestStage(&dynamicAllocation);
    if (request_stage == DNA_INVALID_STAGE)
    {
        LOG_DEBUG(CAN, "Malformed request");
        return;
    }
    LOG_DEBUG(CAN, "Request Stage %u, Length: %u", request_stage, currentUniqueId.len);
    if (request_stage != getExpectedStage(currentUniqueId.len))
    {
        LOG_DEBUG(CAN, "DNA Stage mismatch");
        return;
    }
    if (dynamicAllocation.unique_id.len > DNA_UNIQUE_ID_LENGTH - currentUniqueId.len)
    {
        LOG_DEBUG(CAN, "Malformed request - message unique ID exceeds remaining capacity.");
        return;
    }

    memcpy(currentUniqueId.data + currentUniqueId.len, dynamicAllocation.unique_id.data, dynamicAllocation.unique_id.len);
    currentUniqueId.len += dynamicAllocation.unique_id.len;

    char uidStr[DNA_UNIQUE_ID_LENGTH * 2 + 1];
    for (int i = 0; i < currentUniqueId.len; i++) {
        sprintf(&uidStr[i * 2], "%02x", currentUniqueId.data[i]);
    }
    LOG_DEBUG(CAN, "Received UID part: %s", uidStr);

    if (currentUniqueId.len == DNA_UNIQUE_ID_LENGTH)
    {
        uint8_t assignedNodeId = dnaLookupOrAssignNode(currentUniqueId.data);
        if (assignedNodeId != 0) {
            LOG_DEBUG(CAN, "Assigned Node ID: %u", assignedNodeId);
            dnaSendResponse(assignedNodeId, currentUniqueId.data, currentUniqueId.len);
        }
        memset(currentUniqueId.data, 0, DNA_UNIQUE_ID_LENGTH);
        currentUniqueId.len = 0;
    }
    else
    {
        dnaSendResponse(0, currentUniqueId.data, currentUniqueId.len);
    }

    LOG_DEBUG(CAN, "Received a DNA allocation broadcast. First part is %u", dynamicAllocation.first_part_of_unique_id);
    lastMessageTimestamp = millis();
}

static void dnaSendResponse(uint8_t nodeId, const uint8_t *uid, uint8_t uidLen)
{
    struct uavcan_protocol_dynamic_node_id_Allocation msg;
    uint8_t buffer[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_SIZE];
    static uint8_t transferId;
    CanardTxTransfer outboundTransfer;

    msg.node_id = nodeId;
    msg.first_part_of_unique_id = 0;
    memcpy(msg.unique_id.data, uid, uidLen);
    msg.unique_id.len = uidLen;

    uint32_t len = uavcan_protocol_dynamic_node_id_Allocation_encode(&msg, buffer);

    canardInitTxTransfer(&outboundTransfer);
    outboundTransfer.data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
    outboundTransfer.data_type_id        = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID;
    outboundTransfer.inout_transfer_id   = &transferId;
    outboundTransfer.priority            = CANARD_TRANSFER_PRIORITY_LOW;
    outboundTransfer.payload             = buffer;
    outboundTransfer.payload_len         = (uint16_t)len;

    canardBroadcastObj(&canard, &outboundTransfer);
}

/*
    Search the allocation table for an existing entry matching the given unique
    identifier. If found, return the previously assigned node ID. If not found,
    find the first unused node ID (skipping our own) and record a new allocation.
    Returns 0 if the table is full.
*/
static uint8_t dnaLookupOrAssignNode(const uint8_t *uid)
{
    uint8_t assigned;
    uint8_t assignedNodeId;

    for (int i = 0; i < DRONECAN_MAX_NODES; i++) {
        if (dnaAllocationTable[i].nodeId != 0 &&
            memcmp(dnaAllocationTable[i].uniqueId, uid, DNA_UNIQUE_ID_LENGTH) == 0) {
            return dnaAllocationTable[i].nodeId;
        }
    }

    for (assignedNodeId = CANARD_MIN_NODE_ID; assignedNodeId < CANARD_MAX_NODE_ID; assignedNodeId++) {
        assigned = false;
        if (assignedNodeId == canard.node_id)
            continue;

        for (int i = 0; i < DRONECAN_MAX_NODES; i++) {
            if (assignedNodeId == dnaAllocationTable[i].nodeId) {
                assigned = true;
                break;
            }
        }
        if (assigned == false)
            break;
    }
    for (int i = 0; i < DRONECAN_MAX_NODES; i++) {
        if (dnaAllocationTable[i].nodeId == 0) {
            memcpy(dnaAllocationTable[i].uniqueId, uid, DNA_UNIQUE_ID_LENGTH);
            dnaAllocationTable[i].nodeId = assignedNodeId;
            LOG_DEBUG(CAN, "Added node: %u at index %u", assignedNodeId, i);
            assigned = true;
            break;
        }
    }
    if (!assigned) {
        LOG_ERROR(CAN, "DNA: allocation table full");
        return 0;
    }
    return assignedNodeId;
}

/*
    Determine which stage of the three-part allocation handshake we expect
    next, based on how many bytes of the unique identifier have been accumulated.
*/
static int8_t getExpectedStage(uint8_t currentUniqueIdLength)
{
    if (currentUniqueIdLength == 0)
    {
        LOG_DEBUG(CAN, "Stage 1");
        return DNA_STAGE_1;
    }
    if (currentUniqueIdLength >= (UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST * 2))
    {
        LOG_DEBUG(CAN, "Stage 3");
        return DNA_STAGE_3;
    }
    if (currentUniqueIdLength >= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST)
    {
        LOG_DEBUG(CAN, "Stage 2");
        return DNA_STAGE_2;
    }
    return DNA_INVALID_STAGE;
}

/*
    Classify an incoming allocation request by its stage based on the
    first_part_of_unique_id flag and the unique_id payload length.
    Returns DNA_STAGE_1, DNA_STAGE_2, DNA_STAGE_3, or DNA_INVALID_STAGE.
*/
static int8_t detectRequestStage(struct uavcan_protocol_dynamic_node_id_Allocation *msg)
{
    if ((msg->unique_id.len != UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST) &&
        (msg->unique_id.len != (DNA_UNIQUE_ID_LENGTH - UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST * 2U)) &&
        (msg->unique_id.len != DNA_UNIQUE_ID_LENGTH))
    {
        return DNA_INVALID_STAGE;
    }
    if (msg->first_part_of_unique_id)
    {
        return DNA_STAGE_1;
    }
    if (msg->unique_id.len == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST)
    {
        return DNA_STAGE_2;
    }
    if (msg->unique_id.len < UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST)
    {
        return DNA_STAGE_3;
    }
    return DNA_INVALID_STAGE;
}
