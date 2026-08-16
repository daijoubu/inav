#include "platform.h"
#if defined(USE_DRONECAN)

#include <stdint.h>

#include "build/atomic.h"

#include "common/bitarray.h"
#include "common/log.h"
#include "common/time.h"

#include "drivers/nvic.h"

#include "libcanard/canard.h"

#include <dronecan_msgs.h>

#include "dronecan_actuator.h"

#define ACTUATOR_FLOOR_INTERVAL 20000ULL // 50 Hz check for 25 Hz updates
#define ACTUATOR_COMMANDS_PER_MESSAGE 15 // uavcan.equipment.actuator.ArrayCommand DSDL limit
#define ACTUATOR_CHANNEL_COUNT 18 // matches flight/servos.h's MAX_SUPPORTED_SERVOS

extern CanardInstance canard; /* the FC's own canard instance, owned by dronecan.c */

static struct ardupilot_indication_SafetyState ardupilotSafetyState = {.status = ARDUPILOT_INDICATION_SAFETYSTATE_STATUS_SAFETY_OFF};
static BITARRAY_DECLARE(actuatorDirty, ACTUATOR_CHANNEL_COUNT);          // set on value change (dronecanWriteServo), cleared on send
static BITARRAY_DECLARE(actuatorUpdateAtCheck, ACTUATOR_CHANNEL_COUNT); // set whenever a channel is sent, for the floor check
static uint32_t actuatorUpdateInterval = 20000; // 50 Hz update to be replaced with a setter to match actual update interval.
typedef struct actuatorCommand_s {
    uint8_t actuatorId;
    uint16_t value;
} actuatorCommand_t;
static actuatorCommand_t actuatorCommands[ACTUATOR_CHANNEL_COUNT];

static void sendActuatorCommandArray(const actuatorCommand_t *data, uint8_t len);
static bool sendActuatorCommandBatch(const actuatorCommand_t *data, uint8_t len, int8_t *next);
static void actuatorFloorWindowReset(void);

void dronecanWriteServo(uint8_t servo, uint16_t value)
{
    actuatorCommands[servo].actuatorId = servo+1;
    if(actuatorCommands[servo].value != value)  // only update on change
    {
        actuatorCommands[servo].value = value;
        bitArraySet(actuatorDirty, servo);     // mark dirty
    }
}

void dronecanServoInit(uint16_t updateRate)
{
    if((updateRate >= 25) && (updateRate <= 498))
        actuatorUpdateInterval = 1000000UL / updateRate;
}

void dronecanActuatorUpdate(timeUs_t currentTimeUs)
{
    static timeUs_t next_safety_broadcast_at = 0;
    static timeUs_t next_actuator_floor_check_at = 0;
    static timeUs_t next_actuator_update_at = 0;
    int next = 0;

    if (currentTimeUs >= next_safety_broadcast_at)
    {
        uint32_t len;
        uint8_t buffer[ARDUPILOT_INDICATION_SAFETYSTATE_MAX_SIZE];
        next_safety_broadcast_at = currentTimeUs + 500000ULL; // Broadcast at 2 Hz.
        len = ardupilot_indication_SafetyState_encode(&ardupilotSafetyState, buffer);
        static uint8_t transfer_id;
        int16_t bc_res;
        ATOMIC_BLOCK(NVIC_PRIO_CAN) {
            bc_res = canardBroadcast(&canard,
                                     ARDUPILOT_INDICATION_SAFETYSTATE_SIGNATURE,
                                     ARDUPILOT_INDICATION_SAFETYSTATE_ID,
                                     &transfer_id,
                                     // LOW: INAV only ever sends a static, non-urgent
                                     // unlock signal here, not a real-time safety-stop -
                                     // raise this if that ever changes.
                                     CANARD_TRANSFER_PRIORITY_LOW,
                                     buffer,
                                     len);
        }
        if (bc_res < 0) {
            LOG_WARNING(CAN, "ArduPilot Safety Switch broadcast failed: %d", bc_res);
        }
    }

    if (currentTimeUs >= next_actuator_floor_check_at)
    {
        for(int i = 0; i < ACTUATOR_CHANNEL_COUNT;)
        {
            next = BITARRAY_FIND_FIRST_SET(actuatorUpdateAtCheck, i);
            if (next < 0)
                break;  // No more to update
            bitArraySet(actuatorDirty, next);
            i = next + 1;  // advance past this bit for the next search
        }
        actuatorFloorWindowReset();
        next_actuator_floor_check_at = currentTimeUs + ACTUATOR_FLOOR_INTERVAL;
    }
    if (currentTimeUs > next_actuator_update_at)
    {
        sendActuatorCommandArray(actuatorCommands, ACTUATOR_CHANNEL_COUNT); // send any queued actuator commands
        next_actuator_update_at = currentTimeUs + actuatorUpdateInterval;
    }
}

/*
  Mark every real actuator channel as due for the next floor check. Scoped to
  exactly ACTUATOR_CHANNEL_COUNT bits, unlike BITARRAY_SET_ALL() which would
  set every bit in the underlying storage word (32, for an 18-bit bitarray) -
  including phantom bits past the real channel count.
*/
static void actuatorFloorWindowReset(void)
{
    for (int i = 0; i < ACTUATOR_CHANNEL_COUNT; i++) {
        bitArraySet(actuatorUpdateAtCheck, i);
    }
}

/*
  Pack up to ACTUATOR_COMMANDS_PER_MESSAGE dirty channels (resuming the
  bitarray scan from *next) into one ArrayCommand message and broadcast it.
  Returns true if the batch filled up to the per-message cap (more dirty
  channels may remain), false if the scan ran out of dirty channels first
  (nothing left to send).
*/
static bool sendActuatorCommandBatch(const actuatorCommand_t *data, uint8_t len, int8_t *next)
{
    struct uavcan_equipment_actuator_ArrayCommand commandArray;
    uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_MAX_SIZE];
    uint16_t payload_len;
    int i;
    static uint8_t transfer_id;
    int16_t bc_res;

    i = 0;
    while (i < len && i < ACTUATOR_COMMANDS_PER_MESSAGE)
    {
        *next = BITARRAY_FIND_FIRST_SET(actuatorDirty, *next);
        if (*next < 0) {
            break;
        }
        bitArrayClr(actuatorDirty, *next);
        bitArrayClr(actuatorUpdateAtCheck, *next);
        if (data[*next].value != 0)
        {
            commandArray.commands.data[i].actuator_id = data[*next].actuatorId;
            commandArray.commands.data[i].command_type = UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_PWM;
            commandArray.commands.data[i].command_value = (float)data[*next].value;
            i++;
        }
        // value == 0: bits already cleared above, don't consume an output slot.
    }
    commandArray.commands.len = i;

    if (commandArray.commands.len > 0) {  // Only send if data to send
        payload_len = uavcan_equipment_actuator_ArrayCommand_encode(&commandArray, buffer);

        ATOMIC_BLOCK(NVIC_PRIO_CAN) {
            bc_res = canardBroadcast(&canard,
                                    UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE,
                                    UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID,
                                    &transfer_id,
                                    CANARD_TRANSFER_PRIORITY_HIGH,
                                    buffer,
                                    payload_len);
        }
        if (bc_res < 0) {
            LOG_WARNING(CAN, "ActuatorCommandArray broadcast failed: %d", bc_res);
        }
    }

    return i == ACTUATOR_COMMANDS_PER_MESSAGE;
}

/*
  Send an ActuatorCommandArray broadcast message. This is used to control
  actuators on other nodes, such as ESCs or servos. More channels than fit
  in one message (ACTUATOR_COMMANDS_PER_MESSAGE) are sent as a second batch.
*/
static void sendActuatorCommandArray(const actuatorCommand_t *data, uint8_t len)
{
    int8_t next = 0;

    if (sendActuatorCommandBatch(data, len, &next)) {
        // First batch hit the per-message cap - more dirty channels may
        // remain (as opposed to running out of dirty channels mid-scan).
        sendActuatorCommandBatch(data, len, &next);
    }
}

#endif // USE_DRONECAN
