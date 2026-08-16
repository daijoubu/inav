/**
 * DroneCAN Actuator Output — Regression Test
 *
 * Feature under test: INAV broadcasts uavcan.equipment.actuator.ArrayCommand
 * messages that reflect the mixer's servo[] output, for servos with DroneCAN
 * output enabled.
 *
 * flight/servos.c's writeServos() is the real integration point — it calls
 * dronecanWriteServo(compactedIndex, value) once per active servo on every
 * main-loop iteration. flight/servos.c isn't part of this test's linked
 * sources (its dependency graph is large — programming/logic_condition.h
 * etc.), so these tests call dronecanWriteServo() directly to stand in for
 * that caller, rather than going through a mixer/servo[] stub that
 * dronecan.c itself never reads.
 *
 * Full expected-behavior spec:
 *   claude/projects/active/feature-dronecan-actuator-control/docs.md
 *
 * This test compiles the real drivers/dronecan/dronecan.c against INAV
 * stubs (same pattern as dronecan_application_unittest.cc) and drives the
 * production dronecanUpdate() task loop. A second, independent CanardInstance
 * ("rxIns") acts as a loopback receiving node: every CAN frame the FC's
 * canard instance transmits (captured via the canardSTM32Transmit stub) is
 * immediately handed to rxIns via canardHandleRxFrame(), exactly as if it
 * arrived over a real CAN bus. rxIns decodes any ArrayCommand broadcast with
 * the real DSDL decoder, so assertions check the actual wire contents, not
 * internal INAV state.
 *
 * Coverage:
 *   ACT-1  Servo value change -> ArrayCommand broadcast within one
 *          floor/ceiling window, with correct actuator_id (1:1 default
 *          mapping), command_type=PWM, command_value=raw microsecond value.
 *   ACT-2  Servo value held constant -> ArrayCommand keeps being broadcast
 *          at the 25 Hz keepalive floor (watchdog liveness heartbeat).
 *   ACT-3  More than 15 (the DSDL per-message limit) simultaneously dirty
 *          channels split correctly across two ArrayCommand messages, with
 *          every channel's actuator_id/value intact. Regression coverage
 *          for a real bug where the DSDL's fixed 15-element array was
 *          indexed by raw channel number instead of pack position,
 *          corrupting memory whenever more than 15 of 18 channels were
 *          dirty at once.
 *   ACT-4  A servo commanded to exactly 0, and a channel dronecanWriteServo()
 *          is never called for at all, are both excluded from broadcasts -
 *          regression coverage for two related bugs: sending a literal 0
 *          command (INAV's local-PWM no-signal sentinel, not a real
 *          DroneCAN value) on disarm, and the floor check force-broadcasting
 *          channels outside the mixer's configured range.
 *   ACT-5  A realistic middle count (6 of 18 possible channels dirty,
 *          interspersed with untouched channels) fits in a single
 *          ArrayCommand broadcast, and only the dirty channels appear.
 *          Fills the gap between the ACT-1 (single channel) and ACT-3
 *          (all 18 channels) extremes with the common real-world case of a
 *          handful of CAN servos on a partially-populated channel set.
 *   ACT-6  Exactly 15 simultaneously-dirty channels (the DSDL per-message
 *          cap, ACTUATOR_COMMANDS_PER_MESSAGE) fit in exactly ONE
 *          ArrayCommand broadcast - boundary coverage one below the split
 *          threshold exercised by ACT-3/ACT-7.
 *   ACT-7  Exactly 16 simultaneously-dirty channels - the minimal case that
 *          actually requires a split - produces exactly two ArrayCommand
 *          broadcasts, the first carrying the full 15-command cap and the
 *          second carrying exactly the 1 remaining channel (not truncated,
 *          not silently dropped, not reordered).
 */

#include "gtest/gtest.h"

extern "C" {
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "platform.h"

/* DSDL types used by dronecan.c handlers */
#include "uavcan.protocol.NodeStatus.h"
#include "uavcan.protocol.GetNodeInfo.h"
#include "uavcan.protocol.param.GetSet_res.h"
#include "uavcan.protocol.param.ExecuteOpcode_res.h"
#include "uavcan.protocol.RestartNode_res.h"

/* DSDL types this test decodes on the loopback receiver */
#include "uavcan.equipment.actuator.ArrayCommand.h"
#include "uavcan.equipment.actuator.Command.h"

/* Canard core and STM32 driver declarations */
#include "drivers/dronecan/libcanard/canard.h"
#include "drivers/dronecan/libcanard/canard_stm32_driver.h"

/* INAV headers pulled in by dronecan.c — included here so the types are
   available when we define stub globals below. */
#include "io/gps.h"
#include "sensors/battery_sensor_dronecan.h"
#include "fc/runtime_config.h"
#include "sensors/diagnostics.h"
#include "build/version.h"
#include "common/log.h"

/* Public API we test against */
#include "drivers/dronecan/dronecan.h"

/* Private state made non-static in UNIT_TEST builds */
extern uint8_t activeNodeCount;
extern dronecanNodeInfo_t nodeTable[];
extern CanardInstance canard; /* the FC's own (production, non-static) canard instance */

/* Private functions not exposed in dronecan.h, made non-static under UNIT_TEST */
bool shouldAcceptTransfer(const CanardInstance *ins,
                          uint64_t *out_data_type_signature,
                          uint16_t data_type_id,
                          CanardTransferType transfer_type,
                          uint8_t source_node_id);
void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer);

/* =========================================================================
 * Stubs — provide every symbol dronecan.c references that isn't supplied by
 * the compiled dependencies (dronecan.c, canard.c, DSDL .c files).
 * (Duplicated from dronecan_application_unittest.cc — this file compiles to
 * its own standalone test executable.)
 * ========================================================================= */

/* Controllable time source */
static uint32_t mock_time_ms = 0;
uint32_t millis(void) { return mock_time_ms; }

/* Arming state — dronecan.c reads this for send_NodeStatus vendor code */
uint32_t armingFlags = 0;

/* GPS config — provider != GPS_DRONECAN so all GPS handlers return early */
gpsConfig_t gpsConfig_System;
gpsConfig_t gpsConfig_Copy;

/* Hardware health — dronecan.c reads this in send_NodeStatus */
bool isHardwareHealthy(void) { return true; }

/* Logging — no-op, tests don't assert on logging output */
void _logf(logTopic_e topic, unsigned level, const char *fmt, ...) { (void)topic; (void)level; (void)fmt; }

/* GPS and battery DroneCAN receive stubs */
void dronecanGPSReceiveGNSSFix(const struct uavcan_equipment_gnss_Fix *p) { (void)p; }
void dronecanGPSReceiveGNSSFix2(const struct uavcan_equipment_gnss_Fix2 *p) { (void)p; }
void dronecanGPSReceiveGNSSAuxiliary(const struct uavcan_equipment_gnss_Auxiliary *p) { (void)p; }
void dronecanBatterySensorReceiveInfo(struct uavcan_equipment_power_BatteryInfo *p) { (void)p; }

void saveConfig(void) {}

/* Version strings declared in build/version.h */
const char* const shortGitRevision = "00000000";
const char* const compilerVersion  = "test";
const char* const targetName       = "TEST";
const char* const buildDate        = "Jan 01 2026";
const char* const buildTime        = "00:00:00";

/* =========================================================================
 * Loopback receiver — a second, independent CanardInstance representing a
 * remote DroneCAN actuator node. Captures/decodes whatever the FC's canard
 * instance actually transmits.
 * ========================================================================= */
static CanardInstance rxIns;
static int receivedArrayCommandCount = 0;
static struct uavcan_equipment_actuator_ArrayCommand lastArrayCommand;

/* Cumulative per-actuator_id tracking across every message received during
   a test, not just the last one - needed to verify a batched send (multiple
   ArrayCommand messages in the same cycle) covers every channel correctly. */
#define MAX_TRACKED_ACTUATOR_ID 32
static bool seenActuatorId[MAX_TRACKED_ACTUATOR_ID];
static float seenActuatorValue[MAX_TRACKED_ACTUATOR_ID];
static uint8_t seenActuatorCommandType[MAX_TRACKED_ACTUATOR_ID];

/* commands.len of each ArrayCommand message received during a test, in
   arrival order - needed to verify a split lands the expected way (e.g.
   "15 then 1", not some other split or an extra empty message) rather than
   just checking every channel eventually shows up somewhere. */
#define MAX_TRACKED_BATCHES 8
static uint8_t receivedBatchLengths[MAX_TRACKED_BATCHES];

static void resetActuatorTracking(void)
{
    memset(seenActuatorId, 0, sizeof(seenActuatorId));
    memset(seenActuatorValue, 0, sizeof(seenActuatorValue));
    memset(seenActuatorCommandType, 0, sizeof(seenActuatorCommandType));
    memset(receivedBatchLengths, 0, sizeof(receivedBatchLengths));
}

static bool rxShouldAccept(const CanardInstance *ins,
                            uint64_t *out_data_type_signature,
                            uint16_t data_type_id,
                            CanardTransferType transfer_type,
                            uint8_t source_node_id)
{
    (void)ins; (void)source_node_id;
    if (transfer_type == CanardTransferTypeBroadcast &&
        data_type_id == UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID) {
        *out_data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE;
        return true;
    }
    return false;
}

static void rxOnTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    (void)ins;
    if (transfer->data_type_id != UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID) {
        return;
    }
    struct uavcan_equipment_actuator_ArrayCommand msg;
    memset(&msg, 0, sizeof(msg));
    if (!uavcan_equipment_actuator_ArrayCommand_decode(transfer, &msg)) {
        lastArrayCommand = msg;
        if (receivedArrayCommandCount < MAX_TRACKED_BATCHES) {
            receivedBatchLengths[receivedArrayCommandCount] = (uint8_t)msg.commands.len;
        }
        receivedArrayCommandCount++;
        for (uint8_t i = 0; i < msg.commands.len; i++) {
            const struct uavcan_equipment_actuator_Command *cmd = &msg.commands.data[i];
            if (cmd->actuator_id < MAX_TRACKED_ACTUATOR_ID) {
                seenActuatorId[cmd->actuator_id] = true;
                seenActuatorValue[cmd->actuator_id] = cmd->command_value;
                seenActuatorCommandType[cmd->actuator_id] = cmd->command_type;
            }
        }
    }
}

/* STM32 CAN driver stubs. canardSTM32Transmit is the loopback point: every
   frame the FC's canard instance would put on the physical bus is instead
   handed straight to rxIns, as if received over CAN with zero latency. */
int16_t canardSTM32CAN1_Init(uint32_t b) { (void)b; return CANARD_OK; }
int16_t canardSTM32Receive(CanardCANFrame *f) { (void)f; return 0; }
uint32_t canardSTM32GetAndClearRxDropCount(void) { return 0; }
int16_t canardSTM32Transmit(const CanardCANFrame *f)
{
    canardHandleRxFrame(&rxIns, f, (uint64_t)mock_time_ms * 1000ULL);
    return 1; /* report success so dronecan.c pops the frame off its TX queue */
}
void    canardSTM32GetProtocolStatus(canardProtocolStatus_t *s) { memset(s, 0, sizeof(*s)); }
int32_t canardSTM32GetRxFifoFillLevel(void) { return 0; }
void    canardSTM32RecoverFromBusOff(void) {}
void    canardSTM32GetUniqueID(uint8_t id[16]) { memset(id, 0, 16); }

} /* extern "C" */

/* =========================================================================
 * Fixture
 * ========================================================================= */

class DroneCANActuatorOutputTest : public ::testing::Test {
protected:
    uint8_t txMemPool[1024];
    uint8_t rxMemPool[1024];
    uint64_t simTimeUs;

    void SetUp() override {
        activeNodeCount = 0;
        memset(nodeTable, 0, sizeof(dronecanNodeInfo_t) * DRONECAN_MAX_NODES);

        memset(&canard, 0, sizeof(canard));
        canardInit(&canard, txMemPool, sizeof(txMemPool),
                   onTransferReceived, shouldAcceptTransfer, NULL);
        canardSetLocalNodeID(&canard, 100);

        memset(&rxIns, 0, sizeof(rxIns));
        canardInit(&rxIns, rxMemPool, sizeof(rxMemPool),
                   rxOnTransferReceived, rxShouldAccept, NULL);

        receivedArrayCommandCount = 0;
        memset(&lastArrayCommand, 0, sizeof(lastArrayCommand));
        resetActuatorTracking();
    }

    /* Advance the DroneCAN task by `totalMs`, in `stepMs` increments, keeping
       mock_time_ms and dronecanUpdate's timeUs argument in sync. Mirrors how
       the real scheduler drives dronecanUpdate() from the main loop. */
    void runTaskFor(uint32_t totalMs, uint32_t stepMs) {
        for (uint32_t elapsed = 0; elapsed < totalMs; elapsed += stepMs) {
            mock_time_ms += stepMs;
            simTimeUs += (uint64_t)stepMs * 1000ULL;
            dronecanUpdate(simTimeUs);
        }
    }
};

/* =========================================================================
 * ACT-1: A servo value change should result in an ArrayCommand broadcast
 * carrying the new value, within one floor/ceiling window.
 *
 * Per docs.md: command_type=PWM (raw microsecond value, no conversion),
 * 1:1 servo-index -> actuator_id mapping is the documented common case
 * (servo[0] -> actuator_id 1).
 * ========================================================================= */
TEST_F(DroneCANActuatorOutputTest, ServoValueChangeTriggersArrayCommandBroadcast)
{
    /* Start well clear of any prior test's clock/1Hz-schedule state within
       this binary (dronecanState / next_1hz_service_at are private statics
       that persist across TEST_F cases and cannot be reset from here). */
    simTimeUs = 1000000ULL; /* 1s */
    mock_time_ms = (uint32_t)(simTimeUs / 1000ULL);

    /* Let the task settle into STATE_DRONECAN_NORMAL, establish a baseline
       servo value, and let it broadcast once — then reset the counter so
       only the value change under test is measured. */
    dronecanWriteServo(0, 1500); /* neutral, microseconds */
    runTaskFor(100, 2);
    receivedArrayCommandCount = 0;

    /* The value change under test. */
    dronecanWriteServo(0, 1700);

    /* Ceiling is servo_pwm_rate (>=50Hz => <=20ms period); floor is 25Hz
       (40ms period). 100ms covers several cycles of either. */
    runTaskFor(100, 2);

    ASSERT_GE(receivedArrayCommandCount, 1)
        << "no uavcan.equipment.actuator.ArrayCommand was broadcast after "
           "the servo output changed";

    ASSERT_GE(lastArrayCommand.commands.len, 1u);

    bool found = false;
    for (uint8_t i = 0; i < lastArrayCommand.commands.len; i++) {
        const struct uavcan_equipment_actuator_Command *cmd = &lastArrayCommand.commands.data[i];
        if (cmd->actuator_id == 1) {
            found = true;
            EXPECT_EQ(cmd->command_type, UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_PWM);
            EXPECT_FLOAT_EQ(cmd->command_value, 1700.0f);
        }
    }
    EXPECT_TRUE(found) << "no Command for actuator_id=1 (servo[0], 1:1 default mapping) "
                           "found in the broadcast ArrayCommand";
}

/* =========================================================================
 * ACT-2: A servo value held constant must still be re-broadcast at the
 * 25 Hz keepalive floor, so the receiving node's own command-timeout
 * watchdog (e.g. AP_Periph's 200ms SRV_CMD_TIME_OUT) never lapses.
 * ========================================================================= */
/* =========================================================================
 * HARNESS SANITY CHECK (not part of the feature contract): proves the
 * canardSTM32Transmit -> canardHandleRxFrame loopback actually delivers
 * frames, using the NodeStatus broadcast dronecan.c already implements
 * today (process1HzTasks -> send_NodeStatus). If this test failed to see
 * a NodeStatus, the 0-ArrayCommand result above would be suspect (harness
 * bug). This test accepts NodeStatus specifically via a second loopback
 * receiver instance so it doesn't interfere with rxIns's ArrayCommand-only
 * filter used by the real contract tests.
 * ========================================================================= */
static int receivedNodeStatusCount = 0;
extern "C" bool nsShouldAccept(const CanardInstance *ins, uint64_t *out_sig, uint16_t data_type_id, CanardTransferType transfer_type, uint8_t source_node_id) {
    (void)ins; (void)source_node_id;
    if (transfer_type == CanardTransferTypeBroadcast && data_type_id == UAVCAN_PROTOCOL_NODESTATUS_ID) {
        *out_sig = UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE;
        return true;
    }
    return false;
}
extern "C" void nsOnTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer) {
    (void)ins;
    if (transfer->data_type_id == UAVCAN_PROTOCOL_NODESTATUS_ID) {
        receivedNodeStatusCount++;
    }
}

TEST_F(DroneCANActuatorOutputTest, HarnessSanity_NodeStatusLoopbackWorks)
{
    /* Temporarily point rxIns at the NodeStatus-accepting callbacks to prove
       the loopback plumbing (canardSTM32Transmit capture -> canardHandleRxFrame)
       actually delivers frames end-to-end. */
    receivedNodeStatusCount = 0;
    static uint8_t sanityPool[1024];
    canardInit(&rxIns, sanityPool, sizeof(sanityPool), nsOnTransferReceived, nsShouldAccept, NULL);

    simTimeUs = 90000000ULL; /* 90s: distinct time base from ACT-1/ACT-2 */
    mock_time_ms = (uint32_t)(simTimeUs / 1000ULL);

    /* dronecanUpdate's STATE_DRONECAN_INIT->NORMAL transition schedules the
       first 1Hz tick (which sends NodeStatus) 1s after entering NORMAL. */
    runTaskFor(2500, 2);

    EXPECT_GE(receivedNodeStatusCount, 1)
        << "harness bug: even the already-implemented NodeStatus 1Hz broadcast "
           "was not observed via the loopback capture -- the ArrayCommand "
           "failures above cannot be trusted until this passes.";
}

TEST_F(DroneCANActuatorOutputTest, UnchangedServoStillBroadcastsAtFloorRate)
{
    /* Distinct time base from ACT-1/HarnessSanity, and larger than either,
       so this test doesn't depend on their leftover dronecanState/
       next_1hz_service_at/next_actuator_*_at statics (persistent across
       TEST_F cases within this binary). Time only moves forward in real
       flight, so a later-declared test must use a time base at or beyond
       where the previous test's runTaskFor() calls left those deadlines,
       or checks like `currentTimeUs >= next_actuator_floor_check_at` can
       stay permanently false for the rest of this test's run. */
    simTimeUs = 200000000ULL; /* 200s */
    mock_time_ms = (uint32_t)(simTimeUs / 1000ULL);

    dronecanWriteServo(0, 1500);
    runTaskFor(50, 2); /* let it settle once before measuring */
    receivedArrayCommandCount = 0; /* reset after settling */

    /* 200ms of simulated time with the value never changing. At a 25Hz
       (40ms) floor, expect at least 4 keepalive broadcasts. */
    runTaskFor(200, 2);

    EXPECT_GE(receivedArrayCommandCount, 4)
        << "expected >=4 ArrayCommand keepalive broadcasts (25Hz floor) "
           "over 200ms of an unchanged servo value; got "
        << receivedArrayCommandCount;
}

/* =========================================================================
 * ACT-3: More than one message's worth of dirty channels (the DSDL limit is
 * ACTUATOR_COMMANDS_PER_MESSAGE=15 per ArrayCommand) must be split across
 * two broadcasts in the same cycle, covering every channel correctly - not
 * truncated, corrupted, or silently dropped.
 *
 * Regression coverage: an earlier version of sendActuatorCommandBatch()
 * indexed the DSDL's fixed 15-element commands.data[] array by the raw
 * (0-17) channel/bit-scan index instead of a separate pack-position
 * counter, so any channel found at position >=15 wrote past the end of the
 * array - a stack buffer overflow that only 16+ simultaneously dirty
 * channels could trigger.
 * ========================================================================= */
TEST_F(DroneCANActuatorOutputTest, MoreThanFifteenDirtyChannelsSplitAcrossTwoBatches)
{
    /* Distinct, later time base than every prior test - see the comment on
       UnchangedServoStillBroadcastsAtFloorRate for why this matters. */
    simTimeUs = 300000000ULL; /* 300s */
    mock_time_ms = (uint32_t)(simTimeUs / 1000ULL);

    /* Let any stale forced-dirty state left over from a prior test's
       statics drain out before measuring. */
    runTaskFor(30, 2);
    receivedArrayCommandCount = 0;
    resetActuatorTracking();

    /* All 18 channels become dirty in the same instant - more than fits in
       one ArrayCommand message. */
    for (uint8_t servo = 0; servo < 18; servo++) {
        dronecanWriteServo(servo, (uint16_t)(1000 + servo));
    }

    /* One actuatorUpdateInterval (20ms) tick is enough - both batches are
       sent back-to-back within the same dronecanUpdate() call once due. */
    runTaskFor(30, 2);

    EXPECT_EQ(receivedArrayCommandCount, 2)
        << "expected exactly 2 ArrayCommand messages (15 + 3) for 18 "
           "simultaneously-dirty channels, got " << receivedArrayCommandCount;

    for (uint8_t servo = 0; servo < 18; servo++) {
        uint8_t actuatorId = servo + 1;
        ASSERT_TRUE(seenActuatorId[actuatorId])
            << "actuator_id " << (int)actuatorId << " (servo " << (int)servo
            << ") never appeared in any broadcast ArrayCommand";
        EXPECT_FLOAT_EQ(seenActuatorValue[actuatorId], (float)(1000 + servo))
            << "actuator_id " << (int)actuatorId << " had the wrong command_value";
        EXPECT_EQ(seenActuatorCommandType[actuatorId], UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_PWM);
    }
}

/* =========================================================================
 * ACT-4: A servo commanded to exactly 0 must never appear in a broadcast -
 * 0 is INAV's local-PWM "no pulse" sentinel (e.g. the tricopter-unarmed-
 * servo case), not a real command value a DroneCAN node should receive. A
 * channel dronecanWriteServo() is never called for at all (simulating a
 * servo outside the mixer's compacted range) must be excluded the same
 * way, since both cases collapse to the same value==0 condition by design.
 *
 * Regression coverage for two related bugs: broadcasting a literal
 * command_value=0 on disarm, and the floor check force-broadcasting
 * channels the mixer never actually configured (actuatorFloorWindowReset()
 * used to mark all 18 slots due regardless of whether dronecanWriteServo()
 * had ever touched them).
 * ========================================================================= */
TEST_F(DroneCANActuatorOutputTest, ZeroValueAndNeverWrittenChannelsAreExcluded)
{
    simTimeUs = 400000000ULL; /* 400s: later than every prior test's time base */
    mock_time_ms = (uint32_t)(simTimeUs / 1000ULL);

    runTaskFor(30, 2); /* let stale state from prior tests drain */
    receivedArrayCommandCount = 0;
    resetActuatorTracking();

    dronecanWriteServo(0, 1234); /* a real, nonzero command - should broadcast */
    dronecanWriteServo(1, 0);    /* explicit zero - must be excluded */
    /* servo index 2 (actuator_id 3) is never touched at all - must also be excluded */

    runTaskFor(30, 2);

    EXPECT_TRUE(seenActuatorId[1]) << "actuator_id 1 (a real nonzero command) never appeared";
    EXPECT_FLOAT_EQ(seenActuatorValue[1], 1234.0f);

    EXPECT_FALSE(seenActuatorId[2])
        << "actuator_id 2 (servo 1, explicitly commanded to 0) appeared in a "
           "broadcast - 0 should be treated as no-signal, not a real command";

    EXPECT_FALSE(seenActuatorId[3])
        << "actuator_id 3 (servo 2, never written at all) appeared in a "
           "broadcast - unused channels must stay silent";
}

/* =========================================================================
 * Helper for ACT-5/6/7: force every one of the 18 channels to a known,
 * deterministic value (0) before a boundary test sets up its own exact
 * dirty-channel count.
 *
 * Why this is needed: actuatorCommands[] and the actuatorDirty/
 * actuatorUpdateAtCheck bitarrays are private statics in
 * dronecan_actuator.c that persist for the lifetime of this whole test
 * binary, not just one TEST_F. Worse, the 25Hz floor mechanism
 * (actuatorFloorWindowReset()) unconditionally re-marks *every* one of the
 * 18 channels dirty on every ~20ms floor-check tick, regardless of whether
 * this test ever touched them - so any channel a *previous* TEST_F left at
 * a nonzero value keeps getting swept into every subsequent batch forever
 * (harmlessly, since floor re-dirtying of an untouched channel is by
 * design - see ACT-2). A boundary test that needs an *exact* simultaneously
 * -dirty count (15, 16) cannot tolerate that leftover contamination: it
 * would silently inflate the real count above what the test intends.
 * Driving every channel through dronecanWriteServo(servo, 0) first makes
 * every channel's real value 0 again, so leftover/floor-driven dirty bits
 * for channels outside this test's target set are skipped without
 * consuming an output slot (see the `data[*next].value != 0` guard in
 * sendActuatorCommandBatch) - exactly like ACT-4's "never written" case.
 * ========================================================================= */
static void zeroAllActuatorChannels(void)
{
    for (uint8_t servo = 0; servo < 18; servo++) {
        dronecanWriteServo(servo, 0);
    }
}

/* =========================================================================
 * ACT-5: A realistic middle count - 6 of the 18 possible channels dirty at
 * once, interspersed with channels that are never touched - fits in a
 * single ArrayCommand broadcast, and only the dirty channels appear.
 *
 * ACT-1 covers a single dirty channel and ACT-3 covers all 18 (split into
 * two messages); this fills the gap with the common real-world shape - a
 * handful of CAN servos on a board that doesn't populate every slot -
 * where "does it fit in one message, and are unused channels correctly
 * silent among the dirty ones" hadn't been exercised together.
 * ========================================================================= */
TEST_F(DroneCANActuatorOutputTest, SixOfEighteenChannelsDirtyFitsInSingleBatch)
{
    simTimeUs = 500000000ULL; /* 500s: later than every prior test's time base */
    mock_time_ms = (uint32_t)(simTimeUs / 1000ULL);

    runTaskFor(30, 2); /* let stale state from prior tests drain */
    receivedArrayCommandCount = 0;
    resetActuatorTracking();

    zeroAllActuatorChannels(); /* guarantee a known, deterministic baseline */

    const uint8_t dirtyServos[] = {0, 3, 6, 9, 12, 17};
    const uint16_t dirtyValues[] = {1100, 1200, 1300, 1400, 1500, 1600};
    for (size_t i = 0; i < sizeof(dirtyServos); i++) {
        dronecanWriteServo(dirtyServos[i], dirtyValues[i]);
    }

    runTaskFor(30, 2);

    EXPECT_EQ(receivedArrayCommandCount, 1)
        << "6 simultaneously-dirty channels out of 18 should fit in exactly "
           "1 ArrayCommand message, got " << receivedArrayCommandCount;

    for (size_t i = 0; i < sizeof(dirtyServos); i++) {
        uint8_t actuatorId = dirtyServos[i] + 1;
        ASSERT_TRUE(seenActuatorId[actuatorId])
            << "actuator_id " << (int)actuatorId << " (servo " << (int)dirtyServos[i]
            << ") never appeared in the broadcast ArrayCommand";
        EXPECT_FLOAT_EQ(seenActuatorValue[actuatorId], (float)dirtyValues[i])
            << "actuator_id " << (int)actuatorId << " had the wrong command_value";
    }

    /* Every channel NOT in dirtyServos must stay silent, including channels
       sitting between the dirty ones (e.g. servo 1, 2, 4, 5...). */
    bool isDirty[18] = {false};
    for (size_t i = 0; i < sizeof(dirtyServos); i++) {
        isDirty[dirtyServos[i]] = true;
    }
    for (uint8_t servo = 0; servo < 18; servo++) {
        if (isDirty[servo]) {
            continue;
        }
        uint8_t actuatorId = servo + 1;
        EXPECT_FALSE(seenActuatorId[actuatorId])
            << "actuator_id " << (int)actuatorId << " (servo " << (int)servo
            << ") was never written in this test but appeared in the broadcast";
    }
}

/* =========================================================================
 * ACT-6: Exactly 15 simultaneously-dirty channels (ACTUATOR_COMMANDS_PER_
 * MESSAGE, the DSDL per-message cap) must fit in exactly ONE ArrayCommand
 * broadcast - not two, and not an extra (even empty) wire message.
 *
 * Boundary coverage one channel below the split threshold exercised by
 * ACT-3/ACT-7. sendActuatorCommandBatch()'s pack loop exits this case
 * because the pack count reached the per-message cap (15), not because the
 * dirty-bit scan ran dry - the same code path sendActuatorCommandArray()
 * uses to decide "maybe more to send, try a second batch". This test
 * confirms that even though a second, internal sendActuatorCommandBatch()
 * call is triggered (a genuinely ambiguous case: filling the cap exactly
 * looks identical to filling the cap with more still queued, until the
 * second batch actually scans), the "only send if there is data to send"
 * guard means it produces zero wire traffic - so receivedArrayCommandCount
 * must stay at 1.
 * ========================================================================= */
TEST_F(DroneCANActuatorOutputTest, ExactlyFifteenDirtyChannelsFitInOneMessage)
{
    simTimeUs = 600000000ULL; /* 600s: later than every prior test's time base */
    mock_time_ms = (uint32_t)(simTimeUs / 1000ULL);

    runTaskFor(30, 2); /* let stale state from prior tests drain */
    receivedArrayCommandCount = 0;
    resetActuatorTracking();

    zeroAllActuatorChannels(); /* guarantee a known, deterministic baseline */

    for (uint8_t servo = 0; servo < 15; servo++) {
        dronecanWriteServo(servo, (uint16_t)(1900 + servo));
    }

    runTaskFor(30, 2);

    EXPECT_EQ(receivedArrayCommandCount, 1)
        << "exactly 15 simultaneously-dirty channels (the per-message cap) "
           "must produce exactly 1 ArrayCommand message on the wire, got "
        << receivedArrayCommandCount;

    if (receivedArrayCommandCount >= 1) {
        EXPECT_EQ(receivedBatchLengths[0], 15)
            << "the single ArrayCommand message should carry all 15 commands";
    }

    for (uint8_t servo = 0; servo < 15; servo++) {
        uint8_t actuatorId = servo + 1;
        ASSERT_TRUE(seenActuatorId[actuatorId])
            << "actuator_id " << (int)actuatorId << " (servo " << (int)servo
            << ") never appeared in the broadcast ArrayCommand";
        EXPECT_FLOAT_EQ(seenActuatorValue[actuatorId], (float)(1900 + servo))
            << "actuator_id " << (int)actuatorId << " had the wrong command_value";
    }

    for (uint8_t servo = 15; servo < 18; servo++) {
        uint8_t actuatorId = servo + 1;
        EXPECT_FALSE(seenActuatorId[actuatorId])
            << "actuator_id " << (int)actuatorId << " (servo " << (int)servo
            << ") was never written in this test but appeared in the broadcast";
    }
}

/* =========================================================================
 * ACT-7: Exactly 16 simultaneously-dirty channels - one more than the
 * per-message cap, the minimal case that actually requires a split - must
 * produce exactly two ArrayCommand messages: the first carrying the full
 * 15-command cap, the second carrying exactly the 1 remaining channel.
 *
 * Complements ACT-3 (18 dirty, split 15+3) by pinning down the smallest
 * possible split (15+1) and, via receivedBatchLengths[], verifying the
 * split lands exactly "15 then 1" rather than just checking that both
 * channels eventually show up somewhere across however many messages were
 * sent - which would not catch e.g. a message-count regression that still
 * happened to preserve every actuator_id/value.
 * ========================================================================= */
TEST_F(DroneCANActuatorOutputTest, ExactlySixteenDirtyChannelsSplitFifteenThenOne)
{
    simTimeUs = 700000000ULL; /* 700s: later than every prior test's time base */
    mock_time_ms = (uint32_t)(simTimeUs / 1000ULL);

    runTaskFor(30, 2); /* let stale state from prior tests drain */
    receivedArrayCommandCount = 0;
    resetActuatorTracking();

    zeroAllActuatorChannels(); /* guarantee a known, deterministic baseline */

    for (uint8_t servo = 0; servo < 16; servo++) {
        dronecanWriteServo(servo, (uint16_t)(1500 + servo));
    }

    runTaskFor(30, 2);

    ASSERT_EQ(receivedArrayCommandCount, 2)
        << "exactly 16 simultaneously-dirty channels must produce exactly 2 "
           "ArrayCommand messages (15 + 1), got " << receivedArrayCommandCount;

    EXPECT_EQ(receivedBatchLengths[0], 15)
        << "the first ArrayCommand message should carry the full 15-command cap";
    EXPECT_EQ(receivedBatchLengths[1], 1)
        << "the second ArrayCommand message should carry exactly the 1 "
           "remaining channel, not be dropped or padded";

    for (uint8_t servo = 0; servo < 16; servo++) {
        uint8_t actuatorId = servo + 1;
        ASSERT_TRUE(seenActuatorId[actuatorId])
            << "actuator_id " << (int)actuatorId << " (servo " << (int)servo
            << ") never appeared in any broadcast ArrayCommand";
        EXPECT_FLOAT_EQ(seenActuatorValue[actuatorId], (float)(1500 + servo))
            << "actuator_id " << (int)actuatorId << " had the wrong command_value";
    }

    for (uint8_t servo = 16; servo < 18; servo++) {
        uint8_t actuatorId = servo + 1;
        EXPECT_FALSE(seenActuatorId[actuatorId])
            << "actuator_id " << (int)actuatorId << " (servo " << (int)servo
            << ") was never written in this test but appeared in a broadcast";
    }
}
