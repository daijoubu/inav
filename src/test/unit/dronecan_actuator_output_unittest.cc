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
        receivedArrayCommandCount++;
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
