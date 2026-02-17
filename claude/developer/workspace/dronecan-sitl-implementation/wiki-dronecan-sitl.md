# DroneCAN SITL Testing Guide

This guide explains how to test DroneCAN functionality using INAV SITL (Software In The Loop).

## Overview

DroneCAN is now supported in INAV SITL with the following features:

- **Real CAN Protocol Testing** (Linux): Uses SocketCAN for actual CAN frame transmission
- **Stub Fallback** (All Platforms): Graceful degradation for non-Linux platforms
- **Multi-Node Testing**: Multiple SITL instances can communicate via virtual CAN
- **External Tool Integration**: Use standard CAN tools like `candump` and `cansend`

## Prerequisites

### Linux (Recommended)
- Linux with kernel CAN support (most modern distributions)
- `can-utils` package: `sudo apt-get install can-utils`
- INAV SITL built with DroneCAN support

### macOS / Windows
- DroneCAN runs in stub mode (no actual CAN frames)
- Still useful for testing DroneCAN application logic

## Setup

### 1. Build SITL with DroneCAN Support

```bash
cd inav
mkdir -p build_sitl
cd build_sitl
cmake -DSITL=ON ..
make SITL.elf -j4
```

### 2. Create Virtual CAN Interface (Linux Only)

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

Verify the interface:
```bash
ip link show vcan0
```

### 3. Start SITL

```bash
cd inav/build_sitl
./bin/SITL.elf
```

SITL will automatically detect the vcan0 interface and initialize DroneCAN in SocketCAN mode. If the interface is not available, it falls back to stub mode.

## Testing Procedures

### Basic DroneCAN Initialization

1. **Start SITL** (as shown above)
2. **Connect with INAV Configurator** via `tcp://127.0.0.1:5760`
3. **Enable DroneCAN** in the Configurator:
   - Go to Configuration tab
   - Find "DroneCAN" section
   - Set `dronecan_enabled = ON`
   - Click "Save and Reboot"

4. **Verify DroneCAN is running**:
   - Check SITL console output for messages like:
     ```
     DroneCAN SITL driver initialized (SocketCAN on vcan0)
     SocketCAN initialized on vcan0 at 1000000 bps
     ```

### Linux: Monitoring CAN Traffic

Open a new terminal and monitor the vcan0 interface:

```bash
candump vcan0
```

You should see DroneCAN frames being transmitted, including NodeStatus messages from the SITL instance.

Example output:
```bash
vcan0  107FF0A3   [8]  01 00 00 00 00 00 00 00
vcan0  107FF0A3   [8]  02 00 00 00 00 00 00 00
```

### Multi-Node Testing

You can run multiple SITL instances to simulate multiple DroneCAN nodes:

**Terminal 1 (Node 1):**
```bash
cd inav/build_sitl
./bin/SITL.elf
```

**Terminal 2 (Node 2):**
```bash
cd inav/build_sitl2
./bin/SITL.e lf
```

Each instance will:
- Get a unique node ID based on process ID and timestamp
- Transmit its own NodeStatus messages
- Receive and process messages from other nodes

Monitor all traffic:
```bash
candump vcan0
```

### Sending Test Frames

Use `cansend` to inject test frames:

```bash
# Send a standard frame
cansend vcan0 123#DEADBEEF

# Send an extended frame (DroneCAN uses extended IDs)
cansend vcan0 107FF0A3#1122334455667788

# Send with data length
cansend vcan0 12345678#1122334455667788
```

### Testing with CLI

Connect to SITL via MSP and use CLI commands:

```bash
# Check DroneCAN status
status

# View DroneCAN parameters
dronecan

# Enable/disable DroneCAN
set dronecan_enabled = ON
save
```

## Advanced Testing

### Stress Testing

Monitor SITL performance under heavy CAN traffic:

```bash
# Generate high frame rate in one terminal
genload() { while true; do cansend vcan0 123#1122334455667788; done; }

# Start multiple generators
genload & genload & genload &

# In another terminal, run SITL and monitor
/top -p $(pidof SITL.elf)
```

### Testing Node Discovery

1. Start SITL Node 1
2. Wait 30 seconds for it to stabilize
3. Start SITL Node 2
4. Monitor candump output for node discovery messages
5. Verify both nodes appear in each other's node tables

### Error Testing

Test error handling:

```bash
# Bring interface down (should cause graceful fallback)
sudo ip link set vcan0 down

# Bring it back up
sudo ip link set vcan0 up

# Create invalid frames
cansend vcan0 1FFFFFFF#1122334455667788  # Invalid ID
cansend vcan0 123#                        # No data
```

## Troubleshooting

### SITL Cannot Initialize SocketCAN

**Symptom**: "SocketCAN initialization failed, falling back to stub mode"

**Solutions**:
1. Check vcan0 exists: `ip link show vcan0`
2. Create it if missing: `sudo ip link add dev vcan0 type vcan`
3. Bring it up: `sudo ip link set vcan0 up`
4. Check permissions: `ls -l /dev/can*`

### No Traffic on vcan0

**Symptom**: `candump vcan0` shows no output

**Solutions**:
1. Verify DroneCAN is enabled in SITL:
   ```bash
   # In CLI
   get dronecan_enabled
   ```

2. Check SITL logs for errors
3. Ensure vcan0 is up: `ip link set vcan0 up`
4. Try restarting SITL

### Stub Mode on Linux

**Symptom**: "SITL DroneCAN driver initialized (stub mode)"

**Solutions**:
1. Check vcan0 interface exists
2. Verify socket permissions
3. Check kernel has CAN support: `lsmod | grep can`

### Build Errors

**Symptom**: SITL fails to build with DroneCAN

**Solutions**:
1. Clean build directory: `rm -rf build_sitl`
2. Re-run cmake: `cmake -DSITL=ON ..`
3. Check for missing dependencies
4. Verify USE_DRONECAN is defined in target.h

## Platform-Specific Notes

### Linux (SocketCAN Mode)
- Full CAN protocol testing
- Real frame transmission/reception
- Multi-node support
- External tool integration

### macOS (Stub Mode)
- No actual CAN frames
- DroneCAN application logic still runs
- Useful for development testing

### Windows (Stub Mode)
- Same as macOS
- Use WSL2 for Linux/SocketCAN support

## API Reference

### Driver Modes

The SITL CAN driver operates in two modes:

1. **SocketCAN Mode** (Linux only, preferred)
   - Real CAN frames via kernel
   - Multi-process communication
   - External tool support

2. **Stub Mode** (fallback)
   - No actual transmission
   - Always returns success
   - For non-Linux platforms

### Configuration

In `src/main/target/SITL/target.h`:

```c
#define USE_DRONECAN                    // Enable DroneCAN support
#define DRONECAN_SITL_INTERFACE "vcan0"  // CAN interface name
```

### SocketCAN Interface Selection

The interface name can be changed:

```c
// In target.h or via CLI
#define DRONECAN_SITL_INTERFACE "can0"    // Physical CAN interface
#define DRONECAN_SITL_INTERFACE "vcan1"   // Alternative virtual interface
```

## Performance Notes

- SocketCAN mode adds minimal overhead (< 1% CPU)
- Stub mode has virtually no overhead
- Frame conversion is efficient (no copying where possible)
- Non-blocking I/O prevents stalls

## Example Test Script

For automated testing, see `test_dronecan_sitl.sh` in the repository:

```bash
#!/bin/bash
# Setup vcan0 and test DroneCAN SITL

# Create vcan interface
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up

# Start SITL
./bin/SITL.elf &

# Monitor traffic
candump vcan0 &

# Send test frame
cansend vcan0 123#DEADBEEF
```

## Summary

DroneCAN SITL support enables comprehensive testing of DroneCAN functionality without hardware:

- ✅ Real CAN protocol testing on Linux
- ✅ Multi-node communication
- ✅ External tool integration
- ✅ Graceful fallback for all platforms
- ✅ Minimal performance impact
- ✅ Full DroneCAN feature support

For questions or issues, see the DroneCAN SITL implementation project documentation.
