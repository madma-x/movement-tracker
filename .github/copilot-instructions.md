# Copilot Instructions for movement-tracker Firmware

## Project Overview
This firmware targets the STM32G491KEUx MCU, providing movement tracking using an optical sensor (PAA5163/ADNS9800) and an IMU (BNO08x/BNO085). It communicates via I2C, SPI, UART, CAN, and USB CDC, exposing position and heading data for integration with external systems.

## Architecture & Data Flow
- **Entry Point**: `main.c` initializes hardware and calls `setup()` and `loop()` (see `runtime.c`).
- **Sensors**:
  - **Optical (PAA5163/ADNS9800)**: SPI, provides position deltas. See `paaReadMotion()` and `adnsX()/adnsY()`.
  - **IMU (BNO08x/BNO085)**: SPI, provides orientation (quaternion/yaw) and accuracy. See `bnoProcess()`, `bno_get_yaw()`.
- **Position Calculation**: Sensor fusion in `movement_tracker.c` (or inline in `runtime.c` for V5) combines optical deltas and IMU yaw, applying a relative angle offset.
- **I2C Register Map**: Position and heading are split into bytes and exposed via the `I2C_REGISTERS` array in `runtime.c`.
- **External Commands**: I2C commands allow resetting position, setting position/heading, and updating sensor gain (see `process_data()` in `runtime.c`).

## Key Files & Directories
- `firmware/Core/Src/main.c`: Hardware init, main loop.
- `firmware/Core/Src/runtime.c`: Setup, loop, I2C register handling, command processing.
- `firmware/Core/Src/movement_tracker.c`: Sensor fusion, position calculation (V4 and earlier).
- `firmware/Core/Src/PAA5163.h/c`, `BNO08x.h/c`, `ADNS9800.h/c`, `BNO085.h/c`: Sensor drivers.
- `firmware/Core/Inc/`: Header files for all modules.
- `firmware/Movement_Tracker.ioc`: STM32CubeMX hardware configuration.

## Build & Debug Workflow
- **Build System**: Use STM32CubeIDE or compatible toolchains. Regenerate code/config from `.ioc` if hardware changes.
- **Debugging**: UART (`huart2`) for debug output (`printf`). USB CDC enabled for device communication.
- **Peripheral Pins**: Chip select and reset pins for sensors must be set correctly before init (see `main.c`).

## Project-Specific Patterns
- **Sensor Fusion**: Position is calculated by combining optical sensor deltas with IMU yaw, applying a fixed relative angle offset.
- **I2C Register Protocol**: Floats are split into bytes for transmission. Address 0 is reserved for commands.
- **Debugging**: Enable sensor debug output by calling `adnsEnableDebugReports()` or similar (commented out by default).
- **Custom Gain**: Optical sensor gain can be set via I2C command (see `process_data()`).

## Integration Points
- **External Communication**: Position and heading exposed via I2C. USB CDC and CAN are initialized but not fully documented in code.
- **Sensor Drivers**: Custom drivers for PAA5163/ADNS9800 and BNO08x/BNO085 in `Core/Src` and `Core/Inc`.

## Example: Setting Position via I2C
Send a command packet with address 0 and command 0x02, followed by floats for theta, x, y, relative angle, and gain. See `process_data()` in `runtime.c` for details.

---

For unclear workflows, hardware setup, or integration, ask for clarification or provide missing details.
