# Star Trek Com Badge Firmware

This project contains the firmware for a Star Trek Com Badge replica based on the Nordic NRF54L05-QFAA-R SoC.

## Features
- **Hardware**: NRF54L05-QFAA-R (BLE SoC)
- **Audio**:
    - 2x I2S Microphones
    - 1x I2S Speaker
- **Input**: Capacitive Touch Button (Play/Pause)
- **Connectivity**: Bluetooth Low Energy (BLE)
    - Random Bluetooth Device Name on startup
    - Acts as a Bluetooth Speaker/Microphone (LE Audio target)
- **Power**: 3V Lithium Coin Cell
- **Indication**: LEDs

## Project Structure
- `src/`: Source code
- `boards/`: Board definitions and overlays
- `prj.conf`: Kernel and subsystem configuration

## Prerequisites
- Zephyr SDK
- `west` tool
- Python 3

## Setup and Build

1. **Initialize Workspace** (if not already done):
   ```bash
   west init -m https://github.com/zephyrproject-rtos/zephyr --mr main my-workspace
   cd my-workspace
   west update
   ```

2. **Build the Firmware**:
   We target the `nrf54l15dk/nrf54l05/cpuapp` board as a proxy for the NRF54L05 hardware.
   ```bash
   cd startrek-badge
   west build -b nrf54l15dk/nrf54l05/cpuapp
   ```

3. **Flash the Firmware**:
   ```bash
   west flash
   ```

## Notes
- The NRF54L05 is a BLE-only chip. "Bluetooth Speaker" functionality usually implies Bluetooth Classic (A2DP). This device will likely use **LE Audio** (Unicast) which requires a compatible source (e.g., modern smartphone with LE Audio support).
- I2S pins need to be configured in the device tree overlay.

## TODO
See [TODO.md](TODO.md) for the implementation roadmap.
