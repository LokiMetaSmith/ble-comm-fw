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
    - LE Audio Unicast Server (Speaker/Mic)
- **Power**: 3V Lithium Coin Cell
    - Battery Service (BAS)
- **Indication**: LEDs with status patterns

## Project Structure
- `src/`: Source code
    - `main.c`: Application entry and integration
    - `le_audio.c`: Bluetooth LE Audio BAP implementation
    - `battery.c`: SAADC and Battery Service
    - `led_ctrl.c`: LED state machine
- `boards/`: Board definitions and overlays
- `prj.conf`: Kernel and subsystem configuration

## Prerequisites
- Zephyr SDK (0.16+)
- `west` tool
- Python 3

## Setup and Build

1. **Initialize Workspace** (if not already done):
   Since this project uses Nordic specific drivers (e.g. `dk_buttons_and_leds`), it is recommended to initialize the workspace using the nRF Connect SDK manifest.

   ```bash
   west init -m https://github.com/nrfconnect/sdk-nrf --mr main my-workspace
   cd my-workspace
   west update
   ```

2. **Build the Firmware**:
   We target the `nrf54l15dk/nrf54l05/cpuapp` board as a proxy for the NRF54L05 hardware.

   *Note: Ensure the Zephyr SDK is properly installed and `ZEPHYR_TOOLCHAIN_VARIANT` is set.*

   ```bash
   cd startrek-badge
   west build -b nrf54l15dk/nrf54l05/cpuapp
   ```

3. **Flash the Firmware**:
   ```bash
   west flash
   ```

## Notes
- The NRF54L05 is a BLE-only chip. "Bluetooth Speaker" functionality usually implies Bluetooth Classic (A2DP). This device uses **LE Audio** (Unicast).
- I2S pins are defined in `boards/nrf54l15dk_nrf54l05_cpuapp.overlay`.

## Status
Completed implementation of:
- Basic connectivity and Advertising.
- I2S Driver setup.
- Touch button handling.
- LE Audio skeleton.
- Battery monitoring.
- LED effects.

See [TODO.md](TODO.md) for detailed task list.
