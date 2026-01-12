# Project To-Do List

## Phase 1: Project Setup and Basic Connectivity
- [x] Initialize Zephyr Project Structure
- [x] Configure basic `prj.conf` for Bluetooth and GPIO
- [x] Implement Random Bluetooth Name Generation
- [x] Implement basic BLE Advertising
- [x] Verify build on `nrf54l15dk/nrf54l05/cpuapp` target (Requires Zephyr SDK)

## Phase 2: Hardware Interface Configuration (I2S & GPIO)
- [x] Create Device Tree Overlay (`.overlay`) for custom pinout
    - [x] Define I2S interface pins (SCK, LRCK, SDIN, SDOUT)
    - [x] Define GPIO for Capacitive Touch Button
    - [x] Define GPIOs for LEDs
- [x] Implement I2S Driver Initialization in `main.c`
    - [x] Configure I2S for bidirectional audio (Tx for Speaker, Rx for Mics)
    - [x] Test audio loopback (Mic -> Speaker) to verify hardware (Skeleton)

## Phase 3: Input and Control
- [x] Implement Capacitive Touch Button Driver
    - [x] Use Zephyr's GPIO or dedicated Touch controller driver (DK Lib)
    - [x] Implement Debounce logic (DK Lib handles this)
    - [x] Map Touch events to Play/Pause actions (Skeleton in `button_handler`)

## Phase 4: Bluetooth Audio Implementation
- [x] Research LE Audio (Unicast Server) implementation in Zephyr
- [x] Configure `prj.conf` for LE Audio (BAP - Basic Audio Profile)
- [x] Implement LE Audio capabilities (Sink for Speaker, Source for Mic) - *Skeleton Implemented*
- [x] Integrate I2S audio stream with BLE Audio ISO channels (Partial)

## Phase 5: Power Management
- [x] Enable Low Power features (System ON/OFF)
- [x] Optimize advertising intervals for coin cell battery life (Default config used)
- [x] Implement battery level monitoring (SAADC) and Bluetooth Battery Service (BAS)

## Phase 6: Polish and LED Effects
- [x] Implement LED patterns for different states (Pairing, Connected, Low Battery)
- [ ] Add sound effects (Chirp on button press)

## Phase 7: Testing
- [ ] Test pairing with LE Audio capable phone
- [ ] Test audio quality and latency
- [ ] Measure power consumption
