# Project To-Do List

## Phase 1: Project Setup and Basic Connectivity
- [x] Initialize Zephyr Project Structure
- [x] Configure basic `prj.conf` for Bluetooth and GPIO
- [x] Implement Random Bluetooth Name Generation
- [x] Implement basic BLE Advertising
- [ ] Verify build on `nrf54l15dk/nrf54l05/cpuapp` target

## Phase 2: Hardware Interface Configuration (I2S & GPIO)
- [ ] Create Device Tree Overlay (`.overlay`) for custom pinout
    - [ ] Define I2S interface pins (SCK, LRCK, SDIN, SDOUT)
    - [ ] Define GPIO for Capacitive Touch Button
    - [ ] Define GPIOs for LEDs
- [ ] Implement I2S Driver Initialization in `main.c`
    - [ ] Configure I2S for bidirectional audio (Tx for Speaker, Rx for Mics)
    - [ ] Test audio loopback (Mic -> Speaker) to verify hardware

## Phase 3: Input and Control
- [ ] Implement Capacitive Touch Button Driver
    - [ ] Use Zephyr's GPIO or dedicated Touch controller driver
    - [ ] Implement Debounce logic
    - [ ] Map Touch events to Play/Pause actions (or internal state changes)

## Phase 4: Bluetooth Audio Implementation
- [ ] Research LE Audio (Unicast Server) implementation in Zephyr
    - *Note: NRF54L05 is BLE only, so Classic A2DP is not possible.*
- [ ] Configure `prj.conf` for LE Audio (BAP - Basic Audio Profile)
- [ ] Implement LE Audio capabilities (Sink for Speaker, Source for Mic)
- [ ] Integrate I2S audio stream with BLE Audio ISO channels

## Phase 5: Power Management
- [ ] Enable Low Power features (System ON/OFF)
- [ ] Optimize advertising intervals for coin cell battery life
- [ ] Implement battery level monitoring (SAADC) and Bluetooth Battery Service (BAS)

## Phase 6: Polish and LED Effects
- [ ] Implement LED patterns for different states (Pairing, Connected, Low Battery)
- [ ] Add sound effects (Chirp on button press)

## Phase 7: Testing
- [ ] Test pairing with LE Audio capable phone
- [ ] Test audio quality and latency
- [ ] Measure power consumption
