
## Archived: 2026-11-02
* Battery monitoring is implemented using the SAADC peripheral (node `&adc`) to feed the standard Bluetooth Battery Service (BAS).
* The beamforming implementation defaults to 'Broadside' summation (Left + Right / 2) with 0 delay samples, utilizing a circular buffer for delay handling.
* The MAX98357A audio amplifier control (SD_MODE) is mapped to GPIO P1.05, enabled via `CONFIG_AUDIO_MAX98357A`, and aliased as `audio-amp`.
* The I2S transmitter is pre-loaded with silence blocks upon initialization to prevent buffer underruns.
* The project initialization requires the nRF Connect SDK manifest (`https://github.com/nrfconnect/sdk-nrf`) to support Nordic-specific drivers like `dk_buttons_and_leds`.
* The source code in `le_audio.c` is guarded by `#if defined(CONFIG_BT_BAP_UNICAST_SERVER)` and provides stubs when LE Audio is disabled.
* The play/pause button (DK_BTN3) in `main.c` triggers `le_audio_toggle_mute()` to toggle the MICS mute state.
* The project is firmware for a NRF54L05-QFAA-R (Startrek comm badge) using Zephyr RTOS.
* The capacitive touch button is mapped to GPIO P1.08 and aliased as `touch-button`.
* The LE Audio stream uses the LC3 codec configured for a 16kHz sampling rate and 10ms frame duration.
* The build target for the NRF54L05 SoC is `nrf54l15dk/nrf54l05/cpuapp`.
* The application source code is modularized into `main.c` (integration), `le_audio.c` (Bluetooth BAP), `battery.c` (ADC/BAS), `led_ctrl.c` (status LEDs), and `audio_proc.c` (Audio DSP).
* The user prefers using the nRF SDK and Command Line Interface (CLI) for programming and compiling.
* Development on the DK board uses `DK_BTN3` as a logical proxy for the capacitive touch button (`&button2`) due to library mappings.
* The Microphone Control Service (MICS) Server is enabled (`CONFIG_BT_MICS=y`) to handle 'Tap to Talk' functionality by managing the mute state.
* A secondary build target `nrf52840dk/nrf52840` is configured with LE Audio (BAP) features disabled (`CONFIG_BT_BAP_UNICAST_SERVER=n`) to support development on limited hardware.
* Noise cancellation is implemented in `audio_proc.c` using a Delay-and-Sum beamforming algorithm that processes stereo interleaved input into mono output.
* The I2S interface is configured for 2 channels (stereo), requiring a `PCM_BLOCK_SIZE` of 640 bytes (2 channels * 160 samples * 2 bytes).
* Hardware configuration includes 2x I2S microphones, 1x I2S speaker, a capacitive touch button, and LEDs.
* The audio data path processes stereo I2S RX (Microphones) into mono via beamforming for LE Audio transmission, and duplicates mono LE Audio reception into stereo for I2S TX (Speaker).
* The battery monitoring ADC channel is configured with a 1/6 gain to support the 3V coin cell voltage range against the 0.6V internal reference.
* The NRF54L05 is BLE-only, requiring LE Audio implementation for speaker/microphone functionality.
* When the microphone is muted via MICS, `le_audio_send` transmits a buffer of zeroes (silence) to maintain the ISO stream timing.
* The user prefers to search for and utilize existing libraries before implementing custom solutions for new features.
* For the `nrf52840dk/nrf52840` target, the I2S interface uses instance `&i2s0` mapped to pins P1.01 (SCK), P1.02 (LRCK), P1.03 (SDIN), and P1.04 (SDOUT).
* The I2S interface for the NRF54L05 target is configured on instance `&i2s20` using pins SCK (P1.01), LRCK (P1.02), SDIN (P1.03), and SDOUT (P1.04).
* The I2S memory slab (`i2s_mem_slab`) is sized with 16 blocks to accommodate queuing of generated audio effects (chirps) alongside streaming data.
* User feedback for the touch button is provided by a 200ms LED flash (all LEDs) and a locally synthesized two-tone audio chirp (2kHz/1kHz) sent to the I2S speaker.
* The Bluetooth LE Audio implementation is configured as a Unicast Server (BAP) supporting both Sink and Source roles.
* The LE Audio transmission buffer (`tx_pool`) is sized to support raw PCM frames (350 bytes) to facilitate testing prior to full LC3 encoding integration.
* The project investigated adding support for the WCH CH592 microcontroller but concluded it is highly impractical due to the chip's lack of hardware I2S interfaces, insufficient SRAM (26KB) for the LC3 codec, and an absence of mainline Zephyr OS support.
