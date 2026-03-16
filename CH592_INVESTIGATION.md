# CH592 Investigation Report

## Executive Summary
This report investigates the feasibility of adding support for the WCH CH592 (a RISC-V based BLE 5.4 MCU) to the `startrek-badge` project, alongside the existing Nordic NRF54L05.

**Conclusion:** It is currently **highly impractical** to add support for the CH592 to this codebase without a complete architectural rewrite and significant compromises to the core features (specifically LE Audio and I2S peripherals). The CH592 lacks the necessary hardware peripherals (I2S), has severe memory constraints (26KB SRAM vs NRF54L05's much larger RAM for LC3 encoding), and has zero support in the Zephyr RTOS ecosystem.

---

## 1. Hardware Capabilities: CH592 vs NRF54L05

The current project relies heavily on LE Audio (BAP/Isochronous channels) and hardware I2S for high-quality audio routing (2x Microphones, 1x Speaker).

| Feature | NRF54L05 (Current) | WCH CH592 (Proposed) | Impact / Notes |
| :--- | :--- | :--- | :--- |
| **Architecture** | ARM Cortex-M33 (likely) | 32-bit RISC-V4C (QingKe) | Different toolchains required. |
| **BLE Version** | 5.4 (Full LE Audio / ISO) | 5.4 (Baseband) | While the CH592 advertises BLE 5.4, it is unclear if the WCH proprietary stack supports Isochronous (ISO) channels or the Basic Audio Profile (BAP) necessary for LE Audio. |
| **Audio Interfaces** | Hardware I2S (Multi-channel) | **None** (Only SPI, UART, I2C) | **Critical Blocker:** The CH592 does not have an I2S peripheral. Audio would have to be bit-banged via SPI or GPIO, which is CPU intensive and prone to jitter, making high-quality audio impossible. |
| **Memory** | High (Sufficient for LC3 codec) | 26KB SRAM, 512KB Flash | **Critical Blocker:** 26KB of SRAM is severely insufficient for running the Zephyr RTOS, a Bluetooth Host/Controller stack, and the LC3 audio codec simultaneously. LC3 encoding alone typically requires 30-50KB of RAM depending on the frame size and complexity. |
| **Other Peripherals** | ADC, Touch, PWM | 12-bit ADC, TouchKey, PWM | Compatible for basic battery monitoring and touch input. |

## 2. Zephyr RTOS Support

The `startrek-badge` project is built entirely on the Zephyr RTOS, leveraging its extensive subsystems (Bluetooth Host, Audio framework, Device Tree, Kconfig).

*   **No Mainline Support:** There is currently no support for the WCH CH592 (or its predecessors like CH582/CH573) in the mainline Zephyr project.
*   **No Known Forks:** A search of the Zephyr ecosystem reveals no public forks or HAL integrations for WCH BLE chips.
*   **Proprietary Stack:** WCH typically provides their BLE protocol stack as a pre-compiled binary blob (`.a` file) designed to work with their own bare-metal SDK or FreeRTOS via MounRiver Studio. Integrating this blob into Zephyr's HCI-based Bluetooth architecture would require writing a custom HCI transport driver or reverse-engineering their baseband controller, which is a massive undertaking.

## 3. Architectural Impact of Dual-Chip Support

To support both the NRF54L05 (via Zephyr) and the CH592 (via WCH SDK/FreeRTOS), the project architecture would need to be fundamentally altered:

1.  **Drop Zephyr as the Core OS:** We could no longer rely on Zephyr's standard APIs (`bt_enable`, `i2s_write`, `device_get_binding`, etc.).
2.  **Custom HAL:** We would need to write a custom Hardware Abstraction Layer (HAL) to wrap the disparate APIs of Zephyr and the WCH SDK.
3.  **Feature Degradation:** Because the CH592 cannot support standard LE Audio or hardware I2S, the project would have to split its core functionality:
    *   *NRF Target:* Standard LE Audio, High-quality I2S beamforming.
    *   *CH592 Target:* Custom BLE GATT-based audio streaming (low quality, high latency), bit-banged SPI audio, no standard compatibility with LE Audio phones.

## 4. Recommendations and Next Steps

**Recommendation: Do not proceed with adding CH592 support to this repository.**

The hardware limitations of the CH592 (lack of I2S, extreme RAM constraints) make it unsuitable for an LE Audio Com Badge application. Furthermore, the lack of Zephyr support means it cannot be integrated into the current codebase without discarding the entire existing infrastructure.

If a lower-cost alternative to the NRF54L05 is required, it is highly recommended to look for SoCs that:
1.  Have native hardware I2S.
2.  Have at least 128KB - 256KB of SRAM to handle LE Audio and the LC3 codec.
3.  Are already supported by the Zephyr RTOS (e.g., NRF5340, NRF52840 (for classic/custom audio), or certain ESP32 variants with BLE 5.2+ support).
