# USB PD Stack – Usage Guide

## Overview

This guide explains how to integrate the USB Power Delivery stack with the NXP
PTN5110 TCPC on non-NXP platforms. The repository is partitioned into clearly
scoped modules:

- `include/` – public headers for the core stack, PTN5110 driver and
  platform-agnostic configuration structures.
- `src/` – core C sources grouped by concern (`alt_mode/`, `ptn5110/`, policy,
  timers, …).
- `support/` – HAL abstraction headers (`support/include/`) and their
  implementations (`support/src/<platform>/`).
- `port/` – scheduler/tick glue headers (`port/include/`) with matching source
  files (`port/src/<platform>/`).
- `docs/` – user documentation and Ellisys compliance reports.

Adapters located under `support/src/<platform>/` supply the minimum glue (I²C,
GPIO, timers, mutexes) required by the core state machine, while the port layer
under `port/src/<platform>/` drives the PD timer callbacks for each platform.

## Prerequisites

- Familiarity with USB Type-C roles (Source, Sink, DRP) and PD negotiation.
- A PTN5110 connected through I²C with the ALERT pin wired to a GPIO capable of
  edge interrupts.
- A C toolchain for the target MCU (ESP-IDF, STM32Cube, Arduino, PlatformIO, …).
- Correct pull-ups on SDA/SCL (external or MCU internal pull-ups when supported).

## Key Components

- **Policy Engine (`src/usb_pd_policy.c`)** – Implements the PD state machine.
- **Device Policy Manager (`src/usb_pd_interface.c`)** – Creates PD instances,
  dispatches events and coordinates the power policy callbacks.
- **PTN5110 driver (`third_party/nxp_pd/src/ptn5110/`)** – Handles TCPC register access, ALERT
  interrupts and bus recovery.
- **Platform adapters (`support/src/<platform>/`)** – Provide HAL bindings for
  I²C, GPIO, OS primitives and timers.
- **Port glue (`port/src/<platform>/`)** – Own the scheduler hooks that
  periodically invoke `PD_TimerIsrFunction`.
- **Timers (`src/usb_pd_timer.c`)** – Keeps protocol timeouts and cooperates with
  the adapters to obtain a 1 ms tick.

## Build-Time Configuration

Configuration flags live in `include/usb_pd_config.h`. A helper section ensures
exactly one of the following is set to `1`:

- `PD_CONFIG_TARGET_ESP32S3`
- `PD_CONFIG_TARGET_ARDUINO`
- `PD_CONFIG_TARGET_STM32`

If none is defined the code defaults to ESP32-S3 for backwards compatibility.
Override the macros via compiler definitions or your build system (`build_flags`
in PlatformIO, `add_compile_definitions` in CMake, etc.). Other notable options
include `PD_CONFIG_MAX_PORT`, `PD_CONFIG_ALT_MODE_SUPPORT` and
`PD_CONFIG_PD3_PPS_ENABLE`.

## Typical Initialisation Flow

1. **Describe the hardware** using `pd_phy_<platform>_config_t`:
   - ESP32-S3: assign SDA/SCL GPIOs, desired I²C speed and FreeRTOS priorities.
   - STM32: provide `I2C_HandleTypeDef*`, ALT pin, IRQ priority and timeout.
   - Arduino: pass a `TwoWire*`, GPIO numbers and whether internal pull-ups are
     available.
2. **Fill the PD instance configuration** (`pd_instance_config_t`):
   - Choose roles (source/sink/DRP) and populate PDO tables.
   - Link `phyConfig` to the platform structure and set `phyType = kPD_PhyPTN5110`.
3. **Implement callbacks** for policy decisions and power management via
   `pd_stack_callback_t` and `pd_power_handle_callback_t`.
4. **Call `PD_InstanceInit`** and keep the returned `pd_handle`. The platform
   adapter will automatically register timers/interrupts when available.
5. **Drive the task loop**: run `PD_InstanceTask` in a FreeRTOS task or the main
   loop (Arduino) and make sure the shared 1 ms tick is active (see platform
   sections below).

## Platform Notes

### ESP32-S3

- Implemented on top of ESP-IDF FreeRTOS APIs. The adapter installs a 1 ms
  timer and uses I²C driver instance `i2c_instance` from `pd_phy_esp32s3_config_t`.
- Run `PD_InstanceTask` inside a dedicated task; the adapter registers the timer
  callback as soon as one instance is present.

### Arduino

- Cooperative design: call `PD_PortArduino_TaskTick()` and `PD_InstanceTask()`
  inside `loop()`.
- Uses `TwoWire` for I²C and basic critical sections for ALERT handling.

### STM32 (F3/F4/H7)

- Requires Cube HAL handles and FreeRTOS. Supply the HAL I²C pointer, ALERT GPIO
  information and IRQ priorities in `pd_phy_stm32_config_t`.
- The adapter takes care of EXTI dispatching through `PD_PortStm32_DispatchExti`.

## PlatformIO Example

```ini
[env:esp32s3]
platform    = espressif32
framework   = espidf
board       = esp32s3dev
build_flags = -DPD_CONFIG_TARGET_ESP32S3=1

[env:stm32h743]
platform    = ststm32
framework   = stm32cube
board       = disco_h743zi
build_flags = -DPD_CONFIG_TARGET_STM32=1
```

Include headers from `include/`, `port/include`, `support/include`, and
`third_party/nxp_pd/include`, then add the sources under `src/`, `port/src`,
`support/src`, and `third_party/nxp_pd/src` to your build.

## Diagnostics

- Enable debug macros in `usb_pd_config.h` to trace message exchanges.
- Use `PD_Control(..., PD_CONTROL_GET_PD_STATE, ...)` to inspect the current
  state machine node.
- The PTN5110 driver exposes helpers to read fault status and regulate VBUS.

## Additional Resources

- `README.md` – project summary and roadmap.
- `docs/compliance_test_report/` – Ellisys captures for reference.
- PTN5110 documentation – <https://www.nxp.com/products/interfaces/usb-interfaces/usb-type-c/usb-pd-phy-and-cc-logic/usb-pd-tcpc-phy-ic:PTN5110>
