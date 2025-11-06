# MCUXpresso SDK USBPD Stack

USBPD Stack and PTN5110 TCPC Driver from the NXP MCUXpresso Suite of Software and Tools.

See: <https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-:MCUXPRESSO>

Extracted from version 2.9.0.

SDK_2_9_0_LPCXpresso54114-OM13588\middleware\usb\pd

## About

This is a device independent USBPD Stack. It is designed to work with
the Universal Serial Bus Type-C® Port Controller Interface
Specification (TCPC) as the interface phy, in particular the NXP
PTN5110. However the core state machine is device and phy independent.

This software was release to work with the NXP USB Type-C Shield 2
Demo Kit and the NXP LPC MCU series.

## PlatformIO Integration

You can treat this repository as a local PlatformIO library. The
following steps assume an existing PlatformIO project structure:

1. Place or reference the stack inside your `lib/` directory. Example:
  `git submodule add https://github.com/<your-fork>/USBPD-Stack.git lib/usbpd-stack`.
2. Add a minimal `lib/usbpd-stack/library.json` file so PlatformIO
  knows to compile the `pd/` sources:

  ```json
  {
    "name": "usbpd-stack",
    "version": "0.1.0",
    "build": {
     "srcFilter": "+<pd/> -<pd/compliance_test_report/>"
    }
  }
  ```

  Adjust the `srcFilter` if you only want to build a specific port
  directory.
3. In `platformio.ini`, tell PlatformIO where the library lives (if the
   project and stack are not in the same repository) and set the target
   macro for the port you want to build:

  ```ini
  [env:esp32s3]
  platform = espressif32
  board = esp32s3dev
  framework = espidf
  build_flags =
     -DPD_CONFIG_TARGET_ESP32S3=1
   lib_deps =
     file://lib/usbpd-stack
   
  [env:arduino]
  platform = atmelavr ; or another Arduino-capable platform
  board = uno
  framework = arduino
  build_flags =
     -DPD_CONFIG_TARGET_ARDUINO=1
   lib_deps =
     file://lib/usbpd-stack
  build_src_filter =
     +<*> -<pd/port/esp32s3/*>
  ```

  The optional `build_src_filter` example excludes the ESP32-specific
  files when you target Arduino so the build only scans the Arduino
  adapters.
4. Include the PD headers in your sources (for example `#include
  "usb_pd.h"`) and provide the platform configuration structure when
  creating PD instances (e.g. `pd_phy_esp32s3_config_t` or
  `pd_phy_arduino_config_t`).
5. For cooperative platforms such as Arduino, schedule the platform
  tick helper (see `pd/port/arduino/usb_pd_port_arduino.h`) from your
  main loop so the stack processes timer events.

## ESP32-S3 Port

This repository now contains a lightweight hardware abstraction that
targets the ESP32-S3. The port lives under `pd/port/esp32s3/` and
provides FreeRTOS based implementations for the operating-system,
I2C, GPIO, timer tick, and delay adapters used by the USB PD stack.

To use the stack on ESP32-S3:

- Provide a `pd_phy_esp32s3_config_t` structure when creating the PD
  instance so the port knows the CC alert GPIO and I2C pins.
- Ensure the new headers under `pd/` and `pd/port/esp32s3/` are added
  to your compiler include paths.
- Link the new source files in `pd/port/esp32s3/` into your ESP-IDF
  project so the abstractions are available at build time.

The other software from MCUXpresso is not include in this
repository. This USBPD Stack is independent of the MCUXpresso
middleware.

See:

- <https://www.nxp.com/products/interfaces/usb-interfaces/usb-type-c/usb-pd-phy-and-cc-logic/usb-pd-tcpc-phy-ic:PTN5110>
- <https://cache.nxp.com/docs/en/user-guide/UM11056.pdf>

NXP have released this as BSD-3-Clause software. This version is
unchanged from the NXP release.
