# Changelog

## [Unreleased]

- Document pending work after recent restructuring.
- Move platform ports and HAL support modules into top-level `port/` and
  `support/` directories with dedicated include/src separation.
- Relocate the PTN5110 driver sources and headers to `third_party/nxp_pd/` to
  clearly separate inherited NXP code from STLINE additions.

## [0.1.0] - 2025-11-06

- Reorganized sources under `src/` with matching public headers in `include/`.
- Added FreeRTOS-based port layers for ESP32-S3 and STM32 platforms plus Arduino cooperative loop support.
- Imported PTN5110 HAL and policy engine from the original NXP stack.
