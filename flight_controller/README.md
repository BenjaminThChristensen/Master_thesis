# Flight Controller Setup - Kakute H7 Mini + PX4

## Hardware: Kakute H7 Mini
- Loaded with PX4 (v1.14 or compatible)
- Connected to Raspberry Pi Zero 2 W over UART

## Wiring Summary:
- Pi TX → Kakute RX (TELEM1)
- Pi RX ← Kakute TX (TELEM1)
- Shared GND
- BEC 5V to Pi GPIO (pin 2)
- GND to Pi GPIO (pin 6)

## PX4 Params
See `/params/px4_flight_config.param` for complete configuration used for this build.

## PX4 Settings
- `SERIAL1_BAUD = 57600`
- `SERIAL1_PROTOCOL = MAVLink 1`
- `SDLOG_MODE = 1`
- `MAV_1_CONFIG = TELEM1`
