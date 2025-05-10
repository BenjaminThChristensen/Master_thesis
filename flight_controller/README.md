# Flight Controller Setup - Kakute H7 Mini + PX4

## Hardware: Kakute H7 Mini
- Loaded with PX4 (v1.14 or compatible)
- Connected to Raspberry Pi Zero 2 W over UART

## Wiring Summary:
- Pi TX → Kakute RX (TELEM1)
- Pi RX ← Kakute TX (TELEM1)
- PI GND - Kakute GND
- BEC 5V to Pi GPIO (pin 2)
- GND to Pi GPIO (pin 6)

## PX4 Params
To use the same configuration as this project, upload the PX4 parameters to your board.
See `/params/px4_flight_config.param` for complete configuration used for this build.

