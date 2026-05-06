# SUROC Main Avionics (Black Pill Port)

This project contains the merged flight computer firmware for the SUROC project, migrated from the STM32F103 (Blue Pill) to the STM32F411 (Black Pill) architecture.

## Overview

The firmware combines environmental sensing, IMU orientation tracking, GPS data acquisition, and LoRa telemetry transmission into a unified 10Hz flight loop.

### Hardware Mapping (STM32F411)
- **BME680 (Environment Sensor)**
  - Bus: `I2C2`
  - SCL: `PB10`
  - SDA: `PB3`
- **BNO085 (IMU)**
  - Bus: `I2C1`
  - SCL: `PB8`
  - SDA: `PB9`
  - INT: `PB4`
  - RST: `PA15`
  - BOOT: `PB5`
- **LoRa E22 (Telemetry)**
  - Bus: `USART2`
  - TX/RX: `PA2 / PA3`
  - AUX: `PA1`
- **GPS (Adafruit)**
  - Bus: `USART6`
  - TX/RX: `PA11 / PA12`
- **Debug Console**
  - Bus: `USART1`
  - TX/RX: `PB6 / PB7`
  - Baud: `115200`
- **Status LED**
  - Pin: `PC13` (Active-Low)

## Architecture

1. **Sensor Acquisition Loop (`main_avionic_loop`)**: Continuously polls the BNO085 via the Hillcrest SH-2 library, calculating quaternions and Euler angles (`BNO080_Pitch`, `Roll`, `Yaw`). It also polls the BME680 via the Bosch BME68x API, converting pressure to altitude (`current_alt_m`).
2. **GPS Background Processing**: Uses USART6 RX interrupts to buffer NMEA sentences, parsing them in the main loop via `GPS_Process()`.
3. **Flight Logic & Kalman Filter**: Sensor outputs (`raw_alt`, `raw_pitch`) feed into the flight state machine, which triggers drogue and main parachute deployment (via MOSFETs).
4. **Telemetry Loop**: A strict 10Hz (100ms) timer bundles all processed data into a tightly packed `GroundTelemetry_t` struct and streams it via `USART2` DMA to the ground station.

## Compilation & Flashing

The project is configured to use a standard GCC ARM toolchain and GNU Make.

### Prerequisites
You need the GNU ARM Embedded Toolchain (`arm-none-eabi-gcc`) installed and added to your system `PATH`. 
Alternatively, you can import this directory directly into **STM32CubeIDE** as an existing "Makefile Project".

### Building via Terminal
Open your terminal in the `suroc_black_pill_merged` directory and run:
```bash
make clean
make
```

### Flashing
The resulting binary will be located in `build/Roket.bin` and `build/Roket.elf`. You can flash the `.bin` using STM32CubeProgrammer over ST-Link or DFU mode.

```bash
# Example using st-flash utility
st-flash write build/Roket.bin 0x8000000
```

## Troubleshooting

- **Hard Fault on Boot**: Ensure the BNO085 is properly connected. If the INT (`PB4`) pin triggers an interrupt but the I2C wires are disconnected, it may lock the state machine.
- **BME680 Initialization Fails**: Check that the BME680 is connected to `I2C2` (`PB10`/`PB3`), not `I2C1`. The console will print `Probe chip ID at 0x77...` via USART1.
- **No Telemetry over LoRa**: Verify `PA1` (`LORA_AUX`) is properly transitioning. The DMA transfer waits for AUX to go high before transmitting the packet.
