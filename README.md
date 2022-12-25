# lois_ecu
This repository contains the hardware, firmware and tooling for a general purpose control unit for wheel-based robotics platforms. An image of the real hardware and a component diagram are shown below.
<p align="center">
  <img src=docs/img/realworld.jpg width="20%">
  <img src=docs/img/ecu.png width="60%">
</p>

## Firmware

The firmware implements currently the following features:

- Closed single loop RPM-control for two DC-motors with encoders
- Servo sweep for the [ball distribution device](https://github.com/generationmake/BDD)
- NMEA parser on RS232_1 (used for UBLOX GPS)
- NMEA parser on RS232_2 (used for KVH-C100 fluxgate compass)
- Host communication via UART and/ or USB

### Host communication protocol

The communication protocol is ASCII-based and the messages have the following format:

```:<8-bit command><32-bit payload><8-bit crc><\n>```

The table below shows the currently implemented messages.
| Command | To ECU (E)/ To Host (H) | Payload interpretation |
| - | - | - |
| 0x01 | E | ```int16``` dutycycle left, ```int16``` dutycycle right |
