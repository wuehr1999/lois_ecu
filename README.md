# lois_ecu
This repository contains the [hardware](https://github.com/wuehr1999/lois_ecu/tree/main/hw), [firmware](https://github.com/wuehr1999/lois_ecu/tree/main/firmware) and [tooling](https://github.com/wuehr1999/lois_ecu/tree/main/rpmctrl_tuning) for a general purpose control unit for differential drive based wheeled robotics platforms. An image of the real hardware and a component diagram are shown below.
<p align="center">
  <img src=docs/img/realworld.jpg width="20%">
  <img src=docs/img/ecu.png width="60%">
</p>

## Firmware

The [firmware](https://github.com/wuehr1999/lois_ecu/tree/main/firmware/loisECU) comes as a STM32 CubeIDE project and currently implements the following features:

- Closed single loop RPM-control for two DC-motors with encoders
- Realtime calculation of differential drive odometry
- Servo sweep for the [ball distribution device](https://github.com/generationmake/BDD)
- NMEA parser on RS232_1 (used for UBLOX GPS)
- NMEA parser on RS232_2 (used for KVH-C100 fluxgate compass)
- Host communication via UART and/ or USB
- 2048 Byte reealtime reord buffer ( recording of RPM, torque and compass heading over a given time )
- Easy human readable and debuggable ASCII communication leaned on modbus protocol with optional checksum

### Host communication protocol

The communication protocol is ASCII-based and the messages have the following format:

```:<8-bit command><32-bit payload><8-bit crc><\n>```

All values are written in hexadecimal, so that every message has a fixed length of 14 characters. There is also a [ROS2 driver](https://github.com/wuehr1999/lois_ecu_ros), which uses the given protocol to make the functionalities available via ROS.

The table below shows the currently implemented messages. There are also some periodic messages from the ECU to the Host. By writing the commands of this messages to the ECU, the period can be changed. The payload is then interpreted as an ```uint16``` containing the period time in milliseconds. ```0xfffe``` then means requesting a periodic message once and ```0xffff``` turns the message off completely.

| Command | Function | To ECU (E)/ To Host (H) | Payload interpretation | Periodic |
| - | - | - | - | - |
| ```0x01``` | Dutycycle of left and right motor, RPM controller gets disabled | E | ```2 x int16``` ( -100 % - 100 %)  | |
| ```0x02``` | RPM of left and right motor, RPM controller gets enabled | E | ```2 x int16``` | |
| ```0x03```  | Turn board LED on or off | E | ```bool``` | |
| ```0x04``` | Record a given amount of RPM samples | E / H | ```2 x uint16``` ( m samples left and n right ) to ECU or sequential max(m, n) ``` 2 x uint16``` ( tick timestamps in 100 ns followed by ```0x00000000``` ) to Host |  |
| ```0x05``` | Emergency stop button pressed | H | ```bool``` |  |
| ```0x06``` | Kp left controller | E | ```float32``` |  |
| ```0x07``` | Tn left controller | E | ```float32``` |  |
| ```0x08``` | Td left controller | E | ```float32``` |  |
| ```0x09``` | Kp right controller | E | ```float32``` |  |
| ```0x0a``` | Tn right controller | E | ```float32``` |  |
| ```0x0b``` | Td right controller | E | ```float32``` |  |
| ```0x0c``` | Record a given amount of compass heading samples | E | ```uint32```  n samples to ECU or sequential n + 1 ```2 x int16``` ( heading data in degrees ) followed by ```0x016A016A``` to Host |  |
| ```0x0d``` | GPS position | E / H | 3 sequential ```uint32, 2 x float32``` ( ```0xffffffff```, latitude, longitude ) | x |
| ```0x0e``` | GPS time | E / H | ```3 x uint8``` ( hours, minutes, seconds ) | x |
| ```0x0f``` | GPS date | E / H | ```3 x uint8``` ( year, month, day ) | x |
| ```0x10``` | Compass heading | E / H | ```1 x int16``` ( degrees to north ) | x |
| ```0x11``` | Terminal mode ( forward serial sensor Rx input on this interface ) | E | ```uint8``` ( Host protocol, GPS sensor, compass sensor ) | |
| ```0x12``` | Encoder overall ticks | E / H | 4 sequential ```4 x uint32``` ( ```0xffffffff```, timestamp in nanoseconds, left encoder, right encoder )  | x |
| ```0x13``` | Left encoder short level time correction | E | ```float32``` |  |
| ```0x14``` | Left encoder long level time correction | E | ```float32``` |  |
| ```0x15``` | Right encoder short level time correction | E | ```float32``` |  |
| ```0x16``` | Right encoder long level time correction | E | ```float32``` |  |
| ```0x17``` | Record a given amount of Torque samples | E / H | ```2 x uint16``` ( m samples left and n right ) to ECU or sequential max(m, n) ``` 2 x uint16``` ( torque 12-bit ADC values followed by ```0x00000000``` ) to Host |  |
| ```0x18``` | Reset all controllers, interfaces and counters | E  | - |  |
| ```0x19``` | Use ball distribution device once | E | - |  |
| ```0x20``` | Odometry calculated on robot | E / H | sequential 7 started with ```0xffffffff``` followed by ```6 x float32``` ( x, y, phi, dx, dy, dphi ) | x |

## Encoder and RPM controller tuning

There is [tooling in Scilab](https://github.com/wuehr1999/lois_ecu/tree/main/rpmctrl_tuning) provided, that can be used to find the parameters for the implemented RPM controller. The first step is recording a step response with full torque/ dutycycle and data of free spinning wheels at a constant dutycycle and  saving the data as ```*.csv``` files. This can for instance be done by using the ROS ecosystem of lois. There are also some examples in the controller tuning folder.

### Encoder correction

The ECU determines the RPM values of the robot by measuring the time of high and low levels of the encoder signals. With this implementation, also encoders with fewer steps ( usually mounted directly on the wheel ) can be used with a sufficient resolution. The problem is, that some encoders have inequal high and low times. Therefore [correction factors](https://github.com/wuehr1999/lois_ecu/blob/main/rpmctrl_tuning/edgecorrect/correct.sce) can be calculated and entered at runtime ( commands ```0x13 - 0x16``` ) from the free spinning record.

### Finding the controller parameters

The controller is updated in the ECU with a frequency of ```5 Hz```. It is only a single loop RPM controller without a cascade for torque control, as it is not necessary for most differential drivetrain applications.

There is one band limited PI controller for each motor with the transfer function ```G(s) = Kp * ( 1 + s * Tn ) / ( s * Tn * ( 1 + s * Td ))```.

The Scilab tooling helps to [approximate the plant from the step response as 1st or 2nd order system and find sufficient controller settings](https://github.com/wuehr1999/lois_ecu/blob/main/rpmctrl_tuning/stepresponse/main.sce). The tuning paradigm is the compensation of the most critical pole of the plant followed by gain optimization. The following [goal parameters](https://github.com/wuehr1999/lois_ecu/blob/main/rpmctrl_tuning/stepresponse/init.sce) that can be selected:
- Encoder steps per revolution
- Controller frequency in Hz
- Don't optimize for phase margin with gain optimum ( ```Td = 0``` )
- Maximum overshoot inpercent
- Step response destination in % for optimization ( average RPM value during operation )
- Stabilization time in seconds
- Tolerance band in %

## PCB toplayer

<p align="center">
  <img src=docs/img/toplayer.png width="75%">
</p>


## PCB bottomlayer

<p align="center">
  <img src=docs/img/bottomlayer.png width="75%">
</p>
