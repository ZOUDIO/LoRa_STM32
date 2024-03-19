LoRa STM32 Source Code
===============

Source code for [Dragino LoRaST module](http://www.dragino.com/products/lora/item/127-lora-st.html) based on STM STM32L072CZT6 microcontroller and Semtech SX1276 LoRa transceiver.

This fork from the [original repo](https://github.com/dragino/LoRa_STM32) adds specific functionality for Smeerkees Sunscreen dispensers

The dispensers contain custom hardware which allows users to attach a solar panel, 12V battery, RTC, PIR sensor, proximity sensor and pump to the module.

In normal operation the system is powered by the battery, which gets charged by the solar panel when the sun is shining.
If the user waves their hand in front of the PIR, the proximity sensor gets powered. 
Once the proximity sensors detects movement, the pump is actuated to dispense a portion of sunscreen to the user.
Usage gets saved to non-volatile memory and sent to a dashboard via a LoRa uplink.
The user can reset this count value by pressing the reset button.
Time boundaries can be set via LoRa downlinks to avoid operation and/or abuse outside of the desired hours.

> **_NOTE:_**  This README represents the desired state of the software.\
> The actual implementation does not match this at the moment of writing.

# Uploading firmware
The board has an STDC14 connector which can be used with an STlinkV3 or other SWD compatible programmer.
The latest binary can be found on the [release page](https://github.com/ZOUDIO/LoRa_STM32/releases) of this repo.
In production a Python script is used to write the firmware, read the Lora ID's, set the RTC and print a label. 

## Hardware
The latest hardware files are in the Altium account of Zoudio (info@zoudio.com)\
<img src="https://github.com/ZOUDIO/LoRa_STM32/assets/48258407/b4817111-f6d9-4237-949b-300aaf859e8e" width="500" height="445">

## Pinout
| Signal     | Pin  | Type                        | Function                                 |
|------------|------|-----------------------------|------------------------------------------|
| PIR        | PB2  | Digital input, active-high  | Detect PIR sensor state (optional)       |
| BATT       | PA0  | Analog input                | Read battery voltage                     |
| RESET_SW   | PB5  | Digital input, active-low   | Reset usage counter                      |
| PROX       | PB14 | Digital input, active-high  | Detect proximity sensor state            |
| PUMP       | PB12 | Digital output, active-high | Enable pump                              |
| LORA_LED_R | PA8  | Digital output, active-low  | Red led of the RGB lora status led       |
| LORA_LED_G | PB15 | Digital output, active-low  | Green led of the RGB lora status led     |
| LORA_LED_B | PB13 | Digital output, active-low  | Blue led of the RGB lora status led      |
| TIME_LED_R | PB3  | Digital output, active-low  | Red led of the RGB timelock status led   |
| TIME_LED_G | PB4  | Digital output, active-low  | Green led of the RGB timelock status led |
| TIME_LED_B | PA9  | Digital output, active-low  | Blue led of the RGB timelock status led  |

The onboard RTC (DS3231MZ+TRL) is connected to I2C via PB6 (SCL) and PB7 (SDA) at address 0x68

# Uplinks
Uplinks can be differentiated using their index (first byte). Multi-byte fields are MSB unless noted otherwise.

## BOOT
Sent at boot time to identify device and its firmware version
| Size (bytes) | 1    | 1     | 1     | 1     |
|--------------|------|-------|-------|-------|
| Value        | 0x00 | Major | Minor | Patch |

Example: Device with firmware v1.2.3\
Uplink: 0x00 0x01 0x02 0x03

## COUNT
Sent after sunscreen has been dispensed. Does not repeat if last dispense is less than 6 minutes ago.\
Gets sent every 30 minutes even if the device did not dispense anything.\
Includes a field for battery level, all-time counter and a counter that can be reset by the user (used after refilling the sunscreen container). 
| Size (bytes) | 1    | 2                    | 2                  | 2                   |
|--------------|------|----------------------|--------------------|---------------------|
| Value        | 0x01 | Battery voltage (mV) | Counter cumulative | Counter since reset |

Example: Device with a battery level of 12.34V, cumulative counter of 1234 and counter since reset of 123\
Uplink: 0x01 0x30 0x34 0x04 0xD2 0x00 0x7B

# Downlinks
Downlinks are an extension of [Dragino's command set](http://wiki.dragino.com/xwiki/bin/view/Main/End%20Device%20AT%20Commands%20and%20Downlink%20Command/) and thus start after their last used command index (0x34).\
All downlinks are confirmed with an uplink copy.\
Every downlink has an associated AT command which accepts decimal input. Hexadecimal is also allowed with prefix 0x.\
The current value can be requested by sending '?' (e.g. AT+TIMELOCK=?)

## PUMP_DURATION
| Size (bytes) | 1    | 2                  |
|--------------|------|--------------------|
| Value        | 0x35 | Pump duration (ms) |

Example: Set pump duration to 1000ms\
Downlink: 0x35 0x03 0xE8\
AT command: AT+PUMPDURATION=1000\
Note: default is 250ms

## TIME_LOCK
| Size (bytes) | 1    | 1              | 1                | 1               | 1                 |
|--------------|------|----------------|------------------|-----------------|-------------------|
| Value        | 0x36 | Hour low limit | Minute low limit | Hour high limit | Minute high limit |

Example: Set the time limit from 11:00 to 15:30\
Downlink: 0x36 0x0B 0x00 0x0F 0x1E\
AT command: AT+TIMELOCK=11,0,15,30\
Note: Set all hours/minutes to 255 (0xFF) to disable timelock (default)

## RTC
This is usually done once using AT commands in production, but can also be done via downlink if needed.
| Size (bytes) | 1    | 1    | 1     | 1    | 1     | 1       | 1       |
|--------------|------|------|-------|------|-------|---------|---------|
| Value        | 0x37 | Year | Month | Date | Hours | Minutes | Seconds |

Example: Set the RTC to 12:34:56 on 18 March 2024\
Downlink: 0x36 0x18 0x03 0x12 0x0C 0x22 0x38\
AT command: AT+RTC=24,3,18,12,34,56\
Note: Year is represented by last two digits (e.g. 2024 = 24)

# Status LEDs
## LoRa
| Color  | State          |
|--------|----------------|
| Red    | Disconnected   |
| Orange | Sending uplink |
| Green  | Connected      |

## Timelock
| Color  | State           |
|--------|-----------------|
| Red    | RTC not set     |
| Orange | Timelock active |

# Questions / remarks?
Main repo maintainer (Jesse van der Zouw) can be contacted via info@zoudio.com
