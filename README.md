LoRa STM32 Source Code
===============

Source code for Dragino LoRa module base on STM32 Chip.
Support Products: [LoRaST](http://www.dragino.com/products/lora/item/127-lora-st.html), [LSN50](http://www.dragino.com/products/lora/item/128-lsn50.html).

How To Use Source Code: [For LSN50](http://wiki1.dragino.com/index.php?title=LoRa_Sensor_Node-LSN50#Program_LSN50)

Switch between LSN50 Code and LoRa ST default code:

In file Projects/Multi/Applications/LoRa/DRAGINO-LRWAN(AT)/inc/hw_conf.h

    #define LoRa_Sensor_Node /*LSN50*/      --- For LSN50
    //#define AT_Data_Send /*AT+SEND or AT+SENDB*/    --- For LoRa ST

# Custom work

## Version: V1.8.1

## Pin used

- Sensor pin: PB12 - Pull-down internally, input interrupt with rising edge
- Button pin: PB15- Pull-down internally, input interrupt with rising edge
- Pump pin: PB13 - Output

## Uplink package format

| Size (bytes) |    2               |        4          |
|--------------|--------------------|--------------------
| value        |  12 VBattery (mV)  | Counter value     |

## Adjusting pump-timeout

| Size (bytes) |    1           |        4          |
|--------------|----------------|--------------------
| value        |  0x34          | New pump off timer|

Ex: To set pump time off to 5000ms (5s) -> 0x34 0x00 0x00 0x13 0x18 (Hexadecimal - MSB)

## Setting the time boudaries for pump

0x35 HH_low MM_low HH_high MM_high
| Size (bytes) |    1           |        1           |    1           |        1          |        1          |
|--------------|----------------|--------------------|----------------|-------------------|-------------------
| value        |  0x35          | Hour low limit     | Min low limit  | Hour high limit   | Min high limit    |

Ex: Set the time boundaries so that the pump can only be turned on at 11:25 - 20:45
-> Downlink: 0x35 0x0A 0x19 0x14 0x2D

Note: Send 0x35 0xFF 0xFF 0xFF 0xFF to turn off the time boundaries

## AT commands to time for DS3231 (Input is in decimal)

Set time → AT+TIME=HH,MM,SS
Set date→  AT+DATE=D,DD,MM,YY (D = Day = Sunday -Saturday ~ 1-7)

Ex: Set time to 11:25:00 Saturday 20/11/2020

-> AT+TIME=11,25,00

-> AT+DATE=7,20,11,20

## Version 1.8.2

## Pin used

- Button pin: PB5- Pull-down internally, input interrupt with rising edge
- PROX_SIGNAL_3V3 pin: PB14 - Pull-down internally, input interrupt with rising edge
- PROX_EN pin: PA10 - Input
- Pump pin: PB12 - Output
- LED RED pin: PB13 - Output active low
- LED BLUE pin: PB15 - Output active low
- LED GREEN pin: PA8 - Output (RADIO ANT SWTICH)

## Version 1.8.3

## Uplink read config parameters after received 0x40 0x40 downlink

- Fix reset counter interrupt problem due to pwr_control pin deinit when enter low power mode
- Add AT commands for setting pump time ("AT+PUMPTIME") and time boundaries ("AT+LIMITTIME")
  
   > Set pump time to 5000ms (5s) -> AT+PUMPTIME=5000

   > Set the time boundaries so that the pump can only be turned on at 11:25 - 20:45 -> AT+LIMITTIME=11,25,20,45

   > Note: Send AT+LIMITTIME=255,255,255,255 to turn off the time boundaries

- Add functions to store/read config parameters (pump time, time boundaries) from EEPROM at 0x80800D8
- Add support for copy downlink msg to uplink msg for custom work (pump time, time boundaries)

## Improvements
>
> Current implementation will prevent changing to other working mode and force to run in mode 10 after reset
