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
| Size (bytes) |    2           |        4          |
|--------------|----------------|--------------------
| value        |  Battery (mV)  | Counter           |


## Adjusting pump-timeout (will reset to default value ~2s if the MCU is reset)

| Size (bytes) |    1           |        4          |
|--------------|----------------|--------------------
| value        |  0x34          | New pump off timer|

Ex: To set pump time off to 5000ms (5s) -> 0x34 0x00 0x00 0x13 0x18 
## Note
> Current implementation will prevent changing to other working mode and force to run in mode 10 after reset


	
	





