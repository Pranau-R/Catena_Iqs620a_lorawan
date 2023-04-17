# Understanding MCCI Catena Iqs620a Touch and Proximity Sensor data sent on port 1 format 0x2f

<!-- markdownlint-disable MD033 -->
<!-- markdownlint-capture -->
<!-- markdownlint-disable -->
<!-- TOC depthFrom:2 updateOnSave:true -->

- [Overall Message Format](#overall-message-format)
- [Field format definitions](#field-format-definitions)
	- [Battery Voltage (field 0)](#battery-voltage-field-0)
	- [Bus Voltage (field 1)](#bus-voltage-field-2)
	- [Boot counter (field 2)](#boot-counter-field-3)
	- [SAR Count Channel0 (field 3)](#sar-count-channel-0-field-3)
	- [SAR Count Channel1 (field 4)](#sar-count-channel-1-field-4)
	- [SAR Count Channel2 (field 5)](#sar-count-channel-2-field-5)
	- [Hall Effect Amplitude (field 6)](#hall-effect-amplitude-field-6)
- [Data Formats](#data-formats)
	- [`uint16`](#uint16)
	- [`int16`](#int16)

<!-- /TOC -->

## Overall Message Format

MCCI Catena Iqs620a - Touch and Proximity Sensor format 0x2f messages are always sent on LoRaWAN port 1. Each message has the following layout.

byte | description
:---:|:---
0 | Format code (always 0x2f, decimal 47).
1 | bitmap encoding the fields that follow
2..n | data bytes; use bitmap to decode.

Each bit in byte 1 represent whether a corresponding field in bytes 2..n is present. If all bits are clear, then no data bytes are present. If bit 0 is set, then field 0 is present; if bit 1 is set, then field 1 is present, and so forth.

Fields are appended sequentially in ascending order.  A bitmap of 0000101 indicates that field 0 is present, followed by field 2; the other fields are missing.  A bitmap of 00011010 indicates that fields 1, 3, and 4 are present, in that order, but that fields 0, 2, 5 and 6 are missing.

## Field format definitions

Each field has its own format, as defined in the following table. `int16`, `uint16`, etc. are defined after the table.

Field number (Bitmap bit) | Length of corresponding field (bytes) | Data format |Description
:---:|:---:|:---:|:----
0 | 2 | [int16](#int16) | [Battery voltage](#battery-voltage-field-0)
1 | 2 | [int16](#int16) | [Bus voltage](#bus-voltage-field-1)
2 | 1 | [uint8](#uint8) | [Boot counter](#boot-counter-field-2)
3 | 2 | [uint16](#uint16) | [Sar count channel0](#sar-count-channel-0-field-3)
4 | 2 | [uint16](#uint16) | [Sar count channel1](#sar-count-channel-1-field-4)
5 | 2 | [uint16](#uint16) | [Sar count channel2](#sar-count-channel-2-field-5)
6 | 2 | [int16](#int16) | [Hall effect amplitude](#hall-effect-amplitude-field-6)
7 | n/a | n/a | reserved, must always be zero.

### Battery Voltage (field 0)

Field 0, if present, carries the current battery voltage. To get the voltage, extract the int16 value, and divide by 4096.0. (Thus, this field can represent values from -8.0 volts to 7.998 volts.)

### Bus Voltage (field 1)

Field 1, if present, carries the current voltage from USB VBus. Divide by 4096.0 to convert from counts to volts. (Thus, this field can represent values from -8.0 volts to 7.998 volts.)

_Note:_ this field is not transmitted by some versions of the sketches.

### Boot counter (field 2)

Field 2, if present, is a counter of number of recorded system reboots, modulo 256.

### SAR Count Channel 0 (field 3)

Field 3, if present, is a First Specific Absorption Rate count. It is 2 bytes of [`uint16`](#uint16).

### SAR Count Channel 1 (field 4)

Field 4, if present, is a First Specific Absorption Rate count. It is 2 bytes of [`uint16`](#uint16).

### SAR Count Channel 2 (field 5)

Field 5, if present, is a Second Specific Absorption Rate count. It is 2 bytes of [`uint16`](#uint16).

### Hall Effect Amplitude (field 6)

Field 6, if present, is a hall effect amplitude. It is 2 bytes of [`int16`](#int16). It varies based on the strength of the Magnetic Field.

## Data Formats

All multi-byte data is transmitted with the most significant byte first (big-endian format).  Comments on the individual formats follow.

### `uint16`

an integer from 0 to 65536.

### `int16`

a signed integer from -32,768 to 32,767, in two's complement form. (Thus 0..0x7FFF represent 0 to 32,767; 0x8000 to 0xFFFF represent -32,768 to -1).