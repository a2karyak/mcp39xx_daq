# mcp39xx_daq

Firmware for a data acquisition system based on STM32F103 MCU and MCP3914 ADCs.

## MCP3914 ADC

MCP3914 is a Microchip product targeting energy monitoring, usually referred to as an "analog front-end (AFE)". 
It consists of an 8-channel simultaneously sampling 24-bit ADC and a programmable gain amplifier (PGA). 
The maximum sampling rate is 125 ksa/s, however, effective resolution is degraded to 17 bit at that rate. 
The device communicates over an SPI interface with a couple of additional signals.

Microchip also offers an evaluation board, featuring a 16 bit MCU and a USB to serial interface bridge, which has enough power only for acquisition at very low rate. 
The board is equipped with a 10 MHz crystal, which means the maximum acquisition rate is 78125 sa/s.

The MCP39xx family also includes other models, mainly differing in the number of channels, such as MCP3910, MCP3912, MCP3913, MCP3918, and MCP3919.

[MC3914 Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/20005216A.pdf)  
[MC3914 Eval Board Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/50002176A.pdf)

## STM32 MCU

The controller runs on an STM32F103 MCU, for example, on a ubiquitous Bluepill board. 
The code has also been ported to a more powerful STM32F411, but these changes are not in this repository.

Current pin connections (see input-mcp3914.c):

|Signal|ADC1|ADC2|
|------|----|----|
|RESET|B0|B12|
|MISO|A6|B14|
|MOSI|A7|B15|
|SCK|A5|B13|
|CS|A4|A8|
|DR|A3|A9|

The SPI is configured to run at 18 MHz, which is the highest allowed frequency with 72 MHz system clock.

## Overview

The code is quite a bit optimized so that it can handle data from two MC3914's at the rate limited by the USB throughput, which is about 1 MByte/s. For example, reading data from 2 boards, 8 channels each, using the oversampling factor of 128, at 24 bits from each channel produces 19531.25 samples per second, each containing 48 bytes, resulting the total data rate of 937500 bytes per second. Acquisition at this rate seems to work fine over the USB connection, with some headroom in available clock cycles. If the sampling rate is higher than the communication channel bandwidth, the controller will drop data packets in such a way that the number of lost samples can be detected by the host software, see the protocol description below.

Normally, the MCU receives commands from the serial port and executes them, for example, to read the MCP's registers. When data acquisition is started, the system continuously reads data from the ADCs and transmits it to the host. To avoid race conditions between sending binary packet data and command responses, a "credit" based protocol is used to direct the MCU to send the exact number of packets as requested from the host. 

Data acquisition for each sample is triggered by the DR signal from each ADC, which generates a pin interrupt in the MCU. The DR signal must be configured as a push-pull, as opposed to open drain, using a jumper on the MCP board. Every DR interrupt starts a DMA transfer for the sample data after checking for the correct completion of the previous DMA. There is no separate interrupt for the DMA completion, which saves a few clock cycles on context switching. The sample data received using DMA is concatenated to 1024-byte packets that are sent over the USB once complete. A flow control algorithm is implemented, using most of the onboard RAM (16KB out of 20KB) as a circular packet buffer. The system may gracefully handle short intervals when the host process is slow on reading data.

The firmware detects boards on reset. When two boards are attached, sample data from both is concatenated forming a single double-width sample, e.g. 16 channels when using two 8-channel boards. Since the clocks on two board have a small skew, one sample from the faster board will be discarded when the accumulated time skew between samples grows as high as the sampling period. Unfortunately, there seems to be no way to exactly synchronize multiple MCP3914s.

## Protocol

Communication between the host and the controller happens either through a UART, or a virtual serial port (CDC ACM device) over USB. The serial protocol in both cases is identical, but USB connection has about 10x data throughput (over 1 MByte/s) compared to the default 921600 baud serial port (about 90 KByte/s). 

The protocol is modeled after the original Microchip firmware, but compatibility was not the goal, so there are differences. The protocol for sending acquisition data and the data format are completely different.

Commands are separated by a newline character and multiple commands may be pipelined. However, the size of the command buffer is limited and the system will lock up when it is full.

The full set of commands is not implemented, and error checking is missing.

## Commands

|Command|Response on success|Comment|
|-------|-------------------|-------|
|r[&lt;n&gt;]|r[&lt;n&gt;]=&lt;x&gt;|Read ADC register n (decimal) and print it in hex. Each MCP3914 has 32 registers, numbered 0 to 31. If multiple ADCs are connected, each of them will take a consecutive 32-long ranges starting at 0.|
|w[&lt;n&gt;]=&lt;x&gt;|w0|Write a hex value x to the ADC register n (decimal).|
|C&lt;x&gt;|C0|Set initial channel number to x for acquisition for the first board.|
|C&lt;x&gt;,&lt;y&gt;|C0|Set initial channel number to x and y for both boards.|
|c|c&lt;x&gt;,&lt;y&gt;|Print current channel numbers for both boards.|
|s|s0|Start data acquisition.|
|p|p0|Stop data acquisition. Acquisition stops after all requested packets are sent to the host.|
|!|none|Request a data packet (add a "credit"). The controller only sends to the host one packet for each "!" command. Queuing multiple '!' commands is allowed before the previous data has been read by the host, which makes it possible to pipeline data and fill the USB channel.|

Each data packet is a 1024-byte data structure with a 8 byte header (all fields are little endian):

|u4|start_sample|Number of first sample in the packet. Used to detect lost packets.|
|u2|num_samples|Number of samples in this packet.|
|u2|num_bytes|Total length of sample data in this packet, in bytes.|
|u1|data[124 - 8]|Sample data. There may be padding in the end of packet if the total size of sample data is less than 1024 - 8 bytes.|

The size (16, 24, 32 bits) and number of channels in each sample is determined by the content of the STATUSCOM register. The firmware tracks writes to the STATUSCOM register and adjusts the data format accordingly.

The firmware also implements several performance counters for number of operations performed and time spend in various parts of the code. The counters can be read as registers with numbers 1000 and above. 

## Additional Notes

Building the source code requires additional pieces of code, such as an STM32 library and a USB library, which are not currently published.
