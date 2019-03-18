
## Contents
----------
- [The Overview](#the-overview)
- [Features](#features)
- [Contact me](#contact-me)
- [Information](#information)
- [Issue report template](#issue-report-template)

# The Overview
----------
- This device provides customers with an ultra-low cost LoRa node device solution.With a unique debug interface
- This device has not started selling and is in the debugging phase.Will be available soon.
- This library is transplanted form https://github.com/Lora-net/LoRaMac-node
- Only support the STM32F030C8Tx + LoRa series products made by HelTec Automation(TM), and a LoRa gateway is must needed.
- You may need change some definition in "Commissioning.h" ,"main.c"...
- Use RTC and deep sleep mode supported, only few millisecond wake up in a cycle.

# Features
----------
The STM32F030_LoRaWAN library provides a fairly complete LoRaWAN Class A and Class C implementation, supporting the EU-433 CN-470 EU-868 and US-915 bands.
Only a limited number of features was tested using this port on Heltec hardware, so be careful when using any of the untested features.

The following functions included:

- LoRaWAN protocol V1.0.2 Class A and Class C;
- Use external RTC(32.768KHz);
- STM32F030C8Tx + LoRa boards made by Heltec Automation(TM) can use this library;
- Support deepsleep and stop mode.
- Uniquely designed debug interface for easy maintenance.

# Information
----------
[HelTecDevice](http://www.heltec.cn/proudct_center/internet_of_things/)


 - Gateway: HT-M02 GateWay¡¢[HT-M01 Mini LoRa Gateway](http://www.heltec.cn/project/ht-m01-lora-gateway/?lang=en) + Raspberry Pi Zero W

 - Node: STM32_F030_LoRa

 - LoRaServer: Aliyun ECS + Ubuntu 16.04 + [loraserver.io](https://www.loraserver.io/)

 - MDK-ARM V5.


    | working band | status |
    | :----------------: | :------------:|
    | EU_433 | not test |
    | CN_470_510 | work well |
    | EU_863_870 | not test |
    | US_902_928 | not test |


# Contact me
----------
- [mail](mailto:1327270611@qq.com)

# Issue report template
----------
[for reference](https://github.com/solotaker/STM32_F030_LoRa/issues).     
