#Important NOTE:

Please download the most recent Configurator release to flash.  If the configurator will not flash the new firmware to your device and you received your device before 8/1/2025, please contact us via invictuscockpits.com, provided you have your order number and you are the original purchaser.  



# Invictus HOTAS Firmware

**Firmware for Invictus Cockpit Systems’ HOTAS line of products.**

This firmware is derived from the [FreeJoy project](https://github.com/FreeJoy-Team/FreeJoy), but heavily modified and adapted specifically for Invictus flight control devices. It is **not** intended for general-purpose joystick builds or hobbyist experimentation, but you may find some useful modifications inside the code that can help you with your Freejoy Project

## Description

The Invictus HOTAS Firmware runs on STM32 microcontrollers and communicates with a dedicated Qt-based configuration utility via USB HID. It allows users of Invictus devices to map physical inputs (buttons, encoders, shift registers, axes) to logical outputs used in flight simulators such as DCS, Falcon BMS, and MSFS.

All configuration is handled through the official Invictus HOTAS Configurator. End users do not need to compile or modify the firmware.

## Key Features

- Optimized for Invictus Cockpit Systems’ hardware only
- Will flash to STM32 Blue Pill and custom Invictus Hardware

## Hardware Compatibility

- Invictus HOTAS Controllers
- STM32 Blue Pill



### Flashing Instructions

- Enter bootloader mode by grounding the **BOOT0** pin and resetting the device.
- Use the **Invictus Configurator** to flash the compiled firmware binary.
- Configuration is handled entirely through the GUI after flashing.

## Disclaimer

This firmware is provided as-is and is not intended for use outside of officially supported Invictus products. It is a heavily modified branch of FreeJoy and is not maintained for general-purpose joystick development.

FreeJoy Repository (original project):  
**https://github.com/FreeJoy-Team/FreeJoy**

---

© 2025 Invictus Cockpit Systems. All rights reserved.
