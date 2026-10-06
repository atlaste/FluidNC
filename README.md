<img src="https://github.com/bdring/FluidNC/wiki/images/logos/FluidNC.svg" width="600">

## Introduction

**FluidNC** is a CNC firmware optimized for the ESP32 controller. It is the next generation of firmware from the creators of Grbl_ESP32. It includes a web based UI and the flexibility to operate a wide variety of machine types. This includes the ability to control machines with multiple tool types such as laser plus spindle or a tool changer.  

[![Donate via PayPal](https://img.shields.io/badge/Donate-PayPal-00457C?logo=paypal&logoColor=white)](https://www.paypal.com/donate/?hosted_button_id=XS48ZNCFL6V7G)

## New in this fork

* Asynchronous tasks in a scheduler
* 10/100 MBit Ethernet via DM9051 IC's
* ODrive CAN spindles
* State persistance through FRAM
* Proper S-curve acceleration
* Threading
* Spindle synchronization (so spindle speed feedback)
* Cutter compensation
* Constant spindle speed
* Proper ATC tool changer support
* Tool tables
* Bed leveling, both 1D and 2D
* Led strips
* USB camera support (Logitech C310 ONLY!)

New G-codes:

* Lathe G-codes: threading (G33/G76), 
* CSS (G96), 
* Feed per rev (G95), 
* Diameter mode (G7)
* Cutter radius compensation (G41/G42)

## Post processor

Use the linux cnc post processor for lathe, without canned cycles. Simple as that, it just works.

## Encoders and lathes

Lathes need an encoder to work properly. https://www.zoomwroom.com/Make/Lathe/SpindleEncoder
has the information you need for choosing a proper encoder. 

Encoders won't work with any of the boards that I have, except for the ZoomWroom board that I sell.
The reason is that the pulses require a high speed path, and most inputs on PCB's are designed 
to be slow and have filtering. The ZoomWroom board has a high speed opto-inputs for the encoder.

## Note about this fork/branch

This fork holds the code to run Lathes with FluidNC. It is the result of hundreds of commits
and even more hours of work, testing, and so on. 

This code is meant to be used on ESP32-S3 boards with PSRAM and Flash. I think you 
can theoretically 

Somewhere down the line I decided to run FluidNC on ESP-IDF instead of PlatformIO and 
Arduino. To compile this project, you need ESP-IDF v5.3.1. Running the build requires:

```
idf.py set-target esp32s3
idf.py menuconfig
idf.py build flash monitor
```

## Note about DM9051 Ethernet

If you plan to use Ethernet via the DM9051, you need to patch the ESP-IDF driver. 
I made a ticket at Espressif with the details with the cause and the fix. You can find it here:
https://github.com/espressif/esp-idf/issues/15982 . After that it's rock stable.

## Supporting other webcams

Can I support other webcams? No. The fact of the matter is that webcams usually just 
publish the wrong information over the wire the moment they notice it's USB 1.1. The 
Logitech C310 is the only webcam that I know of that works with the ESP32-S3 USB 
Host driver. Other webcams may work, but I don't have a way of testing them, nor the 
hardware. If you happen to find one that also works on reasonably high resolutions, 
let me know.

## Machine Definition Method

There is no install script here. 

There is documentation about config files here:

* http://wiki.fluidnc.com/en/config/overview for most FluidNC configuration data
* https://www.zoomwroom.com/ for the specific modifications

If you buy a board at the zoomwroom store, it will come with a standard configuration file.

## Basic Grbl Compatibility

The original intent was to maintain as much Grbl compatibility as possible. It is 100% compatible with the day to day operations of running gcode with a sender, so there is no change to the Grbl gcode send/response protocol, and all Grbl gcode are supported. Most of the $ settings have been replaced with easily readable items in the config file.

That being the case, we usually refer to LinuxCNC rather than GRBL for our g-code. In reality, the g-code 
that we interpret in the firmware is more like LinuxCNC than GRBL nowadays.

## WebUI

FluidNC includes a built-in browser-based Web UI (Esp32_WebUI) so you control the machine from a PC, phone, or tablet on the same Wifi network.

## Wiki

[Check out the wiki](http://wiki.fluidnc.com) if you want the learn more about the feature or how to use it.

## Credits

The original [Grbl](https://github.com/gnea/grbl) is an awesome project by Sungeon (Sonny) Jeon. 

FluidNC was originally founded by Barton Dring, Stefan de Bruijn and Mitch Bradley, and has been supported by a lot of people ever since. 

The Wifi and WebUI is based on [this project.](https://github.com/luc-github/ESP3D-WEBUI)  

## Discussion

<img src="http://wiki.fluidnc.com/discord-logo_trans.png" width="180">

We have a Discord server for the development this project. Ask for an invite


## Donations

This project requires a lot of work and often expensive items for testing. Please consider a safe, secure and highly appreciated donation via the PayPal link below.

Via this button you can donate to atlaste: 

[![Donate via PayPal](https://img.shields.io/badge/Donate-PayPal-00457C?logo=paypal&logoColor=white)](https://www.paypal.com/donate/?hosted_button_id=XS48ZNCFL6V7G)

The best way to support this work is to buy a board from the ZoomWroom store. The boards are designed to work with FluidNC and are tested with it.
