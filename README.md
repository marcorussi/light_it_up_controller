# light_it_up_controller
A BLE motion controller based on Nordic nrf51 chipset. Developed under Ubuntu environment using a nrf51 PCA10028 development kit and a custom board. The firmware is based on S130 from Nordic SDK 11.x.x.

This firmware detects a motion state and advertises an associated value in a specific data field. A compatible 4 channels LED dimmer scans the advertising packet and controls the 4 channels PWM values. The firmware detects face orientation and rotation using an accelerometer.
Refer to doc.txt file. 


**Install**

Download Segger JLink tool from https://www.segger.com/jlink-software.html. Unpack it and move it to /opt directory.
Download the Nordic SDK from http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v11.x.x/. Unpack it and move it to /opt directory.

Clone this repo in your projects directory:

    $ git clone https://github.com/marcorussi/light_it_up_controller.git
    $ cd light_it_up_controller
    $ gedit Makefile

Verify and modify following names and paths as required according to your ARM GCC toolchain:

```
PROJECT_NAME := ble_controller
NRFJPROG_PATH := ./tools
SDK_PATH := /opt/nRF5_SDK_11.0.0_89a8197
LINKER_SCRIPT := ble_controller_gcc_nrf51.ld
GNU_INSTALL_ROOT := /home/marco/ARMToolchain/gcc-arm-none-eabi-4_9-2015q2
GNU_VERSION := 4.9.3
GNU_PREFIX := arm-none-eabi
```
Verify also the JLink path in tools/nrfjprog.sh


**Flash**

Connect your nrf51 Dev. Kit, make and flash it:
 
    $ make
    $ make flash_softdevice (for the first time only)
    $ make flash

You can erase the whole flash memory by running:

    $ make erase


**DFU Upgrade**

For creating a .zip packet for DFU upgrade run the following command:

nrfutil dfu genpkg liu_controller.zip --application ble_controller_s130.hex --application-version 0xffffffff --dev-revision 0xffff --dev-type 0xffff --sd-req 0xfffe



