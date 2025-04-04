# Raspberry-Pi Pico / Pico 2 OV5640 Camera Driver

This driver can be used to capture pictures from an OV5640 camera on the Raspberry Pi Pico. I wrote it to avoid circuitpython to avoid unnecessary RAM usage and keeping it simple using C. 

Statemachines/PIO/DMA transfers are based on Adafruit's [ParallelImageCapture module](https://github.com/adafruit/circuitpython/tree/main/ports/raspberrypi/common-hal/imagecapture).
OV5640 interface is based on Adafruit's [OV5640 circuitpython library](https://github.com/adafruit/Adafruit_CircuitPython_OV5640). 

## Project development

VScode settings expects the raspberry pico to have its SWCLK and SWDIO pins connected to debug.

On my end the project was developped using a Raspberry pi 3b+ model as a "debug probe" so there's my I/O pin connection regarding SWD: 
|Pico | Raspberry pi 3b+            |
| ------------- | -------------- |
| `SWCLK`        | Pi pin 22 (GPIO 25)  |
| `GND`        | Pi pin 20  |
| `SWDIO`       | Pi pin 18 (GPIO 24) |

Additionally logs are conveyed via UART as defined in the CMake:
|Pico | Raspberry pi 3b+            |
| ------------- | -------------- |
| `UART TX` GPIO 0       | Pi pin 10 (GPIO 15)  |
| `UART RX` GPIO 1        | Pi pin 8 (GPIO 14)  |
| `GND` PIN 3       | Pi pin 6 |

I highly recommend using a raspberry pi as debug probe, especially if you don't have any SoC development background as I do. I found that installing localy vscode and using an ssh connection with a more capable computer as explained [here](https://www.raspberrypi.com/news/coding-on-raspberry-pi-remotely-with-visual-studio-code/) is very convenient. 

## Circuit

The code in this repo assumes a specific wiring defined in ov5640.h.

| OV5640 Module | Pico           |
| ------------- | -------------- |
| `MCLK_GPIO`        |21|
| `SDA_GPIO`        |2|
| `SCL_GPIO`       |3|
| `PWD_GPIO`        |14|
| `RST_GPIO`        |15|
| `DATA_GPIO`        |6|
| `PCLK_GPIO`          |26|
| `VSYNC_GPIO`          |27|
| `HREF_GPIO`          |4|

Note that:
- All DATA pins should be next to each other and in order.
- MCLK_GPIO must be kept to 21 unless you change the method generating the clock or have another GPIO connected to GPOUT0-3 clock generators. See clock_gpio_init(). 
