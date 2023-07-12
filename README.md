# Breakaway Bridge

> *There may be no more familiar sight in bike racing than the breakaway, the rider or riders who boldly jump ahead of the peloton in a race’s opening kilometers and attempt to steal a victory.*
> [ - bicycling.com](https://www.bicycling.com/tour-de-france/a20031155/cycling-breakaways/)

This project allows a Peloton bike to be used as a standard Bluetooth indoor cycling trainer, so it can be used with apps like Zwift, or otherwise recorded. Importantly, this also frees up the bike screen for other uses, such as playing YouTube videos, Netflix Shows, or - my favorite - baseball games on MLB.tv.


## Building
- Install `esp-idf` [(Guide here)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)
- Run `idf.py build` (use `idf.py flash` to program a connected ESP32 with the USB bootloader)
- View debug console with `idf.py monitor`

## Hardware Requirements
To communicate with the Peloton, you need an RS232 line transceiver. I used the [TRS232](https://www.ti.com/product/TRS232)
from Texas Instruments.

## High Level Design

- `components/ble-trainer` implements a BLE Peripheral device with the Cycling Power Service
- `components/event-handler` main application logic glue between the serial and bluetooth layers
- `components/peloton-serial` 
- `main` contains the program entrypoint and starts the other applications

