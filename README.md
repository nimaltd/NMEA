## NMEA library for STM32 LL
*	Author:     Nima Askari
*	WebSite:    https://www.github.com/NimaLTD
*	Instagram:  https://www.instagram.com/github.NimaLTD
*	LinkedIn:   https://www.linkedin.com/in/NimaLTD
*	Youtube:    https://www.youtube.com/@nimaltd
*	Version 3 edited by: Mahdi Zamani (https://github.com/zamanimahdi)
--------------------------------------------------------------------------------
* http://github.com/nimaltd/NMEA.
--------------------------------------------------------------------------------
Version 3 receives with a **circular DMA** and is **polled** from `nmea_loop()`.
No USART interrupt, no DMA interrupt, no heap, no RTOS dependency.

Supported sentences (any talker: GP, GN, GL, GA, HE, HC, SD, YX ...):

| Sentence | Data |
|---|---|
| GGA | time, latitude, longitude, fix quality, satellites, HDOP, altitude |
| RMC | status, time, date, latitude, longitude, speed, course |
| HDT / HDM | true / magnetic heading |
| DPT | depth, depth offset |
| MTW | water temperature (°F is converted to °C) |
| TXT | antenna status (`ANTSTATUS=OK/OPEN/SHORT`, u-blox) |

--------------------------------------------------------------------------------
## Setup
* In CubeMX enable the USART (LL library), mode **Asynchronous**, direction RX is enough. Set baudrate, stop bits and parity.
* Add a DMA request for **USART_RX**: mode **Circular**, data width **Byte**. No interrupt has to be enabled.
* Configure `nmea_config.h`: USART, baudrate, DMA, DMA channel and, on parts with DMAMUX/CSELR, the DMA request.
* Create a structure of `nmea_t`.
* Call `nmea_init()` after the CubeMX initialization.
* Call `nmea_loop()` in the infinite loop.

`nmea_loop()` must be called at least once every `NMEA_DMA_BUFFER_SIZE * 10 / NMEA_BAUDRATE` seconds
(266 ms with 1024 bytes at 38400 baud). If it is called later, the lost data is skipped and counted in `err_overrun`.

Optionally set `NMEA_HW_INIT` to `1` and the library configures the GPIO, USART and clocks itself (no CubeMX USART needed).

Tested on STM32G0 and compiled for STM32F0. Written for parts with channel-based DMA (F0, F1, F3, G0, G4, L0, L4, WB).

--------------------------------------------------------------------------------
## Example:
```
#include "nmea.h"
nmea_t gps;

int main()
{
  ...
  nmea_init(&gps);
  while (1)
  {
    if (nmea_loop(&gps) & NMEA_MASK_RMC)
    {
      uint16_t year;
      uint8_t month, day, hour, min, sec;
      float lat, lon;
      if (nmea_gnss_utc(&gps, &year, &month, &day, &hour, &min, &sec))
      {
        // date and time of one valid RMC sentence
      }
      if (nmea_gnss_latitude_deg(&gps, &lat) && nmea_gnss_longitude_deg(&gps, &lon))
      {
        // position
      }
    }
  }
}
```

## RTOS example:
```
#include "nmea.h"
nmea_t gps;

void task_nmea(void const * argument)
{
  nmea_init(&gps);
  while (1)
  {
    nmea_loop(&gps);
    osDelay(10);
  }
}
```
All functions of one `nmea_t` must be used from the same task (there is no locking).

--------------------------------------------------------------------------------
## Data validity
* Every getter returns `false` when the value is not valid. The value is written only when the getter returns `true`.
* GNSS values are valid only when the receiver reports a fix (RMC status `A`, GGA quality > 0).
* Values older than `NMEA_DATA_TIMEOUT_MS` are not valid (receiver disconnected).
* `nmea_gnss_utc()` returns the date and the time of the **same** RMC sentence.
* `nmea_available()` / `nmea_available_reset()` work as in version 2.

## Statistics
`sentences_ok`, `err_checksum`, `err_format`, `err_overrun`, `err_dma` in `nmea_t`.

--------------------------------------------------------------------------------
## Changes from version 2
* `nmea_init(&gps)` has no USART / buffer size arguments: they are in `nmea_config.h`.
* `nmea_callback()` is removed: nothing has to be called from the USART interrupt.
* `nmea_loop()` returns a bit mask of the decoded sentences (`NMEA_MASK_GGA`, `NMEA_MASK_RMC` ...).
* `_NMEA_USE_FREERTOS` is not needed anymore.
* Fixed: empty fields (e.g. `$GPRMC,123519.00,V,,,,,,,230394,,,N*xx` without fix) were skipped by `strtok()`, so the following fields were decoded into the wrong values.
* Fixed: negative values (altitude, depth offset, temperature) were rejected.
* Sentences are decoded as soon as they end, instead of after 100 ms of silence (works at 5/10 Hz output rates too).
* New: `nmea_gnss_utc()`, `nmea_gnss_hdop()`, `nmea_gnss_fix_quality()`, `nmea_gnss_antenna()`.
