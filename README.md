## gsm module library for STM32 LL
*	Author:     Nima Askari
*	WebSite:    https://www.github.com/NimaLTD
*	Instagram:  https://www.instagram.com/github.NimaLTD
*	LinkedIn:   https://www.linkedin.com/in/NimaLTD
*	Youtube:    https://www.youtube.com/channel/UCUhY7qY1klJm1d2kulr9ckw 
--------------------------------------------------------------------------------
* 'http://github.com/nimaltd/NMEA'.
-----------------------------------------------------------   
* Enable USART (LL Library) and RX interrupt.
* Set baudrate, stop bits and parity.
* create a structure of `nmea_t`.
* Add `nmea_callback()` in usart interrupt.
* Configure `nmea_config.h`.
* Call `nmea_init()`.
* Call `nmea_loop()` in infinit loop.
* If using FREERTOS, please create a task for nmea with at least 256 word heap size. 
--------------------------------------------------------------------------------
## RTOS example:
```
#include "nmea.h"
nmea_t gps;

int main()
{
  ...  
}

void task_nmea(void const * argument)
{
  nmea_init(&gps, USART1, 1024);
  while (1)
  {
    nmea_loop(&gps);
    osDelay(1);
  }
}

void task_other(void const * argument)
{
 uint8_t time_h; 
 for(;;)
  {
		if (nmea_available(&gps))
		{
      nmea_gnss_time_h(&gps, &time_h);
			nmea_available_reset(&gps);
		}
    osDelay(1);
}
```
in interrupt file

```
#include "nmea.h"
extern nmea_t gps;

void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	nmea_callback(&gps);
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}
```



