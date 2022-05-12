#ifndef _NMEA_H
#define _NMEA_H

/*
 *	Author:     Nima Askari
 *	WebSite:    https://www.github.com/NimaLTD
 *	Instagram:  https://www.instagram.com/github.NimaLTD
 *	LinkedIn:   https://www.linkedin.com/in/NimaLTD
 *	Youtube:    https://www.youtube.com/channel/UCUhY7qY1klJm1d2kulr9ckw
 */

/*
 * Version:	2.0.0
 *
 * History:
 *
 * (2.0.0):	Rewrite again. Support NONE-RTOS, RTOS V1 and RTOS V2. Supports GPS,COMPASS,SOUNDER
 */

#include "main.h"
#include "nmea_config.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#if _NMEA_USE_FREERTOS != 0
#define nmea_delay(x) osDelay(x)
#include "cmsis_os.h"
#else
#define nmea_delay(x) HAL_Delay(x)
#endif

typedef struct
{
  uint16_t        satellite : 1;
  uint16_t        time : 1;
  uint16_t        date : 1;
  uint16_t        latitude : 1;
  uint16_t        longitude : 1;
  uint16_t        precision : 1;
  uint16_t        altitude : 1;
  uint16_t        speed_knots : 1;
  uint16_t        course : 1;

} gnss_valid_t;

typedef struct
{
  gnss_valid_t    valid;
  uint8_t         satellite;
  uint8_t         time_h;
  uint8_t         time_m;
  uint8_t         time_s;
  uint8_t         date_y;
  uint8_t         date_m;
  uint8_t         date_d;
  float           latitude_tmp;
  float           longitude_tmp;
  float           latitude_deg;
  float           longitude_deg;
  float           precision_m;
  float           altitude_m;
  float           speed_knots;
  float           course_deg;

} gnss_t;

typedef struct
{
  uint8_t         true_compass : 1;
  uint8_t         mag_compass : 1;

} compass_valid_t;

typedef struct
{
  compass_valid_t valid;
  float           true_course_deg;
  float           mag_course_deg;

} compass_t;

typedef struct
{
  uint8_t         depth : 1;
  uint8_t         depth_offset : 1;
  uint8_t         temp : 1;

} sounder_valid_t;

typedef struct
{
  sounder_valid_t valid;
  float           depth_m;
  float           depth_offset_m;
  float           temp_c;

} sounder_t;

typedef struct
{
  USART_TypeDef   *usart;
  uint32_t        buf_time;
  char            *buf;
  uint16_t        buf_index;
  uint16_t        buf_size;
  bool            lock;
  bool            available;
  gnss_t          gnss;
  compass_t       compass;
  sounder_t       sounder;

} nmea_t;

//##################################################################################

//								ALL
bool  nmea_init(nmea_t *nmea, USART_TypeDef *usart, uint16_t buf_size);
void  nmea_loop(nmea_t *nmea);
void  nmea_callback(nmea_t *nmea);
bool  nmea_available(nmea_t *nmea);
void  nmea_available_reset(nmea_t *nmea);

//								GNSS
bool  nmea_gnss_time_h(nmea_t *nmea, uint8_t *data);
bool  nmea_gnss_time_m(nmea_t *nmea, uint8_t *data);
bool  nmea_gnss_time_s(nmea_t *nmea, uint8_t *data);
bool  nmea_gnss_date_y(nmea_t *nmea, uint8_t *data);
bool  nmea_gnss_date_m(nmea_t *nmea, uint8_t *data);
bool  nmea_gnss_date_d(nmea_t *nmea, uint8_t *data);
bool  nmea_gnss_satellite(nmea_t *nmea, uint8_t *data);
bool  nmea_gnss_speed_kph(nmea_t *nmea, float *data);
bool  nmea_gnss_speed_knots(nmea_t *nmea, float *data);
bool  nmea_gnss_precision_m(nmea_t *nmea, float *data);
bool  nmea_gnss_course_deg(nmea_t *nmea, float *data);
bool  nmea_gnss_latitude_deg(nmea_t *nmea, float *data);
bool  nmea_gnss_longitude_deg(nmea_t *nmea, float *data);
bool  nmea_gnss_altitude_m(nmea_t *nmea, float *data);

//								COMPASS
bool  nmea_compass_true_course_deg(nmea_t *nmea, float *data);
bool  nmea_compass_mag_course_deg(nmea_t *nmea, float *data);
  
//								SOUNDER
bool  nmea_sounder_depth_m(nmea_t *nmea, float *data);
bool  nmea_sounder_depth_offset_m(nmea_t *nmea, float *data);
bool  nmea_sounder_temp_c(nmea_t *nmea, float *data);

#endif
