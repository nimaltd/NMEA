#ifndef _NMEA_H
#define _NMEA_H

/*
 *	Author:     Nima Askari
 *	WebSite:    https://www.github.com/NimaLTD
 *	Instagram:  https://www.instagram.com/github.NimaLTD
 *	LinkedIn:   https://www.linkedin.com/in/NimaLTD
 *	Youtube:    https://www.youtube.com/channel/UCUhY7qY1klJm1d2kulr9ckw
 *
 *	Version 3 edited by: Mahdi Zamani (https://github.com/zamanimahdi)
 */

/*
 * Version:	3.0.0
 *
 * History:
 *
 * (3.0.0):	Receive with circular DMA, polled from nmea_loop() (no USART/DMA interrupt).
 *          Byte state machine with on-the-fly checksum, decoding as soon as "\r\n" arrives.
 *          Empty fields are kept in place (fixes shifted fields when there is no fix).
 *          Values are valid only with a fix and for NMEA_DATA_TIMEOUT_MS.
 *          No heap, no sscanf/atof/strtok, no RTOS dependency.
 *          Added: nmea_gnss_utc(), nmea_gnss_hdop(), nmea_gnss_fix_quality(),
 *          nmea_gnss_antenna() ($xxTXT ANTSTATUS), error counters.
 * (2.0.1):	Fix receiving data
 * (2.0.0):	Rewrite again. Support NONE-RTOS, RTOS V1 and RTOS V2. Supports GPS,COMPASS,SOUNDER
 */

#include "main.h"
#include "nmea_config.h"
#include <stdbool.h>
#include <stdint.h>

/* Bit mask returned by nmea_loop() */
#define NMEA_MASK_GGA   (1U << 0)
#define NMEA_MASK_RMC   (1U << 1)
#define NMEA_MASK_HDT   (1U << 2)
#define NMEA_MASK_HDM   (1U << 3)
#define NMEA_MASK_DPT   (1U << 4)
#define NMEA_MASK_MTW   (1U << 5)
#define NMEA_MASK_TXT   (1U << 6)

typedef enum
{
  NMEA_ANTENNA_UNKNOWN = 0,
  NMEA_ANTENNA_OK,
  NMEA_ANTENNA_OPEN,
  NMEA_ANTENNA_SHORT

} nmea_antenna_t;

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
  uint16_t        fix_quality : 1;
  uint16_t        utc : 1;          /* time AND date from the same valid RMC */
  uint16_t        antenna : 1;

} gnss_valid_t;

typedef struct
{
  gnss_valid_t    valid;
  uint8_t         satellite;        /* satellites used in the fix              */
  uint8_t         time_h;           /* UTC                                     */
  uint8_t         time_m;
  uint8_t         time_s;
  uint8_t         date_y;           /* 0..99 = 2000..2099                      */
  uint8_t         date_m;
  uint8_t         date_d;
  float           latitude_deg;     /* + North                                 */
  float           longitude_deg;    /* + East                                  */
  float           precision_m;      /* HDOP                                    */
  float           altitude_m;       /* above mean sea level                    */
  float           speed_knots;
  float           course_deg;
  uint8_t         fix_quality;      /* GGA: 0 = no fix, 1 = GPS, 2 = DGPS ...  */
  uint8_t         status_active;    /* RMC: 1 = 'A', 0 = 'V'                   */
  nmea_antenna_t  antenna;
  uint32_t        update_tick;

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
  uint32_t        update_tick;

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
  uint32_t        update_tick;

} sounder_t;

typedef struct
{
  /* receiver */
  uint8_t         dma_buf[NMEA_DMA_BUFFER_SIZE];
  uint16_t        read_pos;
  uint32_t        poll_tick;
  char            line[NMEA_SENTENCE_MAX_LEN];
  uint8_t         line_len;
  uint8_t         checksum;
  uint8_t         checksum_rx;
  uint8_t         state;
  bool            available;

  /* statistics */
  uint32_t        sentences_ok;
  uint32_t        err_checksum;     /* checksum mismatch                        */
  uint32_t        err_format;       /* too long, bad character, no checksum     */
  uint32_t        err_overrun;      /* nmea_loop() called too late, data lost   */
  uint32_t        err_dma;          /* DMA channel stopped and was restarted    */

  /* decoded data */
  gnss_t          gnss;
  compass_t       compass;
  sounder_t       sounder;

} nmea_t;

//##################################################################################

//								ALL
bool      nmea_init(nmea_t *nmea);
uint16_t  nmea_loop(nmea_t *nmea);
bool      nmea_available(nmea_t *nmea);
void      nmea_available_reset(nmea_t *nmea);

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
bool  nmea_gnss_hdop(nmea_t *nmea, float *data);
bool  nmea_gnss_fix_quality(nmea_t *nmea, uint8_t *data);
bool  nmea_gnss_antenna(nmea_t *nmea, nmea_antenna_t *data);
bool  nmea_gnss_utc(nmea_t *nmea, uint16_t *year, uint8_t *month, uint8_t *day,
                    uint8_t *hour, uint8_t *min, uint8_t *sec);

//								COMPASS
bool  nmea_compass_true_course_deg(nmea_t *nmea, float *data);
bool  nmea_compass_mag_course_deg(nmea_t *nmea, float *data);

//								SOUNDER
bool  nmea_sounder_depth_m(nmea_t *nmea, float *data);
bool  nmea_sounder_depth_offset_m(nmea_t *nmea, float *data);
bool  nmea_sounder_temp_c(nmea_t *nmea, float *data);

#endif
