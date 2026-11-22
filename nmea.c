/*
 *	Author:     Nima Askari
 *	WebSite:    https://www.github.com/NimaLTD
 *
 *	Version 3 edited by: Mahdi Zamani (https://github.com/zamanimahdi)
 *
 *	Polled, circular-DMA NMEA-0183 decoder. See nmea.h (history) and README.md.
 */

#include <string.h>
#include "nmea.h"

/* Polling later than this may have let the DMA overwrite unread data */
#define NMEA_OVERRUN_MS   (((NMEA_DMA_BUFFER_SIZE - 1U) * 10U * 1000U) / NMEA_BAUDRATE)
#define NMEA_MAX_FIELDS   24U
#define NMEA_REACK_TIMEOUT_MS 10U

/* Sentence state machine */
enum {
  ST_WAIT_START = 0,   /* waiting for '$'                         */
  ST_PAYLOAD,          /* storing address + data fields, XOR      */
  ST_CHECKSUM_HI,      /* first hex digit after '*'               */
  ST_CHECKSUM_LO,      /* second hex digit                        */
  ST_END               /* waiting for "\r\n"                      */
};

/* ============================================================================
 * Hardware (no interrupts are used)
 * ========================================================================= */

#if defined(USART_RDR_RDR)
#define NMEA_USART_RX_REG   ((uint32_t) &NMEA_USART->RDR)
#else
#define NMEA_USART_RX_REG   ((uint32_t) &NMEA_USART->DR)
#endif

static uint16_t nmea_dma_write_pos(void)
{
  uint32_t remaining = LL_DMA_GetDataLength(NMEA_DMA, NMEA_DMA_CHANNEL);
  return (uint16_t) ((NMEA_DMA_BUFFER_SIZE - remaining) % NMEA_DMA_BUFFER_SIZE);
}

static void nmea_dma_start(nmea_t *nmea)
{
  LL_DMA_DisableChannel(NMEA_DMA, NMEA_DMA_CHANNEL);
  while (LL_DMA_IsEnabledChannel(NMEA_DMA, NMEA_DMA_CHANNEL))
  {
  }
#ifdef NMEA_DMA_REQUEST
  LL_DMA_SetPeriphRequest(NMEA_DMA, NMEA_DMA_CHANNEL, NMEA_DMA_REQUEST);
#endif
  LL_DMA_ConfigTransfer(NMEA_DMA, NMEA_DMA_CHANNEL,
      LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_PRIORITY_LOW
      | LL_DMA_MODE_CIRCULAR | LL_DMA_PERIPH_NOINCREMENT
      | LL_DMA_MEMORY_INCREMENT | LL_DMA_PDATAALIGN_BYTE
      | LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_SetPeriphAddress(NMEA_DMA, NMEA_DMA_CHANNEL, NMEA_USART_RX_REG);
  LL_DMA_SetMemoryAddress(NMEA_DMA, NMEA_DMA_CHANNEL, (uint32_t) nmea->dma_buf);
  LL_DMA_SetDataLength(NMEA_DMA, NMEA_DMA_CHANNEL, NMEA_DMA_BUFFER_SIZE);
  LL_DMA_DisableIT_TC(NMEA_DMA, NMEA_DMA_CHANNEL);
  LL_DMA_DisableIT_HT(NMEA_DMA, NMEA_DMA_CHANNEL);
  LL_DMA_DisableIT_TE(NMEA_DMA, NMEA_DMA_CHANNEL);
  LL_DMA_EnableChannel(NMEA_DMA, NMEA_DMA_CHANNEL);

  nmea->read_pos = 0;
  nmea->state = ST_WAIT_START;
}

static bool nmea_hw_init(nmea_t *nmea)
{
#if NMEA_HW_INIT != 0
  NMEA_RX_GPIO_CLK_ENABLE();
  NMEA_USART_CLK_ENABLE();
  NMEA_DMA_CLK_ENABLE();

  /* RX pin, pull-up keeps the line idle if the receiver is not connected */
  LL_GPIO_InitTypeDef gpio = {0};
  gpio.Pin = NMEA_RX_GPIO_PIN;
  gpio.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio.Speed = LL_GPIO_SPEED_FREQ_LOW;
  gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio.Pull = LL_GPIO_PULL_UP;
  gpio.Alternate = NMEA_RX_GPIO_AF;
  LL_GPIO_Init(NMEA_RX_GPIO_PORT, &gpio);

  /* 8N1, receive only */
  LL_USART_Disable(NMEA_USART);
  LL_USART_InitTypeDef usart = {0};
#if defined(USART_PRESC_PRESCALER)
  usart.PrescalerValue = LL_USART_PRESCALER_DIV1;
#endif
  usart.BaudRate = NMEA_BAUDRATE;
  usart.DataWidth = LL_USART_DATAWIDTH_8B;
  usart.StopBits = LL_USART_STOPBITS_1;
  usart.Parity = LL_USART_PARITY_NONE;
  usart.TransferDirection = LL_USART_DIRECTION_RX;
  usart.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  usart.OverSampling = LL_USART_OVERSAMPLING_16;
  if (LL_USART_Init(NMEA_USART, &usart) != SUCCESS)
    return false;
  LL_USART_ConfigAsyncMode(NMEA_USART);
#endif

  /* The USART is configured (by CubeMX or above); attach the DMA */
  LL_USART_Disable(NMEA_USART);
#if defined(USART_CR1_FIFOEN)
  LL_USART_DisableFIFO(NMEA_USART);
#endif
#if defined(USART_CR3_OVRDIS)
  /* Without overrun detection the receiver never blocks: a lost byte only
   * breaks one sentence, which the checksum then rejects. */
  LL_USART_DisableOverrunDetect(NMEA_USART);
#endif
  LL_USART_EnableDMAReq_RX(NMEA_USART);
  nmea_dma_start(nmea);
  LL_USART_Enable(NMEA_USART);

#if defined(USART_ISR_REACK)
  uint32_t start = HAL_GetTick();
  while (!LL_USART_IsActiveFlag_REACK(NMEA_USART))
  {
    if (HAL_GetTick() - start > NMEA_REACK_TIMEOUT_MS)
      return false;
  }
#endif
  return true;
}

/* ============================================================================
 * Field parsers. A field is a '\0' terminated string, "" when empty.
 * ========================================================================= */

static bool nmea_is_digits(const char *s, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    if (s[i] < '0' || s[i] > '9')
      return false;
  }
  return true;
}

static uint8_t nmea_two_digits(const char *s) {
  return (uint8_t) ((s[0] - '0') * 10 + (s[1] - '0'));
}

/* [-]ddd[.ddd] -> float. False for an empty or malformed field. */
static bool nmea_parse_float(const char *s, float *out) {
  bool negative = false, digits = false;
  float value = 0.0f, scale = 1.0f;

  if (*s == '-' || *s == '+')
    negative = (*s++ == '-');
  while (*s >= '0' && *s <= '9') {
    value = value * 10.0f + (float) (*s++ - '0');
    digits = true;
  }
  if (*s == '.') {
    s++;
    while (*s >= '0' && *s <= '9') {
      scale *= 0.1f;
      value += (float) (*s++ - '0') * scale;
      digits = true;
    }
  }
  if (!digits || *s != '\0')
    return false;
  *out = negative ? -value : value;
  return true;
}

static bool nmea_parse_uint(const char *s, uint32_t *out) {
  uint32_t value = 0;
  if (*s == '\0')
    return false;
  while (*s >= '0' && *s <= '9')
    value = value * 10U + (uint32_t) (*s++ - '0');
  if (*s != '\0')
    return false;
  *out = value;
  return true;
}

/* (d)ddmm.mmmm + hemisphere -> signed decimal degrees.
 * Integer and fraction are parsed separately so the minutes keep their full
 * resolution until the final division. */
static bool nmea_parse_coord(const char *s, const char *hemi, float *out) {
  uint32_t whole = 0, frac = 0, fracScale = 1;

  if (*s < '0' || *s > '9')
    return false;
  while (*s >= '0' && *s <= '9')
    whole = whole * 10U + (uint32_t) (*s++ - '0');
  if (*s == '.') {
    s++;
    while (*s >= '0' && *s <= '9') {
      if (fracScale < 1000000U) { /* up to 6 decimals of a minute */
        frac = frac * 10U + (uint32_t) (*s - '0');
        fracScale *= 10U;
      }
      s++;
    }
  }
  if (*s != '\0')
    return false;

  uint32_t degrees = whole / 100U;
  uint32_t minutes = whole % 100U;
  if (minutes >= 60U || degrees > 180U)
    return false;

  float value = (float) degrees
      + ((float) minutes + (float) frac / (float) fracScale) / 60.0f;

  switch (hemi[0]) {
  case 'N':
  case 'E':
    break;
  case 'S':
  case 'W':
    value = -value;
    break;
  default:
    return false;
  }
  *out = value;
  return true;
}

/* hhmmss[.ss] */
static bool nmea_parse_time(const char *s, uint8_t *h, uint8_t *m, uint8_t *sec) {
  if (!nmea_is_digits(s, 6))
    return false;
  uint8_t hh = nmea_two_digits(&s[0]);
  uint8_t mm = nmea_two_digits(&s[2]);
  uint8_t ss = nmea_two_digits(&s[4]);
  if (hh > 23 || mm > 59 || ss > 60) /* 60 = leap second */
    return false;
  *h = hh;
  *m = mm;
  *sec = (ss == 60) ? 59 : ss;
  return true;
}

/* ddmmyy */
static bool nmea_parse_date(const char *s, uint8_t *d, uint8_t *m, uint8_t *y) {
  if (!nmea_is_digits(s, 6) || s[6] != '\0')
    return false;
  uint8_t dd = nmea_two_digits(&s[0]);
  uint8_t mm = nmea_two_digits(&s[2]);
  if (dd < 1 || dd > 31 || mm < 1 || mm > 12)
    return false;
  *d = dd;
  *m = mm;
  *y = nmea_two_digits(&s[4]);
  return true;
}

/* Splits line in place at ','. Missing fields point to "" so indexing past
 * the last field is always safe. strtok() is not used because it merges
 * empty fields ("V,,,,") and shifts every following field. */
static void nmea_split(char *line, char *field[NMEA_MAX_FIELDS]) {
  static char empty[] = "";
  uint8_t count = 1;

  for (uint8_t i = 0; i < NMEA_MAX_FIELDS; i++)
    field[i] = empty;
  field[0] = line;
  for (char *p = line; *p != '\0'; p++) {
    if (*p == ',') {
      *p = '\0';
      if (count < NMEA_MAX_FIELDS)
        field[count++] = p + 1;
    }
  }
}

/* ============================================================================
 * Sentence decoders. f[0] is the address ("GPRMC"), f[1] the first data field.
 * ========================================================================= */

/* $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,q,ss,h.h,alt,M,geo,M,age,ref*hh */
static void nmea_decode_gga(nmea_t *nmea, char **f) {
  gnss_t *g = &nmea->gnss;
  uint32_t u;
  float v;

  if (nmea_parse_uint(f[6], &u)) {
    g->fix_quality = (uint8_t) u;
    g->valid.fix_quality = 1;
  }
  if (nmea_parse_uint(f[7], &u)) {
    g->satellite = (uint8_t) u;
    g->valid.satellite = 1;
  }

  if (g->valid.fix_quality && g->fix_quality > 0) {
    uint8_t h, m, s;
    if (nmea_parse_time(f[1], &h, &m, &s)) {
      /* GGA has no date: if its time differs from the last RMC, the
       * date belongs to another epoch, so date+time is no longer a pair */
      if (h != g->time_h || m != g->time_m || s != g->time_s)
        g->valid.utc = 0;
      g->time_h = h;
      g->time_m = m;
      g->time_s = s;
      g->valid.time = 1;
    }
    if (nmea_parse_coord(f[2], f[3], &v)) {
      g->latitude_deg = v;
      g->valid.latitude = 1;
    }
    if (nmea_parse_coord(f[4], f[5], &v)) {
      g->longitude_deg = v;
      g->valid.longitude = 1;
    }
    if (nmea_parse_float(f[8], &v)) {
      g->precision_m = v;
      g->valid.precision = 1;
    }
    if (nmea_parse_float(f[9], &v)) {
      g->altitude_m = v;
      g->valid.altitude = 1;
    }
  } else {
    /* No fix: what GGA provides is no longer trustworthy */
    g->valid.latitude = 0;
    g->valid.longitude = 0;
    g->valid.precision = 0;
    g->valid.altitude = 0;
  }
  g->update_tick = HAL_GetTick();
}

/* $--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a[,m[,s]]*hh */
static void nmea_decode_rmc(nmea_t *nmea, char **f) {
  gnss_t *g = &nmea->gnss;
  float v;

  g->status_active = (f[2][0] == 'A') ? 1 : 0;

  if (g->status_active) {
    bool timeOk = nmea_parse_time(f[1], &g->time_h, &g->time_m, &g->time_s);
    bool dateOk = nmea_parse_date(f[9], &g->date_d, &g->date_m, &g->date_y);
    g->valid.time = timeOk;
    g->valid.date = dateOk;
    g->valid.utc = timeOk && dateOk;

    if (nmea_parse_coord(f[3], f[4], &v)) {
      g->latitude_deg = v;
      g->valid.latitude = 1;
    }
    if (nmea_parse_coord(f[5], f[6], &v)) {
      g->longitude_deg = v;
      g->valid.longitude = 1;
    }
    if (nmea_parse_float(f[7], &v)) {
      g->speed_knots = v;
      g->valid.speed_knots = 1;
    }
    if (nmea_parse_float(f[8], &v)) {
      g->course_deg = v;
      g->valid.course = 1;
    } else {
      g->valid.course = 0; /* course is empty when not moving */
    }
  } else {
    g->valid.time = 0;
    g->valid.date = 0;
    g->valid.utc = 0;
    g->valid.latitude = 0;
    g->valid.longitude = 0;
    g->valid.speed_knots = 0;
    g->valid.course = 0;
  }
  g->update_tick = HAL_GetTick();
}

/* $--HDT,x.x,T*hh */
static void nmea_decode_hdt(nmea_t *nmea, char **f) {
  float v;
  nmea->compass.valid.true_compass = nmea_parse_float(f[1], &v);
  if (nmea->compass.valid.true_compass)
    nmea->compass.true_course_deg = v;
  nmea->compass.update_tick = HAL_GetTick();
}

/* $--HDM,x.x,M*hh */
static void nmea_decode_hdm(nmea_t *nmea, char **f) {
  float v;
  nmea->compass.valid.mag_compass = nmea_parse_float(f[1], &v);
  if (nmea->compass.valid.mag_compass)
    nmea->compass.mag_course_deg = v;
  nmea->compass.update_tick = HAL_GetTick();
}

/* $--DPT,depth,offset[,range]*hh */
static void nmea_decode_dpt(nmea_t *nmea, char **f) {
  float v;
  nmea->sounder.valid.depth = nmea_parse_float(f[1], &v);
  if (nmea->sounder.valid.depth)
    nmea->sounder.depth_m = v;
  nmea->sounder.valid.depth_offset = nmea_parse_float(f[2], &v);
  if (nmea->sounder.valid.depth_offset)
    nmea->sounder.depth_offset_m = v;
  nmea->sounder.update_tick = HAL_GetTick();
}

/* $--MTW,x.x,C*hh  (F is converted to C) */
static void nmea_decode_mtw(nmea_t *nmea, char **f) {
  float v;
  bool ok = nmea_parse_float(f[1], &v);
  if (ok && f[2][0] == 'F')
    v = (v - 32.0f) * 5.0f / 9.0f;
  else if (f[2][0] != 'C')
    ok = false;
  nmea->sounder.valid.temp = ok;
  if (ok)
    nmea->sounder.temp_c = v;
  nmea->sounder.update_tick = HAL_GetTick();
}

/* $GNTXT,01,01,02,ANTSTATUS=OK*hh   (u-blox, also OPEN / SHORT / INIT) */
static void nmea_decode_txt(nmea_t *nmea, const char *line) {
  const char *p = strstr(line, "ANTSTATUS=");
  if (p == NULL)
    return;
  p += 10;
  if (strncmp(p, "OK", 2) == 0)
    nmea->gnss.antenna = NMEA_ANTENNA_OK;
  else if (strncmp(p, "OPEN", 4) == 0)
    nmea->gnss.antenna = NMEA_ANTENNA_OPEN;
  else if (strncmp(p, "SHORT", 5) == 0)
    nmea->gnss.antenna = NMEA_ANTENNA_SHORT;
  else
    nmea->gnss.antenna = NMEA_ANTENNA_UNKNOWN;
  nmea->gnss.valid.antenna = 1;
}

/* line = "GPRMC,....." (between '$' and '*'), checksum already verified */
static uint16_t nmea_decode(nmea_t *nmea) {
  char *f[NMEA_MAX_FIELDS];
  char *line = nmea->line;

  /* Address = 2 character talker + 3 character type ("GPRMC").
   * Proprietary sentences ($PUBX, $PGRM...) are ignored. */
  const char *comma = strchr(line, ',');
  if (line[0] == 'P' || comma == NULL || comma - line != 5)
    return 0;
  const char *type = &line[2];

  if (strncmp(type, "TXT", 3) == 0) {
    nmea_decode_txt(nmea, line);
    return NMEA_MASK_TXT;
  }

  nmea_split(line, f);

  if (strncmp(type, "GGA", 3) == 0) {
    nmea_decode_gga(nmea, f);
    return NMEA_MASK_GGA;
  }
  if (strncmp(type, "RMC", 3) == 0) {
    nmea_decode_rmc(nmea, f);
    return NMEA_MASK_RMC;
  }
  if (strncmp(type, "HDT", 3) == 0) {
    nmea_decode_hdt(nmea, f);
    return NMEA_MASK_HDT;
  }
  if (strncmp(type, "HDM", 3) == 0) {
    nmea_decode_hdm(nmea, f);
    return NMEA_MASK_HDM;
  }
  if (strncmp(type, "DPT", 3) == 0) {
    nmea_decode_dpt(nmea, f);
    return NMEA_MASK_DPT;
  }
  if (strncmp(type, "MTW", 3) == 0) {
    nmea_decode_mtw(nmea, f);
    return NMEA_MASK_MTW;
  }
  return 0;
}

/* ============================================================================
 * Byte state machine
 * ========================================================================= */

static uint8_t nmea_hex(char c) {
  if (c >= '0' && c <= '9')
    return (uint8_t) (c - '0');
  if (c >= 'A' && c <= 'F')
    return (uint8_t) (c - 'A' + 10);
  if (c >= 'a' && c <= 'f')
    return (uint8_t) (c - 'a' + 10);
  return 0xFF;
}

/* Returns the NMEA_MASK_xxx of a completed, valid sentence, otherwise 0 */
static uint16_t nmea_process_byte(nmea_t *nmea, char c) {
  if (c == '$') { /* always (re)starts a sentence */
    nmea->line_len = 0;
    nmea->checksum = 0;
    nmea->state = ST_PAYLOAD;
    return 0;
  }

  switch (nmea->state) {
  case ST_PAYLOAD:
    if (c == '*') {
      nmea->line[nmea->line_len] = '\0';
      nmea->checksum_rx = 0;
      nmea->state = ST_CHECKSUM_HI;
    } else if (c < 0x20 || c > 0x7E || nmea->line_len >= NMEA_SENTENCE_MAX_LEN - 1U) {
      nmea->err_format++; /* control character, no checksum, or too long */
      nmea->state = ST_WAIT_START;
    } else {
      nmea->line[nmea->line_len++] = c;
      nmea->checksum ^= (uint8_t) c;
    }
    break;

  case ST_CHECKSUM_HI:
  case ST_CHECKSUM_LO: {
    uint8_t h = nmea_hex(c);
    if (h == 0xFF) {
      nmea->err_format++;
      nmea->state = ST_WAIT_START;
    } else {
      nmea->checksum_rx = (uint8_t) ((nmea->checksum_rx << 4) | h);
      nmea->state++;
    }
  }
    break;

  case ST_END:
    if (c == '\r')
      break;
    nmea->state = ST_WAIT_START;
    if (c != '\n') {
      nmea->err_format++;
      break;
    }
    if (nmea->checksum_rx != nmea->checksum) {
      nmea->err_checksum++;
      break;
    }
    nmea->sentences_ok++;
    {
      uint16_t mask = nmea_decode(nmea);
      if (mask)
        nmea->available = true;
      return mask;
    }

  default: /* ST_WAIT_START: ignore everything until '$' */
    break;
  }
  return 0;
}

/* ============================================================================
 * Public: ALL
 * ========================================================================= */

bool nmea_init(nmea_t *nmea) {
  memset(nmea, 0, sizeof(*nmea));
  bool ok = nmea_hw_init(nmea);
  nmea->poll_tick = HAL_GetTick();
  return ok;
}

uint16_t nmea_loop(nmea_t *nmea) {
  uint16_t mask = 0;
  uint32_t now = HAL_GetTick();

  /* A DMA transfer error disables the channel: restart it */
  if (!LL_DMA_IsEnabledChannel(NMEA_DMA, NMEA_DMA_CHANNEL)) {
    nmea->err_dma++;
    nmea_dma_start(nmea);
    nmea->poll_tick = now;
    return 0;
  }

  /* Late poll: the DMA may have lapped the read position. The unread data
   * cannot be trusted, skip to the newest byte and resynchronise on '$'. */
  if (now - nmea->poll_tick > NMEA_OVERRUN_MS) {
    nmea->err_overrun++;
    nmea->read_pos = nmea_dma_write_pos();
    nmea->state = ST_WAIT_START;
  }
  nmea->poll_tick = now;

#if defined(USART_ICR_FECF)
  /* Framing/noise/parity flags do not stop the DMA; just clear them */
  LL_USART_ClearFlag_FE(NMEA_USART);
  LL_USART_ClearFlag_NE(NMEA_USART);
  LL_USART_ClearFlag_PE(NMEA_USART);
#if !defined(USART_CR3_OVRDIS)
  LL_USART_ClearFlag_ORE(NMEA_USART);
#endif
#endif

  uint16_t writePos = nmea_dma_write_pos();
  while (nmea->read_pos != writePos) {
    char c = (char) nmea->dma_buf[nmea->read_pos];
    nmea->read_pos = (uint16_t) ((nmea->read_pos + 1U) % NMEA_DMA_BUFFER_SIZE);
    mask |= nmea_process_byte(nmea, c);
  }
  return mask;
}

bool nmea_available(nmea_t *nmea) {
  return nmea->available;
}

/* Clears the flag and all "valid" bits (same behaviour as v2) */
void nmea_available_reset(nmea_t *nmea) {
  memset(&nmea->gnss.valid, 0, sizeof(nmea->gnss.valid));
  memset(&nmea->compass.valid, 0, sizeof(nmea->compass.valid));
  memset(&nmea->sounder.valid, 0, sizeof(nmea->sounder.valid));
  nmea->available = false;
}

/* ============================================================================
 * Public: getters
 * ========================================================================= */

static bool nmea_fresh(uint32_t tick) {
  return (HAL_GetTick() - tick) < NMEA_DATA_TIMEOUT_MS;
}

#define NMEA_GET(group, flag, member, type)                              \
  bool nmea_##group##_##member(nmea_t *nmea, type *data) {             \
    if (nmea == NULL || !nmea->group.valid.flag                       \
        || !nmea_fresh(nmea->group.update_tick))                  \
      return false;                                                 \
    if (data != NULL)                                                 \
      *data = nmea->group.member;                                   \
    return true;                                                      \
  }

NMEA_GET(gnss, time, time_h, uint8_t)
NMEA_GET(gnss, time, time_m, uint8_t)
NMEA_GET(gnss, time, time_s, uint8_t)
NMEA_GET(gnss, date, date_y, uint8_t)
NMEA_GET(gnss, date, date_m, uint8_t)
NMEA_GET(gnss, date, date_d, uint8_t)
NMEA_GET(gnss, satellite, satellite, uint8_t)
NMEA_GET(gnss, speed_knots, speed_knots, float)
NMEA_GET(gnss, precision, precision_m, float)
NMEA_GET(gnss, course, course_deg, float)
NMEA_GET(gnss, latitude, latitude_deg, float)
NMEA_GET(gnss, longitude, longitude_deg, float)
NMEA_GET(gnss, altitude, altitude_m, float)
NMEA_GET(gnss, fix_quality, fix_quality, uint8_t)
NMEA_GET(compass, true_compass, true_course_deg, float)
NMEA_GET(compass, mag_compass, mag_course_deg, float)
NMEA_GET(sounder, depth, depth_m, float)
NMEA_GET(sounder, depth_offset, depth_offset_m, float)
NMEA_GET(sounder, temp, temp_c, float)

bool nmea_gnss_speed_kph(nmea_t *nmea, float *data) {
  float knots;
  if (!nmea_gnss_speed_knots(nmea, &knots))
    return false;
  if (data != NULL)
    *data = knots * 1.852f;
  return true;
}

bool nmea_gnss_hdop(nmea_t *nmea, float *data) {
  return nmea_gnss_precision_m(nmea, data);
}

/* Antenna status is only sent at start-up or on change: no timeout */
bool nmea_gnss_antenna(nmea_t *nmea, nmea_antenna_t *data) {
  if (nmea == NULL || !nmea->gnss.valid.antenna)
    return false;
  if (data != NULL)
    *data = nmea->gnss.antenna;
  return true;
}

bool nmea_gnss_utc(nmea_t *nmea, uint16_t *year, uint8_t *month, uint8_t *day,
    uint8_t *hour, uint8_t *min, uint8_t *sec) {
  if (nmea == NULL || !nmea->gnss.valid.utc
      || !nmea_fresh(nmea->gnss.update_tick))
    return false;
  if (year)
    *year = (uint16_t) (2000U + nmea->gnss.date_y);
  if (month)
    *month = nmea->gnss.date_m;
  if (day)
    *day = nmea->gnss.date_d;
  if (hour)
    *hour = nmea->gnss.time_h;
  if (min)
    *min = nmea->gnss.time_m;
  if (sec)
    *sec = nmea->gnss.time_s;
  return true;
}
