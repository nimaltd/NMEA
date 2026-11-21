#ifndef _NMEA_CONFIG_H
#define _NMEA_CONFIG_H

/* ----------------------------------------------------------------------------
 * USART and DMA used by the library
 * ------------------------------------------------------------------------- */
#define NMEA_USART                  USART3
#define NMEA_BAUDRATE               38400U          /* used for the overrun check (and by NMEA_HW_INIT) */

#define NMEA_DMA                    DMA1
#define NMEA_DMA_CHANNEL            LL_DMA_CHANNEL_4
/* DMAMUX / CSELR request. Comment out on parts with a fixed channel mapping. */
#define NMEA_DMA_REQUEST            LL_DMAMUX_REQ_USART3_RX

/* ----------------------------------------------------------------------------
 * Buffers and timing
 * ------------------------------------------------------------------------- */
#define NMEA_DMA_BUFFER_SIZE        1024U   /* circular DMA buffer, bytes                 */
#define NMEA_SENTENCE_MAX_LEN       100U    /* longest accepted sentence (standard: 82)   */
#define NMEA_DATA_TIMEOUT_MS        3000U   /* getters return false for older data        */

/* ----------------------------------------------------------------------------
 * 0: GPIO, USART and DMA clock are configured by CubeMX (recommended).
 * 1: the library configures them itself with the macros below
 *    (families with an LL GPIO alternate function field, i.e. not STM32F1).
 * ------------------------------------------------------------------------- */
#define NMEA_HW_INIT                0

#if NMEA_HW_INIT != 0
#define NMEA_USART_CLK_ENABLE()     LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3)
#define NMEA_DMA_CLK_ENABLE()       LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1)
#define NMEA_RX_GPIO_CLK_ENABLE()   LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOD)
#define NMEA_RX_GPIO_PORT           GPIOD
#define NMEA_RX_GPIO_PIN            LL_GPIO_PIN_9
#define NMEA_RX_GPIO_AF             LL_GPIO_AF_0
#endif

#endif
