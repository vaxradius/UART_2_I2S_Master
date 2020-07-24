#ifndef RELAJET_I2S_H
#define RELAJET_I2S_H

#define BLE_SCHDLE_TIMER 3

#define MCLK_PIN	5
#define MCLK_TIMER	2
#define MCLK_TIMER_SEG AM_HAL_CTIMER_TIMERA

#define BCLK_PIN		47
#define BCLK_TIMER	2
#define BCLK_TIMER_SEG AM_HAL_CTIMER_TIMERB

#define LRCLK_PIN	12
#define LRCLK_TIMER	0
#define LRCLK_TIMER_SEG AM_HAL_CTIMER_TIMERA

#define BCLK_PIN_INV 26
#define BCLK_TIMER_INV	0
#define BCLK_TIMER_SEG_INV AM_HAL_CTIMER_TIMERB

#define SDATA_PIN     27
#define SDATA_TIMER	1

#define BUF_SIZE		128




//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
extern bool g_bPDMDataReady;

extern int16_t i16PDMBuf[2][BUF_SIZE];
extern int16_t i16I2SBuf[2][BUF_SIZE];
extern uint32_t u32PDMPingpong;
extern uint32_t u32I2SPingpong;

void I2S_init(void);
void pdm_init(void);
void pdm_data_get(int16_t *dest);
#endif // RELAJET_I2S_H


