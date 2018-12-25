/*
Website: www.iot-bits.com

Copyright (C) 2016-2018, IoTBits (Author: Pratik Panda), all right reserved.
E-mail: hello@PratikPanda.com
Personal website: www.PratikPanda.com

The code referencing this license is open source software. Redistribution
and use of the code in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    Redistribution of original and modified source code must retain the 
    above copyright notice, this condition and the following disclaimer.

This software is provided by the copyright holder and contributors "AS IS"
and any warranties related to this software are DISCLAIMED. The copyright 
owner or contributors are NOT LIABLE for any damages caused by use of this 
software.
*/

#ifndef _AUDIOSOM32_CODEC_H_
#define _AUDIOSOM32_CODEC_H_

#include "esp_types.h"
#include "audio_hal.h"
#include "driver/i2c.h"

// AudioSOM32 codec address
#define     AS32_CODEC_ADDR                     0x0A
#define     AS32_CODEC_I2C_NUM                  0

// AudioSOM32 codec register addresses
#define		AS32_CHIP_ID						0x0000
#define		AS32_CHIP_DIG_POWER					0x0002
#define		AS32_CHIP_CLK_CTRL					0x0004
#define		AS32_CHIP_I2S_CTRL					0x0006
#define		AS32_CHIP_SSS_CTRL					0x000A
#define		AS32_CHIP_ADCDAC_CTRL				0x000E
#define		AS32_CHIP_DAC_VOL					0x0010
#define		AS32_CHIP_PAD_STRENGTH				0x0014
#define		AS32_CHIP_ANA_ADC_CTRL				0x0020
#define		AS32_CHIP_ANA_HP_CTRL				0x0022
#define		AS32_CHIP_ANA_CTRL					0x0024
#define		AS32_CHIP_LINREG_CTRL				0x0026
#define		AS32_CHIP_REF_CTRL					0x0028
#define		AS32_CHIP_MIC_CTRL					0x002A
#define		AS32_CHIP_LINE_OUT_CTRL				0x002C
#define		AS32_CHIP_LINE_OUT_VOL				0x002E
#define		AS32_CHIP_ANA_POWER					0x0030
#define		AS32_CHIP_PLL_CTRL					0x0032
#define		AS32_CHIP_CLK_TOP_CTRL				0x0034
#define		AS32_SHIP_ANA_STATUS				0x0036
#define		AS32_CHIP_ANA_TEST1					0x0038
#define		AS32_CHIP_ANA_TEST2					0x003A
#define		AS32_CHIP_SHORT_CTRL				0x003C
#define		AS32_DAP_CONTROL					0x0100
#define		AS32_DAP_PEQ						0x0102
#define		AS32_DAP_BASS_ENHANCE				0x0104
#define		AS32_DAP_BASS_ENHANCE_CTRL			0x0106
#define		AS32_DAP_AUDIO_EQ					0x0108
#define		AS32_DAP_SGTL_SURROUND				0x010A
#define		AS32_DAP_FILTER_COEF_ACCESS			0x010C
#define		AS32_DAP_COEF_WR_B0_MSB				0x010E
#define		AS32_DAP_COEF_WR_B0_LSB				0x0110
#define		AS32_DAP_AUDIO_EQ_BASS_BAND0		0x0116
#define		AS32_DAP_AUDIO_EQ_BAND1				0x0118
#define		AS32_DAP_AUDIO_EQ_BAND2				0x011A
#define		AS32_DAP_AUDIO_EQ_BAND3				0x011C
#define		AS32_DAP_AUDIO_EQ_TREBLE_BAND4		0x011E
#define		AS32_DAP_MAIN_CHAN					0x0120
#define		AS32_DAP_MIX_CHAN					0x0122
#define		AS32_DAP_AVC_CTRL					0x0124
#define		AS32_DAP_AVC_THRESHOLD				0x0126
#define		AS32_DAP_AVC_ATTACK					0x0128
#define		AS32_DAP_AVC_DECAY					0x012A

#define		AS32_DAP_COEF_WR_B1_MSB				0x012C
#define		AS32_DAP_COEF_WR_B1_LSB				0x012E
#define		AS32_DAP_COEF_WR_B2_MSB				0x0130
#define		AS32_DAP_COEF_WR_B2_LSB				0x0132
#define		AS32_DAP_COEF_WR_A1_MSB				0x0134
#define		AS32_DAP_COEF_WR_A1_LSB				0x0136
#define		AS32_DAP_COEF_WR_A2_MSB				0x0138
#define		AS32_DAP_COEF_WR_A2_LSB				0x013A

// Interface types for audiosom32
// Changing ADC or DAC has effect on all assoc peripherals
// Tuning HP, LOUT, etc achieves individual control over that input/output
typedef enum{
    AS32_IFACE_MIN = -1,
    AS32_IFACE_ADC = 0,
    AS32_IFACE_MIC,
    AS32_IFACE_DAC,
    AS32_IFACE_HP,
    AS32_IFACE_LOUT,
    AS32_IFACE_MAX
} as32_iface_t;

typedef enum{
    AS32_BLOCK_ADC = 0,
    AS32_BLOCK_DAC,
    AS32_BLOCK_DAP,
    AS32_BLOCK_DIN,
    AS32_BLOCK_DOUT
} as32_block_t;

typedef enum{
    AS32_MODE_MIN = -1,
    AS32_MODE_SLAVE = 0,
    AS32_MODE_MASTER,
    AS32_MODE_MAX
} as32_mode_t;

// I2S data format
typedef enum {
    AS32_I2S_MIN = -1,
    AS32_I2S_NORMAL = 0,
    AS32_I2S_LEFT = 1,
    AS32_I2S_RIGHT = 2,
    AS32_I2S_DSP = 3,
    AS32_I2S_MAX
} as32_i2s_fmt_t;

// I2S bits per sample
typedef enum {
    BIT_LENGTH_MIN =    -1,
    BIT_LENGTH_16BITS = 0x3,
    BIT_LENGTH_20BITS = 0x2,
    BIT_LENGTH_24BITS = 0x1,
    BIT_LENGTH_32BITS = 0x0,
    BIT_LENGTH_MAX
} as32_bits_length_t;

typedef enum {
    D2SE_PGA_GAIN_MIN = -1,
    D2SE_PGA_GAIN_DIS = 0,
    D2SE_PGA_GAIN_EN = 1,
    D2SE_PGA_GAIN_MAX =2,
} as32_pga_t;

// Input sources for ADC
typedef enum {
    ADC_INPUT_MIN =     -1,
    ADC_INPUT_MIC =    0x0,
    ADC_INPUT_LINEIN = 0x1,
    ADC_INPUT_MAX,
} as32_adc_input_t;

// Output channels for the DAC
typedef enum {
    DAC_OUTPUT_MIN =        -1,
    DAC_OUTPUT_HP =         0x1,
    DAC_OUTPUT_LINEOUT =    0x2,
    DAC_OUTPUT_ALL   =      0x3,
    DAC_OUTPUT_MAX,
} as32_dac_output_t;

typedef enum {
    MIC_GAIN_MIN = -1,
    MIC_GAIN_0DB =  0x0,
    MIC_GAIN_20DB = 0x1,
    MIC_GAIN_30DB = 0x2,
    MIC_GAIN_40DB = 0x3,
    MIC_GAIN_MAX,
} as32_mic_gain_t;

typedef enum {
    ES_MODULE_MIN = -1,
    ES_MODULE_ADC = 0x01,
    ES_MODULE_DAC = 0x02,
    ES_MODULE_ADC_DAC = 0x03,
    ES_MODULE_LINE = 0x04,
    ES_MODULE_MAX
} as32_module_t;

/*
typedef enum {
    MUTE_MIN =  -1,
    MUTE_ADC =  0x0,
    MUTE_HP =   0x4,
    MUTE_LO =   0x8,
    MUTE_MAX
} as32_block_t;
*/

typedef enum {
    BLOCK_MIN =     -1,
    BLOCK_MIC =     0,
    BLOCK_LINEIN =  1,
    BLOCK_ADC =     2,
    BLOCK_DAP =     3,
    BLOCK_DAC =     4,
    BLOCK_HP =      5,
    BLOCK_LINEOUT = 6,
    BLOCK_MAX
} as32_block_t;

typedef enum {
    ES_MODE_MIN = -1,
    ES_MODE_SLAVE = 0x00,
    ES_MODE_MASTER = 0x01,
    ES_MODE_MAX,
} as32_mode_t;

typedef enum {
    ES_I2S_MIN = -1,
    ES_I2S_NORMAL = 0,
    ES_I2S_LEFT = 1,
    ES_I2S_RIGHT = 2,
    ES_I2S_DSP = 3,
    ES_I2S_MAX
} as32_i2s_fmt_t;

typedef struct {
    es_sclk_div_t sclk_div;    /*!< bits clock divide */
    es_lclk_div_t lclk_div;    /*!< WS clock divide */
} as32_i2s_clock_t;

/**
 * @brief Initialize audioSOM32 codec chip
 * Sets up the codec as configured in argument
 * Physical interfaces are set up here
 * 
 * @param cfg configuration of audiosom32 codec
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
esp_err_t as32_init(audio_hal_codec_config_t *cfg);

/**
 * @brief Deinitialize audioSOM32 codec chip
 * Resets the codec config and returns it to POR defaults
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
esp_err_t as32_codec_deinit(void);

/**
 * @brief Configure audiosom32 I2S format
 * Config I2S format related registers in codec
 *
 * @param cfg:   audiosom32 I2S format
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
esp_err_t as32_config_fmt(as32_i2s_fmt_t cfg);

/**
 * @brief Configure I2s clock in master mode
 * Configure all ESP32 timed signals based on settings
 *
 * @param cfg:  set bits clock and WS clock
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
esp_err_t as32_i2s_config_clock(es_i2s_clock_t cfg);

/**
 * @brief Configure audiosom32 data sample bits
 * Set sample length in the codec, applies to all DAC, ADC, etc
 * @param bit_per_sample:  bit number of per sample
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
esp_err_t as32_set_bits_per_sample(es_bits_length_t bit_per_sample);

/**
 * @brief  Start up as32 codec chip modules
 * Power on and activate codec modules based on mode input
 *
 * @param mode:  set ADC or DAC or both
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
esp_err_t as32_start(es_module_t mode);

/**
 * @brief  Stop audiosom32 codec chip
 * Power off modules based on input
 *
 * @param mode:  set ADC or DAC or both
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
esp_err_t as32_stop(es_module_t mode);

/**
 * @brief  Set voice volume
 * Set the ADC volume levels
 *
 * @param volume:  voice volume (0~100)
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
esp_err_t as32_set_voice_volume(int volume);

/**
 * @brief Get voice volume, 0-100
 * Return the ADC volume
 *
 * @param[out] *volume:  voice volume (0~100)
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
esp_err_t as32_get_voice_volume(int *volume);

/**
 * @brief Configure audiosom32 DAC mute mode
 * Mute the codec ADCs
 *
 * @param enable enable(1) or disable(0)
 *
 * @return
 *     - ESP_FAIL Parameter error
 *     - ESP_OK   Success
 */
esp_err_t as32_set_voice_mute(int enable);

/**
 * @brief Get audiosom32 DAC mute status
 * Return codec ADC mute status
 *
 * @return
 *     - -1 Parameter error
 *     - 0 voice mute disable
 *     - 1 voice mute enable
 */
esp_err_t as32_get_voice_mute(void);

/**
 * @brief Set audiosom32 mic gain
 * Set MIC gain in dB at codec
 *
 * @param gain db of mic gain
 *
 * @return
 *     - ESP_FAIL Parameter error
 *     - ESP_OK   Success
 */
esp_err_t as32_set_mic_gain(as32_mic_gain_t gain);

/**
 * @brief Set audiosom32 ADC input mode
 * Configure the input signal source for ADC
 *
 * @param input adc input mode
 *
 * @return
 *     - ESP_FAIL Parameter error
 *     - ESP_OK   Success
 */
esp_err_t as32_config_adc_input(as32_adc_input_t input);

/**
 * @brief Set audiosom32 DAC output mode
 * Configure the DAC input source
 *
 * @param output dac output mode
 *
 * @return
 *     - ESP_FAIL Parameter error
 *     - ESP_OK   Success
 */
esp_err_t as32_config_dac_output(as32_dac_output_t output);

/**
 * @brief Configure audiosom32 codec mode and I2S interface
 *
 * @param mode codec mode
 * @param iface I2S config
 *
 * @return
 *     - ESP_FAIL Parameter error
 *     - ESP_OK   Success
 */
esp_err_t as32_config_i2s(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t *iface);

/**
 * @brief Control audiosom32 codec chip
 *
 * @param mode codec mode
 * @param ctrl_state start or stop decode or encode progress
 *
 * @return
 *     - ESP_FAIL Parameter error
 *     - ESP_OK   Success
 */
esp_err_t as32_ctrl_state(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state);

/**
 * @brief Set audiosom32 PA power
 *
 * @param enable true for enable PA power, false for disable PA power
 *
 * @return
 *     - void
 */
void as32_pa_power(bool enable);

#endif