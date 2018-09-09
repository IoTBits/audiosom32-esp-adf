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

typedef enum {
    AS32_I2S_MIN = -1,
    AS32_I2S_NORMAL = 0,
    AS32_I2S_LEFT = 1,
    AS32_I2S_RIGHT = 2,
    AS32_I2S_DSP = 3,
    AS32_I2S_MAX
} as32_i2s_fmt_t;

/**
 * @brief Initialize audioSOM32 codec chip
 *
 * @param cfg configuration of audiosom32 codec
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
esp_err_t as32_codec_init(audio_hal_codec_config_t *cfg);

/**
 * @brief Deinitialize audioSOM32 codec chip
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
esp_err_t as32_codec_deinit(void);

/**
 * @brief Configure audiosom32 I2S format
 *
 * @param mod:  set ADC or DAC or both
 * @param cfg:   audiosom32 I2S format
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
esp_err_t as32_config_fmt(es_module_t mod, es_i2s_fmt_t cfg);

/**
 * @brief Configure I2s clock in master mode
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
 *
 * @param mode:  set ADC or DAC or both
 * @param bit_per_sample:  bit number of per sample
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
esp_err_t as32_set_bits_per_sample(es_module_t mode, es_bits_length_t bit_per_sample);

/**
 * @brief  Start up as32 codec chip
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
 *
 * @param volume:  voice volume (0~100)
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
esp_err_t as32_set_voice_volume(int volume);

/**
 * @brief Get voice volume
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
 *
 * @return
 *     - -1 Parameter error
 *     - 0 voice mute disable
 *     - 1 voice mute enable
 */
esp_err_t as32_get_voice_mute(void);

/**
 * @brief Set audiosom32 mic gain
 *
 * @param gain db of mic gain
 *
 * @return
 *     - ESP_FAIL Parameter error
 *     - ESP_OK   Success
 */
esp_err_t as32_set_mic_gain(es_mic_gain_t gain);

/**
 * @brief Set audiosom32 ADC input mode
 *
 * @param input adc input mode
 *
 * @return
 *     - ESP_FAIL Parameter error
 *     - ESP_OK   Success
 */
esp_err_t as32_config_adc_input(es_adc_input_t input);

/**
 * @brief Set audiosom32 DAC output mode
 *
 * @param output dac output mode
 *
 * @return
 *     - ESP_FAIL Parameter error
 *     - ESP_OK   Success
 */
esp_err_t as32_config_dac_output(es_dac_output_t output);

/**
 * @brief Write audiosom32 codec register
 *
 * @param reg_add address of register
 * @param data data of register
 *
 * @return
 *     - ESP_FAIL Parameter error
 *     - ESP_OK   Success
 */
esp_err_t as32_write_reg(uint8_t reg_add, uint8_t data);

/**
 * @brief Print all audiosom32 codec registers
 *
 * @return
 *     - void
 */
void as32_read_all();

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