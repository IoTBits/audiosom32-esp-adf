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

#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "audiosom32_codec.h"
#include "board.h"

static const char *AS_TAG = "AUDIOSOM_CODEC";

#define ES_ASSERT(a, format, b, ...) \
    if ((a) != 0) { \
        ESP_LOGE(AS_TAG, format, ##__VA_ARGS__); \
        return b;\
    }

// SDA is fixed to IO18
// SCL is fixed at IO5
// Pull-up enabled, regardless of external PU
// Clock speed fixed to 100kHz
static i2c_config_t as32_i2c_conf = {
    .mode = I2C_MODE_MASTER;
    .sda_io_num = IIC_DATA;
    .sda_pullup_en = GPIO_PULLUP_ENABLE;
    .scl_io_num = IIC_CLK;
    .scl_pullup_en = GPIO_PULLUP_ENABLE;
    .master.clk_speed = 100000;
};

/* Write operation:

• Start condition 
• Device address with the R/W bit cleared to indicate write 
• Send two bytes for the 16 bit register address (most significant byte first)
• Send two bytes for the 16 bits of data to be written to the register (most significant byte first)
• Stop condition
*/
esp_err_t audiobit_write_reg (i2c_port_t i2c_num, uint16_t reg_addr, uint16_t reg_val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    uint8_t dwr[4];

    // Start condition
    i2c_master_start(cmd);
    // Address + Write bit
    i2c_master_write_byte(cmd, (AUDIOBIT_I2C_ADDR<<1)|WRITE_BIT, ACK_CHECK_EN);
    // MSB for reg address
    //i2c_master_write_byte(cmd, (reg_addr>>8)&0xFF, ACK_CHECK_EN);
    dwr[0] = (reg_addr>>8)&0xFF;
    // LSB for reg address
    //i2c_master_write_byte(cmd, (reg_addr&0xFF), ACK_CHECK_EN);
    dwr[1] = (reg_addr&0xFF);
    // MSB for reg data
    //i2c_master_write_byte(cmd, (reg_val>>8)&0xFF, ACK_CHECK_EN);
    dwr[2] = (reg_val>>8)&0xFF;
    // LSB for reg data
    //i2c_master_write_byte(cmd, (reg_val&0xFF), ACK_CHECK_EN);
    dwr[3] = (reg_val&0xFF);
    i2c_master_write(cmd, dwr, 4, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    // Execute and return status
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* Read operation:

• Start condition
• Device address with the R/W bit cleared to indicate write 
• Send two bytes for the 16 bit register address (most significant byte first)
• Stop Condition followed by start condition (or a single restart condition)
• Device address with the R/W bit set to indicate read 
• Read two bytes from the addressed register (most significant byte first)
• Stop condition
*/
esp_err_t audiobit_read_reg (i2c_port_t i2c_num, uint16_t reg_addr, uint16_t *reg_val)
{
    uint8_t *byte_val = reg_val;		// This will cause warning, please ignore
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Start condition
    i2c_master_start(cmd);
    // Address + Write bit
    i2c_master_write_byte(cmd, (AUDIOBIT_I2C_ADDR<<1)|WRITE_BIT, ACK_CHECK_EN);
    // MSB for reg address
    i2c_master_write_byte(cmd, (reg_addr>>8)&0xFF, ACK_CHECK_EN);
    // LSB for reg address
    i2c_master_write_byte(cmd, (reg_addr&0xFF), ACK_CHECK_EN);

    // Restart (stop + start)
    i2c_master_start(cmd);

    // Address + read
    i2c_master_write_byte(cmd, (AUDIOBIT_I2C_ADDR<<1)|READ_BIT, ACK_CHECK_EN);
    
    // MSB for reg data
    i2c_master_read(cmd, byte_val + 1, 1, ACK_VAL);
    // LSB for reg data
    i2c_master_read_byte(cmd, byte_val, NACK_VAL);
    
    i2c_master_stop(cmd);
    // Execute and return status, should return 0
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c master initialization
 */
void as32_codec_i2c_init()
{
    int res, i2c_master_port = AS32_CODEC_I2C_NUM;
 
    i2c_param_config(AS32_CODEC_I2C_NUM, &as32_i2c_conf);
    i2c_driver_install(AS32_CODEC_I2C_NUM, as32_i2c_conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief Configure ADC and DAC volume.
 *
 * @param mode:             set ADC or DAC or both
 * @param volume:           0 to 100, -1 sets AVC
 * @param dot:              dot parameter is ignored
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
static int as32_set_volume(as32_iface_t iface, int volume)
{
    if (volume < 0 || volume > 100)
    {
        ESP_LOGW(AS_TAG, "Warning: Volume out of bounds, clipped!\n");
        if (volume < 0)
            volume = 0;
        if (volume > 100)
            volume = 100;
    }

    switch (iface)
    {
        case AS32_IFACE_ADC:
            // Change ADC volume
        break;

        case AS32_IFACE_MIC:
            // Change MIC input volume
        break;

        case AS32_IFACE_DAC:
            // Change DAC volume
        break;

        case AS32_IFACE_HP:
            // Change HP volume
        break;

        case AS32_IFACE_LOUT:
            // Change LOUT volume
        break;

        default:
        return -1;
    }
}

/**
 * @brief Power Management
 *
 * @param mod:      if ES_POWER_CHIP, the whole chip including ADC and DAC is enabled
 * @param enable:   false to disable true to enable
 *
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
int as32_power_control(as32_block_t block, bool en)
{
    if (en)
    {
        // Enable ORed blocks
    }
    else
    {
        // Disable ORed blocks
    }
}

/**
 * @brief Config I2s clock in MSATER mode
 *
 * @param cfg.sclkDiv:      generate SCLK by dividing MCLK in MSATER mode
 * @param cfg.lclkDiv:      generate LCLK by dividing MCLK in MSATER mode
 *
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
int es8388_i2s_config_clock(es_i2s_clock_t cfg)
{
    // ????
}

/**
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
esp_err_t es8388_init(audio_hal_codec_config_t *cfg)
{

}

/**
 * @brief Configure ES8388 I2S format
 *
 * @param mode:           set ADC or DAC or all
 * @param bitPerSample:   see Es8388I2sFmt
 *
 * @return
 *     - (-1) Error
 *     - (0)  Success
 */
int es8388_config_fmt(es_module_t mode, es_i2s_fmt_t fmt)

/**
 * @param volume: 0 ~ 100
 *
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
int es8388_set_voice_volume(int volume)
{

}

/**
 *
 * @return
 *           volume
 */
int es8388_get_voice_volume(int *volume)
{

}

/**
 * @brief Configure ES8388 data sample bits
 *
 * @param mode:             set ADC or DAC or all
 * @param bitPerSample:   see BitsLength
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
int es8388_set_bits_per_sample(es_module_t mode, es_bits_length_t bits_length)
{

}

/**
 * @brief Configure ES8388 DAC mute or not. Basicly you can use this function to mute the output or don't
 *
 * @param enable: enable or disable
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
int es8388_set_voice_mute(int enable)
{

}

int es8388_get_voice_mute(void)
{

}

/**
 * @param gain: Config DAC Output
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
int es8388_config_dac_output(int output)
{

}


/**
 * @param gain: Config ADC input
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
int es8388_config_adc_input(es_adc_input_t input)
{

}

/**
 * @param gain: see es_mic_gain_t
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
int es8388_set_mic_gain(es_mic_gain_t gain)
{

}

int es8388_ctrl_state(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state)
{

}

int es8388_config_i2s(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t *iface)
{

}

void es8388_pa_power(bool enable)
{

}


















/*
static i2s_config_t as32_i2s_config = {
    .mode = I2S_MODE_MASTER | I2S_MODE_TX,                                  // Only TX
    .sample_rate = AUDIOBIT_SAMPLERATE,                                     // Default: 48kHz
    .bits_per_sample = AUDIOBIT_BITSPERSAMPLE,                              //16-bit per channel
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,                           //2-channels
    .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
    .dma_buf_count = 6,
    .dma_buf_len = 512,                                                      //
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1                                //Interrupt level 1
};

static i2s_pin_config_t as32_i2s_pin_config = {
    .bck_io_num = AUDIOBIT_BCK,
    .ws_io_num = AUDIOBIT_LRCLK,
    .data_out_num = AUDIOBIT_DOUT,
    .data_in_num = AUDIOBIT_DIN                                                       //Not used
};
*/
