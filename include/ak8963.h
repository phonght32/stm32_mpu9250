// MIT License

// Copyright (c) 2020 phonght32

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef _AK8963_H_
#define _AK8963_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm_err.h"
#include "driver/i2c.h"

typedef struct ak8963 *ak8963_handle_t;

typedef struct {
    int16_t x_axis;
    int16_t y_axis;
    int16_t z_axis;
} ak8963_raw_data_t;

typedef struct {
    float x_axis;
    float y_axis;
    float z_axis;
} ak8963_cali_data_t;

typedef struct {
    float x_axis;
    float y_axis;
    float z_axis;
} ak8963_scale_data_t;

typedef struct {
    float x_axis;
    float y_axis;
    float z_axis;
} ak8963_hard_iron_bias_t;

typedef struct {
    float x_axis;
    float y_axis;
    float z_axis;
} ak8963_soft_iron_corr_t;

typedef struct {
    float x_axis;
    float y_axis;
    float z_axis;
} ak8963_sens_adj_t;

typedef enum {
    AK8963_MODE_PWR_DOWN = 0x00,                /*!< AK8963 mode power down */
    AK8963_MODE_SINGLE_MEASUREMENT = 0x01,      /*!< AK8963 mode single measurement */
    AK8963_MODE_CONT_MEASUREMENT_1 = 0x02,      /*!< AK8963 mode continous measurement 1 */
    AK8963_MODE_EXT_TRIG_MEASUREMENT = 0x04,    /*!< AK8963 mode external trigger measurement */
    AK8963_MODE_CONT_MEASUREMENT_2 = 0x06,      /*!< AK8963 mode continous measurement 2 */
    AK8963_MODE_SELF_TEST = 0x08,               /*!< AK8963 mode self test */
    AK8963_MODE_FUSE_ROM_ACCESS = 0x0F,         /*!< AK8963 mode fuse ROM access */
    AK8963_MODE_MAX
} ak8963_mode_t;

typedef enum {
    AK8963_MFS_14BIT = 0,                       /*!< Magnetometer 14 bit resolution  */
    AK8963_MFS_16BIT,                           /*!< Magnetometer 16 bit resolution  */
    AK8963_MFS_MAX
} ak8963_mfs_sel_t;

typedef enum {
    AK8963_IF_I2C = 0,                          /*!< Interface over I2C */
    AK8963_IF_SPI,                              /*!< Interface over SPI */
    AK8963_IF_MAX
} ak8963_if_protocol_t;

typedef struct {
    i2c_num_t                   i2c_num;        /*!< AK8963 I2C num */
} ak8963_hardware_info_t;

typedef struct {
    ak8963_mode_t               opr_mode;       /*!< AK8963 operatkion mode */
    ak8963_mfs_sel_t            mfs_sel;        /*!< AK8963 magnetometer full scale range */
    ak8963_hard_iron_bias_t     mag_bias;       /*!< AK8963 bias data */
    ak8963_if_protocol_t        if_protocol;    /*!< AK8963 interface protocol */
    ak8963_hardware_info_t      hw_info;        /*!< AK8963 hardware information */
} ak8963_cfg_t;

/*
 * @brief   Initialize I2C communication and configure AK8963 's parameters
 *          such as clock source, digital low pass filter (DLPF), sleep mode,
 *          gyroscope and accelerometer full scale range, bias value, ...
 * @note:   This function only get I2C_NUM to handler communication, not
 *          configure I2C 's parameters. You have to self configure I2C before
 *          pass I2C into this function.
 * @param   config Struct pointer.
 * @return
 *      - AK8963 handle structure: Success.
 *      - 0: Fail.
 */
ak8963_handle_t ak8963_init(ak8963_cfg_t *config);

/*
 * @brief   Get magnetometer raw value.
 * @param   handle Handle structure.
 * @param   raw_data Raw data.
 * @return
 *      - STM_OK:   Success.
 *      - STM_FAIL: Fail.
 */
stm_err_t ak8963_get_mag_raw(ak8963_handle_t handle, ak8963_raw_data_t *raw_data);

/*
 * @brief   Get magnetometer cali value.
 * @param   handle Handle structure.
 * @param   cali data Cali data.
 * @return
 *      - STM_OK:   Success.
 *      - STM_FAIL: Fail.
 */
stm_err_t ak8963_get_mag_cali(ak8963_handle_t handle, ak8963_cali_data_t *cali_data);

/*
 * @brief   Get magnetometer scale value.
 * @param   handle Handle structure.
 * @param   scale_data Scale data.
 * @return
 *      - STM_OK:   Success.
 *      - STM_FAIL: Fail.
 */
stm_err_t ak8963_get_mag_scale(ak8963_handle_t handle, ak8963_scale_data_t *scale_data);

/*
 * @brief   Set magnetometer hard iron bias value.
 * @param   handle Handle structure.
 * @param   hard_iron_bias Hard iron bias data.
 * @return  None.
 */
void ak8963_set_hard_iron_bias(ak8963_handle_t handle, ak8963_hard_iron_bias_t hard_iron_bias);

/*
 * @brief   Get magnetometer get hard iron bias value.
 * @param   handle Handle structure.
 * @param   hard_iron_bias Hard iron bias data.
 * @return  None.
 */
void ak8963_get_hard_iron_bias(ak8963_handle_t handle, ak8963_hard_iron_bias_t *hard_iron_bias);

/*
 * @brief   Set magnetometer soft iron correction data.
 * @param   handle Handle structure.
 * @param   soft_iron_corr Soft iron correction data.
 * @return  None.
 */
void ak8963_set_soft_iron_corr(ak8963_handle_t handle, ak8963_soft_iron_corr_t soft_iron_corr);

/*
 * @brief   Get magnetometer soft iron correction data.
 * @param   handle Handle structure.
 * @param   soft_iron_corr Soft iron correction data.
 * @return  None.
 */
void ak8963_get_soft_iron_corr(ak8963_handle_t handle, ak8963_soft_iron_corr_t *soft_iron_corr);

/*
 * @brief   Get magnetometer sensitive adjust data.
 * @param   handle Handle structure.
 * @param   mag_sens_adj Sensitive adjustment data.
 * @return  None.
 */
void ak8963_get_mag_sens_adj(ak8963_handle_t handle, ak8963_sens_adj_t *asa);

/*
 * @brief   Auto calibrate all magnetometer bias value.
 * @param   handle Handle structure.
 * @return  None.
 */
void ak8963_auto_calib(ak8963_handle_t handle);


#ifdef __cplusplus
}
#endif

#endif /* _AK8963_H_ */