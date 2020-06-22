#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "stm_log.h"
#include "ak8963.h"

#define AK8963_WHO_AM_I             0x00
#define AK8963_INFO                 0x01
#define AK8963_ST1                  0x02
#define AK8963_XOUT_L               0x03
#define AK8963_XOUT_H               0x04
#define AK8963_YOUT_L               0x05
#define AK8963_YOUT_H               0x06
#define AK8963_ZOUT_L               0x07
#define AK8963_ZOUT_H               0x08
#define AK8963_ST2                  0x09
#define AK8963_CNTL                 0x0A
#define AK8963_RSV                  0x0B
#define AK8963_ASTC                 0x0C
#define AK8963_TS1                  0x0D
#define AK8963_TS2                  0x0E
#define AK8963_I2CDIS               0x0F
#define AK8963_ASAX                 0x10
#define AK8963_ASAY                 0x11
#define AK8963_ASAZ                 0x12
#define AK8963_ADDR                 (0x0C<<1)

#define TIMEOUT_MS_DEFAULT          100         /*!< Default MPU9250 I2C communiation timeout */
#define BUFFER_CALIB_DEFAULT        1000        /*!< Default the number of sample data when calibrate */

#define AK8963_INIT_ERR_STR        "ak8963 init error"
#define AK8963_MALLOC_ERR_STR      "ak8963 malloc error"
#define AK8963_TRANS_ERR_STR       "ak8963 write registers error"
#define AK8963_REC_ERR_STR         "ak8963 read registers error"
#define AK8963_GET_DATA_ERR_STR    "ak8963 get data error"

#define mutex_lock(x)               while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x)             xSemaphoreGive(x)
#define mutex_create()              xSemaphoreCreateMutex()
#define mutex_destroy(x)            vQueueDelete(x)

static const char* AK8963_TAG = "AK8963";
#define AK8963_CHECK(a, str, ret)  if(!(a)) {                                               \
        STM_LOGE(AK8963_TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);        \
        return (ret);                                                                       \
        }

typedef stm_err_t (*read_func)(ak8963_handle_t handle, uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms);
typedef stm_err_t (*write_func)(ak8963_handle_t handle, uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms);

typedef struct ak8963 {
    ak8963_mode_t               opr_mode;               /*!< AK8963 operatkion mode */
    ak8963_mfs_sel_t            mfs_sel;                /*!< AK8963 magnetometer full scale range */
    ak8963_if_protocol_t        if_protocol;            /*!< AK8963 interface protocol */
    ak8963_hard_iron_bias_t     hard_iron_bias;         /*!< AK8963 magnetometer bias data */
    ak8963_sens_adj_t           asa;                    /*!< AK8963 magnetometer sensitive adjust data */
    ak8963_soft_iron_corr_t     soft_iron_corr;         /*!< AK8963 magnetometer scale */
    float                       mag_scaling_factor;     /*!< AK8963 magnetometer scaling factor */
    i2c_num_t                   i2c_num;                /*!< AK8963 I2C num */
    SemaphoreHandle_t           lock;                   /*!< AK8963 mutex */
    read_func                   _read;                  /*!< AK8963 read function */
    write_func                  _write;                 /*!< AK8963 write function */
} ak8963_t;

static stm_err_t _i2c_write_func(ak8963_handle_t handle, uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    uint8_t buf_send[len + 1];
    buf_send[0] = reg_addr;
    for (uint8_t i = 0; i < len; i++)
    {
        buf_send[i + 1] = buf[i];
    }

    AK8963_CHECK(!i2c_write_bytes(handle->i2c_num, AK8963_ADDR, buf_send, len + 1, timeout_ms), AK8963_TRANS_ERR_STR, STM_FAIL);
    return STM_OK;
}

static stm_err_t _i2c_read_func(ak8963_handle_t handle, uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    uint8_t buffer[1];
    buffer[0] = reg_addr;
    AK8963_CHECK(!i2c_write_bytes(handle->i2c_num, AK8963_ADDR, buffer, 1, timeout_ms), AK8963_REC_ERR_STR, STM_FAIL);
    AK8963_CHECK(!i2c_read_bytes(handle->i2c_num, AK8963_ADDR, buf, len, timeout_ms), AK8963_REC_ERR_STR, STM_FAIL);

    return STM_OK;
}

static void _ak8963_set_func(ak8963_handle_t handle, read_func _read, write_func _write)
{
    handle->_read = _read;
    handle->_write = _write;
}

ak8963_handle_t ak8963_init(ak8963_cfg_t *config)
{
    /* Check if init structure is empty */
    AK8963_CHECK(config, AK8963_INIT_ERR_STR, NULL);
    AK8963_CHECK(config->opr_mode < AK8963_MODE_MAX, AK8963_INIT_ERR_STR, NULL);
    AK8963_CHECK(config->mfs_sel < AK8963_MFS_MAX, AK8963_INIT_ERR_STR, NULL);
    AK8963_CHECK(config->if_protocol < AK8963_IF_MAX, AK8963_INIT_ERR_STR, NULL);

    /* Allocate memory for handle structure */
    ak8963_handle_t handle;
    handle = calloc(1, sizeof(ak8963_t));
    AK8963_CHECK(handle, AK8963_INIT_ERR_STR, NULL);

    handle->i2c_num = config->i2c_num;

    if (config->if_protocol == AK8963_IF_I2C)
    {
        _ak8963_set_func(handle, _i2c_read_func, _i2c_write_func);
    }

    /* Power down AK8963 magnetic sensor */
    uint8_t buffer = 0;
    buffer = 0x00;
    AK8963_CHECK(!handle->_write(handle, AK8963_CNTL, &buffer, 1, TIMEOUT_MS_DEFAULT), AK8963_INIT_ERR_STR, NULL);
    HAL_Delay(10);

    /* Set fuse ROM access mode */
    buffer = 0x0F;
    AK8963_CHECK(!handle->_write(handle, AK8963_CNTL, &buffer, 1, TIMEOUT_MS_DEFAULT), AK8963_INIT_ERR_STR, NULL);
    HAL_Delay(10);

    /* Read magnetic sensitivity adjustment */
    uint8_t mag_raw_data[3];
    AK8963_CHECK(!handle->_read(handle, AK8963_ASAX, mag_raw_data, 3, TIMEOUT_MS_DEFAULT), AK8963_INIT_ERR_STR, NULL);

    handle->asa.x_axis = (float)(mag_raw_data[0] - 128) / 256.0f + 1.0f;
    handle->asa.y_axis = (float)(mag_raw_data[1] - 128) / 256.0f + 1.0f;
    handle->asa.z_axis = (float)(mag_raw_data[2] - 128) / 256.0f + 1.0f;

    /* Power down AK8963 magnetic sensor */
    buffer = 0x00;
    AK8963_CHECK(!handle->_write(handle, AK8963_CNTL, &buffer, 1, TIMEOUT_MS_DEFAULT), AK8963_INIT_ERR_STR, NULL);
    HAL_Delay(10);

    /* Configure magnetic operation mode and range */
    buffer = 0;
    buffer = (config->opr_mode) & 0x0F;
    buffer |= (config->mfs_sel << 4) & 0x10;
    AK8963_CHECK(!handle->_write(handle, AK8963_CNTL, &buffer, 1, TIMEOUT_MS_DEFAULT), AK8963_INIT_ERR_STR, NULL);
    HAL_Delay(10);

    /* Update magnetometer scaling factor */
    switch (config->mfs_sel)
    {
    case AK8963_MFS_14BIT:
        handle->mag_scaling_factor = 10.0f * 4912.0f / 8190.0f;
        break;

    case AK8963_MFS_16BIT:
        handle->mag_scaling_factor = 10.0f * 4912.0f / 32760.0f;
        break;

    default:
        break;
    }

    /* Update handle structure */
    handle->opr_mode = config->opr_mode;
    handle->mfs_sel = config->mfs_sel;
    handle->hard_iron_bias.x_axis = 0;
    handle->hard_iron_bias.y_axis = 0;
    handle->hard_iron_bias.z_axis = 0;
    handle->if_protocol = config->if_protocol;
    handle->lock = mutex_create();

    return handle;
}

stm_err_t ak8963_get_mag_raw(ak8963_handle_t handle, ak8963_raw_data_t *raw_data)
{
    mutex_lock(handle->lock);

    int ret;
    uint8_t mag_raw_data[7];

    ret = handle->_read(handle, AK8963_XOUT_L, mag_raw_data, 7, TIMEOUT_MS_DEFAULT);
    if (ret) {
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    if ((mag_raw_data[6] & 0x08)) {
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    raw_data->x_axis = (int16_t)((int16_t)(mag_raw_data[1] << 8) | mag_raw_data[0]);
    raw_data->y_axis = (int16_t)((int16_t)(mag_raw_data[3] << 8) | mag_raw_data[2]);
    raw_data->z_axis = (int16_t)((int16_t)(mag_raw_data[5] << 8) | mag_raw_data[4]);

    mutex_unlock(handle->lock);
    return 0;
}

stm_err_t ak8963_get_mag_cali(ak8963_handle_t handle, ak8963_cali_data_t *cali_data)
{
    mutex_lock(handle->lock);

    int ret;
    uint8_t mag_raw_data[7];

    ret = handle->_read(handle, AK8963_XOUT_L, mag_raw_data, 7, TIMEOUT_MS_DEFAULT);
    if (ret) {
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    if ((mag_raw_data[6] & 0x08)) {
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    int16_t temp_x = (int16_t)((int16_t)(mag_raw_data[1] << 8) | mag_raw_data[0]);
    int16_t temp_y = (int16_t)((int16_t)(mag_raw_data[3] << 8) | mag_raw_data[2]);
    int16_t temp_z = (int16_t)((int16_t)(mag_raw_data[5] << 8) | mag_raw_data[4]);

    cali_data->x_axis = ((float)temp_x * handle->asa.x_axis - handle->hard_iron_bias.x_axis / handle->mag_scaling_factor) * handle->soft_iron_corr.x_axis;
    cali_data->y_axis = ((float)temp_y * handle->asa.y_axis - handle->hard_iron_bias.y_axis / handle->mag_scaling_factor) * handle->soft_iron_corr.y_axis;
    cali_data->z_axis = ((float)temp_z * handle->asa.z_axis - handle->hard_iron_bias.z_axis / handle->mag_scaling_factor) * handle->soft_iron_corr.z_axis;

    mutex_unlock(handle->lock);
    return 0;
}

stm_err_t ak8963_get_mag_scale(ak8963_handle_t handle, ak8963_scale_data_t *scale_data)
{
    mutex_lock(handle->lock);

    int ret;
    uint8_t mag_raw_data[7];

    ret = handle->_read(handle, AK8963_XOUT_L, mag_raw_data, 7, TIMEOUT_MS_DEFAULT);
    if (ret) {
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    if ((mag_raw_data[6] & 0x08)) {
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    int16_t temp_x = (int16_t)((int16_t)(mag_raw_data[1] << 8) | mag_raw_data[0]);
    int16_t temp_y = (int16_t)((int16_t)(mag_raw_data[3] << 8) | mag_raw_data[2]);
    int16_t temp_z = (int16_t)((int16_t)(mag_raw_data[5] << 8) | mag_raw_data[4]);

    scale_data->x_axis = ((float)temp_x * handle->asa.x_axis * handle->mag_scaling_factor - handle->hard_iron_bias.x_axis) * handle->soft_iron_corr.x_axis;
    scale_data->y_axis = ((float)temp_y * handle->asa.y_axis * handle->mag_scaling_factor - handle->hard_iron_bias.y_axis) * handle->soft_iron_corr.y_axis;
    scale_data->z_axis = ((float)temp_z * handle->asa.z_axis * handle->mag_scaling_factor - handle->hard_iron_bias.z_axis) * handle->soft_iron_corr.z_axis;

    mutex_unlock(handle->lock);
    return 0;
}

void ak8963_set_hard_iron_bias(ak8963_handle_t handle, ak8963_hard_iron_bias_t hard_iron_bias)
{
    mutex_lock(handle->lock);
    handle->hard_iron_bias.x_axis = hard_iron_bias.x_axis;
    handle->hard_iron_bias.y_axis = hard_iron_bias.y_axis;
    handle->hard_iron_bias.z_axis = hard_iron_bias.z_axis;
    mutex_unlock(handle->lock);
}

void ak8963_get_hard_iron_bias(ak8963_handle_t handle, ak8963_hard_iron_bias_t *hard_iron_bias)
{
    mutex_lock(handle->lock);
    hard_iron_bias->x_axis = handle->hard_iron_bias.x_axis;
    hard_iron_bias->y_axis = handle->hard_iron_bias.y_axis;
    hard_iron_bias->z_axis = handle->hard_iron_bias.z_axis;
    mutex_unlock(handle->lock);
}

void ak8963_set_soft_iron_corr(ak8963_handle_t handle, ak8963_soft_iron_corr_t soft_iron_corr)
{
    mutex_lock(handle->lock);
    handle->soft_iron_corr.x_axis = soft_iron_corr.x_axis;
    handle->soft_iron_corr.y_axis = soft_iron_corr.y_axis;
    handle->soft_iron_corr.z_axis = soft_iron_corr.z_axis;
    mutex_unlock(handle->lock);
}

void ak8963_get_soft_iron_corr(ak8963_handle_t handle, ak8963_soft_iron_corr_t *soft_iron_corr)
{
    mutex_lock(handle->lock);
    soft_iron_corr->x_axis = handle->soft_iron_corr.x_axis;
    soft_iron_corr->y_axis = handle->soft_iron_corr.y_axis;
    soft_iron_corr->z_axis = handle->soft_iron_corr.z_axis;
    mutex_unlock(handle->lock);
}

void ak8963_get_mag_sens_adj(ak8963_handle_t handle, ak8963_sens_adj_t *asa)
{
    mutex_lock(handle->lock);
    asa->x_axis = handle->asa.x_axis;
    asa->y_axis = handle->asa.y_axis;
    asa->z_axis = handle->asa.z_axis;
    mutex_unlock(handle->lock);
}

void ak8963_auto_calib(ak8963_handle_t handle)
{
    uint32_t i;
    int16_t mag_max[3] = { -32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767};
    uint16_t sample_count;

    if (handle->mfs_sel == AK8963_MFS_14BIT)
        sample_count = 50;
    if (handle->mfs_sel == AK8963_MFS_16BIT)
        sample_count = 1500;

    for (i = 0; i < sample_count + 100; i++)            /*!< Dismiss 100 first value */
    {
        if (i > 100 && i <= (sample_count + 100))
        {
            ak8963_raw_data_t mag_raw;
            ak8963_get_mag_raw(handle, &mag_raw);

            if (mag_raw.x_axis > mag_max[0])
                mag_max[0] = mag_raw.x_axis;
            if (mag_raw.x_axis < mag_min[0])
                mag_min[0] = mag_raw.x_axis;

            if (mag_raw.y_axis > mag_max[1])
                mag_max[1] = mag_raw.y_axis;
            if (mag_raw.y_axis < mag_min[1])
                mag_min[1] = mag_raw.y_axis;

            if (mag_raw.z_axis > mag_max[2])
                mag_max[2] = mag_raw.z_axis;
            if (mag_raw.z_axis < mag_min[2])
                mag_min[2] = mag_raw.z_axis;
        }
        if (handle->mfs_sel == AK8963_MFS_14BIT)
            HAL_Delay(150);
        if (handle->mfs_sel == AK8963_MFS_16BIT)
            HAL_Delay(15);
    }

    handle->hard_iron_bias.x_axis = (float)((mag_max[0] + mag_min[0]) / 2) * handle->mag_scaling_factor * handle->asa.x_axis;
    handle->hard_iron_bias.y_axis = (float)((mag_max[1] + mag_min[1]) / 2) * handle->mag_scaling_factor * handle->asa.y_axis;
    handle->hard_iron_bias.z_axis = (float)((mag_max[2] + mag_min[2]) / 2) * handle->mag_scaling_factor * handle->asa.z_axis;

    float scale_temp[3];

    scale_temp[0] = (mag_max[0] - mag_min[0]) / 2;
    scale_temp[1] = (mag_max[1] - mag_min[1]) / 2;
    scale_temp[2] = (mag_max[2] - mag_min[2]) / 2;

    float mag_scale_avg = (scale_temp[0] + scale_temp[1] + scale_temp[2]) / 3.0f;

    handle->soft_iron_corr.x_axis = mag_scale_avg / ((float)scale_temp[0]);
    handle->soft_iron_corr.y_axis = mag_scale_avg / ((float)scale_temp[1]);
    handle->soft_iron_corr.z_axis = mag_scale_avg / ((float)scale_temp[2]);
}