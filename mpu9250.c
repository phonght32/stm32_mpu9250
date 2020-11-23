#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "stm_log.h"
#include "include/mpu9250.h"

#define MPU9250_SELF_TEST_X_GYRO        0x00        /*!< Gyroscope self-test registers */
#define MPU9250_SELF_TEST_Y_GYRO        0x01
#define MPU9250_SELF_TEST_Z_GYRO        0x02
#define MPU9250_SELF_TEST_X_ACCEL       0x0D        /*!< Accelerometer self-test registers */
#define MPU9250_SELF_TEST_Y_ACCEL       0x0E
#define MPU9250_SELF_TEST_Z_ACCEL       0x0F
#define MPU9250_XG_OFFSET_H             0x13        /*!< Gyroscope offset registers */
#define MPU9250_XG_OFFSET_L             0x14
#define MPU9250_YG_OFFSET_H             0x14
#define MPU9250_YG_OFFSET_L             0x16
#define MPU9250_ZG_OFFSET_H             0x17
#define MPU9250_ZG_OFFSET_L             0x18
#define MPU9250_SMPLRT_DIV              0x19        /*!< Sample rate divider */
#define MPU9250_CONFIG                  0x1A        /*!< Configuration */
#define MPU9250_GYRO_CONFIG             0x1B        /*!< Gyroscope configuration */
#define MPU9250_ACCEL_CONFIG            0x1C        /*!< Accelerometer configuration */
#define MPU9250_ACCEL_CONFIG2           0x1D        /*!< Accelerometer configuration 2 */
#define MPU9250_LP_ACCEL_ODR            0x1E        /*!< Low power accelerometer ODR control */
#define MPU9250_WOM_THR                 0x1F        /*!< Wake-on motion threshold */
#define MPU9250_FIFO_EN                 0x23        /*!< FIFO enable */
#define MPU9250_I2C_MST_CTRL            0x24        /*!< I2C master control */
#define MPU9250_I2C_SLV0_ADDR           0x25        /*!< I2C slave 0 control */
#define MPU9250_I2C_SLV0_REG            0x26
#define MPU9250_I2C_SLV0_CTRL           0x27
#define MPU9250_I2C_SLV1_ADDR           0x28        /*!< I2C slave 1 control */
#define MPU9250_I2C_SLV1_REG            0x29
#define MPU9250_I2C_SLV1_CTRL           0x2A
#define MPU9250_I2C_SLV2_ADDR           0x2B        /*!< I2C slave 2 control */
#define MPU9250_I2C_SLV2_REG            0x2C
#define MPU9250_I2C_SLV2_CTRL           0x2D
#define MPU9250_I2C_SLV3_ADDR           0x2E        /*!< I2C slave 3 control */
#define MPU9250_I2C_SLV3_REG            0x2F
#define MPU9250_I2C_SLV3_CTRL           0x30
#define MPU9250_I2C_SLV4_ADDR           0x31        /*!< I2C slave 4 control */
#define MPU9250_I2C_SLV4_REG            0x32
#define MPU9250_I2C_SLV4_DO             0x33
#define MPU9250_I2C_SLV4_CTRL           0x34
#define MPU9250_I2C_SLV4_DI             0x35
#define MPU9250_I2C_MST_STATUS          0x36        /*!< I2C master status */
#define MPU9250_INT_PIN_CFG             0x37        /*!< Interrupt pin/bypass enable configuration */
#define MPU9250_INT_ENABLE              0x38        /*!< Interrupt enable */
#define MPU9250_INT_STATUS              0x3A        /*!< Interrupt status */
#define MPU9250_ACCEL_XOUT_H            0x3B        /*!< Accelerometer measurements */
#define MPU9250_ACCEL_XOUT_L            0x3C
#define MPU9250_ACCEL_YOUT_H            0x3D
#define MPU9250_ACCEL_YOUT_L            0x3E
#define MPU9250_ACCEL_ZOUT_H            0x3F
#define MPU9250_ACCEL_ZOUT_L            0x40
#define MPU9250_TEMP_OUT_H              0x41        /*!< Temperature measurements */
#define MPU9250_TEMP_OUT_L              0x42
#define MPU9250_GYRO_XOUT_H             0x43        /*!< Gyroscope measurements */
#define MPU9250_GYRO_XOUT_L             0x44
#define MPU9250_GYRO_YOUT_H             0x45
#define MPU9250_GYRO_YOUT_L             0x46
#define MPU9250_GYRO_ZOUT_H             0x47
#define MPU9250_GYRO_ZOUT_L             0x48
#define MPU9250_EXT_SENS_DATA_00        0x49        /*!< External sensor data */
#define MPU9250_EXT_SENS_DATA_01        0x4A
#define MPU9250_EXT_SENS_DATA_02        0x4B
#define MPU9250_EXT_SENS_DATA_03        0x4C
#define MPU9250_EXT_SENS_DATA_04        0x4D
#define MPU9250_EXT_SENS_DATA_05        0x4E
#define MPU9250_EXT_SENS_DATA_06        0x4F
#define MPU9250_EXT_SENS_DATA_07        0x50
#define MPU9250_EXT_SENS_DATA_08        0x51
#define MPU9250_EXT_SENS_DATA_09        0x52
#define MPU9250_EXT_SENS_DATA_10        0x53
#define MPU9250_EXT_SENS_DATA_11        0x54
#define MPU9250_EXT_SENS_DATA_12        0x55
#define MPU9250_EXT_SENS_DATA_13        0x56
#define MPU9250_EXT_SENS_DATA_14        0x57
#define MPU9250_EXT_SENS_DATA_15        0x58
#define MPU9250_EXT_SENS_DATA_16        0x59
#define MPU9250_EXT_SENS_DATA_17        0x5A
#define MPU9250_EXT_SENS_DATA_18        0x5B
#define MPU9250_EXT_SENS_DATA_19        0x5C
#define MPU9250_EXT_SENS_DATA_20        0x5D
#define MPU9250_EXT_SENS_DATA_21        0x5E
#define MPU9250_EXT_SENS_DATA_22        0x5F
#define MPU9250_EXT_SENS_DATA_23        0x60
#define MPU9250_I2C_SLV0_DO             0x63        /*!< I2C slave 0 data out */
#define MPU9250_I2C_SLV1_DO             0x64        /*!< I2C slave 1 data out */
#define MPU9250_I2C_SLV2_DO             0x65        /*!< I2C slave 2 data out */
#define MPU9250_I2C_SLV3_DO             0x66        /*!< I2C slave 3 data out */
#define MPU9250_I2C_MST_DELAY_CTRL      0x67        /*!< I2C master delay control */
#define MPU9250_SIGNAL_PATH_RESET       0x68        /*!< Signal path reset */
#define MPU9250_MOT_DETECT_CTRL         0x69        /*!< Acelerometer interrupt control */
#define MPU9250_USER_CTRL               0x6A        /*!< User control */
#define MPU9250_PWR_MGMT_1              0x6B        /*!< Power management 1 */
#define MPU9250_PWR_MGMT_2              0x6C        /*!< Power management 2 */
#define MPU9250_FIFO_COUNTH             0x72        /*!< FIFO counter registers */
#define MPU9250_FIFO_COUNTL             0x73
#define MPU9250_FIFP_R_W                0x74        /*!< FIFO read write */
#define MPU9250_WHO_AM_I                0x75        /*!< Who am I */
#define MPU9250_XA_OFFSET_H             0x77        /*!< Accelerometer offset registers */
#define MPU9250_XA_OFFSET_L             0x78
#define MPU9250_YA_OFFSET_H             0x7A
#define MPU9250_YA_OFFSET_L             0x7B
#define MPU9250_ZA_OFFSET_H             0x7D
#define MPU9250_ZA_OFFSET_L             0x7E
#define MPU9250_ADDR                    (0x68<<1)   /*!< MPU9250 Address */

#define TIMEOUT_MS_DEFAULT              100         /*!< Default MPU9250 I2C communiation timeout */
#define BUFFER_CALIB_DEFAULT            1000        /*!< Default the number of sample data when calibrate */

#define MPU9250_INIT_ERR_STR            "mpu9250 init error"
#define MPU9250_MALLOC_ERR_STR          "mpu9250 malloc error"
#define MPU9250_TRANS_ERR_STR           "mpu9250 write registers error"
#define MPU9250_REC_ERR_STR             "mpu9250 read registers error"
#define MPU9250_GET_DATA_ERR_STR        "mpu9250 get data error"
#define MPU9250_SET_BIAS_ERR_STR        "mu9250 set bias error"
#define MPU9250_GET_BIAS_ERR_STR        "mu9250 get bias error"

#define mutex_lock(x)                   while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x)                 xSemaphoreGive(x)
#define mutex_create()                  xSemaphoreCreateMutex()
#define mutex_destroy(x)                vQueueDelete(x)

static const char* MPU9250_TAG = "MPU9250";
#define MPU9250_CHECK(a, str, action)  if(!(a)) {                                           \
        STM_LOGE(MPU9250_TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);       \
        action;                                                                             \
        }

typedef stm_err_t (*init_func)(mpu9250_hw_info_t hw_info);
typedef stm_err_t (*read_func)(mpu9250_hw_info_t hw_info, uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms);
typedef stm_err_t (*write_func)(mpu9250_hw_info_t hw_info, uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms);

typedef struct mpu9250 {
    mpu9250_clksel_t        clksel;                 /*!< MPU9250 clock source */
    mpu9250_dlpf_cfg_t      dlpf_cfg;               /*!< MPU9250 digital low pass filter (DLPF) */
    mpu9250_sleep_mode_t    sleep_mode;             /*!< MPU9250 sleep mode */
    mpu9250_afs_sel_t       afs_sel;                /*!< MPU9250 accelerometer full scale range */
    mpu9250_fs_sel_t        fs_sel;                 /*!< MPU9250 gyroscope full scale range */
    mpu9250_accel_bias_t    accel_bias;             /*!< MPU9250 acclerometer bias */
    mpu9250_gyro_bias_t     gyro_bias;              /*!< MPU9250 gyroscope bias */
    mpu9250_comm_mode_t     comm_mode;              /*!< MPU9250 interface protocol */
    float                   accel_scaling_factor;   /*!< MPU9250 accelerometer scaling factor */
    float                   gyro_scaling_factor;    /*!< MPU9250 gyroscope scaling factor */
    SemaphoreHandle_t       lock;                   /*!< MPU9250 mutex */
    mpu9250_hw_info_t       hw_info;                /*!< MPU9250 hardware information */
    read_func               _read;                  /*!< MPU9250 read function */
    write_func              _write;                 /*!< MPU9250 write function */
} mpu9250_t;

static stm_err_t _init_i2c(mpu9250_hw_info_t hw_info)
{
    i2c_cfg_t i2c_cfg;
    i2c_cfg.i2c_num = hw_info.i2c_num;
    i2c_cfg.i2c_pins_pack = hw_info.i2c_pins_pack;
    i2c_cfg.clk_speed = hw_info.i2c_speed;
    MPU9250_CHECK(!i2c_config(&i2c_cfg), MPU9250_INIT_ERR_STR, return STM_FAIL);

    return STM_OK;
}

static stm_err_t _init_spi(mpu9250_hw_info_t hw_info)
{
    return STM_OK;
}

static stm_err_t _i2c_write_func(mpu9250_hw_info_t hw_info, uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    uint8_t buf_send[len + 1];
    buf_send[0] = reg_addr;
    for (uint8_t i = 0; i < len; i++)
    {
        buf_send[i + 1] = buf[i];
    }

    MPU9250_CHECK(!i2c_master_write_bytes(hw_info.i2c_num, MPU9250_ADDR, buf_send, len + 1, timeout_ms), MPU9250_TRANS_ERR_STR, return STM_FAIL);
    return STM_OK;
}

static stm_err_t _spi_write_func(mpu9250_hw_info_t hw_info, uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    return STM_OK;
}

static stm_err_t _i2c_read_func(mpu9250_hw_info_t hw_info, uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    uint8_t buffer[1];
    buffer[0] = reg_addr;
    MPU9250_CHECK(!i2c_master_write_bytes(hw_info.i2c_num, MPU9250_ADDR, buffer, 1, timeout_ms), MPU9250_REC_ERR_STR, return STM_FAIL);
    MPU9250_CHECK(!i2c_master_read_bytes(hw_info.i2c_num, MPU9250_ADDR, buf, len, timeout_ms), MPU9250_REC_ERR_STR, return STM_FAIL);

    return STM_OK;
}

static stm_err_t _spi_read_func(mpu9250_hw_info_t hw_info, uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    return STM_OK;
}

static init_func _get_init_func(mpu9250_comm_mode_t comm_mode)
{
    if (comm_mode == MPU9250_COMM_MODE_I2C) {
        return _init_i2c;
    } else {
        return _init_spi;
    }

    return NULL;
}

static read_func _get_read_func(mpu9250_comm_mode_t comm_mode)
{
    if (comm_mode == MPU9250_COMM_MODE_I2C) {
        return _i2c_read_func;
    } else {
        return _spi_read_func;
    }

    return NULL;
}

static write_func _get_write_func(mpu9250_comm_mode_t comm_mode)
{
    if (comm_mode == MPU9250_COMM_MODE_I2C) {
        return _i2c_write_func;
    } else {
        return _spi_write_func;
    }

    return NULL;
}

static void _mpu9250_cleanup(mpu9250_handle_t handle)
{
    free(handle);
}

mpu9250_handle_t mpu9250_init(mpu9250_cfg_t *config)
{
    /* Check if init structure is empty */
    MPU9250_CHECK(config, MPU9250_INIT_ERR_STR, return NULL);
    MPU9250_CHECK(config->clksel < MPU9250_CLKSEL_MAX, MPU9250_INIT_ERR_STR, return NULL);
    MPU9250_CHECK(config->dlpf_cfg < MPU9250_DLPF_CFG_MAX, MPU9250_INIT_ERR_STR, return NULL);
    MPU9250_CHECK(config->sleep_mode < MPU9250_SLEEP_MODE_MAX, MPU9250_INIT_ERR_STR, return NULL);
    MPU9250_CHECK(config->fs_sel < MPU9250_FS_SEL_MAX, MPU9250_INIT_ERR_STR, return NULL);
    MPU9250_CHECK(config->afs_sel < MPU9250_AFS_SEL_MAX, MPU9250_INIT_ERR_STR, return NULL);
    MPU9250_CHECK(config->comm_mode < MPU9250_COMM_MODE_MAX, MPU9250_INIT_ERR_STR, return NULL);

    /* Allocate memory for handle structure */
    mpu9250_handle_t handle;
    handle = calloc(1, sizeof(mpu9250_t));
    MPU9250_CHECK(handle, MPU9250_INIT_ERR_STR, return NULL);

    /* Init hardware */
    if (!config->hw_info.is_init) {
        init_func _init = _get_init_func(config->comm_mode);
        MPU9250_CHECK(!_init(config->hw_info), MPU9250_INIT_ERR_STR, {_mpu9250_cleanup(handle); return NULL;});
    }

    /* Get write function */
    write_func _write = _get_write_func(config->comm_mode);

    /* Reset mpu9250 */
    uint8_t buffer = 0;
    buffer = 0x80;
    MPU9250_CHECK(!_write(config->hw_info, MPU9250_PWR_MGMT_1, &buffer, 1, TIMEOUT_MS_DEFAULT), MPU9250_INIT_ERR_STR, {_mpu9250_cleanup(handle); return NULL;});
    vTaskDelay(100 / portTICK_PERIOD_MS);

    /* Configure clock source and sleep mode */
    buffer = config->clksel & 0x07;
    buffer |= (config->sleep_mode << 6) & 0x40;
    MPU9250_CHECK(!_write(config->hw_info, MPU9250_PWR_MGMT_1, &buffer, 1, TIMEOUT_MS_DEFAULT), MPU9250_INIT_ERR_STR, {_mpu9250_cleanup(handle); return NULL;});
    vTaskDelay(100 / portTICK_PERIOD_MS);

    /* Configure digital low pass filter */
    buffer = 0;
    buffer = config->dlpf_cfg & 0x07;
    MPU9250_CHECK(!_write(config->hw_info, MPU9250_CONFIG, &buffer, 1, TIMEOUT_MS_DEFAULT), MPU9250_INIT_ERR_STR, {_mpu9250_cleanup(handle); return NULL;});

    /* Configure gyroscope range */
    buffer = 0;
    buffer = (config->fs_sel << 3) & 0x18;
    MPU9250_CHECK(!_write(config->hw_info, MPU9250_GYRO_CONFIG, &buffer, 1, TIMEOUT_MS_DEFAULT), MPU9250_INIT_ERR_STR, {_mpu9250_cleanup(handle); return NULL;});

    /* Configure accelerometer range */
    buffer = 0;
    buffer = (config->afs_sel << 3) & 0x18;
    MPU9250_CHECK(!_write(config->hw_info, MPU9250_ACCEL_CONFIG, &buffer, 1, TIMEOUT_MS_DEFAULT), MPU9250_INIT_ERR_STR, {_mpu9250_cleanup(handle); return NULL;});

    /* Configure sample rate divider */
    buffer = 0;
    buffer = 0x04;
    MPU9250_CHECK(!_write(config->hw_info, MPU9250_SMPLRT_DIV, &buffer, 1, TIMEOUT_MS_DEFAULT), MPU9250_INIT_ERR_STR, {_mpu9250_cleanup(handle); return NULL;});

    /* Configure interrupt and enable bypass.
     * Set Interrupt pin active high, push-pull, Clear and read of INT_STATUS,
     * enable I2C_BYPASS_EN in INT_PIN_CFG register so additional chips can
     * join the I2C bus and can be controlled by master.
     */
    buffer = 0x22;
    MPU9250_CHECK(!_write(config->hw_info, MPU9250_INT_PIN_CFG, &buffer, 1, TIMEOUT_MS_DEFAULT), MPU9250_INIT_ERR_STR, {_mpu9250_cleanup(handle); return NULL;});
    buffer = 0x01;
    MPU9250_CHECK(!_write(config->hw_info, MPU9250_INT_ENABLE, &buffer, 1, TIMEOUT_MS_DEFAULT), MPU9250_INIT_ERR_STR, {_mpu9250_cleanup(handle); return NULL;});

    /* Update accelerometer scaling factor */
    switch (config->afs_sel)
    {
    case MPU9250_AFS_SEL_2G:
        handle->accel_scaling_factor = (2.0f / 32768.0f);
        break;

    case MPU9250_AFS_SEL_4G:
        handle->accel_scaling_factor = (4.0f / 32768.0f);
        break;

    case MPU9250_AFS_SEL_8G:
        handle->accel_scaling_factor = (8.0f / 32768.0f);
        break;

    case MPU9250_AFS_SEL_16G:
        handle->accel_scaling_factor = (16.0f / 32768.0f);
        break;

    default:
        break;
    }

    /* Update gyroscope scaling factor */
    switch (config->fs_sel)
    {
    case MPU9250_FS_SEL_250:
        handle->gyro_scaling_factor = 250.0f / 32768.0f;
        break;

    case MPU9250_FS_SEL_500:
        handle->gyro_scaling_factor = 500.0f / 32768.0f;
        break;

    case MPU9250_FS_SEL_1000:
        handle->gyro_scaling_factor = 1000.0f / 32768.0f;
        break;

    case MPU9250_FS_SEL_2000:
        handle->gyro_scaling_factor = 2000.0f / 32768.0f;
        break;

    default:
        break;
    }

    /* Update handle structure */
    handle->accel_bias.x_axis = config->accel_bias.x_axis;
    handle->accel_bias.y_axis = config->accel_bias.y_axis;
    handle->accel_bias.z_axis = config->accel_bias.z_axis;
    handle->gyro_bias.x_axis = config->gyro_bias.x_axis;
    handle->gyro_bias.y_axis = config->gyro_bias.y_axis;
    handle->gyro_bias.z_axis = config->gyro_bias.z_axis;
    handle->afs_sel = config->afs_sel;
    handle->clksel = config->clksel;
    handle->dlpf_cfg = config->dlpf_cfg;
    handle->fs_sel = config->fs_sel;
    handle->sleep_mode = config->sleep_mode;
    handle->comm_mode = config->comm_mode;
    handle->lock = mutex_create();
    handle->hw_info = config->hw_info;
    handle->_read = _get_read_func(config->comm_mode);
    handle->_write = _get_write_func(config->comm_mode);

    return handle;
}

stm_err_t mpu9250_get_accel_raw(mpu9250_handle_t handle, mpu9250_raw_data_t *raw_data)
{
    MPU9250_CHECK(handle, MPU9250_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);
    MPU9250_CHECK(raw_data, MPU9250_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);

    int ret;
    uint8_t accel_raw_data[6];

    ret = handle->_read(handle->hw_info, MPU9250_ACCEL_XOUT_H, accel_raw_data, 6, TIMEOUT_MS_DEFAULT);
    if (ret) {
        STM_LOGE(MPU9250_TAG, MPU9250_GET_DATA_ERR_STR);
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    raw_data->x_axis = (int16_t)((accel_raw_data[0] << 8) + accel_raw_data[1]);
    raw_data->y_axis = (int16_t)((accel_raw_data[2] << 8) + accel_raw_data[3]);
    raw_data->z_axis = (int16_t)((accel_raw_data[4] << 8) + accel_raw_data[5]);

    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t mpu9250_get_accel_cali(mpu9250_handle_t handle, mpu9250_cali_data_t *cali_data)
{
    MPU9250_CHECK(handle, MPU9250_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);
    MPU9250_CHECK(cali_data, MPU9250_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);

    int ret;
    uint8_t accel_raw_data[6];

    ret = handle->_read(handle->hw_info, MPU9250_ACCEL_XOUT_H, accel_raw_data, 6, TIMEOUT_MS_DEFAULT);
    if (ret) {
        STM_LOGE(MPU9250_TAG, MPU9250_GET_DATA_ERR_STR);
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    cali_data->x_axis = (int16_t)((accel_raw_data[0] << 8) + accel_raw_data[1]) - handle->accel_bias.x_axis;
    cali_data->y_axis = (int16_t)((accel_raw_data[2] << 8) + accel_raw_data[3]) - handle->accel_bias.y_axis;
    cali_data->z_axis = (int16_t)((accel_raw_data[4] << 8) + accel_raw_data[5]) - handle->accel_bias.z_axis;

    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t mpu9250_get_accel_scale(mpu9250_handle_t handle, mpu9250_scale_data_t *scale_data)
{
    MPU9250_CHECK(handle, MPU9250_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);
    MPU9250_CHECK(scale_data, MPU9250_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);

    int ret;
    uint8_t accel_raw_data[6];

    ret = handle->_read(handle->hw_info, MPU9250_ACCEL_XOUT_H, accel_raw_data, 6, TIMEOUT_MS_DEFAULT);
    if (ret) {
        STM_LOGE(MPU9250_TAG, MPU9250_GET_DATA_ERR_STR);
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    scale_data->x_axis = (float)((int16_t)((accel_raw_data[0] << 8) + accel_raw_data[1]) - handle->accel_bias.x_axis) * handle->accel_scaling_factor;
    scale_data->y_axis = (float)((int16_t)((accel_raw_data[2] << 8) + accel_raw_data[3]) - handle->accel_bias.y_axis) * handle->accel_scaling_factor;
    scale_data->z_axis = (float)((int16_t)((accel_raw_data[4] << 8) + accel_raw_data[5]) - handle->accel_bias.z_axis) * handle->accel_scaling_factor;

    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t mpu9250_get_gyro_raw(mpu9250_handle_t handle, mpu9250_raw_data_t *raw_data)
{
    MPU9250_CHECK(handle, MPU9250_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);
    MPU9250_CHECK(raw_data, MPU9250_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);

    int ret;
    uint8_t gyro_raw_data[6];

    ret = handle->_read(handle->hw_info, MPU9250_GYRO_XOUT_H, gyro_raw_data, 6, TIMEOUT_MS_DEFAULT);
    if (ret) {
        STM_LOGE(MPU9250_TAG, MPU9250_GET_DATA_ERR_STR);
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    raw_data->x_axis = (int16_t)((gyro_raw_data[0] << 8) + gyro_raw_data[1]);
    raw_data->y_axis = (int16_t)((gyro_raw_data[2] << 8) + gyro_raw_data[3]);
    raw_data->z_axis = (int16_t)((gyro_raw_data[4] << 8) + gyro_raw_data[5]);

    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t mpu9250_get_gyro_cali(mpu9250_handle_t handle, mpu9250_cali_data_t *cali_data)
{
    MPU9250_CHECK(handle, MPU9250_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);
    MPU9250_CHECK(cali_data, MPU9250_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);

    int ret;
    uint8_t gyro_raw_data[6];

    ret = handle->_read(handle->hw_info, MPU9250_GYRO_XOUT_H, gyro_raw_data, 6, TIMEOUT_MS_DEFAULT);
    if (ret) {
        STM_LOGE(MPU9250_TAG, MPU9250_GET_DATA_ERR_STR);
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    cali_data->x_axis = (int16_t)((gyro_raw_data[0] << 8) + gyro_raw_data[1]) - handle->gyro_bias.x_axis;
    cali_data->y_axis = (int16_t)((gyro_raw_data[2] << 8) + gyro_raw_data[3]) - handle->gyro_bias.y_axis;
    cali_data->z_axis = (int16_t)((gyro_raw_data[4] << 8) + gyro_raw_data[5]) - handle->gyro_bias.z_axis;

    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t mpu9250_get_gyro_scale(mpu9250_handle_t handle, mpu9250_scale_data_t *scale_data)
{
    MPU9250_CHECK(handle, MPU9250_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);
    MPU9250_CHECK(scale_data, MPU9250_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);

    int ret;
    uint8_t gyro_raw_data[6];
    ret = handle->_read(handle->hw_info, MPU9250_GYRO_XOUT_H, gyro_raw_data, 6, TIMEOUT_MS_DEFAULT);
    if (ret) {
        STM_LOGE(MPU9250_TAG, MPU9250_GET_DATA_ERR_STR);
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    scale_data->x_axis = (float)((int16_t)((gyro_raw_data[0] << 8) + gyro_raw_data[1]) - handle->gyro_bias.x_axis) * handle->gyro_scaling_factor;
    scale_data->y_axis = (float)((int16_t)((gyro_raw_data[2] << 8) + gyro_raw_data[3]) - handle->gyro_bias.y_axis) * handle->gyro_scaling_factor;
    scale_data->z_axis = (float)((int16_t)((gyro_raw_data[4] << 8) + gyro_raw_data[5]) - handle->gyro_bias.z_axis) * handle->gyro_scaling_factor;

    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t mpu9250_set_accel_bias(mpu9250_handle_t handle, mpu9250_accel_bias_t accel_bias)
{
    MPU9250_CHECK(handle, MPU9250_SET_BIAS_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);
    handle->accel_bias.x_axis = accel_bias.x_axis;
    handle->accel_bias.y_axis = accel_bias.y_axis;
    handle->accel_bias.z_axis = accel_bias.z_axis;
    mutex_unlock(handle->lock);

    return STM_OK;
}

stm_err_t mpu9250_set_gyro_bias(mpu9250_handle_t handle, mpu9250_gyro_bias_t gyro_bias)
{
    MPU9250_CHECK(handle, MPU9250_SET_BIAS_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);
    handle->gyro_bias.x_axis = gyro_bias.x_axis;
    handle->gyro_bias.y_axis = gyro_bias.y_axis;
    handle->gyro_bias.z_axis = gyro_bias.z_axis;
    mutex_unlock(handle->lock);

    return STM_OK;
}

stm_err_t mpu9250_get_accel_bias(mpu9250_handle_t handle, mpu9250_accel_bias_t *accel_bias)
{
    MPU9250_CHECK(handle, MPU9250_GET_BIAS_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);
    accel_bias->x_axis = handle->accel_bias.x_axis;
    accel_bias->y_axis = handle->accel_bias.y_axis;
    accel_bias->z_axis = handle->accel_bias.z_axis;
    mutex_unlock(handle->lock);

    return STM_OK;
}

stm_err_t mpu9250_get_gyro_bias(mpu9250_handle_t handle, mpu9250_gyro_bias_t *gyro_bias)
{
    MPU9250_CHECK(handle, MPU9250_GET_BIAS_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);
    gyro_bias->x_axis = handle->gyro_bias.x_axis;
    gyro_bias->y_axis = handle->gyro_bias.y_axis;
    gyro_bias->z_axis = handle->gyro_bias.z_axis;
    mutex_unlock(handle->lock);

    return STM_OK;
}

void mpu9250_auto_calib(mpu9250_handle_t handle)
{
    int buffersize = BUFFER_CALIB_DEFAULT;
    int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
    long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

    while (i < (buffersize + 101))                  /*!< Dismiss 100 first value */
    {
        mpu9250_raw_data_t accel_raw, gyro_raw;
        mpu9250_get_accel_raw(handle, &accel_raw);
        mpu9250_get_gyro_raw(handle, &gyro_raw);

        if (i > 100 && i <= (buffersize + 100))
        {
            buff_ax += accel_raw.x_axis;
            buff_ay += accel_raw.y_axis;
            buff_az += accel_raw.z_axis;
            buff_gx += gyro_raw.x_axis;
            buff_gy += gyro_raw.y_axis;
            buff_gz += gyro_raw.z_axis;
        }
        if (i == (buffersize + 100))
        {
            mean_ax = buff_ax / buffersize;
            mean_ay = buff_ay / buffersize;
            mean_az = buff_az / buffersize;
            mean_gx = buff_gx / buffersize;
            mean_gy = buff_gy / buffersize;
            mean_gz = buff_gz / buffersize;
        }
        i++;
    }

    handle->accel_bias.x_axis = mean_ax;
    handle->accel_bias.y_axis = mean_ay;
    handle->accel_bias.z_axis = mean_az - 1.0f / handle->accel_scaling_factor;
    handle->gyro_bias.x_axis = mean_gx;
    handle->gyro_bias.y_axis = mean_gy;
    handle->gyro_bias.z_axis = mean_gz;
}

void mpu9250_destroy(mpu9250_handle_t handle)
{
    _mpu9250_cleanup(handle);
}
