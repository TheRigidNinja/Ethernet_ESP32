// get_imu.c — LSM9DS1 (BerryIMU v2) over I²C

#include "get_imu.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_NUM             I2C_NUM_0
#define SDA_IO              GPIO_NUM_13   // EVB UEXT pin 9
#define SCL_IO              GPIO_NUM_16   // EVB UEXT pin 10
#define I2C_FREQ_HZ         400000
#define IMU_ADDR            0x6A          // accel/gyro address on BerryIMU

// LSM9DS1 register map
#define WHO_AM_I            0x0F
#define CTRL_REG1_G         0x10
#define CTRL_REG6_XL        0x20
#define OUT_X_L_G           0x18
#define OUT_X_L_XL          0x28

// ODR and scale settings
#define GYRO_ON             0x60  // ODR=119Hz, FS=±245dps
#define ACCEL_ON            0x60  // ODR=119Hz, FS=±2g

#define ACC_SENS            16384.0f
#define GYR_SENS            131.0f

static const char *TAG = "IMU";

// helper: write one byte
static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    return i2c_master_write_to_device(
        I2C_NUM, IMU_ADDR, data, 2, pdMS_TO_TICKS(100)
    );
}

// helper: read multiple bytes (auto-inc)
static esp_err_t read_regs(uint8_t start, uint8_t *buf, size_t len) {
    start |= 0x80;
    return i2c_master_write_read_device(
        I2C_NUM, IMU_ADDR, &start, 1, buf, len, pdMS_TO_TICKS(100)
    );
}

esp_err_t imu_init(void) {
    // 1) I2C init
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_IO,
        .scl_io_num = SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    esp_err_t r = i2c_param_config(I2C_NUM, &cfg) ||
                  i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(r));
        return r;
    }

    // 2) sanity WHO_AM_I
    uint8_t id = 0;
    if (i2c_master_write_read_device(
            I2C_NUM, IMU_ADDR, (uint8_t[]){WHO_AM_I}, 1, &id, 1, pdMS_TO_TICKS(100)
        ) == ESP_OK) {
        ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", id);
    } else {
        ESP_LOGW(TAG, "WHO_AM_I read failed");
    }

    // 3) enable gyro & accel
    if ((r = write_reg(CTRL_REG1_G, GYRO_ON)) != ESP_OK) {
        ESP_LOGE(TAG, "Gyro on failed: %s", esp_err_to_name(r));
        return r;
    }
    if ((r = write_reg(CTRL_REG6_XL, ACCEL_ON)) != ESP_OK) {
        ESP_LOGE(TAG, "Accel on failed: %s", esp_err_to_name(r));
        return r;
    }

    return ESP_OK;
}

esp_err_t get_imu_data(imu_data_t *out) {
    if (!out) return ESP_ERR_INVALID_ARG;
    uint8_t buf[6];
    esp_err_t r;

    // read gyro
    if ((r = read_regs(OUT_X_L_G, buf, 6)) != ESP_OK) {
        ESP_LOGE(TAG, "Gyro read fail: %s", esp_err_to_name(r));
        return r;
    }
    for (int i = 0; i < 3; i++) {
        int16_t v = (int16_t)(buf[2*i] | (buf[2*i+1] << 8));
        out->gyro[i] = v / GYR_SENS;
    }

    // read accel
    if ((r = read_regs(OUT_X_L_XL, buf, 6)) != ESP_OK) {
        ESP_LOGE(TAG, "Accel read fail: %s", esp_err_to_name(r));
        return r;
    }
    for (int i = 0; i < 3; i++) {
        int16_t v = (int16_t)(buf[2*i] | (buf[2*i+1] << 8));
        out->accel[i] = v / ACC_SENS;
    }

    return ESP_OK;
}
