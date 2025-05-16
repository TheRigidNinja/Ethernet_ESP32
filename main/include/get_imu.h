#ifndef GET_IMU_H
#define GET_IMU_H

#include <stdint.h>
#include "esp_err.h"

// IMU data struct
typedef struct {
    float accel[3];
    float gyro[3];
} imu_data_t;

// Initialize IMU hardware (I2C/SPI + sensor)
esp_err_t imu_init(void);
// Read one snapshot of accelerometer + gyro data
esp_err_t get_imu_data(imu_data_t *out);

#endif // GET_IMU_H