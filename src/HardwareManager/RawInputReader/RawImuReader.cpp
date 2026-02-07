#include "HardwareManager/RawInputReader/RawImuReader.hpp"

#define IMU_ADDR                0x68

#define REG_DEVICE_CONFIG       0x11
#define REG_PWR_MGMT0           0x4E
#define REG_GYRO_CONFIG0        0x4F
#define REG_ACCEL_CONFIG0       0x50
#define REG_WHO_AM_I            0x75
#define REG_ACCEL_DATA_X1       0x1F
#define REG_PWR_MGMT0           0x4E


#define BIT_SOFT_RESET          0x01
#define WHO_AM_I_EXPECTED       0x47
#define PWR_LN_ALL_ON           0x0F
#define PWR_MODE_SLEEP          0x00

#define IMU_GYRO_2000DPS_1KHZ   0x06
#define IMU_ACCEL_8G_1KHZ       0x26

int RawImuReader::init() {
    int err = 0;
    uint8_t chip_id = 0;

    if (!i2c_is_ready_dt(&imu_i2c))
    {
        return -ENODEV;
    }

    err = i2c_reg_read_byte_dt(&imu_i2c, REG_WHO_AM_I, &chip_id);
    if (err) {
        return err;
    }

    if (chip_id != WHO_AM_I_EXPECTED) {
        return -ENODEV;
    }

    err = i2c_reg_write_byte_dt(&imu_i2c, REG_DEVICE_CONFIG, BIT_SOFT_RESET);
    if (err) {
        return err;
    }
    k_msleep(2);

    err = i2c_reg_write_byte_dt(&imu_i2c, REG_PWR_MGMT0, PWR_LN_ALL_ON);
    if (err) {
        return err;
    }
    k_msleep(30);

    err = i2c_reg_write_byte_dt(&imu_i2c, REG_GYRO_CONFIG0, IMU_GYRO_2000DPS_1KHZ);
    if (err) {
        return err;
    }

    err = i2c_reg_write_byte_dt(&imu_i2c, REG_ACCEL_CONFIG0, IMU_ACCEL_8G_1KHZ);
    if (err) {
        return err;
    }

    return 0;
}

int RawImuReader::deinit() {
    int err = i2c_reg_read_byte_dt(&imu_i2c, REG_PWR_MGMT0, PWR_MODE_SLEEP);
    if (err)
    {
        return err;
    }
    return 0;
}

int RawImuReader::getRawData(IMUData & raw_imu) {
    uint8_t raw[12];

    int err = i2c_burst_read_dt(&imu_i2c, REG_ACCEL_DATA_X1, raw, 12);
    if (err) {
        return err;
    }

    raw_imu.accel[0] = (int16_t)((raw[0] << 8) | raw[1]);
    raw_imu.accel[1] = (int16_t)((raw[2] << 8) | raw[3]);
    raw_imu.accel[2] = (int16_t)((raw[4] << 8) | raw[5]);

    raw_imu.gyro[0] = (int16_t)((raw[6] << 8) | raw[7]);
    raw_imu.gyro[1] = (int16_t)((raw[8] << 8) | raw[9]);
    raw_imu.gyro[2] = (int16_t)((raw[10] << 8) | raw[11]);

    return 0;
}
