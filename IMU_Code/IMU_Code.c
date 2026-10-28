#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"

// Configuration: 1 for 9-DOF (Gyro + Accel + Mag), 0 for 6-DOF (Gyro + Accel)
#define USE_9DOF_MODE   1

#define ACC_SPI_PORT    spi0
#define ACC_SCK         2
#define ACC_MOSI        3
#define ACC_MISO        4
#define ACC_CS          5
#define ACC_INT         0

#define GYRO_SPI_PORT   spi1
#define GYRO_SCK        10
#define GYRO_MOSI       11
#define GYRO_MISO       12
#define GYRO_CS         13
#define GYRO_INT        14

#define MAG_I2C_PORT    i2c1
#define MAG_SDA         6
#define MAG_SCL         7
#define MAG_INT         8

#define MLX90393_I2C_ADDR_DEFAULT  0x0C
#define MLX90393_I2C_ADDR_ALT      0x18
static uint8_t mlx90393_addr = MLX90393_I2C_ADDR_DEFAULT;

#define IAM_REG_PWR_MGMT_1      0x6B
#define IAM_REG_PWR_MGMT_2      0x6C
#define IAM_REG_GYRO_CONFIG     0x1B
#define IAM_REG_GYRO_XOUT_H     0x43
#define IAM_REG_WHO_AM_I        0x75

#define ADXL_REG_DEVID          0x00
#define ADXL_REG_XDATA_H        0x15
#define ADXL_REG_OP_MODE        0x26
#define ADXL_REG_DIG_EN         0x27

#define MLX_CMD_SB              0x1E
#define MLX_CMD_SW_RESET        0xF0
#define MLX_CMD_RM              0x4E
#define MLX_CMD_EX              0x80

#define DEG_TO_RAD              (0.017453292519943295f)
#define RAD_TO_DEG              (57.29577951308232f)
#define ADXL380_G_PER_LSB       (0.00013333f)
#define IAM20380_DPS_PER_LSB    (1.0f / 131.0f)
#define IAM20380_RAD_PER_LSB    (IAM20380_DPS_PER_LSB * DEG_TO_RAD)
#define MADGWICK_BETA_DEFAULT   0.1f

static inline void cs_select(uint pin) {
    asm volatile("nop \n nop \n nop");
    gpio_put(pin, 0);
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect(uint pin) {
    asm volatile("nop \n nop \n nop");
    gpio_put(pin, 1);
    asm volatile("nop \n nop \n nop");
}

static void iam20380_write_register(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), value };
    cs_select(GYRO_CS);
    spi_write_blocking(GYRO_SPI_PORT, tx, 2);
    cs_deselect(GYRO_CS);
}

static void iam20380_read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    uint8_t cmd = reg | 0x80;
    cs_select(GYRO_CS);
    spi_write_blocking(GYRO_SPI_PORT, &cmd, 1);
    spi_read_blocking(GYRO_SPI_PORT, 0x00, buf, len);
    cs_deselect(GYRO_CS);
}

static void adxl380_write_register(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { (uint8_t)((reg << 1) | 0x00), value };
    cs_select(ACC_CS);
    spi_write_blocking(ACC_SPI_PORT, tx, 2);
    cs_deselect(ACC_CS);
}

static void adxl380_read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    uint8_t cmd = (uint8_t)((reg << 1) | 0x01);
    cs_select(ACC_CS);
    spi_write_blocking(ACC_SPI_PORT, &cmd, 1);
    spi_read_blocking(ACC_SPI_PORT, 0x00, buf, len);
    cs_deselect(ACC_CS);
}

static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};

bool accel_config(void) {
    uint8_t devid = 0;
    adxl380_read_registers(ADXL_REG_DEVID, &devid, 1);
    printf("[ADXL380] Dev ID: 0x%02X\n", devid);

    adxl380_write_register(ADXL_REG_OP_MODE, 0x0C);
    sleep_ms(5);
    adxl380_write_register(ADXL_REG_DIG_EN, 0xE0);
    sleep_ms(5);
    return true;
}

bool gyro_config(void) {
    uint8_t whoami = 0;
    iam20380_read_registers(IAM_REG_WHO_AM_I, &whoami, 1);
    printf("[IAM-20380] WHO_AM_I: 0x%02X\n", whoami);

    iam20380_write_register(IAM_REG_PWR_MGMT_1, 0x01);
    sleep_ms(10);
    iam20380_write_register(IAM_REG_PWR_MGMT_2, 0x00);
    sleep_ms(5);
    iam20380_write_register(IAM_REG_GYRO_CONFIG, 0x00);
    sleep_ms(10);
    return true;
}

bool mag_config(void) {
    uint8_t test_cmd = MLX_CMD_SW_RESET;
    int ret = i2c_write_timeout_us(MAG_I2C_PORT, mlx90393_addr, &test_cmd, 1, false, 20000);
    if (ret < 0) {
        mlx90393_addr = MLX90393_I2C_ADDR_ALT;
        ret = i2c_write_timeout_us(MAG_I2C_PORT, mlx90393_addr, &test_cmd, 1, false, 20000);
        if (ret < 0) {
            printf("[MLX90393] Sensor not detected on I2C bus.\n");
            return false;
        }
    }

    printf("[MLX90393] Found at 0x%02X.\n", mlx90393_addr);
    sleep_ms(15);

    uint8_t sb_cmd = MLX_CMD_SB;
    ret = i2c_write_timeout_us(MAG_I2C_PORT, mlx90393_addr, &sb_cmd, 1, false, 20000);
    if (ret < 0) return false;

    sleep_ms(10);
    return true;
}

void calibrate_gyro(void) {
    printf("[CALIB] Calibrating gyro offset. Keep sensor still...\n");
    int32_t sum[3] = {0, 0, 0};
    const int samples = 250;

    for (int i = 0; i < samples; i++) {
        uint8_t buf[6];
        iam20380_read_registers(IAM_REG_GYRO_XOUT_H, buf, 6);
        for (int a = 0; a < 3; a++) {
            sum[a] += (int16_t)((buf[a * 2] << 8) | buf[(a * 2) + 1]);
        }
        sleep_ms(5);
    }

    for (int a = 0; a < 3; a++) {
        gyro_bias[a] = ((float)sum[a] / samples) * IAM20380_RAD_PER_LSB;
    }

    printf("[CALIB] Gyro bias: X=%.4f, Y=%.4f, Z=%.4f\n",
           gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}

bool read_accel(float accel[3]) {
    uint8_t buf[6];
    adxl380_read_registers(ADXL_REG_XDATA_H, buf, 6);
    for (int i = 0; i < 3; i++) {
        accel[i] = (float)((int16_t)((buf[i * 2] << 8) | buf[(i * 2) + 1])) * ADXL380_G_PER_LSB;
    }
    return true;
}

bool read_gyro(float gyro[3]) {
    uint8_t buf[6];
    iam20380_read_registers(IAM_REG_GYRO_XOUT_H, buf, 6);
    for (int i = 0; i < 3; i++) {
        gyro[i] = ((float)((int16_t)((buf[i * 2] << 8) | buf[(i * 2) + 1])) * IAM20380_RAD_PER_LSB) - gyro_bias[i];
    }
    return true;
}

bool read_mag(float mag[3]) {
    uint8_t cmd = MLX_CMD_RM;
    int ret = i2c_write_timeout_us(MAG_I2C_PORT, mlx90393_addr, &cmd, 1, false, 5000);
    if (ret < 0) return false;

    uint8_t buf[7];
    ret = i2c_read_timeout_us(MAG_I2C_PORT, mlx90393_addr, buf, 7, false, 5000);
    if (ret < 0 || (buf[0] & 0x01)) return false;

    for (int i = 0; i < 3; i++) {
        mag[i] = (float)((int16_t)((buf[(i * 2) + 1] << 8) | buf[(i * 2) + 2]));
    }
    return true;
}

typedef struct {
    float q0, q1, q2, q3;
    float beta;
} MadgwickFilter;

static MadgwickFilter ahrs = {
    .q0 = 1.0f,
    .q1 = 0.0f,
    .q2 = 0.0f,
    .q3 = 0.0f,
    .beta = MADGWICK_BETA_DEFAULT
};

// Unified Madgwick Filter Update: supports 6-DOF (use_mag=false) and 9-DOF (use_mag=true)
void madgwick_update(MadgwickFilter *filter, float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz,
                     float dt, bool use_mag) {
    float q0 = filter->q0, q1 = filter->q1, q2 = filter->q2, q3 = filter->q3;
    float recipNorm;
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;

    // Gyroscope angular rate integration
    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // Accelerometer feedback (Gravity vector correction)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
        float q0q0 = q0 * q0,   q1q1 = q1 * q1,   q2q2 = q2 * q2,   q3q3 = q3 * q3;

        float fg1 = 2.0f * (q1 * q3 - q0 * q2) - ax;
        float fg2 = 2.0f * (q0 * q1 + q2 * q3) - ay;
        float fg3 = 2.0f * (0.5f - q1q1 - q2q2) - az;

        s0 = -_2q2 * fg1 + _2q1 * fg2;
        s1 =  _2q3 * fg1 + _2q0 * fg2 - 4.0f * q1 * fg3;
        s2 = -_2q0 * fg1 + _2q3 * fg2 - 4.0f * q2 * fg3;
        s3 =  _2q1 * fg1 + _2q2 * fg2;

        // Magnetometer feedback (Heading correction for 9-DOF)
        if (use_mag && !((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
            recipNorm = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            float _2q0mx = 2.0f * q0 * mx, _2q0my = 2.0f * q0 * my, _2q0mz = 2.0f * q0 * mz, _2q1mx = 2.0f * q1 * mx;

            float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + 2.0f * q1 * my * q2 + 2.0f * q1 * mz * q3 - mx * q2q2 - mx * q3q3;
            float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + 2.0f * q2 * mz * q3 - my * q3q3;
            float _2bx = sqrtf(hx * hx + hy * hy);
            float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + 2.0f * q2 * my * q3 - mz * q2q2 + mz * q3q3;
            float _4bx = 2.0f * _2bx, _4bz = 2.0f * _2bz;

            float fb1 = _2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1 * q3 - q0 * q2) - mx;
            float fb2 = _2bx * (q1 * q2 - q0 * q3)  + _2bz * (q0 * q1 + q2 * q3) - my;
            float fb3 = _2bx * (q0 * q2 + q1 * q3)  + _2bz * (0.5f - q1q1 - q2q2) - mz;

            s0 += -_2bz * q2 * fb1 + (-_2bx * q3 + _2bz * q1) * fb2 + _2bx * q2 * fb3;
            s1 +=  _2bz * q3 * fb1 + ( _2bx * q2 + _2bz * q0) * fb2 + (_2bx * q3 - _4bz * q1) * fb3;
            s2 += (-_4bx * q2 - _2bz * q0) * fb1 + (_2bx * q1 + _2bz * q3) * fb2 + (_2bx * q0 - _4bz * q2) * fb3;
            s3 += (-_4bx * q3 + _2bz * q1) * fb1 + (-_2bx * q0 + _2bz * q2) * fb2 + _2bx * q1 * fb3;
        }

        recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        qDot1 -= filter->beta * s0;
        qDot2 -= filter->beta * s1;
        qDot3 -= filter->beta * s2;
        qDot4 -= filter->beta * s3;
    }

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    filter->q0 = q0 * recipNorm;
    filter->q1 = q1 * recipNorm;
    filter->q2 = q2 * recipNorm;
    filter->q3 = q3 * recipNorm;
}

void madgwick_get_euler(const MadgwickFilter *filter, float *roll, float *pitch, float *yaw) {
    float q0 = filter->q0, q1 = filter->q1, q2 = filter->q2, q3 = filter->q3;

    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(90.0f, sinp);
    else
        *pitch = asinf(sinp) * RAD_TO_DEG;

    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    *yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

int main(void) {
    stdio_init_all();
    sleep_ms(2500);

    // Mode switch: set to true for 9-DOF, false for 6-DOF
    bool use_9dof = USE_9DOF_MODE;

    spi_init(GYRO_SPI_PORT, 1000 * 1000);
    gpio_set_function(GYRO_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(GYRO_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(GYRO_MISO, GPIO_FUNC_SPI);
    gpio_init(GYRO_CS);
    gpio_set_dir(GYRO_CS, GPIO_OUT);
    gpio_put(GYRO_CS, 1);

    spi_init(ACC_SPI_PORT, 1000 * 1000);
    gpio_set_function(ACC_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(ACC_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(ACC_MISO, GPIO_FUNC_SPI);
    gpio_init(ACC_CS);
    gpio_set_dir(ACC_CS, GPIO_OUT);
    gpio_put(ACC_CS, 1);

    bool mag_available = false;
    if (use_9dof) {
        i2c_init(MAG_I2C_PORT, 400 * 1000);
        gpio_set_function(MAG_SDA, GPIO_FUNC_I2C);
        gpio_set_function(MAG_SCL, GPIO_FUNC_I2C);
        gpio_pull_up(MAG_SDA);
        gpio_pull_up(MAG_SCL);
    }

    sleep_ms(50);

    accel_config();
    gyro_config();
    if (use_9dof) {
        mag_available = mag_config();
    }

    calibrate_gyro();

    printf("\nStarting IMU Filter in [%s] Mode...\n", use_9dof ? "9-DOF MARG" : "6-DOF IMU");

    absolute_time_t t_last = get_absolute_time();
    uint32_t print_counter = 0;

    while (true) {
        absolute_time_t t_now = get_absolute_time();
        int64_t dt_us = absolute_time_diff_us(t_last, t_now);
        t_last = t_now;
        float dt = (float)dt_us * 1e-6f;
        if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

        float accel[3] = {0.0f, 0.0f, 0.0f};
        float gyro[3]  = {0.0f, 0.0f, 0.0f};
        float mag[3]   = {0.0f, 0.0f, 0.0f};

        read_accel(accel);
        read_gyro(gyro);

        if (use_9dof && mag_available) {
            if (!read_mag(mag)) {
                mag[0] = mag[1] = mag[2] = 0.0f;
            }
        }

        madgwick_update(&ahrs,
                        gyro[0], gyro[1], gyro[2],
                        accel[0], accel[1], accel[2],
                        mag[0], mag[1], mag[2],
                        dt, (use_9dof && mag_available));

        if (++print_counter % 10 == 0) {
            float roll, pitch, yaw;
            madgwick_get_euler(&ahrs, &roll, &pitch, &yaw);

            printf("[%s] Roll:%6.1f Pitch:%6.1f Yaw:%6.1f | Ax:%5.2f Ay:%5.2f Az:%5.2f | Gx:%6.1f Gy:%6.1f Gz:%6.1f\n",
                   (use_9dof && mag_available) ? "9-DOF" : "6-DOF",
                   roll, pitch, yaw,
                   accel[0], accel[1], accel[2],
                   gyro[0] * RAD_TO_DEG, gyro[1] * RAD_TO_DEG, gyro[2] * RAD_TO_DEG);
        }

        sleep_ms(10);
    }

    return 0;
}
