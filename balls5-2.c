#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <hw/i2c.h>

// MPU6050 Constants
#define MPU6050_ADDR         0x68
#define WHO_AM_I             0x75
#define PWR_MGMT_1           0x6B
#define CONFIG               0x1A
#define ACCEL_XOUT_H         0x3B
#define GYRO_XOUT_H          0x43

#define I2C_SUCCESS 0
#define I2C_ERROR   -1

// I2C Message Structures
struct i2c_recv_data_msg_t {
    i2c_sendrecv_t hdr;
    uint8_t bytes[0];
};

struct i2c_send_data_msg_t {
    i2c_send_t hdr;
    uint8_t bytes[0];
};

static int smbus_fd = -1;

// ---------- I2C Utilities ----------
int open_i2c_bus(unsigned bus_number) {
    char device[20];
    snprintf(device, sizeof(device), "/dev/i2c%d", bus_number);
    smbus_fd = open(device, O_RDWR);
    if (smbus_fd < 0) {
        perror("I2C open failed");
        return I2C_ERROR;
    }
    return I2C_SUCCESS;
}

void close_i2c_bus() {
    if (smbus_fd != -1) {
        close(smbus_fd);
        smbus_fd = -1;
    }
}

int mpu6050_write_byte(uint8_t addr, uint8_t reg, uint8_t val) {
    struct i2c_send_data_msg_t *msg = malloc(sizeof(*msg) + 2);
    if (!msg) return I2C_ERROR;
    msg->bytes[0] = reg;
    msg->bytes[1] = val;
    msg->hdr.slave.addr = addr;
    msg->hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    msg->hdr.len = 2;
    msg->hdr.stop = 1;
    int err = devctl(smbus_fd, DCMD_I2C_SEND, msg, sizeof(*msg) + 2, NULL);
    free(msg);
    return (err == EOK) ? I2C_SUCCESS : I2C_ERROR;
}

int mpu6050_read_block(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    struct i2c_recv_data_msg_t *msg = malloc(sizeof(*msg) + len);
    if (!msg) return I2C_ERROR;
    msg->bytes[0] = reg;
    msg->hdr.slave.addr = addr;
    msg->hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    msg->hdr.send_len = 1;
    msg->hdr.recv_len = len;
    msg->hdr.stop = 1;
    int status, err = devctl(smbus_fd, DCMD_I2C_SENDRECV, msg, sizeof(*msg) + len, &status);
    if (err == EOK) {
        memcpy(buf, msg->bytes, len);
        free(msg);
        return I2C_SUCCESS;
    } else {
        free(msg);
        return I2C_ERROR;
    }
}

int mpu6050_read_byte(uint8_t addr, uint8_t reg, uint8_t *val) {
    return mpu6050_read_block(addr, reg, val, 1);
}

// ---------- Helper ----------
short to_int16(uint8_t hi, uint8_t lo) {
    return (short)((hi << 8) | lo);
}

// ---------- Fast Offset Calibration ----------
void quick_calibrate(uint8_t addr, float *ax_bias, float *ay_bias, float *az_bias) {
    *ax_bias = *ay_bias = *az_bias = 0;
    uint8_t buf[6];

    for (int i = 0; i < 5; i++) {
        if (mpu6050_read_block(addr, ACCEL_XOUT_H, buf, 6) == I2C_SUCCESS) {
            *ax_bias += to_int16(buf[0], buf[1]);
            *ay_bias += to_int16(buf[2], buf[3]);
            *az_bias += to_int16(buf[4], buf[5]);
        }
        usleep(2000); // 2ms delay between samples
    }

    *ax_bias /= 5.0f;
    *ay_bias /= 5.0f;
    *az_bias /= 5.0f;
}

// ---------- Continuous Read ----------
void continuous_read_fast(uint8_t addr) {
    uint8_t data[14];
    float ax_bias, ay_bias, az_bias;

    quick_calibrate(addr, &ax_bias, &ay_bias, &az_bias);
    printf("Quick Calibration Done. Bias X=%.1f Y=%.1f Z=%.1f\n", ax_bias, ay_bias, az_bias);

    while (1) {
        if (mpu6050_read_block(addr, ACCEL_XOUT_H, data, 14) != I2C_SUCCESS) {
            printf("Read failed\n");
            break;
        }

        short raw_ax = to_int16(data[0], data[1]);
        short raw_ay = to_int16(data[2], data[3]);
        short raw_az = to_int16(data[4], data[5]);
        short temp_raw = to_int16(data[6], data[7]);
        short raw_gx = to_int16(data[8], data[9]);
        short raw_gy = to_int16(data[10], data[11]);
        short raw_gz = to_int16(data[12], data[13]);

        float ax = (raw_ax - ax_bias) / 16384.0f;
        float ay = (raw_ay - ay_bias) / 16384.0f;
        float az = (raw_az - az_bias) / 16384.0f;

        float gx = raw_gx / 131.0f;
        float gy = raw_gy / 131.0f;
        float gz = raw_gz / 131.0f;

        float temp_c = (temp_raw / 340.0f) + 36.53f;

        printf("\nAccel: X=%.2f g, Y=%.2f g, Z=%.2f g\n", ax, ay, az);
        printf("Gyro:  X=%.2f °/s, Y=%.2f °/s, Z=%.2f °/s\n", gx, gy, gz);
        printf("Temp:  %.2f °C\n", temp_c);

        usleep(50000); // 50 ms delay => ~20Hz
    }
}

// ---------- Main ----------
int main() {
    printf("MPU6050 Fast Read Mode (QNX)\n");

    if (open_i2c_bus(1) != I2C_SUCCESS) {
        return 1;
    }

    uint8_t id = 0;
    if (mpu6050_read_byte(MPU6050_ADDR, WHO_AM_I, &id) != I2C_SUCCESS || id != 0x68) {
        printf("MPU6050 not detected. ID = 0x%02X\n", id);
        close_i2c_bus();
        return 1;
    }

    // Wake up and set DLPF
    mpu6050_write_byte(MPU6050_ADDR, PWR_MGMT_1, 0x00);
    mpu6050_write_byte(MPU6050_ADDR, CONFIG, 0x02); // DLPF_CFG = 2 (~94 Hz)

    usleep(100000); // wait 100ms for stabilization

    continuous_read_fast(MPU6050_ADDR);

    close_i2c_bus();
    return 0;
}
