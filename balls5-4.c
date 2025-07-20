#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <hw/i2c.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>


// MPU6050 Constants
#define MPU6050_ADDR1        0x68
#define MPU6050_ADDR2        0x69
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

int mpu6050_read_byte(uint8_t addr, uint8_t reg, uint8_t *val);
int mpu6050_write_byte(uint8_t addr, uint8_t reg, uint8_t val);
void close_i2c_bus();
void quick_calibrate(uint8_t addr, float *ax_bias, float *ay_bias, float *az_bias);
void print_sensor_data(uint8_t addr, float ax_bias, float ay_bias, float az_bias, const char *label);



void* mpu_thread_func(void* arg) {
    float ax_bias, ay_bias, az_bias;

    const char* label = (const char*)arg;
    uint8_t sensor_addr;

    if (strcmp(label, "Right") == 0){
    	sensor_addr = 0x69;
    }
    else
    	sensor_addr = 0x68;

    uint8_t id1 = 0;
    if (mpu6050_read_byte(sensor_addr, WHO_AM_I, &id1) != I2C_SUCCESS || id1 != 0x68) {
        printf("Sensor %X not detected. ID = 0x%02X\n", sensor_addr, id1);
        close_i2c_bus();
        return NULL;
    }

    mpu6050_write_byte(sensor_addr, PWR_MGMT_1, 0x00);
    mpu6050_write_byte(sensor_addr, CONFIG, 0x02);

    quick_calibrate(sensor_addr, &ax_bias, &ay_bias, &az_bias);


    while (1) {

        print_sensor_data(sensor_addr, ax_bias, ay_bias, az_bias, label);

        usleep(500000); // 5ms delay
    }
    return NULL;
}




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
        usleep(2000);
    }

    *ax_bias /= 5.0f;
    *ay_bias /= 5.0f;
    *az_bias /= 5.0f;
}

// ---------- Continuous Read ----------
void print_sensor_data(uint8_t addr, float ax_bias, float ay_bias, float az_bias, const char *label) {
    uint8_t data[14];

    if (mpu6050_read_block(addr, ACCEL_XOUT_H, data, 14) != I2C_SUCCESS) {
        printf("[%s] Read failed\n", label);
        return;
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

    printf("\n[%s] Accel: X=%.2f g, Y=%.2f g, Z=%.2f g\n", label, ax, ay, az);
    printf("[%s] Gyro:  X=%.2f °/s, Y=%.2f °/s, Z=%.2f °/s\n", label, gx, gy, gz);
    printf("[%s] Temp:  %.2f °C\n", label, temp_c);
}

void continuous_dual_read() {


    printf("Calibration Done.\n");

    while (1) {
        //print_sensor_data(MPU6050_ADDR1, ax_bias1, ay_bias1, az_bias1, "Sensor 1");
        //print_sensor_data(MPU6050_ADDR2, ax_bias2, ay_bias2, az_bias2, "Sensor 2");

        usleep(50000); // total ~20Hz for both sensors
    }
}

// ---------- Main ----------
int main() {

	pthread_t right, left;


    if (open_i2c_bus(1) != I2C_SUCCESS) {
        return 1;
    }
	pthread_create(&right, NULL, mpu_thread_func, "Right");
	pthread_create(&left, NULL, mpu_thread_func, "Left");

	struct sched_param param;
	param.sched_priority = 20;
	pthread_setschedparam(right, SCHED_FIFO, &param);
	param.sched_priority = 19;
	pthread_setschedparam(left, SCHED_FIFO, &param);


    printf("MPU6050 Dual Sensor Mode (QNX)\n");



    usleep(100000); // wait 100ms

    pthread_join(right, NULL);
    pthread_join(left, NULL);
    close_i2c_bus();
    return 0;
}
