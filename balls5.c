/*
 * MPU6050 I2C Test for QNX 8.0
 * Tests I2C communication with GY-521 (MPU6050) sensor
 * Based on BlackBerry QNX I2C API pattern
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <hw/i2c.h>

// Return codes
#define I2C_SUCCESS 0
#define I2C_ERROR_NOT_CONNECTED -1
#define I2C_ERROR_ALLOC_FAILED -2
#define I2C_ERROR_OPERATION_FAILED -3

// MPU6050 Configuration
#define MPU6050_ADDR_LOW  0x68  // AD0 connected to GND
#define MPU6050_ADDR_HIGH 0x69  // AD0 connected to VCC

// MPU6050 Registers
#define WHO_AM_I          0x75  // Should return 0x68
#define PWR_MGMT_1        0x6B  // Power management register
#define ACCEL_XOUT_H      0x3B  // Accelerometer data registers start
#define GYRO_XOUT_H       0x43  // Gyroscope data registers start

// I2C message structures
struct i2c_recv_data_msg_t
{
    i2c_sendrecv_t hdr;
    uint8_t bytes[0];
};

struct i2c_send_data_msg_t
{
    i2c_send_t hdr;
    uint8_t bytes[0];
};

// Global file descriptor for I2C
static int smbus_fd = -1;

// Open I2C bus
int open_i2c_bus(unsigned bus_number)
{
    char device_name[20];
    sprintf(device_name, "/dev/i2c%d", bus_number);

    if (smbus_fd == -1)
    {
        smbus_fd = open(device_name, O_RDWR);
        if (smbus_fd < 0)
        {
            printf("Failed to open %s: %s\n", device_name, strerror(errno));
            return I2C_ERROR_NOT_CONNECTED;
        }
    }

    return I2C_SUCCESS;
}

// Close I2C bus
void close_i2c_bus()
{
    if (smbus_fd != -1)
    {
        close(smbus_fd);
        smbus_fd = -1;
    }
}

// Read a single byte from MPU6050
int mpu6050_read_byte(uint8_t i2c_address, uint8_t register_val, uint8_t *value)
{
    int err;

    // Allocate memory for the message
    struct i2c_recv_data_msg_t *msg = malloc(sizeof(struct i2c_recv_data_msg_t) + 1);
    if (!msg)
    {
        printf("Memory allocation failed\n");
        return I2C_ERROR_ALLOC_FAILED;
    }

    // Assign which register to read
    msg->bytes[0] = register_val;

    // Configure I2C message
    msg->hdr.slave.addr = i2c_address;
    msg->hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    msg->hdr.send_len = 1;
    msg->hdr.recv_len = 1;
    msg->hdr.stop = 1;

    // Send the I2C message
    int status;
    err = devctl(smbus_fd, DCMD_I2C_SENDRECV, msg, sizeof(*msg) + 1, &status);
    if (err != EOK)
    {
        printf("I2C read failed: %s\n", strerror(errno));
        free(msg);
        return I2C_ERROR_OPERATION_FAILED;
    }

    // Return the read data
    *value = msg->bytes[0];

    free(msg);
    return I2C_SUCCESS;
}

// Write a single byte to MPU6050
int mpu6050_write_byte(uint8_t i2c_address, uint8_t register_val, uint8_t value)
{
    int err;

    // Allocate memory for the message
    struct i2c_send_data_msg_t *msg = malloc(sizeof(struct i2c_send_data_msg_t) + 2);
    if (!msg)
    {
        printf("Memory allocation failed\n");
        return I2C_ERROR_ALLOC_FAILED;
    }

    // Set register and value
    msg->bytes[0] = register_val;
    msg->bytes[1] = value;

    // Configure I2C message
    msg->hdr.slave.addr = i2c_address;
    msg->hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    msg->hdr.len = 2;
    msg->hdr.stop = 1;

    // Send the I2C message
    err = devctl(smbus_fd, DCMD_I2C_SEND, msg, sizeof(struct i2c_send_data_msg_t) + 2, NULL);
    if (err != EOK)
    {
        printf("I2C write failed: %s\n", strerror(errno));
        free(msg);
        return I2C_ERROR_OPERATION_FAILED;
    }

    free(msg);
    return I2C_SUCCESS;
}

// Read multiple bytes from MPU6050
int mpu6050_read_block(uint8_t i2c_address, uint8_t register_val, uint8_t *buffer, uint8_t length)
{
    int err;

    // Allocate memory for the message
    struct i2c_recv_data_msg_t *msg = malloc(sizeof(struct i2c_recv_data_msg_t) + length);
    if (!msg)
    {
        printf("Memory allocation failed\n");
        return I2C_ERROR_ALLOC_FAILED;
    }

    // Assign which register to start reading from
    msg->bytes[0] = register_val;

    // Configure I2C message
    msg->hdr.slave.addr = i2c_address;
    msg->hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    msg->hdr.send_len = 1;
    msg->hdr.recv_len = length;
    msg->hdr.stop = 1;

    // Send the I2C message
    int status;
    err = devctl(smbus_fd, DCMD_I2C_SENDRECV, msg, sizeof(struct i2c_recv_data_msg_t) + length, &status);
    if (err != EOK)
    {
        printf("I2C block read failed: %s\n", strerror(errno));
        free(msg);
        return I2C_ERROR_OPERATION_FAILED;
    }

    // Copy data to output buffer
    for (int i = 0; i < length; i++)
    {
        buffer[i] = msg->bytes[i];
    }

    free(msg);
    return I2C_SUCCESS;
}

// Test MPU6050 connection and functionality
int test_mpu6050(unsigned bus_number, uint8_t address)
{
    uint8_t device_id;
    uint8_t sensor_data[14];

    printf("\nTesting MPU6050 on bus %d at address 0x%02X\n", bus_number, address);

    // Open I2C bus
    if (open_i2c_bus(bus_number) != I2C_SUCCESS)
    {
        printf("Failed to open I2C bus %d\n", bus_number);
        return -1;
    }

    // Read WHO_AM_I register
    if (mpu6050_read_byte(address, WHO_AM_I, &device_id) != I2C_SUCCESS)
    {
        printf("Failed to read WHO_AM_I register\n");
        close_i2c_bus();
        return -1;
    }

    printf("Device ID: 0x%02X ", device_id);
    if (device_id == 0x68)
    {
        printf("(Correct - MPU6050 found!)\n");
    }
    else
    {
        printf("(Incorrect - expected 0x68)\n");
        close_i2c_bus();
        return -1;
    }

    // Wake up the device
    printf("Waking up MPU6050...\n");
    if (mpu6050_write_byte(address, PWR_MGMT_1, 0x00) != I2C_SUCCESS)
    {
        printf("Failed to wake up device\n");
        close_i2c_bus();
        return -1;
    }

    // Small delay after wakeup
    usleep(100000); // 100ms

    // Read accelerometer and gyroscope data (14 bytes total)
    printf("Reading sensor data...\n");
    if (mpu6050_read_block(address, ACCEL_XOUT_H, sensor_data, 14) != I2C_SUCCESS)
    {
        printf("Failed to read sensor data\n");
        close_i2c_bus();
        return -1;
    }

    // Parse the data
    short accel_x = (sensor_data[0] << 8) | sensor_data[1];
    short accel_y = (sensor_data[2] << 8) | sensor_data[3];
    short accel_z = (sensor_data[4] << 8) | sensor_data[5];
    short temp = (sensor_data[6] << 8) | sensor_data[7];
    short gyro_x = (sensor_data[8] << 8) | sensor_data[9];
    short gyro_y = (sensor_data[10] << 8) | sensor_data[11];
    short gyro_z = (sensor_data[12] << 8) | sensor_data[13];

    // Display the results
    printf("\nSensor Data (Raw Values):\n");
    printf("Accelerometer: X=%6d, Y=%6d, Z=%6d\n", accel_x, accel_y, accel_z);
    printf("Temperature:   %6d (raw)\n", temp);
    printf("Gyroscope:     X=%6d, Y=%6d, Z=%6d\n", gyro_x, gyro_y, gyro_z);

    // Convert temperature to Celsius (rough approximation)
    float temp_c = (temp / 340.0) + 36.53;
    printf("Temperature:   %.1f°C\n", temp_c);

    close_i2c_bus();
    return 0;
}

int main()
{
    printf("MPU6050 I2C Test for QNX 8.0\n");
    printf("=============================\n");

    // Test different bus numbers and addresses
    unsigned buses[] = {0, 1};
    uint8_t addresses[] = {MPU6050_ADDR_LOW, MPU6050_ADDR_HIGH};

    for (int bus_idx = 0; bus_idx < 2; bus_idx++)
    {
        for (int addr_idx = 0; addr_idx < 2; addr_idx++)
        {
            if (test_mpu6050(buses[bus_idx], addresses[addr_idx]) == 0)
            {
                printf("\n*** SUCCESS! MPU6050 working on bus %d at address 0x%02X ***\n",
                       buses[bus_idx], addresses[addr_idx]);
                return 0;
            }
        }
    }

    printf("\n*** NO WORKING MPU6050 FOUND ***\n");
    printf("Check:\n");
    printf("1. Wiring (VCC=3.3V, GND, SDA, SCL)\n");
    printf("2. I2C pull-up resistors (4.7kΩ)\n");
    printf("3. AD0 pin connection (GND=0x68, VCC=0x69)\n");
    printf("4. I2C bus permissions\n");

    return 1;
}
