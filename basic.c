#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <hw/inout.h>
#include <hw/i2c.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <sys/neutrino.h>
#include <audio/audio.h>

// MPU6050 I2C address and registers
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

// Drum detection parameters
#define ACCEL_THRESHOLD 15000  // Acceleration threshold for hit detection
#define GYRO_THRESHOLD 5000    // Gyro threshold for motion detection
#define HIT_COOLDOWN_MS 150    // Minimum time between hits
#define SAMPLE_RATE 100        // Sensor sampling rate (Hz)

// Drum zones based on accelerometer position
#define NUM_DRUMS 4
typedef enum {
    KICK_DRUM = 0,
    SNARE_DRUM = 1,
    HI_HAT = 2,
    CRASH = 3
} drum_type_t;

// Sensor data structure
typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    uint32_t timestamp;
} sensor_data_t;

// Drum hit structure
typedef struct {
    drum_type_t drum;
    uint8_t velocity;
    uint32_t timestamp;
} drum_hit_t;

// Global variables
static int i2c_fd;
static pthread_t sensor_thread, audio_thread;
static volatile int running = 1;
static sensor_data_t current_data;
static pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;
static drum_hit_t hit_queue[64];
static int hit_queue_head = 0, hit_queue_tail = 0;
static pthread_mutex_t queue_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t queue_cond = PTHREAD_COND_INITIALIZER;

// Audio samples for each drum (simplified - in real implementation load from files)
static int16_t kick_sample[8192];
static int16_t snare_sample[8192];
static int16_t hihat_sample[4096];
static int16_t crash_sample[12288];

// Function prototypes
int init_mpu6050(void);
void read_sensor_data(sensor_data_t *data);
drum_type_t detect_drum_zone(int16_t ax, int16_t ay, int16_t az);
uint8_t calculate_velocity(int16_t ax, int16_t ay, int16_t az);
void queue_drum_hit(drum_type_t drum, uint8_t velocity);
void *sensor_thread_func(void *arg);
void *audio_thread_func(void *arg);
void play_drum_sound(drum_type_t drum, uint8_t velocity);
void generate_samples(void);
uint32_t get_timestamp_ms(void);

int main(int argc, char *argv[]) {
    printf("Air Drums - Starting...\n");
    
    // Initialize I2C
    i2c_fd = open("/dev/i2c1", O_RDWR);
    if (i2c_fd < 0) {
        perror("Failed to open I2C device");
        return -1;
    }
    
    // Initialize MPU6050
    if (init_mpu6050() != 0) {
        printf("Failed to initialize MPU6050\n");
        close(i2c_fd);
        return -1;
    }
    
    // Generate drum samples
    generate_samples();
    
    // Create threads
    if (pthread_create(&sensor_thread, NULL, sensor_thread_func, NULL) != 0) {
        perror("Failed to create sensor thread");
        close(i2c_fd);
        return -1;
    }
    
    if (pthread_create(&audio_thread, NULL, audio_thread_func, NULL) != 0) {
        perror("Failed to create audio thread");
        running = 0;
        pthread_join(sensor_thread, NULL);
        close(i2c_fd);
        return -1;
    }
    
    printf("Air Drums running. Press Enter to quit...\n");
    getchar();
    
    // Cleanup
    running = 0;
    pthread_cond_broadcast(&queue_cond);
    pthread_join(sensor_thread, NULL);
    pthread_join(audio_thread, NULL);
    close(i2c_fd);
    
    printf("Air Drums stopped.\n");
    return 0;
}

int init_mpu6050(void) {
    struct {
        i2c_send_t hdr;
        uint8_t reg;
        uint8_t data;
    } msg;
    
    msg.hdr.slave.addr = MPU6050_ADDR;
    msg.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    msg.hdr.len = 2;
    msg.hdr.stop = 1;
    
    // Wake up the sensor
    msg.reg = MPU6050_PWR_MGMT_1;
    msg.data = 0x00;
    if (devctl(i2c_fd, DCMD_I2C_SEND, &msg, sizeof(msg), NULL) != EOK) {
        return -1;
    }
    
    usleep(100000); // Wait 100ms
    
    // Configure accelerometer (±4g)
    msg.reg = MPU6050_ACCEL_CONFIG;
    msg.data = 0x08;
    if (devctl(i2c_fd, DCMD_I2C_SEND, &msg, sizeof(msg), NULL) != EOK) {
        return -1;
    }
    
    // Configure gyroscope (±1000°/s)
    msg.reg = MPU6050_GYRO_CONFIG;
    msg.data = 0x10;
    if (devctl(i2c_fd, DCMD_I2C_SEND, &msg, sizeof(msg), NULL) != EOK) {
        return -1;
    }
    
    // Set sample rate to 100Hz
    msg.reg = MPU6050_CONFIG;
    msg.data = 0x03;
    if (devctl(i2c_fd, DCMD_I2C_SEND, &msg, sizeof(msg), NULL) != EOK) {
        return -1;
    }
    
    printf("MPU6050 initialized successfully\n");
    return 0;
}

void read_sensor_data(sensor_data_t *data) {
    struct {
        i2c_sendrecv_t hdr;
        uint8_t reg;
    } msg;
    
    uint8_t buffer[14];
    
    msg.hdr.slave.addr = MPU6050_ADDR;
    msg.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    msg.hdr.send_len = 1;
    msg.hdr.recv_len = 14;
    msg.hdr.stop = 1;
    msg.reg = MPU6050_ACCEL_XOUT_H;
    
    if (devctl(i2c_fd, DCMD_I2C_SENDRECV, &msg, sizeof(msg), buffer) == EOK) {
        data->accel_x = (buffer[0] << 8) | buffer[1];
        data->accel_y = (buffer[2] << 8) | buffer[3];
        data->accel_z = (buffer[4] << 8) | buffer[5];
        // Skip temperature (bytes 6-7)
        data->gyro_x = (buffer[8] << 8) | buffer[9];
        data->gyro_y = (buffer[10] << 8) | buffer[11];
        data->gyro_z = (buffer[12] << 8) | buffer[13];
        data->timestamp = get_timestamp_ms();
    }
}

drum_type_t detect_drum_zone(int16_t ax, int16_t ay, int16_t az) {
    // Determine drum based on dominant acceleration direction
    int16_t abs_x = abs(ax);
    int16_t abs_y = abs(ay);
    int16_t abs_z = abs(az);
    
    if (abs_z > abs_x && abs_z > abs_y) {
        // Vertical motion - kick drum or snare
        return (az < 0) ? KICK_DRUM : SNARE_DRUM;
    } else if (abs_x > abs_y) {
        // Horizontal X motion - hi-hat
        return HI_HAT;
    } else {
        // Horizontal Y motion - crash
        return CRASH;
    }
}

uint8_t calculate_velocity(int16_t ax, int16_t ay, int16_t az) {
    // Calculate magnitude of acceleration
    float magnitude = sqrt(ax*ax + ay*ay + az*az);
    
    // Map to MIDI velocity (0-127)
    uint8_t velocity = (uint8_t)(magnitude / 500.0);
    if (velocity > 127) velocity = 127;
    if (velocity < 20) velocity = 20;  // Minimum velocity
    
    return velocity;
}

void queue_drum_hit(drum_type_t drum, uint8_t velocity) {
    pthread_mutex_lock(&queue_mutex);
    
    int next_tail = (hit_queue_tail + 1) % 64;
    if (next_tail != hit_queue_head) {
        hit_queue[hit_queue_tail].drum = drum;
        hit_queue[hit_queue_tail].velocity = velocity;
        hit_queue[hit_queue_tail].timestamp = get_timestamp_ms();
        hit_queue_tail = next_tail;
        pthread_cond_signal(&queue_cond);
    }
    
    pthread_mutex_unlock(&queue_mutex);
}

void *sensor_thread_func(void *arg) {
    sensor_data_t prev_data = {0};
    uint32_t last_hit_time[NUM_DRUMS] = {0};
    
    printf("Sensor thread started\n");
    
    while (running) {
        sensor_data_t data;
        read_sensor_data(&data);
        
        pthread_mutex_lock(&data_mutex);
        current_data = data;
        pthread_mutex_unlock(&data_mutex);
        
        // Calculate acceleration magnitude for hit detection
        float accel_mag = sqrt(data.accel_x*data.accel_x + 
                              data.accel_y*data.accel_y + 
                              data.accel_z*data.accel_z);
        
        // Detect drum hits
        if (accel_mag > ACCEL_THRESHOLD) {
            drum_type_t drum = detect_drum_zone(data.accel_x, data.accel_y, data.accel_z);
            uint32_t now = get_timestamp_ms();
            
            // Check cooldown period
            if (now - last_hit_time[drum] > HIT_COOLDOWN_MS) {
                uint8_t velocity = calculate_velocity(data.accel_x, data.accel_y, data.accel_z);
                queue_drum_hit(drum, velocity);
                last_hit_time[drum] = now;
                
                printf("Hit detected: Drum %d, Velocity %d\n", drum, velocity);
            }
        }
        
        prev_data = data;
        usleep(1000000 / SAMPLE_RATE); // Sample rate delay
    }
    
    printf("Sensor thread stopped\n");
    return NULL;
}

void *audio_thread_func(void *arg) {
    printf("Audio thread started\n");
    
    while (running) {
        pthread_mutex_lock(&queue_mutex);
        
        while (hit_queue_head == hit_queue_tail && running) {
            pthread_cond_wait(&queue_cond, &queue_mutex);
        }
        
        if (!running) {
            pthread_mutex_unlock(&queue_mutex);
            break;
        }
        
        drum_hit_t hit = hit_queue[hit_queue_head];
        hit_queue_head = (hit_queue_head + 1) % 64;
        
        pthread_mutex_unlock(&queue_mutex);
        
        play_drum_sound(hit.drum, hit.velocity);
    }
    
    printf("Audio thread stopped\n");
    return NULL;
}

void play_drum_sound(drum_type_t drum, uint8_t velocity) {
    // This is a simplified implementation
    // In a real implementation, you'd use QNX audio services
    printf("Playing drum %d with velocity %d\n", drum, velocity);
    
    // For demonstration, just write to /dev/dsp or use audio library
    // The actual implementation would depend on your audio setup
}

void generate_samples(void) {
    // Generate simple synthetic drum sounds
    for (int i = 0; i < 8192; i++) {
        float t = (float)i / 44100.0;
        
        // Kick drum - low frequency sine with exponential decay
        kick_sample[i] = (int16_t)(32767 * sin(2 * M_PI * 60 * t) * exp(-t * 20));
        
        // Snare drum - noise with exponential decay
        snare_sample[i] = (int16_t)(32767 * (rand() / (float)RAND_MAX - 0.5) * exp(-t * 15));
    }
    
    for (int i = 0; i < 4096; i++) {
        float t = (float)i / 44100.0;
        // Hi-hat - high frequency noise with quick decay
        hihat_sample[i] = (int16_t)(16383 * (rand() / (float)RAND_MAX - 0.5) * exp(-t * 50));
    }
    
    for (int i = 0; i < 12288; i++) {
        float t = (float)i / 44100.0;
        // Crash - complex waveform with slow decay
        crash_sample[i] = (int16_t)(24575 * (sin(2 * M_PI * 400 * t) + 
                                            0.5 * sin(2 * M_PI * 800 * t) +
                                            0.3 * (rand() / (float)RAND_MAX - 0.5)) * exp(-t * 5));
    }
    
    printf("Drum samples generated\n");
}

uint32_t get_timestamp_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}
