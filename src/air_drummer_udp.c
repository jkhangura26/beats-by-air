#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <hw/i2c.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/mman.h>

// UDP Configuration
#define MAC_IP "192.168.137.83"
#define UDP_PORT 8888

// MPU6050 Constants
#define MPU6050_ADDR1        0x68
#define MPU6050_ADDR2        0x69
#define WHO_AM_I             0x75
#define PWR_MGMT_1           0x6B
#define CONFIG               0x1A
#define ACCEL_XOUT_H         0x3B
#define GYRO_CONFIG          0x1B
#define ACCEL_CONFIG         0x1C
#define GYRO_XOUT_H          0x43

#define I2C_SUCCESS 0
#define I2C_ERROR   -1

// OPTIMIZED: Better velocity thresholds for more varied response
#define GESTURE_BUFFER_SIZE    6
#define GESTURE_THRESHOLD      1.8f     // Lower threshold for softer hits
#define DECELERATION_THRESHOLD -1.8f    // Less aggressive deceleration
#define PEAK_DETECT_TIME_MS    45
#define SAMPLE_RATE_US        1500
#define MIN_HIT_INTERVAL      80000
#define HIT_COOLDOWN_PERIOD   25000

// UPDATED: Square-like zone layout - each hand gets 2 zones
// RIGHT HAND: Upper zones (SNARE top, TOM bottom)
#define RIGHT_SNARE_PITCH_MIN    10.0f   // Upper zone
#define RIGHT_SNARE_PITCH_MAX    90.0f

#define RIGHT_TOM_PITCH_MIN      -90.0f  // Lower zone
#define RIGHT_TOM_PITCH_MAX      10.0f

// LEFT HAND: Side zones (HIHAT right, CRASH left)
#define LEFT_HIHAT_ROLL_MIN      10.0f   // Right zone
#define LEFT_HIHAT_ROLL_MAX      90.0f

#define LEFT_CRASH_ROLL_MIN      -90.0f  // Left zone
#define LEFT_CRASH_ROLL_MAX      10.0f

// Neutral zone (center) - smaller for more precise control
#define NEUTRAL_PITCH_MIN    -10.0f
#define NEUTRAL_PITCH_MAX    10.0f
#define NEUTRAL_ROLL_MIN     -10.0f
#define NEUTRAL_ROLL_MAX     10.0f

#define LEFT_DEFAULT_DRUM    DRUM_HIHAT
#define RIGHT_DEFAULT_DRUM   DRUM_SNARE

// UPDATED: More granular velocity thresholds
#define VELOCITY_SOFT_MAX      2.5f    // Below this = soft
#define VELOCITY_MEDIUM_MAX    4.0f    // Below this = medium
#define VELOCITY_HARD_MAX      6.0f    // Below this = hard
// Above VELOCITY_HARD_MAX = very_hard

typedef enum {
    DRUM_SNARE = 0,
    DRUM_HIHAT = 1,
    DRUM_TOM = 2,
    DRUM_CRASH = 3,
    DRUM_KICK = 4,
    DRUM_RIDE = 5,
    DRUM_NEUTRAL = 6,
    DRUM_NONE = -1
} drum_type_t;

typedef enum {
    HAND_LEFT = 0,
    HAND_RIGHT = 1
} hand_type_t;

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    float total_accel;
    float total_gyro;
    float pitch, roll;
    uint64_t timestamp;
} motion_sample_t;

typedef struct {
    motion_sample_t samples[GESTURE_BUFFER_SIZE];
    int buffer_index;
    drum_type_t current_zone;
    drum_type_t predicted_zone;
    uint64_t last_hit_time;
    uint64_t last_display_time;
    float hit_intensity;
    int detecting_hit;
    uint64_t hit_start_time;
    float max_accel_during_hit;
    float min_accel_during_hit;
    float ax_bias, ay_bias, az_bias;
    float gx_bias, gy_bias, gz_bias;
    hand_type_t hand;
    char last_display[64];

    float accel_smooth[3];
    float gyro_smooth[3];
    float alpha;

    float velocity_estimate;
    int consecutive_hits;
} drumstick_state_t;

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
static int udp_socket = -1;
static struct sockaddr_in mac_addr;

static struct i2c_recv_data_msg_t *read_msg_left = NULL;
static struct i2c_recv_data_msg_t *read_msg_right = NULL;
static struct i2c_send_data_msg_t *write_msg = NULL;

// Function declarations
const char* get_hand_emoji(hand_type_t hand);
const char* get_drum_emoji(drum_type_t drum);
drum_type_t get_drum_type_from_name(const char* name);
int mpu6050_read_byte(uint8_t addr, uint8_t reg, uint8_t *val);
void close_i2c_bus();
int mpu6050_write_byte(uint8_t addr, uint8_t reg, uint8_t val);
void init_drumstick_state(drumstick_state_t *state, hand_type_t hand);
void calibrate_drumstick(uint8_t addr, drumstick_state_t *state);
void print_instructions();
int process_stick_data(uint8_t addr, drumstick_state_t *state);

// UPDATED: Improved velocity calculation
const char* get_velocity_name(float intensity) {
    if (intensity <= 0.3f) return "soft";
    if (intensity <= 0.6f) return "medium";
    if (intensity <= 0.85f) return "hard";
    return "very_hard";
}

// Batch I2C read for both accel and gyro data
int mpu6050_read_motion_data(uint8_t addr, int16_t *accel, int16_t *gyro) {
    struct i2c_recv_data_msg_t *msg = (addr == MPU6050_ADDR1) ? read_msg_left : read_msg_right;

    msg->bytes[0] = ACCEL_XOUT_H;
    msg->hdr.slave.addr = addr;
    msg->hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    msg->hdr.send_len = 1;
    msg->hdr.recv_len = 14;
    msg->hdr.stop = 1;

    int status, err = devctl(smbus_fd, DCMD_I2C_SENDRECV, msg, sizeof(*msg) + 14, &status);
    if (err == EOK) {
        accel[0] = (int16_t)((msg->bytes[0] << 8) | msg->bytes[1]);
        accel[1] = (int16_t)((msg->bytes[2] << 8) | msg->bytes[3]);
        accel[2] = (int16_t)((msg->bytes[4] << 8) | msg->bytes[5]);

        gyro[0] = (int16_t)((msg->bytes[8] << 8) | msg->bytes[9]);
        gyro[1] = (int16_t)((msg->bytes[10] << 8) | msg->bytes[11]);
        gyro[2] = (int16_t)((msg->bytes[12] << 8) | msg->bytes[13]);

        return I2C_SUCCESS;
    }
    return I2C_ERROR;
}

int init_i2c_buffers() {
    read_msg_left = malloc(sizeof(*read_msg_left) + 14);
    read_msg_right = malloc(sizeof(*read_msg_right) + 14);
    write_msg = malloc(sizeof(*write_msg) + 2);

    if (!read_msg_left || !read_msg_right || !write_msg) {
        return I2C_ERROR;
    }
    return I2C_SUCCESS;
}

void cleanup_i2c_buffers() {
    if (read_msg_left) { free(read_msg_left); read_msg_left = NULL; }
    if (read_msg_right) { free(read_msg_right); read_msg_right = NULL; }
    if (write_msg) { free(write_msg); write_msg = NULL; }
}

void* mpu_thread_func(void* arg) {
    struct sched_param param;
    param.sched_priority = (strcmp((const char*)arg, "Right") == 0) ? 21 : 20;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        printf("Warning: Could not lock memory\n");
    }

    const char* label = (const char*)arg;
    uint8_t sensor_addr;
    hand_type_t hand;
    drumstick_state_t stick;
    bool space;

    if (strcmp(label, "Right") == 0) {
        sensor_addr = MPU6050_ADDR2;
        hand = HAND_RIGHT;
        space = false;
    } else {
        sensor_addr = MPU6050_ADDR1;
        hand = HAND_LEFT;
        space = true;
    }

    uint8_t id1 = 0;
    if (mpu6050_read_byte(sensor_addr, WHO_AM_I, &id1) != I2C_SUCCESS || id1 != 0x68) {
        printf("Sensor %X not detected. ID = 0x%02X\n", sensor_addr, id1);
        return NULL;
    }

    mpu6050_write_byte(sensor_addr, PWR_MGMT_1, 0x00);
    mpu6050_write_byte(sensor_addr, CONFIG, 0x00);
    mpu6050_write_byte(sensor_addr, ACCEL_CONFIG, 0x10);
    mpu6050_write_byte(sensor_addr, GYRO_CONFIG, 0x10);
    usleep(10000);

    init_drumstick_state(&stick, hand);

    printf("🎯 Calibrating %s stick...\n", label);
    calibrate_drumstick(sensor_addr, &stick);
    printf("\n✅ Calibration complete!\n");

    if (hand == HAND_LEFT) {
        print_instructions();
        printf("🚀 Air Drumming active! Start playing!\n\n");
    }

    while (1) {
        int result = process_stick_data(sensor_addr, &stick);
        if (result == 2) {
            printf("%s\n", stick.last_display);
            if (space) printf("\n");
        }

        struct timespec sleep_time = {0, SAMPLE_RATE_US * 1000};
        nanosleep(&sleep_time, NULL);
    }

    return NULL;
}

// UDP Functions
int init_udp() {
    udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket < 0) {
        perror("UDP socket creation failed");
        return -1;
    }

    int flags = fcntl(udp_socket, F_GETFL, 0);
    fcntl(udp_socket, F_SETFL, flags | O_NONBLOCK);

    memset(&mac_addr, 0, sizeof(mac_addr));
    mac_addr.sin_family = AF_INET;
    mac_addr.sin_port = htons(UDP_PORT);
    inet_pton(AF_INET, MAC_IP, &mac_addr.sin_addr);

    printf("✅ UDP initialized - sending to %s:%d\n", MAC_IP, UDP_PORT);
    return 0;
}

void send_drum_command(const char* drum_name, float intensity, hand_type_t hand) {
    if (udp_socket < 0) return;

    char command[64];
    const char* velocity = get_velocity_name(intensity);

    snprintf(command, sizeof(command), "%s:%s", drum_name, velocity);

    ssize_t sent = sendto(udp_socket, command, strlen(command), MSG_DONTWAIT,
                         (struct sockaddr*)&mac_addr, sizeof(mac_addr));

    if (sent > 0) {
        printf("%s %s %s HIT! (Vol: %.0f%%, %s) -> UDP\n",
               get_hand_emoji(hand),
               get_drum_emoji(get_drum_type_from_name(drum_name)),
               drum_name,
               intensity * 100, velocity);
    }
}

void close_udp() {
    if (udp_socket >= 0) {
        close(udp_socket);
        udp_socket = -1;
    }
}

drum_type_t get_drum_type_from_name(const char* name) {
    if (strcmp(name, "snare") == 0) return DRUM_SNARE;
    if (strcmp(name, "hihat") == 0) return DRUM_HIHAT;
    if (strcmp(name, "tom") == 0) return DRUM_TOM;
    if (strcmp(name, "crash") == 0) return DRUM_CRASH;
    return DRUM_NONE;
}

// I2C Functions
int open_i2c_bus(unsigned bus_number) {
    char device[20];
    snprintf(device, sizeof(device), "/dev/i2c%d", bus_number);
    smbus_fd = open(device, O_RDWR);
    if (smbus_fd < 0) {
        perror("I2C open failed");
        return I2C_ERROR;
    }
    return init_i2c_buffers();
}

void close_i2c_bus() {
    if (smbus_fd != -1) {
        close(smbus_fd);
        smbus_fd = -1;
    }
    cleanup_i2c_buffers();
}

int mpu6050_write_byte(uint8_t addr, uint8_t reg, uint8_t val) {
    write_msg->bytes[0] = reg;
    write_msg->bytes[1] = val;
    write_msg->hdr.slave.addr = addr;
    write_msg->hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    write_msg->hdr.len = 2;
    write_msg->hdr.stop = 1;
    int err = devctl(smbus_fd, DCMD_I2C_SEND, write_msg, sizeof(*write_msg) + 2, NULL);
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

short to_int16(uint8_t hi, uint8_t lo) {
    return (short)((hi << 8) | lo);
}

uint64_t get_time_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}

void calculate_orientation(float ax, float ay, float az, float *pitch, float *roll) {
    static float last_pitch = 0, last_roll = 0;
    const float filter_alpha = 0.8f;

    float raw_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
    float raw_roll = atan2f(ay, az) * 180.0f / M_PI;

    *pitch = filter_alpha * raw_pitch + (1.0f - filter_alpha) * last_pitch;
    *roll = filter_alpha * raw_roll + (1.0f - filter_alpha) * last_roll;

    *pitch = fmaxf(-90.0f, fminf(90.0f, *pitch));
    *roll = fmaxf(-180.0f, fminf(180.0f, *roll));

    last_pitch = *pitch;
    last_roll = *roll;
}

// UPDATED: Square-layout drum zone detection
drum_type_t get_drum_zone(float pitch, float roll, hand_type_t hand) {
    static drum_type_t last_zone[2] = {DRUM_NEUTRAL, DRUM_NEUTRAL};
    const float hysteresis = 5.0f;

    drum_type_t current_zone = DRUM_NEUTRAL;

    if (hand == HAND_RIGHT) {
        // RIGHT HAND: Vertical zones (pitch-based)
        if (pitch >= (RIGHT_SNARE_PITCH_MIN - hysteresis)) {
            current_zone = DRUM_SNARE;  // Upper zone
        } else if (pitch <= (RIGHT_TOM_PITCH_MAX + hysteresis)) {
            current_zone = DRUM_TOM;    // Lower zone
        }
    } else {
        // LEFT HAND: Horizontal zones (roll-based)
        if (roll >= (LEFT_HIHAT_ROLL_MIN - hysteresis)) {
            current_zone = DRUM_HIHAT;  // Right zone
        } else if (roll <= (LEFT_CRASH_ROLL_MAX + hysteresis)) {
            current_zone = DRUM_CRASH;  // Left zone
        }
    }

    // Check neutral zone for both hands
    if (pitch >= NEUTRAL_PITCH_MIN && pitch <= NEUTRAL_PITCH_MAX &&
        roll >= NEUTRAL_ROLL_MIN && roll <= NEUTRAL_ROLL_MAX) {
        current_zone = DRUM_NEUTRAL;
    }

    // Apply hysteresis to prevent zone flipping
    if (current_zone != last_zone[hand] && current_zone != DRUM_NEUTRAL) {
        last_zone[hand] = current_zone;
    } else if (current_zone == DRUM_NEUTRAL) {
        // Allow immediate neutral zone detection
        last_zone[hand] = current_zone;
    }

    return last_zone[hand];
}

const char* get_drum_name(drum_type_t drum) {
    switch (drum) {
        case DRUM_SNARE: return "snare";
        case DRUM_HIHAT: return "hihat";
        case DRUM_TOM: return "tom";
        case DRUM_CRASH: return "crash";
        case DRUM_KICK: return "kick";
        case DRUM_RIDE: return "ride";
        case DRUM_NEUTRAL: return "neutral";
        default: return "unknown";
    }
}

const char* get_drum_emoji(drum_type_t drum) {
    switch (drum) {
        case DRUM_SNARE: return "🥁";
        case DRUM_HIHAT: return "🔔";
        case DRUM_TOM: return "🪘";
        case DRUM_CRASH: return "💥";
        case DRUM_KICK: return "👣";
        case DRUM_RIDE: return "🔆";
        case DRUM_NEUTRAL: return "👆";
        default: return "❓";
    }
}

const char* get_hand_emoji(hand_type_t hand) {
    return hand == HAND_LEFT ? "👈" : "👉";
}

void play_drum_sound(drum_type_t drum, float intensity, hand_type_t hand) {
    if (drum == DRUM_NEUTRAL || drum == DRUM_NONE) return;
    const char* drum_name = get_drum_name(drum);
    send_drum_command(drum_name, intensity, hand);
}

void add_sample_to_buffer(drumstick_state_t *state, float ax, float ay, float az,
                          float gx, float gy, float gz, float pitch, float roll) {
    motion_sample_t *sample = &state->samples[state->buffer_index];

    state->accel_smooth[0] = state->alpha * ax + (1.0f - state->alpha) * state->accel_smooth[0];
    state->accel_smooth[1] = state->alpha * ay + (1.0f - state->alpha) * state->accel_smooth[1];
    state->accel_smooth[2] = state->alpha * az + (1.0f - state->alpha) * state->accel_smooth[2];

    state->gyro_smooth[0] = state->alpha * gx + (1.0f - state->alpha) * state->gyro_smooth[0];
    state->gyro_smooth[1] = state->alpha * gy + (1.0f - state->alpha) * state->gyro_smooth[1];
    state->gyro_smooth[2] = state->alpha * gz + (1.0f - state->alpha) * state->gyro_smooth[2];

    sample->ax = state->accel_smooth[0];
    sample->ay = state->accel_smooth[1];
    sample->az = state->accel_smooth[2];
    sample->gx = state->gyro_smooth[0];
    sample->gy = state->gyro_smooth[1];
    sample->gz = state->gyro_smooth[2];

    sample->total_accel = sqrtf(ax*ax + ay*ay + az*az);
    sample->total_gyro = sqrtf(gx*gx + gy*gy + gz*gz);
    sample->pitch = pitch;
    sample->roll = roll;
    sample->timestamp = get_time_us();

    state->buffer_index = (state->buffer_index + 1) % GESTURE_BUFFER_SIZE;
}

motion_sample_t* get_sample(drumstick_state_t *state, int offset) {
    int index = (state->buffer_index - 1 - offset + GESTURE_BUFFER_SIZE) % GESTURE_BUFFER_SIZE;
    return &state->samples[index];
}

// UPDATED: Better velocity range distribution
int detect_hit_gesture(drumstick_state_t *state) {
    uint64_t current_time = get_time_us();
    if ((current_time - state->last_hit_time) <= MIN_HIT_INTERVAL) {
        return 0;
    }

    motion_sample_t *current = get_sample(state, 0);
    if (current->timestamp == 0) return 0;

    if (!state->detecting_hit) {
        motion_sample_t *prev1 = get_sample(state, 1);
        motion_sample_t *prev2 = get_sample(state, 2);
        if (prev1->timestamp == 0 || prev2->timestamp == 0) return 0;

        float accel_delta = current->total_accel - prev2->total_accel;
        float gyro_delta = current->total_gyro - prev2->total_gyro;
        float combined_motion = sqrtf(accel_delta*accel_delta + gyro_delta*gyro_delta);

        if (combined_motion > 2.0f && current->total_accel > 1.3f) {  // Lower threshold
            state->detecting_hit = 1;
            state->hit_start_time = current_time;
            state->max_accel_during_hit = current->total_accel;
            state->min_accel_during_hit = current->total_accel;
            state->current_zone = get_drum_zone(current->pitch, current->roll, state->hand);
            return 0;
        }
        return 0;
    }

    if (state->detecting_hit) {
        if (current->total_accel > state->max_accel_during_hit) {
            state->max_accel_during_hit = current->total_accel;
        }
        if (current->total_accel < state->min_accel_during_hit) {
            state->min_accel_during_hit = current->total_accel;
        }

        motion_sample_t *prev1 = get_sample(state, 1);
        if (prev1->timestamp == 0) return 0;

        float accel_change = current->total_accel - prev1->total_accel;
        float velocity_change = current->total_gyro - prev1->total_gyro;

        if (accel_change < DECELERATION_THRESHOLD ||
            (state->max_accel_during_hit - current->total_accel) > 1.0f ||
            velocity_change < -12.0f) {

            // UPDATED: More balanced intensity calculation for better velocity distribution
            float raw_intensity = state->max_accel_during_hit;
            float normalized_intensity;

            // Map acceleration ranges to velocity levels more evenly
            if (raw_intensity <= VELOCITY_SOFT_MAX) {
                normalized_intensity = 0.2f + (raw_intensity / VELOCITY_SOFT_MAX) * 0.15f; // 0.2-0.35
            } else if (raw_intensity <= VELOCITY_MEDIUM_MAX) {
                normalized_intensity = 0.35f + ((raw_intensity - VELOCITY_SOFT_MAX) / (VELOCITY_MEDIUM_MAX - VELOCITY_SOFT_MAX)) * 0.25f; // 0.35-0.6
            } else if (raw_intensity <= VELOCITY_HARD_MAX) {
                normalized_intensity = 0.6f + ((raw_intensity - VELOCITY_MEDIUM_MAX) / (VELOCITY_HARD_MAX - VELOCITY_MEDIUM_MAX)) * 0.25f; // 0.6-0.85
            } else {
                normalized_intensity = 0.85f + fminf(0.15f, (raw_intensity - VELOCITY_HARD_MAX) / 3.0f); // 0.85-1.0
            }

            state->hit_intensity = fmaxf(0.2f, fminf(1.0f, normalized_intensity));

            state->detecting_hit = 0;
            state->last_hit_time = current_time;
            state->consecutive_hits++;
            return 1;
        }

        if ((current_time - state->hit_start_time) > PEAK_DETECT_TIME_MS * 1000) {
            state->detecting_hit = 0;
            return 0;
        }
    }
    return 0;
}

void calibrate_drumstick(uint8_t addr, drumstick_state_t *state) {
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    const int samples = 40;

    int16_t accel[3], gyro[3];

    for (int i = 0; i < samples; i++) {
        if (mpu6050_read_motion_data(addr, accel, gyro) == I2C_SUCCESS) {
            sum_ax += accel[0];
            sum_ay += accel[1];
            sum_az += accel[2];
            sum_gx += gyro[0];
            sum_gy += gyro[1];
            sum_gz += gyro[2];
        }
        usleep(5000);
        if (i % 8 == 0) {
            printf(".");
            fflush(stdout);
        }
    }

    state->ax_bias = sum_ax / samples;
    state->ay_bias = sum_ay / samples;
    state->az_bias = (sum_az / samples) - 4096.0f;
    state->gx_bias = sum_gx / samples;
    state->gy_bias = sum_gy / samples;
    state->gz_bias = sum_gz / samples;
}

// UPDATED: New zone layout instructions
void print_instructions() {
    printf("\n🥁🥁 SQUARE LAYOUT AIR DRUMMING 🥁🥁\n");
    printf("===================================\n");
    printf("RIGHT HAND %s (Vertical):     LEFT HAND %s (Horizontal):\n",
           get_hand_emoji(HAND_RIGHT), get_hand_emoji(HAND_LEFT));
    printf("• %s SNARE (tilt UP)        • %s HI-HAT (tilt RIGHT)\n",
           get_drum_emoji(DRUM_SNARE), get_drum_emoji(DRUM_HIHAT));
    printf("• %s TOM (tilt DOWN)        • %s CRASH (tilt LEFT)\n",
           get_drum_emoji(DRUM_TOM), get_drum_emoji(DRUM_CRASH));
    printf("\n🎯 ZONE LAYOUT (like a square):\n");
    printf("   CRASH %s ←  👆 NEUTRAL →  %s HIHAT\n",
           get_drum_emoji(DRUM_CRASH), get_drum_emoji(DRUM_HIHAT));
    printf("                    ↑\n");
    printf("                  SNARE %s\n", get_drum_emoji(DRUM_SNARE));
    printf("                    ↓\n");
    printf("                  TOM %s\n", get_drum_emoji(DRUM_TOM));
    printf("\n🎚️  Velocities: soft → medium → hard → very_hard\n");
    printf("📡 Sounds -> Mac at %s:%d\n", MAC_IP, UDP_PORT);
    printf("===================================\n\n");
}

void init_drumstick_state(drumstick_state_t *state, hand_type_t hand) {
    memset(state, 0, sizeof(drumstick_state_t));
    state->current_zone = DRUM_NEUTRAL;
    state->hand = hand;
    state->buffer_index = 0;
    state->detecting_hit = 0;
    state->last_hit_time = 0;
    state->last_display_time = 0;
    state->alpha = 0.7f;
    state->consecutive_hits = 0;
}

int process_stick_data(uint8_t addr, drumstick_state_t *state) {
    int16_t accel[3], gyro[3];
    if (mpu6050_read_motion_data(addr, accel, gyro) != I2C_SUCCESS) {
        return 0;
    }

    float ax = (accel[0] - state->ax_bias) / 4096.0f;
    float ay = (accel[1] - state->ay_bias) / 4096.0f;
    float az = (accel[2] - state->az_bias) / 4096.0f;
    float gx = (gyro[0] - state->gx_bias) / 32.8f;
    float gy = (gyro[1] - state->gy_bias) / 32.8f;
    float gz = (gyro[2] - state->gz_bias) / 32.8f;

    float pitch, roll;
    calculate_orientation(ax, ay, az, &pitch, &roll);
    add_sample_to_buffer(state, ax, ay, az, gx, gy, gz, pitch, roll);

    drum_type_t zone = get_drum_zone(pitch, roll, state->hand);
    if (zone != DRUM_NONE && zone != DRUM_NEUTRAL) {
        state->current_zone = zone;
    }

    if (detect_hit_gesture(state)) {
        play_drum_sound(state->current_zone, state->hit_intensity, state->hand);
        return 1;
    }

    uint64_t current_time = get_time_us();
    if ((current_time - state->last_display_time) >= 250000) {
        motion_sample_t *sample = get_sample(state, 0);
        if (sample->timestamp > 0) {
            snprintf(state->last_display, sizeof(state->last_display),
                    "%s %s %s | P:%.1f° R:%.1f° | A:%.2f G:%.1f",
                    get_hand_emoji(state->hand),
                    state->detecting_hit ? "⚡" : "✓",
                    get_drum_name(state->current_zone),
                    sample->pitch, sample->roll,
                    sample->total_accel, sample->total_gyro);
            state->last_display_time = current_time;
            return 2;
        }
    }
    return 0;
}

int main() {
    pthread_t right, left;

    printf("🥁 SQUARE LAYOUT DUAL STICK AIR DRUMMER 🥁\n");
    printf("==========================================\n");
    printf("Updated: 2025-07-20 07:04:24\n");
    printf("User: KeerathS\n");
    printf("Layout: Square zones, better velocity distribution\n\n");

    if (init_udp() != 0) {
        printf("❌ Failed to initialize UDP\n");
        return 1;
    }

    if (open_i2c_bus(1) != I2C_SUCCESS) {
        printf("❌ Failed to open I2C bus\n");
        close_udp();
        return 1;
    }

    if (pthread_create(&right, NULL, mpu_thread_func, "Right") != 0) {
        printf("❌ Failed to create right thread\n");
        close_i2c_bus();
        close_udp();
        return 1;
    }

    if (pthread_create(&left, NULL, mpu_thread_func, "Left") != 0) {
        printf("❌ Failed to create left thread\n");
        pthread_cancel(right);
        close_i2c_bus();
        close_udp();
        return 1;
    }

    printf("✅ Both MPU6050 sensors detected with square layout!\n");

    pthread_join(right, NULL);
    pthread_join(left, NULL);

    close_i2c_bus();
    close_udp();
    return 0;
}
