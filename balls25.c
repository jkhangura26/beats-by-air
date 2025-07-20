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

// UDP Configuration
#define MAC_IP "192.168.137.83"  // Your Mac's IP
#define UDP_PORT 8888

// MPU6050 Constants (keeping all your original constants)
#define MPU6050_ADDR1        0x68  // Left stick
#define MPU6050_ADDR2        0x69  // Right stick
#define WHO_AM_I             0x75
#define PWR_MGMT_1           0x6B
#define CONFIG               0x1A
#define ACCEL_XOUT_H         0x3B
#define GYRO_CONFIG          0x1B
#define ACCEL_CONFIG         0x1C

#define I2C_SUCCESS 0
#define I2C_ERROR   -1

// Gesture-Based Hit Detection Parameters (keeping all your original parameters)
#define GESTURE_BUFFER_SIZE    8
#define GESTURE_THRESHOLD      2.5f
#define DECELERATION_THRESHOLD -2.2f
#define PEAK_DETECT_TIME_MS    60
#define SAMPLE_RATE_US        2500
#define MIN_HIT_INTERVAL      120000
#define HIT_COOLDOWN_PERIOD   40000

// Zone Configuration (keeping all your original zones)
#define NEUTRAL_PITCH_MIN    -15.0f
#define NEUTRAL_PITCH_MAX    30.0f
#define NEUTRAL_ROLL_MIN     -20.0f
#define NEUTRAL_ROLL_MAX     20.0f

#define SNARE_PITCH_MIN      -90.0f
#define SNARE_PITCH_MAX      -10.0f

#define HIHAT_ROLL_MIN       30.0f
#define HIHAT_ROLL_MAX       90.0f

#define CRASH_ROLL_MIN       -90.0f
#define CRASH_ROLL_MAX       -30.0f

#define TOM_PITCH_MIN        40.0f
#define TOM_PITCH_MAX        90.0f

#define LEFT_DEFAULT_DRUM    DRUM_HIHAT
#define RIGHT_DEFAULT_DRUM   DRUM_SNARE

// Drum Types (keeping all your original types)
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

// Motion Sample Structure (keeping all your original structures)
typedef struct {
    float ax, ay, az;
    float total_accel;
    float pitch, roll;
    uint64_t timestamp;
} motion_sample_t;

typedef struct {
    motion_sample_t samples[GESTURE_BUFFER_SIZE];
    int buffer_index;
    drum_type_t current_zone;
    uint64_t last_hit_time;
    uint64_t last_display_time;
    float hit_intensity;
    int detecting_hit;
    uint64_t hit_start_time;
    float max_accel_during_hit;
    float min_accel_during_hit;
    float ax_bias, ay_bias, az_bias;
    hand_type_t hand;
    char last_display[64];
} drumstick_state_t;

// I2C Message Structures (keeping all your original I2C code)
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


const char* get_hand_emoji(hand_type_t hand);
const char* get_drum_emoji(drum_type_t drum);
drum_type_t get_drum_type_from_name(const char* name);

// ---------- UDP Functions ----------
int init_udp() {
    udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket < 0) {
        perror("UDP socket creation failed");
        return -1;
    }

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
    snprintf(command, sizeof(command), "%s", drum_name);

    ssize_t sent = sendto(udp_socket, command, strlen(command), 0,
                         (struct sockaddr*)&mac_addr, sizeof(mac_addr));

    if (sent > 0) {
        printf("%s %s %s HIT! (Vol: %.0f%%) -> Sent UDP\n",
               get_hand_emoji(hand),
               get_drum_emoji(get_drum_type_from_name(drum_name)),
               drum_name,
               intensity * 100);
    } else {
        printf("❌ UDP send failed for %s\n", drum_name);
    }
}

void close_udp() {
    if (udp_socket >= 0) {
        close(udp_socket);
        udp_socket = -1;
    }
}

// Helper to get drum type from name
drum_type_t get_drum_type_from_name(const char* name) {
    if (strcmp(name, "snare") == 0) return DRUM_SNARE;
    if (strcmp(name, "hihat") == 0) return DRUM_HIHAT;
    if (strcmp(name, "tom") == 0) return DRUM_TOM;
    if (strcmp(name, "crash") == 0) return DRUM_CRASH;
    return DRUM_NONE;
}

// ---------- All your original I2C functions (unchanged) ----------
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

// ---------- All your original helper functions (unchanged) ----------
short to_int16(uint8_t hi, uint8_t lo) {
    return (short)((hi << 8) | lo);
}

uint64_t get_time_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}

void calculate_orientation(float ax, float ay, float az, float *pitch, float *roll) {
    *pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
    *roll = atan2f(ay, az) * 180.0f / M_PI;
    if (*pitch > 90.0f) *pitch = 90.0f;
    if (*pitch < -90.0f) *pitch = -90.0f;
    if (*roll > 180.0f) *roll = 180.0f;
    if (*roll < -180.0f) *roll = -180.0f;
}

drum_type_t get_drum_zone(float pitch, float roll, hand_type_t hand) {
    if (pitch >= SNARE_PITCH_MIN && pitch <= SNARE_PITCH_MAX) {
        return DRUM_SNARE;
    }
    if (roll >= HIHAT_ROLL_MIN && roll <= HIHAT_ROLL_MAX) {
        return DRUM_HIHAT;
    }
    if (roll >= CRASH_ROLL_MIN && roll <= CRASH_ROLL_MAX) {
        return DRUM_CRASH;
    }
    if (pitch >= TOM_PITCH_MIN && pitch <= TOM_PITCH_MAX) {
        return DRUM_TOM;
    }
    if (pitch >= NEUTRAL_PITCH_MIN && pitch <= NEUTRAL_PITCH_MAX &&
        roll >= NEUTRAL_ROLL_MIN && roll <= NEUTRAL_ROLL_MAX) {
        return DRUM_NEUTRAL;
    }
    return hand == HAND_LEFT ? LEFT_DEFAULT_DRUM : RIGHT_DEFAULT_DRUM;
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

// ---------- MODIFIED: UDP-enabled drum sound function ----------
void play_drum_sound(drum_type_t drum, float intensity, hand_type_t hand) {
    if (drum == DRUM_NEUTRAL || drum == DRUM_NONE) return;

    const char* drum_name = get_drum_name(drum);
    send_drum_command(drum_name, intensity, hand);
}

// ---------- All your original motion detection functions (unchanged) ----------
void add_sample_to_buffer(drumstick_state_t *state, float ax, float ay, float az,
                          float pitch, float roll) {
    motion_sample_t *sample = &state->samples[state->buffer_index];
    sample->ax = ax;
    sample->ay = ay;
    sample->az = az;
    sample->total_accel = sqrtf(ax*ax + ay*ay + az*az);
    sample->pitch = pitch;
    sample->roll = roll;
    sample->timestamp = get_time_us();
    state->buffer_index = (state->buffer_index + 1) % GESTURE_BUFFER_SIZE;
}

motion_sample_t* get_sample(drumstick_state_t *state, int offset) {
    int index = (state->buffer_index - 1 - offset + GESTURE_BUFFER_SIZE) % GESTURE_BUFFER_SIZE;
    return &state->samples[index];
}

int detect_hit_gesture(drumstick_state_t *state) {
    // All your original hit detection logic unchanged
    uint64_t current_time = get_time_us();
    if ((current_time - state->last_hit_time) <= MIN_HIT_INTERVAL) {
        return 0;
    }
    motion_sample_t *current = get_sample(state, 0);
    if (current->timestamp == 0) {
        return 0;
    }
    if (!state->detecting_hit) {
        motion_sample_t *prev1 = get_sample(state, 1);
        motion_sample_t *prev2 = get_sample(state, 2);
        if (prev1->timestamp == 0 || prev2->timestamp == 0) {
            return 0;
        }
        float z_accel_delta = current->az - prev2->az;
        float total_accel_delta = current->total_accel - prev2->total_accel;
        if ((fabsf(z_accel_delta) > 1.8f || fabsf(total_accel_delta) > 2.0f) &&
            (current->total_accel > 1.8f)) {
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
        motion_sample_t *prev2 = get_sample(state, 2);
        if (prev1->timestamp == 0 || prev2->timestamp == 0) {
            return 0;
        }
        float z_accel_change = current->az - prev1->az;
        if (z_accel_change < DECELERATION_THRESHOLD ||
            (state->max_accel_during_hit - current->total_accel) > 1.5f) {
            state->hit_intensity = fminf(1.0f, state->max_accel_during_hit / 4.0f);
            if (state->hit_intensity < 0.3f) state->hit_intensity = 0.3f;
            state->detecting_hit = 0;
            state->last_hit_time = current_time;
            return 1;
        }
        if ((current_time - state->hit_start_time) > PEAK_DETECT_TIME_MS * 1000) {
            state->detecting_hit = 0;
            return 0;
        }
    }
    return 0;
}

// All your remaining functions unchanged (calibrate_drumstick, print_instructions, etc.)
void calibrate_drumstick(uint8_t addr, drumstick_state_t *state) {
    float sum_x = 0, sum_y = 0, sum_z = 0;
    uint8_t buf[6];
    const int samples = 60;

    for (int i = 0; i < samples; i++) {
        if (mpu6050_read_block(addr, ACCEL_XOUT_H, buf, 6) == I2C_SUCCESS) {
            float x = to_int16(buf[0], buf[1]);
            float y = to_int16(buf[2], buf[3]);
            float z = to_int16(buf[4], buf[5]);
            sum_x += x;
            sum_y += y;
            sum_z += z;
        }
        usleep(10000);
        if (i % 10 == 0) {
            printf(".");
            fflush(stdout);
        }
    }
    state->ax_bias = sum_x / samples;
    state->ay_bias = sum_y / samples;
    state->az_bias = (sum_z / samples) - 8192.0f;
}

void print_instructions() {
    printf("\n🥁🥁 DUAL STICK AIR DRUMMING (UDP) 🥁🥁\n");
    printf("=====================================\n");
    printf("LEFT HAND %s:  RIGHT HAND %s:\n", get_hand_emoji(HAND_LEFT), get_hand_emoji(HAND_RIGHT));
    printf("• %s CRASH (left)  • %s SNARE (down)\n", get_drum_emoji(DRUM_CRASH), get_drum_emoji(DRUM_SNARE));
    printf("• %s HI-HAT (right) • %s TOM (up)\n", get_drum_emoji(DRUM_HIHAT), get_drum_emoji(DRUM_TOM));
    printf("Sounds will play on Mac at %s:%d\n", MAC_IP, UDP_PORT);
    printf("=====================================\n\n");
}

void init_drumstick_state(drumstick_state_t *state, hand_type_t hand) {
    memset(state, 0, sizeof(drumstick_state_t));
    state->current_zone = DRUM_NEUTRAL;
    state->hand = hand;
    state->buffer_index = 0;
    state->detecting_hit = 0;
    state->last_hit_time = 0;
    state->last_display_time = 0;
}

int process_stick_data(uint8_t addr, drumstick_state_t *state) {
    // All your original processing logic unchanged
    uint8_t data[6];
    if (mpu6050_read_block(addr, ACCEL_XOUT_H, data, 6) != I2C_SUCCESS) {
        return 0;
    }
    float ax = (to_int16(data[0], data[1]) - state->ax_bias) / 8192.0f;
    float ay = (to_int16(data[2], data[3]) - state->ay_bias) / 8192.0f;
    float az = (to_int16(data[4], data[5]) - state->az_bias) / 8192.0f;
    float pitch, roll;
    calculate_orientation(ax, ay, az, &pitch, &roll);
    add_sample_to_buffer(state, ax, ay, az, pitch, roll);
    drum_type_t zone = get_drum_zone(pitch, roll, state->hand);
    if (zone != DRUM_NONE && zone != DRUM_NEUTRAL) {
        state->current_zone = zone;
    }
    if (detect_hit_gesture(state)) {
        play_drum_sound(state->current_zone, state->hit_intensity, state->hand);
        return 1;
    }
    uint64_t current_time = get_time_us();
    if ((current_time - state->last_display_time) >= 500000) {
        motion_sample_t *sample = get_sample(state, 0);
        if (sample->timestamp > 0) {
            snprintf(state->last_display, sizeof(state->last_display),
                    "%s %s %s | P:%.1f° R:%.1f° | Accel: %.2f",
                    get_hand_emoji(state->hand),
                    state->detecting_hit ? "⚡" : "✓",
                    get_drum_name(state->current_zone),
                    sample->pitch, sample->roll, sample->total_accel);
            state->last_display_time = current_time;
            return 2;
        }
    }
    return 0;
}

void dual_stick_drumming_loop() {
    drumstick_state_t left_stick, right_stick;
    init_drumstick_state(&left_stick, HAND_LEFT);
    init_drumstick_state(&right_stick, HAND_RIGHT);

    printf("🎯 Calibrating LEFT stick...\n");
    calibrate_drumstick(MPU6050_ADDR1, &left_stick);
    printf("\n🎯 Calibrating RIGHT stick...\n");
    calibrate_drumstick(MPU6050_ADDR2, &right_stick);
    printf("\n✅ Calibration complete!\n");

    print_instructions();
    printf("🚀 Air Drumming active! Start playing!\n\n");

    while (1) {
        int left_result = process_stick_data(MPU6050_ADDR1, &left_stick);
        int right_result = process_stick_data(MPU6050_ADDR2, &right_stick);
        if (left_result == 2 || right_result == 2) {
            printf("%s\n%s\n\n", left_stick.last_display, right_stick.last_display);
        }
        usleep(SAMPLE_RATE_US);
    }
}

// ---------- MODIFIED Main with UDP ----------
int main() {
    printf("🥁 DUAL STICK AIR DRUMMER (UDP VERSION) 🥁\n");
    printf("==========================================\n");
    printf("Current date: 2025-07-20 02:47:45\n");
    printf("User: KeerathS\n\n");

    // Initialize UDP first
    if (init_udp() != 0) {
        printf("❌ Failed to initialize UDP\n");
        return 1;
    }

    if (open_i2c_bus(1) != I2C_SUCCESS) {
        printf("❌ Failed to open I2C bus\n");
        close_udp();
        return 1;
    }

    uint8_t id1 = 0, id2 = 0;
    if (mpu6050_read_byte(MPU6050_ADDR1, WHO_AM_I, &id1) != I2C_SUCCESS || id1 != 0x68) {
        printf("❌ Left stick MPU6050 not detected. ID = 0x%02X\n", id1);
        close_i2c_bus();
        close_udp();
        return 1;
    }
    if (mpu6050_read_byte(MPU6050_ADDR2, WHO_AM_I, &id2) != I2C_SUCCESS || id2 != 0x68) {
        printf("❌ Right stick MPU6050 not detected. ID = 0x%02X\n", id2);
        close_i2c_bus();
        close_udp();
        return 1;
    }

    printf("✅ Both MPU6050 sensors detected!\n");

    // Initialize sensors
    mpu6050_write_byte(MPU6050_ADDR1, PWR_MGMT_1, 0x00);
    mpu6050_write_byte(MPU6050_ADDR1, CONFIG, 0x01);
    mpu6050_write_byte(MPU6050_ADDR1, ACCEL_CONFIG, 0x08);
    mpu6050_write_byte(MPU6050_ADDR2, PWR_MGMT_1, 0x00);
    mpu6050_write_byte(MPU6050_ADDR2, CONFIG, 0x01);
    mpu6050_write_byte(MPU6050_ADDR2, ACCEL_CONFIG, 0x08);
    usleep(50000);

    dual_stick_drumming_loop();

    close_i2c_bus();
    close_udp();
    return 0;
}
