#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <hw/i2c.h>

// MPU6050 Constants
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

// Gesture-Based Hit Detection Parameters
#define GESTURE_BUFFER_SIZE    8       // Store 8 most recent acceleration samples
#define GESTURE_THRESHOLD      2.5f    // Acceleration change threshold for hit detection
#define DECELERATION_THRESHOLD -2.2f   // Negative acceleration indicating hit impact
#define PEAK_DETECT_TIME_MS    60      // 60ms window to detect a hitting peak

// Faster Sampling for Better Motion Capture
#define SAMPLE_RATE_US        2500     // 2.5ms = 400Hz sampling (very fast)
#define MIN_HIT_INTERVAL      120000   // 120ms minimum between hits (microseconds)
#define HIT_COOLDOWN_PERIOD   40000    // 40ms cooldown after hit

// Zone Configuration - Broader zones for easier targeting
#define NEUTRAL_PITCH_MIN    -15.0f
#define NEUTRAL_PITCH_MAX    30.0f
#define NEUTRAL_ROLL_MIN     -20.0f
#define NEUTRAL_ROLL_MAX     20.0f

#define SNARE_PITCH_MIN      -90.0f    // Downward tilt
#define SNARE_PITCH_MAX      -10.0f

#define HIHAT_ROLL_MIN       30.0f     // Right side movement
#define HIHAT_ROLL_MAX       90.0f

#define CRASH_ROLL_MIN       -90.0f    // Left side movement
#define CRASH_ROLL_MAX       -30.0f

#define TOM_PITCH_MIN        40.0f     // Upward tilt
#define TOM_PITCH_MAX        90.0f

// Right-Left Hand Default Drums
#define LEFT_DEFAULT_DRUM    DRUM_HIHAT
#define RIGHT_DEFAULT_DRUM   DRUM_SNARE

// Drum Types
typedef enum {
    DRUM_SNARE = 0,
    DRUM_HIHAT = 1,
    DRUM_TOM = 2,
    DRUM_CRASH = 3,
    DRUM_KICK = 4,    // Added kick drum
    DRUM_RIDE = 5,    // Added ride cymbal
    DRUM_NEUTRAL = 6,
    DRUM_NONE = -1
} drum_type_t;

// Hand Identifier
typedef enum {
    HAND_LEFT = 0,
    HAND_RIGHT = 1
} hand_type_t;

// Motion Sample Structure
typedef struct {
    float ax, ay, az;         // Raw acceleration values
    float total_accel;        // Total acceleration magnitude
    float pitch, roll;        // Orientation angles
    uint64_t timestamp;       // Sample timestamp
} motion_sample_t;

// Gesture State Tracking
typedef struct {
    // Motion history buffer
    motion_sample_t samples[GESTURE_BUFFER_SIZE];
    int buffer_index;

    // Current state
    drum_type_t current_zone;
    uint64_t last_hit_time;
    uint64_t last_display_time;

    // Hit detection state
    float hit_intensity;
    int detecting_hit;
    uint64_t hit_start_time;
    float max_accel_during_hit;
    float min_accel_during_hit;

    // Calibration values
    float ax_bias, ay_bias, az_bias;

    // Hand type
    hand_type_t hand;

    // Visualization data
    char last_display[64];
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

// ---------- Helper Functions ----------
short to_int16(uint8_t hi, uint8_t lo) {
    return (short)((hi << 8) | lo);
}

uint64_t get_time_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}

// Calculate orientation angles
void calculate_orientation(float ax, float ay, float az, float *pitch, float *roll) {
    *pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
    *roll = atan2f(ay, az) * 180.0f / M_PI;

    // Ensure angles stay in valid ranges
    if (*pitch > 90.0f) *pitch = 90.0f;
    if (*pitch < -90.0f) *pitch = -90.0f;
    if (*roll > 180.0f) *roll = 180.0f;
    if (*roll < -180.0f) *roll = -180.0f;
}

// Determine drum zone from orientation
drum_type_t get_drum_zone(float pitch, float roll, hand_type_t hand) {
    // Snare: Downward tilt
    if (pitch >= SNARE_PITCH_MIN && pitch <= SNARE_PITCH_MAX) {
        return DRUM_SNARE;
    }

    // Hi-Hat: Right side
    if (roll >= HIHAT_ROLL_MIN && roll <= HIHAT_ROLL_MAX) {
        return DRUM_HIHAT;
    }

    // Crash: Left side
    if (roll >= CRASH_ROLL_MIN && roll <= CRASH_ROLL_MAX) {
        return DRUM_CRASH;
    }

    // Tom: Upward/backward tilt
    if (pitch >= TOM_PITCH_MIN && pitch <= TOM_PITCH_MAX) {
        return DRUM_TOM;
    }

    // Default to neutral
    if (pitch >= NEUTRAL_PITCH_MIN && pitch <= NEUTRAL_PITCH_MAX &&
        roll >= NEUTRAL_ROLL_MIN && roll <= NEUTRAL_ROLL_MAX) {
        return DRUM_NEUTRAL;
    }

    // Default to hand-specific drum if in an undefined zone
    return hand == HAND_LEFT ? LEFT_DEFAULT_DRUM : RIGHT_DEFAULT_DRUM;
}

// Get drum name string
const char* get_drum_name(drum_type_t drum) {
    switch (drum) {
        case DRUM_SNARE: return "SNARE";
        case DRUM_HIHAT: return "HI-HAT";
        case DRUM_TOM: return "TOM";
        case DRUM_CRASH: return "CRASH";
        case DRUM_KICK: return "KICK";
        case DRUM_RIDE: return "RIDE";
        case DRUM_NEUTRAL: return "NEUTRAL";
        default: return "UNKNOWN";
    }
}

// Get drum emoji for visual feedback
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

// Get hand emoji
const char* get_hand_emoji(hand_type_t hand) {
    return hand == HAND_LEFT ? "👈" : "👉";
}

// Add motion sample to the buffer
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

// Get sample from the circular buffer
motion_sample_t* get_sample(drumstick_state_t *state, int offset) {
    int index = (state->buffer_index - 1 - offset + GESTURE_BUFFER_SIZE) % GESTURE_BUFFER_SIZE;
    return &state->samples[index];
}

// Detect drum hit based on motion pattern
int detect_hit_gesture(drumstick_state_t *state) {
    uint64_t current_time = get_time_us();

    // Exit early if we've hit recently
    if ((current_time - state->last_hit_time) <= MIN_HIT_INTERVAL) {
        return 0;
    }

    // Get the most recent samples
    motion_sample_t *current = get_sample(state, 0);

    // Not enough samples yet
    if (current->timestamp == 0) {
        return 0;
    }

    // Check for start of hit - sharp acceleration change
    if (!state->detecting_hit) {
        // Look for sudden acceleration in any axis
        motion_sample_t *prev1 = get_sample(state, 1);
        motion_sample_t *prev2 = get_sample(state, 2);

        // Need enough samples
        if (prev1->timestamp == 0 || prev2->timestamp == 0) {
            return 0;
        }

        // Look for acceleration change pattern - especially in Z axis (up/down)
        float z_accel_delta = current->az - prev2->az;
        float total_accel_delta = current->total_accel - prev2->total_accel;

        // Check for start of hit gesture
        if ((fabsf(z_accel_delta) > 1.8f || fabsf(total_accel_delta) > 2.0f) &&
            (current->total_accel > 1.8f)) {

            // Hit started!
            state->detecting_hit = 1;
            state->hit_start_time = current_time;
            state->max_accel_during_hit = current->total_accel;
            state->min_accel_during_hit = current->total_accel;

            // Determine the drum zone at the start of the hit
            state->current_zone = get_drum_zone(current->pitch, current->roll, state->hand);

            return 0; // Still detecting, not confirmed yet
        }
        return 0; // No hit detected
    }

    // Already detecting a hit - track maximum acceleration
    if (state->detecting_hit) {
        // Update min/max acceleration
        if (current->total_accel > state->max_accel_during_hit) {
            state->max_accel_during_hit = current->total_accel;
        }
        if (current->total_accel < state->min_accel_during_hit) {
            state->min_accel_during_hit = current->total_accel;
        }

        // Look for hit impact - sudden deceleration after acceleration
        motion_sample_t *prev1 = get_sample(state, 1);
        motion_sample_t *prev2 = get_sample(state, 2);

        // Need enough samples
        if (prev1->timestamp == 0 || prev2->timestamp == 0) {
            return 0;
        }

        // Check for deceleration after initial acceleration
        float z_accel_change = current->az - prev1->az;

        // Hit confirmed when we see deceleration (especially in z-axis)
        if (z_accel_change < DECELERATION_THRESHOLD ||
            (state->max_accel_during_hit - current->total_accel) > 1.5f) {

            // Hit confirmed!
            state->hit_intensity = fminf(1.0f, state->max_accel_during_hit / 4.0f);
            if (state->hit_intensity < 0.3f) state->hit_intensity = 0.3f;

            // End detection
            state->detecting_hit = 0;
            state->last_hit_time = current_time;

            return 1; // Hit detected and confirmed!
        }

        // Hit detection timeout - if we've been detecting for too long without confirmation
        if ((current_time - state->hit_start_time) > PEAK_DETECT_TIME_MS * 1000) {
            state->detecting_hit = 0; // Reset detection state
            return 0;
        }
    }

    return 0; // No hit confirmed yet
}

// Play drum sound with visual feedback for specific hand
void play_drum_sound(drum_type_t drum, float intensity, hand_type_t hand) {
    if (drum == DRUM_NEUTRAL || drum == DRUM_NONE) return;

    char command[256];
    const char* sound_file;
    const char* hand_name = (hand == HAND_LEFT) ? "LEFT" : "RIGHT";

    // Map drum types to sound files
    switch (drum) {
        case DRUM_SNARE:
            sound_file = "/tmp/snare.wav";
            break;
        case DRUM_HIHAT:
            sound_file = "/tmp/hihat.wav";
            break;
        case DRUM_TOM:
            sound_file = "/tmp/tom.wav";
            break;
        case DRUM_CRASH:
            sound_file = "/tmp/crash.wav";
            break;
        case DRUM_KICK:
            sound_file = "/tmp/kick.wav";
            break;
        case DRUM_RIDE:
            sound_file = "/tmp/ride.wav";
            break;
        default:
            return;
    }

    // Calculate volume based on hit intensity
    int volume = (int)(intensity * 80) + 20;  // 20-100 range
    if (volume > 100) volume = 100;

    // Visual feedback specific to hits with hand indicator
    printf("%s %s %s HIT! (Vol: %d%%)\n",
           get_hand_emoji(hand),
           get_drum_emoji(drum),
           get_drum_name(drum),
           volume);

    snprintf(command, sizeof(command), "qplay -v %d %s &", volume, sound_file);
    // Uncomment when you have sound files ready
    // system(command);
}

// Calibration routine for a single sensor
void calibrate_drumstick(uint8_t addr, drumstick_state_t *state) {
    float ax_bias = 0, ay_bias = 0, az_bias = 0;
    uint8_t buf[6];
    const int samples = 60;  // Quick calibration

    float sum_x = 0, sum_y = 0, sum_z = 0;

    for (int i = 0; i < samples; i++) {
        if (mpu6050_read_block(addr, ACCEL_XOUT_H, buf, 6) == I2C_SUCCESS) {
            float x = to_int16(buf[0], buf[1]);
            float y = to_int16(buf[2], buf[3]);
            float z = to_int16(buf[4], buf[5]);

            sum_x += x;
            sum_y += y;
            sum_z += z;
        }
        usleep(10000); // 10ms between samples

        if (i % 10 == 0) {
            printf(".");
            fflush(stdout);
        }
    }

    // Calculate mean
    state->ax_bias = sum_x / samples;
    state->ay_bias = sum_y / samples;
    state->az_bias = (sum_z / samples) - 8192.0f; // Remove 1g from Z (gravity), adjusted for ±4g range
}

// Print instructions for dual stick drumming
void print_instructions() {
    printf("\n🥁🥁 DUAL STICK AIR DRUMMING 🥁🥁\n");
    printf("================================\n");
    printf("LEFT HAND %s:  RIGHT HAND %s:\n", get_hand_emoji(HAND_LEFT), get_hand_emoji(HAND_RIGHT));
    printf("• %s CRASH (left)  • %s SNARE (down)\n", get_drum_emoji(DRUM_CRASH), get_drum_emoji(DRUM_SNARE));
    printf("• %s HI-HAT (right) • %s TOM (up)\n", get_drum_emoji(DRUM_HIHAT), get_drum_emoji(DRUM_TOM));
    printf("\nMake drumming MOTIONS to trigger sounds!\n");
    printf("================================\n\n");
}

// Initialize drumstick state
void init_drumstick_state(drumstick_state_t *state, hand_type_t hand) {
    memset(state, 0, sizeof(drumstick_state_t));
    state->current_zone = DRUM_NEUTRAL;
    state->hand = hand;
    state->buffer_index = 0;
    state->detecting_hit = 0;
    state->last_hit_time = 0;
    state->last_display_time = 0;
}

// Process sensor data for a single stick
int process_stick_data(uint8_t addr, drumstick_state_t *state) {
    uint8_t data[6]; // We only need accelerometer data

    // Read accelerometer data
    if (mpu6050_read_block(addr, ACCEL_XOUT_H, data, 6) != I2C_SUCCESS) {
        return 0; // Failed to read
    }

    // Convert to g's (±4g range, 8192 LSB/g)
    float ax = (to_int16(data[0], data[1]) - state->ax_bias) / 8192.0f;
    float ay = (to_int16(data[2], data[3]) - state->ay_bias) / 8192.0f;
    float az = (to_int16(data[4], data[5]) - state->az_bias) / 8192.0f;

    // Calculate orientation
    float pitch, roll;
    calculate_orientation(ax, ay, az, &pitch, &roll);

    // Add sample to buffer
    add_sample_to_buffer(state, ax, ay, az, pitch, roll);

    // Update current zone
    drum_type_t zone = get_drum_zone(pitch, roll, state->hand);
    if (zone != DRUM_NONE && zone != DRUM_NEUTRAL) {
        state->current_zone = zone;
    }

    // Detect hits based on motion pattern
    if (detect_hit_gesture(state)) {
        play_drum_sound(state->current_zone, state->hit_intensity, state->hand);
        return 1; // Hit detected
    }

    // Status update
    uint64_t current_time = get_time_us();
    if ((current_time - state->last_display_time) >= 500000) { // 0.5 second interval
        motion_sample_t *sample = get_sample(state, 0);
        if (sample->timestamp > 0) {
            // Format status string for this stick
            snprintf(state->last_display, sizeof(state->last_display),
                    "%s %s %s | P:%.1f° R:%.1f° | Accel: %.2f",
                    get_hand_emoji(state->hand),
                    state->detecting_hit ? "⚡" : "✓",
                    get_drum_name(state->current_zone),
                    sample->pitch, sample->roll, sample->total_accel);

            state->last_display_time = current_time;
            return 2; // Display update
        }
    }

    return 0; // No special event
}

// ---------- Main Dual Stick Loop ----------
void dual_stick_drumming_loop() {
    drumstick_state_t left_stick, right_stick;

    // Initialize both sticks
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
        // Process left stick
        int left_result = process_stick_data(MPU6050_ADDR1, &left_stick);

        // Process right stick
        int right_result = process_stick_data(MPU6050_ADDR2, &right_stick);

        // Display status updates if needed
        if (left_result == 2 || right_result == 2) {
            printf("%s\n%s\n\n",
                   left_stick.last_display,
                   right_stick.last_display);
        }

        // Alternate between sticks to maintain high sampling rate
        usleep(SAMPLE_RATE_US);
    }
}

// ---------- Main ----------
int main() {
    printf("🥁 DUAL STICK AIR DRUMMER 🥁\n");
    printf("===========================\n");
    printf("Current date: 2025-07-19 23:15:56\n");
    printf("User: jkhangura26\n\n");

    if (open_i2c_bus(1) != I2C_SUCCESS) {
        printf("❌ Failed to open I2C bus\n");
        return 1;
    }

    uint8_t id1 = 0, id2 = 0;

    // Check if both sensors are available
    if (mpu6050_read_byte(MPU6050_ADDR1, WHO_AM_I, &id1) != I2C_SUCCESS || id1 != 0x68) {
        printf("❌ Left stick MPU6050 not detected. ID = 0x%02X\n", id1);
        close_i2c_bus();
        return 1;
    }

    if (mpu6050_read_byte(MPU6050_ADDR2, WHO_AM_I, &id2) != I2C_SUCCESS || id2 != 0x68) {
        printf("❌ Right stick MPU6050 not detected. ID = 0x%02X\n", id2);
        close_i2c_bus();
        return 1;
    }

    printf("✅ Both MPU6050 sensors detected!\n");

    // Initialize both sensors with higher sensitivity settings
    mpu6050_write_byte(MPU6050_ADDR1, PWR_MGMT_1, 0x00);  // Wake up
    mpu6050_write_byte(MPU6050_ADDR1, CONFIG, 0x01);      // DLPF_CFG = 1 (184 Hz bandwidth)
    mpu6050_write_byte(MPU6050_ADDR1, ACCEL_CONFIG, 0x08); // Full scale = ±4g

    mpu6050_write_byte(MPU6050_ADDR2, PWR_MGMT_1, 0x00);  // Wake up
    mpu6050_write_byte(MPU6050_ADDR2, CONFIG, 0x01);      // DLPF_CFG = 1 (184 Hz bandwidth)
    mpu6050_write_byte(MPU6050_ADDR2, ACCEL_CONFIG, 0x08); // Full scale = ±4g

    usleep(50000); // Short stabilization time

    // Start the dual stick drumming loop
    dual_stick_drumming_loop();

    close_i2c_bus();
    return 0;
}
