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
#define MPU6050_ADDR         0x68
#define WHO_AM_I             0x75
#define PWR_MGMT_1           0x6B
#define CONFIG               0x1A
#define ACCEL_XOUT_H         0x3B
#define GYRO_CONFIG          0x1B
#define ACCEL_CONFIG         0x1C

#define I2C_SUCCESS 0
#define I2C_ERROR   -1

// Gesture-Based Hit Detection Parameters
#define GESTURE_BUFFER_SIZE    10      // Store 10 most recent acceleration samples
#define GESTURE_WINDOW_SIZE    5       // Use 5 samples for gesture pattern detection
#define GESTURE_THRESHOLD      2.5f    // Acceleration change threshold for hit detection
#define DECELERATION_THRESHOLD -2.5f   // Negative acceleration indicating hit impact
#define PEAK_DETECT_TIME_MS    60      // 60ms window to detect a hitting peak

// Faster Sampling for Better Motion Capture
#define SAMPLE_RATE_US        2500     // 2.5ms = 400Hz sampling (very fast)
#define MIN_HIT_INTERVAL      100000   // 100ms minimum between hits (microseconds)
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

// Drum Types
typedef enum {
    DRUM_SNARE = 0,
    DRUM_HIHAT = 1,
    DRUM_TOM = 2,
    DRUM_CRASH = 3,
    DRUM_NEUTRAL = 4,
    DRUM_NONE = -1
} drum_type_t;

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

    // Zone detection
    drum_type_t last_zone;
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
drum_type_t get_drum_zone(float pitch, float roll) {
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

    return DRUM_NONE;
}

// Get drum name string
const char* get_drum_name(drum_type_t drum) {
    switch (drum) {
        case DRUM_SNARE: return "SNARE";
        case DRUM_HIHAT: return "HI-HAT";
        case DRUM_TOM: return "TOM";
        case DRUM_CRASH: return "CRASH";
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
        case DRUM_NEUTRAL: return "👆";
        default: return "❓";
    }
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
        if ((fabsf(z_accel_delta) > 1.8f || fabsf(total_accel_delta) > 2.2f) &&
            (current->total_accel > 2.0f)) {

            // Hit started!
            state->detecting_hit = 1;
            state->hit_start_time = current_time;
            state->max_accel_during_hit = current->total_accel;
            state->min_accel_during_hit = current->total_accel;

            // Determine the drum zone at the start of the hit
            state->current_zone = get_drum_zone(current->pitch, current->roll);

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

// Play drum sound with visual feedback
void play_drum_sound(drum_type_t drum, float intensity) {
    if (drum == DRUM_NEUTRAL || drum == DRUM_NONE) return;

    char command[256];
    const char* sound_file;

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
        default:
            return;
    }

    // Calculate volume based on hit intensity
    int volume = (int)(intensity * 80) + 20;  // 20-100 range
    if (volume > 100) volume = 100;

    // Visual feedback specific to hits
    printf("💥 %s HIT! (Vol: %d%%)\n", get_drum_name(drum), volume);

    snprintf(command, sizeof(command), "qplay -v %d %s &", volume, sound_file);
    // Uncomment when you have sound files ready
    // system(command);
}

// Calibration routine
void calibrate_drumstick(uint8_t addr, float *ax_bias, float *ay_bias, float *az_bias) {
    printf("🎯 Quick calibration...\n");
    printf("Hold drumstick still...\n");

    *ax_bias = *ay_bias = *az_bias = 0;
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
        usleep(15000); // 15ms between samples

        if (i % 10 == 0) {
            printf(".");
            fflush(stdout);
        }
    }

    // Calculate mean
    *ax_bias = sum_x / samples;
    *ay_bias = sum_y / samples;
    *az_bias = (sum_z / samples) - 16384.0f; // Remove 1g from Z (gravity)

    printf("\n✅ Ready!\n");
}

// Print instructions
void print_instructions() {
    printf("\n🥁 GESTURE-BASED AIR DRUMMING 🥁\n");
    printf("===============================\n");
    printf("• Make drumming MOTION in any zone\n");
    printf("• Zones determine which drum sounds:\n");
    printf("  - Down swing → 🥁 SNARE\n");
    printf("  - Right swing → 🔔 HI-HAT\n");
    printf("  - Left swing → 💥 CRASH\n");
    printf("  - Up/back swing → 🪘 TOM\n");
    printf("===============================\n\n");
}

// ---------- Main Drumstick Loop ----------
void virtual_drumstick_loop(uint8_t addr) {
    uint8_t data[6];
    float ax_bias, ay_bias, az_bias;

    // Initialize state
    drumstick_state_t state = {0};
    state.current_zone = DRUM_NEUTRAL;
    state.last_hit_time = 0;
    state.last_display_time = 0;
    state.detecting_hit = 0;
    state.buffer_index = 0;

    // Calibrate the drumstick
    calibrate_drumstick(addr, &ax_bias, &ay_bias, &az_bias);
    print_instructions();

    printf("🚀 Air Drummer active! Make drumming motions!\n\n");

    uint64_t status_interval = 500000; // Status update every 0.5 second

    while (1) {
        uint64_t current_time = get_time_us();

        // Read accelerometer data
        if (mpu6050_read_block(addr, ACCEL_XOUT_H, data, 6) != I2C_SUCCESS) {
            printf("❌ Read failed\n");
            break;
        }

        // Convert to g's (±4g range, 8192 LSB/g)
        float ax = (to_int16(data[0], data[1]) - ax_bias) / 8192.0f;
        float ay = (to_int16(data[2], data[3]) - ay_bias) / 8192.0f;
        float az = (to_int16(data[4], data[5]) - az_bias) / 8192.0f;

        // Calculate orientation
        float pitch, roll;
        calculate_orientation(ax, ay, az, &pitch, &roll);

        // Add sample to buffer
        add_sample_to_buffer(&state, ax, ay, az, pitch, roll);

        // Always update current zone without waiting for hit
        drum_type_t zone = get_drum_zone(pitch, roll);
        if (zone != DRUM_NONE && zone != DRUM_NEUTRAL) {
            state.current_zone = zone;
        }

        // Detect hits based on motion pattern
        if (detect_hit_gesture(&state)) {
            play_drum_sound(state.current_zone, state.hit_intensity);
        }

        // Status display at regular intervals
        if ((current_time - state.last_display_time) >= status_interval) {
            // Compute average acceleration over recent samples
            float avg_accel = 0;
            int count = 0;
            for (int i = 0; i < 3; i++) {
                motion_sample_t *s = get_sample(&state, i);
                if (s->timestamp > 0) {
                    avg_accel += s->total_accel;
                    count++;
                }
            }
            if (count > 0) avg_accel /= count;

            // Status display
            const char* status = state.detecting_hit ? "⚡" : "✓";
            printf("%s Zone: %s | P:%.1f° R:%.1f° | Accel: %.2f,%.2f,%.2f (Mag:%.2f)\n",
                   status, get_drum_name(state.current_zone),
                   pitch, roll, ax, ay, az, avg_accel);

            state.last_display_time = current_time;
        }

        usleep(SAMPLE_RATE_US);
    }
}

// ---------- Main ----------
int main() {
    printf("🥁 GESTURE-BASED AIR DRUMMER 🥁\n");
    printf("==============================\n");
    printf("Current date: 2025-07-19 (jkhangura26)\n");

    if (open_i2c_bus(1) != I2C_SUCCESS) {
        printf("❌ Failed to open I2C bus\n");
        return 1;
    }

    uint8_t id = 0;
    if (mpu6050_read_block(MPU6050_ADDR, WHO_AM_I, &id, 1) != I2C_SUCCESS || id != 0x68) {
        printf("❌ MPU6050 not detected. ID = 0x%02X\n", id);
        close_i2c_bus();
        return 1;
    }

    printf("✅ MPU6050 detected!\n");

    // Initialize MPU6050 with higher sensitivity settings
    mpu6050_write_byte(MPU6050_ADDR, PWR_MGMT_1, 0x00);  // Wake up
    mpu6050_write_byte(MPU6050_ADDR, CONFIG, 0x01);      // DLPF_CFG = 1 (184 Hz bandwidth)
    mpu6050_write_byte(MPU6050_ADDR, ACCEL_CONFIG, 0x08); // Full scale = ±4g for better sensitivity

    usleep(50000); // Short stabilization time

    // Start the drumstick loop
    virtual_drumstick_loop(MPU6050_ADDR);

    close_i2c_bus();
    return 0;
}
